//! Clock initialization for WS63.
//!
//! Based on the fbb_ws63 C SDK boot sequence analysis:
//!
//! ## What the boot ROM / bootloader does
//!
//! The flashboot bootloader (`flashboot_ws63/startup/main.c`) runs before
//! the application and performs:
//!
//! 1. `boot_clock_adapt()` — detects TCXO (24/40MHz), configures UART/WDT tick rates
//! 2. `switch_flash_clock_to_pll()` — sets CLDO_CRG_CLK_SEL bit 18 to switch
//!    the **flash controller** clock source from TCXO to PLL. Does NOT switch
//!    CPU, UART, or other peripheral clocks.
//! 3. Initializes watchdog, eFuse, SPI flash, partition table
//! 4. Loads and jumps to the application image
//!
//! The **CPU PLL** (240MHz) is configured by the boot ROM before the bootloader
//! runs. The bootloader inherits this configuration.
//!
//! ## What the application must do
//!
//! The application-level `clock_init.c` in the LiteOS SDK performs:
//!
//! 1. `switch_clock()` — switches peripheral clocks from TCXO to PLL:
//!    - UART0/1/2: CLDO_CRG_CLK_SEL bits 1,2,3
//!    - WiFi MAC: bit 20, WiFi PHY: bit 19
//!    - RF_CTL: bit 0
//!    - SPI: bit 6 (spi_porting.c)
//! 2. `set_uart_tcxo_clock_period()` — configures UART baud base, timer tick,
//!    watchdog period, I2C clock based on detected TCXO frequency
//!
//! For a bare-metal Rust application (no LiteOS), we provide:
//! - `probe_clocks()` — non-invasive: detect TCXO and PLL status
//! - `init_clocks()` — full init: switch flash to PLL + switch UART/SPI to PLL
//!
//! # CLDO_CRG_CLK_SEL bit map (from fbb_ws63 clock_init.c)
//!
//! | Bit | Peripheral | Description |
//! |-----|-----------|-------------|
//! | 0 | RF_CTL | RF control clock → PLL |
//! | 1 | UART0 | UART0 clock → PLL |
//! | 2 | UART1 | UART1 clock → PLL |
//! | 3 | UART2 | UART2 clock → PLL |
//! | 6 | SPI | SPI clock → PLL |
//! | 18 | FLASH | Flash/SFC controller → PLL |
//! | 19 | WiFi PHY | WiFi PHY clock → PLL |
//! | 20 | WiFi MAC | WiFi MAC clock → PLL |
//!
//! # Register map (from fbb_ws63)
//!
//! | Register | Address | Description |
//! |----------|---------|-------------|
//! | HW_CTL | 0x4000_0014 | TCXO frequency detect (bit[0]: 0=24MHz, 1=40MHz) |
//! | REG_EXCEP_RO_RG | 0x4000_319C | PLL lock status (bit 12) |
//! | CMU_NEW_CFG1 | 0x4000_34A4 | Flash clock control |
//! | CLDO_CRG_CLK_SEL | 0x4400_1134 | Clock source select |
//! | CLDO_SUB_CRG_CKEN_CTL1 | 0x4400_1104 | UART clock gate control |
//!
//! # Clock tree (from ws63-guide ch2_system.md)
//!
//! | Domain | Frequency | Clock Source |
//! |--------|-----------|-------------|
//! | CPU | 240 MHz | PLL |
//! | CPU Bus | 240 MHz | PLL |
//! | GPIO | 120 MHz | PLL / 2 |
//! | UART | 160 MHz | PLL-derived |
//! | SPI | 160 MHz | PLL-derived |
//! | I2C | 80 MHz | PLL-derived |
//! | QSPI | 64 MHz | PLL-derived |
//! | Timer | 32 kHz | Crystal |
//! | WDT | 32 kHz | Crystal |
//! | RTC | 32 kHz | Crystal |
//! | Crystal | 40/24 MHz | TCXO |

use crate::peripherals::{CldoCrg, SysCtl0};
use crate::soc::ws63::SYSTEM_CLOCK_HZ;

// ── Register addresses (from fbb_ws63 soc_porting.c / pm_porting.c) ──

/// Hardware control register — TCXO frequency detect.
const HW_CTL: *mut u32 = 0x4000_0014 as *mut u32;
/// Exception RO register — PLL lock status (bit 12).
const REG_EXCEP_RO_RG: *mut u32 = 0x4000_319C as *mut u32;
/// CMU PLL signal register — PLL power-down control (bit 15).
#[allow(dead_code)]
const REG_CMU_FNPLL_SIG: *mut u32 = 0x4000_342C as *mut u32;
/// Flash clock control register.
const CMU_NEW_CFG1: *mut u32 = 0x4000_34A4 as *mut u32;

// ── TCXO frequency ────────────────────────────────────────────────

/// TCXO crystal frequency in Hz.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TcxoFreq {
    /// 24 MHz crystal.
    MHz24 = 24_000_000,
    /// 40 MHz crystal.
    MHz40 = 40_000_000,
}

impl TcxoFreq {
    /// Detect the TCXO frequency by reading the HW_CTL register.
    pub fn detect() -> Self {
        // SAFETY: HW_CTL (0x4000_0014) is a valid physical MMIO register per fbb_ws63
        let hw_ctl = unsafe { HW_CTL.read_volatile() };
        if hw_ctl & 0x01 == 0 { TcxoFreq::MHz24 } else { TcxoFreq::MHz40 }
    }

    /// Return the frequency in Hz.
    pub const fn hz(&self) -> u32 {
        *self as u32
    }
}

// ── PLL status ────────────────────────────────────────────────────

/// Result of PLL lock check.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PllStatus {
    /// PLL is locked and stable.
    Locked,
    /// PLL is unlocked — system runs from TCXO.
    Unlocked,
}

/// Check if the PLL is locked by reading REG_EXCEP_RO_RG bit 12.
fn pll_is_locked() -> bool {
    // fbb_ws63: cmu_is_fnpll_locked() reads REG_EXCEP_RO_RG bit 12
    (unsafe { REG_EXCEP_RO_RG.read_volatile() } >> 12) & 1 == 1
}

/// Poll the PLL lock status with retries.
///
/// * `retry_count` — Number of polling attempts (default 30).
/// * `retry_delay_us` — Delay between attempts in µs (default 1000 = 1ms).
fn wait_pll_lock(retry_count: u32, retry_delay_us: u32) -> PllStatus {
    for _ in 0..retry_count {
        if pll_is_locked() {
            return PllStatus::Locked;
        }
        // Busy-wait delay (approximate at TCXO speed before PLL is on)
        let cycles = TcxoFreq::detect().hz() as u64 * retry_delay_us as u64 / 1_000_000;
        for _ in 0..cycles / 3 {
            core::hint::spin_loop();
        }
    }
    PllStatus::Unlocked
}

// ── Clock initialization ──────────────────────────────────────────

/// System clock configuration after initialization.
#[derive(Debug, Clone, Copy)]
pub struct SystemClocks {
    /// CPU clock (PLL output, 240 MHz).
    pub cpu_clk: u32,
    /// Peripheral bus clock (PCLK, 240 MHz default).
    pub pclk: u32,
    /// TCXO crystal frequency.
    pub tcxo_freq: TcxoFreq,
    /// Whether the PLL is locked.
    pub pll_locked: bool,
}

impl SystemClocks {
    /// Default clocks (assumes boot ROM has configured PLL).
    pub const fn assumed() -> Self {
        Self { cpu_clk: SYSTEM_CLOCK_HZ, pclk: SYSTEM_CLOCK_HZ, tcxo_freq: TcxoFreq::MHz40, pll_locked: true }
    }
}

/// Initialize the system clock tree.
///
/// Performs the full clock initialization sequence from fbb_ws63:
///
/// 1. Detect TCXO frequency (24 or 40 MHz) via HW_CTL register
/// 2. Switch flash clock to PLL (bootloader already did this, but we re-apply)
///    — CMU_NEW_CFG1 sequence + CLDO_CRG_CLK_SEL bit 18
/// 3. Switch UART0/1/2 clocks from TCXO to PLL
///    — Disable UART clock gates → set CLDO_CRG_CLK_SEL bits 1,2,3 → re-enable gates
/// 4. Switch SPI clock to PLL — set CLDO_CRG_CLK_SEL bit 6
/// 5. Verify PLL lock via REG_EXCEP_RO_RG bit 12
///
/// # Arguments
///
/// * `_sys_ctl0` — SYS_CTL0 peripheral (reserved for future PLL config).
/// * `_cldo_crg` — CLDO_CRG peripheral (reserved for future divider config).
///
/// # Returns
///
/// The resolved [`SystemClocks`] configuration.
///
/// # Safety
///
/// This writes to raw MMIO registers. Should only be called once at boot,
/// before any peripheral drivers are initialized.
pub fn init_clocks(_sys_ctl0: &SysCtl0<'_>, _cldo_crg: &CldoCrg<'_>) -> SystemClocks {
    let tcxo_freq = TcxoFreq::detect();
    let clk_sel_ptr = 0x4400_1134 as *mut u32;
    let clk_gate_ptr = 0x4400_1104 as *mut u32; // CLDO_SUB_CRG_CKEN_CTL1

    // ── Step 1: Switch flash clock to PLL ────────────────────
    // (fbb_ws63: switch_flash_clock_to_pll in soc_porting.c)
    // SAFETY: CMU_NEW_CFG1 (0x4000_34A4), CLDO_CRG_CLK_SEL (0x4400_1134)
    // are valid physical MMIO addresses per fbb_ws63 register map.
    unsafe { CMU_NEW_CFG1.write_volatile(0x1) }; // CPU_DIV_FLASH_RSTN_SYNC
    for _ in 0..tcxo_freq.hz() / 1_000_000 / 3 {
        core::hint::spin_loop(); // delay 1µs
    }
    unsafe { CMU_NEW_CFG1.write_volatile(0x3) }; // CPU_DIV_FLASH_RSTN
    unsafe {
        let val = clk_sel_ptr.read_volatile();
        clk_sel_ptr.write_volatile(val | (1 << 18)); // bit 18: flash → PLL
    }

    // ── Step 2: Switch UART clocks to PLL ───────────────────
    // (fbb_ws63: switch_clock in clock_init.c)
    unsafe {
        // Disable UART clock gates (bits 18,19,20 in CLDO_SUB_CRG_CKEN_CTL1)
        let mut gate = clk_gate_ptr.read_volatile();
        gate &= !((1 << 18) | (1 << 19) | (1 << 20));
        clk_gate_ptr.write_volatile(gate);

        // Set CLDO_CRG_CLK_SEL bits 1,2,3: UART0/1/2 → PLL
        let mut sel = clk_sel_ptr.read_volatile();
        sel |= (1 << 1) | (1 << 2) | (1 << 3);
        clk_sel_ptr.write_volatile(sel);

        // Re-enable UART clock gates
        gate |= (1 << 18) | (1 << 19) | (1 << 20);
        clk_gate_ptr.write_volatile(gate);
    }

    // ── Step 3: Switch SPI clock to PLL ─────────────────────
    // (fbb_ws63: spi_porting.c sets CLDO_CRG_CLK_SEL bit 6)
    unsafe {
        let val = clk_sel_ptr.read_volatile();
        clk_sel_ptr.write_volatile(val | (1 << 6)); // bit 6: SPI → PLL
    }

    // ── Step 4: Verify PLL lock ─────────────────────────────
    let pll_locked = match wait_pll_lock(30, 1000) {
        PllStatus::Locked => true,
        PllStatus::Unlocked => false,
    };

    SystemClocks {
        cpu_clk: if pll_locked { SYSTEM_CLOCK_HZ } else { tcxo_freq.hz() },
        pclk: if pll_locked { SYSTEM_CLOCK_HZ } else { tcxo_freq.hz() },
        tcxo_freq,
        pll_locked,
    }
}

/// Simple version: detect clocks without modifying them.
///
/// Reads TCXO frequency and checks PLL status. Does NOT reconfigure
/// the clock tree. Safe to call when boot ROM has already configured
/// the PLL (which is the default on WS63).
pub fn probe_clocks() -> SystemClocks {
    let tcxo_freq = TcxoFreq::detect();
    let pll_locked = pll_is_locked();

    SystemClocks {
        cpu_clk: if pll_locked { SYSTEM_CLOCK_HZ } else { tcxo_freq.hz() },
        pclk: if pll_locked { SYSTEM_CLOCK_HZ } else { tcxo_freq.hz() },
        tcxo_freq,
        pll_locked,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_tcxo_freq_values() {
        assert_eq!(TcxoFreq::MHz24.hz(), 24_000_000);
        assert_eq!(TcxoFreq::MHz40.hz(), 40_000_000);
    }

    #[test]
    fn test_default_clocks() {
        let c = SystemClocks::assumed();
        assert_eq!(c.cpu_clk, 240_000_000);
        assert_eq!(c.pclk, 240_000_000);
        assert!(c.pll_locked);
    }
}
