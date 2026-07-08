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
//! | HW_CTL | 0x4000_0014 | TCXO frequency detect (bit[0]: 0=40MHz, 1=24MHz) |
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
use crate::soc::chip::SYSTEM_CLOCK_HZ;

fn sys_ctl0_regs() -> &'static crate::soc::pac::sys_ctl0::RegisterBlock {
    unsafe { &*SysCtl0::ptr() }
}

fn cmu_regs() -> &'static crate::soc::pac::cmu::RegisterBlock {
    unsafe { &*crate::soc::pac::Cmu::ptr() }
}

fn cldo_crg_regs() -> &'static crate::soc::pac::cldo_crg::RegisterBlock {
    unsafe { &*CldoCrg::ptr() }
}

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
        if sys_ctl0_regs().hw_ctl().read().refclk_freq_status().bit_is_clear() {
            TcxoFreq::MHz40
        } else {
            TcxoFreq::MHz24
        }
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
    cmu_regs().excep_ro_rg().read().fnpll_lock_grm().bit_is_set()
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
    let cmu = cmu_regs();
    let cldo = cldo_crg_regs();

    // ── Step 1: Switch flash clock to PLL ────────────────────
    // (fbb_ws63: switch_flash_clock_to_pll in soc_porting.c)
    cmu.cmu_new_cfg1().write(|w| w.cpu_div_flash_rstn_sync().set_bit());
    for _ in 0..tcxo_freq.hz() / 1_000_000 / 3 {
        core::hint::spin_loop(); // delay 1µs
    }
    cmu.cmu_new_cfg1().write(|w| w.cpu_div_flash_rstn_sync().set_bit().cpu_div_flash_rstn().set_bit());
    cldo.clk_sel().modify(|_, w| w.flash_clk_sel().set_bit());

    // ── Step 2: Switch UART clocks to PLL ───────────────────
    // (fbb_ws63: switch_clock in clock_init.c)
    // Disable UART clock gates (bits 18,19,20 in CLDO_SUB_CRG_CKEN_CTL1).
    cldo.cken_ctl1().modify(|_, w| unsafe { w.uart_cken().bits(0) });

    // Set CLDO_CRG_CLK_SEL bits 1,2,3: UART0/1/2 → PLL.
    cldo.clk_sel().modify(|_, w| w.uart0_clk_sel().set_bit().uart1_clk_sel().set_bit().uart2_clk_sel().set_bit());

    // Re-enable UART clock gates.
    cldo.cken_ctl1().modify(|_, w| unsafe { w.uart_cken().bits(0b111) });

    // ── Step 3: Switch SPI clock to PLL ─────────────────────
    // (fbb_ws63: spi_porting.c sets CLDO_CRG_CLK_SEL bit 6)
    cldo.clk_sel().modify(|_, w| w.spi_clk_sel().set_bit());

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

#[cfg(all(test, not(target_arch = "riscv32")))]
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

// ── Property-based fuzz tests ──────────────────────────────────

#[cfg(all(test, not(target_arch = "riscv32")))]
mod proptests {
    use super::{SYSTEM_CLOCK_HZ, TcxoFreq};
    use proptest::prelude::*;

    /// Re-derive the busy-wait cycle count from `wait_pll_lock`:
    /// `cycles = tcxo_hz * delay_us / 1_000_000`, then the loop runs `cycles / 3`
    /// iterations. All arithmetic is done in u64 exactly as the driver does it.
    fn pll_wait_cycles(tcxo_hz: u32, delay_us: u32) -> u64 {
        let cycles = tcxo_hz as u64 * delay_us as u64 / 1_000_000;
        cycles / 3
    }

    /// Re-derive the 1µs delay-loop count from `init_clocks` step 1:
    /// `tcxo_hz / 1_000_000 / 3`.
    fn flash_delay_iters(tcxo_hz: u32) -> u32 {
        tcxo_hz / 1_000_000 / 3
    }

    /// Re-derive the `cpu_clk`/`pclk` selection from `init_clocks`/`probe_clocks`:
    /// PLL output when locked, else the TCXO frequency.
    fn resolved_clk(pll_locked: bool, tcxo: TcxoFreq) -> u32 {
        if pll_locked { SYSTEM_CLOCK_HZ } else { tcxo.hz() }
    }

    /// Re-derive the CLDO_CRG_CLK_SEL word that `init_clocks` produces by OR-ing in
    /// the flash (18), UART0/1/2 (1,2,3), and SPI (6) source-select bits onto the
    /// pre-existing register value `init`.
    fn clk_sel_word(init: u32) -> u32 {
        let mut sel = init;
        sel |= 1 << 18; // step 1: flash → PLL
        sel |= (1 << 1) | (1 << 2) | (1 << 3); // step 2: UART0/1/2 → PLL
        sel |= 1 << 6; // step 3: SPI → PLL
        sel
    }

    /// Re-derive the CLDO_SUB_CRG_CKEN_CTL1 gate word after `init_clocks` step 2,
    /// which clears bits 18/19/20 then sets them back — net: those bits end up set,
    /// every other bit preserved from `init`.
    fn gate_word_final(init: u32) -> u32 {
        let mut gate = init;
        gate &= !((1 << 18) | (1 << 19) | (1 << 20)); // disable
        gate |= (1 << 18) | (1 << 19) | (1 << 20); // re-enable
        gate
    }

    proptest! {
        /// Fuzz: the PLL-wait cycle formula never panics for any tcxo/delay inputs.
        #[test]
        fn pll_wait_cycles_never_panics(tcxo in any::<u32>(), delay in any::<u32>()) {
            let _ = pll_wait_cycles(tcxo, delay);
        }

        /// Fuzz: the PLL-wait cycle count is monotonic in the delay (longer delay,
        /// never fewer spin cycles) at fixed TCXO.
        #[test]
        fn pll_wait_cycles_monotonic_in_delay(tcxo in any::<u32>(), a in any::<u32>(), b in any::<u32>()) {
            let (lo, hi) = if a <= b { (a, b) } else { (b, a) };
            prop_assert!(pll_wait_cycles(tcxo, hi) >= pll_wait_cycles(tcxo, lo));
        }

        /// Fuzz: at the two real TCXO frequencies the 1ms (1000µs) delay matches the
        /// hand-computed cycle counts — 24MHz→8000, 40MHz→13333 (after /3).
        #[test]
        fn pll_wait_cycles_known_boundaries(_dummy in any::<u8>()) {
            prop_assert_eq!(pll_wait_cycles(TcxoFreq::MHz24.hz(), 1000), 24_000_000u64 * 1000 / 1_000_000 / 3);
            prop_assert_eq!(pll_wait_cycles(TcxoFreq::MHz40.hz(), 1000), 40_000_000u64 * 1000 / 1_000_000 / 3);
            prop_assert_eq!(pll_wait_cycles(TcxoFreq::MHz24.hz(), 1000), 8000);
            prop_assert_eq!(pll_wait_cycles(TcxoFreq::MHz40.hz(), 1000), 13333);
        }

        /// Fuzz: the flash 1µs delay-loop count never panics and stays well within
        /// u32 for any TCXO value.
        #[test]
        fn flash_delay_iters_never_panics(tcxo in any::<u32>()) {
            let _ = flash_delay_iters(tcxo);
        }

        /// Fuzz: at the real TCXO frequencies the flash delay-loop count is exact.
        #[test]
        fn flash_delay_iters_known(_dummy in any::<u8>()) {
            prop_assert_eq!(flash_delay_iters(TcxoFreq::MHz24.hz()), 8); // 24/3
            prop_assert_eq!(flash_delay_iters(TcxoFreq::MHz40.hz()), 13); // 40/3 truncated
        }

        /// Fuzz: the resolved CPU/peripheral clock is exactly PLL-when-locked and
        /// the (24M or 40M) TCXO frequency otherwise — never any other value.
        #[test]
        fn resolved_clk_is_pll_or_tcxo(locked in any::<bool>(), use40 in any::<bool>()) {
            let tcxo = if use40 { TcxoFreq::MHz40 } else { TcxoFreq::MHz24 };
            let r = resolved_clk(locked, tcxo);
            if locked {
                prop_assert_eq!(r, SYSTEM_CLOCK_HZ);
            } else {
                prop_assert_eq!(r, tcxo.hz());
                prop_assert!(r == 24_000_000 || r == 40_000_000);
            }
        }

        /// Fuzz: the CLK_SEL encoder sets exactly the documented source-select bits
        /// (1,2,3,6,18) and never disturbs any other bit of the prior value.
        #[test]
        fn clk_sel_sets_only_documented_bits(init in any::<u32>()) {
            const SET: u32 = (1 << 1) | (1 << 2) | (1 << 3) | (1 << 6) | (1 << 18);
            let out = clk_sel_word(init);
            // all documented bits are set
            prop_assert_eq!(out & SET, SET);
            // every bit outside the SET mask is unchanged from `init`
            prop_assert_eq!(out & !SET, init & !SET);
            // the encoder is purely additive (OR): it never clears a bit
            prop_assert_eq!(out & init, init);
        }

        /// Fuzz: the gate word ends with bits 18/19/20 SET regardless of their prior
        /// state, and all other bits are preserved (the clear-then-set round-trips).
        #[test]
        fn gate_word_round_trips_other_bits(init in any::<u32>()) {
            const GATE: u32 = (1 << 18) | (1 << 19) | (1 << 20);
            let out = gate_word_final(init);
            prop_assert_eq!(out & GATE, GATE); // re-enabled
            prop_assert_eq!(out & !GATE, init & !GATE); // others untouched
        }

        /// Fuzz: TcxoFreq::hz() only ever yields one of the two valid crystal
        /// frequencies, and the enum discriminant equals that frequency.
        #[test]
        fn tcxo_hz_is_valid(use40 in any::<bool>()) {
            let f = if use40 { TcxoFreq::MHz40 } else { TcxoFreq::MHz24 };
            let hz = f.hz();
            prop_assert!(hz == 24_000_000 || hz == 40_000_000);
            prop_assert_eq!(hz, f as u32); // discriminant == frequency
        }
    }
}
