//! WS63 chip-specific PAC re-export and configuration.
#![allow(dead_code)]

pub use ws63_pac::interrupt::ExternalInterrupt as Interrupt;

/// Interrupt fired by the TIMER channel the embassy-time driver uses for alarms.
///
/// WS63 routes TIMER channel 0 to `TIMER_INT0` (IRQ 26, a standard `mie`-bit
/// local interrupt). The `embassy` time-driver and the app's trap handler reach
/// this through [`crate::soc::chip::ALARM_INTERRUPT`] so the alarm wiring is
/// chip-neutral (BS2X uses a different IRQ — see `soc/bs21.rs`).
pub const ALARM_INTERRUPT: Interrupt = Interrupt::TIMER_INT0;

/// System clock frequency (240 MHz).
///
/// # Clock initialization
///
/// **Boot ROM configures the CPU PLL to 240 MHz before loading the application.**
///
/// The fbb_ws63 flashboot bootloader sequence:
/// 1. `boot_clock_adapt()` — detects TCXO (24/40 MHz) via HW_CTL bit[0],
///    configures UART baud base and WDT tick counter to match TCXO freq
/// 2. `switch_flash_clock_to_pll()` — sets CLDO_CRG_CLK_SEL bit[18] to switch
///    the flash/SFC controller clock from TCXO to PLL
/// 3. Jumps to application entry point
///
/// **What the application must do:** switch peripheral clocks (UART, SPI, I2C)
/// from TCXO to PLL source. Use `clock_init::init_clocks()` for this.
///
/// If the PLL is NOT locked (unlikely, indicates hardware issue), the CPU
/// runs from the TCXO at 24 or 40 MHz. All timing calculations will be wrong.
/// Call `clock_init::probe_clocks()` to verify.
pub const SYSTEM_CLOCK_HZ: u32 = 240_000_000;

/// TCXO crystal frequency (24 MHz on standard WS63 modules).
///
/// This is the **counting clock of the Timer and Watchdog peripherals and the
/// TCXO time base** — NOT the 240 MHz CPU/PLL clock ([`SYSTEM_CLOCK_HZ`]).
///
/// The vendor SDK programs both the timer and the WDT to the TCXO crystal in
/// `clock_init` — `timer_porting_clock_value_set(REQ_24M)` and
/// `watchdog_port_set_clock(REQ_24M)` (fbb_ws63 `clock_init.c`), where
/// `REQ_24M = 24_000_000`. The 32 MHz `CONFIG_TIMER_CLOCK_VALUE` Kconfig default
/// and the 24 MHz "FPGA" WDT note are placeholders the silicon path overrides to
/// this value. ch2 of ws63-guide lists Timer = "晶体分频" (crystal-derived).
///
/// 40 MHz-crystal boards run the timer/WDT at 40 MHz instead; override there.
pub const TCXO_HZ: u32 = 24_000_000;

/// Timer / Watchdog counting clock (= [`TCXO_HZ`], 24 MHz).
///
/// Use this — **not** [`SYSTEM_CLOCK_HZ`] — to convert microseconds/milliseconds
/// to Timer or WDT counter ticks. The timer is clocked by the crystal, so at
/// 24 MHz one microsecond is 24 ticks (the vendor `timer_porting_us_2_cycle`
/// computes `us * (clock / 1_000_000)` = `us * 24`).
pub const TIMER_CLOCK_HZ: u32 = TCXO_HZ;

/// UART baud-base clock (160 MHz, PLL-derived).
///
/// After boot, `clock_init` switches the UART clock from TCXO to PLL and sets the
/// baud base to `UART_PLL_CLOCK = 160_000_000` (fbb_ws63 `clock_init.c`); ch2 of
/// ws63-guide also lists UART = 160 MHz. The baud divisor is `clock / (16 * baud)`,
/// so this — not the 240 MHz CPU clock — is the divisor base.
///
/// **On-silicon caveat (2026-06-14, see hisi-riscv-rs#10/#15):** examples that do
/// NOT run the (XIP-unsafe) full `clock_init` inherit flashboot's UART clock, which
/// is NOT this PLL base — so their baud is wrong on real hardware. The boot UART
/// clock is the raw TCXO crystal (24 or 40 MHz), selected per board; use
/// [`crate::uart::UartClock::Boot`] for those examples.
pub const UART_CLOCK_HZ: u32 = 160_000_000;

/// Boot (pre-`clock_init`) UART baud base for a 24 MHz-TCXO board.
pub const UART_BOOT_CLOCK_24M_HZ: u32 = 24_000_000;
/// Boot (pre-`clock_init`) UART baud base for a 40 MHz-TCXO board.
pub const UART_BOOT_CLOCK_40M_HZ: u32 = 40_000_000;

/// Live flashboot UART baud base (the raw TCXO crystal, 24 or 40 MHz) for use
/// BEFORE `clock_init` runs.
///
/// flashboot's `boot_clock_adapt()` sets the console UART baud base to the raw
/// TCXO — NOT the 160 MHz PLL — keyed on the `HW_CTL` TCXO-detect strap (fbb_ws63
/// `soc_porting.c` `get_tcxo_freq`/`boot_clock_adapt`). This mirrors that read.
/// **Confirmed on silicon (2026-06-14):** a 40 MHz board reads `HW_CTL = 0x2`
/// (bit 0 clear), and at 115200 8N1 that yields the measured flashboot divider
/// `div = 21, div_fra = 44` — the divider the HAL must reproduce for examples
/// that skip `clock_init` (hisi-riscv-rs#10/#15).
///
/// [`crate::uart::UartClock::Boot`] uses this value so the divider is computed
/// against the real base.
pub fn uart_boot_clock_hz() -> u32 {
    // HW_CTL TCXO-frequency-detect strap; bit 0: 1 = 24 MHz TCXO, 0 = 40 MHz
    // (CLK24M_TCXO = 1, CLK40M_TCXO = 0 in the vendor `soc_porting.h`).
    let sys_ctl0 = unsafe { &*crate::soc::pac::SysCtl0::ptr() };
    if sys_ctl0.hw_ctl().read().refclk_freq_status().bit_is_set() {
        UART_BOOT_CLOCK_24M_HZ
    } else {
        UART_BOOT_CLOCK_40M_HZ
    }
}

/// SPI controller input clock / SSI_CLK (160 MHz, PLL-derived).
///
/// SCK = SSI_CLK / SCKDV. WS63 derives the SPI clock from the 480 MHz FNPLL tap via
/// a two-stage divider: a CLDO_CRG divider sets SSI_CLK (= this value, 480/3), then
/// the in-controller SCKDV divides to SCK. `spi.rs::configure_spi_source_clock`
/// programs the CRG divider (`DIV_CTL3`) + switches the source to PLL on init —
/// mirroring the vendor `spi_porting_clock_init` — so this SSI_CLK is established,
/// not just assumed. ch2 of ws63-guide lists SPI = 160 MHz; the QEMU model matches.
pub const SPI_CLOCK_HZ: u32 = 160_000_000;

/// I2C peripheral clock (= [`TCXO_HZ`], 24 MHz).
///
/// Unlike UART/SPI, the I2C clock is **not** switched to the PLL: `clock_init` sets
/// it to the TCXO crystal via `i2c_port_set_clock_value(REQ_24M)` (fbb_ws63
/// `clock_init.c`), and the SCL divisor math (`SCL = clock / (2*(scl_h+scl_l)*2)`)
/// is computed against this value. (ch2's nominal "I2C = 80 MHz" is the bus-capability
/// figure, not the divisor base the SDK uses.)
pub const I2C_CLOCK_HZ: u32 = TCXO_HZ;

/// Number of GPIO pins (19: GPIO0[7:0] + GPIO1[15:8] + GPIO2[18:16]).
pub const GPIO_COUNT: usize = 19;

/// Number of ULP GPIO pins (8: GPIO107-114).
pub const ULP_GPIO_COUNT: usize = 8;

/// Number of UART instances.
pub const UART_COUNT: usize = 3;

/// Number of I2C instances.
pub const I2C_COUNT: usize = 2;

/// Number of SPI instances.
pub const SPI_COUNT: usize = 2;

/// Number of PWM channels.
pub const PWM_CHANNEL_COUNT: usize = 8;

/// Number of DMA channels (per controller).
pub const DMA_CHANNEL_COUNT: usize = 4;

/// Number of TIMER instances.
pub const TIMER_COUNT: usize = 3;

/// Number of LSADC channels.
pub const LSADC_CHANNEL_COUNT: usize = 6;

/// TCXO counter width in bits.
pub const TCXO_COUNTER_WIDTH: usize = 64;

/// RTC counter width in bits.
pub const RTC_COUNTER_WIDTH: usize = 48;
