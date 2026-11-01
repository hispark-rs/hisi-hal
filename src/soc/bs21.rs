//! BS21 / BS2X chip-specific description (SparkLink/NearLink, no Wi-Fi).
//!
//! The active SoC description under `--features chip-bs21,unstable`. Provides the same
//! contract `soc/ws63.rs` does (the `Interrupt` enum + clock constants + instance
//! counts) so the chip-neutral drivers read BS21 numbers through `soc::chip`.
//!
//! Facts from the fbb_bs2x SDK (see `docs/bs21-recon.md`). Some clock values are
//! placeholders pending the BS21 `clocks.c` audit; they do not affect milestone-1
//! (blinky busy-loops on the CPU clock, and the QEMU UART model ignores baud).
#![allow(dead_code)]

pub use bs2x_pac::interrupt::ExternalInterrupt as Interrupt;

/// TIMER channel 0 interrupt (legacy constant name retained for compatibility).
/// This identifies hardware only; it reserves no channel for any scheduler.
pub const ALARM_INTERRUPT: Interrupt = Interrupt::TIMER_0;

/// CPU clock (64 MHz — BS21/BS21E/BS22 all run the app core at 64 MHz).
pub const SYSTEM_CLOCK_HZ: u32 = 64_000_000;

/// TCXO / crystal reference — the Timer/WDT counting clock and TCXO time base.
/// TODO(bs21): confirm the exact crystal (32 MHz assumed) from `bs2x/clocks`.
pub const TCXO_HZ: u32 = 32_000_000;

/// Timer / Watchdog counting clock (= [`TCXO_HZ`]).
pub const TIMER_CLOCK_HZ: u32 = TCXO_HZ;

/// UART baud-base clock. TODO(bs21): confirm from `bs2x/clocks` (32 MHz assumed).
pub const UART_CLOCK_HZ: u32 = 32_000_000;

/// SPI controller input clock. TODO(bs21): confirm (32 MHz assumed).
pub const SPI_CLOCK_HZ: u32 = 32_000_000;

/// I2C peripheral clock (= [`TCXO_HZ`]).
pub const I2C_CLOCK_HZ: u32 = TCXO_HZ;

/// Number of GPIO pins (S_MGPIO0..31).
pub const GPIO_COUNT: usize = 32;

/// Number of ULP GPIO pins.
pub const ULP_GPIO_COUNT: usize = 8;

/// Number of UART instances (UART_L0 / UART_H0 / UART_L1).
pub const UART_COUNT: usize = 3;

/// Number of I2C instances.
pub const I2C_COUNT: usize = 2;

/// Number of SPI instances (SPI_M0 / SPI_MS_1 / SPI_MS_2).
pub const SPI_COUNT: usize = 3;

/// Number of PWM channels.
pub const PWM_CHANNEL_COUNT: usize = 12;

/// Number of DMA channels (M_DMA = 8; S_DMA = 4).
pub const DMA_CHANNEL_COUNT: usize = 8;

/// Number of TIMER instances (TIMER_0..3).
pub const TIMER_COUNT: usize = 4;

/// Number of ADC (GADC) channels (8-channel 13-bit).
pub const LSADC_CHANNEL_COUNT: usize = 8;

/// TCXO counter width in bits.
pub const TCXO_COUNTER_WIDTH: usize = 64;

/// RTC counter width in bits.
pub const RTC_COUNTER_WIDTH: usize = 48;
