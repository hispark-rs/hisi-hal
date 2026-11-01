//! Prelude — re-exports commonly used traits and types.

pub use embedded_hal::delay::DelayNs as _;
pub use embedded_hal::digital::InputPin as _;
pub use embedded_hal::digital::OutputPin as _;
pub use embedded_hal::digital::StatefulOutputPin as _;
pub use embedded_hal::i2c::I2c as _;
pub use embedded_hal::spi::SpiBus as _;
pub use embedded_io::{Read as _, Write as _};
pub use nb::block;

pub use crate::Peripherals;
#[cfg(feature = "unstable")]
#[cfg(not(feature = "chip-ws53"))]
pub use crate::delay::Delay;
#[cfg(feature = "unstable")]
#[cfg(not(feature = "chip-ws53"))]
pub use crate::gpio::InterruptTrigger;
#[cfg(not(feature = "chip-ws53"))]
pub use crate::gpio::{AnyPin, Flex, GpioBank, Input, InputConfig, Output, OutputConfig, Pull};
pub use crate::interrupt::{Interrupt, Priority, Threshold};
#[cfg(not(feature = "chip-ws53"))]
pub use crate::tcxo::TcxoDriver;
pub use crate::time::{Duration, Instant, Rate};
#[cfg(feature = "unstable")]
#[cfg(not(feature = "chip-ws53"))]
pub use crate::timer::{OneShotTimer, PeriodicTimer};
pub use crate::timer::{TimerChannel, TimerDriver};
pub use crate::uart::{Uart, UartClock, UartPort};
#[cfg(feature = "chip-ws63")]
pub use crate::{io_config::IoConfigDriver, system::System};

// WS63-only drivers (gated to chip-ws63 until ported to BS21).
#[cfg(feature = "chip-ws63")]
pub use crate::clock::Peripheral;
#[cfg(all(feature = "chip-ws63", feature = "unstable"))]
pub use crate::dma::{Dma0, DmaDriver, Sdma0};
#[cfg(feature = "chip-ws63")]
pub use crate::efuse::{EfuseByteAddress, EfuseDriver};
#[cfg(feature = "chip-ws63")]
pub use crate::i2c::I2c;
#[cfg(feature = "chip-ws63")]
pub use crate::i2s::I2sDriver;
#[cfg(feature = "chip-ws63")]
pub use crate::lsadc::LsAdc;
#[cfg(feature = "chip-ws63")]
pub use crate::pwm::{Duty, PwmChannel, PwmChannelId, PwmPeriod};
#[cfg(all(feature = "chip-ws63", feature = "unstable"))]
pub use crate::rtc::RtcDriver;
#[cfg(all(feature = "chip-ws63", feature = "unstable"))]
pub use crate::sfc::SfcDriver;
#[cfg(feature = "chip-ws63")]
pub use crate::spi::Spi;
#[cfg(feature = "chip-ws63")]
pub use crate::trng::TrngDriver;
#[cfg(feature = "chip-ws63")]
pub use crate::tsensor::TempSensor;
#[cfg(all(feature = "chip-ws63", feature = "unstable"))]
pub use crate::ulp_gpio::UlpGpioPin;
#[cfg(feature = "chip-ws63")]
pub use crate::wdt::Watchdog;
