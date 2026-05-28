//! Prelude — re-exports commonly used traits and types.

pub use embedded_hal::digital::InputPin as _;
pub use embedded_hal::digital::OutputPin as _;
pub use embedded_hal::digital::StatefulOutputPin as _;
pub use embedded_io::{Read as _, Write as _};
pub use nb::block;

pub use crate::gpio::{GpioPin, Input, Output, Pull};
pub use crate::i2c::I2c;
pub use crate::pwm::PwmChannel;
pub use crate::spi::Spi;
pub use crate::timer::TimerDriver;
pub use crate::uart::Uart;
pub use crate::Peripherals;
