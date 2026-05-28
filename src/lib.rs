//! # ws63-hal — Hardware Abstraction Layer for HiSilicon WS63 (RISC-V).
//!
//! GPIO, UART, I2C, SPI, PWM, Timer, Clock, Interrupt management.
#![no_std]
#![allow(non_camel_case_types)]

pub mod clock;
pub mod gpio;
pub mod i2c;
pub mod interrupt;
pub mod peripherals;
pub mod prelude;
pub mod pwm;
pub mod spi;
pub mod system;
pub mod timer;
pub mod uart;

mod soc;

pub use peripherals::Peripherals;
pub use system::System;
