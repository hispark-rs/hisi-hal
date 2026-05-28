//! GPIO driver for WS63 (19 pins: GPIO0[7:0], GPIO1[15:8], GPIO2[18:16]).
//!
//! Three GPIO blocks at 0x4402_8000, 0x4402_9000, 0x4402_A000.

use core::marker::PhantomData;
use crate::peripherals::{Gpio0, Gpio1, Gpio2, IoConfig};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Pull { None, Up, Down }

pub struct Input;
pub struct Output;

pub struct GpioPin<'d, MODE> {
    block: u8,
    bit: u8,
    _mode: PhantomData<&'d MODE>,
}

fn regs(block: u8) -> &'static ws63_pac::gpio0::RegisterBlock {
    unsafe {
        match block {
            0 => &*Gpio0::ptr(),
            1 => &*Gpio1::ptr(),
            2 => &*Gpio2::ptr(),
            _ => unreachable!(),
        }
    }
}

impl<MODE> GpioPin<'_, MODE> {
    pub fn number(&self) -> u8 { self.block * 8 + self.bit }
}

impl GpioPin<'_, Output> {
    pub fn set_high(&mut self) { unsafe { regs(self.block).gpio_data_set().write(|w| w.bits(1 << self.bit)) }; }
    pub fn set_low(&mut self) { unsafe { regs(self.block).gpio_data_clr().write(|w| w.bits(1 << self.bit)) }; }
    pub fn toggle(&mut self) {
        let r = regs(self.block);
        let val = r.gpio_sw_out().read().bits();
        if val & (1 << self.bit) != 0 {
            unsafe { r.gpio_data_clr().write(|w| w.bits(1 << self.bit)) };
        } else {
            unsafe { r.gpio_data_set().write(|w| w.bits(1 << self.bit)) };
        }
    }
    pub fn is_set_high(&self) -> bool {
        (regs(self.block).gpio_sw_out().read().bits() >> self.bit) & 1 != 0
    }
    pub fn into_input(self) -> GpioPin<'static, Input> {
        regs(self.block).gpio_sw_oen().modify(|r, w| unsafe { w.bits(r.bits() | (1 << self.bit)) });
        GpioPin { block: self.block, bit: self.bit, _mode: PhantomData }
    }
}

impl GpioPin<'_, Input> {
    pub fn is_high(&self) -> bool {
        (regs(self.block).gpio_sw_out().read().bits() >> self.bit) & 1 != 0
    }
    pub fn is_low(&self) -> bool { !self.is_high() }
    pub fn enable_interrupt(&self) {
        regs(self.block).gpio_int_en().modify(|r, w| unsafe { w.bits(r.bits() | (1 << self.bit)) });
    }
    pub fn disable_interrupt(&self) {
        regs(self.block).gpio_int_en().modify(|r, w| unsafe { w.bits(r.bits() & !(1 << self.bit)) });
    }
    pub fn clear_interrupt(&self) {
        unsafe { regs(self.block).gpio_int_eoi().write(|w| w.bits(1 << self.bit)) };
    }
    pub fn interrupt_pending(&self) -> bool {
        (regs(self.block).gpio_int_raw().read().bits() >> self.bit) & 1 != 0
    }
    pub fn into_output(self) -> GpioPin<'static, Output> {
        regs(self.block).gpio_sw_oen().modify(|r, w| unsafe { w.bits(r.bits() & !(1 << self.bit)) });
        GpioPin { block: self.block, bit: self.bit, _mode: PhantomData }
    }
}

impl embedded_hal::digital::ErrorType for GpioPin<'_, Output> {
    type Error = core::convert::Infallible;
}
impl embedded_hal::digital::OutputPin for GpioPin<'_, Output> {
    fn set_low(&mut self) -> Result<(), Self::Error> { GpioPin::set_low(self); Ok(()) }
    fn set_high(&mut self) -> Result<(), Self::Error> { GpioPin::set_high(self); Ok(()) }
}
impl embedded_hal::digital::StatefulOutputPin for GpioPin<'_, Output> {
    fn is_set_high(&mut self) -> Result<bool, Self::Error> { Ok(GpioPin::is_set_high(self)) }
    fn is_set_low(&mut self) -> Result<bool, Self::Error> { Ok(!GpioPin::is_set_high(self)) }
}
impl embedded_hal::digital::ErrorType for GpioPin<'_, Input> {
    type Error = core::convert::Infallible;
}
impl embedded_hal::digital::InputPin for GpioPin<'_, Input> {
    fn is_high(&mut self) -> Result<bool, Self::Error> { Ok(GpioPin::is_high(self)) }
    fn is_low(&mut self) -> Result<bool, Self::Error> { Ok(!GpioPin::is_high(self)) }
}

/// IO MUX configuration.
pub struct Io<'d> { pub io_config: IoConfig<'d> }

impl<'d> Io<'d> {
    pub fn new(io_config: IoConfig<'d>) -> Self { Self { io_config } }
    pub fn register_block(&self) -> &ws63_pac::io_config::RegisterBlock { self.io_config.register_block() }
}

pub fn create_input_pin(pin: u8) -> GpioPin<'static, Input> {
    let block = pin / 8;
    let bit = pin % 8;
    regs(block).gpio_sw_oen().modify(|r, w| unsafe { w.bits(r.bits() | (1 << bit)) });
    GpioPin { block, bit, _mode: PhantomData }
}

pub fn create_output_pin(pin: u8) -> GpioPin<'static, Output> {
    let block = pin / 8;
    let bit = pin % 8;
    regs(block).gpio_sw_oen().modify(|r, w| unsafe { w.bits(r.bits() & !(1 << bit)) });
    GpioPin { block, bit, _mode: PhantomData }
}
