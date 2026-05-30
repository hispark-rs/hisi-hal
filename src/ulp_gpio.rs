//! Ultra-Low-Power GPIO (ULP_GPIO) driver for WS63.
//!
//! The WS63 ULP_GPIO provides 8 additional GPIO pins (GPIO107-114) that
//! can operate in ultra-low-power modes. The register layout is identical
//! to the standard GPIO blocks.
//!
//! These pins are accessible at physical address 0x5703_0000 and use
//! the same register layout as GPIO0.

use crate::peripherals::UlpGpio;
use core::marker::PhantomData;

/// ULP GPIO pin mode marker types.
pub struct Input;
pub struct Output;

/// ULP GPIO pin driver.
pub struct UlpGpioPin<'d, MODE> {
    bit: u8,
    _mode: PhantomData<&'d MODE>,
}

fn regs() -> &'static ws63_pac::gpio0::RegisterBlock {
    // SAFETY: PAC peripheral pointer is a static physical MMIO address, always valid
    unsafe { &*UlpGpio::ptr() }
}

impl<MODE> UlpGpioPin<'_, MODE> {
    /// Get the pin number (107-114).
    pub fn number(&self) -> u8 {
        107 + self.bit
    }
}

impl UlpGpioPin<'_, Output> {
    /// Set the pin high.
    pub fn set_high(&mut self) {
        unsafe { regs().gpio_data_set().write(|w| w.bits(1 << self.bit)) };
    }

    /// Set the pin low.
    pub fn set_low(&mut self) {
        unsafe { regs().gpio_data_clr().write(|w| w.bits(1 << self.bit)) };
    }

    /// Toggle the pin output level.
    pub fn toggle(&mut self) {
        let val = regs().gpio_sw_out().read().bits();
        if val & (1 << self.bit) != 0 {
            unsafe { regs().gpio_data_clr().write(|w| w.bits(1 << self.bit)) };
        } else {
            unsafe { regs().gpio_data_set().write(|w| w.bits(1 << self.bit)) };
        }
    }

    /// Check if the output is set high.
    pub fn is_set_high(&self) -> bool {
        (regs().gpio_sw_out().read().bits() >> self.bit) & 1 != 0
    }

    /// Convert the pin to input mode.
    pub fn into_input(self) -> UlpGpioPin<'static, Input> {
        regs().gpio_sw_oen().modify(|r, w| unsafe { w.bits(r.bits() | (1 << self.bit)) });
        UlpGpioPin { bit: self.bit, _mode: PhantomData }
    }
}

impl UlpGpioPin<'_, Input> {
    /// Check if the input is high.
    pub fn is_high(&self) -> bool {
        (regs().gpio_sw_out().read().bits() >> self.bit) & 1 != 0
    }

    /// Check if the input is low.
    pub fn is_low(&self) -> bool {
        !self.is_high()
    }

    /// Enable the interrupt for this pin.
    pub fn enable_interrupt(&self) {
        regs().gpio_int_en().modify(|r, w| unsafe { w.bits(r.bits() | (1 << self.bit)) });
    }

    /// Disable the interrupt for this pin.
    pub fn disable_interrupt(&self) {
        regs().gpio_int_en().modify(|r, w| unsafe { w.bits(r.bits() & !(1 << self.bit)) });
    }

    /// Clear the interrupt for this pin.
    pub fn clear_interrupt(&self) {
        unsafe { regs().gpio_int_eoi().write(|w| w.bits(1 << self.bit)) };
    }

    /// Check if an interrupt is pending for this pin.
    pub fn interrupt_pending(&self) -> bool {
        (regs().gpio_int_raw().read().bits() >> self.bit) & 1 != 0
    }

    /// Convert the pin to output mode.
    pub fn into_output(self) -> UlpGpioPin<'static, Output> {
        regs().gpio_sw_oen().modify(|r, w| unsafe { w.bits(r.bits() & !(1 << self.bit)) });
        UlpGpioPin { bit: self.bit, _mode: PhantomData }
    }
}

// ── embedded-hal traits ────────────────────────────────────────────

impl embedded_hal::digital::ErrorType for UlpGpioPin<'_, Output> {
    type Error = core::convert::Infallible;
}
impl embedded_hal::digital::OutputPin for UlpGpioPin<'_, Output> {
    fn set_low(&mut self) -> Result<(), Self::Error> {
        UlpGpioPin::set_low(self);
        Ok(())
    }
    fn set_high(&mut self) -> Result<(), Self::Error> {
        UlpGpioPin::set_high(self);
        Ok(())
    }
}
impl embedded_hal::digital::StatefulOutputPin for UlpGpioPin<'_, Output> {
    fn is_set_high(&mut self) -> Result<bool, Self::Error> {
        Ok(UlpGpioPin::is_set_high(self))
    }
    fn is_set_low(&mut self) -> Result<bool, Self::Error> {
        Ok(!UlpGpioPin::is_set_high(self))
    }
}
impl embedded_hal::digital::ErrorType for UlpGpioPin<'_, Input> {
    type Error = core::convert::Infallible;
}
impl embedded_hal::digital::InputPin for UlpGpioPin<'_, Input> {
    fn is_high(&mut self) -> Result<bool, Self::Error> {
        Ok(UlpGpioPin::is_high(self))
    }
    fn is_low(&mut self) -> Result<bool, Self::Error> {
        Ok(!UlpGpioPin::is_high(self))
    }
}

/// Create a ULP GPIO pin in input mode.
pub fn create_input_pin(pin: u8) -> UlpGpioPin<'static, Input> {
    let bit = pin - 107;
    assert!(bit < 8, "ULP GPIO pin must be 107-114");
    regs().gpio_sw_oen().modify(|r, w| unsafe { w.bits(r.bits() | (1 << bit)) });
    UlpGpioPin { bit, _mode: PhantomData }
}

/// Create a ULP GPIO pin in output mode.
pub fn create_output_pin(pin: u8) -> UlpGpioPin<'static, Output> {
    let bit = pin - 107;
    assert!(bit < 8, "ULP GPIO pin must be 107-114");
    regs().gpio_sw_oen().modify(|r, w| unsafe { w.bits(r.bits() & !(1 << bit)) });
    UlpGpioPin { bit, _mode: PhantomData }
}
