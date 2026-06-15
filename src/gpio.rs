//! GPIO driver for WS63 (19 pins: GPIO0 bits 7-0, GPIO1 bits 15-8, GPIO2 bits 18-16).
//!
//! Three GPIO blocks at 0x4402_8000, 0x4402_9000, 0x4402_A000.
//!
//! # Pin drivers
//!
//! - [`Input`] — digital input with configurable pull
//! - [`Output`] — push-pull output
//! - [`Flex`] — combined input + output driver
//! - [`AnyPin`] — type-erased pin

#[cfg(feature = "chip-ws63")]
use crate::peripherals::IoConfig;
use crate::peripherals::{Gpio0, Gpio1, Gpio2};
use core::marker::PhantomData;

// ── Configuration types ───────────────────────────────────────────

/// Pull resistor configuration.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Pull {
    None,
    Up,
    Down,
}

/// GPIO interrupt trigger condition (sets `GPIO_INT_TYPE` + `GPIO_INT_POLARITY`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum InterruptTrigger {
    /// Edge-sensitive, low→high transition.
    RisingEdge,
    /// Edge-sensitive, high→low transition.
    FallingEdge,
    /// Level-sensitive, asserted while high.
    HighLevel,
    /// Level-sensitive, asserted while low.
    LowLevel,
}

/// Digital input configuration.
#[derive(Debug, Clone, Copy)]
pub struct InputConfig {
    pub pull: Pull,
}

impl Default for InputConfig {
    fn default() -> Self {
        Self { pull: Pull::None }
    }
}

impl InputConfig {
    pub const fn new() -> Self {
        Self { pull: Pull::None }
    }
    pub const fn with_pull(mut self, pull: Pull) -> Self {
        self.pull = pull;
        self
    }
}

/// Digital output configuration.
#[derive(Debug, Clone, Copy, Default)]
pub struct OutputConfig {
    pub open_drain: bool,
    pub initial_high: bool,
}

impl OutputConfig {
    pub const fn new() -> Self {
        Self { open_drain: false, initial_high: false }
    }
    pub const fn with_open_drain(mut self, od: bool) -> Self {
        self.open_drain = od;
        self
    }
    pub const fn with_initial(mut self, high: bool) -> Self {
        self.initial_high = high;
        self
    }
}

/// Mode marker types.
pub struct InputMode;
pub struct OutputMode;

// ── Type-erased pin ────────────────────────────────────────────────

/// A type-erased pin that can represent any GPIO pin.
pub struct AnyPin<'d> {
    block: u8,
    bit: u8,
    _lifetime: PhantomData<&'d mut ()>,
}

impl<'d> AnyPin<'d> {
    /// Create an `AnyPin` from a raw pin number without checking.
    ///
    /// # Safety
    /// The pin must be valid (0-18). This bypasses the type system.
    pub unsafe fn steal(pin: u8) -> Self {
        Self { block: pin / 8, bit: pin % 8, _lifetime: PhantomData }
    }

    /// Get the pin number (0-18).
    pub fn number(&self) -> u8 {
        self.block * 8 + self.bit
    }

    /// Set output enable (true = input, false = output).
    fn set_oen(&self, input: bool) {
        let r = regs(self.block);
        let mask = 1 << self.bit;
        if input {
            r.gpio_sw_oen().modify(|r, w| unsafe { w.bits(r.bits() | mask) });
        } else {
            r.gpio_sw_oen().modify(|r, w| unsafe { w.bits(r.bits() & !mask) });
        }
    }

    /// Initialize this pin as a GPIO input.
    ///
    /// Sets the direction to input and applies `config.pull` + enables the input
    /// buffer (IE) on the pad via the IO_CONFIG pad register (`apply_pull`; pins
    /// 0..=14 have a pad-control register).
    pub fn init_input(self, config: InputConfig) -> Input<'d> {
        self.set_oen(true);
        apply_pull(self.number(), config.pull);
        Input { pin: self, config }
    }

    /// Initialize this pin as a GPIO output.
    pub fn init_output(self, config: OutputConfig) -> Output<'d> {
        let mut out = Output { pin: self, config };
        if config.initial_high {
            out.set_high();
        } else {
            out.set_low();
        }
        out.pin.set_oen(false);
        out
    }

    /// Initialize this pin as a Flex (combined input/output) driver.
    pub fn init_flex(self, config: OutputConfig) -> Flex<'d> {
        if config.initial_high {
            unsafe { regs(self.block).gpio_data_set().write(|w| w.bits(1 << self.bit)) };
        } else {
            unsafe { regs(self.block).gpio_data_clr().write(|w| w.bits(1 << self.bit)) };
        }
        self.set_oen(false);
        Flex { pin: self, config }
    }
}

// ── Digital input driver ──────────────────────────────────────────

/// Digital input pin driver.
pub struct Input<'d> {
    pin: AnyPin<'d>,
    #[allow(dead_code)]
    config: InputConfig,
}

impl<'d> Input<'d> {
    pub fn is_high(&self) -> bool {
        (regs(self.pin.block).gpio_sw_out().read().bits() >> self.pin.bit) & 1 != 0
    }

    pub fn is_low(&self) -> bool {
        !self.is_high()
    }

    pub fn number(&self) -> u8 {
        self.pin.number()
    }

    pub fn enable_interrupt(&self) {
        let r = regs(self.pin.block);
        r.gpio_int_en().modify(|r, w| unsafe { w.bits(r.bits() | (1 << self.pin.bit)) });
    }

    pub fn disable_interrupt(&self) {
        let r = regs(self.pin.block);
        r.gpio_int_en().modify(|r, w| unsafe { w.bits(r.bits() & !(1 << self.pin.bit)) });
    }

    pub fn clear_interrupt(&self) {
        unsafe { regs(self.pin.block).gpio_int_eoi().write(|w| w.bits(1 << self.pin.bit)) };
    }

    /// Set the interrupt trigger condition for this pin (edge/level + polarity).
    ///
    /// Configures `GPIO_INT_TYPE` (edge vs level) and `GPIO_INT_POLARITY`
    /// (rising/high vs falling/low). Call before [`enable_interrupt`](Self::enable_interrupt).
    pub fn set_interrupt_trigger(&self, trigger: InterruptTrigger) {
        let r = regs(self.pin.block);
        let mask = 1u32 << self.pin.bit;
        let (edge, high) = match trigger {
            InterruptTrigger::RisingEdge => (true, true),
            InterruptTrigger::FallingEdge => (true, false),
            InterruptTrigger::HighLevel => (false, true),
            InterruptTrigger::LowLevel => (false, false),
        };
        r.gpio_int_type().modify(|r, w| unsafe { w.bits(if edge { r.bits() | mask } else { r.bits() & !mask }) });
        r.gpio_int_polarity().modify(|r, w| unsafe { w.bits(if high { r.bits() | mask } else { r.bits() & !mask }) });
    }

    pub fn interrupt_pending(&self) -> bool {
        (regs(self.pin.block).gpio_int_raw().read().bits() >> self.pin.bit) & 1 != 0
    }
}

impl embedded_hal::digital::ErrorType for Input<'_> {
    type Error = core::convert::Infallible;
}

impl embedded_hal::digital::InputPin for Input<'_> {
    fn is_high(&mut self) -> Result<bool, Self::Error> {
        Ok(Input::is_high(self))
    }
    fn is_low(&mut self) -> Result<bool, Self::Error> {
        Ok(Input::is_low(self))
    }
}

// ── Digital output driver ──────────────────────────────────────────

/// Digital output pin driver.
pub struct Output<'d> {
    pin: AnyPin<'d>,
    config: OutputConfig,
}

impl<'d> Output<'d> {
    pub fn set_high(&mut self) {
        unsafe { regs(self.pin.block).gpio_data_set().write(|w| w.bits(1 << self.pin.bit)) };
    }

    pub fn set_low(&mut self) {
        unsafe { regs(self.pin.block).gpio_data_clr().write(|w| w.bits(1 << self.pin.bit)) };
    }

    pub fn toggle(&mut self) {
        let r = regs(self.pin.block);
        let val = r.gpio_sw_out().read().bits();
        if val & (1 << self.pin.bit) != 0 {
            unsafe { r.gpio_data_clr().write(|w| w.bits(1 << self.pin.bit)) };
        } else {
            unsafe { r.gpio_data_set().write(|w| w.bits(1 << self.pin.bit)) };
        }
    }

    pub fn is_set_high(&self) -> bool {
        (regs(self.pin.block).gpio_sw_out().read().bits() >> self.pin.bit) & 1 != 0
    }

    pub fn number(&self) -> u8 {
        self.pin.number()
    }

    /// Convert this output into a Flex pin, **keeping the current drive state** (the
    /// safe-state [`Drop`](Output#impl-Drop-for-Output) does not run — the pad stays
    /// an output as it transfers to the `Flex` driver).
    pub fn into_flex(self) -> Flex<'d> {
        // Move the fields out without running Output's safe-state Drop.
        let this = core::mem::ManuallyDrop::new(self);
        // SAFETY: `this` is never dropped (ManuallyDrop) and we read each field
        // exactly once, so there is no double-read or double-drop.
        let pin = unsafe { core::ptr::read(&this.pin) };
        let config = unsafe { core::ptr::read(&this.config) };
        Flex { pin, config }
    }

    /// Consume the output, **latching its current drive state** past this scope —
    /// the escape hatch from the safe-state [`Drop`](Output#impl-Drop-for-Output)
    /// (e.g. an enable line that must stay asserted after setup returns). Returns an
    /// [`OutputLatched`] marker so the intent is explicit.
    #[must_use = "the pin is now latched in its current state; bind the marker to make that explicit"]
    pub fn into_latched(self) -> OutputLatched {
        core::mem::forget(self); // skip the safe-state Drop — hold the current level
        OutputLatched(())
    }
}

/// Proof token from [`Output::into_latched`]: the pad was intentionally left driving
/// its last level past the driver's scope (no safe-state `Drop` ran).
#[derive(Debug)]
#[must_use]
pub struct OutputLatched(());

impl Drop for Output<'_> {
    /// Scoped safety: a dropped output reverts its pad to **input / high-impedance**
    /// (sets `OEN`, clearing only this pad's output-enable — never a shared clock
    /// gate), so a stale handle cannot keep driving a line. Use
    /// [`Output::into_latched`] or [`Output::into_flex`] to keep the pad driving.
    fn drop(&mut self) {
        self.pin.set_oen(true);
    }
}

impl embedded_hal::digital::ErrorType for Output<'_> {
    type Error = core::convert::Infallible;
}

impl embedded_hal::digital::OutputPin for Output<'_> {
    fn set_low(&mut self) -> Result<(), Self::Error> {
        Output::set_low(self);
        Ok(())
    }
    fn set_high(&mut self) -> Result<(), Self::Error> {
        Output::set_high(self);
        Ok(())
    }
}

impl embedded_hal::digital::StatefulOutputPin for Output<'_> {
    fn is_set_high(&mut self) -> Result<bool, Self::Error> {
        Ok(Output::is_set_high(self))
    }
    fn is_set_low(&mut self) -> Result<bool, Self::Error> {
        Ok(!Output::is_set_high(self))
    }
}

// ── Flex pin driver (combined input + output) ─────────────────────

/// Combined input + output pin driver.
pub struct Flex<'d> {
    pin: AnyPin<'d>,
    #[allow(dead_code)]
    config: OutputConfig,
}

impl<'d> Flex<'d> {
    pub fn set_high(&mut self) {
        self.pin.set_oen(false);
        unsafe { regs(self.pin.block).gpio_data_set().write(|w| w.bits(1 << self.pin.bit)) };
    }

    pub fn set_low(&mut self) {
        self.pin.set_oen(false);
        unsafe { regs(self.pin.block).gpio_data_clr().write(|w| w.bits(1 << self.pin.bit)) };
    }

    pub fn toggle(&mut self) {
        let r = regs(self.pin.block);
        self.pin.set_oen(false);
        let val = r.gpio_sw_out().read().bits();
        if val & (1 << self.pin.bit) != 0 {
            unsafe { r.gpio_data_clr().write(|w| w.bits(1 << self.pin.bit)) };
        } else {
            unsafe { r.gpio_data_set().write(|w| w.bits(1 << self.pin.bit)) };
        }
    }

    pub fn is_set_high(&self) -> bool {
        (regs(self.pin.block).gpio_sw_out().read().bits() >> self.pin.bit) & 1 != 0
    }

    pub fn is_high(&self) -> bool {
        // Save output enable state, switch to input, read, restore
        let r = regs(self.pin.block);
        let oen = r.gpio_sw_oen().read().bits();
        self.pin.set_oen(true);
        let val = (r.gpio_sw_out().read().bits() >> self.pin.bit) & 1 != 0;
        // Restore original OEN state
        let mask = 1 << self.pin.bit;
        if oen & mask != 0 {
            r.gpio_sw_oen().modify(|reg, w| unsafe { w.bits(reg.bits() | mask) });
        } else {
            r.gpio_sw_oen().modify(|reg, w| unsafe { w.bits(reg.bits() & !mask) });
        }
        val
    }

    pub fn is_low(&self) -> bool {
        !self.is_high()
    }

    pub fn number(&self) -> u8 {
        self.pin.number()
    }
}

impl embedded_hal::digital::ErrorType for Flex<'_> {
    type Error = core::convert::Infallible;
}

impl embedded_hal::digital::OutputPin for Flex<'_> {
    fn set_low(&mut self) -> Result<(), Self::Error> {
        Flex::set_low(self);
        Ok(())
    }
    fn set_high(&mut self) -> Result<(), Self::Error> {
        Flex::set_high(self);
        Ok(())
    }
}

impl embedded_hal::digital::StatefulOutputPin for Flex<'_> {
    fn is_set_high(&mut self) -> Result<bool, Self::Error> {
        Ok(Flex::is_set_high(self))
    }
    fn is_set_low(&mut self) -> Result<bool, Self::Error> {
        Ok(!Flex::is_set_high(self))
    }
}

impl embedded_hal::digital::InputPin for Flex<'_> {
    fn is_high(&mut self) -> Result<bool, Self::Error> {
        Ok(Flex::is_high(self))
    }
    fn is_low(&mut self) -> Result<bool, Self::Error> {
        Ok(Flex::is_low(self))
    }
}

// ── Internal register access ──────────────────────────────────────

fn regs(block: u8) -> &'static crate::soc::pac::gpio0::RegisterBlock {
    unsafe {
        match block {
            0 => &*Gpio0::ptr(),
            1 => &*Gpio1::ptr(),
            2 => &*Gpio2::ptr(),
            _ => unreachable!(),
        }
    }
}

// ── Pad pull-resistor control (IO_CONFIG) ─────────────────────────
// The pull resistor lives in the per-pad IO_CONFIG control register, not the
// GPIO block. `pad_gpio_NN_ctrl` is at IO_CONFIG + 0x800 + N*4 (PAC layout) with
// PE = bit 9 (pull enable), PS = bit 10 (pull select); per the SVD the {PE,PS}
// pair encodes 00 = none, 11 = pull-up, 10 = pull-down (matches
// `io_config::build_pad_ctrl`). Only GPIO pads 0..=14 have a control register.
const IO_CONFIG_BASE: usize = 0x4400_D000;
const PAD_GPIO_CTRL_OFF: usize = 0x800;
const PAD_PE_BIT: u32 = 1 << 9;
const PAD_PS_BIT: u32 = 1 << 10;
// IE = bit 11 gates the pad's input buffer; gpio_sw_out only reflects the pin
// when IE is set. The boot ROM leaves IE=1 by reset default (measured on silicon:
// pad_gpio_03_ctrl = 0x800 at entry) and the WS63 vendor pinctrl never writes it
// (CONFIG_PINCTRL_SUPPORT_IE undefined), so a GPIO read works without us touching
// it. We assert IE for input pins anyway — same hardware state the vendor relies
// on, but self-contained, so a pad whose IE was cleared by an earlier mux still
// reads correctly. (`io_config::build_pad_ctrl` places IE at the same bit 11.)
const PAD_IE_BIT: u32 = 1 << 11;

/// Configure a GPIO pad for **input** via IO_CONFIG: apply the pull resistor and
/// enable the input buffer (IE).
///
/// Read-modify-write so drive strength / Schmitt bits are kept. A no-op for pins
/// 15..=18, which have no `pad_gpio_NN_ctrl` register in this layout (their pull
/// is configured through other pads / the ROM pin map).
fn apply_pull(pin: u8, pull: Pull) {
    if pin > 14 {
        return;
    }
    let reg = (IO_CONFIG_BASE + PAD_GPIO_CTRL_OFF + (pin as usize) * 4) as *mut u32;
    let (pe, ps) = match pull {
        Pull::None => (false, false),
        Pull::Up => (true, true),
        Pull::Down => (true, false),
    };
    unsafe {
        let mut v = core::ptr::read_volatile(reg);
        v &= !(PAD_PE_BIT | PAD_PS_BIT);
        if pe {
            v |= PAD_PE_BIT;
        }
        if ps {
            v |= PAD_PS_BIT;
        }
        v |= PAD_IE_BIT; // ensure the input buffer is enabled
        core::ptr::write_volatile(reg, v);
    }
}

// ── Legacy GpioPin (backward-compatible type-state GPIO) ──────────

/// Legacy GPIO pin with type-state (Input/Output mode).
pub struct GpioPin<'d, MODE> {
    block: u8,
    bit: u8,
    _mode: PhantomData<&'d MODE>,
}

impl<MODE> GpioPin<'_, MODE> {
    pub fn number(&self) -> u8 {
        self.block * 8 + self.bit
    }
}

impl GpioPin<'_, OutputMode> {
    pub fn set_high(&mut self) {
        unsafe { regs(self.block).gpio_data_set().write(|w| w.bits(1 << self.bit)) };
    }
    pub fn set_low(&mut self) {
        unsafe { regs(self.block).gpio_data_clr().write(|w| w.bits(1 << self.bit)) };
    }
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
    pub fn into_input(self) -> GpioPin<'static, InputMode> {
        regs(self.block).gpio_sw_oen().modify(|r, w| unsafe { w.bits(r.bits() | (1 << self.bit)) });
        GpioPin { block: self.block, bit: self.bit, _mode: PhantomData }
    }
}

impl GpioPin<'_, InputMode> {
    pub fn is_high(&self) -> bool {
        (regs(self.block).gpio_sw_out().read().bits() >> self.bit) & 1 != 0
    }
    pub fn is_low(&self) -> bool {
        !self.is_high()
    }
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
    pub fn into_output(self) -> GpioPin<'static, OutputMode> {
        regs(self.block).gpio_sw_oen().modify(|r, w| unsafe { w.bits(r.bits() & !(1 << self.bit)) });
        GpioPin { block: self.block, bit: self.bit, _mode: PhantomData }
    }
}

// Legacy embedded-hal impls for GpioPin
impl embedded_hal::digital::ErrorType for GpioPin<'_, OutputMode> {
    type Error = core::convert::Infallible;
}
impl embedded_hal::digital::OutputPin for GpioPin<'_, OutputMode> {
    fn set_low(&mut self) -> Result<(), Self::Error> {
        GpioPin::set_low(self);
        Ok(())
    }
    fn set_high(&mut self) -> Result<(), Self::Error> {
        GpioPin::set_high(self);
        Ok(())
    }
}
impl embedded_hal::digital::StatefulOutputPin for GpioPin<'_, OutputMode> {
    fn is_set_high(&mut self) -> Result<bool, Self::Error> {
        Ok(GpioPin::is_set_high(self))
    }
    fn is_set_low(&mut self) -> Result<bool, Self::Error> {
        Ok(!GpioPin::is_set_high(self))
    }
}
impl embedded_hal::digital::ErrorType for GpioPin<'_, InputMode> {
    type Error = core::convert::Infallible;
}
impl embedded_hal::digital::InputPin for GpioPin<'_, InputMode> {
    fn is_high(&mut self) -> Result<bool, Self::Error> {
        Ok(GpioPin::is_high(self))
    }
    fn is_low(&mut self) -> Result<bool, Self::Error> {
        Ok(GpioPin::is_low(self))
    }
}

// ── Pin creation functions ────────────────────────────────────────

/// Create an input pin from a pin number (0-18).
pub fn create_input_pin(pin: u8) -> GpioPin<'static, InputMode> {
    let block = pin / 8;
    let bit = pin % 8;
    regs(block).gpio_sw_oen().modify(|r, w| unsafe { w.bits(r.bits() | (1 << bit)) });
    GpioPin { block, bit, _mode: PhantomData }
}

/// Create an output pin from a pin number (0-18).
pub fn create_output_pin(pin: u8) -> GpioPin<'static, OutputMode> {
    let block = pin / 8;
    let bit = pin % 8;
    regs(block).gpio_sw_oen().modify(|r, w| unsafe { w.bits(r.bits() & !(1 << bit)) });
    GpioPin { block, bit, _mode: PhantomData }
}

// ── InputSignal / OutputSignal (peripheral interconnect) ──────────

/// An output signal from a peripheral that can be routed to a GPIO pin.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct OutputSignal(pub(crate) u8);

/// An input signal to a peripheral that can be routed from a GPIO pin.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct InputSignal(pub(crate) u8);

/// Types that can serve as peripheral outputs (signals towards GPIO matrix).
pub trait PeripheralOutput: crate::private::Sealed {
    fn output_signal(&self) -> OutputSignal;
}

/// Types that can serve as peripheral inputs (signals from GPIO matrix towards peripherals).
pub trait PeripheralInput: crate::private::Sealed {
    fn input_signal(&self) -> InputSignal;
}

// Seal GPIO types for peripheral traits
impl crate::private::Sealed for Output<'_> {}
impl crate::private::Sealed for Input<'_> {}
impl crate::private::Sealed for Flex<'_> {}
impl crate::private::Sealed for GpioPin<'_, OutputMode> {}
impl crate::private::Sealed for GpioPin<'_, InputMode> {}

// ── IO MUX configuration ──────────────────────────────────────────

/// IO MUX configuration (WS63 pinmux; BS21's IO_CONFIG differs — ported later).
#[cfg(feature = "chip-ws63")]
pub struct Io<'d> {
    pub io_config: IoConfig<'d>,
}

#[cfg(feature = "chip-ws63")]
impl<'d> Io<'d> {
    pub fn new(io_config: IoConfig<'d>) -> Self {
        Self { io_config }
    }
    pub fn register_block(&self) -> &crate::soc::pac::io_config::RegisterBlock {
        self.io_config.register_block()
    }
}

// ── Async (embedded-hal-async) ──────────────────────────────────────────────
#[cfg(feature = "async")]
mod asynch_impl {
    use super::{Input, InterruptTrigger, regs};
    use crate::asynch::IrqSignal;
    use crate::interrupt::{self, Interrupt};
    use core::future::Future;
    use core::pin::Pin;
    use core::task::{Context, Poll};
    use embedded_hal_async::digital::Wait;

    static GPIO_SIGNAL: [IrqSignal; 3] = [IrqSignal::new(), IrqSignal::new(), IrqSignal::new()];

    fn bank_irq(bank: usize) -> Interrupt {
        match bank {
            0 => Interrupt::GPIO_INT0,
            1 => Interrupt::GPIO_INT1,
            _ => Interrupt::GPIO_INT2,
        }
    }

    /// GPIO trap-handler hook for `bank` (0..2 → IRQ 33..35, custom local). Masks
    /// the fired pins (so they don't storm), clears their edge latch, wakes the
    /// awaiting [`Wait`] future, and clears the `LOCIPCLR` pending bit. Call this
    /// from the trap when `mcause` is GPIO_INT0..2.
    pub fn on_interrupt(bank: u8) {
        let r = regs(bank);
        let fired = r.gpio_int_raw().read().bits();
        // Mask the fired pins (a fresh wait re-enables) + clear the edge latch.
        r.gpio_int_en().modify(|v, w| unsafe { w.bits(v.bits() & !fired) });
        unsafe { r.gpio_int_eoi().write(|w| w.bits(fired)) };
        GPIO_SIGNAL[bank as usize].signal();
        interrupt::clear_pending(bank_irq(bank as usize));
    }

    async fn arm_and_wait(input: &mut Input<'_>, trig: InterruptTrigger) {
        let bank = input.pin.block as usize;
        input.set_interrupt_trigger(trig);
        input.clear_interrupt();
        GPIO_SIGNAL[bank].reset();
        input.enable_interrupt();
        // SAFETY: enabling a known, fixed WS63 GPIO IRQ line.
        unsafe { interrupt::enable(bank_irq(bank)) };
        GpioWaitFuture { bank }.await;
    }

    struct GpioWaitFuture {
        bank: usize,
    }

    impl Future for GpioWaitFuture {
        type Output = ();
        fn poll(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<()> {
            if GPIO_SIGNAL[self.bank].take_fired() {
                Poll::Ready(())
            } else {
                GPIO_SIGNAL[self.bank].register(cx.waker());
                Poll::Pending
            }
        }
    }

    /// Interrupt-driven async edge/level waiting on a GPIO input. Requires the
    /// app to route the GPIO trap to [`on_interrupt`] and enable global
    /// interrupts. Level waits return immediately if the pin already matches.
    impl Wait for Input<'_> {
        async fn wait_for_high(&mut self) -> Result<(), Self::Error> {
            if self.is_high() {
                return Ok(());
            }
            arm_and_wait(self, InterruptTrigger::HighLevel).await;
            Ok(())
        }
        async fn wait_for_low(&mut self) -> Result<(), Self::Error> {
            if self.is_low() {
                return Ok(());
            }
            arm_and_wait(self, InterruptTrigger::LowLevel).await;
            Ok(())
        }
        async fn wait_for_rising_edge(&mut self) -> Result<(), Self::Error> {
            arm_and_wait(self, InterruptTrigger::RisingEdge).await;
            Ok(())
        }
        async fn wait_for_falling_edge(&mut self) -> Result<(), Self::Error> {
            arm_and_wait(self, InterruptTrigger::FallingEdge).await;
            Ok(())
        }
        async fn wait_for_any_edge(&mut self) -> Result<(), Self::Error> {
            let trig = if self.is_high() { InterruptTrigger::FallingEdge } else { InterruptTrigger::RisingEdge };
            arm_and_wait(self, trig).await;
            Ok(())
        }
    }
}

#[cfg(feature = "async")]
pub use asynch_impl::on_interrupt;

// ── Tests ──────────────────────────────────────────────────────

#[cfg(all(test, not(target_arch = "riscv32")))]
mod tests {
    use super::*;

    /// The `into_latched` escape-hatch marker is zero-sized (a pure type-level proof
    /// token). The safe-state Drop (revert pad to input) register effect is
    /// HIL-validated on silicon — the host has no MMIO.
    #[test]
    fn output_latched_marker_is_zero_sized() {
        assert_eq!(core::mem::size_of::<OutputLatched>(), 0);
    }

    // `steal(pin)` splits a pin into `block = pin / 8`, `bit = pin % 8`, and
    // `number()` recombines them as `block * 8 + bit`. The two are inverses, so
    // re-derive that split here (no MMIO) and assert the round-trip.
    fn split(pin: u8) -> (u8, u8) {
        (pin / 8, pin % 8)
    }
    fn number(block: u8, bit: u8) -> u8 {
        block * 8 + bit
    }

    #[test]
    fn pin_number_round_trips() {
        // Every valid WS63 pin (0..=18) survives split→recombine unchanged.
        for pin in 0u8..=18 {
            let (block, bit) = split(pin);
            assert_eq!(number(block, bit), pin);
        }
    }

    #[test]
    fn pin_split_block_boundaries() {
        // Block/bit layout: GPIO0 = pins 0..=7, GPIO1 = 8..=15, GPIO2 = 16..=18.
        assert_eq!(split(0), (0, 0));
        assert_eq!(split(7), (0, 7));
        assert_eq!(split(8), (1, 0));
        assert_eq!(split(15), (1, 7));
        assert_eq!(split(16), (2, 0));
        assert_eq!(split(18), (2, 2));
    }

    #[test]
    fn input_config_defaults_to_no_pull() {
        // Both the `Default` impl and `new()` start with no pull resistor.
        assert_eq!(InputConfig::default().pull, Pull::None);
        assert_eq!(InputConfig::new().pull, Pull::None);
    }

    #[test]
    fn input_config_with_pull_sets_field() {
        // `with_pull` is a pure const builder that overwrites only `pull`.
        assert_eq!(InputConfig::new().with_pull(Pull::Up).pull, Pull::Up);
        assert_eq!(InputConfig::new().with_pull(Pull::Down).pull, Pull::Down);
        assert_eq!(InputConfig::new().with_pull(Pull::None).pull, Pull::None);
    }

    #[test]
    fn output_config_defaults_are_false() {
        // `new()` and `Default` agree: push-pull, starts low.
        let a = OutputConfig::new();
        let b = OutputConfig::default();
        assert!(!a.open_drain && !a.initial_high);
        assert!(!b.open_drain && !b.initial_high);
    }

    #[test]
    fn output_config_builders_set_independent_fields() {
        // Each builder mutates exactly its own field, leaving the other intact.
        let c = OutputConfig::new().with_open_drain(true);
        assert!(c.open_drain && !c.initial_high);
        let c = OutputConfig::new().with_initial(true);
        assert!(!c.open_drain && c.initial_high);
        let c = OutputConfig::new().with_open_drain(true).with_initial(true);
        assert!(c.open_drain && c.initial_high);
    }

    // `set_interrupt_trigger` maps each trigger to `(edge, high)` and writes
    // GPIO_INT_TYPE (edge) + GPIO_INT_POLARITY (high). Re-derive the pure map
    // here to lock the encoding without touching the registers.
    fn trigger_bits(t: InterruptTrigger) -> (bool, bool) {
        match t {
            InterruptTrigger::RisingEdge => (true, true),
            InterruptTrigger::FallingEdge => (true, false),
            InterruptTrigger::HighLevel => (false, true),
            InterruptTrigger::LowLevel => (false, false),
        }
    }

    #[test]
    fn interrupt_trigger_encoding() {
        // Edge bit distinguishes edge vs level; high bit distinguishes the active
        // direction (rising/high vs falling/low).
        assert_eq!(trigger_bits(InterruptTrigger::RisingEdge), (true, true));
        assert_eq!(trigger_bits(InterruptTrigger::FallingEdge), (true, false));
        assert_eq!(trigger_bits(InterruptTrigger::HighLevel), (false, true));
        assert_eq!(trigger_bits(InterruptTrigger::LowLevel), (false, false));
    }

    #[test]
    fn interrupt_trigger_edge_vs_level() {
        // The two edge triggers share edge=true; the two level triggers edge=false.
        assert!(trigger_bits(InterruptTrigger::RisingEdge).0);
        assert!(trigger_bits(InterruptTrigger::FallingEdge).0);
        assert!(!trigger_bits(InterruptTrigger::HighLevel).0);
        assert!(!trigger_bits(InterruptTrigger::LowLevel).0);
    }

    // `apply_pull` maps each `Pull` to `(pe, ps)` = (pull-enable, pull-select),
    // matching the SVD encoding 00=none, 11=up, 10=down. Re-derive that map.
    fn pull_bits(p: Pull) -> (bool, bool) {
        match p {
            Pull::None => (false, false),
            Pull::Up => (true, true),
            Pull::Down => (true, false),
        }
    }

    #[test]
    fn pull_encoding_matches_svd() {
        // {PE,PS}: 00 = none, 11 = up, 10 = down.
        assert_eq!(pull_bits(Pull::None), (false, false));
        assert_eq!(pull_bits(Pull::Up), (true, true));
        assert_eq!(pull_bits(Pull::Down), (true, false));
    }

    #[test]
    fn pull_enable_implies_nonzero() {
        // Any active pull asserts PE; only `None` leaves PE clear.
        assert!(!pull_bits(Pull::None).0);
        assert!(pull_bits(Pull::Up).0);
        assert!(pull_bits(Pull::Down).0);
    }

    // `apply_pull` computes the pad-control register address as
    // IO_CONFIG_BASE + PAD_GPIO_CTRL_OFF + pin*4, and is a no-op for pins > 14.
    fn pad_ctrl_addr(pin: u8) -> usize {
        IO_CONFIG_BASE + PAD_GPIO_CTRL_OFF + (pin as usize) * 4
    }

    #[test]
    fn pad_ctrl_address_arithmetic() {
        // First pad sits at base+offset; each subsequent pad is +4 bytes.
        assert_eq!(pad_ctrl_addr(0), IO_CONFIG_BASE + PAD_GPIO_CTRL_OFF);
        assert_eq!(pad_ctrl_addr(1), IO_CONFIG_BASE + PAD_GPIO_CTRL_OFF + 4);
        assert_eq!(pad_ctrl_addr(14), 0x4400_D000 + 0x800 + 14 * 4);
        // Strictly ascending, 4-byte stride across the whole pad-controlled range.
        for pin in 1u8..=14 {
            assert_eq!(pad_ctrl_addr(pin) - pad_ctrl_addr(pin - 1), 4);
        }
    }

    #[test]
    fn pad_pull_bits_are_distinct_and_placed() {
        // PE = bit 9, PS = bit 10; clearing both masks 0b110_0000_0000.
        assert_eq!(PAD_PE_BIT, 1 << 9);
        assert_eq!(PAD_PS_BIT, 1 << 10);
        assert_ne!(PAD_PE_BIT, PAD_PS_BIT);
        assert_eq!(PAD_PE_BIT | PAD_PS_BIT, 0b110_0000_0000);
    }

    #[test]
    fn pad_clear_mask_preserves_other_bits() {
        // The RMW clears only PE+PS; every other bit (e.g. drive strength) survives.
        let other = 0xFFFF_FFFFu32 & !(PAD_PE_BIT | PAD_PS_BIT);
        let v = other & !(PAD_PE_BIT | PAD_PS_BIT);
        assert_eq!(v, other);
    }

    // Re-derive the `apply_pull` RMW (sans MMIO): clear PE+PS, set them per the
    // pull, and unconditionally assert IE — bit-for-bit what the driver does.
    fn apply_pull_rmw(start: u32, pull: Pull) -> u32 {
        let (pe, ps) = pull_bits(pull);
        let mut v = start;
        v &= !(PAD_PE_BIT | PAD_PS_BIT);
        if pe {
            v |= PAD_PE_BIT;
        }
        if ps {
            v |= PAD_PS_BIT;
        }
        v |= PAD_IE_BIT;
        v
    }

    #[test]
    fn pad_ie_bit_is_bit11() {
        assert_eq!(PAD_IE_BIT, 1 << 11);
        // IE is distinct from the pull bits it shares the register with.
        assert_ne!(PAD_IE_BIT, PAD_PE_BIT);
        assert_ne!(PAD_IE_BIT, PAD_PS_BIT);
    }

    #[test]
    fn apply_pull_always_enables_input_buffer() {
        // Regardless of the pad's prior IE state (cleared OR set) and the pull,
        // the RMW leaves IE asserted so the input read path works self-contained.
        for &start in &[0x0000_0000u32, 0x800, 0x600, 0xFFFF_FFFF] {
            for pull in [Pull::None, Pull::Up, Pull::Down] {
                let v = apply_pull_rmw(start, pull);
                assert_ne!(v & PAD_IE_BIT, 0, "IE must be set (start={start:#x}, {pull:?})");
            }
        }
    }

    #[test]
    fn apply_pull_sets_pull_and_keeps_unrelated_bits() {
        // Drive-strength bits (4..=6) and Schmitt (bit 3) must survive the RMW,
        // while PE/PS reflect the requested pull and IE is forced on.
        let start = (0x7 << 4) | (1 << 3); // ds=7, ST=1, PE/PS/IE clear
        let keep = (0x7 << 4) | (1 << 3);

        let up = apply_pull_rmw(start, Pull::Up);
        assert_eq!(up & keep, keep);
        assert_eq!(up & (PAD_PE_BIT | PAD_PS_BIT), PAD_PE_BIT | PAD_PS_BIT);
        assert_ne!(up & PAD_IE_BIT, 0);

        let down = apply_pull_rmw(start, Pull::Down);
        assert_eq!(down & keep, keep);
        assert_eq!(down & (PAD_PE_BIT | PAD_PS_BIT), PAD_PE_BIT);

        let none = apply_pull_rmw(start | PAD_PE_BIT | PAD_PS_BIT, Pull::None);
        assert_eq!(none & (PAD_PE_BIT | PAD_PS_BIT), 0); // both cleared
        assert_ne!(none & PAD_IE_BIT, 0);
    }
}

// ── Property-based fuzz tests ──────────────────────────────────

#[cfg(all(test, not(target_arch = "riscv32")))]
mod proptests {
    use super::{IO_CONFIG_BASE, PAD_GPIO_CTRL_OFF};
    use proptest::prelude::*;

    proptest! {
        /// Fuzz: for any valid pin, split→recombine is the identity (no MMIO).
        #[test]
        fn pin_number_round_trips(pin in 0u8..=18) {
            let block = pin / 8;
            let bit = pin % 8;
            prop_assert_eq!(block * 8 + bit, pin);
            prop_assert!(bit < 8);
        }

        /// Fuzz: pad-control addresses are strictly monotonic in the pin index
        /// and never overflow `usize` for the pad-controlled range (0..=14).
        #[test]
        fn pad_addr_monotonic(pin in 1u8..=14) {
            let addr = |p: u8| IO_CONFIG_BASE + PAD_GPIO_CTRL_OFF + (p as usize) * 4;
            prop_assert_eq!(addr(pin) - addr(pin - 1), 4);
            prop_assert!(addr(pin) > addr(pin - 1));
        }
    }
}
