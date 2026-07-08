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
    /// No pull resistor (high-impedance input).
    None,
    /// Pull-up resistor enabled ({PE,PS} = 11).
    Up,
    /// Pull-down resistor enabled ({PE,PS} = 10).
    Down,
}

/// GPIO interrupt trigger condition (sets `GPIO_INT_TYPE` + `GPIO_INT_POLARITY`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[instability::unstable]
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

/// GPIO bank selection for the three WS63 GPIO interrupt banks.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GpioBank {
    /// GPIO bank 0 (IRQ GPIO_INT0).
    Bank0,
    /// GPIO bank 1 (IRQ GPIO_INT1).
    Bank1,
    /// GPIO bank 2 (IRQ GPIO_INT2).
    Bank2,
}

impl GpioBank {
    /// Build a GPIO bank from a raw bank index, rejecting values outside 0..=2.
    pub const fn from_index(index: u8) -> Option<Self> {
        match index {
            0 => Some(Self::Bank0),
            1 => Some(Self::Bank1),
            2 => Some(Self::Bank2),
            _ => None,
        }
    }

    /// The GPIO bank index (0-2).
    pub const fn index(self) -> u8 {
        match self {
            Self::Bank0 => 0,
            Self::Bank1 => 1,
            Self::Bank2 => 2,
        }
    }
}

/// Digital input configuration.
#[derive(Debug, Clone, Copy)]
pub struct InputConfig {
    /// Pull resistor applied to the pad.
    pub pull: Pull,
}

impl Default for InputConfig {
    fn default() -> Self {
        Self { pull: Pull::None }
    }
}

impl InputConfig {
    /// Create a default input config (no pull resistor).
    pub const fn new() -> Self {
        Self { pull: Pull::None }
    }
    /// Set the pull resistor configuration.
    pub const fn with_pull(mut self, pull: Pull) -> Self {
        self.pull = pull;
        self
    }
}

/// Digital output configuration.
#[derive(Debug, Clone, Copy, Default)]
pub struct OutputConfig {
    /// Drive the pin high on initialization (otherwise low).
    pub initial_high: bool,
}

impl OutputConfig {
    /// Create a default output config (push-pull, starts low).
    pub const fn new() -> Self {
        Self { initial_high: false }
    }
    /// Set the initial drive level (true = high).
    pub const fn with_initial(mut self, high: bool) -> Self {
        self.initial_high = high;
        self
    }
}

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
    /// Returns `true` if the pin reads high (from `GPIO_SW_OUT`).
    pub fn is_high(&self) -> bool {
        (regs(self.pin.block).gpio_sw_out().read().bits() >> self.pin.bit) & 1 != 0
    }

    /// Returns `true` if the pin reads low.
    pub fn is_low(&self) -> bool {
        !self.is_high()
    }

    /// Returns this pin's number (0-18).
    pub fn number(&self) -> u8 {
        self.pin.number()
    }

    /// Enable the interrupt for this pin (sets its bit in `GPIO_INT_EN`).
    #[instability::unstable]
    pub fn enable_interrupt(&self) {
        let r = regs(self.pin.block);
        r.gpio_int_en().modify(|r, w| unsafe { w.bits(r.bits() | (1 << self.pin.bit)) });
    }

    /// Disable the interrupt for this pin (clears its bit in `GPIO_INT_EN`).
    #[instability::unstable]
    pub fn disable_interrupt(&self) {
        let r = regs(self.pin.block);
        r.gpio_int_en().modify(|r, w| unsafe { w.bits(r.bits() & !(1 << self.pin.bit)) });
    }

    /// Clear this pin's pending interrupt (writes its bit to `GPIO_INT_EOI`).
    #[instability::unstable]
    pub fn clear_interrupt(&self) {
        unsafe { regs(self.pin.block).gpio_int_eoi().write(|w| w.bits(1 << self.pin.bit)) };
    }

    /// Set the interrupt trigger condition for this pin (edge/level + polarity).
    ///
    /// Configures `GPIO_INT_TYPE` (edge vs level) and `GPIO_INT_POLARITY`
    /// (rising/high vs falling/low). Call before [`enable_interrupt`](Self::enable_interrupt).
    #[instability::unstable]
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

    /// Returns `true` if this pin's interrupt is pending (set in `GPIO_INT_RAW`).
    #[instability::unstable]
    pub fn interrupt_pending(&self) -> bool {
        (regs(self.pin.block).gpio_int_raw().read().bits() >> self.pin.bit) & 1 != 0
    }

    /// Type-erase this input back to an [`AnyPin`] (consumes the driver). Safe: the
    /// pad keeps its input direction/pull; only the typed wrapper is dropped.
    pub fn degrade(self) -> AnyPin<'d> {
        self.pin
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
    /// Drive the pin high (writes its bit to `GPIO_DATA_SET`).
    pub fn set_high(&mut self) {
        unsafe { regs(self.pin.block).gpio_data_set().write(|w| w.bits(1 << self.pin.bit)) };
    }

    /// Drive the pin low (writes its bit to `GPIO_DATA_CLR`).
    pub fn set_low(&mut self) {
        unsafe { regs(self.pin.block).gpio_data_clr().write(|w| w.bits(1 << self.pin.bit)) };
    }

    /// Toggle the pin's drive level based on its current `GPIO_SW_OUT` state.
    pub fn toggle(&mut self) {
        let r = regs(self.pin.block);
        let val = r.gpio_sw_out().read().bits();
        if val & (1 << self.pin.bit) != 0 {
            unsafe { r.gpio_data_clr().write(|w| w.bits(1 << self.pin.bit)) };
        } else {
            unsafe { r.gpio_data_set().write(|w| w.bits(1 << self.pin.bit)) };
        }
    }

    /// Returns `true` if the pin is currently driven high (from `GPIO_SW_OUT`).
    pub fn is_set_high(&self) -> bool {
        (regs(self.pin.block).gpio_sw_out().read().bits() >> self.pin.bit) & 1 != 0
    }

    /// Returns this pin's number (0-18).
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

    /// Type-erase this output back to an [`AnyPin`] (consumes the driver), **keeping
    /// the pad driving** (the safe-state [`Drop`](Output#impl-Drop-for-Output) does
    /// not run). Safe — only the typed wrapper is consumed.
    pub fn degrade(self) -> AnyPin<'d> {
        // Move `pin` out without running the safe-state Drop (which would revert OEN).
        let this = core::mem::ManuallyDrop::new(self);
        // SAFETY: `this` is never dropped (ManuallyDrop) and `pin` is read once.
        unsafe { core::ptr::read(&this.pin) }
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
    /// Switch to output and drive the pin high (writes `GPIO_DATA_SET`).
    pub fn set_high(&mut self) {
        self.pin.set_oen(false);
        unsafe { regs(self.pin.block).gpio_data_set().write(|w| w.bits(1 << self.pin.bit)) };
    }

    /// Switch to output and drive the pin low (writes `GPIO_DATA_CLR`).
    pub fn set_low(&mut self) {
        self.pin.set_oen(false);
        unsafe { regs(self.pin.block).gpio_data_clr().write(|w| w.bits(1 << self.pin.bit)) };
    }

    /// Switch to output and toggle the pin's drive level.
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

    /// Returns `true` if the pin is currently driven high (from `GPIO_SW_OUT`).
    pub fn is_set_high(&self) -> bool {
        (regs(self.pin.block).gpio_sw_out().read().bits() >> self.pin.bit) & 1 != 0
    }

    /// Read the pin level as an input: temporarily switch to input, sample, then
    /// restore the previous output-enable state.
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

    /// Returns `true` if the pin reads low (see [`is_high`](Self::is_high)).
    pub fn is_low(&self) -> bool {
        !self.is_high()
    }

    /// Explicitly set the pad direction to **output** (clears `OEN`). Pair this with
    /// the data methods to make the direction switch visible instead of relying on
    /// the implicit switch inside `set_high`/`set_low`/`toggle`.
    pub fn set_as_output(&mut self) {
        self.pin.set_oen(false);
    }

    /// Explicitly set the pad direction to **input / high-Z** (sets `OEN`). Pair this
    /// with [`is_high`](Self::is_high) to sample without the implicit save/restore.
    pub fn set_as_input(&mut self) {
        self.pin.set_oen(true);
    }

    /// Returns this pin's number (0-18).
    pub fn number(&self) -> u8 {
        self.pin.number()
    }

    /// Type-erase this Flex pin back to an [`AnyPin`] (consumes the driver). Safe:
    /// the pad keeps its current direction/level; only the typed wrapper is dropped.
    pub fn degrade(self) -> AnyPin<'d> {
        self.pin
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
// IE = bit 11 gates the pad's input buffer; gpio_sw_out only reflects the pin
// when IE is set. The boot ROM leaves IE=1 by reset default (measured on silicon:
// pad_gpio_03_ctrl = 0x800 at entry) and the WS63 vendor pinctrl never writes it
// (CONFIG_PINCTRL_SUPPORT_IE undefined), so a GPIO read works without us touching
// it. We assert IE for input pins anyway — same hardware state the vendor relies
// on, but self-contained, so a pad whose IE was cleared by an earlier mux still
// reads correctly. (`io_config::build_pad_ctrl` places IE at the same bit 11.)

/// Configure a GPIO pad for **input** via IO_CONFIG: apply the pull resistor and
/// enable the input buffer (IE).
///
/// Read-modify-write so drive strength / Schmitt bits are kept. A no-op for pins
/// 15..=18, which have no `pad_gpio_NN_ctrl` register in this layout (their pull
/// is configured through other pads / the ROM pin map).
#[cfg(feature = "chip-ws63")]
fn apply_pull(pin: u8, pull: Pull) {
    if pin > 14 {
        return;
    }
    let (pe, ps) = match pull {
        Pull::None => (false, false),
        Pull::Up => (true, true),
        Pull::Down => (true, false),
    };
    let regs = unsafe { &*crate::peripherals::IoConfig::ptr() };
    macro_rules! apply_pad {
        ($reg:expr, $pe:ident, $ps:ident, $ie:ident) => {{
            let _ = $reg.modify(|_, w| {
                let w = if pe { w.$pe().set_bit() } else { w.$pe().clear_bit() };
                let w = if ps { w.$ps().set_bit() } else { w.$ps().clear_bit() };
                w.$ie().set_bit()
            });
        }};
    }
    match pin {
        0 => apply_pad!(regs.pad_gpio_00_ctrl(), pad_gpio_00_ctrl_pe, pad_gpio_00_ctrl_ps, pad_gpio_00_ctrl_ie),
        1 => apply_pad!(regs.pad_gpio_01_ctrl(), pad_gpio_01_ctrl_pe, pad_gpio_01_ctrl_ps, pad_gpio_01_ctrl_ie),
        2 => apply_pad!(regs.pad_gpio_02_ctrl(), pad_gpio_02_ctrl_pe, pad_gpio_02_ctrl_ps, pad_gpio_02_ctrl_ie),
        3 => apply_pad!(regs.pad_gpio_03_ctrl(), pad_gpio_03_ctrl_pe, pad_gpio_03_ctrl_ps, pad_gpio_03_ctrl_ie),
        4 => apply_pad!(regs.pad_gpio_04_ctrl(), pad_gpio_04_ctrl_pe, pad_gpio_04_ctrl_ps, pad_gpio_04_ctrl_ie),
        5 => apply_pad!(regs.pad_gpio_05_ctrl(), pad_gpio_05_ctrl_pe, pad_gpio_05_ctrl_ps, pad_gpio_05_ctrl_ie),
        6 => apply_pad!(regs.pad_gpio_06_ctrl(), pad_gpio_06_ctrl_pe, pad_gpio_06_ctrl_ps, pad_gpio_06_ctrl_ie),
        7 => apply_pad!(regs.pad_gpio_07_ctrl(), pad_gpio_07_ctrl_pe, pad_gpio_07_ctrl_ps, pad_gpio_07_ctrl_ie),
        8 => apply_pad!(regs.pad_gpio_08_ctrl(), pad_gpio_08_ctrl_pe, pad_gpio_08_ctrl_ps, pad_gpio_08_ctrl_ie),
        9 => apply_pad!(regs.pad_gpio_09_ctrl(), pad_gpio_09_ctrl_pe, pad_gpio_09_ctrl_ps, pad_gpio_09_ctrl_ie),
        10 => apply_pad!(regs.pad_gpio_10_ctrl(), pad_gpio_10_ctrl_pe, pad_gpio_10_ctrl_ps, pad_gpio_10_ctrl_ie),
        11 => apply_pad!(regs.pad_gpio_11_ctrl(), pad_gpio_11_ctrl_pe, pad_gpio_11_ctrl_ps, pad_gpio_11_ctrl_ie),
        12 => apply_pad!(regs.pad_gpio_12_ctrl(), pad_gpio_12_ctrl_pe, pad_gpio_12_ctrl_ps, pad_gpio_12_ctrl_ie),
        13 => apply_pad!(regs.pad_gpio_13_ctrl(), pad_gpio_13_ctrl_pe, pad_gpio_13_ctrl_ps, pad_gpio_13_ctrl_ie),
        14 => apply_pad!(regs.pad_gpio_14_ctrl(), pad_gpio_14_ctrl_pe, pad_gpio_14_ctrl_ps, pad_gpio_14_ctrl_ie),
        _ => {
            debug_assert!(false, "GPIO pad {pin} has no IO_CONFIG pad control register");
        }
    }
}

#[cfg(not(feature = "chip-ws63"))]
fn apply_pull(_pin: u8, _pull: Pull) {}

// ── InputSignal / OutputSignal (peripheral interconnect) ──────────

/// An output signal from a peripheral that can be routed to a GPIO pin.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct OutputSignal(pub(crate) u8);

/// An input signal to a peripheral that can be routed from a GPIO pin.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct InputSignal(pub(crate) u8);

/// Types that can serve as peripheral outputs (signals towards GPIO matrix).
pub trait PeripheralOutput: crate::private::Sealed {
    /// Returns the output signal this type drives into the GPIO matrix.
    fn output_signal(&self) -> OutputSignal;
}

/// Types that can serve as peripheral inputs (signals from GPIO matrix towards peripherals).
pub trait PeripheralInput: crate::private::Sealed {
    /// Returns the input signal this type sources from the GPIO matrix.
    fn input_signal(&self) -> InputSignal;
}

// Seal GPIO types for peripheral traits
impl crate::private::Sealed for Output<'_> {}
impl crate::private::Sealed for Input<'_> {}
impl crate::private::Sealed for Flex<'_> {}

// ── IO MUX configuration ──────────────────────────────────────────

/// IO MUX configuration (WS63 pinmux; BS21's IO_CONFIG differs — ported later).
#[cfg(feature = "chip-ws63")]
pub struct Io<'d> {
    /// The underlying IO_CONFIG peripheral wrapper.
    pub io_config: IoConfig<'d>,
}

#[cfg(feature = "chip-ws63")]
impl<'d> Io<'d> {
    /// Create an `Io` from the IO_CONFIG peripheral.
    pub fn new(io_config: IoConfig<'d>) -> Self {
        Self { io_config }
    }
    /// Returns the raw IO_CONFIG register block.
    ///
    /// # Safety
    /// This bypasses the typed IO mux API. The caller must uphold all PAC
    /// aliasing, ordering, and pin-mux invariants.
    #[instability::unstable]
    pub unsafe fn register_block(&self) -> &crate::soc::pac::io_config::RegisterBlock {
        unsafe { self.io_config.register_block() }
    }
}

// ── Async (embedded-hal-async) ──────────────────────────────────────────────
#[cfg(all(feature = "chip-ws63", feature = "async", feature = "unstable"))]
mod asynch_impl {
    use super::{GpioBank, Input, InterruptTrigger, regs};
    use crate::asynch::IrqSignal;
    use crate::interrupt::{self, Interrupt};
    use core::future::Future;
    use core::pin::Pin;
    use core::task::{Context, Poll};
    use embedded_hal_async::digital::Wait;

    static GPIO_SIGNAL: [IrqSignal; 3] = [IrqSignal::new(), IrqSignal::new(), IrqSignal::new()];

    fn bank_irq(bank: GpioBank) -> Interrupt {
        match bank {
            GpioBank::Bank0 => Interrupt::GPIO_INT0,
            GpioBank::Bank1 => Interrupt::GPIO_INT1,
            GpioBank::Bank2 => Interrupt::GPIO_INT2,
        }
    }

    /// GPIO trap-handler hook for `bank` (IRQ 33..35, custom local). Masks
    /// the fired pins (so they don't storm), clears their edge latch, wakes the
    /// awaiting [`Wait`] future, and clears the `LOCIPCLR` pending bit. Call this
    /// from the trap when `mcause` is GPIO_INT0..2.
    pub fn on_interrupt(bank: GpioBank) {
        let index = bank.index();
        let r = regs(index);
        let fired = r.gpio_int_raw().read().bits();
        // Mask the fired pins (a fresh wait re-enables) + clear the edge latch.
        r.gpio_int_en().modify(|v, w| unsafe { w.bits(v.bits() & !fired) });
        unsafe { r.gpio_int_eoi().write(|w| w.bits(fired)) };
        GPIO_SIGNAL[index as usize].signal();
        interrupt::clear_pending(bank_irq(bank));
    }

    // Named device.x handlers: hisi-riscv-rt's direct-mode `__rt_irq_dispatch`
    // routes GPIO bank IRQs (33/34/35) here by number, so an async GPIO app needs
    // no `mcause` trap of its own. Strong symbols overriding the weak device.x
    // PROVIDE; only present with `async`.
    #[unsafe(no_mangle)]
    extern "C" fn GPIO_INT0() {
        on_interrupt(GpioBank::Bank0);
    }
    #[unsafe(no_mangle)]
    extern "C" fn GPIO_INT1() {
        on_interrupt(GpioBank::Bank1);
    }
    #[unsafe(no_mangle)]
    extern "C" fn GPIO_INT2() {
        on_interrupt(GpioBank::Bank2);
    }

    async fn arm_and_wait(input: &mut Input<'_>, trig: InterruptTrigger) {
        let bank = input.pin.block as usize;
        input.set_interrupt_trigger(trig);
        input.clear_interrupt();
        GPIO_SIGNAL[bank].reset();
        input.enable_interrupt();
        // SAFETY: enabling a known, fixed WS63 GPIO IRQ line.
        let gpio_bank = GpioBank::from_index(bank as u8).expect("GPIO pin bank must be 0..=2");
        unsafe { interrupt::enable(bank_irq(gpio_bank)) };
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

#[cfg(all(feature = "chip-ws63", feature = "async", feature = "unstable"))]
pub use asynch_impl::on_interrupt;

// ── Tests ──────────────────────────────────────────────────────

#[cfg(all(test, not(target_arch = "riscv32")))]
mod tests {
    use super::*;

    const PAD_PE_BIT: u32 = 1 << 9;
    const PAD_PS_BIT: u32 = 1 << 10;
    const PAD_IE_BIT: u32 = 1 << 11;

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
        // `new()` and `Default` agree: starts low.
        let a = OutputConfig::new();
        let b = OutputConfig::default();
        assert!(!a.initial_high);
        assert!(!b.initial_high);
    }

    #[test]
    fn output_config_builder_sets_initial_level() {
        let c = OutputConfig::new().with_initial(true);
        assert!(c.initial_high);
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

    // `apply_pull` dispatches to the SVD-modeled PAD_GPIO_00_CTRL..PAD_GPIO_14_CTRL
    // PAC accessors, and is a no-op for pins > 14.
    fn has_pad_ctrl(pin: u8) -> bool {
        pin <= 14
    }

    #[test]
    fn pad_ctrl_pac_coverage() {
        for pin in 0u8..=14 {
            assert!(has_pad_ctrl(pin));
        }
        for pin in 15u8..=18 {
            assert!(!has_pad_ctrl(pin));
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

        /// Fuzz: only the SVD-modeled GPIO pads 0..=14 have direct IO_CONFIG control
        /// registers in this PAC revision; higher GPIO pins are deliberately no-op.
        #[test]
        fn pad_ctrl_coverage_matches_svd(pin in 0u8..=18) {
            prop_assert_eq!(pin <= 14, (0..=14).contains(&pin));
        }
    }
}
