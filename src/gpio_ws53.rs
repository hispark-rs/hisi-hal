//! WS53 digital GPIO on pads already configured by an audited boot environment.
//!
//! Bank ownership and pad ownership are separate. Each bank needs an explicit
//! `Unclaimed::assume_application_ownership` handoff; each `AnyPin` needs a pad
//! handoff with mux, input buffer, pull, drive and live clock prerequisites.
//! GPIO1 construction enables its SDK AGPIO gate before any bank access and
//! checks the complete halfword. It preserves the source and other bits and
//! never disables this shared clock on Drop. Upstream clocks, PM coordination
//! and pad setup still require handoff; no control-core negotiation is implied.
//! No debounce timing is assumed: notification setup disables it.
//!
//! One borrowed Input/Output per bank serializes configuration RMW operations.
//! Different banks can operate concurrently. The ordinary ISR snapshots/acknowledges status.
//! The async ISR also masks/disables its source and wakes outside critical sections.
//! Neither handler polls hardware. Reset and SWD pads cannot become AnyPin values.
use crate::gpio_traits::{impl_error_type, impl_input_pin, impl_output_pin};
use crate::interrupt;
use crate::peripherals::{Gpio0, Gpio1, Gpio2};
use crate::soc::ws53::pins::GpioLine;
pub use crate::soc::ws53::pins::{GpioBank, Pad};
use core::marker::PhantomData;

/// A pad cannot be attached to this digital GPIO driver.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PinError {
    /// Reset has pad control but no GPIO line.
    NotGpio,
    /// SWD pads stay reserved for the debugger.
    DebugReserved,
    /// The pad's canonical GPIO line belongs to a different controller.
    WrongBank,
}

/// GPIO1 could not acquire its shared AGPIO clock without changing the profile.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Gpio1ClockError {
    /// The enable bit was still clear after the bounded enable/readback sequence.
    EnableNotLatched,
    /// Source or other shared bits changed; no rollback is attempted.
    ConfigurationChanged {
        /// Original complete halfword with only the enable bit added.
        expected: u16,
        /// Complete readback after the acquisition attempt.
        observed: u16,
    },
}

fn prepare_gpio1_clock(mut read: impl FnMut() -> u16, mut enable: impl FnMut()) -> Result<(), Gpio1ClockError> {
    let before = read();
    if before & 1 == 0 {
        enable();
        #[cfg(target_arch = "riscv32")]
        unsafe {
            core::arch::asm!("fence iorw, iorw", options(nostack))
        };
    }
    let after = read();
    if after & 1 == 0 {
        return Err(Gpio1ClockError::EnableNotLatched);
    }
    let expected = before | 1;
    if after != expected {
        return Err(Gpio1ClockError::ConfigurationChanged { expected, observed: after });
    }
    Ok(())
}

/// Exclusive permission to use an already configured GPIO pad.
/// Unlike `Pad`, this token cannot be copied or cloned.
/// ```compile_fail
/// use hisi_hal::gpio::AnyPin;
/// fn duplicate(pin: AnyPin<'_>) { let other = pin.clone(); }
/// ```
pub struct AnyPin<'d> {
    pad: Pad,
    line: GpioLine,
    _ownership: PhantomData<&'d mut ()>,
}
impl<'d> AnyPin<'d> {
    /// Attach an externally prepared GPIO pad without changing its mux or pulls.
    ///
    /// # Safety
    /// The caller must hold exclusive ownership of the physical pad and all its
    /// aliases for `'d`, with no vendor/control-core/peripheral users. Its mux
    /// must select GPIO software control; its input buffer, pulls and drive must
    /// be configured appropriately and remain so. The pad and GPIO clocks must
    /// remain enabled. External wiring must permit the requested input/output
    /// operation. No other AnyPin or raw accessor may use this pad concurrently.
    pub unsafe fn from_configured(pad: Pad) -> Result<Self, PinError> {
        let line = permitted_line(pad)?;
        Ok(Self { pad, line, _ownership: PhantomData })
    }
    /// Canonical pad identity, not a new ownership token.
    pub fn pad(&self) -> Pad {
        self.pad
    }
}

fn permitted_line(pad: Pad) -> Result<GpioLine, PinError> {
    if pad.is_debug() {
        return Err(PinError::DebugReserved);
    }
    pad.gpio_line().ok_or(PinError::NotGpio)
}

// PAC fields are u32/u16/u8 for GPIO0/1/2 respectively.
fn bitmap(value: impl Into<u32>) -> u32 {
    value.into()
}

/// GPIO interrupt trigger. Debounce is deliberately disabled by this backend.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum InterruptTrigger {
    /// Low to high transition.
    RisingEdge,
    /// High to low transition.
    FallingEdge,
    /// Either transition.
    BothEdges,
    /// Assert while high; EOI cannot clear an active level.
    HighLevel,
    /// Assert while low; EOI cannot clear an active level.
    LowLevel,
}
impl InterruptTrigger {
    fn bits(self) -> (bool, bool, bool) {
        match self {
            Self::RisingEdge => (true, true, false),
            Self::FallingEdge => (true, false, false),
            Self::BothEdges => (true, false, true),
            Self::HighLevel => (false, true, false),
            Self::LowLevel => (false, false, false),
        }
    }
}

// Each PAC bank has its own field width and type. No layout casts between banks.
macro_rules! bank {
    ($bank:expr, $r:ident, $body:expr) => {
        match $bank {
            GpioBank::Bank0 => {
                let $r = unsafe { &*Gpio0::ptr() };
                $body
            }
            GpioBank::Bank1 => {
                let $r = unsafe { &*Gpio1::ptr() };
                $body
            }
            GpioBank::Bank2 => {
                let $r = unsafe { &*Gpio2::ptr() };
                $body
            }
        }
    };
}
macro_rules! bit_update {
    ($bank:expr, $mask:expr, $register:ident, $set:expr) => {
        bit_update!($bank, $mask, $register, $register, $set)
    };
    ($bank:expr, $mask:expr, $register:ident, $field:ident, $set:expr) => {{
        let mask = $mask;
        let set = $set;
        bank!(
            $bank,
            r,
            r.$register().modify(|old, w| unsafe {
                // The mask is a validated GpioLine in this bank. Preserve every
                // other modeled bit; the PAC also preserves unmodeled bits.
                let old = bitmap(old.$field().bits());
                w.$field().bits((if set { old | mask } else { old & !mask }) as _)
            })
        );
    }};
}
macro_rules! command {
    ($bank:expr, $register:ident, $mask:expr) => {
        bank!($bank, r, unsafe { r.$register().write_with_zero(|w| w.$register().bits($mask as _)) })
    };
}

#[cfg(feature = "async")]
#[path = "gpio_ws53/asynch.rs"]
mod asynch;
#[cfg(feature = "async")]
pub use asynch::{AsyncBindError, AsyncInput};

// Own the exact bank singleton for the driver's lifetime. The enum is private.
enum BankToken<'d> {
    Zero(Gpio0<'d>),
    One(Gpio1<'d>),
    Two(Gpio2<'d>),
}

/// Exclusive GPIO bank access after clock and resource handoff.
/// The bank cannot be borrowed for two live pin drivers:
/// ```compile_fail
/// use hisi_hal::gpio::{GpioDriver, AnyPin};
/// fn alias<'d>(bank: &mut GpioDriver<'d>, a: AnyPin<'d>, b: AnyPin<'d>) {
///     let input = bank.init_input(a);
///     let output = bank.init_output(b, false);
///     drop(input);
///     drop(output);
/// }
/// ```
pub struct GpioDriver<'d> {
    token: BankToken<'d>,
}
impl<'d> GpioDriver<'d> {
    /// Attach GPIO0. Performs no reset or pad configuration.
    pub fn new_gpio0(token: Gpio0<'d>) -> Self {
        Self { token: BankToken::Zero(token) }
    }
    /// Enable the AGPIO clock and attach GPIO1 without touching bank/pad state.
    ///
    /// SDK `dev_work_clock_init` sets SystemMemory GPIO_CLOCK.enable. The
    /// constructor uses only the halfword view and preserves the full source
    /// profile. It performs no GPIO1 access until the gate is confirmed. A
    /// failure returns the bank token; a changed shared clock is not rolled back.
    ///
    /// # Hardware requirements
    /// The unsafe ownership handoff must include permission to enable this
    /// shared gate, a live upstream source, and quiescent peer/PM clock writers.
    /// Local interrupts are excluded here; this does not synchronize another core.
    /// No Drop path disables the clock or claims low-power support.
    pub fn new_gpio1(token: Gpio1<'d>) -> Result<Self, (Gpio1ClockError, Gpio1<'d>)> {
        let clocks = unsafe { &*ws53_pac::SystemMemory::PTR };
        let result = critical_section::with(|_| {
            prepare_gpio1_clock(
                || clocks.gpio_clock().read().bits(),
                || {
                    clocks.gpio_clock().modify(|_, w| w.enable().set_bit());
                },
            )
        });
        match result {
            Ok(()) => Ok(Self { token: BankToken::One(token) }),
            Err(error) => Err((error, token)),
        }
    }
    /// Attach GPIO2; SWD pads remain unavailable to this driver.
    pub fn new_gpio2(token: Gpio2<'d>) -> Self {
        Self { token: BankToken::Two(token) }
    }
    /// Controller represented by this singleton.
    pub fn bank(&self) -> GpioBank {
        match &self.token {
            BankToken::Zero(_) => GpioBank::Bank0,
            BankToken::One(_) => GpioBank::Bank1,
            BankToken::Two(_) => GpioBank::Bank2,
        }
    }
    fn check(&self, pin: &AnyPin<'_>) -> Result<(), PinError> {
        if self.bank() == pin.line.bank() { Ok(()) } else { Err(PinError::WrongBank) }
    }
    /// Borrow this bank for digital input. On failure the pad token is returned.
    /// Disables this bank's local IRQ before changing any interrupt configuration.
    pub fn init_input<'a, 'p>(&'a mut self, pin: AnyPin<'p>) -> Result<Input<'a, 'd, 'p>, (PinError, AnyPin<'p>)> {
        if let Err(error) = self.check(&pin) {
            return Err((error, pin));
        }
        cancel(pin.line);
        bit_update!(self.bank(), pin.line.mask(), gpio_sw_oen, pins, true);
        let line = pin.line;
        Ok(Input { bank: self, pin: Some(pin), line })
    }
    /// Borrow this bank for output, preload its latch, then enable its driver.
    /// This requires a prepared pad whose external wiring permits output.
    /// On failure the unchanged pad token is returned.
    pub fn init_output<'a, 'p>(
        &'a mut self,
        pin: AnyPin<'p>,
        initial_high: bool,
    ) -> Result<Output<'a, 'd, 'p>, (PinError, AnyPin<'p>)> {
        if let Err(error) = self.check(&pin) {
            return Err((error, pin));
        }
        cancel(pin.line);
        let line = pin.line;
        // Disable output while initializing its latch; never expose stale data.
        bit_update!(self.bank(), line.mask(), gpio_sw_oen, pins, true);
        write_level(line, initial_high);
        bit_update!(self.bank(), line.mask(), gpio_sw_oen, pins, false);
        Ok(Output { bank: self, pin: Some(pin), line })
    }
    /// Capture one notification, acknowledge captured non-debug GPIO bits and
    /// leave the bank's local IRQ disabled. Level sources can remain asserted.
    /// Call `interrupt::enable(bank.interrupt())` explicitly after servicing it.
    ///
    /// # Safety
    /// A live application-owned GpioDriver must own this bank with no other
    /// CPU/DMA users. Call only in its ISR or with that local IRQ disabled.
    /// No other ISR may concurrently service/reconfigure the bank. This function
    /// never borrows the main-context driver or modifies its configuration words.
    pub unsafe fn on_interrupt(bank: GpioBank) -> InterruptStatus {
        unsafe {
            interrupt::disable(bank.interrupt());
        }
        let pending = bank!(bank, r, bitmap(r.gpio_intr().read().gpio_intr().bits()));
        let bits = pending & usable_mask(bank);
        command!(bank, gpio_int_eoi, bits);
        interrupt::clear_pending(bank.interrupt());
        InterruptStatus { bank, bits }
    }
}

fn usable_mask(bank: GpioBank) -> u32 {
    Pad::ALL
        .iter()
        .filter_map(|&pad| permitted_line(pad).ok())
        .filter(|line| line.bank() == bank)
        .fold(0, |mask, line| mask | line.mask())
}

/// Captured interrupt bits, tagged with their controller to prevent alias confusion.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct InterruptStatus {
    bank: GpioBank,
    bits: u32,
}
impl InterruptStatus {
    /// Whether the captured notification contains this canonical pad.
    pub fn contains(self, pad: Pad) -> bool {
        pad.gpio_line().is_some_and(|line| line.bank() == self.bank && self.bits & line.mask() != 0)
    }
    /// True for a spurious or unsupported-source notification.
    pub fn is_empty(self) -> bool {
        self.bits == 0
    }
}

fn cancel(line: GpioLine) {
    unsafe {
        interrupt::disable(line.bank().interrupt());
    }
    bit_update!(line.bank(), line.mask(), gpio_int_mask, true);
    bit_update!(line.bank(), line.mask(), gpio_int_en, false);
    command!(line.bank(), gpio_int_eoi, line.mask());
    interrupt::clear_pending(line.bank().interrupt());
}
fn read_level(line: GpioLine) -> bool {
    bank!(line.bank(), r, bitmap(r.gpio_sw_out().read().gpio_sw_out().bits()) & line.mask() != 0)
}
fn write_level(line: GpioLine, high: bool) {
    if high {
        command!(line.bank(), gpio_data_set, line.mask());
    } else {
        command!(line.bank(), gpio_data_clr, line.mask());
    }
}

/// Input driver holding the bank's exclusive configuration borrow.
pub struct Input<'a, 'd, 'p> {
    bank: &'a mut GpioDriver<'d>,
    pin: Option<AnyPin<'p>>,
    line: GpioLine,
}
impl<'p> Input<'_, '_, 'p> {
    /// Sample the digital input. Pad input buffer and mux must already be valid.
    pub fn is_high(&self) -> bool {
        read_level(self.line)
    }
    /// Configure a notification with debounce off. Discards this pin's previous
    /// edge, preserves other pin configuration, and leaves the local IRQ off.
    /// Install a handler and explicitly enable the bank's local IRQ afterward.
    pub fn listen(&mut self, trigger: InterruptTrigger) {
        cancel(self.line);
        let bank = self.bank.bank();
        let mask = self.line.mask();
        let (edge, high, both) = trigger.bits();
        bit_update!(bank, mask, gpio_int_debounce, false);
        bit_update!(bank, mask, gpio_int_type, pins, edge);
        bit_update!(bank, mask, gpio_int_polarity, pins, high);
        bit_update!(bank, mask, gpio_int_dedge, pins, both);
        command!(bank, gpio_int_eoi, mask);
        bit_update!(bank, mask, gpio_int_en, true);
        bit_update!(bank, mask, gpio_int_mask, false);
    }
    /// Cancel peripheral and local-controller notification, including pending edges.
    pub fn unlisten(&mut self) {
        cancel(self.line);
    }
    /// Release the pad token after cancelling notifications. The pad remains input.
    pub fn free(mut self) -> AnyPin<'p> {
        self.unlisten();
        self.pin.take().unwrap()
    }
}
impl Drop for Input<'_, '_, '_> {
    fn drop(&mut self) {
        if self.pin.is_some() {
            cancel(self.line);
        }
    }
}

/// Digital output. Drop disables its GPIO output driver, preserving pad configuration.
pub struct Output<'a, 'd, 'p> {
    bank: &'a mut GpioDriver<'d>,
    pin: Option<AnyPin<'p>>,
    line: GpioLine,
}
impl<'p> Output<'_, '_, 'p> {
    /// Set only this output latch using the SET command; never read-modify-write it.
    pub fn set_high(&mut self) {
        write_level(self.line, true);
    }
    /// Clear only this output latch using the CLR command.
    pub fn set_low(&mut self) {
        write_level(self.line, false);
    }
    /// Read the output latch through SW_OUT, as used by the SDK.
    pub fn is_set_high(&self) -> bool {
        read_level(self.line)
    }
    /// Disable the output driver and return its prepared pad token.
    pub fn free(mut self) -> AnyPin<'p> {
        bit_update!(self.bank.bank(), self.line.mask(), gpio_sw_oen, pins, true);
        self.pin.take().unwrap()
    }
}
impl Drop for Output<'_, '_, '_> {
    fn drop(&mut self) {
        if self.pin.is_some() {
            bit_update!(self.bank.bank(), self.line.mask(), gpio_sw_oen, pins, true);
        }
    }
}

impl_error_type!(Input<'_, '_, '_>);
impl_input_pin!(Input<'_, '_, '_>, Input);
impl_error_type!(Output<'_, '_, '_>);
impl_output_pin!(Output<'_, '_, '_>, Output);

#[cfg(all(test, not(target_arch = "riscv32")))]
mod tests {
    use super::*;
    #[test]
    fn gpio1_clock_acquisition_preserves_profile_and_avoids_redundant_writes() {
        use core::cell::Cell;
        for initial in 0..=u16::MAX {
            let register = Cell::new(initial);
            let reads = Cell::new(0);
            let writes = Cell::new(0);
            assert_eq!(
                prepare_gpio1_clock(
                    || {
                        reads.set(reads.get() + 1);
                        register.get()
                    },
                    || {
                        writes.set(writes.get() + 1);
                        register.set(register.get() | 1)
                    },
                ),
                Ok(())
            );
            assert_eq!(register.get(), initial | 1);
            assert_eq!(reads.get(), 2);
            assert_eq!(writes.get(), u32::from(initial & 1 == 0));
        }
    }
    #[test]
    fn gpio1_clock_rejects_failed_enable_and_a_changed_shared_source() {
        use core::cell::Cell;
        assert_eq!(prepare_gpio1_clock(|| 0, || {}), Err(Gpio1ClockError::EnableNotLatched));
        let register = Cell::new(0);
        assert_eq!(
            prepare_gpio1_clock(|| register.get(), || register.set(3)),
            Err(Gpio1ClockError::ConfigurationChanged { expected: 1, observed: 3 })
        );
        assert_eq!(register.get(), 3); // Do not roll back an unknown shared writer.
    }
    #[test]
    fn reset_and_debug_cannot_be_claimed_as_digital_gpio() {
        assert_eq!(permitted_line(Pad::Reset), Err(PinError::NotGpio));
        assert_eq!(permitted_line(Pad::Sgpio0), Err(PinError::DebugReserved));
        assert_eq!(permitted_line(Pad::Sgpio1), Err(PinError::DebugReserved));
        assert_eq!(usable_mask(GpioBank::Bank2), 4);
        assert_eq!(usable_mask(GpioBank::Bank1), 0xfff);
    }
    #[test]
    fn status_is_bank_tagged_and_aliases_share_one_bit() {
        let status = InterruptStatus { bank: GpioBank::Bank1, bits: 1 << 6 };
        assert!(status.contains(Pad::Agpio6));
        assert!(!status.contains(Pad::Mgpio2));
        assert!(!status.contains(Pad::Reset));
        assert!(!status.is_empty());
    }
    #[test]
    fn wrong_bank_returns_the_pin_without_touching_mmio() {
        // Host-only: the constructor and rejected paths do not dereference PAC.
        let mut bank = GpioDriver::new_gpio0(unsafe { Gpio0::steal() });
        let pin = unsafe { AnyPin::from_configured(Pad::Agpio3) }.unwrap();
        let (error, pin) = match bank.init_input(pin) {
            Err(error) => error,
            Ok(_) => panic!("wrong bank accepted"),
        };
        assert_eq!(error, PinError::WrongBank);
        assert_eq!(pin.pad(), Pad::Agpio3);
        let (error, pin) = match bank.init_output(pin, true) {
            Err(error) => error,
            Ok(_) => panic!("wrong bank accepted"),
        };
        assert_eq!(error, PinError::WrongBank);
        assert_eq!(pin.pad(), Pad::Agpio3);
    }
}
