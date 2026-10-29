//! WS53 RTC v150 application channels, source SDK 163d9150 / RDL 0ddce57.
//!
//! # Hardware requirements
//! Handoff must establish a running input clock, accessible bus, released
//! reset/FSM and exclusive channel/IRQ ownership. This driver does not provision
//! a crystal, clock/reset registers, common-bank locks, ROM or sleep services.
//! Only the selected 32-bit profile is supported. The owned DelayNs provider
//! must work independently of this RTC and retain its clock through Drop.
//! Start/stop use the source SDK's 65/32 us synchronization waits. A-board RAM
//! tests qualify active-high masking and IRQ49/50, but not low-power behavior.
//! The bounded A-board configuration measures approximately32kHz, not the
//! SDK32768Hz duration-conversion profile: use direct ticks for that board.
//! EOI completion is asynchronous. Suspend local delivery before acknowledging,
//! then check raw deassertion in bounded main-context work before clearing local
//! pending and restoring delivery. An immediate ISR return can retrigger.
use crate::interrupt::{self, Interrupt};
use crate::peripherals::{Rtc0, Rtc1};
use core::num::{NonZeroU16, NonZeroU32};
use core::time::Duration;
use embedded_hal::delay::DelayNs;

/// Required input-clock profile; not a measured board frequency.
pub const RTC_CLOCK_HZ: u32 = 32_768;

/// A channel identity does not grant hardware ownership.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RtcChannel {
    /// Application RTC0, absolute local cause 49.
    Channel0,
    /// Application RTC1, absolute local cause 50.
    Channel1,
}
impl RtcChannel {
    /// Associated application interrupt, not a PLIC source number.
    pub const fn interrupt(self) -> Interrupt {
        match self {
            Self::Channel0 => Interrupt::RTC_0,
            Self::Channel1 => Interrupt::RTC_1,
        }
    }
}

/// Owned channel token; each variant requires its own explicit unsafe handoff.
pub enum RtcPeripheral<'d> {
    /// RTC0 token.
    Rtc0(Rtc0<'d>),
    /// RTC1 token.
    Rtc1(Rtc1<'d>),
}
impl<'d> From<Rtc0<'d>> for RtcPeripheral<'d> {
    fn from(token: Rtc0<'d>) -> Self {
        Self::Rtc0(token)
    }
}
impl<'d> From<Rtc1<'d>> for RtcPeripheral<'d> {
    fn from(token: Rtc1<'d>) -> Self {
        Self::Rtc1(token)
    }
}

/// SDK mode encodings. Alias encoding 2 is represented by OneShot (0).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RtcMode {
    /// Count down once, then stop.
    OneShot,
    /// Reload and repeat after expiry.
    Periodic,
    /// Free-running down-counter mode (SDK encoding 3).
    FreeRunning,
}

/// Nonzero 32-bit counter load; no silent high-word truncation.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct RtcTicks(NonZeroU32);
impl RtcTicks {
    /// Validate a direct counter load. Zero has no supported alarm contract.
    pub const fn from_count(count: u32) -> Option<Self> {
        match NonZeroU32::new(count) {
            Some(count) => Some(Self(count)),
            None => None,
        }
    }
    /// Convert at the required clock profile, rounding upward to whole ticks.
    /// Rejects zero or values outside u32; never floors a requested interval.
    /// Actual expiry edges and synchronization overhead still need silicon tests.
    pub const fn from_duration(duration: Duration) -> Option<Self> {
        let scaled = duration.as_nanos() * RTC_CLOCK_HZ as u128;
        let ticks = scaled.div_ceil(1_000_000_000);
        if ticks > u32::MAX as u128 { None } else { Self::from_count(ticks as u32) }
    }
    /// Value programmed into LOAD_COUNT0; LOAD_COUNT1 is untouched.
    pub const fn count(self) -> u32 {
        self.0.get()
    }
}

/// Snapshot failures are distinct from a successfully captured zero count.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RtcError {
    /// Enable was clear before requesting a snapshot; no request was issued.
    Disabled,
    /// CNT_LOCK did not assert within the bounded number of status reads.
    SnapshotTimeout,
    /// A one-shot expiry was pending after the request. Its counter is stopped,
    /// but the original peripheral event is retained for the registered ISR.
    Expired,
}

/// Owns one application RTC and an independent synchronization-delay provider.
/// Drop stops/masks/acknowledges this channel with the required stop delay.
/// A reserved channel cannot bypass the explicit ownership handoff:
///
/// ```compile_fail
/// use core::num::NonZeroU16;
/// use hisi_hal::{peripherals::{Rtc0, Unclaimed}, rtc::RtcDriver};
/// fn without_handoff(token: Unclaimed<Rtc0<'static>>, delay: impl embedded_hal::delay::DelayNs) {
///     let _rtc = RtcDriver::new(token, delay, NonZeroU16::new(32).unwrap());
/// }
/// ```
pub struct RtcDriver<'d, D: DelayNs> {
    _token: RtcPeripheral<'d>,
    io: PacRegisters,
    delay: D,
    polls: NonZeroU16,
}
impl<'d, D: DelayNs> RtcDriver<'d, D> {
    /// Attach without MMIO. Dropping the owner does quiesce its channel, even
    /// if start was never called. `polls` counts reads, not elapsed time.
    pub fn new(token: impl Into<RtcPeripheral<'d>>, delay: D, polls: NonZeroU16) -> Self {
        let token = token.into();
        let channel = match &token {
            RtcPeripheral::Rtc0(_) => RtcChannel::Channel0,
            RtcPeripheral::Rtc1(_) => RtcChannel::Channel1,
        };
        Self { _token: token, io: PacRegisters { channel }, delay, polls }
    }
    /// Channel identity for handler installation; grants no additional token.
    pub const fn channel(&self) -> RtcChannel {
        self.io.channel
    }
    /// Cancel any old alarm, configure the 32-bit load and start with IRQ masked.
    /// Waits 32 us after stopping and 65 us after enabling, as in the SDK profile.
    pub fn start(&mut self, mode: RtcMode, ticks: RtcTicks) {
        start(&mut self.io, &mut self.delay, mode, ticks);
    }
    /// Stop/mask this channel and acknowledge peripheral and local pending state.
    /// The independent delay provider and both clocks must still be available.
    pub fn stop(&mut self) {
        stop(&mut self.io, &mut self.delay);
    }
    /// Capture the remaining low 32 bits; never read stale data after timeout.
    /// Temporarily suspends only this channel's local IRQ, preserving its
    /// priority and pending event. Other IRQs retain their prior delivery state.
    pub fn read_count(&mut self) -> Result<u32, RtcError> {
        // The handler must not consume RAW_INTR between the request RMW and
        // expiry check. A stale ENABLE=1 write can otherwise rearm a one-shot
        // which expired between reading and writing CONTROL.
        // Ownership handoff excludes competing controller changes. Poll with
        // only this source suspended: a stopped snapshot must not block other
        // interrupts for the entire polling budget.
        let (result, stopped) = unsafe {
            interrupt::with_source_suspended(self.channel().interrupt(), || snapshot(&mut self.io, self.polls))
        };
        if stopped {
            // Interrupts are restored before a possibly interrupt-driven delay.
            self.delay.delay_us(32);
        }
        result
    }
    /// Raw pending state, including when the peripheral interrupt is masked.
    pub fn interrupt_pending(&self) -> bool {
        self.io.pending()
    }
    /// Unmask the peripheral without discarding an already pending event.
    /// The caller installs a handler, then explicitly enables the local IRQ
    /// using `interrupt::enable(channel().interrupt())`. No global MIE change.
    pub fn unmask_interrupt(&mut self) {
        self.io.mask(false);
    }
    /// Mask the peripheral and disable/clear its local IRQ, retaining raw status.
    pub fn mask_interrupt(&mut self) {
        self.io.mask(true);
        self.io.disable_local();
    }
}
/// Acknowledge from a registered handler without borrowing the interrupted
/// main-context driver. No delays or snapshot polling.
/// This posts EOI; it does **not** wait for hardware acknowledgement completion.
/// On A-board, RAW_INTR remains1 immediately after the write. Disable this
/// local IRQ before calling, then wait for `interrupt_pending()` to become false
/// in bounded main-context work. Clear local pending and re-enable only after
/// deassertion; do not count repeated ISR entries as additional expirations.
///
/// # Safety
/// The channel must belong to a live RtcDriver with a running bus clock.
/// Call only from its handler or while that IRQ is disabled; do not race
/// stop/reconfiguration or another core/NMI owner. Never borrow the driver's
/// mutable main-context reference across an interrupt.
pub unsafe fn acknowledge_interrupt(channel: RtcChannel) {
    let mut io = PacRegisters { channel };
    io.acknowledge();
    io.clear_local();
}
impl<D: DelayNs> Drop for RtcDriver<'_, D> {
    fn drop(&mut self) {
        self.stop();
    }
}

trait Registers {
    fn enable(&mut self, enabled: bool);
    fn enabled(&mut self) -> bool;
    fn mask(&mut self, masked: bool);
    fn configure(&mut self, mode: RtcMode, ticks: RtcTicks);
    fn request(&mut self);
    fn valid(&mut self) -> bool;
    fn count(&mut self) -> u32;
    fn acknowledge(&mut self);
    fn disable_local(&mut self);
    fn clear_local(&mut self);
    fn pending(&self) -> bool;
    fn one_shot(&self) -> bool;
}
fn stop(io: &mut impl Registers, delay: &mut impl DelayNs) {
    io.mask(true);
    io.disable_local();
    io.enable(false);
    delay.delay_us(32);
    io.acknowledge();
    io.clear_local();
}
fn start(io: &mut impl Registers, delay: &mut impl DelayNs, mode: RtcMode, ticks: RtcTicks) {
    stop(io, delay);
    io.configure(mode, ticks);
    io.enable(true);
    delay.delay_us(65);
}
fn snapshot(io: &mut impl Registers, polls: NonZeroU16) -> (Result<u32, RtcError>, bool) {
    if !io.enabled() {
        return (Err(RtcError::Disabled), false);
    }
    io.request();
    let mut sampled = Err(RtcError::SnapshotTimeout);
    for _ in 0..polls.get() {
        if io.valid() {
            sampled = Ok(io.count());
            break;
        }
        core::hint::spin_loop();
    }
    if io.one_shot() && io.pending() {
        // Preserve RAW_INTR/local pending and mask state: the ISR still owns
        // the legitimate expiry notification. Only undo a possible rearm.
        io.enable(false);
        (Err(RtcError::Expired), true)
    } else {
        (sampled, false)
    }
}

struct PacRegisters {
    channel: RtcChannel,
}
// Expand identical field access against each generated register-block type.
// No numeric address casts, type punning or handwritten MMIO structs.
macro_rules! regs {
    ($this:expr, $r:ident, $body:expr) => {
        match $this.channel {
            RtcChannel::Channel0 => {
                let $r = unsafe { &*Rtc0::ptr() };
                $body
            }
            RtcChannel::Channel1 => {
                let $r = unsafe { &*Rtc1::ptr() };
                $body
            }
        }
    };
}
impl Registers for PacRegisters {
    fn enable(&mut self, enabled: bool) {
        regs!(self, r, r.control().modify(|_, w| w.cnt_req().clear_bit().enable().bit(enabled)));
    }
    fn enabled(&mut self) -> bool {
        regs!(self, r, r.control().read().enable().bit_is_set())
    }
    fn mask(&mut self, masked: bool) {
        regs!(self, r, r.control().modify(|_, w| w.cnt_req().clear_bit().int_mask().bit(masked)));
    }
    fn configure(&mut self, mode: RtcMode, ticks: RtcTicks) {
        regs!(self, r, {
            r.control().modify(|_, w| {
                let w = w.cnt_req().clear_bit();
                match mode {
                    RtcMode::OneShot => w.mode().one_shot(),
                    RtcMode::Periodic => w.mode().periodic(),
                    RtcMode::FreeRunning => w.mode().free_run(),
                }
            });
            // Complete 32-bit load, no inferred reset value or high-word access.
            unsafe { r.load_count0().write_with_zero(|w| w.load_count0().bits(ticks.count())) };
        });
    }
    fn request(&mut self) {
        regs!(self, r, r.control().modify(|_, w| w.cnt_req().set_bit()));
    }
    fn valid(&mut self) -> bool {
        regs!(self, r, r.control().read().cnt_lock().bit_is_set())
    }
    fn count(&mut self) -> u32 {
        regs!(self, r, r.current_value0().read().current_value0().bits())
    }
    fn acknowledge(&mut self) {
        // Complete SDK acknowledgement command; never read or RMW EOI.
        regs!(self, r, unsafe { r.eoi_ren().write_with_zero(|w| w.eoi().set_bit()) });
    }
    fn disable_local(&mut self) {
        unsafe { interrupt::disable(self.channel.interrupt()) };
    }
    fn clear_local(&mut self) {
        interrupt::clear_pending(self.channel.interrupt());
    }
    fn pending(&self) -> bool {
        regs!(self, r, r.raw_intr().read().int_status().bit_is_set())
    }
    fn one_shot(&self) -> bool {
        // SDK mode encodings 0 and 2 are equivalent one-shot variants.
        regs!(self, r, r.control().read().mode().bits() & 1 == 0)
    }
}

#[cfg(all(test, not(target_arch = "riscv32")))]
#[path = "rtc_ws53/tests.rs"]
mod tests;
