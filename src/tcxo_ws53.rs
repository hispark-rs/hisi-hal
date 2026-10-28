//! WS53 TCXO v150 application snapshot window, SDK 163d9150.
//!
//! The four 32-bit data slots contain 16 bits each. Values are raw input-clock
//! ticks, not microseconds. The reference SDK uses 32 ticks/us, but the actual
//! clock, shared counter ownership and epoch must be established by the caller.
//! This driver does not provide a global Instant, sleep compensation, clock
//! provisioning, or a promise that an independently clocked core has stopped.
use crate::peripherals::Tcxo;
use core::num::{NonZeroU16, NonZeroU32};
use core::time::Duration;

/// Snapshot failure, distinct from a successfully sampled zero counter.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TcxoError {
    /// The counter is not enabled; no refresh command was sent.
    Disabled,
    /// The refresh valid flag did not arrive within the configured read budget.
    SnapshotTimeout,
    /// A cancelled/timed-out request has not completed, or a snapshot is active.
    SnapshotBusy,
}

/// Raw captured ticks. No implicit ordering across reset, sleep or clock changes.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct CounterTicks(u64);
impl CounterTicks {
    /// All 64 counter bits, without an implicit frequency conversion.
    pub const fn ticks(self) -> u64 {
        self.0
    }
    /// Convert the raw count at an externally established, constant frequency.
    /// Uses quotient/remainder to avoid a tick*1e9 overflow; floors sub-nanosecond fractions.
    /// This is time since this counter's epoch, not wall clock or compensated uptime.
    pub const fn as_duration(self, frequency: NonZeroU32) -> Duration {
        let hz = frequency.get() as u64;
        let seconds = self.0 / hz;
        let nanos = ((self.0 % hz) * 1_000_000_000 / hz) as u32;
        Duration::new(seconds, nanos)
    }
    /// Duration between samples from the same uninterrupted counter/clock epoch.
    /// Backwards ticks (including wrap) return None instead of saturating to zero.
    /// The caller must exclude reset/clear/sleep/rate changes between samples;
    /// arithmetic alone cannot detect all epoch changes.
    pub const fn checked_duration_since(self, earlier: Self, frequency: NonZeroU32) -> Option<Duration> {
        match self.0.checked_sub(earlier.0) {
            Some(ticks) => Some(Self(ticks).as_duration(frequency)),
            None => None,
        }
    }
}

/// Exclusive access to a handed-over application snapshot window.
/// All commands and snapshots require &mut self; a Rust critical section alone
/// would not protect the window from a control-core or NMI user.
pub struct TcxoDriver<'d> {
    _token: Tcxo<'d>,
    max_polls: NonZeroU16,
    pending: bool,
}
impl<'d> TcxoDriver<'d> {
    /// Attach without MMIO, enabling, clearing or changing clocks.
    /// `max_polls` counts VALID reads for a fresh request, not microseconds.
    /// Draining a cancelled request may add one prior VALID read (maximum 65535
    /// fresh polls). The application must hand over an idle snapshot window.
    pub fn new(token: Tcxo<'d>, max_polls: NonZeroU16) -> Self {
        Self { _token: token, max_polls, pending: false }
    }
    fn regs(&self) -> &'static crate::soc::pac::tcxo::RegisterBlock {
        unsafe { &*Tcxo::ptr() }
    }
    /// Enable counting without resetting the inherited epoch or latching data.
    /// A retained snapshot must first complete and be discarded; otherwise Busy
    /// is returned without changing the status register.
    pub fn enable(&mut self) -> Result<(), TcxoError> {
        if self.pending {
            return Err(TcxoError::SnapshotBusy);
        }
        self.regs().tcxo_status().modify(|_, w| {
            // Do not replay command bits from an unspecified hardware readback.
            w.refresh().clear_bit().clear().clear_bit().enable().set_bit()
        });
        Ok(())
    }
    /// Clear the enabled counter. All previously captured timestamps now belong
    /// to an obsolete epoch. This is explicit; neither construction nor Drop clears it.
    /// Refuses a retained snapshot with Busy, even if hardware has since finished.
    pub fn clear_counter(&mut self) -> Result<(), TcxoError> {
        if self.pending {
            return Err(TcxoError::SnapshotBusy);
        }
        if !self.regs().tcxo_status().read().enable().bit_is_set() {
            return Err(TcxoError::Disabled);
        }
        self.regs().tcxo_status().modify(|_, w| w.refresh().clear_bit().clear().set_bit());
        Ok(())
    }
    /// Request a snapshot and read the four 16-bit pieces in SDK order, high to low.
    /// Does not synthesize zero on timeout. No unbounded SDK-style 0xffffffff loop.
    /// Timeout retains the request. A retry returns Busy until that old request
    /// completes; it then discards the old result and starts a fresh snapshot.
    pub fn read_counter(&mut self) -> Result<CounterTicks, TcxoError> {
        let regs = self.regs();
        snapshot(&mut PacRegisters(regs), self.max_polls, &mut self.pending)
    }
    /// Start one nonblocking refresh, borrowing the window until completion or Drop.
    /// A cancelled/timeout request is discarded only once VALID arrives. A new
    /// request is never issued over an unfinished one; that case returns Busy.
    /// This method does not poll or wait. Clock/epoch ownership remains external.
    pub fn begin_snapshot(&mut self) -> Result<CounterSnapshot<'_, 'd>, TcxoError> {
        let regs = self.regs();
        begin(&mut PacRegisters(regs), &mut self.pending)?;
        Ok(CounterSnapshot { driver: self, completed: None })
    }
    /// Discard a previously cancelled or timed-out snapshot once it completes.
    /// Checks VALID at most once, reads no counter data and does not alter the
    /// epoch or send a refresh. WouldBlock retains ownership of the old request.
    pub fn try_discard_snapshot(&mut self) -> nb::Result<(), TcxoError> {
        let regs = self.regs();
        discard(&mut PacRegisters(regs), &mut self.pending)
    }
    // The global time driver owns this window permanently after an idle handoff.
    // Its pending request remains active across calls, rather than being cancelled
    // by a dropped public CounterSnapshot. Exactly one VALID read, never a loop.
    #[cfg(feature = "embassy")]
    pub(crate) fn poll_time_counter(&mut self) -> nb::Result<CounterTicks, TcxoError> {
        let regs = self.regs();
        let mut io = PacRegisters(regs);
        if !self.pending {
            begin(&mut io, &mut self.pending).map_err(nb::Error::Other)?;
        }
        poll(&mut io, &mut self.pending)
    }
    #[cfg(feature = "embassy")]
    pub(crate) fn time_source_ready(&self) -> Result<(), TcxoError> {
        if self.pending {
            return Err(TcxoError::SnapshotBusy);
        }
        if !self.regs().tcxo_status().read().enable().bit_is_set() {
            return Err(TcxoError::Disabled);
        }
        Ok(())
    }
}

/// One exclusive TCXO refresh transaction. No allocation, IRQ or waker is used.
/// `try_read` checks VALID once; callers decide when to retry. Completed reads
/// are cached so repeated calls never consume a different hardware sample.
/// Drop does not pretend to cancel the hardware: the driver retains a pending
/// request until it can be discarded. No clear/enable/refresh bypass is offered.
/// ```compile_fail
/// use hisi_hal::tcxo::TcxoDriver;
/// fn overlap(tcxo: &mut TcxoDriver<'_>) {
///     let mut snapshot = tcxo.begin_snapshot().unwrap();
///     tcxo.clear_counter().unwrap();
///     let _ = snapshot.try_read();
/// }
/// ```
pub struct CounterSnapshot<'a, 'd> {
    driver: &'a mut TcxoDriver<'d>,
    completed: Option<CounterTicks>,
}
impl CounterSnapshot<'_, '_> {
    /// Return the captured value, or WouldBlock without reading stale data.
    pub fn try_read(&mut self) -> nb::Result<CounterTicks, TcxoError> {
        if let Some(value) = self.completed {
            return Ok(value);
        }
        let regs = self.driver.regs();
        let value = poll(&mut PacRegisters(regs), &mut self.driver.pending)?;
        self.completed = Some(value);
        Ok(value)
    }
}

trait Registers {
    fn enabled(&mut self) -> bool;
    fn request(&mut self);
    fn valid(&mut self) -> bool;
    fn words_high_to_low(&mut self) -> [u16; 4];
}
struct PacRegisters(&'static crate::soc::pac::tcxo::RegisterBlock);
impl Registers for PacRegisters {
    fn enabled(&mut self) -> bool {
        self.0.tcxo_status().read().enable().bit_is_set()
    }
    fn request(&mut self) {
        self.0.tcxo_status().modify(|_, w| w.clear().clear_bit().refresh().set_bit());
    }
    fn valid(&mut self) -> bool {
        self.0.tcxo_status().read().valid().bit_is_set()
    }
    fn words_high_to_low(&mut self) -> [u16; 4] {
        // Field readers mask the reserved high halves, unlike raw .bits().
        [
            self.0.tcxo_count3().read().count3().bits(),
            self.0.tcxo_count2().read().count2().bits(),
            self.0.tcxo_count1().read().count1().bits(),
            self.0.tcxo_count0().read().count0().bits(),
        ]
    }
}
fn discard(io: &mut impl Registers, pending: &mut bool) -> nb::Result<(), TcxoError> {
    if *pending {
        if !io.valid() {
            return Err(nb::Error::WouldBlock);
        }
        *pending = false;
    }
    Ok(())
}
fn begin(io: &mut impl Registers, pending: &mut bool) -> Result<(), TcxoError> {
    if !io.enabled() {
        return Err(TcxoError::Disabled);
    }
    discard(io, pending).map_err(|_| TcxoError::SnapshotBusy)?;
    io.request();
    *pending = true;
    Ok(())
}
fn poll(io: &mut impl Registers, pending: &mut bool) -> nb::Result<CounterTicks, TcxoError> {
    if !io.valid() {
        return Err(nb::Error::WouldBlock);
    }
    let [high, mid_high, mid_low, low] = io.words_high_to_low();
    *pending = false;
    Ok(CounterTicks(crate::tcxo_words::join([
        u64::from(high),
        u64::from(mid_high),
        u64::from(mid_low),
        u64::from(low),
    ])))
}
fn snapshot(io: &mut impl Registers, max_polls: NonZeroU16, pending: &mut bool) -> Result<CounterTicks, TcxoError> {
    begin(io, pending)?;
    for _ in 0..max_polls.get() {
        match poll(io, pending) {
            Ok(value) => return Ok(value),
            Err(nb::Error::WouldBlock) => (),
            Err(nb::Error::Other(error)) => return Err(error),
        }
        core::hint::spin_loop();
    }
    Err(TcxoError::SnapshotTimeout)
}

#[cfg(all(test, not(target_arch = "riscv32")))]
#[path = "tcxo_ws53/tests.rs"]
mod tests;
