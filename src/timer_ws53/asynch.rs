//! Single-hart, one-shot interrupt-driven delay on an explicitly owned TIMER.
use super::{TIMER_CLOCK_HZ, TimerChannel, TimerDriver, TimerTicks};
use crate::timer_async::{self, AlarmIo, Signal, Wait};
use crate::{interrupt, peripherals::Timer};
use core::{marker::PhantomData, num::NonZeroU32};

static SIGNALS: [Signal; 2] = [const { Signal::new() }; 2];

/// Refused async takeover; no peripheral/controller state has been changed.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AsyncBindError {
    /// The selected counter is still enabled.
    Running,
    /// Its local interrupt is still enabled.
    InterruptsEnabled,
    /// Its peripheral or local pending latch must first be explicitly cleared.
    PendingInterrupt,
}

/// Exclusive borrow of TIMER for asynchronous one-shot delays on one channel.
///
/// Requires a stable, established 32 MHz TIMER clock. Implements
/// `embedded_hal_async::delay::DelayNs` with upward rounding and multi-period
/// splitting for long requests. Zero duration performs no MMIO or wait.
/// Dropping a Pending delay stops/masks/acknowledges only the selected channel
/// and removes its waker; it can then be reused. The other channel is untouched.
/// The borrow covers the whole existing `TimerDriver` (no channel splitting).
/// A stalled clock or missing ISR leaves the future Pending until cancelled.
/// No global interrupt, shared clock, sleep, or priority-threshold setup is done.
/// Local IRQ enabling uses the existing controller API's priority 1 policy.
pub struct AsyncDelay<'a, 'd> {
    timer: &'a mut TimerDriver<'d>,
    channel: TimerChannel,
    _local: PhantomData<*mut ()>,
}

impl<'d> TimerDriver<'d> {
    /// Borrow a stopped, acknowledged channel for async delays.
    ///
    /// # Safety
    /// Route the selected channel's non-reentrant ISR to `on_async_interrupt`
    /// on this application hart. Maintain the handed-over timer/clock and that
    /// routing until the adapter is dropped. No other core/NMI/ISR may modify
    /// this channel; do not access the borrowed driver from the handler. Global
    /// interrupts must allow delivery while awaiting. Wakers must be ISR-safe.
    pub unsafe fn async_delay(&mut self, ch: TimerChannel) -> Result<AsyncDelay<'_, 'd>, AsyncBindError> {
        let r = self.regs();
        if channel!(ch, r, reg, timer_control0, timer_control1, reg.read().enable().bit()) {
            return Err(AsyncBindError::Running);
        }
        if interrupt::is_enabled(ch.interrupt()) {
            return Err(AsyncBindError::InterruptsEnabled);
        }
        if self.interrupt_pending(ch) || interrupt::is_pending(ch.interrupt()) {
            return Err(AsyncBindError::PendingInterrupt);
        }
        timer_async::cancel(&mut PacAlarm(ch), &SIGNALS[ch.index()]);
        Ok(AsyncDelay { timer: self, channel: ch, _local: PhantomData })
    }

    /// Stop/acknowledge an actual async expiry and wake its waiter once.
    /// Returns false for a stale/spurious notification and does not complete a
    /// still-counting future. Does not borrow the main-context driver or poll.
    ///
    /// # Safety
    /// Call only from the bound channel's ISR or with its IRQ disabled, on the
    /// owning hart, while the application timer handoff remains valid. This is
    /// the only timer service allowed in that ISR. Waking must be ISR-safe.
    pub unsafe fn on_async_interrupt(ch: TimerChannel) -> bool {
        timer_async::notify(&mut PacAlarm(ch), &SIGNALS[ch.index()])
    }
}

struct PacAlarm(TimerChannel);
impl AlarmIo for PacAlarm {
    fn stop(&mut self) {
        unsafe { interrupt::disable(self.0.interrupt()) };
        let r = unsafe { &*Timer::ptr() };
        channel!(
            self.0,
            r,
            reg,
            timer_control0,
            timer_control1,
            reg.modify(|_, w| w.enable().clear_bit().int_mask().set_bit())
        );
        unsafe { TimerDriver::acknowledge_interrupt(self.0) };
    }
    fn arm(&mut self, ticks: NonZeroU32) {
        let r = unsafe { &*Timer::ptr() };
        channel!(self.0, r, reg, timer_load_count0, timer_load_count1, unsafe {
            reg.write_with_zero(|w| w.load_count().bits(ticks.get()))
        });
        channel!(
            self.0,
            r,
            reg,
            timer_control0,
            timer_control1,
            reg.modify(|_, w| w
                .enable()
                .clear_bit()
                .mode()
                .one_shot()
                .int_mask()
                .clear_bit()
                .rstfsm()
                .clear_bit()
                .cnt_req()
                .clear_bit())
        );
        unsafe { interrupt::enable(self.0.interrupt()) };
        channel!(self.0, r, reg, timer_control0, timer_control1, reg.modify(|_, w| w.enable().set_bit()));
    }
    fn pending(&mut self) -> bool {
        // Clear only the controller first, then inspect peripheral raw status.
        // A new expiry after the check remains latched at the peripheral; never
        // clear EOI after observing false and accidentally discard that expiry.
        interrupt::clear_pending(self.0.interrupt());
        let r = unsafe { &*Timer::ptr() };
        channel!(self.0, r, reg, timer_raw_intr0, timer_raw_intr1, reg.read().raw_intr().bit())
    }
}
impl Drop for AsyncDelay<'_, '_> {
    fn drop(&mut self) {
        timer_async::cancel(&mut PacAlarm(self.channel), &SIGNALS[self.channel.index()]);
    }
}
fn ticks_for(value: u32, units_per_second: u32) -> u64 {
    crate::timer_math::delay_ticks(value, TIMER_CLOCK_HZ, units_per_second)
}
fn next_chunk(remaining: &mut u64) -> Option<TimerTicks> {
    crate::timer_math::next_delay_chunk(remaining).and_then(TimerTicks::from_count)
}
impl AsyncDelay<'_, '_> {
    /// Wait for one nonzero raw period at the established clock.
    /// Starts on first poll, stops/acknowledges on cancellation; no busy polling.
    pub async fn delay_ticks(&mut self, ticks: TimerTicks) {
        // Keep the whole TIMER borrow for the outer future's lifetime.
        let _owner = &mut *self.timer;
        Wait::new(PacAlarm(self.channel), &SIGNALS[self.channel.index()], ticks.0).await;
    }
    async fn wait_total(&mut self, mut ticks: u64) {
        while let Some(chunk) = next_chunk(&mut ticks) {
            self.delay_ticks(chunk).await;
        }
    }
}
impl embedded_hal_async::delay::DelayNs for AsyncDelay<'_, '_> {
    async fn delay_ns(&mut self, ns: u32) {
        self.wait_total(ticks_for(ns, 1_000_000_000)).await;
    }
    async fn delay_us(&mut self, us: u32) {
        self.wait_total(ticks_for(us, 1_000_000)).await;
    }
    async fn delay_ms(&mut self, ms: u32) {
        self.wait_total(ticks_for(ms, 1_000)).await;
    }
}

#[cfg(all(test, not(target_arch = "riscv32")))]
mod tests {
    use super::*;
    #[test]
    fn rounding_and_long_period_splits_never_shorten_or_lose_duration() {
        for units in [1_000, 1_000_000, 1_000_000_000] {
            for value in [0, 1, 31, 32, 33, 999, 1000, 134217, 134218, u32::MAX] {
                let ticks = ticks_for(value, units);
                let requested = u64::from(value) * u64::from(TIMER_CLOCK_HZ);
                assert!(ticks * u64::from(units) >= requested);
                assert!(ticks == 0 || (ticks - 1) * u64::from(units) < requested);
                let mut remaining = ticks;
                let mut total = 0;
                while let Some(chunk) = next_chunk(&mut remaining) {
                    total += u64::from(chunk.count());
                }
                assert_eq!(total, ticks);
                assert_eq!(remaining, 0);
            }
        }
        assert_eq!(ticks_for(1, 1_000_000_000), 1);
        assert_eq!(ticks_for(32, 1_000_000_000), 2);
    }
}
