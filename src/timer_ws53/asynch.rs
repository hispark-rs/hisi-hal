//! Single-hart, one-shot interrupt-driven delay on an explicitly owned TIMER.
use super::{TIMER_CLOCK_HZ, TimerChannel, TimerDriver, TimerTicks};
use crate::{interrupt, peripherals::Timer};
use core::{
    cell::Cell,
    future::poll_fn,
    marker::PhantomData,
    task::{Context, Poll, Waker},
};
use critical_section::Mutex;

struct Signal {
    active: Cell<bool>,
    fired: Cell<bool>,
    waker: Cell<Option<Waker>>,
}
impl Signal {
    const fn new() -> Self {
        Self { active: Cell::new(false), fired: Cell::new(false), waker: Cell::new(None) }
    }
}
static SIGNALS: Mutex<[Signal; 2]> = Mutex::new([const { Signal::new() }; 2]);

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
/// Requires a stable, established32MHz TIMER clock. Implements
/// `embedded_hal_async::delay::DelayNs` with upward rounding and multi-period
/// splitting for long requests. Zero duration performs no MMIO or wait.
/// Dropping a Pending delay stops/masks/acknowledges only the selected channel
/// and removes its waker; it can then be reused. The other channel is untouched.
/// The borrow covers the whole existing `TimerDriver` (no channel splitting).
/// A stalled clock or missing ISR leaves the future Pending until cancelled.
/// No global interrupt, shared clock, sleep, or priority-threshold setup is done.
/// Local IRQ enabling uses the existing controller API's priority1 policy.
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
        let old = critical_section::with(|cs| cancel(&mut PacAlarm(ch), &SIGNALS.borrow(cs)[ch.index()]));
        drop(old);
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
        let (fired, waker) = critical_section::with(|cs| notify(&mut PacAlarm(ch), &SIGNALS.borrow(cs)[ch.index()]));
        if let Some(waker) = waker {
            waker.wake();
        }
        fired
    }
}

trait AlarmIo {
    fn stop(&mut self);
    fn arm(&mut self, ticks: TimerTicks);
    fn pending(&mut self) -> bool;
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
    fn arm(&mut self, ticks: TimerTicks) {
        let r = unsafe { &*Timer::ptr() };
        channel!(self.0, r, reg, timer_load_count0, timer_load_count1, unsafe {
            reg.write_with_zero(|w| w.load_count().bits(ticks.count()))
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
fn cancel(io: &mut impl AlarmIo, signal: &Signal) -> Option<Waker> {
    io.stop();
    signal.active.set(false);
    signal.fired.set(false);
    signal.waker.take()
}
fn notify(io: &mut impl AlarmIo, signal: &Signal) -> (bool, Option<Waker>) {
    if !io.pending() {
        return (false, None);
    }
    io.stop();
    if signal.active.replace(false) {
        signal.fired.set(true);
        (true, signal.waker.take())
    } else {
        (false, None)
    }
}
type PollOutcome = (Poll<()>, Option<Waker>, Option<Waker>);
fn poll_wait(
    io: &mut impl AlarmIo,
    signal: &Signal,
    started: &mut bool,
    ticks: TimerTicks,
    waker: Waker,
) -> PollOutcome {
    let old = signal.waker.replace(Some(waker));
    if !*started {
        io.stop();
        signal.fired.set(false);
        signal.active.set(true);
        *started = true;
        io.arm(ticks);
    }
    if signal.fired.replace(false) { (Poll::Ready(()), old, signal.waker.take()) } else { (Poll::Pending, old, None) }
}
struct Wait<'a, 'd> {
    _timer: &'a mut TimerDriver<'d>,
    channel: TimerChannel,
    ticks: TimerTicks,
    started: bool,
}
impl Wait<'_, '_> {
    fn poll(&mut self, cx: &mut Context<'_>) -> Poll<()> {
        let waker = cx.waker().clone();
        let (result, old, done) = critical_section::with(|cs| {
            poll_wait(
                &mut PacAlarm(self.channel),
                &SIGNALS.borrow(cs)[self.channel.index()],
                &mut self.started,
                self.ticks,
                waker,
            )
        });
        drop((old, done));
        result
    }
}
impl Drop for Wait<'_, '_> {
    fn drop(&mut self) {
        if self.started {
            let old = critical_section::with(|cs| {
                cancel(&mut PacAlarm(self.channel), &SIGNALS.borrow(cs)[self.channel.index()])
            });
            drop(old);
        }
    }
}
impl Drop for AsyncDelay<'_, '_> {
    fn drop(&mut self) {
        let old =
            critical_section::with(|cs| cancel(&mut PacAlarm(self.channel), &SIGNALS.borrow(cs)[self.channel.index()]));
        drop(old);
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
        let mut wait = Wait { _timer: &mut *self.timer, channel: self.channel, ticks, started: false };
        poll_fn(|cx| wait.poll(cx)).await;
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
    use std::{
        sync::{
            Arc,
            atomic::{AtomicUsize, Ordering},
        },
        task::Wake,
    };
    struct Counter(AtomicUsize);
    impl Wake for Counter {
        fn wake(self: Arc<Self>) {
            self.0.fetch_add(1, Ordering::SeqCst);
        }
    }
    #[derive(Default)]
    struct Io {
        armed: bool,
        pending: bool,
        arms: usize,
        count: u32,
    }
    impl AlarmIo for Io {
        fn stop(&mut self) {
            self.armed = false;
            self.pending = false;
        }
        fn arm(&mut self, ticks: TimerTicks) {
            self.armed = true;
            self.arms += 1;
            self.count = ticks.count();
        }
        fn pending(&mut self) -> bool {
            self.pending
        }
    }
    const TICKS: TimerTicks = TimerTicks::from_count(32000).unwrap();
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
    #[test]
    fn repoll_and_spurious_irq_do_not_restart_or_complete_timer() {
        let signal = Signal::new();
        let mut io = Io::default();
        let mut started = false;
        let a = Arc::new(Counter(AtomicUsize::new(0)));
        let b = Arc::new(Counter(AtomicUsize::new(0)));
        assert!(poll_wait(&mut io, &signal, &mut started, TICKS, Waker::from(a.clone())).0.is_pending());
        assert!(!notify(&mut io, &signal).0);
        assert!(io.armed);
        let (result, old, done) = poll_wait(&mut io, &signal, &mut started, TICKS, Waker::from(b.clone()));
        drop((old, done));
        assert!(result.is_pending());
        assert_eq!(io.arms, 1);
        assert_eq!(Arc::strong_count(&a), 1);
        io.pending = true;
        let (fired, waker) = notify(&mut io, &signal);
        assert!(fired);
        waker.unwrap().wake();
        assert_eq!(b.0.load(Ordering::SeqCst), 1);
        assert_eq!(a.0.load(Ordering::SeqCst), 0);
        assert!(poll_wait(&mut io, &signal, &mut started, TICKS, Waker::from(b.clone())).0.is_ready());
        assert_eq!(Arc::strong_count(&b), 1);
        assert!(!io.armed);
    }
    #[test]
    fn cancel_before_or_after_expiry_discards_old_wake_and_allows_fresh_wait() {
        for expire in [false, true] {
            let signal = Signal::new();
            let mut io = Io::default();
            let mut started = false;
            let count = Arc::new(Counter(AtomicUsize::new(0)));
            assert!(poll_wait(&mut io, &signal, &mut started, TICKS, Waker::from(count.clone())).0.is_pending());
            io.pending = expire;
            drop(cancel(&mut io, &signal));
            assert!(!io.armed && !io.pending && !signal.active.get() && !signal.fired.get());
            assert_eq!(Arc::strong_count(&count), 1);
            assert!(!notify(&mut io, &signal).0);
            assert_eq!(count.0.load(Ordering::SeqCst), 0);
            started = false;
            assert!(poll_wait(&mut io, &signal, &mut started, TICKS, Waker::from(count.clone())).0.is_pending());
            io.pending = true;
            let (fired, waker) = notify(&mut io, &signal);
            assert!(fired);
            waker.unwrap().wake();
            assert!(poll_wait(&mut io, &signal, &mut started, TICKS, Waker::from(count.clone())).0.is_ready());
            assert_eq!(count.0.load(Ordering::SeqCst), 1);
        }
    }
    #[test]
    fn completion_before_repoll_is_sticky_and_cancellation_clears_it() {
        let signal = Signal::new();
        let mut io = Io::default();
        let mut started = false;
        assert!(poll_wait(&mut io, &signal, &mut started, TICKS, Waker::noop().clone()).0.is_pending());
        io.pending = true;
        let (_, waker) = notify(&mut io, &signal);
        drop(waker);
        assert!(signal.fired.get());
        assert!(!notify(&mut io, &signal).0);
        drop(cancel(&mut io, &signal));
        assert!(!signal.fired.get());
        started = false;
        assert!(poll_wait(&mut io, &signal, &mut started, TICKS, Waker::noop().clone()).0.is_pending());
        assert_eq!(io.arms, 2);
    }
}
