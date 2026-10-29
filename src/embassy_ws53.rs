//! Experimental WS53 single-hart Embassy time driver, fixed32MHz inherited clocks.
//!
//! Explicitly consumes the application TIMER/TCXO drivers; uses TIMER1/IRQ54.
//! No pad, clock-tree, counter-clear, sleep or control-core negotiation occurs.
//! Before init, now() returns zero and future deadlines are queued without MMIO.
//! TCXO polls never spin; while a snapshot is pending, the last monotonic time is
//! returned. Snapshot/clock faults are visible through status(), not fake zeros.
use crate::{
    interrupt,
    peripherals::Timer,
    tcxo::{TcxoDriver, TcxoError},
    timer::{TimerChannel, TimerDriver, TimerMode, TimerTicks},
};
use core::{cell::RefCell, num::NonZeroU32, task::Waker};
use critical_section::Mutex;
use embassy_time_driver::Driver;

/// Reserved TIMER1 interrupt (absolute mcause number).
pub const ALARM_IRQ: u32 = 54;
/// Maximum distinct task wakers retained, matching the existing generic queue16 policy.
pub const QUEUE_CAPACITY: usize = 16;
const CHANNEL: TimerChannel = TimerChannel::Channel1;
const COUNTS_PER_US: u64 = 32;
const COUNTS_PER_TICK: NonZeroU32 = NonZeroU32::new(COUNTS_PER_US as u32).unwrap();
const RETRY_COUNTS: u32 = 32_000; // at most1ms retry while TCXO cannot supply fresh time
const _: () = assert!(embassy_time_driver::TICK_HZ == 1_000_000, "WS53 Embassy requires tick-hz-1_000_000");

/// Current time-source observation. No fault is silently represented as timestamp0.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ClockStatus {
    /// No resources installed; now is zero and no hardware is touched.
    Uninitialized,
    /// Last snapshot advanced or preserved the timestamp.
    Ready,
    /// A nonblocking snapshot has not completed; now returns its previous value.
    SnapshotPending,
    /// Snapshot could not be started because TCXO was disabled externally.
    Disabled,
    /// Snapshot moved backwards; now holds its previous value. Epoch contract was broken.
    Backwards,
}
/// Read-only diagnostic state; no ownership or counter reset is implied.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Status {
    /// Driver resources have been installed.
    pub initialized: bool,
    /// Latest snapshot observation.
    pub clock: ClockStatus,
    /// Last accepted microsecond timestamp.
    pub ticks: u64,
    /// Number of distinct task wakers currently retained.
    pub pending_wakers: usize,
}
/// Refused time-driver installation; both resources are returned to the caller.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum InitError {
    /// Installation already completed or is in progress.
    AlreadyInitialized,
    /// Either TIMER channel is running.
    TimerRunning,
    /// Either TIMER local interrupt is enabled or a pending source exists.
    TimerInterruptActive,
    /// TCXO was disabled/busy or its initial bounded snapshot did not complete.
    Clock(TcxoError),
}

struct Entry {
    at: u64,
    waker: Waker,
}
struct Queue {
    entries: [Option<Entry>; QUEUE_CAPACITY],
}
impl Queue {
    const fn new() -> Self {
        Self { entries: [const { None }; QUEUE_CAPACITY] }
    }
    // Cloning and dropping Wakers belongs to the caller, outside its critical section.
    // Later deadlines for the same task retain the earliest outstanding wake.
    fn insert(&mut self, at: u64, waker: Waker) -> (Option<Waker>, Option<Waker>) {
        if let Some(entry) = self.entries.iter_mut().flatten().find(|e| e.waker.will_wake(&waker)) {
            entry.at = entry.at.min(at);
            return (Some(waker), None);
        }
        let index =
            self.entries.iter().position(Option::is_none).unwrap_or_else(|| {
                self.entries.iter().enumerate().max_by_key(|(_, e)| e.as_ref().unwrap().at).unwrap().0
            });
        let evicted = self.entries[index].replace(Entry { at, waker }).map(|e| e.waker);
        (None, evicted)
    }
    fn drain(&mut self, now: u64, all: bool) -> [Option<Waker>; QUEUE_CAPACITY] {
        core::array::from_fn(|i| {
            if self.entries[i].as_ref().is_some_and(|e| all || e.at <= now) {
                self.entries[i].take().map(|e| e.waker)
            } else {
                None
            }
        })
    }
    fn next(&self) -> Option<u64> {
        self.entries.iter().flatten().map(|e| e.at).min()
    }
    fn len(&self) -> usize {
        self.entries.iter().flatten().count()
    }
}
struct Resources {
    timer: TimerDriver<'static>,
    clock: TcxoDriver<'static>,
}
struct State {
    resources: Option<Resources>,
    initialized: bool,
    installing: bool,
    busy: bool,
    dirty: bool,
    last: u64,
    health: ClockStatus,
    queue: Queue,
}
impl State {
    const fn new() -> Self {
        Self {
            resources: None,
            initialized: false,
            installing: false,
            busy: false,
            dirty: false,
            last: 0,
            health: ClockStatus::Uninitialized,
            queue: Queue::new(),
        }
    }
    fn accept_ticks(&mut self, raw: u64) {
        let ticks = crate::embassy_time_math::counter_ticks(raw, COUNTS_PER_TICK);
        if ticks < self.last {
            self.health = ClockStatus::Backwards;
        } else {
            self.last = ticks;
            self.health = ClockStatus::Ready;
        }
    }
    fn accept(&mut self, sample: nb::Result<crate::tcxo::CounterTicks, TcxoError>) {
        match sample {
            Ok(value) => {
                self.accept_ticks(value.ticks());
            }
            Err(nb::Error::WouldBlock) => self.health = ClockStatus::SnapshotPending,
            Err(nb::Error::Other(_)) => self.health = ClockStatus::Disabled,
        }
    }
}
struct Ws53TimeDriver {
    state: Mutex<RefCell<State>>,
}
fn alarm_counts(at: u64, now: u64, retry: bool) -> TimerTicks {
    // Hardware periods are segments of the retained absolute deadline, not a
    // truncated deadline. u64::MAX is a real deadline, never an empty sentinel.
    let count = crate::embassy_time_math::alarm_counts(at, now, COUNTS_PER_TICK);
    TimerTicks::from_count(if retry { count.min(RETRY_COUNTS) } else { count }).unwrap()
}
fn stop(timer: &mut TimerDriver<'_>) {
    timer.disable(CHANNEL);
    timer.set_interrupt_mask(CHANNEL, true);
    timer.clear_interrupt(CHANNEL);
}
fn wake_all(batch: [Option<Waker>; QUEUE_CAPACITY]) {
    for waker in batch.into_iter().flatten() {
        waker.wake();
    }
}
impl Ws53TimeDriver {
    fn with<R>(&self, f: impl FnOnce(&mut State) -> R) -> R {
        critical_section::with(|cs| f(&mut self.state.borrow(cs).borrow_mut()))
    }
    fn service(&self, from_irq: bool) -> u64 {
        let resources = self.with(|s| {
            if !s.initialized {
                return None;
            }
            if s.busy {
                if from_irq {
                    s.dirty = true;
                    unsafe { interrupt::disable(CHANNEL.interrupt()) };
                }
                return None;
            }
            s.busy = true;
            s.resources.take()
        });
        let Some(mut resources) = resources else {
            return self.with(|s| s.last);
        };
        unsafe { interrupt::disable(CHANNEL.interrupt()) };
        // No polling loop and no MMIO within the driver metadata critical section.
        let sample = resources.clock.poll_time_counter();
        let (expired, next, now, retry) = self.with(|s| {
            s.accept(sample);
            let expired = s.queue.drain(s.last, false);
            let next = s.queue.next();
            s.dirty = false;
            (expired, next, s.last, s.health != ClockStatus::Ready)
        });
        stop(&mut resources.timer);
        if let Some(at) = next {
            resources.timer.configure(CHANNEL, TimerMode::OneShot, alarm_counts(at, now, retry));
            resources.timer.enable(CHANNEL);
        }
        let changed = self.with(|s| {
            // Another ISR may have scheduled an earlier deadline while registers
            // were being programmed. Wake those queued tasks for a fresh poll,
            // rather than losing them behind the stale alarm. The queue is bounded.
            let changed = if s.dirty { s.queue.drain(0, true) } else { [const { None }; QUEUE_CAPACITY] };
            s.dirty = false;
            s.resources = Some(resources);
            s.busy = false;
            // Only local-controller bookkeeping is performed in this section.
            if next.is_some() {
                unsafe { interrupt::enable(CHANNEL.interrupt()) };
            }
            changed
        });
        wake_all(expired);
        wake_all(changed);
        self.with(|s| s.last)
    }
}
impl Driver for Ws53TimeDriver {
    fn now(&self) -> u64 {
        self.service(false)
    }
    fn schedule_wake(&self, at: u64, waker: &Waker) {
        let cloned = waker.clone();
        let (old, evicted, immediate) = self.with(|s| {
            if at <= s.last {
                return (None, None, Some(cloned));
            }
            let (old, evicted) = s.queue.insert(at, cloned);
            s.dirty = true;
            (old, evicted, None)
        });
        self.service(false);
        drop(old);
        if let Some(waker) = evicted {
            waker.wake();
        }
        if let Some(waker) = immediate {
            waker.wake();
        }
    }
}
embassy_time_driver::time_driver_impl!(static DRIVER: Ws53TimeDriver = Ws53TimeDriver {state:Mutex::new(RefCell::new(State::new()))});

/// Consume two explicitly handed-over drivers for the process lifetime.
/// No deinitialization or resource return after success is provided. Both TIMER
/// channels must be stopped/acknowledged; TIMER0 registers are left unchanged.
/// TCXO must already be enabled at32MHz, without an outstanding snapshot.
/// Initialization uses one bounded snapshot outside critical sections; its
/// failure returns both resources (a timed-out TCXO request remains retained).
///
/// # Safety
/// Establish exclusive application ownership of both blocks, clocks and this
/// snapshot window, quiescing vendor/control-core/DMA/NMI users. All Embassy
/// time calls and this ISR must run on the same application hart permanently.
/// Preserve the uninterrupted32MHz TCXO/TIMER epoch (no reset/sleep/rate changes).
/// Route TIMER1/IRQ54 exclusively to on_alarm_interrupt without reentrant IRQ
/// delivery. Wakers must be ISR-safe; global MIE and threshold are caller-owned.
pub unsafe fn init(
    timer: TimerDriver<'static>,
    mut clock: TcxoDriver<'static>,
) -> Result<(), (InitError, TimerDriver<'static>, TcxoDriver<'static>)> {
    let reserved = DRIVER.with(|s| {
        if s.initialized || s.installing {
            false
        } else {
            s.installing = true;
            true
        }
    });
    if !reserved {
        return Err((InitError::AlreadyInitialized, timer, clock));
    }
    let check = (|| {
        let r = unsafe { &*Timer::ptr() };
        if r.timer_control0().read().enable().bit() || r.timer_control1().read().enable().bit() {
            return Err(InitError::TimerRunning);
        }
        for ch in [TimerChannel::Channel0, TimerChannel::Channel1] {
            if interrupt::is_enabled(ch.interrupt())
                || interrupt::is_pending(ch.interrupt())
                || timer.interrupt_pending(ch)
            {
                return Err(InitError::TimerInterruptActive);
            }
        }
        clock.time_source_ready().map_err(InitError::Clock)?;
        clock.read_counter().map_err(InitError::Clock)
    })();
    match check {
        Err(error) => {
            DRIVER.with(|s| s.installing = false);
            Err((error, timer, clock))
        }
        Ok(value) => {
            DRIVER.with(|s| {
                s.accept(Ok(value));
                s.resources = Some(Resources { timer, clock });
                s.initialized = true;
                s.installing = false;
            });
            DRIVER.service(false);
            Ok(())
        }
    }
}
/// Service TIMER1. Does not spin on the TCXO or invoke a waker with state borrowed.
/// Call only from the installed ISR (or while its local interrupt is disabled).
/// Spurious invocations recheck deadlines; they never complete time early.
pub fn on_alarm_interrupt() {
    DRIVER.service(true);
}
/// Observe initialization, monotonic cache and bounded queue occupancy without MMIO.
pub fn status() -> Status {
    DRIVER.with(|s| Status {
        initialized: s.initialized,
        clock: s.health,
        ticks: s.last,
        pending_wakers: s.queue.len(),
    })
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
    struct Count(AtomicUsize);
    impl Wake for Count {
        fn wake(self: Arc<Self>) {
            self.0.fetch_add(1, Ordering::SeqCst);
        }
    }
    fn counter() -> (Arc<Count>, Waker) {
        let c = Arc::new(Count(AtomicUsize::new(0)));
        let w = Waker::from(c.clone());
        (c, w)
    }
    #[test]
    fn same_task_keeps_earliest_and_callbacks_run_only_after_dequeue() {
        let mut q = Queue::new();
        let (c, w) = counter();
        drop(q.insert(50, w.clone()));
        drop(q.insert(100, w.clone()));
        assert_eq!(q.next(), Some(50));
        drop(q.insert(20, w.clone()));
        assert_eq!(q.len(), 1);
        assert_eq!(q.next(), Some(20));
        wake_all(q.drain(19, false));
        assert_eq!(c.0.load(Ordering::SeqCst), 0);
        let batch = q.drain(20, false);
        assert_eq!(q.len(), 0);
        assert_eq!(c.0.load(Ordering::SeqCst), 0);
        wake_all(batch);
        assert_eq!(c.0.load(Ordering::SeqCst), 1);
    }
    #[test]
    fn overflow_wakes_evicted_task_and_retains_new_deadline() {
        let mut q = Queue::new();
        let mut counters = std::vec::Vec::new();
        for i in 0..=QUEUE_CAPACITY {
            let (c, w) = counter();
            counters.push(c);
            let (old, wake) = q.insert(i as u64 + 1, w);
            drop(old);
            if let Some(w) = wake {
                w.wake();
            }
        }
        assert_eq!(q.len(), QUEUE_CAPACITY);
        assert_eq!(q.next(), Some(1));
        assert_eq!(counters[QUEUE_CAPACITY - 1].0.load(Ordering::SeqCst), 1);
        wake_all(q.drain(u64::MAX, false));
        assert!(counters.iter().all(|c| c.0.load(Ordering::SeqCst) == 1));
    }
    #[test]
    fn concurrent_change_drains_all_for_repoll_without_losing_far_future() {
        let mut q = Queue::new();
        let (c, w) = counter();
        drop(q.insert(u64::MAX, w));
        assert_eq!(q.next(), Some(u64::MAX));
        assert_eq!(q.len(), 1);
        wake_all(q.drain(0, true));
        assert_eq!(c.0.load(Ordering::SeqCst), 1);
        assert_eq!(q.next(), None);
    }
    #[test]
    fn alarm_segments_keep_absolute_deadline_without_overflow() {
        assert_eq!(alarm_counts(0, 1, false).count(), 1);
        assert_eq!(alarm_counts(10, 9, false).count(), 32);
        assert_eq!(alarm_counts(u64::MAX, 0, false).count(), u32::MAX);
        assert_eq!(alarm_counts(u64::MAX, 0, true).count(), RETRY_COUNTS);
        assert_eq!(alarm_counts(10, 9, true).count(), 32);
        for delta in [1, 2, 1000, 134217727, 134217728, u64::MAX] {
            let count = u64::from(alarm_counts(delta, 0, false).count());
            assert!(count <= u32::MAX as u64);
            if delta <= u32::MAX as u64 / 32 {
                assert_eq!(count, delta * 32);
            } else {
                assert_eq!(count, u32::MAX as u64);
            }
        }
    }
    #[test]
    fn clock_faults_hold_epoch_and_recovery_never_rewinds() {
        let mut state = State::new();
        state.accept_ticks(32_000);
        assert_eq!(state.last, 1000);
        state.accept(Err(nb::Error::WouldBlock));
        assert_eq!(state.health, ClockStatus::SnapshotPending);
        assert_eq!(state.last, 1000);
        state.accept(Err(nb::Error::Other(TcxoError::Disabled)));
        assert_eq!(state.health, ClockStatus::Disabled);
        assert_eq!(state.last, 1000);
        state.accept_ticks(0);
        assert_eq!(state.health, ClockStatus::Backwards);
        assert_eq!(state.last, 1000);
        state.accept_ticks(64_000);
        assert_eq!(state.health, ClockStatus::Ready);
        assert_eq!(state.last, 2000);
        state.accept_ticks(u64::MAX);
        assert_eq!(state.last, u64::MAX / 32);
    }
    #[test]
    fn uninitialized_now_never_accesses_mmio_and_past_schedule_wakes() {
        let driver = Ws53TimeDriver { state: Mutex::new(RefCell::new(State::new())) };
        assert_eq!(driver.now(), 0);
        let (c, w) = counter();
        driver.schedule_wake(0, &w);
        assert_eq!(c.0.load(Ordering::SeqCst), 1);
        driver.schedule_wake(100, &w);
        assert_eq!(driver.now(), 0);
        assert_eq!(driver.with(|s| s.queue.len()), 1);
    }
}
