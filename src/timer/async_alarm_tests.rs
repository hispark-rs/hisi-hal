//! Logical register/ISR model, not hardware timing or controller qualification.
use super::*;
use std::{
    rc::Rc,
    sync::{
        Arc,
        atomic::{AtomicUsize, Ordering},
        mpsc,
    },
    task::Wake,
    time::Duration,
};

fn unlocked() {
    let (tx, rx) = mpsc::channel();
    let worker = std::thread::spawn(move || critical_section::with(|_| tx.send(()).ok()));
    rx.recv_timeout(Duration::from_secs(2)).expect("callback or IO inside metadata critical section");
    worker.join().unwrap();
}
#[derive(Default)]
struct Registers {
    enabled: Cell<bool>,
    pending: Cell<bool>,
    irq_enabled: Cell<bool>,
    masked: Cell<bool>,
    arms: Cell<usize>,
    stops: Cell<usize>,
    count: Cell<u32>,
    expire_on_arm: Cell<bool>,
    expire_after_check: Cell<bool>,
}
#[derive(Clone)]
struct Io<'a> {
    r: Rc<Registers>,
    signal: &'a Signal,
}
impl<'a> Io<'a> {
    fn new(signal: &'a Signal) -> Self {
        Self { r: Rc::default(), signal }
    }
}
impl AlarmIo for Io<'_> {
    fn stop(&mut self) {
        unlocked();
        self.r.irq_enabled.set(false);
        self.r.enabled.set(false);
        self.r.masked.set(true);
        self.r.pending.set(false);
        self.r.stops.set(self.r.stops.get() + 1);
    }
    fn arm(&mut self, ticks: NonZeroU32) {
        unlocked();
        assert!(!self.r.enabled.get() && !self.r.irq_enabled.get() && !self.r.pending.get());
        self.r.arms.set(self.r.arms.get() + 1);
        self.r.count.set(ticks.get());
        self.r.masked.set(false);
        self.r.irq_enabled.set(true);
        self.r.enabled.set(true);
        if self.r.expire_on_arm.replace(false) {
            self.r.pending.set(true);
            assert!(notify(self, self.signal));
        }
    }
    fn pending(&mut self) -> bool {
        unlocked();
        let pending = self.r.pending.get();
        if self.r.expire_after_check.replace(false) {
            self.r.pending.set(true);
        }
        pending
    }
}
#[derive(Default)]
struct Counter(AtomicUsize);
impl Wake for Counter {
    fn wake(self: Arc<Self>) {
        unlocked();
        self.0.fetch_add(1, Ordering::SeqCst);
    }
}
const TICKS: NonZeroU32 = NonZeroU32::new(32000).unwrap();
fn poll(wait: &mut Wait<'_, Io<'_>>, waker: &Waker) -> Poll<()> {
    Pin::new(wait).poll(&mut Context::from_waker(waker))
}

#[test]
fn unpolled_future_does_not_touch_hardware() {
    let signal = Signal::new();
    let io = Io::new(&signal);
    drop(Wait::new(io.clone(), &signal, TICKS));
    assert_eq!((io.r.arms.get(), io.r.stops.get()), (0, 0));
}

#[test]
fn repoll_and_spurious_irq_do_not_restart_or_complete_timer() {
    let signal = Signal::new();
    let mut io = Io::new(&signal);
    let a = Arc::new(Counter::default());
    let b = Arc::new(Counter::default());
    let mut wait = Wait::new(io.clone(), &signal, TICKS);
    assert!(poll(&mut wait, &Waker::from(a.clone())).is_pending());
    assert!(!notify(&mut io, &signal));
    assert!(io.r.enabled.get());
    assert!(poll(&mut wait, &Waker::from(b.clone())).is_pending());
    assert_eq!(io.r.arms.get(), 1);
    assert_eq!(Arc::strong_count(&a), 1);
    io.r.pending.set(true);
    assert!(notify(&mut io, &signal));
    assert_eq!(b.0.load(Ordering::SeqCst), 1);
    assert_eq!(a.0.load(Ordering::SeqCst), 0);
    assert!(poll(&mut wait, &Waker::from(b.clone())).is_ready());
    assert_eq!(Arc::strong_count(&b), 1);
    assert!(!io.r.enabled.get() && !io.r.irq_enabled.get() && io.r.masked.get());
}

#[test]
fn drop_before_or_after_expiry_clears_old_wait_and_allows_reuse() {
    for expire in [false, true] {
        let signal = Signal::new();
        let mut io = Io::new(&signal);
        let count = Arc::new(Counter::default());
        let mut wait = Wait::new(io.clone(), &signal, TICKS);
        assert!(poll(&mut wait, &Waker::from(count.clone())).is_pending());
        io.r.pending.set(expire);
        drop(wait);
        assert!(!io.r.enabled.get() && !io.r.pending.get() && !io.r.irq_enabled.get());
        assert!(io.r.masked.get());
        assert_eq!(Arc::strong_count(&count), 1);
        assert!(!notify(&mut io, &signal));
        assert_eq!(count.0.load(Ordering::SeqCst), 0);
        let mut wait = Wait::new(io.clone(), &signal, TICKS);
        assert!(poll(&mut wait, &Waker::from(count.clone())).is_pending());
        io.r.pending.set(true);
        assert!(notify(&mut io, &signal));
        assert!(poll(&mut wait, &Waker::from(count.clone())).is_ready());
        assert_eq!(count.0.load(Ordering::SeqCst), 1);
    }
}

#[test]
fn cancellation_after_notified_expiry_does_not_complete_next_wait() {
    let signal = Signal::new();
    let mut io = Io::new(&signal);
    let mut wait = Wait::new(io.clone(), &signal, TICKS);
    assert!(poll(&mut wait, Waker::noop()).is_pending());
    io.r.pending.set(true);
    assert!(notify(&mut io, &signal));
    assert!(!notify(&mut io, &signal));
    drop(wait);
    let mut wait = Wait::new(io.clone(), &signal, TICKS);
    assert!(poll(&mut wait, Waker::noop()).is_pending());
    assert_eq!(io.r.arms.get(), 2);
}

#[test]
fn immediate_expiry_during_arm_is_observed_without_another_irq() {
    let signal = Signal::new();
    let io = Io::new(&signal);
    let count = Arc::new(Counter::default());
    io.r.expire_on_arm.set(true);
    let mut wait = Wait::new(io.clone(), &signal, TICKS);
    assert!(poll(&mut wait, &Waker::from(count.clone())).is_ready());
    assert_eq!(count.0.load(Ordering::SeqCst), 1);
    assert!(!io.r.enabled.get());
}

#[test]
fn expiry_after_raw_status_check_is_not_cleared_by_spurious_irq() {
    let signal = Signal::new();
    let mut io = Io::new(&signal);
    let mut wait = Wait::new(io.clone(), &signal, TICKS);
    assert!(poll(&mut wait, Waker::noop()).is_pending());
    io.r.expire_after_check.set(true);
    assert!(!notify(&mut io, &signal));
    assert!(io.r.pending.get() && io.r.enabled.get());
    assert!(notify(&mut io, &signal));
    assert!(poll(&mut wait, Waker::noop()).is_ready());
}

#[test]
fn channel_cancellation_does_not_touch_another_channel() {
    let a = Signal::new();
    let b = Signal::new();
    let io_a = Io::new(&a);
    let mut io_b = Io::new(&b);
    let mut wa = Wait::new(io_a.clone(), &a, TICKS);
    let mut wb = Wait::new(io_b.clone(), &b, TICKS);
    assert!(poll(&mut wa, Waker::noop()).is_pending());
    assert!(poll(&mut wb, Waker::noop()).is_pending());
    drop(wa);
    assert!(io_b.r.enabled.get() && io_b.r.irq_enabled.get());
    io_b.r.pending.set(true);
    assert!(notify(&mut io_b, &b));
    assert!(poll(&mut wb, Waker::noop()).is_ready());
}

static CHECK_LOCK: core::task::RawWakerVTable = core::task::RawWakerVTable::new(
    |_| {
        unlocked();
        core::task::RawWaker::new(core::ptr::null(), &CHECK_LOCK)
    },
    |_| unlocked(),
    |_| unlocked(),
    |_| unlocked(),
);
#[test]
fn all_waker_callbacks_and_backend_operations_are_outside_metadata_lock() {
    // SAFETY: static callbacks ignore the null data; no borrowed state or allocation.
    let waker = unsafe { Waker::from_raw(core::task::RawWaker::new(core::ptr::null(), &CHECK_LOCK)) };
    let signal = Signal::new();
    let mut io = Io::new(&signal);
    let mut wait = Wait::new(io.clone(), &signal, TICKS);
    assert!(poll(&mut wait, &waker).is_pending());
    assert!(poll(&mut wait, &waker).is_pending());
    io.r.pending.set(true);
    assert!(notify(&mut io, &signal));
    assert!(poll(&mut wait, &waker).is_ready());
    drop(wait);
    let mut wait = Wait::new(io, &signal, TICKS);
    assert!(poll(&mut wait, &waker).is_pending());
    drop(wait);
}
