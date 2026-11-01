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

#[derive(Default)]
struct StateIo {
    armed: Cell<bool>,
    paused: Cell<bool>,
    configure: Cell<usize>,
    stops: Cell<usize>,
    level: Cell<bool>,
    immediate_irq: Cell<bool>,
    check_lock: Cell<bool>,
}
fn unlocked() {
    let (tx, rx) = mpsc::channel();
    let worker = std::thread::spawn(move || critical_section::with(|_| tx.send(()).unwrap()));
    rx.recv_timeout(Duration::from_secs(2)).expect("operation ran inside metadata lock");
    worker.join().unwrap();
}
struct Io<'a> {
    state: Rc<StateIo>,
    signal: &'a Signal<u8>,
    line: u8,
}
impl Io<'_> {
    fn check(&self) {
        if self.state.check_lock.get() {
            unlocked();
        }
    }
}
impl WaitIo for Io<'_> {
    type Line = u8;
    fn line(&self) -> u8 {
        self.line
    }
    fn pause(&mut self) {
        self.check();
        self.state.paused.set(true);
    }
    fn configure(&mut self) {
        self.check();
        assert!(self.state.paused.get());
        self.state.configure.set(self.state.configure.get() + 1);
        self.state.armed.set(true);
    }
    fn resume(&mut self) {
        self.check();
        self.state.paused.set(false);
        if self.state.immediate_irq.replace(false) {
            self.state.armed.set(false); // backend stopped the observed source
            self.signal.notify(self.line);
        }
    }
    fn stop(&mut self) {
        self.check();
        self.state.armed.set(false);
        self.state.stops.set(self.state.stops.get() + 1);
    }
    fn level_matches(&mut self) -> bool {
        self.check();
        self.state.level.get()
    }
}
#[derive(Default)]
struct Counter(AtomicUsize);
impl Wake for Counter {
    fn wake(self: Arc<Self>) {
        self.0.fetch_add(1, Ordering::SeqCst);
    }
}
fn io<'a>(signal: &'a Signal<u8>, state: &Rc<StateIo>, line: u8) -> Io<'a> {
    Io { signal, state: state.clone(), line }
}
fn poll<B: WaitIo + Unpin>(wait: &mut Wait<'_, B>, waker: &Waker) -> Poll<()> {
    Pin::new(wait).poll(&mut Context::from_waker(waker))
}

#[test]
fn unpolled_drop_never_touches_hardware() {
    let signal = Signal::new();
    let state = Rc::new(StateIo::default());
    drop(Wait::new(io(&signal, &state, 1), &signal));
    assert_eq!(state.configure.get(), 0);
    assert_eq!(state.stops.get(), 0);
}
#[test]
fn cancellation_retires_source_and_waker_and_next_wait_starts_fresh() {
    let signal = Signal::new();
    let state = Rc::new(StateIo::default());
    let count = Arc::new(Counter::default());
    let waker = Waker::from(count.clone());
    let mut wait = Wait::new(io(&signal, &state, 1), &signal);
    assert!(poll(&mut wait, &waker).is_pending());
    drop(wait);
    assert!(!state.armed.get());
    assert_eq!(state.stops.get(), 1);
    assert_eq!(Arc::strong_count(&count), 2);
    assert!(!signal.notify(1));
    assert_eq!(count.0.load(Ordering::SeqCst), 0);
    let mut next = Wait::new(io(&signal, &state, 1), &signal);
    assert!(poll(&mut next, &waker).is_pending());
    assert_eq!(state.configure.get(), 2);
}
#[test]
fn unrelated_and_duplicate_events_do_not_complete_or_wake() {
    let signal = Signal::new();
    let state = Rc::new(StateIo::default());
    let count = Arc::new(Counter::default());
    let waker = Waker::from(count.clone());
    let mut wait = Wait::new(io(&signal, &state, 3), &signal);
    assert!(poll(&mut wait, &waker).is_pending());
    assert!(!signal.notify(4));
    assert!(poll(&mut wait, &waker).is_pending());
    assert!(signal.notify(3));
    assert!(!signal.notify(3));
    assert_eq!(count.0.load(Ordering::SeqCst), 1);
    assert!(poll(&mut wait, &waker).is_ready());
}
#[test]
fn irq_on_enable_is_latched_and_wakes_before_pending_returns() {
    let signal = Signal::new();
    let state = Rc::new(StateIo::default());
    state.immediate_irq.set(true);
    let count = Arc::new(Counter::default());
    let waker = Waker::from(count.clone());
    let mut wait = Wait::new(io(&signal, &state, 1), &signal);
    assert!(poll(&mut wait, &waker).is_pending());
    assert_eq!(count.0.load(Ordering::SeqCst), 1);
    assert!(poll(&mut wait, &waker).is_ready());
    assert_eq!(state.configure.get(), 1);
}
#[test]
fn asserted_level_completes_without_enabling_delivery() {
    let signal = Signal::new();
    let state = Rc::new(StateIo::default());
    state.level.set(true);
    let mut wait = Wait::new(io(&signal, &state, 1), &signal);
    assert!(poll(&mut wait, Waker::noop()).is_ready());
    assert!(state.paused.get());
    assert!(!state.armed.get());
    assert_eq!(signal.active_line(), None);
}
#[test]
fn distinct_slots_and_replaced_waiters_remain_independent() {
    let a = Signal::new();
    let b = Signal::new();
    let sa = Rc::new(StateIo::default());
    let sb = Rc::new(StateIo::default());
    let ca = Arc::new(Counter::default());
    let cb = Arc::new(Counter::default());
    let wa = Waker::from(ca.clone());
    let wb = Waker::from(cb.clone());
    let mut fa = Wait::new(io(&a, &sa, 1), &a);
    let mut fb = Wait::new(io(&b, &sb, 2), &b);
    assert!(poll(&mut fa, &wa).is_pending());
    assert!(poll(&mut fb, &wb).is_pending());
    assert!(poll(&mut fa, &wb).is_pending());
    assert_eq!(Arc::strong_count(&ca), 2);
    a.notify(1);
    assert_eq!(ca.0.load(Ordering::SeqCst), 0);
    assert_eq!(cb.0.load(Ordering::SeqCst), 1);
    assert!(poll(&mut fb, &wb).is_pending());
    drop(fa);
    assert!(sb.armed.get());
}
static LOCK_CHECK: core::task::RawWakerVTable = core::task::RawWakerVTable::new(
    |_| {
        unlocked();
        core::task::RawWaker::new(core::ptr::null(), &LOCK_CHECK)
    },
    |_| unlocked(),
    |_| unlocked(),
    |_| unlocked(),
);
#[test]
fn io_clone_replacement_drop_and_wake_all_run_outside_metadata_lock() {
    // SAFETY: callbacks ignore the null data, own nothing and clone the same static vtable.
    let waker = unsafe { Waker::from_raw(core::task::RawWaker::new(core::ptr::null(), &LOCK_CHECK)) };
    let signal = Signal::new();
    let state = Rc::new(StateIo::default());
    state.check_lock.set(true);
    let mut wait = Wait::new(io(&signal, &state, 1), &signal);
    assert!(poll(&mut wait, &waker).is_pending());
    assert!(poll(&mut wait, &waker).is_pending());
    signal.notify(1);
    assert!(poll(&mut wait, &waker).is_ready());
    drop(wait);
    let mut cancelled = Wait::new(io(&signal, &state, 1), &signal);
    assert!(poll(&mut cancelled, &waker).is_pending());
    drop(cancelled);
}
