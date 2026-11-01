//! Software rendezvous tests. These do not model peripheral acknowledgement,
//! DMA cancellation, cross-core ownership, or RISC-V WFI timing.
use super::*;
use std::sync::{
    Arc,
    atomic::{AtomicUsize, Ordering as StdOrdering},
    mpsc,
};
use std::task::Wake;
use std::time::Duration;

#[derive(Default)]
struct Counter(AtomicUsize);
impl Wake for Counter {
    fn wake(self: Arc<Self>) {
        self.0.fetch_add(1, StdOrdering::SeqCst);
    }
}
fn waiter() -> (Arc<Counter>, Waker) {
    let count = Arc::new(Counter::default());
    (count.clone(), Waker::from(count))
}

#[test]
fn event_between_check_and_registration_wakes_waiter() {
    let signal = IrqSignal::new();
    let (count, waker) = waiter();
    assert!(!signal.take_fired());
    signal.signal(); // ISR in the old check/register gap.
    signal.register(&waker);
    assert_eq!(count.0.load(StdOrdering::SeqCst), 1);
    assert!(signal.take_fired()); // Registration must not consume completion.
    assert!(!signal.take_fired());
}

#[test]
fn registered_waiter_is_woken_and_events_coalesce() {
    let signal = IrqSignal::new();
    let (count, waker) = waiter();
    signal.register(&waker);
    signal.signal();
    signal.signal();
    assert_eq!(count.0.load(StdOrdering::SeqCst), 1);
    assert!(signal.take_fired());
    assert!(!signal.take_fired());
}

#[test]
fn replacement_and_reset_do_not_wake_retired_waiters() {
    let signal = IrqSignal::new();
    let (first, a) = waiter();
    let (second, b) = waiter();
    signal.register(&a);
    signal.register(&b);
    signal.signal();
    assert_eq!(first.0.load(StdOrdering::SeqCst), 0);
    assert_eq!(second.0.load(StdOrdering::SeqCst), 1);
    signal.reset();
    assert!(!signal.take_fired());
    signal.register(&a);
    signal.reset();
    signal.signal();
    assert_eq!(first.0.load(StdOrdering::SeqCst), 0);
    assert!(signal.take_fired());
}

// A callback must allow another thread to acquire the host critical-section
// lock. Merely reentering on the same thread would miss nested-CS violations.
fn assert_metadata_lock_released() {
    let (tx, rx) = mpsc::channel();
    let worker = std::thread::spawn(move || {
        critical_section::with(|_| tx.send(()).ok());
    });
    // On failure, detach rather than join while the calling lock is still held.
    // Unwinding releases that lock and lets the worker exit.
    rx.recv_timeout(Duration::from_secs(2)).expect("Waker callback ran under metadata lock");
    worker.join().unwrap();
}

static CHECK_LOCK: RawWakerVTable = RawWakerVTable::new(
    |_| {
        assert_metadata_lock_released();
        RawWaker::new(core::ptr::null(), &CHECK_LOCK)
    },
    |_| assert_metadata_lock_released(),
    |_| assert_metadata_lock_released(),
    |_| assert_metadata_lock_released(),
);

#[test]
fn clone_replace_drop_reset_and_wake_callbacks_are_outside_lock() {
    // SAFETY: every callback ignores the null data and returns the same static
    // vtable. No allocation, borrowed data, or reference count is involved.
    let waker = unsafe { Waker::from_raw(RawWaker::new(core::ptr::null(), &CHECK_LOCK)) };
    let signal = IrqSignal::new();
    signal.register(&waker); // clone
    signal.register(&waker); // clone + drop old
    signal.signal(); // wake parked
    signal.register(&waker); // clone + wake already-fired
    signal.reset();
    signal.register(&waker);
    signal.reset(); // drop parked
}

struct Reenter(Arc<IrqSignal>);
impl Wake for Reenter {
    fn wake(self: Arc<Self>) {
        self.0.reset();
        self.0.register(Waker::noop());
    }
}

#[test]
fn wake_can_reenter_and_register_next_waiter() {
    let signal = Arc::new(IrqSignal::new());
    let waker = Waker::from(Arc::new(Reenter(signal.clone())));
    signal.register(&waker);
    signal.signal();
    assert!(!signal.take_fired());
    signal.signal();
    assert!(signal.take_fired());
}

// Trigger an ISR while cloning the Waker: this is a deterministic injection
// into poll_wait's registration window, not a probabilistic scheduling test.
unsafe fn clone_and_signal(data: *const ()) -> RawWaker {
    // SAFETY: test owns the Arc for every live RawWaker reference; clone creates
    // one additional strong reference, wake/drop below each consume exactly one.
    let signal = core::mem::ManuallyDrop::new(unsafe { Arc::from_raw(data.cast::<IrqSignal>()) });
    signal.signal();
    RawWaker::new(Arc::into_raw(Arc::clone(&signal)).cast(), &INJECT)
}
unsafe fn drop_signal(data: *const ()) {
    // SAFETY: consumes the one strong reference owned by this RawWaker.
    drop(unsafe { Arc::from_raw(data.cast::<IrqSignal>()) });
}
static INJECT: RawWakerVTable = RawWakerVTable::new(clone_and_signal, drop_signal, |_| {}, drop_signal);

#[test]
fn event_during_waker_clone_is_not_lost() {
    let signal = Arc::new(IrqSignal::new());
    // SAFETY: INJECT preserves Arc ownership as documented above.
    let waker = unsafe { Waker::from_raw(RawWaker::new(Arc::into_raw(signal.clone()).cast(), &INJECT)) };
    let mut cx = Context::from_waker(&waker);
    assert!(signal.poll_wait(&mut cx).is_pending());
    assert!(signal.poll_wait(&mut cx).is_ready());
}

#[test]
fn common_poll_parks_then_consumes_one_completion() {
    let signal = IrqSignal::new();
    let (count, waker) = waiter();
    let mut cx = Context::from_waker(&waker);
    assert!(signal.poll_wait(&mut cx).is_pending());
    signal.signal();
    assert_eq!(count.0.load(StdOrdering::SeqCst), 1);
    assert!(signal.poll_wait(&mut cx).is_ready());
    assert!(signal.poll_wait(&mut cx).is_pending());
}
