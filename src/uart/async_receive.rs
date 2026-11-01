//! Shared, bounded UART receive polling. No executor, queue or PAC access.
#[cfg(test)]
use core::cell::Cell;
use core::task::{Context, Poll, Waker};

// Metadata only: no MMIO, callbacks, Waker cloning or destruction under CS.
pub(super) trait WakerSlot {
    fn replace(&self, waker: Option<Waker>) -> Option<Waker>;
    fn take(&self) -> Option<Waker> {
        self.replace(None)
    }
}
#[cfg(test)]
impl WakerSlot for Cell<Option<Waker>> {
    fn replace(&self, waker: Option<Waker>) -> Option<Waker> {
        Cell::replace(self, waker)
    }
}

pub(super) trait RxIo {
    type Error;
    fn read(&mut self) -> nb::Result<u8, Self::Error>;
    fn arm(&mut self);
    fn disarm(&mut self);
}
// Only the slot methods hold a short metadata CS. Register before checking,
// then check again after arm: bytes/errors arriving at either boundary cannot
// leave a pending future without a registered wake or retained hardware source.
// Old/completed wakers are returned for destruction OUTSIDE the critical section.
type PollOutcome<E> = (Poll<Result<u8, E>>, Option<Waker>, Option<Waker>);
pub(super) fn poll_rx<I: RxIo>(io: &mut I, slot: &impl WakerSlot, waker: Waker) -> PollOutcome<I::Error> {
    let old = slot.replace(Some(waker));
    io.disarm();
    let mut result = io.read();
    if matches!(result, Err(nb::Error::WouldBlock)) {
        io.arm();
        result = io.read();
    }
    match result {
        Err(nb::Error::WouldBlock) => (Poll::Pending, old, None),
        result => {
            io.disarm();
            let result = match result {
                Ok(byte) => Ok(byte),
                Err(nb::Error::Other(error)) => Err(error),
                Err(nb::Error::WouldBlock) => unreachable!(),
            };
            (Poll::Ready(result), old, slot.take())
        }
    }
}

/// Own the one outstanding receive operation, including cancellation cleanup.
/// No byte is consumed on Pending. Cancellation masks notification, releases
/// the waker and leaves the FIFO untouched. The I/O backend retains ownership
/// of its UART borrow until this guard is dropped.
pub(super) struct ReceiveWait<I: RxIo, S: WakerSlot> {
    io: I,
    slot: S,
    started: bool,
}
impl<I: RxIo, S: WakerSlot> ReceiveWait<I, S> {
    pub(super) fn new(io: I, slot: S) -> Self {
        Self { io, slot, started: false }
    }
    pub(super) fn poll(&mut self, cx: &mut Context<'_>) -> Poll<Result<u8, I::Error>> {
        self.started = true;
        let (result, old, completed) = poll_rx(&mut self.io, &self.slot, cx.waker().clone());
        drop((old, completed));
        result
    }
}
impl<I: RxIo, S: WakerSlot> Drop for ReceiveWait<I, S> {
    fn drop(&mut self) {
        if self.started {
            self.io.disarm();
            drop(self.slot.take());
        }
    }
}

#[cfg(all(test, not(target_arch = "riscv32")))]
mod tests {
    use super::*;
    use std::{
        collections::VecDeque,
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
    struct Io {
        reads: VecDeque<nb::Result<u8, u8>>,
        armed: bool,
    }
    impl RxIo for Io {
        type Error = u8;
        fn read(&mut self) -> nb::Result<u8, u8> {
            self.reads.pop_front().unwrap_or(Err(nb::Error::WouldBlock))
        }
        fn arm(&mut self) {
            self.armed = true;
        }
        fn disarm(&mut self) {
            self.armed = false;
        }
    }
    #[test]
    fn arrival_before_check_or_during_arm_completes_without_parked_waker() {
        for reads in [vec![Ok(0x53)], vec![Err(nb::Error::WouldBlock), Ok(0x53)]] {
            let mut io = Io { reads: reads.into(), armed: false };
            let slot = Cell::new(None);
            let count = Arc::new(Counter(AtomicUsize::new(0)));
            let (result, old, done) = poll_rx(&mut io, &slot, Waker::from(count.clone()));
            assert_eq!(result, Poll::Ready(Ok(0x53)));
            assert!(!io.armed);
            assert!(slot.take().is_none());
            drop((old, done));
            assert_eq!(Arc::strong_count(&count), 1);
        }
    }
    #[test]
    fn error_before_or_during_arm_is_reported_and_following_byte_preserved() {
        for mut reads in [vec![], vec![Err(nb::Error::WouldBlock)]] {
            reads.extend([Err(nb::Error::Other(7)), Ok(0x55)]);
            let mut io = Io { reads: reads.into(), armed: false };
            let slot = Cell::new(None);
            let waker = Waker::noop();
            assert_eq!(poll_rx(&mut io, &slot, waker.clone()).0, Poll::Ready(Err(7)));
            assert!(!io.armed);
            assert!(slot.take().is_none());
            assert_eq!(poll_rx(&mut io, &slot, waker.clone()).0, Poll::Ready(Ok(0x55)));
        }
    }
    #[test]
    fn pending_repoll_replaces_waker_and_notification_wakes_latest_task() {
        let mut io = Io { reads: VecDeque::new(), armed: false };
        let slot = Cell::new(None);
        let first = Arc::new(Counter(AtomicUsize::new(0)));
        let last = Arc::new(Counter(AtomicUsize::new(0)));
        assert_eq!(poll_rx(&mut io, &slot, Waker::from(first.clone())).0, Poll::Pending);
        assert!(io.armed);
        let (result, old, done) = poll_rx(&mut io, &slot, Waker::from(last.clone()));
        assert_eq!(result, Poll::Pending);
        drop((old, done));
        assert_eq!(Arc::strong_count(&first), 1);
        io.disarm();
        slot.take().unwrap().wake();
        assert_eq!(first.0.load(Ordering::SeqCst), 0);
        assert_eq!(last.0.load(Ordering::SeqCst), 1);
        io.reads.push_back(Ok(0xa5));
        assert_eq!(poll_rx(&mut io, &slot, Waker::from(last.clone())).0, Poll::Ready(Ok(0xa5)));
        assert!(!io.armed);
        assert_eq!(Arc::strong_count(&last), 1);
    }

    #[test]
    fn interrupt_during_disarm_schedules_repoll_before_pending_return() {
        struct Race<'a> {
            slot: &'a Cell<Option<Waker>>,
            interrupt_once: bool,
        }
        impl RxIo for Race<'_> {
            type Error = u8;
            fn read(&mut self) -> nb::Result<u8, u8> {
                Err(nb::Error::WouldBlock)
            }
            fn arm(&mut self) {}
            fn disarm(&mut self) {
                if core::mem::take(&mut self.interrupt_once) {
                    // Model the ISR taking and waking the newly registered
                    // waker before the pending poll returns.
                    self.slot.take().unwrap().wake();
                }
            }
        }
        let slot = Cell::new(None);
        let count = Arc::new(Counter(AtomicUsize::new(0)));
        let mut io = Race { slot: &slot, interrupt_once: true };
        assert!(poll_rx(&mut io, &slot, Waker::from(count.clone())).0.is_pending());
        assert_eq!(count.0.load(Ordering::SeqCst), 1);
        assert!(slot.take().is_none());
        // The wake schedules a later poll; that poll installs a fresh waker.
        assert!(poll_rx(&mut io, &slot, Waker::from(count.clone())).0.is_pending());
        slot.take().unwrap().wake();
        assert_eq!(count.0.load(Ordering::SeqCst), 2);
    }
}

#[cfg(all(test, not(target_arch = "riscv32")))]
mod lifecycle_tests {
    use super::*;
    use core::cell::{Cell, RefCell};
    use std::{collections::VecDeque, sync::Arc, task::Wake};
    // Unlike Waker::noop this has an observable reference count, and a stale
    // notification after cancellation must fail the test rather than disappear.
    struct CancelledWaiter;
    impl Wake for CancelledWaiter {
        fn wake(self: Arc<Self>) {
            panic!("cancelled UART waiter was woken");
        }
    }
    impl WakerSlot for &Cell<Option<Waker>> {
        fn replace(&self, waker: Option<Waker>) -> Option<Waker> {
            Cell::replace(self, waker)
        }
    }
    #[derive(Default)]
    struct Fifo {
        bytes: RefCell<VecDeque<u8>>,
        armed: Cell<bool>,
        masks: Cell<usize>,
    }
    impl RxIo for &Fifo {
        type Error = core::convert::Infallible;
        fn read(&mut self) -> nb::Result<u8, Self::Error> {
            self.bytes.borrow_mut().pop_front().ok_or(nb::Error::WouldBlock)
        }
        fn arm(&mut self) {
            self.armed.set(true);
        }
        fn disarm(&mut self) {
            self.armed.set(false);
            self.masks.set(self.masks.get() + 1);
        }
    }
    fn notify(fifo: &Fifo, slot: &Cell<Option<Waker>>) {
        (&*fifo).disarm();
        if let Some(waker) = slot.take() {
            waker.wake();
        }
    }
    #[test]
    fn unpolled_drop_does_not_touch_backend_or_an_existing_waker() {
        let fifo = Fifo::default();
        let slot = Cell::new(Some(Waker::noop().clone()));
        drop(ReceiveWait::new(&fifo, &slot));
        assert_eq!(fifo.masks.get(), 0);
        assert!(slot.take().is_some());
    }
    #[test]
    fn cancellation_releases_waiter_and_preserves_arrival_for_next_reader() {
        let fifo = Fifo::default();
        let slot = Cell::new(None);
        let owner = Arc::new(CancelledWaiter);
        let waker = Waker::from(owner.clone());
        let mut cx = Context::from_waker(&waker);
        let mut wait = ReceiveWait::new(&fifo, &slot);
        assert!(wait.poll(&mut cx).is_pending());
        assert!(fifo.armed.get());
        assert_eq!(Arc::strong_count(&owner), 3);
        fifo.bytes.borrow_mut().push_back(0x53);
        drop(wait);
        assert!(!fifo.armed.get());
        assert!(slot.take().is_none());
        assert_eq!(Arc::strong_count(&owner), 2);
        // A late notification cannot consume the retained byte.
        notify(&fifo, &slot);
        let mut next = ReceiveWait::new(&fifo, &slot);
        assert_eq!(next.poll(&mut cx), Poll::Ready(Ok(0x53)));
        assert!(!fifo.armed.get());
    }
    #[test]
    fn coalesced_and_duplicate_notifications_never_overwrite_unread_bytes() {
        let fifo = Fifo::default();
        let slot = Cell::new(None);
        let mut cx = Context::from_waker(Waker::noop());
        let mut wait = ReceiveWait::new(&fifo, &slot);
        assert!(wait.poll(&mut cx).is_pending());
        fifo.bytes.borrow_mut().extend([0x11, 0x22, 0x33]);
        notify(&fifo, &slot);
        notify(&fifo, &slot);
        assert_eq!(fifo.bytes.borrow().len(), 3);
        assert_eq!(wait.poll(&mut cx), Poll::Ready(Ok(0x11)));
        drop(wait);
        for byte in [0x22, 0x33] {
            let mut next = ReceiveWait::new(&fifo, &slot);
            assert_eq!(next.poll(&mut cx), Poll::Ready(Ok(byte)));
        }
        assert!(fifo.bytes.borrow().is_empty());
    }
    #[test]
    fn empty_notification_rearms_without_returning_a_fabricated_byte() {
        let fifo = Fifo::default();
        let slot = Cell::new(None);
        let mut cx = Context::from_waker(Waker::noop());
        let mut wait = ReceiveWait::new(&fifo, &slot);
        assert!(wait.poll(&mut cx).is_pending());
        notify(&fifo, &slot);
        assert!(wait.poll(&mut cx).is_pending());
        assert!(fifo.armed.get());
        assert!(slot.take().is_some());
    }
}
