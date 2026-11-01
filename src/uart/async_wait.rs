//! Shared, bounded UART operation polling. No executor, queue or PAC access.
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

pub(super) trait WaitIo {
    type Output;
    type Error;
    /// One bounded attempt. WouldBlock must not accept or consume data.
    /// For flush, readiness must include the backend's line-completion check.
    fn try_complete(&mut self) -> nb::Result<Self::Output, Self::Error>;
    /// Enable notification without resetting FIFO contents or changing ownership.
    fn arm(&mut self);
    /// Mask/ack notification only; do not discard accepted or pending data.
    fn disarm(&mut self);
}
// Only the slot methods hold a short metadata CS. Register before checking,
// then check again after arm: bytes/errors arriving at either boundary cannot
// leave a pending future without a registered wake or retained hardware source.
// Old/completed wakers are returned for destruction OUTSIDE the critical section.
type PollOutcome<O, E> = (Poll<Result<O, E>>, Option<Waker>, Option<Waker>);
pub(super) fn poll_io<I: WaitIo>(io: &mut I, slot: &impl WakerSlot, waker: Waker) -> PollOutcome<I::Output, I::Error> {
    let old = slot.replace(Some(waker));
    io.disarm();
    let mut result = io.try_complete();
    if matches!(result, Err(nb::Error::WouldBlock)) {
        io.arm();
        result = io.try_complete();
    }
    match result {
        Err(nb::Error::WouldBlock) => (Poll::Pending, old, None),
        result => {
            io.disarm();
            let result = match result {
                Ok(value) => Ok(value),
                Err(nb::Error::Other(error)) => Err(error),
                Err(nb::Error::WouldBlock) => unreachable!(),
            };
            (Poll::Ready(result), old, slot.take())
        }
    }
}

/// Own one outstanding operation, including cancellation cleanup.
/// The backend must report completion immediately after consuming/accepting data;
/// WouldBlock must have no unreported data transfer. Cancellation disarms only
/// notification, preserving FIFO contents and already accepted TX bytes.
/// The backend retains its UART borrow until this guard is dropped.
pub(super) struct Wait<I: WaitIo, S: WakerSlot> {
    io: I,
    slot: S,
    started: bool,
}
impl<I: WaitIo, S: WakerSlot> Wait<I, S> {
    pub(super) fn new(io: I, slot: S) -> Self {
        Self { io, slot, started: false }
    }
    pub(super) fn poll(&mut self, cx: &mut Context<'_>) -> Poll<Result<I::Output, I::Error>> {
        self.started = true;
        let (result, old, completed) = poll_io(&mut self.io, &self.slot, cx.waker().clone());
        drop((old, completed));
        result
    }
}
impl<I: WaitIo, S: WakerSlot> Drop for Wait<I, S> {
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
    impl WaitIo for Io {
        type Output = u8;
        type Error = u8;
        fn try_complete(&mut self) -> nb::Result<u8, u8> {
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
            let (result, old, done) = poll_io(&mut io, &slot, Waker::from(count.clone()));
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
            assert_eq!(poll_io(&mut io, &slot, waker.clone()).0, Poll::Ready(Err(7)));
            assert!(!io.armed);
            assert!(slot.take().is_none());
            assert_eq!(poll_io(&mut io, &slot, waker.clone()).0, Poll::Ready(Ok(0x55)));
        }
    }
    #[test]
    fn pending_repoll_replaces_waker_and_notification_wakes_latest_task() {
        let mut io = Io { reads: VecDeque::new(), armed: false };
        let slot = Cell::new(None);
        let first = Arc::new(Counter(AtomicUsize::new(0)));
        let last = Arc::new(Counter(AtomicUsize::new(0)));
        assert_eq!(poll_io(&mut io, &slot, Waker::from(first.clone())).0, Poll::Pending);
        assert!(io.armed);
        let (result, old, done) = poll_io(&mut io, &slot, Waker::from(last.clone()));
        assert_eq!(result, Poll::Pending);
        drop((old, done));
        assert_eq!(Arc::strong_count(&first), 1);
        io.disarm();
        slot.take().unwrap().wake();
        assert_eq!(first.0.load(Ordering::SeqCst), 0);
        assert_eq!(last.0.load(Ordering::SeqCst), 1);
        io.reads.push_back(Ok(0xa5));
        assert_eq!(poll_io(&mut io, &slot, Waker::from(last.clone())).0, Poll::Ready(Ok(0xa5)));
        assert!(!io.armed);
        assert_eq!(Arc::strong_count(&last), 1);
    }

    #[test]
    fn interrupt_during_disarm_schedules_repoll_before_pending_return() {
        struct Race<'a> {
            slot: &'a Cell<Option<Waker>>,
            interrupt_once: bool,
        }
        impl WaitIo for Race<'_> {
            type Output = u8;
            type Error = u8;
            fn try_complete(&mut self) -> nb::Result<u8, u8> {
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
        assert!(poll_io(&mut io, &slot, Waker::from(count.clone())).0.is_pending());
        assert_eq!(count.0.load(Ordering::SeqCst), 1);
        assert!(slot.take().is_none());
        // The wake schedules a later poll; that poll installs a fresh waker.
        assert!(poll_io(&mut io, &slot, Waker::from(count.clone())).0.is_pending());
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
    impl WaitIo for &Fifo {
        type Output = u8;
        type Error = core::convert::Infallible;
        fn try_complete(&mut self) -> nb::Result<u8, Self::Error> {
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
        drop(Wait::new(&fifo, &slot));
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
        let mut wait = Wait::new(&fifo, &slot);
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
        let mut next = Wait::new(&fifo, &slot);
        assert_eq!(next.poll(&mut cx), Poll::Ready(Ok(0x53)));
        assert!(!fifo.armed.get());
    }
    #[test]
    fn coalesced_and_duplicate_notifications_never_overwrite_unread_bytes() {
        let fifo = Fifo::default();
        let slot = Cell::new(None);
        let mut cx = Context::from_waker(Waker::noop());
        let mut wait = Wait::new(&fifo, &slot);
        assert!(wait.poll(&mut cx).is_pending());
        fifo.bytes.borrow_mut().extend([0x11, 0x22, 0x33]);
        notify(&fifo, &slot);
        notify(&fifo, &slot);
        assert_eq!(fifo.bytes.borrow().len(), 3);
        assert_eq!(wait.poll(&mut cx), Poll::Ready(Ok(0x11)));
        drop(wait);
        for byte in [0x22, 0x33] {
            let mut next = Wait::new(&fifo, &slot);
            assert_eq!(next.poll(&mut cx), Poll::Ready(Ok(byte)));
        }
        assert!(fifo.bytes.borrow().is_empty());
    }
    #[test]
    fn empty_notification_rearms_without_returning_a_fabricated_byte() {
        let fifo = Fifo::default();
        let slot = Cell::new(None);
        let mut cx = Context::from_waker(Waker::noop());
        let mut wait = Wait::new(&fifo, &slot);
        assert!(wait.poll(&mut cx).is_pending());
        notify(&fifo, &slot);
        assert!(wait.poll(&mut cx).is_pending());
        assert!(fifo.armed.get());
        assert!(slot.take().is_some());
    }
}

#[cfg(all(test, not(target_arch = "riscv32")))]
mod tx_tests {
    use super::*;
    use core::task::Poll;
    use std::{
        collections::VecDeque,
        sync::{
            Arc,
            atomic::{AtomicUsize, Ordering},
        },
        task::Wake,
    };
    #[derive(Clone, Copy)]
    enum TxOperation {
        Write(u8),
        Flush,
    }
    struct Count(AtomicUsize);
    impl Wake for Count {
        fn wake(self: Arc<Self>) {
            self.0.fetch_add(1, Ordering::SeqCst);
        }
    }
    struct Io {
        ready: VecDeque<bool>,
        writes: std::vec::Vec<u8>,
        armed: bool,
        operation: TxOperation,
    }
    impl WaitIo for Io {
        type Output = ();
        type Error = core::convert::Infallible;
        fn try_complete(&mut self) -> nb::Result<(), Self::Error> {
            let ready = self.ready.pop_front().unwrap_or(false);
            if ready && let TxOperation::Write(byte) = self.operation {
                self.writes.push(byte);
            }
            if ready { Ok(()) } else { Err(nb::Error::WouldBlock) }
        }
        fn arm(&mut self) {
            self.armed = true;
        }
        fn disarm(&mut self) {
            self.armed = false;
        }
    }
    #[test]
    fn room_before_check_or_during_arm_writes_exactly_one_byte() {
        for ready in [std::vec![true], std::vec![false, true]] {
            let mut io =
                Io { ready: ready.into(), writes: std::vec![], armed: false, operation: TxOperation::Write(0xa5) };
            let slot = Cell::new(None);
            let (result, old, done) = poll_io(&mut io, &slot, Waker::noop().clone());
            drop((old, done));
            assert_eq!(result, Poll::Ready(Ok(())));
            assert_eq!(io.writes, [0xa5]);
            assert!(!io.armed);
            assert!(slot.take().is_none());
        }
    }
    #[test]
    fn pending_repoll_has_no_unreported_write_and_replaces_waker() {
        let mut io = Io { ready: VecDeque::new(), writes: std::vec![], armed: false, operation: TxOperation::Write(1) };
        let slot = Cell::new(None);
        let first = Arc::new(Count(AtomicUsize::new(0)));
        let last = Arc::new(Count(AtomicUsize::new(0)));
        assert_eq!(poll_io(&mut io, &slot, Waker::from(first.clone())).0, Poll::Pending);
        let (result, old, done) = poll_io(&mut io, &slot, Waker::from(last.clone()));
        drop((old, done));
        assert_eq!(result, Poll::Pending);
        assert!(io.writes.is_empty());
        assert_eq!(Arc::strong_count(&first), 1);
        io.disarm();
        slot.take().unwrap().wake();
        assert_eq!(last.0.load(Ordering::SeqCst), 1);
        io.ready.push_back(true);
        assert_eq!(poll_io(&mut io, &slot, Waker::from(last.clone())).0, Poll::Ready(Ok(())));
        assert_eq!(io.writes, [1]);
        assert_eq!(Arc::strong_count(&last), 1);
    }

    #[test]
    fn interrupt_during_disarm_schedules_repoll_without_writing() {
        struct Race<'a> {
            slot: &'a Cell<Option<Waker>>,
            interrupt_once: bool,
        }
        impl WaitIo for Race<'_> {
            type Output = ();
            type Error = core::convert::Infallible;
            fn try_complete(&mut self) -> nb::Result<(), Self::Error> {
                Err(nb::Error::WouldBlock)
            }
            fn arm(&mut self) {}
            fn disarm(&mut self) {
                if core::mem::take(&mut self.interrupt_once) {
                    self.slot.take().unwrap().wake();
                }
            }
        }
        let slot = Cell::new(None);
        let count = Arc::new(Count(AtomicUsize::new(0)));
        let mut io = Race { slot: &slot, interrupt_once: true };
        assert!(poll_io(&mut io, &slot, Waker::from(count.clone())).0.is_pending());
        assert_eq!(count.0.load(Ordering::SeqCst), 1);
        assert!(slot.take().is_none());
        assert!(poll_io(&mut io, &slot, Waker::from(count.clone())).0.is_pending());
        slot.take().unwrap().wake();
        assert_eq!(count.0.load(Ordering::SeqCst), 2);
    }
    #[test]
    fn fifo_notification_does_not_complete_flush_until_serializer_is_empty() {
        let mut io = Io { ready: VecDeque::new(), writes: std::vec![], armed: false, operation: TxOperation::Flush };
        let slot = Cell::new(None);
        for _ in 0..3 {
            assert_eq!(poll_io(&mut io, &slot, Waker::noop().clone()).0, Poll::Pending);
            io.disarm();
            slot.take().unwrap().wake();
        }
        io.ready.push_back(true);
        assert_eq!(poll_io(&mut io, &slot, Waker::noop().clone()).0, Poll::Ready(Ok(())));
        assert!(io.writes.is_empty());
        assert!(!io.armed);
        assert!(slot.take().is_none());
    }

    impl WaitIo for &mut Io {
        type Output = ();
        type Error = core::convert::Infallible;
        fn try_complete(&mut self) -> nb::Result<(), Self::Error> {
            (**self).try_complete()
        }
        fn arm(&mut self) {
            (**self).arm();
        }
        fn disarm(&mut self) {
            (**self).disarm();
        }
    }

    #[test]
    fn cancelled_write_emits_nothing_and_cancelled_flush_preserves_accepted_data() {
        let mut io =
            Io { ready: VecDeque::new(), writes: std::vec![], armed: false, operation: TxOperation::Write(0xa5) };
        let slot = Cell::new(None);
        let count = Arc::new(Count(AtomicUsize::new(0)));
        let waker = Waker::from(count.clone());
        let mut cx = Context::from_waker(&waker);
        let mut pending = Wait::new(&mut io, &slot);
        assert!(pending.poll(&mut cx).is_pending());
        assert_eq!(Arc::strong_count(&count), 3);
        drop(pending);
        assert!(io.writes.is_empty());
        assert!(!io.armed);
        assert!(slot.take().is_none());
        assert_eq!(Arc::strong_count(&count), 2);

        io.ready.push_back(true);
        let mut accepted = Wait::new(&mut io, &slot);
        assert_eq!(accepted.poll(&mut cx), Poll::Ready(Ok(())));
        drop(accepted);
        assert_eq!(io.writes, [0xa5]);
        io.operation = TxOperation::Flush;
        let mut flushing = Wait::new(&mut io, &slot);
        assert!(flushing.poll(&mut cx).is_pending());
        drop(flushing);
        assert_eq!(io.writes, [0xa5]);
        assert!(!io.armed);
        assert!(slot.take().is_none());
        assert_eq!(Arc::strong_count(&count), 2);
        assert_eq!(count.0.load(Ordering::SeqCst), 0);
    }
}
