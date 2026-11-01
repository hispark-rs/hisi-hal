//! Shared, bounded UART receive polling. No executor, queue or PAC access.
#[cfg(test)]
use core::cell::Cell;
use core::task::{Poll, Waker};

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
