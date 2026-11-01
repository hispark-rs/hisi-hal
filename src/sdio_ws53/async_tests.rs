use super::super::tests::{event, new, running};
use super::*;
use std::sync::{
    Arc,
    atomic::{AtomicUsize, Ordering},
};
use std::task::Wake;

#[derive(Default)]
struct Counter(AtomicUsize);
impl Wake for Counter {
    fn wake(self: Arc<Self>) {
        self.0.fetch_add(1, Ordering::SeqCst);
    }
}
struct Irq<'a> {
    enabled: bool,
    arms: usize,
    slot: &'a Cell<Option<Waker>>,
    interrupt_on_arm: bool,
}
impl LocalIrq for Irq<'_> {
    fn disable(&mut self) {
        self.enabled = false;
    }
    fn enable(&mut self) {
        // An interrupt may run immediately on enable, before poll's recheck.
        assert!(
            self.slot
                .take()
                .map(|w| {
                    self.slot.set(Some(w));
                })
                .is_some()
        );
        self.enabled = true;
        self.arms += 1;
        if self.interrupt_on_arm {
            self.enabled = false;
            self.slot.take().unwrap().wake();
        }
    }
}
fn irq(slot: &Cell<Option<Waker>>) -> Irq<'_> {
    Irq { enabled: false, arms: 0, slot, interrupt_on_arm: false }
}
fn counter() -> (Arc<Counter>, Waker) {
    let count = Arc::new(Counter::default());
    (count.clone(), Waker::from(count))
}

#[test]
fn pending_replaces_waker_and_cancel_retains_without_publication() {
    let mut engine = new(Direction::HostWrite);
    engine.start().unwrap();
    let slot = Cell::new(None);
    let mut irq = irq(&slot);
    let (old, old_waker) = counter();
    assert_eq!(poll_event(&mut engine, &slot, &mut irq, old_waker), Poll::Pending);
    assert!(irq.enabled);
    let (next, next_waker) = counter();
    assert_eq!(poll_event(&mut engine, &slot, &mut irq, next_waker), Poll::Pending);
    assert_eq!(Arc::strong_count(&old), 1);
    assert_eq!(Arc::strong_count(&next), 2);
    cancel(&mut engine, true, false, &slot, &mut irq);
    assert!(!irq.enabled && slot.take().is_none());
    assert_eq!(Arc::strong_count(&next), 1);
    assert_eq!(engine.state, State::Retained);
    assert!(engine.io.published.is_empty());
    assert_eq!(engine.poll(), Err(TransferError::Retained));
}

#[test]
fn request_before_arm_and_during_arm_has_no_lost_event() {
    for during_arm in [false, true] {
        let mut engine = new(Direction::HostRead);
        engine.start().unwrap();
        if during_arm {
            engine.io.events.push_back(EventSnapshot::default());
        }
        engine.io.events.push_back(event(8));
        let slot = Cell::new(None);
        let mut irq = irq(&slot);
        irq.interrupt_on_arm = during_arm;
        let (count, waker) = counter();
        let expected = TransferPoll { progress: TransferProgress::Running, events: event(8) };
        assert_eq!(poll_event(&mut engine, &slot, &mut irq, waker), Poll::Ready(Ok(expected)));
        assert_eq!(irq.arms, usize::from(during_arm));
        assert_eq!(count.0.load(Ordering::SeqCst), usize::from(during_arm));
        assert!(!irq.enabled && slot.take().is_none());
        assert_eq!(engine.io.published, [0x20010000]);
        let trace = engine.io.trace.clone();
        cancel(&mut engine, true, true, &slot, &mut irq);
        assert_eq!(engine.state, State::Running);
        assert_eq!(engine.io.trace, trace);
    }
}

#[test]
fn spurious_immediate_interrupt_wakes_for_repoll() {
    let mut engine = new(Direction::HostWrite);
    engine.start().unwrap();
    let slot = Cell::new(None);
    let mut irq = irq(&slot);
    irq.interrupt_on_arm = true;
    let (count, waker) = counter();
    assert_eq!(poll_event(&mut engine, &slot, &mut irq, waker.clone()), Poll::Pending);
    assert_eq!(count.0.load(Ordering::SeqCst), 1);
    assert!(!irq.enabled && slot.take().is_none());
    irq.interrupt_on_arm = false;
    assert_eq!(poll_event(&mut engine, &slot, &mut irq, waker), Poll::Pending);
    assert!(irq.enabled && slot.take().is_some());
    assert_eq!(engine.state, State::Waiting);
}

#[test]
fn notifications_return_captured_payload_without_cancelling() {
    let mut engine = new(Direction::HostWrite);
    engine.start().unwrap();
    let mut notification = event(0x200);
    notification.host_message = Some(0xa5a5_1234);
    engine.io.events.push_back(notification);
    let slot = Cell::new(None);
    let mut irq = irq(&slot);
    let (_, waker) = counter();
    assert_eq!(
        poll_event(&mut engine, &slot, &mut irq, waker),
        Poll::Ready(Ok(TransferPoll { progress: TransferProgress::WaitingForRequest, events: notification }))
    );
    assert_eq!(engine.io.acked, [notification]);
    assert_eq!(engine.state, State::Waiting);
    assert!(!irq.enabled && slot.take().is_none());
}

#[test]
fn completion_and_device_error_preserve_status_and_retention() {
    for bits in [0x21, 0x80] {
        let mut engine = running(Direction::HostWrite);
        let before_ack = engine.io.acked.clone();
        engine.io.events.push_back(event(bits));
        let slot = Cell::new(None);
        let mut irq = irq(&slot);
        let (_, waker) = counter();
        let result = poll_event(&mut engine, &slot, &mut irq, waker);
        if bits == 0x21 {
            assert_eq!(
                result,
                Poll::Ready(Ok(TransferPoll { progress: TransferProgress::CompletionObserved, events: event(bits) }))
            );
        } else {
            assert_eq!(result, Poll::Ready(Err(TransferError::DeviceError(event(bits)))));
        }
        assert!(!irq.enabled && slot.take().is_none());
        assert_eq!(engine.state, State::Retained);
        assert_eq!(engine.io.acked, before_ack);
    }
}

#[test]
fn clock_loss_avoids_sdio_window_and_unknown_events_are_not_acknowledged() {
    for clock_loss in [false, true] {
        let mut engine = new(Direction::HostRead);
        engine.start().unwrap();
        engine.io.trace.clear();
        engine.io.clock = !clock_loss;
        engine.io.events.push_back(event(1 << 14));
        let slot = Cell::new(None);
        let mut irq = irq(&slot);
        let (_, waker) = counter();
        let expected = if clock_loss { TransferError::ClockGate } else { TransferError::Unsupported(event(1 << 14)) };
        assert_eq!(poll_event(&mut engine, &slot, &mut irq, waker), Poll::Ready(Err(expected)));
        if clock_loss {
            assert_eq!(engine.io.trace, ["clock", "clock"]);
        }
        assert!(engine.io.acked.is_empty() && engine.io.published.is_empty());
        assert!(!irq.enabled && slot.take().is_none());
        assert_eq!(engine.state, State::Retained);
    }
}

#[test]
fn unpolled_drop_is_inert_and_running_cancel_does_not_claim_abort() {
    let mut engine = running(Direction::HostWrite);
    let slot = Cell::new(None);
    let mut irq = irq(&slot);
    let trace = engine.io.trace.clone();
    cancel(&mut engine, false, false, &slot, &mut irq);
    assert_eq!(engine.io.trace, trace);
    assert_eq!(engine.state, State::Running);
    cancel(&mut engine, true, false, &slot, &mut irq);
    assert_eq!(&engine.io.trace[trace.len()..], ["clock", "mask"]);
    assert_eq!(engine.io.published, [0x20010000]);
    assert_eq!(engine.state, State::Retained);
}
