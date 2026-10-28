//! Shared IRQ58 notification, with per-channel resources retained on every exit.
use super::*;
use core::{
    cell::Cell,
    future::Future,
    pin::Pin,
    task::{Context, Poll, Waker},
};
use critical_section::Mutex;

static WAKERS: Mutex<[Cell<Option<Waker>>; 8]> = Mutex::new([const { Cell::new(None) }; 8]);

pub(super) trait Slot {
    fn replace(&self, value: Option<Waker>) -> Option<Waker>;
}
pub(super) struct ChannelSlot(pub(super) ChannelId);
impl Slot for ChannelSlot {
    fn replace(&self, value: Option<Waker>) -> Option<Waker> {
        critical_section::with(|cs| WAKERS.borrow(cs)[self.0 as usize].replace(value))
    }
}
#[cfg(test)]
impl Slot for Cell<Option<Waker>> {
    fn replace(&self, value: Option<Waker>) -> Option<Waker> {
        Cell::replace(self, value)
    }
}

/// Mask shared IRQ58 and wake every registered DMA waiter, outside the metadata
/// critical section. Raw peripheral flags and channel configuration are retained.
/// Each waiter checks its own raw completion/error state and rearms delivery if
/// necessary. A notification may be spurious for any particular channel.
///
/// # Safety
/// Install this as the sole non-reentrant MDMA service on the owning application
/// hart. All IRQ58 users must participate in this notification scheme: no
/// unrelated interrupt-enabled channel, handler or other-core owner may depend
/// on the shared local enable remaining set. Wakers must be ISR-safe and schedule
/// polling in normal context; they must not synchronously poll these transfers.
pub unsafe fn on_async_interrupt() {
    unsafe { interrupt::disable(interrupt::Interrupt::M_DMA) };
    wake_registered();
}
fn wake_registered() {
    let wake: [Option<Waker>; 8] = critical_section::with(|cs| core::array::from_fn(|i| WAKERS.borrow(cs)[i].take()));
    for waker in wake.into_iter().flatten() {
        waker.wake();
    }
}

/// A one-shot memory-copy future. Completion, error and cancellation all retain
/// the channel and both static buffers in the borrowed `Transfer`.
///
/// The first poll configures/starts the transfer and arms IRQ58. It returns
/// Pending even if hardware has already completed, so progress requires actual
/// interrupt delivery. Later polls perform one finite status/configuration
/// check, not a busy wait. No completion event is a bus-drain proof; releasing
/// buffers still requires `release_after_external_quiescence` and its independent
/// unsafe proof. Cancellation requests halt/disable only for this channel and
/// removes its waker, leaving raw flags and other channels unchanged.
///
/// Dropping an unpolled future makes no MMIO changes. After any started attempt,
/// the transfer is retained and cannot restart. There is no implicit deadline;
/// a timeout combinator may cancel it. This future is neither Send nor Sync.
///
/// ```compile_fail
/// use hisi_hal::dma::AsyncTransfer;
/// fn send<T: Send>() {}
/// send::<AsyncTransfer<'static, 'static, u32>>();
/// ```
/// ```compile_fail
/// use hisi_hal::dma::Transfer;
/// fn exclusive(t: &mut Transfer<'_, u32>) {
///     let pending = unsafe { t.run_async() }.unwrap();
///     t.request_stop();
///     drop(pending);
/// }
/// ```
pub struct AsyncTransfer<'a, 'd, T: Element> {
    transfer: &'a mut Transfer<'d, T>,
    started: bool,
    _local: PhantomData<*mut ()>,
}
impl<'d, T: Element> Transfer<'d, T> {
    /// Borrow a ready memory transfer for interrupt-driven start and completion.
    /// Already running/retained transfers are refused without changing hardware.
    ///
    /// # Safety
    /// Establish `on_async_interrupt`'s shared-controller routing contract on the
    /// same hart for this borrow. Preserve exclusive controller/channel ownership,
    /// coherent static buffers and source/bus clocks. Global MIE and threshold
    /// must permit IRQ58 delivery. Other simultaneous users may only be these
    /// async transfers; none may reinitialize the controller or change IRQ58's
    /// service. This does not negotiate resources with the control core.
    pub unsafe fn run_async(&mut self) -> Result<AsyncTransfer<'_, 'd, T>, Error> {
        if self.engine.state != State::Ready {
            return Err(Error::NotReady);
        }
        Ok(AsyncTransfer { transfer: self, started: false, _local: PhantomData })
    }
}

// Register before starting/enabling so an immediate or shared interrupt cannot
// lose the wake. The ISR never touches Engine/PacRegisters Rust state. No MMIO,
// polling, Waker cloning/drop or callback executes under our metadata CS.
pub(super) fn poll_engine<R: InterruptRegisters>(
    engine: &mut Engine<R>,
    started: &mut bool,
    slot: &impl Slot,
    waker: Waker,
    enable_irq: impl FnOnce(),
) -> Poll<Result<Completion, Error>> {
    drop(slot.replace(Some(waker)));
    let result = if !*started {
        *started = true;
        engine.listen(InterruptEvents::All).and_then(|()| engine.start()).map(|()| None)
    } else {
        engine.poll()
    };
    match result {
        Ok(None) => {
            enable_irq();
            Poll::Pending
        }
        result => {
            engine.stop();
            drop(slot.replace(None));
            Poll::Ready(match result {
                Ok(Some(done)) => Ok(done),
                Err(e) => Err(e),
                _ => unreachable!(),
            })
        }
    }
}
pub(super) fn cancel<R: InterruptRegisters>(engine: &mut Engine<R>, started: bool, slot: &impl Slot) {
    if started {
        engine.stop();
        drop(slot.replace(None));
    }
}
impl<T: Element> Future for AsyncTransfer<'_, '_, T> {
    type Output = Result<Completion, Error>;
    fn poll(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
        // The future has no self-reference and never pins the DMA allocations.
        let this = self.get_mut();
        let slot = ChannelSlot(this.transfer.engine.io.id);
        poll_engine(&mut this.transfer.engine, &mut this.started, &slot, cx.waker().clone(), || {
            unsafe { interrupt::enable(interrupt::Interrupt::M_DMA) };
        })
    }
}
impl<T: Element> Drop for AsyncTransfer<'_, '_, T> {
    fn drop(&mut self) {
        let slot = ChannelSlot(self.transfer.engine.io.id);
        cancel(&mut self.transfer.engine, self.started, &slot);
    }
}

#[cfg(all(test, not(target_arch = "riscv32")))]
mod tests {
    use super::super::tests::{Event, engine};
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
    #[test]
    fn immediate_completion_requires_registered_notification_then_retains() {
        let (mut e, delay) = engine();
        e.io.ready = 1;
        let slot = Cell::new(None);
        let mut started = false;
        let count = Arc::new(Count(AtomicUsize::new(0)));
        assert!(
            poll_engine(&mut e, &mut started, &slot, Waker::from(count.clone()), || {
                slot.take().unwrap().wake();
            })
            .is_pending()
        );
        assert_eq!(count.0.load(Ordering::SeqCst), 1);
        assert_eq!(
            poll_engine(&mut e, &mut started, &slot, Waker::noop().clone(), || panic!()),
            Poll::Ready(Ok(Completion { elements: 3 }))
        );
        assert!(e.state == State::Retained && slot.take().is_none());
        assert_eq!(delay.0.borrow().iter().filter(|x| **x == Event::Stop).count(), 1);
        cancel(&mut e, started, &slot);
        assert_eq!(delay.0.borrow().iter().filter(|x| **x == Event::Stop).count(), 1);
    }
    #[test]
    fn spurious_repoll_replaces_waker_without_restart_and_cancel_retains() {
        let (mut e, delay) = engine();
        e.io.ready = 100;
        let slot = Cell::new(None);
        let mut started = false;
        let old = Arc::new(Count(AtomicUsize::new(0)));
        let new = Arc::new(Count(AtomicUsize::new(0)));
        assert!(poll_engine(&mut e, &mut started, &slot, Waker::from(old.clone()), || {}).is_pending());
        assert!(poll_engine(&mut e, &mut started, &slot, Waker::from(new.clone()), || {}).is_pending());
        assert_eq!(Arc::strong_count(&old), 1);
        cancel(&mut e, started, &slot);
        assert_eq!(Arc::strong_count(&new), 1);
        assert!(e.state == State::Retained);
        assert_eq!(delay.0.borrow().iter().filter(|x| **x == Event::Start).count(), 1);
        assert_eq!(e.start(), Err(Error::NotReady));
    }
    #[test]
    fn unpolled_cancel_does_not_touch_hardware_or_consume_ready_state() {
        let (mut e, delay) = engine();
        cancel(&mut e, false, &Cell::new(None));
        assert!(delay.0.borrow().is_empty() && e.state == State::Ready);
    }
    #[test]
    fn bus_error_and_invalid_completion_stop_without_acknowledging_raw_evidence() {
        for bus in [false, true] {
            let (mut e, delay) = engine();
            e.io.ready = 1;
            e.io.error = bus;
            e.io.bad_done = !bus;
            let slot = Cell::new(None);
            let mut started = false;
            assert!(poll_engine(&mut e, &mut started, &slot, Waker::noop().clone(), || {}).is_pending());
            assert_eq!(
                poll_engine(&mut e, &mut started, &slot, Waker::noop().clone(), || panic!()),
                Poll::Ready(Err(if bus { Error::Bus } else { Error::CompletionState }))
            );
            assert!(e.state == State::Retained && slot.take().is_none());
            assert!(!delay.0.borrow().iter().any(|x| matches!(x, Event::Acknowledge(_))));
        }
    }
    #[test]
    fn shared_notification_takes_all_waiters_before_waking_and_preserves_empty_slots() {
        let first = Arc::new(Count(AtomicUsize::new(0)));
        let last = Arc::new(Count(AtomicUsize::new(0)));
        assert!(ChannelSlot(ChannelId::Channel0).replace(Some(Waker::from(first.clone()))).is_none());
        assert!(ChannelSlot(ChannelId::Channel7).replace(Some(Waker::from(last.clone()))).is_none());
        wake_registered();
        assert_eq!(first.0.load(Ordering::SeqCst), 1);
        assert_eq!(last.0.load(Ordering::SeqCst), 1);
        wake_registered();
        assert_eq!(first.0.load(Ordering::SeqCst), 1);
        assert_eq!(last.0.load(Ordering::SeqCst), 1);
        assert_eq!(Arc::strong_count(&first), 1);
        assert_eq!(Arc::strong_count(&last), 1);
    }
}
