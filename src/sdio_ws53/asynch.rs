//! Exclusive IRQ46 notification: ISR touches no SDIO register or driver borrow.
//! Experimental: host race tests and actual CPU IRQ46 delivery under the opt-in
//! QEMU ADMA assumptions; physical board operation and bus drain remain unqualified.
use super::*;
use crate::interrupt;
use core::{
    cell::Cell,
    future::Future,
    pin::Pin,
    task::{Context, Poll, Waker},
};
use critical_section::Mutex;

static WAKER: Mutex<Cell<Option<Waker>>> = Mutex::new(Cell::new(None));
trait Slot {
    fn replace(&self, value: Option<Waker>) -> Option<Waker>;
}
struct DeviceSlot;
impl Slot for DeviceSlot {
    fn replace(&self, value: Option<Waker>) -> Option<Waker> {
        critical_section::with(|cs| WAKER.borrow(cs).replace(value))
    }
}
#[cfg(test)]
impl Slot for Cell<Option<Waker>> {
    fn replace(&self, value: Option<Waker>) -> Option<Waker> {
        Cell::replace(self, value)
    }
}
trait LocalIrq {
    fn disable(&mut self);
    fn enable(&mut self);
}
struct Irq46;
impl LocalIrq for Irq46 {
    fn disable(&mut self) {
        // SAFETY: the unsafe adapter/handler contract owns IRQ46 on this hart.
        unsafe { interrupt::disable(interrupt::Interrupt::SDIO) };
    }
    fn enable(&mut self) {
        // SAFETY: caller installed the sole handler before binding the adapter.
        unsafe { interrupt::enable(interrupt::Interrupt::SDIO) };
    }
}

/// Mask IRQ46, clear its local-controller latch, and schedule its sole waiter.
/// Does not read or acknowledge SDIO status, alter DMA, or borrow `Transfer`.
/// Raw peripheral events remain for normal-context `poll` to capture/service.
///
/// # Safety
/// Install as the sole non-reentrant SDIO service on the owning application hart.
/// All SDIO functions/global sources belong to that same exclusive owner; no
/// control-core/other handler may depend on IRQ46 remaining enabled. Wakers must
/// be ISR-safe and schedule normal-context polling without synchronous reentry.
pub unsafe fn on_async_interrupt() {
    Irq46.disable();
    if let Some(waker) = DeviceSlot.replace(None) {
        // Metadata CS has ended before any executor callback runs.
        waker.wake();
    }
}

/// An async event borrow could not be established; no state was changed.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AsyncBindError {
    /// Start the transfer first; a completed/failed/cancelled transfer cannot rearm.
    NotActive,
    /// The exclusively owned local IRQ46 must be masked before binding.
    InterruptsEnabled,
}

/// Wait for one captured notification or transfer event, preserving its snapshot.
///
/// Messages/acknowledgements and start/ADMA events return to the caller rather
/// than being discarded while awaiting completion. Call `wait_async` again for
/// the next event. No arbitrary hardware polling loop or hidden deadline exists.
/// A timeout combinator may cancel this future: after the first poll, Drop masks
/// notification and retains the entire transfer. Unpolled Drop changes nothing.
/// Successful intermediate events leave the transfer active with IRQ46 masked.
/// Completion/error retain all static allocations; this is never a drain proof.
/// A completed future must not be polled again. This borrow is not Send or Sync.
///
/// ```compile_fail
/// use hisi_hal::sdio::AsyncEvent;
/// fn send<T: Send>() {}
/// send::<AsyncEvent<'static, 'static>>();
/// ```
/// ```compile_fail
/// use hisi_hal::sdio::Transfer;
/// fn overlap(t: &mut Transfer<'_>) {
///     let wait = unsafe { t.wait_async() }.unwrap();
///     t.cancel();
///     drop(wait);
/// }
/// ```
pub struct AsyncEvent<'a, 'd> {
    transfer: &'a mut Transfer<'d>,
    polled: bool,
    finished: bool,
    _local: PhantomData<*mut ()>,
}
impl<'d> Transfer<'d> {
    /// Borrow a started transfer for one IRQ-driven captured event.
    ///
    /// # Safety
    /// Establish `on_async_interrupt`'s sole-handler/ownership contract on this
    /// hart. Keep source clocks, memory coherence and control-core handoff valid.
    /// Global MIE and priority threshold must allow IRQ46 delivery when waiting.
    /// This adapter may enable all three known SDIO global groups; no other
    /// function/user may share their delivery. This does not negotiate a host or
    /// control-core handoff and does not establish physical descriptor termination.
    pub unsafe fn wait_async(&mut self) -> Result<AsyncEvent<'_, 'd>, AsyncBindError> {
        if !matches!(self.engine.state, State::Waiting | State::Running) {
            return Err(AsyncBindError::NotActive);
        }
        if interrupt::is_enabled(interrupt::Interrupt::SDIO) {
            return Err(AsyncBindError::InterruptsEnabled);
        }
        Ok(AsyncEvent { transfer: self, polled: false, finished: false, _local: PhantomData })
    }
}

fn poll_event<R: Registers>(
    engine: &mut Engine<R>,
    slot: &impl Slot,
    irq: &mut impl LocalIrq,
    waker: Waker,
) -> Poll<Result<TransferPoll, TransferError>> {
    // Clone/drop/wake and all MMIO occur outside the metadata-only slot CS.
    drop(slot.replace(Some(waker)));
    irq.disable();
    let mut result = engine.poll();
    if matches!(&result, Ok(r) if !pending(r.events) && r.progress != TransferProgress::CompletionObserved) {
        engine.io.listen();
        irq.enable();
        // Recheck after arming. An event is either captured here or retains its
        // raw source plus registered wake; ISR never acknowledges those flags.
        result = engine.poll();
    }
    match result {
        Ok(r) if !pending(r.events) && r.progress != TransferProgress::CompletionObserved => Poll::Pending,
        result => {
            irq.disable();
            // If the bus disappeared, engine.poll already skipped SDIO access.
            if engine.io.clocks() {
                engine.io.mask();
            }
            drop(slot.replace(None));
            Poll::Ready(result)
        }
    }
}
fn cancel<R: Registers>(
    engine: &mut Engine<R>,
    polled: bool,
    finished: bool,
    slot: &impl Slot,
    irq: &mut impl LocalIrq,
) {
    if polled && !finished {
        irq.disable();
        engine.cancel();
        drop(slot.replace(None));
    }
}
impl Future for AsyncEvent<'_, '_> {
    type Output = Result<TransferPoll, TransferError>;
    fn poll(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
        let this = self.get_mut();
        assert!(!this.finished, "polled completed SDIO event wait");
        this.polled = true;
        let result = poll_event(&mut this.transfer.engine, &DeviceSlot, &mut Irq46, cx.waker().clone());
        this.finished = result.is_ready();
        result
    }
}
impl Drop for AsyncEvent<'_, '_> {
    fn drop(&mut self) {
        cancel(&mut self.transfer.engine, self.polled, self.finished, &DeviceSlot, &mut Irq46);
    }
}

#[cfg(all(test, not(target_arch = "riscv32")))]
#[path = "async_tests.rs"]
mod tests;
