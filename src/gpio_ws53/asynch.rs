//! Single-hart GPIO waits; one waiter per exclusively borrowed bank.
use super::{GpioBank, GpioDriver, GpioLine, Input, InterruptTrigger, bitmap, cancel, read_level};
use crate::{
    interrupt,
    peripherals::{Gpio0, Gpio1, Gpio2},
};
use core::{
    cell::Cell,
    convert::Infallible,
    future::poll_fn,
    marker::PhantomData,
    task::{Context, Poll, Waker},
};
use critical_section::Mutex;

struct Signal {
    line: Cell<Option<GpioLine>>,
    fired: Cell<bool>,
    waker: Cell<Option<Waker>>,
}
impl Signal {
    const fn new() -> Self {
        Self { line: Cell::new(None), fired: Cell::new(false), waker: Cell::new(None) }
    }
    fn clear(&self) -> Option<Waker> {
        self.line.set(None);
        self.fired.set(false);
        self.waker.take()
    }
    fn fire(&self, line: GpioLine) -> Option<Waker> {
        if self.line.get() != Some(line) {
            return None;
        }
        self.line.set(None);
        self.fired.set(true);
        self.waker.take()
    }
}
static SIGNALS: Mutex<[Signal; 3]> = Mutex::new([const { Signal::new() }; 3]);
fn index(bank: GpioBank) -> usize {
    match bank {
        GpioBank::Bank0 => 0,
        GpioBank::Bank1 => 1,
        GpioBank::Bank2 => 2,
    }
}
fn signal<R>(bank: GpioBank, f: impl FnOnce(&Signal) -> R) -> R {
    critical_section::with(|cs| f(&SIGNALS.borrow(cs)[index(bank)]))
}
fn reset(line: GpioLine) {
    // MMIO is outside critical sections; this bank's IRQ is disabled first.
    cancel(line);
    drop(signal(line.bank(), Signal::clear));
}

/// Refused async takeover. No registers or wakers have been changed.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AsyncBindError {
    /// A peripheral notification in this bank, or its local IRQ, is enabled.
    InterruptsEnabled,
    /// The bank's local pending latch has not been explicitly acknowledged.
    PendingInterrupt,
}

/// Borrow one prepared input and its whole GPIO bank for interrupt-driven waits.
/// Implements all five `embedded_hal_async::digital::Wait` operations.
///
/// A level wait may complete immediately. An edge wait starts observing on its
/// first poll (after clearing stale edges), not when its future is constructed.
/// Events latched while armed remain completed even if the input changes again.
/// Dropping a pending future masks/disables/acknowledges its line and removes its
/// waker; a later wait starts afresh. Pad/mux/pull/clock configuration is unchanged.
/// No debounce, global MIE, executor, sleep or clock setup is performed. A missing
/// ISR/clock leaves an unsatisfied wait pending. Local enable uses priority1.
/// This adapter is neither Send nor Sync and permits one active wait per bank.
/// ```compile_fail
/// use hisi_hal::gpio::AsyncInput;
/// fn require_send<T: Send>() {}
/// require_send::<AsyncInput<'static, 'static, 'static, 'static>>();
/// ```
pub struct AsyncInput<'b, 'a, 'd, 'p> {
    input: &'b mut Input<'a, 'd, 'p>,
    _local: PhantomData<*mut ()>,
}
impl<'a, 'd, 'p> Input<'a, 'd, 'p> {
    /// Borrow an input whose bank has no enabled notification sources.
    ///
    /// # Safety
    /// Route this bank's non-reentrant ISR exclusively to
    /// `GpioDriver::on_async_interrupt` on the owning application hart until the
    /// adapter is dropped. No other core/NMI/ISR may access the bank. Preserve the
    /// input's pad/clock ownership prerequisites and allow global IRQ delivery
    /// while awaiting. Wakers must support waking from this ISR. Do not use the
    /// ordinary GPIO handler alongside the async handler.
    pub unsafe fn async_wait(&mut self) -> Result<AsyncInput<'_, 'a, 'd, 'p>, AsyncBindError> {
        let bank = self.line.bank();
        if interrupt::is_enabled(bank.interrupt())
            || bank!(bank, r, bitmap(r.gpio_int_en().read().gpio_int_en().bits())) != 0
        {
            return Err(AsyncBindError::InterruptsEnabled);
        }
        if interrupt::is_pending(bank.interrupt()) {
            return Err(AsyncBindError::PendingInterrupt);
        }
        reset(self.line);
        Ok(AsyncInput { input: self, _local: PhantomData })
    }
}
impl GpioDriver<'_> {
    /// Complete the bound line's actual notification, leaving it disabled.
    /// A spurious IRQ does not consume a wait or leave its notification disabled.
    /// Waking occurs after all driver critical sections have ended.
    ///
    /// # Safety
    /// Call on the owning hart from the exclusively bound GPIO ISR, or with its
    /// IRQ disabled. The bank handoff and async ISR contract must remain valid.
    pub unsafe fn on_async_interrupt(bank: GpioBank) -> bool {
        unsafe { interrupt::disable(bank.interrupt()) };
        // Clear controller BEFORE observing the peripheral, so a later edge is
        // not cleared after a false observation. Never EOI an unobserved edge.
        interrupt::clear_pending(bank.interrupt());
        let Some(line) = signal(bank, |s| s.line.get()) else {
            return false;
        };
        let pending = bank!(bank, r, bitmap(r.gpio_intr().read().gpio_intr().bits()));
        if pending & line.mask() == 0 {
            unsafe { interrupt::enable(bank.interrupt()) };
            return false;
        }
        cancel(line);
        let waker = signal(bank, |s| s.fire(line));
        if let Some(waker) = waker {
            waker.wake();
        }
        true
    }
}
fn level_satisfied(trigger: InterruptTrigger, high: bool) -> bool {
    matches!((trigger, high), (InterruptTrigger::HighLevel, true) | (InterruptTrigger::LowLevel, false))
}
struct Wait<'b, 'a, 'd, 'p> {
    input: &'b mut Input<'a, 'd, 'p>,
    trigger: InterruptTrigger,
    started: bool,
}
impl Wait<'_, '_, '_, '_> {
    fn poll(&mut self, cx: &mut Context<'_>) -> Poll<()> {
        let waker = cx.waker().clone();
        let line = self.input.line;
        let bank = line.bank();
        unsafe { interrupt::disable(bank.interrupt()) };
        if !self.started {
            self.input.listen(self.trigger);
            signal(bank, |s| {
                s.fired.set(false);
                s.line.set(Some(line));
            });
            self.started = true;
        }
        // Register before inspecting the condition or allowing ISR delivery.
        let (old, fired) = signal(bank, |s| (s.waker.replace(Some(waker)), s.fired.get()));
        let ready = fired || level_satisfied(self.trigger, read_level(line));
        if ready {
            reset(line);
        } else {
            unsafe { interrupt::enable(bank.interrupt()) };
        }
        drop(old);
        if ready { Poll::Ready(()) } else { Poll::Pending }
    }
}
impl Drop for Wait<'_, '_, '_, '_> {
    fn drop(&mut self) {
        if self.started {
            reset(self.input.line);
        }
    }
}
impl Drop for AsyncInput<'_, '_, '_, '_> {
    fn drop(&mut self) {
        reset(self.input.line);
    }
}
impl AsyncInput<'_, '_, '_, '_> {
    async fn wait(&mut self, trigger: InterruptTrigger) {
        let mut wait = Wait { input: &mut *self.input, trigger, started: false };
        poll_fn(|cx| wait.poll(cx)).await;
    }
}
impl embedded_hal::digital::ErrorType for AsyncInput<'_, '_, '_, '_> {
    type Error = Infallible;
}
impl embedded_hal_async::digital::Wait for AsyncInput<'_, '_, '_, '_> {
    async fn wait_for_high(&mut self) -> Result<(), Self::Error> {
        self.wait(InterruptTrigger::HighLevel).await;
        Ok(())
    }
    async fn wait_for_low(&mut self) -> Result<(), Self::Error> {
        self.wait(InterruptTrigger::LowLevel).await;
        Ok(())
    }
    async fn wait_for_rising_edge(&mut self) -> Result<(), Self::Error> {
        self.wait(InterruptTrigger::RisingEdge).await;
        Ok(())
    }
    async fn wait_for_falling_edge(&mut self) -> Result<(), Self::Error> {
        self.wait(InterruptTrigger::FallingEdge).await;
        Ok(())
    }
    async fn wait_for_any_edge(&mut self) -> Result<(), Self::Error> {
        self.wait(InterruptTrigger::BothEdges).await;
        Ok(())
    }
}

#[cfg(all(test, not(target_arch = "riscv32")))]
mod tests {
    use super::super::Pad;
    use super::*;
    use std::{
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
    #[test]
    fn level_match_never_completes_an_edge_wait() {
        for trigger in [InterruptTrigger::RisingEdge, InterruptTrigger::FallingEdge, InterruptTrigger::BothEdges] {
            assert!(!level_satisfied(trigger, false));
            assert!(!level_satisfied(trigger, true));
        }
        for high in [false, true] {
            assert_eq!(level_satisfied(InterruptTrigger::HighLevel, high), high);
            assert_eq!(level_satisfied(InterruptTrigger::LowLevel, high), !high);
        }
    }
    #[test]
    fn unrelated_duplicate_and_cancelled_events_do_not_wake_or_complete() {
        let s = Signal::new();
        let line = Pad::Mgpio17.gpio_line().unwrap();
        let other = Pad::Mgpio18.gpio_line().unwrap();
        let a = Arc::new(Counter(AtomicUsize::new(0)));
        let b = Arc::new(Counter(AtomicUsize::new(0)));
        s.line.set(Some(line));
        s.waker.set(Some(Waker::from(a.clone())));
        drop(s.waker.replace(Some(Waker::from(b.clone()))));
        assert_eq!(Arc::strong_count(&a), 1);
        assert!(s.fire(other).is_none());
        assert!(!s.fired.get());
        s.fire(line).unwrap().wake();
        assert!(s.fired.get());
        assert!(s.fire(line).is_none());
        assert_eq!(b.0.load(Ordering::SeqCst), 1);
        drop(s.clear());
        assert!(!s.fired.get());
        s.line.set(Some(line));
        s.waker.set(Some(Waker::from(a.clone())));
        drop(s.clear());
        assert!(s.fire(line).is_none());
        assert_eq!(Arc::strong_count(&a), 1);
        assert_eq!(a.0.load(Ordering::SeqCst), 0);
    }
}
