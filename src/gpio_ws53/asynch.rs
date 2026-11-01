//! Single-hart GPIO waits; one waiter per exclusively borrowed bank.
use super::{GpioBank, GpioDriver, GpioLine, Input, InterruptTrigger, bitmap, cancel, read_level};
use crate::gpio_async::{Signal, Wait, WaitIo};
use crate::{
    interrupt,
    peripherals::{Gpio0, Gpio1, Gpio2},
};
use core::{convert::Infallible, marker::PhantomData};

static SIGNALS: [Signal<GpioLine>; 3] = [const { Signal::new() }; 3];
fn index(bank: GpioBank) -> usize {
    match bank {
        GpioBank::Bank0 => 0,
        GpioBank::Bank1 => 1,
        GpioBank::Bank2 => 2,
    }
}
fn signal(bank: GpioBank) -> &'static Signal<GpioLine> {
    &SIGNALS[index(bank)]
}
fn reset(line: GpioLine) {
    cancel(line);
    signal(line.bank()).clear();
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
        let Some(line) = signal(bank).active_line() else {
            return false;
        };
        let pending = bank!(bank, r, bitmap(r.gpio_intr().read().gpio_intr().bits()));
        if pending & line.mask() == 0 {
            unsafe { interrupt::enable(bank.interrupt()) };
            return false;
        }
        cancel(line);
        signal(bank).notify(line)
    }
}
fn level_satisfied(trigger: InterruptTrigger, high: bool) -> bool {
    matches!((trigger, high), (InterruptTrigger::HighLevel, true) | (InterruptTrigger::LowLevel, false))
}
struct InputWait<'b, 'a, 'd, 'p> {
    input: &'b mut Input<'a, 'd, 'p>,
    trigger: InterruptTrigger,
}
impl WaitIo for InputWait<'_, '_, '_, '_> {
    type Line = GpioLine;
    fn line(&self) -> GpioLine {
        self.input.line
    }
    fn pause(&mut self) {
        unsafe { interrupt::disable(self.line().bank().interrupt()) };
    }
    fn configure(&mut self) {
        self.input.listen(self.trigger);
    }
    fn resume(&mut self) {
        unsafe { interrupt::enable(self.line().bank().interrupt()) };
    }
    fn stop(&mut self) {
        cancel(self.line());
    }
    fn level_matches(&mut self) -> bool {
        level_satisfied(self.trigger, read_level(self.line()))
    }
}
impl Drop for AsyncInput<'_, '_, '_, '_> {
    fn drop(&mut self) {
        reset(self.input.line);
    }
}
impl AsyncInput<'_, '_, '_, '_> {
    async fn wait(&mut self, trigger: InterruptTrigger) {
        let signal = signal(self.input.line.bank());
        Wait::new(InputWait { input: &mut *self.input, trigger }, signal).await;
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
    use super::*;
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
}
