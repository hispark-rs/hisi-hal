//! Legacy per-pin backend; wait lifecycle is shared with WS53.
use super::{GpioBank, Input, InterruptTrigger, regs};
use crate::{
    gpio_async::{Signal, Wait as WaitFuture, WaitIo},
    interrupt::{self, Interrupt},
    soc::chip,
};
use embedded_hal_async::digital::Wait;

static SIGNALS: [Signal<u8>; chip::GPIO_COUNT] = [const { Signal::new() }; chip::GPIO_COUNT];
fn bank_irq(bank: GpioBank) -> Interrupt {
    chip::GPIO_INTERRUPTS[bank.index() as usize]
}
fn bank(input: &Input<'_>) -> GpioBank {
    GpioBank::from_index(input.pin.block).expect("GPIO pin bank must be valid")
}
fn resume_if_enabled(bank: GpioBank) {
    if regs(bank.index()).gpio_int_en().read().gpio_int_en().bits() != 0 {
        unsafe { interrupt::enable(bank_irq(bank)) };
    }
}

/// Service enabled pending pins, not an entire bank-wide completion flag.
/// Route GPIO_INT0..2 exclusively here while using async GPIO. This hook masks
/// local delivery, clears the controller latch before observing sources, stops
/// and acknowledges captured pins, then wakes their individual waiters. Empty
/// or unrelated IRQs do not complete a wait. Uncaptured pins remain enabled.
/// This is a single-hart, non-reentrant ISR hook, not cross-core synchronization.
pub fn on_interrupt(bank: GpioBank) {
    unsafe { interrupt::disable(bank_irq(bank)) };
    interrupt::clear_pending(bank_irq(bank));
    let r = regs(bank.index());
    let fired = r.gpio_int_raw().read().gpio_int_raw().bits() & r.gpio_int_en().read().gpio_int_en().bits();
    r.gpio_int_en().modify(|v, w| unsafe { w.gpio_int_en().bits(v.gpio_int_en().bits() & !fired) });
    unsafe { r.gpio_int_eoi().write(|w| w.gpio_int_eoi().bits(fired)) };
    let first = bank.index() as usize * 8;
    let end = (first + 8).min(chip::GPIO_COUNT);
    for (offset, signal) in SIGNALS[first..end].iter().enumerate() {
        if fired & (1 << offset) != 0
            && let Some(line) = signal.active_line()
        {
            signal.notify(line);
        }
    }
    resume_if_enabled(bank);
}

struct InputWait<'a, 'd> {
    input: &'a mut Input<'d>,
    trigger: InterruptTrigger,
}
impl WaitIo for InputWait<'_, '_> {
    type Line = u8;
    fn line(&self) -> u8 {
        self.input.number()
    }
    fn pause(&mut self) {
        unsafe { interrupt::disable(bank_irq(bank(self.input))) };
    }
    fn configure(&mut self) {
        self.input.disable_interrupt();
        self.input.set_interrupt_trigger(self.trigger);
        self.input.clear_interrupt();
        self.input.enable_interrupt();
    }
    fn resume(&mut self) {
        unsafe { interrupt::enable(bank_irq(bank(self.input))) };
    }
    fn stop(&mut self) {
        self.pause();
        self.input.disable_interrupt();
        self.input.clear_interrupt();
        // Do not clear the bank controller latch here: another pin may have an
        // event. A stale latch is harmless; on_interrupt checks actual sources.
        resume_if_enabled(bank(self.input));
    }
    fn level_matches(&mut self) -> bool {
        match self.trigger {
            InterruptTrigger::HighLevel => self.input.is_high(),
            InterruptTrigger::LowLevel => self.input.is_low(),
            _ => false,
        }
    }
}
async fn arm_and_wait(input: &mut Input<'_>, trigger: InterruptTrigger) {
    let signal = &SIGNALS[input.number() as usize];
    WaitFuture::new(InputWait { input, trigger }, signal).await;
}

/// Interrupt-driven async edge/level waiting on a GPIO input. Requires the
/// app to route the GPIO trap to [`on_interrupt`] and enable global
/// interrupts. Level waits return immediately if the pin already matches.
/// Different pins have independent waiters, including within the same bank.
/// Dropping a pending future disables and acknowledges only its pin.
/// Pin configuration/IRQ routing must remain exclusively owned; this does
/// not provide cross-core exclusion or make raw GPIO access concurrent-safe.
/// `wait_for_any_edge` retains the legacy polarity chosen from the initial
/// input sample; it does not claim hardware both-edge capture during setup.
impl Wait for Input<'_> {
    async fn wait_for_high(&mut self) -> Result<(), Self::Error> {
        if self.is_high() {
            return Ok(());
        }
        arm_and_wait(self, InterruptTrigger::HighLevel).await;
        Ok(())
    }
    async fn wait_for_low(&mut self) -> Result<(), Self::Error> {
        if self.is_low() {
            return Ok(());
        }
        arm_and_wait(self, InterruptTrigger::LowLevel).await;
        Ok(())
    }
    async fn wait_for_rising_edge(&mut self) -> Result<(), Self::Error> {
        arm_and_wait(self, InterruptTrigger::RisingEdge).await;
        Ok(())
    }
    async fn wait_for_falling_edge(&mut self) -> Result<(), Self::Error> {
        arm_and_wait(self, InterruptTrigger::FallingEdge).await;
        Ok(())
    }
    async fn wait_for_any_edge(&mut self) -> Result<(), Self::Error> {
        let trig = if self.is_high() { InterruptTrigger::FallingEdge } else { InterruptTrigger::RisingEdge };
        arm_and_wait(self, trig).await;
        Ok(())
    }
}
