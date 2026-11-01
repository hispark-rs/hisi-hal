//! WS63 register/IRQ adapter for the shared single-hart one-shot lifecycle.
use super::{TIMER_CLOCK_HZ, Timer, TimerChannel, TimerDriver};
use crate::{
    interrupt::{self, Interrupt},
    timer_async::{self, AlarmIo, Signal, Wait},
};
use core::num::NonZeroU32;

static SIGNALS: [Signal; 3] = [const { Signal::new() }; 3];

fn ch_irq(ch: TimerChannel) -> Interrupt {
    match ch {
        TimerChannel::Channel0 => Interrupt::TIMER_INT0,
        TimerChannel::Channel1 => Interrupt::TIMER_INT1,
        TimerChannel::Channel2 => Interrupt::TIMER_INT2,
    }
}
struct PacAlarm(TimerChannel);
impl AlarmIo for PacAlarm {
    fn stop(&mut self) {
        // The singleton TIMER token and ISR routing are owned by AsyncDelay.
        unsafe { interrupt::disable(ch_irq(self.0)) };
        let r = unsafe { &*Timer::ptr() };
        let control = r.timer0_control(self.0.index());
        control.modify(|_, w| w.enable().clear_bit().int_mask().set_bit());
        // EOI is a write-only complete command, never a read-modify-write.
        unsafe { r.timer0_eoi(self.0.index()).write(|w| w.bits(1)) };
        interrupt::clear_pending(ch_irq(self.0));
    }
    fn arm(&mut self, ticks: NonZeroU32) {
        let r = unsafe { &*Timer::ptr() };
        unsafe { r.timer0_load_count(self.0.index()).write(|w| w.bits(ticks.get())) };
        r.timer0_control(self.0.index()).write(|w| w.enable().clear_bit().mode().one_shot().int_mask().clear_bit());
        unsafe { interrupt::enable(ch_irq(self.0)) };
        r.timer0_control(self.0.index()).modify(|_, w| w.enable().set_bit());
    }
    fn pending(&mut self) -> bool {
        interrupt::clear_pending(ch_irq(self.0));
        let r = unsafe { &*Timer::ptr() };
        // Do not write EOI after a false sample: that would erase a later expiry.
        r.timer0_raw_intr(self.0.index()).read().bits() & 1 != 0
    }
}

/// Route the selected TIMER_INT0..2 trap here while AsyncDelay owns TIMER.
/// Stop/mask/acknowledge an actual raw expiry, then wake its waiter. Empty or
/// duplicate notifications do not complete a still-counting future.
///
/// Call on the owning hart from the non-reentrant timer ISR or with IRQ delivery
/// disabled during platform integration checks; no other handler,
/// core or NMI may operate that channel. Wake callbacks must be ISR-safe.
/// This legacy entry retains its signature; it is not a general timer ISR for
/// channels configured through TimerDriver or an RTOS/Embassy time provider.
pub fn on_interrupt(ch: TimerChannel) {
    timer_async::notify(&mut PacAlarm(ch), &SIGNALS[ch.index()]);
}

/// Owns the complete TIMER peripheral for one asynchronous channel.
///
/// Uses the established TIMER clock, not the CPU clock. Delay conversion rounds
/// upward and splits long durations into 32-bit periods. Zero does not arm.
/// Cancelling a polled delay stops/masks/acknowledges only this channel and
/// retires its Waker, allowing subsequent reuse; an unpolled delay does no MMIO.
/// The application routes the selected IRQ to on_interrupt and enables global
/// interrupts. Missing IRQ/clock progress leaves the Future Pending until drop.
/// No Embassy/RTOS component or global interrupt setup is implied.
pub struct AsyncDelay<'d> {
    driver: TimerDriver<'d>,
    channel: TimerChannel,
}
impl<'d> AsyncDelay<'d> {
    /// Consume the singleton TIMER token. Setup is deferred until the first wait.
    /// Keep the established clock and exclusive, non-reentrant ISR routing for
    /// this adapter's lifetime. Dropping the adapter quiesces its channel.
    pub fn new(timer: Timer<'d>, channel: TimerChannel) -> Self {
        Self { driver: TimerDriver::new(timer), channel }
    }
    async fn wait_total(&mut self, mut ticks: u64) {
        let _owner = &mut self.driver;
        while let Some(chunk) = crate::timer_math::next_delay_chunk(&mut ticks) {
            // The common splitter returns only nonzero chunks.
            Wait::new(PacAlarm(self.channel), &SIGNALS[self.channel.index()], NonZeroU32::new(chunk).unwrap()).await;
        }
    }
}
impl Drop for AsyncDelay<'_> {
    fn drop(&mut self) {
        timer_async::cancel(&mut PacAlarm(self.channel), &SIGNALS[self.channel.index()]);
    }
}
impl embedded_hal_async::delay::DelayNs for AsyncDelay<'_> {
    async fn delay_ns(&mut self, ns: u32) {
        self.wait_total(crate::timer_math::delay_ticks(ns, TIMER_CLOCK_HZ, 1_000_000_000)).await;
    }
    async fn delay_us(&mut self, us: u32) {
        self.wait_total(crate::timer_math::delay_ticks(us, TIMER_CLOCK_HZ, 1_000_000)).await;
    }
    async fn delay_ms(&mut self, ms: u32) {
        self.wait_total(crate::timer_math::delay_ticks(ms, TIMER_CLOCK_HZ, 1_000)).await;
    }
}
