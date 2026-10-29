//! WS53 application TIMER v150, two 32-bit channels in the MCU_MODE_1 profile.
//! Register facts come from hisi-registers. Clock/resource ownership must be
//! established through the WS53 peripheral handoff before constructing a driver.
use crate::interrupt::{self, Interrupt};
use crate::peripherals::Timer;
use crate::soc::chip::TIMER_CLOCK_HZ;

/// One of the two application timer channels.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TimerChannel {
    /// SDK TIMER_INDEX_0, IRQ 53.
    Channel0,
    /// SDK TIMER_INDEX_1, IRQ 54.
    Channel1,
}
impl TimerChannel {
    /// Reject indices that do not belong to this WS53 product profile.
    pub const fn from_index(index: u8) -> Option<Self> {
        match index {
            0 => Some(Self::Channel0),
            1 => Some(Self::Channel1),
            _ => None,
        }
    }
    /// Channel index for SDK interoperability.
    pub const fn index(self) -> usize {
        self as usize
    }
    /// Absolute IRQ cause number associated with the channel.
    pub const fn interrupt(self) -> Interrupt {
        match self {
            Self::Channel0 => Interrupt::TIMER_0,
            Self::Channel1 => Interrupt::TIMER_1,
        }
    }
}

/// Hardware timer mode.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TimerMode {
    /// Expire once and stop counting.
    OneShot,
    /// Reload the configured count after each expiry.
    Periodic,
}

/// Nonzero delay representable by the 32-bit load register.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TimerTicks(core::num::NonZeroU32);
impl TimerTicks {
    /// Validate the raw counter period without truncating or clamping.
    pub const fn from_count(count: u32) -> Option<Self> {
        match core::num::NonZeroU32::new(count) {
            Some(n) => Some(Self(n)),
            None => None,
        }
    }
    /// Convert microseconds using the established 32 MHz SDK clock profile.
    pub const fn from_micros(us: u32) -> Option<Self> {
        match crate::timer_math::checked_ticks(us, TIMER_CLOCK_HZ, core::num::NonZeroU32::new(1_000_000).unwrap()) {
            Some(ticks) => Self::from_count(ticks),
            None => None,
        }
    }
    /// Period in hardware counter ticks.
    pub const fn count(self) -> u32 {
        self.0.get()
    }
}

/// A snapshot request failed to complete before its finite polling budget.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TimerError {
    /// The hardware did not assert CNT_LOCK.
    SnapshotTimeout,
}

/// Owns the two TIMER channels after their explicit application-core handoff.
pub struct TimerDriver<'d> {
    _timer: Timer<'d>,
}

// Expand field-level PAC operations for both independently named RDL channels.
// No numeric pointer casts or hand-written register layouts are needed.
macro_rules! channel {
    ($ch:expr, $r:ident, $reg:ident, $zero:ident, $one:ident, $body:expr) => {
        match $ch {
            TimerChannel::Channel0 => {
                let $reg = $r.$zero();
                $body
            }
            TimerChannel::Channel1 => {
                let $reg = $r.$one();
                $body
            }
        }
    };
}

#[cfg(feature = "async")]
#[path = "timer_ws53/asynch.rs"]
mod asynch;
#[cfg(feature = "async")]
pub use asynch::{AsyncBindError, AsyncDelay};

impl<'d> TimerDriver<'d> {
    /// Construct from a handed-over, clocked application timer controller.
    pub fn new(timer: Timer<'d>) -> Self {
        Self { _timer: timer }
    }

    fn regs(&self) -> &'static crate::soc::pac::timer::RegisterBlock {
        unsafe { &*Timer::ptr() }
    }

    /// Stop, acknowledge and configure a channel; counting starts with `enable`.
    pub fn configure(&mut self, ch: TimerChannel, mode: TimerMode, ticks: TimerTicks) {
        self.disable(ch);
        self.clear_interrupt(ch);
        let r = self.regs();
        channel!(
            ch,
            r,
            reg,
            timer_load_count0,
            timer_load_count1,
            // All 32 load bits are intentional; no silicon reset value is assumed.
            unsafe { reg.write_with_zero(|w| w.load_count().bits(ticks.count())) }
        );
        channel!(
            ch,
            r,
            reg,
            timer_control0,
            timer_control1,
            reg.modify(|_, w| {
                // Preserve unmodeled bits. Explicitly select the initial control
                // state rather than relying on an inherited zero reset value.
                let w = w.enable().clear_bit().int_mask().clear_bit().rstfsm().clear_bit().cnt_req().clear_bit();
                match mode {
                    TimerMode::OneShot => w.mode().one_shot(),
                    TimerMode::Periodic => w.mode().periodic(),
                }
            })
        );
    }

    /// Start counting the previously configured period.
    pub fn enable(&mut self, ch: TimerChannel) {
        let r = self.regs();
        channel!(ch, r, reg, timer_control0, timer_control1, reg.modify(|_, w| w.enable().set_bit()));
    }

    /// Stop counting; does not consume a pending interrupt.
    pub fn disable(&mut self, ch: TimerChannel) {
        let r = self.regs();
        channel!(ch, r, reg, timer_control0, timer_control1, reg.modify(|_, w| w.enable().clear_bit()));
    }

    /// Mask or unmask peripheral interrupt delivery without discarding raw status.
    pub fn set_interrupt_mask(&mut self, ch: TimerChannel, masked: bool) {
        let r = self.regs();
        channel!(ch, r, reg, timer_control0, timer_control1, reg.modify(|_, w| w.int_mask().bit(masked)));
    }

    /// Request a coherent counter snapshot. A stalled clock returns an error.
    pub fn current_value(&mut self, ch: TimerChannel) -> Result<u32, TimerError> {
        let r = self.regs();
        let enabled = channel!(ch, r, reg, timer_control0, timer_control1, reg.read().enable().bit_is_set());
        if !enabled {
            return Ok(0);
        }
        channel!(ch, r, reg, timer_control0, timer_control1, reg.modify(|_, w| w.cnt_req().set_bit()));
        for _ in 0..0xffff {
            if channel!(ch, r, reg, timer_control0, timer_control1, reg.read().cnt_lock().bit_is_set()) {
                return Ok(channel!(
                    ch,
                    r,
                    reg,
                    timer_current_value0,
                    timer_current_value1,
                    reg.read().current_value().bits()
                ));
            }
            core::hint::spin_loop();
        }
        Err(TimerError::SnapshotTimeout)
    }

    /// Raw pending state, independent of the mask bit.
    pub fn interrupt_pending(&self, ch: TimerChannel) -> bool {
        let r = self.regs();
        channel!(ch, r, reg, timer_raw_intr0, timer_raw_intr1, reg.read().raw_intr().bit_is_set())
    }

    /// Acknowledge this channel at the peripheral and local interrupt controller.
    pub fn clear_interrupt(&mut self, ch: TimerChannel) {
        // The driver token proves application ownership.
        unsafe {
            Self::acknowledge_interrupt(ch);
        }
    }

    /// Acknowledge from the registered interrupt handler without borrowing the
    /// main-context driver across a preemption point. Performs no polling.
    ///
    /// # Safety
    /// This channel must belong to a live application TimerDriver. Call only
    /// from its IRQ handler or while that IRQ is disabled; do not reconfigure
    /// the peripheral concurrently or use this to acknowledge another owner.
    pub unsafe fn acknowledge_interrupt(ch: TimerChannel) {
        let r = unsafe { &*Timer::ptr() };
        // The SDK writes a complete acknowledgement word; never read EOI.
        channel!(ch, r, reg, timer_eoi0, timer_eoi1, unsafe { reg.write_with_zero(|w| w.eoi().clear_bit_by_one()) });
        interrupt::clear_pending(ch.interrupt());
    }
}

#[cfg(all(test, not(target_arch = "riscv32")))]
mod tests {
    use super::*;
    #[test]
    fn rejects_nonexistent_channel_and_unrepresentable_periods() {
        assert_eq!(TimerChannel::from_index(2), None);
        assert_eq!(TimerTicks::from_count(0), None);
        assert_eq!(TimerTicks::from_micros(10_000).unwrap().count(), 320_000);
        assert_eq!(TimerTicks::from_micros(u32::MAX), None);
        assert!(TimerTicks::from_count(u32::MAX).is_some());
    }
}
