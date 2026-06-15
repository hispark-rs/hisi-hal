//! Timer driver for WS63 (3 independent 32-bit timers).
//!
//! Each timer can operate in one-shot or periodic mode.
//! The timer counts at the **TCXO crystal clock** ([`TIMER_CLOCK_HZ`] = 24 MHz on
//! 24 MHz-crystal boards) — NOT the 240 MHz CPU/PLL clock. The vendor SDK programs
//! the timer to the crystal via `timer_porting_clock_value_set(REQ_24M)`.
//!
//! # Usage
//!
//! ```no_run
//! let timer = TimerDriver::new(peripherals.TIMER);
//! let mut oneshot = timer.oneshot(0);
//! oneshot.start(24_000); // 1ms at 24MHz
//! while !oneshot.expired() {}
//! ```

use crate::peripherals::Timer;
use crate::soc::chip::TIMER_CLOCK_HZ;

/// Timer operating mode.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TimerMode {
    /// One-shot: timer counts once and stops.
    OneShot = 0,
    /// Periodic: timer reloads and repeats.
    Periodic = 1,
}

/// Timer driver managing 3 independent timer channels.
pub struct TimerDriver<'d> {
    _timer: Timer<'d>,
}

impl<'d> TimerDriver<'d> {
    /// Create a new timer driver.
    pub fn new(timer: Timer<'d>) -> Self {
        Self { _timer: timer }
    }

    fn regs(&self) -> &'static crate::soc::pac::timer::RegisterBlock {
        // SAFETY: PAC peripheral pointer is a static physical MMIO address, always valid
        unsafe { &*Timer::ptr() }
    }

    /// Configure a timer channel.
    pub fn configure(&self, n: usize, mode: TimerMode, load_value: u32) {
        let r = self.regs();
        match n {
            0 => r.timer0_load_count(0).write(|w| unsafe { w.bits(load_value) }),
            1 => r.timer0_load_count(1).write(|w| unsafe { w.bits(load_value) }),
            2 => r.timer0_load_count(2).write(|w| unsafe { w.bits(load_value) }),
            _ => unreachable!(),
        };
        let ctl = ((mode as u32) & 0x3) << 1;
        match n {
            0 => r.timer0_control(0).write(|w| unsafe { w.bits(ctl) }),
            1 => r.timer0_control(1).write(|w| unsafe { w.bits(ctl) }),
            2 => r.timer0_control(2).write(|w| unsafe { w.bits(ctl) }),
            _ => unreachable!(),
        };
    }

    /// Enable a timer channel.
    pub fn enable(&self, n: usize) {
        let r = self.regs();
        let prev = match n {
            0 => r.timer0_control(0).read().bits(),
            1 => r.timer0_control(1).read().bits(),
            2 => r.timer0_control(2).read().bits(),
            _ => unreachable!(),
        };
        match n {
            0 => r.timer0_control(0).write(|w| unsafe { w.bits(prev | 1) }),
            1 => r.timer0_control(1).write(|w| unsafe { w.bits(prev | 1) }),
            2 => r.timer0_control(2).write(|w| unsafe { w.bits(prev | 1) }),
            _ => unreachable!(),
        };
    }

    /// Disable a timer channel.
    pub fn disable(&self, n: usize) {
        let r = self.regs();
        let prev = match n {
            0 => r.timer0_control(0).read().bits(),
            1 => r.timer0_control(1).read().bits(),
            2 => r.timer0_control(2).read().bits(),
            _ => unreachable!(),
        };
        match n {
            0 => r.timer0_control(0).write(|w| unsafe { w.bits(prev & !1) }),
            1 => r.timer0_control(1).write(|w| unsafe { w.bits(prev & !1) }),
            2 => r.timer0_control(2).write(|w| unsafe { w.bits(prev & !1) }),
            _ => unreachable!(),
        };
    }

    /// Read the current counter value.
    ///
    /// On WS63 (TIMER_V150) the per-channel `current_value0` register is a
    /// **latched snapshot**, not a live counter: it only refreshes after a
    /// `cnt_req`/`cnt_lock` handshake. Reading it raw returns the same stale
    /// value forever — the vendor `hal_timer_v150_get_current_value()` always
    /// performs this handshake; QEMU exposes a live counter and ignores it,
    /// which is why the un-handshaked read passed under emulation but froze on
    /// silicon (hisi-riscv-rs#10). A disabled channel keeps a stale latch, so we
    /// return 0 there (matching the vendor HAL) rather than a meaningless value.
    pub fn current_value(&self, n: usize) -> u32 {
        // The poll bound mirrors the vendor TIMER_CURRENT_COUNT_LOCK_TIMEOUT
        // (0xFFFF) so a never-locking channel cannot spin forever. cnt_req
        // (bit 5) / cnt_lock (bit 6) are now named fields in ws63-pac.
        const LOCK_TIMEOUT: u32 = 0xFFFF;

        let r = self.regs();
        // The control + current-value registers are a 3-element array; index it
        // per channel (mirrors enable()/disable()).
        let ctrl = |n: usize| match n {
            0 => r.timer0_control(0),
            1 => r.timer0_control(1),
            2 => r.timer0_control(2),
            _ => unreachable!(),
        };
        let read_value = |n: usize| match n {
            0 => r.timer0_current_value(0).read().bits(),
            1 => r.timer0_current_value(1).read().bits(),
            2 => r.timer0_current_value(2).read().bits(),
            _ => unreachable!(),
        };
        // cnt_req (bit 5) / cnt_lock (bit 6) of TIMER_V150's control register. WS63's
        // ws63-pac models them as named fields; BS2X's bs2x-pac does not (same V150
        // IP, the field just isn't in its SVD), so chip-bs21 pokes the raw bits.
        let set_cnt_req = |n: usize| {
            #[cfg(feature = "chip-ws63")]
            ctrl(n).modify(|_, w| w.cnt_req().set_bit());
            #[cfg(feature = "chip-bs21")]
            ctrl(n).modify(|r, w| unsafe { w.bits(r.bits() | (1 << 5)) });
        };
        let cnt_locked = |n: usize| -> bool {
            #[cfg(feature = "chip-ws63")]
            {
                ctrl(n).read().cnt_lock().bit_is_set()
            }
            #[cfg(feature = "chip-bs21")]
            {
                (ctrl(n).read().bits() & (1 << 6)) != 0
            }
        };

        // A disabled timer holds a stale latch; the vendor HAL returns 0.
        if ctrl(n).read().enable().bit_is_clear() {
            return 0;
        }
        // Request a fresh snapshot of the down-counter into current_value0
        // (modify preserves enable/mode/int_mask)…
        set_cnt_req(n);
        // …then wait (bounded) for the hardware to latch it.
        let mut timeout = 0u32;
        while timeout < LOCK_TIMEOUT {
            if cnt_locked(n) {
                return read_value(n);
            }
            timeout += 1;
        }
        // Latch never asserted (should not happen on a running timer); best effort.
        read_value(n)
    }

    /// Check if a timer interrupt is pending.
    pub fn interrupt_pending(&self, n: usize) -> bool {
        let r = self.regs();
        match n {
            0 => r.timer0_raw_intr(0).read().bits() & 1 != 0,
            1 => r.timer0_raw_intr(1).read().bits() & 1 != 0,
            2 => r.timer0_raw_intr(2).read().bits() & 1 != 0,
            _ => unreachable!(),
        }
    }

    /// Clear a timer interrupt (per-channel EOI).
    pub fn clear_interrupt(&self, n: usize) {
        let r = self.regs();
        match n {
            0 => {
                let _ = r.timer0_eoi(0).read().bits();
            }
            1 => {
                let _ = r.timer0_eoi(1).read().bits();
            }
            2 => {
                let _ = r.timer0_eoi(2).read().bits();
            }
            _ => unreachable!(),
        }
    }

    /// Create a one-shot timer wrapper for the given channel.
    pub fn oneshot(&self, channel: usize) -> OneShotTimer<'_> {
        OneShotTimer { driver: self, channel }
    }

    /// Create a periodic timer wrapper for the given channel.
    pub fn periodic(&self, channel: usize) -> PeriodicTimer<'_> {
        PeriodicTimer { driver: self, channel }
    }
}

// ── One-shot timer ────────────────────────────────────────────────

/// One-shot timer wrapper.
///
/// Counts down from a loaded value and stops when it reaches zero.
pub struct OneShotTimer<'a> {
    driver: &'a TimerDriver<'a>,
    channel: usize,
}

impl OneShotTimer<'_> {
    /// Start the one-shot timer with a count value.
    ///
    /// `count` is in timer clock ticks (TCXO = 24MHz → 1 tick ≈ 41.7ns).
    pub fn start(&mut self, count: u32) {
        self.driver.configure(self.channel, TimerMode::OneShot, count);
        self.driver.enable(self.channel);
    }

    /// Start the timer for the given duration in microseconds.
    ///
    /// Max duration: ~178 seconds at 24MHz (u32 ticks limit).
    /// For longer durations, use `start_millis()` or loop `start_micros()`.
    pub fn start_micros(&mut self, us: u32) {
        let ticks64 = TIMER_CLOCK_HZ as u64 * us as u64 / 1_000_000;
        // Clamp to u32 max — delays longer than ~178s at 24MHz
        let ticks = if ticks64 > u32::MAX as u64 { u32::MAX } else { ticks64 as u32 };
        self.start(ticks);
    }

    /// Start the timer for the given duration in milliseconds.
    ///
    /// Max duration: ~178,956ms (~178s) at 24MHz.
    /// For longer durations, repeat this call in a loop.
    pub fn start_millis(&mut self, ms: u32) {
        let ticks64 = TIMER_CLOCK_HZ as u64 * ms as u64 / 1_000;
        let ticks = if ticks64 > u32::MAX as u64 { u32::MAX } else { ticks64 as u32 };
        self.start(ticks);
    }

    /// Check if the timer has expired.
    pub fn expired(&self) -> bool {
        self.driver.interrupt_pending(self.channel)
    }

    /// Wait for the timer to expire (busy-loop).
    pub fn wait(&self) {
        while !self.expired() {}
        self.driver.clear_interrupt(self.channel);
    }

    /// Get the current counter value.
    pub fn current(&self) -> u32 {
        self.driver.current_value(self.channel)
    }

    /// Stop the timer.
    pub fn stop(&self) {
        self.driver.disable(self.channel);
    }

    /// Clear the interrupt flag.
    pub fn clear(&self) {
        self.driver.clear_interrupt(self.channel);
    }
}

impl embedded_hal::delay::DelayNs for OneShotTimer<'_> {
    fn delay_ns(&mut self, ns: u32) {
        let ticks64 = (TIMER_CLOCK_HZ as u64 * ns as u64) / 1_000_000_000;
        let ticks = if ticks64 > u32::MAX as u64 { u32::MAX } else { ticks64 as u32 };
        if ticks > 0 {
            self.start(ticks);
            self.wait();
        }
    }

    fn delay_us(&mut self, us: u32) {
        self.start_micros(us);
        self.wait();
    }

    fn delay_ms(&mut self, ms: u32) {
        self.start_millis(ms);
        self.wait();
    }
}

// ── Periodic timer ────────────────────────────────────────────────

/// Periodic timer wrapper.
///
/// Counts down and automatically reloads, generating an interrupt each cycle.
pub struct PeriodicTimer<'a> {
    driver: &'a TimerDriver<'a>,
    channel: usize,
}

impl PeriodicTimer<'_> {
    /// Start the periodic timer with the given period in ticks.
    pub fn start(&mut self, period: u32) {
        self.driver.configure(self.channel, TimerMode::Periodic, period);
        self.driver.enable(self.channel);
    }

    /// Start the periodic timer with the period in microseconds.
    pub fn start_micros(&mut self, us: u32) {
        let ticks64 = TIMER_CLOCK_HZ as u64 * us as u64 / 1_000_000;
        let ticks = if ticks64 > u32::MAX as u64 { u32::MAX } else { ticks64 as u32 };
        self.start(ticks);
    }

    /// Check if a period has elapsed (interrupt pending).
    pub fn tick_elapsed(&self) -> bool {
        self.driver.interrupt_pending(self.channel)
    }

    /// Wait for the next timer tick.
    pub fn wait_tick(&self) {
        while !self.tick_elapsed() {}
        self.driver.clear_interrupt(self.channel);
    }

    /// Stop the timer.
    pub fn stop(&self) {
        self.driver.disable(self.channel);
    }

    /// Get the current counter value.
    pub fn current(&self) -> u32 {
        self.driver.current_value(self.channel)
    }

    /// Clear the tick interrupt flag.
    pub fn clear_tick(&self) {
        self.driver.clear_interrupt(self.channel);
    }
}

// ── Tests ──────────────────────────────────────────────────────

#[cfg(all(test, not(target_arch = "riscv32")))]
mod tests {
    use crate::soc::chip::TIMER_CLOCK_HZ;

    // The timer counts at the TCXO crystal clock (TIMER_CLOCK_HZ = 24 MHz), so
    // there are TICKS_PER_US ticks per microsecond and the u32 one-shot caps at
    // MAX_SAFE_US (≈178 s at 24 MHz) before the conversion saturates.
    const TICKS_PER_US: u64 = TIMER_CLOCK_HZ as u64 / 1_000_000;
    const MAX_SAFE_US: u64 = u32::MAX as u64 / TICKS_PER_US;

    fn ticks_for_us(us: u64) -> u32 {
        let t = TIMER_CLOCK_HZ as u64 * us / 1_000_000;
        if t > u32::MAX as u64 { u32::MAX } else { t as u32 }
    }

    #[test]
    fn oneshot_overflow_clamps() {
        // µs beyond the safe range saturates to u32::MAX (no wrap, no panic).
        assert_eq!(ticks_for_us(MAX_SAFE_US + 1_000_000), u32::MAX);
    }

    #[test]
    fn small_value_does_not_clamp() {
        // 100 µs → 100 * TICKS_PER_US ticks, well within u32.
        assert_eq!(ticks_for_us(100), (100 * TICKS_PER_US) as u32);
    }

    #[test]
    fn max_safe_value_not_clamped() {
        let ticks64 = TIMER_CLOCK_HZ as u64 * MAX_SAFE_US / 1_000_000;
        assert!(ticks64 <= u32::MAX as u64);
        assert_eq!(ticks_for_us(MAX_SAFE_US), ticks64 as u32);
    }
}

// ── Property-based fuzz tests ──────────────────────────────────

#[cfg(all(test, not(target_arch = "riscv32")))]
mod proptests {
    use crate::soc::chip::TIMER_CLOCK_HZ;
    use proptest::prelude::*;

    const MAX_SAFE_US: u64 = u32::MAX as u64 / (TIMER_CLOCK_HZ as u64 / 1_000_000);

    fn ticks64(us: u64) -> u64 {
        TIMER_CLOCK_HZ as u64 * us / 1_000_000
    }

    proptest! {
        /// Fuzz: ticks calculation + clamp never panics for any u32 µs input.
        #[test]
        fn ticks_never_panics(us in any::<u32>()) {
            let t = ticks64(us as u64);
            let _ = if t > u32::MAX as u64 { u32::MAX } else { t as u32 };
        }

        /// Fuzz: µs within the safe range never overflow u32.
        #[test]
        fn safe_range_not_clamped(us in 0u64..=MAX_SAFE_US) {
            prop_assert!(ticks64(us) <= u32::MAX as u64, "safe us={} -> ticks64={}", us, ticks64(us));
        }

        /// Fuzz: µs beyond the safe range always overflow u32 (and thus clamp).
        #[test]
        fn overflow_always_clamps(us in (MAX_SAFE_US + 1)..=u32::MAX as u64) {
            prop_assert!(ticks64(us) > u32::MAX as u64);
        }
    }
}

// ── Async (embedded-hal-async) ──────────────────────────────────────────────
#[cfg(feature = "async")]
mod asynch_impl {
    use super::{TIMER_CLOCK_HZ, Timer, TimerDriver, TimerMode};
    use crate::asynch::IrqSignal;
    use crate::interrupt::{self, Interrupt};
    use core::future::Future;
    use core::pin::Pin;
    use core::task::{Context, Poll};

    static TIMER_SIGNAL: [IrqSignal; 3] = [IrqSignal::new(), IrqSignal::new(), IrqSignal::new()];

    fn ch_irq(ch: usize) -> Interrupt {
        match ch {
            0 => Interrupt::TIMER_INT0,
            1 => Interrupt::TIMER_INT1,
            _ => Interrupt::TIMER_INT2,
        }
    }

    /// Timer trap-handler hook: stop channel `ch`'s one-shot, clear its
    /// interrupt, and wake the awaiting [`AsyncDelay`] future. Call this from the
    /// machine-interrupt trap when `mcause` is TIMER_INT0..2 (IRQ 26..28). The
    /// EOI clears `mip`, so no `LOCIPCLR` is needed for these MIE-class lines.
    pub fn on_interrupt(ch: usize) {
        // SAFETY: RMW of the timer MMIO block. The AsyncDelay owns the peripheral
        // handle, but the ISR uses raw register access (the standard ISR pattern).
        let r = unsafe { &*Timer::ptr() };
        match ch {
            0 => {
                let prev = r.timer0_control(0).read().bits();
                r.timer0_control(0).write(|w| unsafe { w.bits(prev & !1) }); // stop (clear EN)
                let _ = r.timer0_eoi(0).read().bits(); // EOI (read-clear)
            }
            1 => {
                let prev = r.timer0_control(1).read().bits();
                r.timer0_control(1).write(|w| unsafe { w.bits(prev & !1) });
                let _ = r.timer0_eoi(1).read().bits();
            }
            _ => {
                let prev = r.timer0_control(2).read().bits();
                r.timer0_control(2).write(|w| unsafe { w.bits(prev & !1) });
                let _ = r.timer0_eoi(2).read().bits();
            }
        }
        TIMER_SIGNAL[ch].signal();
    }

    /// Async delay backed by one WS63 TIMER channel (one-shot + completion IRQ).
    ///
    /// Implements [`embedded_hal_async::delay::DelayNs`]: each `delay_*().await`
    /// arms the channel one-shot, parks the task until the channel's IRQ fires,
    /// then returns. The app must route the timer trap to [`on_interrupt`] and
    /// have enabled global interrupts (see `ws63-examples/async_delay`).
    pub struct AsyncDelay<'d> {
        driver: TimerDriver<'d>,
        channel: usize,
    }

    impl<'d> AsyncDelay<'d> {
        /// Create an async delay on `channel` (0..=2).
        pub fn new(timer: Timer<'d>, channel: usize) -> Self {
            Self { driver: TimerDriver::new(timer), channel }
        }

        async fn delay_ticks(&mut self, ticks: u32) {
            let ch = self.channel;
            TIMER_SIGNAL[ch].reset();
            self.driver.clear_interrupt(ch);
            self.driver.configure(ch, TimerMode::OneShot, ticks.max(1));
            // SAFETY: enabling a known, fixed WS63 timer IRQ line.
            unsafe { interrupt::enable(ch_irq(ch)) };
            self.driver.enable(ch);
            DelayFuture { ch }.await;
        }
    }

    struct DelayFuture {
        ch: usize,
    }

    impl Future for DelayFuture {
        type Output = ();
        fn poll(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<()> {
            if TIMER_SIGNAL[self.ch].take_fired() {
                Poll::Ready(())
            } else {
                TIMER_SIGNAL[self.ch].register(cx.waker());
                Poll::Pending
            }
        }
    }

    impl embedded_hal_async::delay::DelayNs for AsyncDelay<'_> {
        async fn delay_ns(&mut self, ns: u32) {
            let ticks = (TIMER_CLOCK_HZ as u64 * ns as u64 / 1_000_000_000) as u32;
            self.delay_ticks(ticks).await;
        }
    }
}

#[cfg(feature = "async")]
pub use asynch_impl::{AsyncDelay, on_interrupt};
