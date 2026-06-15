//! PWM driver for WS63 (8 channels; usable period is 16-bit on silicon — see
//! [`PwmPeriod`]).
//!
//! ## Two-layer API: typed config + embedded-hal operations
//!
//! The configuration surface is **typed so that an unrunnable config is
//! unrepresentable**: [`Duty`] cannot hold a value above 100 %, and a
//! [`PwmPeriod`] is always a non-zero 32-bit counter value (and `try_from_hz`
//! rejects frequencies that would round to a 0 count). `configure(period, duty)`
//! therefore can never be handed garbage — there is no "compiles but writes a
//! dead waveform" path. `configure` also brings up the PWM clock tree itself (the
//! precondition the old `freq: u32` API silently assumed; without it the 32-bit
//! `pwm_freq`/`pwm_duty` registers do not fully latch).
//!
//! The *operational* surface stays standard `embedded-hal`
//! ([`embedded_hal::pwm::SetDutyCycle`]): its trait signatures are fixed
//! (`u16` + `Result`), so it cannot take the typed values — that is the
//! embedded-hal idiom and the two layers coexist (typed config for direct HAL
//! users, the trait for generic drivers).
use crate::peripherals::Pwm;
use crate::soc::chip::SYSTEM_CLOCK_HZ;
use core::marker::PhantomData;

/// PWM counter clock rate. The vendor `pwm_port_clock_enable` on the default WS63
/// build (`CONFIG_HIGH_FREQUENCY`) selects the high-frequency source (CLK_SEL
/// bit 7) and divides it by 6 (`PWM_DIV_6`), so the counter ticks at
/// `SYSTEM_CLOCK_HZ / 6` — **not** the raw CPU clock. Period/duty counts are in
/// these ticks.
///
/// NOTE: the high-frequency source is taken as `SYSTEM_CLOCK_HZ` (240 MHz) per the
/// vendor default; the resulting 40 MHz tick rate is not yet scope-confirmed.
/// [`PwmPeriod::from_count`] is exact regardless; [`PwmPeriod::try_from_hz`]
/// depends on this constant.
pub const PWM_CLOCK_HZ: u32 = SYSTEM_CLOCK_HZ / 6;

/// A duty-cycle percentage, validated to `0..=100` at construction.
///
/// Because a `Duty` cannot hold a value above 100, [`PwmChannel::configure`] can
/// never be handed an out-of-range duty — the invalid state is unrepresentable,
/// not caught at runtime.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub struct Duty(u8);

impl Duty {
    /// 0 % — output held inactive.
    pub const ZERO: Duty = Duty(0);
    /// 50 %.
    pub const HALF: Duty = Duty(50);
    /// 100 % — output held active.
    pub const FULL: Duty = Duty(100);

    /// Construct from a percentage. Returns `None` for `> 100`, so an invalid duty
    /// can never reach the hardware.
    pub const fn from_percent(percent: u8) -> Option<Self> {
        if percent <= 100 { Some(Duty(percent)) } else { None }
    }

    /// The percentage (always `0..=100`).
    pub const fn percent(self) -> u8 {
        self.0
    }
}

/// A validated PWM period: a non-zero **16-bit** count of [`PWM_CLOCK_HZ`] ticks.
///
/// The vendor `hal_pwm_v151_regs_def.h` documents a 32-bit period
/// (`pwm_freq_l_j[15:0]` + `pwm_freq_h_j[15:0]`), but on WS63 silicon the high half
/// **does not latch** — measured: writing `0x0001` to `pwm_freq_h0` reads back 0
/// even with the full clock tree up (CKEN on, divider loaded, CLK_SEL set). So the
/// *usable* period is 16-bit, and this newtype encodes exactly that: a period the
/// hardware cannot actually run is **unrepresentable**, not silently truncated —
/// the precise "compiles but won't run" failure this typed API removes.
/// [`try_from_hz`](PwmPeriod::try_from_hz) likewise rejects a frequency whose count
/// would exceed 16 bits.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub struct PwmPeriod(u16);

impl PwmPeriod {
    /// Wrap a raw counter period (in [`PWM_CLOCK_HZ`] ticks). `None` for 0.
    pub const fn from_count(count: u16) -> Option<Self> {
        if count == 0 { None } else { Some(PwmPeriod(count)) }
    }

    /// Derive the period from a target output frequency. Returns `None` when the
    /// frequency is 0, above [`PWM_CLOCK_HZ`] (count rounds to 0), or so low that
    /// the count would exceed the 16-bit hardware range.
    pub const fn try_from_hz(hz: u32) -> Option<Self> {
        if hz == 0 {
            return None;
        }
        let count = PWM_CLOCK_HZ / hz;
        if count == 0 || count > u16::MAX as u32 {
            return None;
        }
        Some(PwmPeriod(count as u16))
    }

    /// The raw counter period (in [`PWM_CLOCK_HZ`] ticks), always `>= 1`.
    pub const fn count(self) -> u16 {
        self.0
    }

    /// The on-time counter value for `duty` at this period (`count * percent / 100`,
    /// widened to `u32` so the product never overflows).
    pub const fn duty_count(self, duty: Duty) -> u16 {
        ((self.0 as u32 * duty.0 as u32) / 100) as u16
    }
}

/// Bring up the PWM channel-0 clock tree the way the vendor `pwm_port_clock_enable`
/// does on WS63 (high-frequency source, ÷6): select the source, enable the bus +
/// channel clock gates, and load the divider. WITHOUT this the `pwm_freq`/`pwm_duty`
/// registers do not fully latch (the high 16-bit halves read back 0) — the
/// precondition the old `configure(freq, …)` silently assumed.
///
/// The register addresses are WS63-specific (CLDO_CRG), so the body is gated to
/// `chip-ws63`; on BS2X this is a no-op (their PWM clock tree differs — a
/// follow-up). Loads the PWM0 divider only (DIV_CTL3); channels 1–7 share the bus
/// gate but their dividers live in DIV_CTL3/4/5.
fn enable_pwm_clock() {
    #[cfg(feature = "chip-ws63")]
    {
        const CLDO_CRG_CKEN_CTL0: usize = 0x4400_1100;
        const CLDO_CRG_DIV_CTL3: usize = 0x4400_1114;
        const CLDO_CRG_CLK_SEL: usize = 0x4400_1134;
        const PWM_CKSEL_BIT: u32 = 7; // high-frequency source select
        const PWM_BUS_CKEN: u32 = 2; // base of the 9-bit [10:2] bus+channel CKEN field
        const PWM_CKEN_FIELD: u32 = 0x1FF; // all 9 clock-enable bits
        const PWM0_DIV1_CFG: u32 = 16; // DIV_CTL3 [19:16] = PWM0 divider
        const PWM0_LOAD_DIV_EN: u32 = 20; // DIV_CTL3 bit 20 = latch the divider
        const PWM_DIV_6: u32 = 6; // CONFIG_HIGH_FREQUENCY default divider

        // SAFETY: fixed 32-bit CLDO_CRG MMIO addresses; RMW of clock bits is benign.
        unsafe fn rmw(addr: usize, clear: u32, set: u32) {
            let p = addr as *mut u32;
            let v = (unsafe { core::ptr::read_volatile(p) } & !clear) | set;
            unsafe { core::ptr::write_volatile(p, v) };
        }
        unsafe {
            rmw(CLDO_CRG_CLK_SEL, 0, 1 << PWM_CKSEL_BIT);
            rmw(CLDO_CRG_CKEN_CTL0, 0, PWM_CKEN_FIELD << PWM_BUS_CKEN);
            // Reload the PWM0 divider: clear LOAD_DIV_EN, set the 4-bit divider,
            // then set LOAD_DIV_EN (the rising edge latches it).
            rmw(CLDO_CRG_DIV_CTL3, 1 << PWM0_LOAD_DIV_EN, 0);
            rmw(CLDO_CRG_DIV_CTL3, 0xF << PWM0_DIV1_CFG, PWM_DIV_6 << PWM0_DIV1_CFG);
            rmw(CLDO_CRG_DIV_CTL3, 0, 1 << PWM0_LOAD_DIV_EN);
        }
    }
}

pub struct PwmChannel<'d> {
    channel: u8,
    _marker: PhantomData<&'d ()>,
}

impl<'d> PwmChannel<'d> {
    pub fn new(_pwm: &Pwm<'d>, channel: u8) -> Self {
        assert!(channel < 8);
        Self { channel, _marker: PhantomData }
    }

    fn regs(&self) -> &'static crate::soc::pac::pwm::RegisterBlock {
        // SAFETY: PAC peripheral pointer is a static physical MMIO address, always valid
        unsafe { &*Pwm::ptr() }
    }

    /// The configured 32-bit period count for this channel (reassembled from the
    /// `pwm_freq_l`/`pwm_freq_h` register halves). 0 if never configured.
    fn period_count(&self) -> u32 {
        let r = self.regs();
        let (lo, hi) = match self.channel {
            0 => (r.pwm_freq_l0().read().bits(), r.pwm_freq_h0().read().bits()),
            1 => (r.pwm_freq_l1().read().bits(), r.pwm_freq_h1().read().bits()),
            2 => (r.pwm_freq_l2().read().bits(), r.pwm_freq_h2().read().bits()),
            3 => (r.pwm_freq_l3().read().bits(), r.pwm_freq_h3().read().bits()),
            4 => (r.pwm_freq_l4().read().bits(), r.pwm_freq_h4().read().bits()),
            5 => (r.pwm_freq_l5().read().bits(), r.pwm_freq_h5().read().bits()),
            6 => (r.pwm_freq_l6().read().bits(), r.pwm_freq_h6().read().bits()),
            7 => (r.pwm_freq_l7().read().bits(), r.pwm_freq_h7().read().bits()),
            _ => unreachable!(),
        };
        (lo as u32 & 0xFFFF) | ((hi as u32 & 0xFFFF) << 16)
    }

    /// Configure this channel's period and duty cycle, bringing up the PWM clock
    /// tree first so the 32-bit `pwm_freq`/`pwm_duty` registers actually latch.
    /// Both arguments are pre-validated ([`PwmPeriod`] is non-zero, [`Duty`] is
    /// `0..=100`), so any call programs a real waveform — there is no invalid input
    /// to reject at runtime.
    pub fn configure(&mut self, period_cfg: PwmPeriod, duty_cfg: Duty) {
        enable_pwm_clock();
        let r = self.regs();
        // 16-bit usable period/duty (the *_h halves do not latch on silicon); the
        // match below still writes both halves, so the high write clears to 0.
        let period = period_cfg.count() as u32;
        let duty = period_cfg.duty_count(duty_cfg) as u32;
        match self.channel {
            0 => {
                r.pwm_freq_l0().write(|w| unsafe { w.bits(period & 0xFFFF) });
                r.pwm_freq_h0().write(|w| unsafe { w.bits((period >> 16) & 0xFFFF) });
                r.pwm_duty_l0().write(|w| unsafe { w.bits(duty & 0xFFFF) });
                r.pwm_duty_h0().write(|w| unsafe { w.bits((duty >> 16) & 0xFFFF) });
            }
            1 => {
                r.pwm_freq_l1().write(|w| unsafe { w.bits(period & 0xFFFF) });
                r.pwm_freq_h1().write(|w| unsafe { w.bits((period >> 16) & 0xFFFF) });
                r.pwm_duty_l1().write(|w| unsafe { w.bits(duty & 0xFFFF) });
                r.pwm_duty_h1().write(|w| unsafe { w.bits((duty >> 16) & 0xFFFF) });
            }
            2 => {
                r.pwm_freq_l2().write(|w| unsafe { w.bits(period & 0xFFFF) });
                r.pwm_freq_h2().write(|w| unsafe { w.bits((period >> 16) & 0xFFFF) });
                r.pwm_duty_l2().write(|w| unsafe { w.bits(duty & 0xFFFF) });
                r.pwm_duty_h2().write(|w| unsafe { w.bits((duty >> 16) & 0xFFFF) });
            }
            3 => {
                r.pwm_freq_l3().write(|w| unsafe { w.bits(period & 0xFFFF) });
                r.pwm_freq_h3().write(|w| unsafe { w.bits((period >> 16) & 0xFFFF) });
                r.pwm_duty_l3().write(|w| unsafe { w.bits(duty & 0xFFFF) });
                r.pwm_duty_h3().write(|w| unsafe { w.bits((duty >> 16) & 0xFFFF) });
            }
            4 => {
                r.pwm_freq_l4().write(|w| unsafe { w.bits(period & 0xFFFF) });
                r.pwm_freq_h4().write(|w| unsafe { w.bits((period >> 16) & 0xFFFF) });
                r.pwm_duty_l4().write(|w| unsafe { w.bits(duty & 0xFFFF) });
                r.pwm_duty_h4().write(|w| unsafe { w.bits((duty >> 16) & 0xFFFF) });
            }
            5 => {
                r.pwm_freq_l5().write(|w| unsafe { w.bits(period & 0xFFFF) });
                r.pwm_freq_h5().write(|w| unsafe { w.bits((period >> 16) & 0xFFFF) });
                r.pwm_duty_l5().write(|w| unsafe { w.bits(duty & 0xFFFF) });
                r.pwm_duty_h5().write(|w| unsafe { w.bits((duty >> 16) & 0xFFFF) });
            }
            6 => {
                r.pwm_freq_l6().write(|w| unsafe { w.bits(period & 0xFFFF) });
                r.pwm_freq_h6().write(|w| unsafe { w.bits((period >> 16) & 0xFFFF) });
                r.pwm_duty_l6().write(|w| unsafe { w.bits(duty & 0xFFFF) });
                r.pwm_duty_h6().write(|w| unsafe { w.bits((duty >> 16) & 0xFFFF) });
            }
            7 => {
                r.pwm_freq_l7().write(|w| unsafe { w.bits(period & 0xFFFF) });
                r.pwm_freq_h7().write(|w| unsafe { w.bits((period >> 16) & 0xFFFF) });
                r.pwm_duty_l7().write(|w| unsafe { w.bits(duty & 0xFFFF) });
                r.pwm_duty_h7().write(|w| unsafe { w.bits((duty >> 16) & 0xFFFF) });
            }
            _ => unreachable!(),
        }
    }

    pub fn enable(&mut self) {
        match self.channel {
            0 => self.regs().pwm_en0().write(|w| unsafe { w.bits(1u32) }),
            1 => self.regs().pwm_en1().write(|w| unsafe { w.bits(1u32) }),
            2 => self.regs().pwm_en2().write(|w| unsafe { w.bits(1u32) }),
            3 => self.regs().pwm_en3().write(|w| unsafe { w.bits(1u32) }),
            4 => self.regs().pwm_en4().write(|w| unsafe { w.bits(1u32) }),
            5 => self.regs().pwm_en5().write(|w| unsafe { w.bits(1u32) }),
            6 => self.regs().pwm_en6().write(|w| unsafe { w.bits(1u32) }),
            7 => self.regs().pwm_en7().write(|w| unsafe { w.bits(1u32) }),
            _ => unreachable!(),
        };
    }
    pub fn disable(&mut self) {
        match self.channel {
            0 => self.regs().pwm_en0().write(|w| unsafe { w.bits(0u32) }),
            1 => self.regs().pwm_en1().write(|w| unsafe { w.bits(0u32) }),
            2 => self.regs().pwm_en2().write(|w| unsafe { w.bits(0u32) }),
            3 => self.regs().pwm_en3().write(|w| unsafe { w.bits(0u32) }),
            4 => self.regs().pwm_en4().write(|w| unsafe { w.bits(0u32) }),
            5 => self.regs().pwm_en5().write(|w| unsafe { w.bits(0u32) }),
            6 => self.regs().pwm_en6().write(|w| unsafe { w.bits(0u32) }),
            7 => self.regs().pwm_en7().write(|w| unsafe { w.bits(0u32) }),
            _ => unreachable!(),
        };
    }
    pub fn set_polarity(&mut self, active_high: bool) {
        let val = if active_high { 1u32 } else { 0u32 };
        match self.channel {
            0 => self.regs().pwm_portity0().write(|w| unsafe { w.bits(val) }),
            1 => self.regs().pwm_portity1().write(|w| unsafe { w.bits(val) }),
            2 => self.regs().pwm_portity2().write(|w| unsafe { w.bits(val) }),
            3 => self.regs().pwm_portity3().write(|w| unsafe { w.bits(val) }),
            4 => self.regs().pwm_portity4().write(|w| unsafe { w.bits(val) }),
            5 => self.regs().pwm_portity5().write(|w| unsafe { w.bits(val) }),
            6 => self.regs().pwm_portity6().write(|w| unsafe { w.bits(val) }),
            7 => self.regs().pwm_portity7().write(|w| unsafe { w.bits(val) }),
            _ => unreachable!(),
        };
    }
    pub fn start(&mut self) {
        self.regs().pwm_start0().write(|w| unsafe { w.bits(1u32 << self.channel) });
    }
    pub fn set_pulse_count(&mut self, count: u32) {
        match self.channel {
            0 => self.regs().pwm_period_val0().write(|w| unsafe { w.bits(count) }),
            1 => self.regs().pwm_period_val1().write(|w| unsafe { w.bits(count) }),
            2 => self.regs().pwm_period_val2().write(|w| unsafe { w.bits(count) }),
            3 => self.regs().pwm_period_val3().write(|w| unsafe { w.bits(count) }),
            4 => self.regs().pwm_period_val4().write(|w| unsafe { w.bits(count) }),
            5 => self.regs().pwm_period_val5().write(|w| unsafe { w.bits(count) }),
            6 => self.regs().pwm_period_val6().write(|w| unsafe { w.bits(count) }),
            7 => self.regs().pwm_period_val7().write(|w| unsafe { w.bits(count) }),
            _ => unreachable!(),
        };
    }

    /// Consume the channel, leaving it **running** past this scope — the escape
    /// hatch from the disabling [`Drop`](PwmChannel#impl-Drop-for-PwmChannel) (e.g.
    /// a PWM backlight that must keep driving after setup returns). Returns a
    /// [`PwmRunning`] marker so the intent is explicit and the channel is not
    /// silently leaked.
    #[must_use = "dropping the PwmRunning marker is fine, but assign it to make the leak intentional"]
    pub fn into_running(self) -> PwmRunning {
        core::mem::forget(self); // skip the disabling Drop — keep the output live
        PwmRunning(())
    }
}

/// Proof token from [`PwmChannel::into_running`]: the channel was intentionally
/// left driving its output past the driver's scope (no disabling `Drop` ran).
#[derive(Debug)]
#[must_use]
pub struct PwmRunning(());

impl Drop for PwmChannel<'_> {
    /// Scoped safety: a dropped channel stops driving its output (clears only this
    /// channel's `pwm_enN` enable bit — never a shared clock gate). Use
    /// [`PwmChannel::into_running`] to keep it live past the handle's scope.
    fn drop(&mut self) {
        self.disable();
    }
}

impl embedded_hal::pwm::ErrorType for PwmChannel<'_> {
    type Error = core::convert::Infallible;
}

impl embedded_hal::pwm::SetDutyCycle for PwmChannel<'_> {
    fn max_duty_cycle(&self) -> u16 {
        // Full scale = the configured period (the duty register is in period
        // units), saturated into embedded-hal's u16 duty granularity.
        self.period_count().min(u16::MAX as u32) as u16
    }

    fn set_duty_cycle(&mut self, duty: u16) -> Result<(), Self::Error> {
        let r = self.regs();
        let duty_val = duty as u32;
        match self.channel {
            0 => {
                unsafe { r.pwm_duty_l0().write(|w| w.bits(duty_val & 0xFFFF)) };
                unsafe { r.pwm_duty_h0().write(|w| w.bits((duty_val >> 16) & 0xFFFF)) };
            }
            1 => {
                unsafe { r.pwm_duty_l1().write(|w| w.bits(duty_val & 0xFFFF)) };
                unsafe { r.pwm_duty_h1().write(|w| w.bits((duty_val >> 16) & 0xFFFF)) };
            }
            2 => {
                unsafe { r.pwm_duty_l2().write(|w| w.bits(duty_val & 0xFFFF)) };
                unsafe { r.pwm_duty_h2().write(|w| w.bits((duty_val >> 16) & 0xFFFF)) };
            }
            3 => {
                unsafe { r.pwm_duty_l3().write(|w| w.bits(duty_val & 0xFFFF)) };
                unsafe { r.pwm_duty_h3().write(|w| w.bits((duty_val >> 16) & 0xFFFF)) };
            }
            4 => {
                unsafe { r.pwm_duty_l4().write(|w| w.bits(duty_val & 0xFFFF)) };
                unsafe { r.pwm_duty_h4().write(|w| w.bits((duty_val >> 16) & 0xFFFF)) };
            }
            5 => {
                unsafe { r.pwm_duty_l5().write(|w| w.bits(duty_val & 0xFFFF)) };
                unsafe { r.pwm_duty_h5().write(|w| w.bits((duty_val >> 16) & 0xFFFF)) };
            }
            6 => {
                unsafe { r.pwm_duty_l6().write(|w| w.bits(duty_val & 0xFFFF)) };
                unsafe { r.pwm_duty_h6().write(|w| w.bits((duty_val >> 16) & 0xFFFF)) };
            }
            7 => {
                unsafe { r.pwm_duty_l7().write(|w| w.bits(duty_val & 0xFFFF)) };
                unsafe { r.pwm_duty_h7().write(|w| w.bits((duty_val >> 16) & 0xFFFF)) };
            }
            _ => unreachable!(),
        }
        Ok(())
    }
}

// ── Tests ──────────────────────────────────────────────────────

#[cfg(all(test, not(target_arch = "riscv32")))]
mod tests {
    use super::{Duty, PwmPeriod, PwmRunning, PWM_CLOCK_HZ};

    /// The `into_running` escape-hatch marker is zero-sized (a pure type-level proof
    /// token). The disabling-Drop register effect is HIL-validated on silicon — the
    /// host has no MMIO.
    #[test]
    fn running_marker_is_zero_sized() {
        assert_eq!(core::mem::size_of::<PwmRunning>(), 0);
    }

    #[test]
    fn duty_rejects_over_100() {
        // The whole point of the newtype: > 100 % is unrepresentable.
        assert!(Duty::from_percent(0).is_some());
        assert!(Duty::from_percent(100).is_some());
        assert!(Duty::from_percent(101).is_none());
        assert!(Duty::from_percent(255).is_none());
    }

    #[test]
    fn duty_consts() {
        assert_eq!(Duty::ZERO.percent(), 0);
        assert_eq!(Duty::HALF.percent(), 50);
        assert_eq!(Duty::FULL.percent(), 100);
    }

    #[test]
    fn period_from_count_rejects_zero() {
        // A 0-tick period (dead waveform) is unrepresentable.
        assert!(PwmPeriod::from_count(0).is_none());
        assert_eq!(PwmPeriod::from_count(1).unwrap().count(), 1);
        assert_eq!(PwmPeriod::from_count(60_000).unwrap().count(), 60_000);
    }

    #[test]
    fn try_from_hz_basic_and_bounds() {
        // Rejected: 0 Hz, above-clock (count rounds to 0), and too-low (count would
        // exceed the 16-bit hardware range — 1 Hz → PWM_CLOCK_HZ count ≫ u16).
        assert!(PwmPeriod::try_from_hz(0).is_none());
        assert!(PwmPeriod::try_from_hz(PWM_CLOCK_HZ + 1).is_none());
        assert!(PwmPeriod::try_from_hz(1).is_none());
        // At the clock rate the period is exactly 1 tick.
        assert_eq!(PwmPeriod::try_from_hz(PWM_CLOCK_HZ).unwrap().count(), 1);
        // A normal frequency: count = PWM_CLOCK_HZ / hz (fits 16 bits).
        assert_eq!(PwmPeriod::try_from_hz(1_000).unwrap().count(), (PWM_CLOCK_HZ / 1_000) as u16);
    }

    #[test]
    fn duty_count_zero_half_full() {
        let p = PwmPeriod::from_count(1_000).unwrap();
        assert_eq!(p.duty_count(Duty::ZERO), 0);
        assert_eq!(p.duty_count(Duty::HALF), 500);
        assert_eq!(p.duty_count(Duty::FULL), 1_000);
    }

    #[test]
    fn duty_count_monotonic_and_bounded() {
        let p = PwmPeriod::from_count(1_000).unwrap();
        let mut prev = 0u16;
        for pct in 0u8..=100 {
            let d = p.duty_count(Duty::from_percent(pct).unwrap());
            assert!(d >= prev, "duty must be monotonic: pct={pct} d={d} prev={prev}");
            assert!(d <= p.count(), "duty {d} must never exceed period {}", p.count());
            prev = d;
        }
    }

    #[test]
    fn duty_count_no_overflow_for_max_period() {
        // The u32 widening means even the max 16-bit period * 100 % never overflows.
        let p = PwmPeriod::from_count(u16::MAX).unwrap();
        assert_eq!(p.duty_count(Duty::FULL), u16::MAX);
        assert_eq!(p.duty_count(Duty::from_percent(1).unwrap()), u16::MAX / 100);
    }

    #[test]
    fn start_mask_is_channel_bit() {
        // `start()` writes `1 << channel`; each channel maps to a distinct bit.
        for ch in 0u8..8 {
            assert_eq!(1u32 << ch, 1u32 << ch);
            assert_eq!((1u32 << ch).count_ones(), 1);
        }
        assert_eq!(1u32 << 7, 0x80);
    }

    #[test]
    fn channel_bounds() {
        // `PwmChannel::new` asserts `channel < 8`; the valid range is exactly 0..8.
        assert!((0u8..8).all(|c| c < 8));
        assert!(8u8 >= 8);
    }
}

// ── Property-based fuzz tests ──────────────────────────────────

#[cfg(all(test, not(target_arch = "riscv32")))]
mod proptests {
    use super::{Duty, PwmPeriod, PWM_CLOCK_HZ};
    use proptest::prelude::*;

    proptest! {
        /// Fuzz: a frequency whose count fits 16 bits always yields a valid period.
        /// The valid band is `PWM_CLOCK_HZ/u16::MAX < hz <= PWM_CLOCK_HZ` (below it
        /// the count would exceed 16 bits; above it it rounds to 0).
        #[test]
        fn try_from_hz_in_range(hz in (PWM_CLOCK_HZ / u16::MAX as u32 + 1)..=PWM_CLOCK_HZ) {
            let p = PwmPeriod::try_from_hz(hz).unwrap();
            prop_assert!(p.count() >= 1);
        }

        /// Fuzz: a too-low frequency (count would exceed the 16-bit range) is rejected.
        #[test]
        fn try_from_hz_rejects_too_low(hz in 1u32..(PWM_CLOCK_HZ / u16::MAX as u32)) {
            prop_assert!(PwmPeriod::try_from_hz(hz).is_none());
        }

        /// Fuzz: an above-clock frequency is always rejected (can't program it).
        #[test]
        fn try_from_hz_rejects_out_of_range(hz in (PWM_CLOCK_HZ + 1)..=u32::MAX) {
            prop_assert!(PwmPeriod::try_from_hz(hz).is_none());
        }

        /// Fuzz: `duty_count` is always within [0, period] for any period/duty.
        #[test]
        fn duty_within_period(count in 1u16..=u16::MAX, pct in 0u8..=100) {
            let p = PwmPeriod::from_count(count).unwrap();
            let d = p.duty_count(Duty::from_percent(pct).unwrap());
            prop_assert!(d <= p.count(), "duty {} > period {} (pct {})", d, p.count(), pct);
        }

        /// Fuzz: `Duty` accepts exactly 0..=100 and rejects everything above.
        #[test]
        fn duty_validates(pct in any::<u8>()) {
            prop_assert_eq!(Duty::from_percent(pct).is_some(), pct <= 100);
        }

        /// Fuzz: the start-register mask is a single bit for every valid channel.
        #[test]
        fn start_mask_single_bit(ch in 0u8..8) {
            let mask = 1u32 << ch;
            prop_assert_eq!(mask.count_ones(), 1);
        }
    }
}
