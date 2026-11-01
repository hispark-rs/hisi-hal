//! Timekeeping types for WS63 HAL.
//!
//! Provides `Instant`, `Duration`, and `Rate` types for timing operations.
//! Uses the TCXO 64-bit counter as the hardware time source.

#[cfg(not(feature = "chip-ws53"))]
use crate::peripherals::Tcxo;

/// A measurement of a monotonically non-decreasing clock.
/// Opaque and useful with `Duration`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub struct Instant(u64);

impl Instant {
    /// Returns an instant corresponding to "now".
    #[instability::unstable]
    #[cfg(not(feature = "chip-ws53"))]
    pub fn now() -> Self {
        // SAFETY: TCXO PAC pointer is a valid physical MMIO address (0x4400_04C0)
        let tcxo = unsafe { &*Tcxo::ptr() };
        // Latch the counter
        let status = tcxo.tcxo_status().read().bits();
        unsafe {
            tcxo.tcxo_status().write(|w| w.bits(status | 0x01));
        }
        while tcxo.tcxo_status().read().bits() & 0x10 == 0 {}
        // Read 64-bit counter
        let c0 = tcxo.tcxo_count0().read().bits() as u64;
        let c1 = tcxo.tcxo_count1().read().bits() as u64;
        let c2 = tcxo.tcxo_count2().read().bits() as u64;
        let c3 = tcxo.tcxo_count3().read().bits() as u64;
        Instant((c3 << 48) | (c2 << 32) | (c1 << 16) | c0)
    }

    /// Returns the amount of time elapsed since this instant.
    #[instability::unstable]
    #[cfg(not(feature = "chip-ws53"))]
    pub fn elapsed(&self) -> Duration {
        Instant::now().checked_duration_since(*self).unwrap_or(Duration::from_micros(0))
    }

    /// Returns the amount of time elapsed from another instant to this one.
    pub fn duration_since(&self, earlier: Instant) -> Duration {
        self.checked_duration_since(earlier).unwrap_or(Duration::from_micros(0))
    }

    /// Returns `Some(Duration)` if `earlier` is earlier than `self`, or `None` otherwise.
    pub fn checked_duration_since(&self, earlier: Instant) -> Option<Duration> {
        if self.0 >= earlier.0 { Some(Duration::from_micros(self.0 - earlier.0)) } else { None }
    }

    /// Returns the raw counter value.
    pub fn raw(&self) -> u64 {
        self.0
    }
}

/// A duration of time.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Default)]
pub struct Duration(u64); // microseconds

impl Duration {
    /// Create a duration from microseconds.
    pub const fn from_micros(micros: u64) -> Self {
        Duration(micros)
    }

    /// Create a duration from milliseconds.
    pub const fn from_millis(millis: u64) -> Self {
        Duration(millis * 1_000)
    }

    /// Create a duration from seconds.
    pub const fn from_secs(secs: u64) -> Self {
        Duration(secs * 1_000_000)
    }

    /// Return the total number of microseconds.
    pub const fn as_micros(&self) -> u64 {
        self.0
    }

    /// Return the total number of milliseconds.
    pub const fn as_millis(&self) -> u64 {
        self.0 / 1_000
    }

    /// Return the total number of seconds.
    pub const fn as_secs(&self) -> u64 {
        self.0 / 1_000_000
    }
}

impl core::ops::Add for Duration {
    type Output = Self;
    fn add(self, rhs: Self) -> Self {
        Duration(self.0 + rhs.0)
    }
}

impl core::ops::AddAssign for Duration {
    fn add_assign(&mut self, rhs: Self) {
        self.0 += rhs.0;
    }
}

impl core::ops::Sub for Duration {
    type Output = Self;
    fn sub(self, rhs: Self) -> Self {
        Duration(self.0.saturating_sub(rhs.0))
    }
}

/// A rate in Hz.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Rate(u32);

impl Rate {
    /// Create a rate from a value in Hz.
    pub const fn from_hz(hz: u32) -> Self {
        Rate(hz)
    }
    /// Create a rate from a value in kHz.
    pub const fn from_khz(khz: u32) -> Self {
        Rate(khz * 1_000)
    }
    /// Create a rate from a value in MHz.
    pub const fn from_mhz(mhz: u32) -> Self {
        Rate(mhz * 1_000_000)
    }
    /// Return the rate in Hz.
    pub const fn to_hz(&self) -> u32 {
        self.0
    }
    /// Return the rate in kHz.
    pub const fn to_khz(&self) -> u32 {
        self.0 / 1_000
    }
}

// ── Tests ────────────────────────────────────────────────────────

#[cfg(all(test, not(target_arch = "riscv32")))]
mod tests {
    use super::*;

    #[test]
    fn test_duration_constructors() {
        assert_eq!(Duration::from_micros(500).as_micros(), 500);
        assert_eq!(Duration::from_millis(1).as_micros(), 1000);
        assert_eq!(Duration::from_secs(1).as_micros(), 1_000_000);
        assert_eq!(Duration::from_micros(1500).as_millis(), 1);
    }

    #[test]
    fn test_duration_add_sub() {
        let a = Duration::from_micros(100);
        let b = Duration::from_micros(50);
        assert_eq!((a + b).as_micros(), 150);
        assert_eq!((a - b).as_micros(), 50);
        // Saturating subtract
        assert_eq!((b - a).as_micros(), 0);
    }

    #[test]
    fn test_duration_default() {
        assert_eq!(Duration::default().as_micros(), 0);
    }

    #[test]
    fn test_rate_constructors() {
        assert_eq!(Rate::from_hz(1000).to_hz(), 1000);
        assert_eq!(Rate::from_khz(1).to_hz(), 1000);
        assert_eq!(Rate::from_mhz(1).to_hz(), 1_000_000);
        assert_eq!(Rate::from_mhz(240).to_khz(), 240_000);
    }

    #[test]
    fn test_instant_checked_duration() {
        let early = Instant(100);
        let late = Instant(200);
        assert_eq!(late.checked_duration_since(early).unwrap().as_micros(), 100);
        assert!(early.checked_duration_since(late).is_none());
    }
}

// ── Property-based fuzz tests ──────────────────────────────────

#[cfg(all(test, not(target_arch = "riscv32")))]
mod proptests {
    use super::{Duration, Instant, Rate};
    use proptest::prelude::*;

    proptest! {
        /// Fuzz: ms→µs→ms round-trips exactly (from_millis multiplies by 1_000,
        /// as_millis divides by 1_000), as long as the µs product fits in u64.
        #[test]
        fn duration_millis_roundtrip(ms in 0u64..=(u64::MAX / 1_000)) {
            let d = Duration::from_millis(ms);
            prop_assert_eq!(d.as_micros(), ms * 1_000);
            prop_assert_eq!(d.as_millis(), ms);
        }

        /// Fuzz: s→µs→s round-trips exactly (from_secs * 1_000_000, as_secs / 1_000_000).
        #[test]
        fn duration_secs_roundtrip(s in 0u64..=(u64::MAX / 1_000_000)) {
            let d = Duration::from_secs(s);
            prop_assert_eq!(d.as_micros(), s * 1_000_000);
            prop_assert_eq!(d.as_secs(), s);
        }

        /// Fuzz: as_millis / as_secs are truncating-floor divisions of the raw µs;
        /// they never exceed the exact ratio and lose less than one unit.
        #[test]
        fn duration_unit_conversions_floor(us in any::<u64>()) {
            let d = Duration::from_micros(us);
            prop_assert_eq!(d.as_micros(), us);
            prop_assert_eq!(d.as_millis(), us / 1_000);
            prop_assert_eq!(d.as_secs(), us / 1_000_000);
            // Floor division: recovered µs never exceeds the original.
            prop_assert!(d.as_millis() * 1_000 <= us);
            prop_assert!(d.as_secs() * 1_000_000 <= us);
            // …and is within one whole unit of it.
            prop_assert!(us - d.as_millis() * 1_000 < 1_000);
            prop_assert!(us - d.as_secs() * 1_000_000 < 1_000_000);
        }

        /// Fuzz: Duration subtraction saturates at zero — it never wraps, no panic.
        /// (Add is plain wrapping arithmetic, so we only feed it non-overflowing inputs.)
        #[test]
        fn duration_sub_saturates(a in any::<u64>(), b in any::<u64>()) {
            let da = Duration::from_micros(a);
            let db = Duration::from_micros(b);
            let diff = (da - db).as_micros();
            prop_assert_eq!(diff, a.saturating_sub(b));
            // Saturating: when b >= a the result floors at 0, never underflows.
            if b >= a {
                prop_assert_eq!(diff, 0);
            }
        }

        /// Fuzz: Add is the exact u64 sum when it does not overflow.
        #[test]
        fn duration_add_exact(a in any::<u64>(), b in any::<u64>()) {
            prop_assume!(a.checked_add(b).is_some());
            let sum = (Duration::from_micros(a) + Duration::from_micros(b)).as_micros();
            prop_assert_eq!(sum, a + b);
        }

        /// Fuzz: checked_duration_since matches its `self >= earlier` guard exactly —
        /// Some(self-earlier) when ordered, None otherwise, and never underflows.
        #[test]
        fn instant_checked_duration_since(later in any::<u64>(), earlier in any::<u64>()) {
            let li = Instant(later);
            let ei = Instant(earlier);
            match li.checked_duration_since(ei) {
                Some(d) => {
                    prop_assert!(later >= earlier);
                    prop_assert_eq!(d.as_micros(), later - earlier);
                }
                None => prop_assert!(later < earlier),
            }
        }

        /// Fuzz: duration_since / elapsed-style fallback never panics and floors at 0
        /// when earlier > self (uses the same saturating unwrap_or path).
        #[test]
        fn instant_duration_since_floors(a in any::<u64>(), b in any::<u64>()) {
            let ai = Instant(a);
            let bi = Instant(b);
            // duration_since(earlier): self - earlier, or 0 if earlier is later.
            prop_assert_eq!(ai.duration_since(bi).as_micros(), a.saturating_sub(b));
        }

        /// Fuzz: Instant::raw is the identity over the latched 64-bit counter.
        #[test]
        fn instant_raw_identity(v in any::<u64>()) {
            prop_assert_eq!(Instant(v).raw(), v);
        }

        /// Fuzz: kHz→Hz→kHz round-trips exactly (from_khz * 1_000, to_khz / 1_000)
        /// for any kHz value whose Hz product fits in u32.
        #[test]
        fn rate_khz_roundtrip(khz in 0u32..=(u32::MAX / 1_000)) {
            let r = Rate::from_khz(khz);
            prop_assert_eq!(r.to_hz(), khz * 1_000);
            prop_assert_eq!(r.to_khz(), khz);
        }

        /// Fuzz: MHz→Hz round-trips and from_hz is the identity into to_hz.
        #[test]
        fn rate_hz_and_mhz(hz in any::<u32>(), mhz in 0u32..=(u32::MAX / 1_000_000)) {
            prop_assert_eq!(Rate::from_hz(hz).to_hz(), hz);
            prop_assert_eq!(Rate::from_mhz(mhz).to_hz(), mhz * 1_000_000);
            // to_khz is a floor division of the stored Hz.
            prop_assert_eq!(Rate::from_hz(hz).to_khz(), hz / 1_000);
        }
    }
}
