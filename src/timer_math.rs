//! Shared integer timer conversion; no clock or hardware ownership implied.

/// Convert an integer duration at a supplied clock rate, rejecting 32-bit overflow.
/// The caller chooses the units and still owns zero-duration policy. The two
/// `u32` factors fit in `u64`; integer division deliberately rounds down.
pub(crate) const fn checked_ticks(value: u32, clock_hz: u32, units_per_second: core::num::NonZeroU32) -> Option<u32> {
    let ticks = (value as u64 * clock_hz as u64) / units_per_second.get() as u64;
    if ticks > u32::MAX as u64 { None } else { Some(ticks as u32) }
}

#[cfg(test)]
mod tests {
    use super::*;
    const MICROS: core::num::NonZeroU32 = core::num::NonZeroU32::new(1_000_000).unwrap();
    #[test]
    fn checked_conversion_preserves_zero_rounding_and_overflow_boundaries() {
        assert_eq!(checked_ticks(0, 24_000_000, MICROS), Some(0));
        assert_eq!(checked_ticks(1, 32_000_000, MICROS), Some(32));
        assert_eq!(checked_ticks(1, 32_768, MICROS), Some(0));
        for rate in [24, 32] {
            let bound = u32::MAX / rate;
            assert_eq!(checked_ticks(bound, rate * 1_000_000, MICROS), Some(bound * rate));
            assert_eq!(checked_ticks(bound + 1, rate * 1_000_000, MICROS), None);
        }
    }
}
