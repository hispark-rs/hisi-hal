//! Shared integer timer conversion; no clock or hardware ownership implied.

/// Convert an integer duration at a supplied clock rate, rejecting 32-bit overflow.
/// The caller chooses the units and still owns zero-duration policy. The two
/// `u32` factors fit in `u64`; integer division deliberately rounds down.
pub(crate) const fn checked_ticks(value: u32, clock_hz: u32, units_per_second: core::num::NonZeroU32) -> Option<u32> {
    let ticks = (value as u64 * clock_hz as u64) / units_per_second.get() as u64;
    if ticks > u32::MAX as u64 { None } else { Some(ticks as u32) }
}

/// Round a delay up to full ticks; the product of two `u32`s fits in `u64`.
pub(crate) const fn delay_ticks(value: u32, clock_hz: u32, units_per_second: u32) -> u64 {
    (value as u64 * clock_hz as u64).div_ceil(units_per_second as u64)
}

/// Consume at most one hardware counter period, retaining the remainder.
pub(crate) fn next_delay_chunk(remaining: &mut u64) -> Option<u32> {
    let chunk = (*remaining).min(u64::from(u32::MAX)) as u32;
    *remaining -= u64::from(chunk);
    (chunk != 0).then_some(chunk)
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

    #[test]
    fn delay_rounds_up_and_segments_without_losing_time() {
        for clock in [24_000_000, 32_000_000, 32_768] {
            for unit in [1_000, 1_000_000, 1_000_000_000] {
                for value in [0, 1, 42, 178_957, u32::MAX] {
                    let ticks = delay_ticks(value, clock, unit);
                    let numerator = u64::from(value) * u64::from(clock);
                    assert!(ticks * u64::from(unit) >= numerator);
                    if ticks > 0 {
                        assert!((ticks - 1) * u64::from(unit) < numerator);
                    }
                    let mut remaining = ticks;
                    let mut total = 0u64;
                    while let Some(chunk) = next_delay_chunk(&mut remaining) {
                        total += u64::from(chunk);
                    }
                    assert_eq!(total, ticks);
                    assert_eq!(remaining, 0);
                }
            }
        }
    }
}
