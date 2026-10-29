//! Integer divider selection shared by SPI backends with an even SCK divisor.

/// Smallest legal even divisor that keeps the exact rate at or below `hz`.
/// The caller supplies the qualified input clock; no hardware is configured.
pub(crate) const fn checked_divider(clock: u32, hz: u32) -> Option<u16> {
    if clock == 0 || hz == 0 || hz as u64 * 2 > clock as u64 {
        return None;
    }
    let ceil = (clock as u64).div_ceil(hz as u64);
    let even = (ceil + 1) & !1;
    if even > 65534 { None } else { Some(even as u16) }
}

#[cfg(all(test, not(target_arch = "riscv32")))]
mod tests {
    use super::checked_divider;

    #[test]
    fn divider_never_exceeds_requested_rate_at_boundaries() {
        for clock in [32_000_000u32, 160_000_000] {
            for requested in [489, 2442, 1_000_000, 7_000_000, 30_000_000, clock / 2] {
                if let Some(div) = checked_divider(clock, requested) {
                    assert!(u64::from(clock) <= u64::from(requested) * u64::from(div));
                    assert_eq!(div % 2, 0);
                }
            }
            assert_eq!(checked_divider(clock, 0), None);
            assert_eq!(checked_divider(clock, clock / 2 + 1), None);
        }
        assert_eq!(checked_divider(160_000_000, 30_000_000), Some(6));
        assert_eq!(checked_divider(32_000_000, 7_000_000), Some(6));
    }
}
