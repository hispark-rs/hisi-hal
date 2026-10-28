//! Assembly of already-sampled TCXO counter words, independent of MMIO.

/// Join words in high-to-low order. Inputs are not masked: WS63 historically
/// combines raw register values, while WS53 supplies masked 16-bit fields.
pub(crate) const fn join([high, mid_high, mid_low, low]: [u64; 4]) -> u64 {
    (high << 48) | (mid_high << 32) | (mid_low << 16) | low
}

#[cfg(all(test, not(target_arch = "riscv32")))]
mod tests {
    use super::join;
    use proptest::prelude::*;

    #[test]
    fn joins_sampled_words_and_preserves_raw_overlap() {
        assert_eq!(join([0x1234, 0x5678, 0x9abc, 0xdef0]), 0x1234_5678_9abc_def0);
        assert_eq!(join([0, 0, 0, 0]), 0);
        assert_eq!(join([0xffff; 4]), u64::MAX);
        // Raw WS63 register values are intentionally not truncated here.
        assert_eq!(join([0, 0, 0, 0x1_0000]), 0x1_0000);
    }

    proptest! {
        #[test]
        fn agrees_with_legacy_raw_register_assembly(words in any::<[u32; 4]>()) {
            let [a, b, c, d] = words.map(u64::from);
            let original = (a << 48) | (b << 32) | (c << 16) | d;
            prop_assert_eq!(join([a, b, c, d]), original);
        }
    }
}
