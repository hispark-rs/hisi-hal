//! Assembly of already-sampled TCXO counter words, independent of MMIO.

/// Join words in high-to-low order. Inputs are not masked: BS21 historically
/// combines raw register values, while WS53 supplies masked 16-bit fields.
#[cfg(any(feature = "chip-ws53", feature = "chip-bs21", test))]
pub(crate) const fn join([high, mid_high, mid_low, low]: [u64; 4]) -> u64 {
    (high << 48) | (mid_high << 32) | (mid_low << 16) | low
}

/// Read the selected chip's latched representation, retaining its register
/// access width and order. WS63 SDK enables CONFIG_TCXO_WITH_TWO_DATA_REGS;
/// BS21 retains its previous four-register read behavior.
#[cfg(not(feature = "chip-ws53"))]
pub(crate) fn read_counter(regs: &crate::soc::pac::tcxo::RegisterBlock) -> u64 {
    #[cfg(feature = "chip-ws63")]
    {
        let high = u64::from(regs.tcxo_count1().read().bits());
        let low = u64::from(regs.tcxo_count0().read().bits());
        (high << 32) | low
    }
    #[cfg(feature = "chip-bs21")]
    {
        let c0 = u64::from(regs.tcxo_count0().read().bits());
        let c1 = u64::from(regs.tcxo_count1().read().bits());
        let c2 = u64::from(regs.tcxo_count2().read().bits());
        let c3 = u64::from(regs.tcxo_count3().read().bits());
        join([c3, c2, c1, c0])
    }
}

#[cfg(not(feature = "chip-ws53"))]
pub(crate) fn read_counter32(regs: &crate::soc::pac::tcxo::RegisterBlock) -> u32 {
    let c0 = regs.tcxo_count0().read().bits();
    #[cfg(feature = "chip-ws63")]
    {
        c0
    }
    #[cfg(feature = "chip-bs21")]
    {
        (regs.tcxo_count1().read().bits() << 16) | c0
    }
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
        // Raw BS21 register values are intentionally not truncated here.
        assert_eq!(join([0, 0, 0, 0x1_0000]), 0x1_0000);
    }

    #[test]
    #[cfg(not(feature = "chip-ws53"))]
    fn register_layout_matches_selected_chip() {
        // Private host memory is used only as a PAC register-view fixture.
        let mut words = [0u32, 0x89ab_cdef, 0x1234_5678, 0xaaaa, 0xbbbb];
        let regs = unsafe { &*words.as_mut_ptr().cast::<crate::soc::pac::tcxo::RegisterBlock>() };
        #[cfg(feature = "chip-ws63")]
        {
            assert_eq!(super::read_counter(regs), 0x1234_5678_89ab_cdef);
            assert_eq!(super::read_counter32(regs), 0x89ab_cdef);
        }
        #[cfg(feature = "chip-bs21")]
        {
            assert_eq!(super::read_counter(regs), join([0xbbbb, 0xaaaa, 0x1234_5678, 0x89ab_cdef]));
            assert_eq!(super::read_counter32(regs), (0x1234_5678u32 << 16) | 0x89ab_cdef);
        }
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
