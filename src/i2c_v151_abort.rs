//! Common v151 `TX_ABRT_SLV_INTX` interpretation, independent of MMIO.
//! The separate WS53 `TX_ABRT_SOURCE` still requires its own validation.

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum Cause {
    AddressNack,
    DataNack,
    MixedNack,
    ArbitrationLost,
    Other,
}

pub(crate) const fn classify(detail: u32) -> Cause {
    // Slave arbitration loss (bit14) is documented to assert bit12 as well.
    if detail == 0x1000 || detail == 0x5000 {
        return Cause::ArbitrationLost;
    }
    // Bit4 is general-call address NACK; bit3 is data NACK.
    if detail != 0 && detail & !0x1f == 0 {
        return match (detail & 0x17 != 0, detail & 0x08 != 0) {
            (true, false) => Cause::AddressNack,
            (false, true) => Cause::DataNack,
            (true, true) => Cause::MixedNack,
            _ => Cause::Other,
        };
    }
    Cause::Other
}

#[cfg(test)]
mod tests {
    use super::{Cause, classify};

    #[test]
    fn all_nack_combinations_and_unrelated_flags_are_distinguished() {
        for value in 1..=0x1f {
            let expected = match (value & 0x17 != 0, value & 8 != 0) {
                (true, false) => Cause::AddressNack,
                (false, true) => Cause::DataNack,
                (true, true) => Cause::MixedNack,
                _ => unreachable!(),
            };
            assert_eq!(classify(value), expected);
            for bit in 5..32 {
                assert_eq!(classify(value | (1u32 << bit)), Cause::Other);
            }
        }
        assert_eq!(classify(0), Cause::Other);
        assert_eq!(classify(0x1000), Cause::ArbitrationLost);
        assert_eq!(classify(0x5000), Cause::ArbitrationLost);
        assert_eq!(classify(0x4000), Cause::Other);
        assert_eq!(classify(0x9000), Cause::Other);
    }
}
