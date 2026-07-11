//! Clock definitions for WS63.
//!
//! The WS63 uses a CLDO_CRG (Clock and Reset Generator) for peripheral clock
//! gating. Clocks default to **enabled** out of reset, so the drivers do not
//! gate them; this module keeps the [`Peripheral`] enum and its CKEN bit map
//! ([`Peripheral::cken_info`]) as a peripheral → clock-gate reference, used by
//! `safety.rs`'s drift checks and available to future clock-gating code.
//!
//! CKEN-bit provenance (audited against fbb_ws63 porting + the WS63 SVD):
//! - **SDK/SVD-confirmed**: PWM `CKEN_CTL0` bits [10:2] (base 2; `pwm_porting.c`),
//!   I2S `CKEN_CTL0` bit 11 (bus) + bit 12 (clk) (`sio_porting.c`), UART0/1/2
//!   `CKEN_CTL1` bits 18/19/20 (`clock_init.c` + SVD `uart_cken[20:18]`), SPI
//!   `CKEN_CTL1` bit 25 (`spi_porting.c` + SVD `spi_cken[25]`).
//! - **Not individually gated by the SDK** (rely on the reset-default clock; the
//!   bit is not attested by the SVD or porting code): I2C, Timer, LSADC, Tsensor,
//!   TRNG, Security, DMA, SDMA, SFC, SPI1 — `cken_info` returns `None` for these
//!   rather than fabricating a bit. The Wi-Fi entry gate at `CKEN_CTL1` bit 13
//!   is sequenced by [`crate::rf_power::RfPower`] and is not in this enum.
//!
//! The earlier `ClockControl` / `PeripheralGuard` RAII layer was removed: it had
//! zero consumers (the drivers rely on the reset-default clocks) and was dead
//! scaffolding. Re-introduce a clock-gating API alongside a real consumer if one
//! is needed, deriving the gate bits from [`Peripheral::cken_info`].

/// Enumeration of all peripheral clocks.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Peripheral {
    /// UART0 clock (`CKEN_CTL1` bit 18).
    Uart0,
    /// UART1 clock (`CKEN_CTL1` bit 19).
    Uart1,
    /// UART2 clock (`CKEN_CTL1` bit 20).
    Uart2,
    /// I2C0 clock (not individually gated by the SDK; reset-default on).
    I2c0,
    /// I2C1 clock (not individually gated by the SDK; reset-default on).
    I2c1,
    /// SPI0 clock (`CKEN_CTL1` bit 25).
    Spi0,
    /// SPI1 clock (not individually gated by the SDK; reset-default on).
    Spi1,
    /// PWM clock (`CKEN_CTL0` bits 2..=10; base bit 2).
    Pwm,
    /// Timer clock (not individually gated by the SDK; reset-default on).
    Timer,
    /// LSADC clock (not individually gated by the SDK; reset-default on).
    Lsadc,
    /// Temperature-sensor clock (not individually gated by the SDK; reset-default on).
    Tsensor,
    /// I2S clock (`CKEN_CTL0` bit 12 clk; bit 11 bus).
    I2s,
    /// DMA-controller clock (not individually gated by the SDK; reset-default on).
    Dma,
    /// Secure-DMA clock (not individually gated by the SDK; reset-default on).
    Sdma,
    /// SFC (serial flash controller) clock (not individually gated by the SDK; reset-default on).
    Sfc,
    /// TRNG clock (not individually gated by the SDK; reset-default on).
    Trng,
    /// Security-group clock (not individually gated by the SDK; reset-default on).
    SecurityGroup,
}

impl Peripheral {
    /// The CLDO_CRG clock-gate register index (0 = `CKEN_CTL0`, 1 = `CKEN_CTL1`)
    /// and bit position for this peripheral.
    ///
    /// The CLDO_CRG clock-gate register index (0 = `CKEN_CTL0`, 1 = `CKEN_CTL1`)
    /// and bit position for this peripheral, or `None` if the SDK does **not**
    /// individually gate it (it relies on the reset-default clock; no bit is
    /// attested by the SVD or porting code).
    ///
    /// PWM occupies 9 contiguous gates (`CKEN_CTL0` bits 2..=10); this returns its
    /// base bit (2). I2S returns its clk gate (bit 12); it also has a bus gate at
    /// bit 11. See the module docs for the provenance of each.
    pub fn cken_info(&self) -> Option<(u8, u8)> {
        match self {
            // ── SDK/SVD-confirmed gates ──
            Peripheral::Pwm => Some((0, 2)), // CKEN_CTL0 [10:2], base bit 2 (pwm_porting.c)
            Peripheral::I2s => Some((0, 12)), // CKEN_CTL0 bit 12 (clk); bit 11 = bus (sio_porting.c)
            Peripheral::Uart0 => Some((1, 18)), // SVD uart_cken[20:18] + clock_init.c
            Peripheral::Uart1 => Some((1, 19)),
            Peripheral::Uart2 => Some((1, 20)),
            Peripheral::Spi0 => Some((1, 25)), // SVD spi_cken[25] + spi_porting.c
            // ── Not individually gated by the SDK (default-on) — no authoritative bit ──
            Peripheral::I2c0
            | Peripheral::I2c1
            | Peripheral::Timer
            | Peripheral::Lsadc
            | Peripheral::Tsensor
            | Peripheral::Trng
            | Peripheral::SecurityGroup
            | Peripheral::Dma
            | Peripheral::Sdma
            | Peripheral::Sfc
            | Peripheral::Spi1 => None,
        }
    }
}

/// Number of [`Peripheral`] enum variants. Update when adding variants.
pub const PERIPHERAL_COUNT: usize = 17;

// ── Tests ──────────────────────────────────────────────────────

#[cfg(all(test, not(target_arch = "riscv32")))]
mod tests {
    use super::*;
    use crate::soc::chip::SYSTEM_CLOCK_HZ;

    #[test]
    fn test_system_clock_240mhz() {
        assert_eq!(SYSTEM_CLOCK_HZ, 240_000_000);
    }

    #[test]
    fn test_peripheral_count() {
        assert_eq!(PERIPHERAL_COUNT, 17);
    }

    #[test]
    fn test_peripheral_cken_info_bounds_and_gating() {
        // SDK/SVD-confirmed gates return Some with in-range (reg, bit).
        let gated = [
            Peripheral::Pwm,
            Peripheral::I2s,
            Peripheral::Uart0,
            Peripheral::Uart1,
            Peripheral::Uart2,
            Peripheral::Spi0,
        ];
        for p in &gated {
            let (reg, bit) = p.cken_info().unwrap_or_else(|| panic!("{:?} should be gated", p));
            assert!(reg <= 1, "Peripheral {:?} has invalid reg={}", p, reg);
            assert!(bit < 32, "Peripheral {:?} has invalid bit={}", p, bit);
        }
        // Peripherals the SDK does not individually gate return None (not a fake bit).
        let ungated = [
            Peripheral::I2c0,
            Peripheral::I2c1,
            Peripheral::Timer,
            Peripheral::Lsadc,
            Peripheral::Tsensor,
            Peripheral::Trng,
            Peripheral::SecurityGroup,
            Peripheral::Dma,
            Peripheral::Sdma,
            Peripheral::Sfc,
            Peripheral::Spi1,
        ];
        for p in &ungated {
            assert_eq!(p.cken_info(), None, "Peripheral {:?} should not be gated", p);
        }
    }

    #[test]
    fn test_pwm_cken_info_returns_base_bit() {
        assert_eq!(Peripheral::Pwm.cken_info(), Some((0, 2)));
    }

    #[test]
    fn test_i2s_cken_info_is_clk_gate() {
        // I2S clk gate is CKEN_CTL0 bit 12 (was wrongly bit 24 before the SDK audit).
        assert_eq!(Peripheral::I2s.cken_info(), Some((0, 12)));
    }

    #[test]
    fn test_peripheral_variants_are_unique() {
        let variants: [Peripheral; 17] = [
            Peripheral::Uart0,
            Peripheral::Uart1,
            Peripheral::Uart2,
            Peripheral::I2c0,
            Peripheral::I2c1,
            Peripheral::Spi0,
            Peripheral::Spi1,
            Peripheral::Pwm,
            Peripheral::Timer,
            Peripheral::Lsadc,
            Peripheral::Tsensor,
            Peripheral::I2s,
            Peripheral::Dma,
            Peripheral::Sdma,
            Peripheral::Sfc,
            Peripheral::Trng,
            Peripheral::SecurityGroup,
        ];
        for i in 0..variants.len() {
            for j in (i + 1)..variants.len() {
                assert_ne!(variants[i], variants[j]);
            }
        }
    }
}

// ── Property-based fuzz tests ──────────────────────────────────

#[cfg(all(test, not(target_arch = "riscv32")))]
mod proptests {
    use super::{PERIPHERAL_COUNT, Peripheral};
    use proptest::prelude::*;

    /// All [`Peripheral`] variants, in the canonical order. Kept in sync with the
    /// enum / [`PERIPHERAL_COUNT`]; a fuzz index selects one without touching MMIO.
    const ALL: [Peripheral; PERIPHERAL_COUNT] = [
        Peripheral::Uart0,
        Peripheral::Uart1,
        Peripheral::Uart2,
        Peripheral::I2c0,
        Peripheral::I2c1,
        Peripheral::Spi0,
        Peripheral::Spi1,
        Peripheral::Pwm,
        Peripheral::Timer,
        Peripheral::Lsadc,
        Peripheral::Tsensor,
        Peripheral::I2s,
        Peripheral::Dma,
        Peripheral::Sdma,
        Peripheral::Sfc,
        Peripheral::Trng,
        Peripheral::SecurityGroup,
    ];

    proptest! {
        /// Fuzz: `cken_info()` never panics for any variant and, when it returns a
        /// gate, the (reg, bit) addresses a real CLDO_CRG clock-gate location —
        /// reg ∈ {0,1} (CKEN_CTL0/CTL1) and bit < 32 (fits the 32-bit register
        /// word). A bit >= 32 would make the `1 << bit` gate mask below overflow.
        #[test]
        fn cken_info_gate_is_always_in_a_register_word(idx in 0usize..PERIPHERAL_COUNT) {
            let p = ALL[idx];
            if let Some((reg, bit)) = p.cken_info() {
                prop_assert!(reg <= 1, "{:?}: reg={} out of CKEN_CTL0/CTL1 range", p, reg);
                prop_assert!(bit < 32, "{:?}: bit={} outside 32-bit register word", p, bit);
                // The gate mask is `1 << bit`: re-derive it the way a gating write
                // would, asserting it lands on exactly one in-word bit (no overflow,
                // no stray bits). `checked_shl` catches an out-of-range shift without
                // panicking the fuzz runner.
                let mask = 1u32.checked_shl(bit as u32);
                prop_assert!(mask.is_some(), "{:?}: 1 << {} overflows the word", p, bit);
                prop_assert_eq!(mask.unwrap().count_ones(), 1, "{:?}: gate mask must set one bit", p);
            }
        }

        /// Fuzz: the gated/ungated partition is total and stable — every variant is
        /// classified exactly once (`Some` xor `None`), and the result is pure
        /// (idempotent across repeated calls), so no index maps into a gap.
        #[test]
        fn cken_info_is_pure_and_total(idx in 0usize..PERIPHERAL_COUNT) {
            let p = ALL[idx];
            let first = p.cken_info();
            prop_assert_eq!(first, p.cken_info(), "{:?}: cken_info must be deterministic", p);
        }

        /// Fuzz: distinct *gated* peripherals on the SAME register never collide on
        /// the same bit — each gate owns a unique (reg, bit), so OR-ing one gate's
        /// mask can never disturb another's. Quantified over all index pairs.
        #[test]
        fn distinct_gates_do_not_alias(i in 0usize..PERIPHERAL_COUNT, j in 0usize..PERIPHERAL_COUNT) {
            prop_assume!(i != j);
            if let (Some(a), Some(b)) = (ALL[i].cken_info(), ALL[j].cken_info()) {
                prop_assert_ne!(a, b, "{:?} and {:?} share gate {:?}", ALL[i], ALL[j], a);
            }
        }
    }
}
