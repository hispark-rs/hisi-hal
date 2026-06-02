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
//!   TRNG, Security, DMA, SDMA, SFC, SPI1 — the bits below are placeholders, not
//!   verified against silicon. WiFi/BT entry gates (`CKEN_CTL1` 13 / 8–12 / 29)
//!   are owned by the radio blobs and are not in this enum.
//!
//! The earlier `ClockControl` / `PeripheralGuard` RAII layer was removed: it had
//! zero consumers (the drivers rely on the reset-default clocks) and was dead
//! scaffolding. Re-introduce a clock-gating API alongside a real consumer if one
//! is needed, deriving the gate bits from [`Peripheral::cken_info`].

/// Enumeration of all peripheral clocks.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Peripheral {
    Uart0,
    Uart1,
    Uart2,
    I2c0,
    I2c1,
    Spi0,
    Spi1,
    Pwm,
    Timer,
    Lsadc,
    Tsensor,
    I2s,
    Dma,
    Sdma,
    Sfc,
    Trng,
    SecurityGroup,
}

impl Peripheral {
    /// The CLDO_CRG clock-gate register index (0 = `CKEN_CTL0`, 1 = `CKEN_CTL1`)
    /// and bit position for this peripheral.
    ///
    /// PWM occupies 9 contiguous gates (`CKEN_CTL0` bits 2..=10); this returns
    /// its base bit (2), and a bulk write would be needed to gate all nine.
    /// I2S returns its clk gate (bit 12); it also has a bus gate at bit 11.
    /// See the module docs for which entries are SDK/SVD-confirmed vs placeholders.
    pub fn cken_info(&self) -> (u8, u8) {
        match self {
            // ── SDK/SVD-confirmed gates ──
            Peripheral::Pwm => (0, 2),    // CKEN_CTL0 [10:2], base bit 2 (pwm_porting.c)
            Peripheral::I2s => (0, 12),   // CKEN_CTL0 bit 12 (clk); bit 11 = bus (sio_porting.c)
            Peripheral::Uart0 => (1, 18), // SVD uart_cken[20:18] + clock_init.c
            Peripheral::Uart1 => (1, 19),
            Peripheral::Uart2 => (1, 20),
            Peripheral::Spi0 => (1, 25), // SVD spi_cken[25] + spi_porting.c
            // ── Not individually gated by the SDK (default-on); placeholders, unverified ──
            Peripheral::I2c0 => (0, 18),
            Peripheral::I2c1 => (0, 19),
            Peripheral::Timer => (0, 21),
            Peripheral::Lsadc => (0, 22),
            Peripheral::Tsensor => (0, 23),
            Peripheral::Trng => (0, 25),
            Peripheral::SecurityGroup => (0, 26),
            Peripheral::Dma => (1, 22),
            Peripheral::Sdma => (1, 23),
            Peripheral::Sfc => (1, 24),
            Peripheral::Spi1 => (1, 26),
        }
    }
}

/// Number of [`Peripheral`] enum variants. Update when adding variants.
pub const PERIPHERAL_COUNT: usize = 17;

// ── Tests ──────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::soc::ws63::SYSTEM_CLOCK_HZ;

    #[test]
    fn test_system_clock_240mhz() {
        assert_eq!(SYSTEM_CLOCK_HZ, 240_000_000);
    }

    #[test]
    fn test_peripheral_count() {
        assert_eq!(PERIPHERAL_COUNT, 17);
    }

    #[test]
    fn test_peripheral_cken_info_coverage() {
        let peripherals = [
            Peripheral::Pwm,
            Peripheral::I2c0,
            Peripheral::I2c1,
            Peripheral::Timer,
            Peripheral::Lsadc,
            Peripheral::Tsensor,
            Peripheral::I2s,
            Peripheral::Trng,
            Peripheral::SecurityGroup,
            Peripheral::Uart0,
            Peripheral::Uart1,
            Peripheral::Uart2,
            Peripheral::Dma,
            Peripheral::Sdma,
            Peripheral::Sfc,
            Peripheral::Spi0,
            Peripheral::Spi1,
        ];
        for p in &peripherals {
            let (reg, bit) = p.cken_info();
            assert!(reg <= 1, "Peripheral {:?} has invalid reg={}", p, reg);
            assert!(bit < 32, "Peripheral {:?} has invalid bit={}", p, bit);
        }
    }

    #[test]
    fn test_pwm_cken_info_returns_base_bit() {
        let (reg, bit) = Peripheral::Pwm.cken_info();
        assert_eq!(reg, 0);
        assert_eq!(bit, 2);
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
