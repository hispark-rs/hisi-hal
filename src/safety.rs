//! Compile-time safety checks for hisi-riscv-hal.
//!
//! Provides:
//! 1. Const assertions that key peripheral MMIO addresses are within range and
//!    that timer-tick arithmetic cannot overflow at the timer clock (24 MHz TCXO)
//! 2. Newtype helpers (`PeripheralIndex`, `GpioPinIndex`) for bounds-checked indices
//!
//! Tautological `const X == <literal>` count assertions were removed: pinning a
//! `soc` constant to a duplicated magic number verifies nothing about the code
//! that indexes with it.

// ── Compile-time assertions ──────────────────────────────────────

/// Compile-time assertion. If `$cond` is false, compilation fails with `$msg`.
macro_rules! const_assert {
    ($cond:expr, $msg:expr) => {
        const _: () = {
            #[allow(dead_code, clippy::unit_arg)]
            const ASSERT: () = if !$cond {
                panic!($msg)
            };
        };
    };
}

/// Assert that a pointer is 4-byte aligned (RISC-V MMIO requirement).
#[allow(unused_macros)]
macro_rules! ptr_aligned {
    ($ptr:expr) => {
        const_assert!($ptr as usize % 4 == 0, "MMIO pointer must be 4-byte aligned");
    };
}

// ── Verify PAC peripheral addresses ──────────────────────────────

/// All PAC peripherals must be within the WS63 MMIO address range.
const MMIO_LOW: usize = 0x4000_0000;
const MMIO_HIGH: usize = 0x5704_0000; // ULP_GPIO at 0x5703_0000

// Verify key peripheral addresses are within range
const_assert!(
    0x4401_0000 >= MMIO_LOW && 0x4401_2000 <= MMIO_HIGH,
    "UART region (UART0@0x4401_0000..UART2@0x4401_2000) out of MMIO range"
);
const_assert!(0x4000_6000 >= MMIO_LOW && 0x4000_6100 <= MMIO_HIGH, "WDT region (0x4000_6000) out of MMIO range");
const_assert!(0x4800_0000 >= MMIO_LOW && 0x4800_0100 <= MMIO_HIGH, "SFC base out of MMIO range");
const_assert!(0x4A00_0000 >= MMIO_LOW && 0x4A00_0000 <= MMIO_HIGH, "DMA base out of MMIO range");
const_assert!(0x4410_0000 >= MMIO_LOW && 0x4411_4000 <= MMIO_HIGH, "Crypto base out of MMIO range");

// PERIPHERAL_COUNT (clock.rs) bounds the PeripheralIndex newtype below.
use crate::clock::PERIPHERAL_COUNT;

// ── Verify timer tick arithmetic doesn't overflow at compile time ─

const_assert!(crate::soc::chip::SYSTEM_CLOCK_HZ == 240_000_000, "SYSTEM_CLOCK_HZ must be 240MHz (CPU/PLL clock)");
// The Timer/WDT count at the TCXO crystal (TIMER_CLOCK_HZ), not the CPU clock.
const_assert!(
    crate::soc::chip::TIMER_CLOCK_HZ.is_multiple_of(1_000_000) && crate::soc::chip::TIMER_CLOCK_HZ >= 1_000_000,
    "TIMER_CLOCK_HZ must be a whole number of MHz so us->ticks is exact"
);
// Verify the maximum safe us value for the timer is computable at the timer clock.
const MAX_SAFE_TIMER_US: u64 = u32::MAX as u64 / (crate::soc::chip::TIMER_CLOCK_HZ as u64 / 1_000_000);
const_assert!(MAX_SAFE_TIMER_US > 17_000_000, "Timer max safe period must cover at least 17 seconds");

// ── Type-level safety invariant helpers ──────────────────────────

/// Newtype proving a value is a valid peripheral index (0-16).
/// Can be constructed via `PeripheralIndex::try_from(peripheral as usize)`.
#[derive(Debug, Clone, Copy)]
pub struct PeripheralIndex(u8);

impl PeripheralIndex {
    /// SAFETY: `idx` must be < PERIPHERAL_COUNT (17).
    #[allow(clippy::missing_safety_doc)]
    pub const unsafe fn new_unchecked(idx: u8) -> Self {
        PeripheralIndex(idx)
    }

    /// Returns the wrapped peripheral index as a `usize`.
    pub const fn get(&self) -> usize {
        self.0 as usize
    }
}

impl TryFrom<usize> for PeripheralIndex {
    type Error = ();
    fn try_from(idx: usize) -> Result<Self, ()> {
        if idx < PERIPHERAL_COUNT { Ok(PeripheralIndex(idx as u8)) } else { Err(()) }
    }
}

/// Newtype proving a value is a valid GPIO pin number (0-18).
#[derive(Debug, Clone, Copy)]
pub struct GpioPinIndex(#[allow(dead_code)] u8);

impl GpioPinIndex {
    /// Constructs a valid pin index, returning `None` if `pin` >= `GPIO_COUNT`.
    pub const fn new(pin: u8) -> Option<Self> {
        if pin < crate::soc::chip::GPIO_COUNT as u8 { Some(GpioPinIndex(pin)) } else { None }
    }
}

// ── Tests ────────────────────────────────────────────────────────

#[cfg(all(test, not(target_arch = "riscv32")))]
mod tests {
    use super::*;

    #[test]
    fn test_peripheral_index_bounds() {
        assert!(PeripheralIndex::try_from(0).is_ok());
        assert!(PeripheralIndex::try_from(16).is_ok());
        assert!(PeripheralIndex::try_from(17).is_err());
    }

    #[test]
    fn test_gpio_pin_bounds() {
        assert!(GpioPinIndex::new(0).is_some());
        assert!(GpioPinIndex::new(18).is_some());
        assert!(GpioPinIndex::new(19).is_none());
    }

    #[test]
    fn test_soc_constants_consistency() {
        use crate::soc::chip;
        assert_eq!(chip::SYSTEM_CLOCK_HZ, 240_000_000);
        assert_eq!(chip::TIMER_COUNT, 3);
        assert_eq!(chip::PWM_CHANNEL_COUNT, 8);
        assert_eq!(chip::DMA_CHANNEL_COUNT, 4);
        assert_eq!(chip::SPI_COUNT, 2);
        assert_eq!(chip::UART_COUNT, 3);
        assert_eq!(chip::I2C_COUNT, 2);
        assert_eq!(chip::GPIO_COUNT, 19);
        assert_eq!(chip::ULP_GPIO_COUNT, 8);
        assert_eq!(chip::LSADC_CHANNEL_COUNT, 6);
    }

    #[test]
    fn test_max_safe_timer_us() {
        // The timer counts at the TCXO crystal (TIMER_CLOCK_HZ, 24 MHz), so the
        // max safe one-shot period without u32 overflow is u32::MAX / ticks_per_us.
        let ticks_per_us = crate::soc::chip::TIMER_CLOCK_HZ as u64 / 1_000_000;
        let max_safe: u64 = u32::MAX as u64 / ticks_per_us;
        assert!(max_safe > 17_000_000); // at least 17 seconds
        let overflow: u64 = crate::soc::chip::TIMER_CLOCK_HZ as u64 * (max_safe + 1) / 1_000_000;
        assert!(overflow > u32::MAX as u64); // beyond safe range overflows
    }

    #[test]
    fn test_pwm_channel_count_fits_u8() {
        // 8 PWM channels max, channel index 0-7
        assert!(crate::soc::chip::PWM_CHANNEL_COUNT <= 8);
    }

    #[test]
    fn test_dma_channel_bound_check() {
        // DMA channels 0-3 are valid, 4+ is out of bounds
        assert!(crate::soc::chip::DMA_CHANNEL_COUNT == 4);
        for ch in 0u8..4 {
            assert!(ch < crate::soc::chip::DMA_CHANNEL_COUNT as u8);
        }
        assert!(4u8 >= crate::soc::chip::DMA_CHANNEL_COUNT as u8);
    }
}

// ── Property-based fuzz tests ──────────────────────────────────

#[cfg(all(test, not(target_arch = "riscv32")))]
mod proptests {
    use super::{GpioPinIndex, PeripheralIndex};
    use crate::clock::PERIPHERAL_COUNT;
    use proptest::prelude::*;

    // Mirror the driver's max-safe-µs derivation exactly (TIMER_CLOCK_HZ is a
    // whole number of MHz, so ticks_per_us divides evenly — see const_assert!).
    const TICKS_PER_US: u64 = crate::soc::chip::TIMER_CLOCK_HZ as u64 / 1_000_000;
    const MAX_SAFE_TIMER_US: u64 = u32::MAX as u64 / TICKS_PER_US;

    proptest! {
        /// Fuzz: `PeripheralIndex::try_from` never panics for any usize input.
        #[test]
        fn periph_index_never_panics(idx in any::<usize>()) {
            let _ = PeripheralIndex::try_from(idx);
        }

        /// Fuzz: try_from accepts EXACTLY the in-range indices (0..PERIPHERAL_COUNT)
        /// and rejects everything else — the bound is the only gate.
        #[test]
        fn periph_index_bound_is_exact(idx in any::<usize>()) {
            prop_assert_eq!(PeripheralIndex::try_from(idx).is_ok(), idx < PERIPHERAL_COUNT);
        }

        /// Fuzz: when accepted, `get()` round-trips the index losslessly. The
        /// constructor stores `idx as u8`; since idx < 17 the narrowing cast can
        /// never truncate/wrap (the cast-before-clamp bug class). If the bound
        /// ever exceeded 256, this would catch the lost high bits.
        #[test]
        fn periph_index_get_roundtrips(idx in 0usize..PERIPHERAL_COUNT) {
            let pi = PeripheralIndex::try_from(idx).unwrap();
            prop_assert_eq!(pi.get(), idx);
            // And the stored value always fits the documented 0..17 range.
            prop_assert!(pi.get() < PERIPHERAL_COUNT);
        }

        /// Fuzz: out-of-range indices are always rejected (never silently
        /// truncated into the valid window by the u8 cast).
        #[test]
        fn periph_index_rejects_out_of_range(idx in PERIPHERAL_COUNT..=usize::MAX) {
            prop_assert!(PeripheralIndex::try_from(idx).is_err());
        }

        /// Fuzz: `GpioPinIndex::new` accepts EXACTLY pins < GPIO_COUNT and never
        /// panics, for any u8 (covers the full 0..=255 range incl. the boundary).
        #[test]
        fn gpio_pin_bound_is_exact(pin in any::<u8>()) {
            let valid = (pin as usize) < crate::soc::chip::GPIO_COUNT;
            prop_assert_eq!(GpioPinIndex::new(pin).is_some(), valid);
        }

        /// Fuzz: the timer max-safe-µs window is self-consistent — every µs at or
        /// below MAX_SAFE_TIMER_US converts to ticks without exceeding u32::MAX
        /// (no overflow inside the documented safe range).
        #[test]
        fn timer_safe_us_never_overflows(us in 0u64..=MAX_SAFE_TIMER_US) {
            let ticks = crate::soc::chip::TIMER_CLOCK_HZ as u64 * us / 1_000_000;
            prop_assert!(ticks <= u32::MAX as u64, "safe us={} -> ticks={}", us, ticks);
        }

        /// Fuzz: just past the safe window, the same conversion always overflows
        /// u32 — proving MAX_SAFE_TIMER_US is the true (not a conservative) bound.
        #[test]
        fn timer_just_over_safe_us_overflows(us in (MAX_SAFE_TIMER_US + 1)..=(MAX_SAFE_TIMER_US * 2 + 1)) {
            let ticks = crate::soc::chip::TIMER_CLOCK_HZ as u64 * us / 1_000_000;
            prop_assert!(ticks > u32::MAX as u64);
        }
    }
}
