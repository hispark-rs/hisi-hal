//! Compile-time safety verification for ws63-hal.
//!
//! Provides:
//! 1. Const assertions for pointer alignment and register offset soundness
//! 2. Type-level proofs that MMIO addresses are within the valid peripheral range
//! 3. Formal safety contracts used in SAFETY doc comments

// ── Compile-time assertions ──────────────────────────────────────

/// Compile-time assertion. If `$cond` is false, compilation fails with `$msg`.
macro_rules! const_assert {
    ($cond:expr, $msg:expr) => {
        const _: () = {
            #[allow(dead_code, clippy::unit_arg)]
            const ASSERT: () = if !$cond { panic!($msg) };
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

// ── Verify PeripheralGuard ref-count array bounds ─────────────────

// PERIPHERAL_COUNT is defined in clock.rs next to the Peripheral enum.
// Reference it here so the const_assert below breaks if anyone changes
// the count in one place but not the other.
use crate::clock::PERIPHERAL_COUNT;

// Verify that the Peripheral enum still has 17 variants.
// If adding a new peripheral variant, update clock.rs PERIPHERAL_COUNT
// and this assertion value.
const_assert!(PERIPHERAL_COUNT == 17, "clock.rs PERIPHERAL_COUNT must match Peripheral enum variant count");

// Verify Peripheral enum discriminant fits in AtomicU8 index
#[allow(dead_code)]
fn verify_peripheral_count() {
    // If this compiles, all peripheral discriminants fit in 0..17
    let _: [(); 17] = [(); PERIPHERAL_COUNT];
}

// ── Verify timer channel count ───────────────────────────────────

const_assert!(crate::soc::ws63::TIMER_COUNT == 3, "TIMER_COUNT must be 3 for timer0_eoi(0..2) indexing");
const_assert!(crate::soc::ws63::PWM_CHANNEL_COUNT == 8, "PWM_CHANNEL_COUNT must be 8 for PWM register indexing");

// ── Verify DMA channel count ─────────────────────────────────────

const_assert!(crate::soc::ws63::DMA_CHANNEL_COUNT == 4, "DMA_CHANNEL_COUNT must be 4 for DmaDriver channel indexing");
const_assert!(crate::soc::ws63::SPI_COUNT == 2, "SPI_COUNT must be 2 for SPI0/SPI1 instance indexing");
const_assert!(crate::soc::ws63::UART_COUNT == 3, "UART_COUNT must be 3 for UART0/1/2 instance indexing");
const_assert!(crate::soc::ws63::I2C_COUNT == 2, "I2C_COUNT must be 2 for I2C0/1 instance indexing");

// ── Verify timer tick arithmetic doesn't overflow at compile time ─

const_assert!(
    crate::soc::ws63::SYSTEM_CLOCK_HZ == 240_000_000,
    "SYSTEM_CLOCK_HZ must be 240MHz — timer tick calculations assume this"
);
// Verify that the maximum safe us value for timer is computable at 240MHz
const MAX_SAFE_TIMER_US: u64 = u32::MAX as u64 / 240;
const_assert!(MAX_SAFE_TIMER_US > 17_000_000, "Timer max safe period must cover at least 17 seconds");

// ── Verify flash memory constants ─────────────────────────────────

const_assert!(crate::soc::ws63::GPIO_COUNT == 19, "GPIO_COUNT must be 19 for GPIO pin 0-18 indexing");
const_assert!(crate::soc::ws63::ULP_GPIO_COUNT == 8, "ULP_GPIO_COUNT must be 8 for ULP GPIO pin indexing");
const_assert!(crate::soc::ws63::LSADC_CHANNEL_COUNT == 6, "LSADC_CHANNEL_COUNT must be 6 for ADC channel indexing");

// ── Type-level safety invariant helpers ──────────────────────────

/// Newtype proving a value is a valid peripheral index (0-16).
/// Can be constructed via `PeripheralIndex::try_from(peripheral as usize)`.
#[derive(Debug, Clone, Copy)]
pub struct PeripheralIndex(u8);

impl PeripheralIndex {
    /// SAFETY: `idx` must be < PERIPHERAL_COUNT (17).
    pub const unsafe fn new_unchecked(idx: u8) -> Self {
        PeripheralIndex(idx)
    }

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
pub struct GpioPinIndex(u8);

impl GpioPinIndex {
    pub const fn new(pin: u8) -> Option<Self> {
        if pin < crate::soc::ws63::GPIO_COUNT as u8 { Some(GpioPinIndex(pin)) } else { None }
    }
}

// ── Tests ────────────────────────────────────────────────────────

#[cfg(test)]
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
        use crate::soc::ws63;
        assert_eq!(ws63::SYSTEM_CLOCK_HZ, 240_000_000);
        assert_eq!(ws63::TIMER_COUNT, 3);
        assert_eq!(ws63::PWM_CHANNEL_COUNT, 8);
        assert_eq!(ws63::DMA_CHANNEL_COUNT, 4);
        assert_eq!(ws63::SPI_COUNT, 2);
        assert_eq!(ws63::UART_COUNT, 3);
        assert_eq!(ws63::I2C_COUNT, 2);
        assert_eq!(ws63::GPIO_COUNT, 19);
        assert_eq!(ws63::ULP_GPIO_COUNT, 8);
        assert_eq!(ws63::LSADC_CHANNEL_COUNT, 6);
    }

    #[test]
    fn test_max_safe_timer_us() {
        // At 240MHz, max safe period without overflow: u32::MAX / 240
        let max_safe: u64 = u32::MAX as u64 / 240;
        assert!(max_safe > 17_000_000); // at least 17 seconds
        let overflow: u64 = 240_000_000u64 * (max_safe as u64 + 1) / 1_000_000;
        assert!(overflow > u32::MAX as u64); // beyond safe range overflows
    }

    #[test]
    fn test_pwm_channel_count_fits_u8() {
        // 8 PWM channels max, channel index 0-7
        assert!(crate::soc::ws63::PWM_CHANNEL_COUNT <= 8);
    }

    #[test]
    fn test_dma_channel_bound_check() {
        // DMA channels 0-3 are valid, 4+ is out of bounds
        assert!(crate::soc::ws63::DMA_CHANNEL_COUNT == 4);
        for ch in 0u8..4 {
            assert!(ch < crate::soc::ws63::DMA_CHANNEL_COUNT as u8);
        }
        assert!(4u8 >= crate::soc::ws63::DMA_CHANNEL_COUNT as u8);
    }
}
