//! Clock configuration for WS63.
//!
//! The WS63 uses a CLDO_CRG (Clock and Reset Generator) for peripheral clock
//! enables, dividers, and clock source selection. The system boots at 240MHz
//! and this module provides helpers for enabling/disabling peripheral clocks.
//!
//! # Peripheral clock guards
//!
//! The [`PeripheralGuard`] type provides RAII-based clock management:
//! the clock is enabled when the guard is created and disabled on drop,
//! with reference counting to handle multiple users of the same peripheral.

use crate::peripherals::CldoCrg;
use crate::system::{Clocks, System};
use core::marker::PhantomData;
use core::sync::atomic::Ordering;
use portable_atomic::AtomicU8;

// ── Peripheral enum + RAII guards ──────────────────────────────────

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
    /// Get the clock register and bit position for this peripheral.
    pub fn cken_info(&self) -> (u8, u8) {
        match self {
            // PWM has 9 clock gates (bits 2:10); cken_info returns base bit.
            // peripheral_guard handles the 9-bit bulk write specially.
            Peripheral::Pwm => (0, 2),
            Peripheral::I2c0 => (0, 18),
            Peripheral::I2c1 => (0, 19),
            Peripheral::Timer => (0, 21),
            Peripheral::Lsadc => (0, 22),
            Peripheral::Tsensor => (0, 23),
            Peripheral::I2s => (0, 24),
            Peripheral::Trng => (0, 25),
            Peripheral::SecurityGroup => (0, 26),
            Peripheral::Uart0 => (1, 18),
            Peripheral::Uart1 => (1, 19),
            Peripheral::Uart2 => (1, 20),
            Peripheral::Dma => (1, 22),
            Peripheral::Sdma => (1, 23),
            Peripheral::Sfc => (1, 24),
            Peripheral::Spi0 => (1, 25),
            Peripheral::Spi1 => (1, 26),
        }
    }
}

/// Number of Peripheral enum variants (= REF_COUNTS array size).
/// Must be updated when adding new Peripheral variants.
pub const PERIPHERAL_COUNT: usize = 17;

static REF_COUNTS: [AtomicU8; PERIPHERAL_COUNT] = {
    const ZERO: AtomicU8 = AtomicU8::new(0);
    [ZERO; PERIPHERAL_COUNT]
};

/// RAII guard that enables a peripheral clock on creation.
///
/// Uses reference counting — the clock is only disabled when
/// all guards for that peripheral are dropped. Stores a raw
/// pointer to the CLDO_CRG register block so Drop can write
/// the disable bit.
pub struct PeripheralGuard<'d> {
    peripheral: Peripheral,
    // raw pointer to CLDO_CRG register block (static MMIO @ 0x4400_1100)
    // Provenance: captured from &CldoCrg via register_block() in peripheral_guard().
    // Verified: 0x4400_1100 ∈ [MMIO_LOW=0x4000_0000, MMIO_HIGH=0x5704_0000] (see safety.rs).
    cldo_crg: *const (),
    _marker: PhantomData<&'d ()>,
}

// SAFETY: MMIO register blocks are Sync on single-core RISC-V
unsafe impl Send for PeripheralGuard<'_> {}
unsafe impl Sync for PeripheralGuard<'_> {}

impl Drop for PeripheralGuard<'_> {
    fn drop(&mut self) {
        let idx = self.peripheral as usize;
        // fetch_sub returns the OLD value. If old == 1, we're the last guard.
        // Underflow (prev==0) means double-drop or bug — caught by debug_assert.
        let prev = REF_COUNTS[idx].fetch_sub(1, Ordering::Relaxed);
        debug_assert!(prev > 0, "PeripheralGuard double-drop detected");
        if prev == 1 {
            // Actually disable the clock in hardware (last guard dropped)
            // SAFETY: cldo_crg is a raw pointer to the CLDO_CRG MMIO register block
            // (0x4400_1100). The pointer was captured during PeripheralGuard construction
            // from a valid &CldoCrg reference. MMIO addresses are static and always valid.
            let cken = unsafe { &*(self.cldo_crg as *const ws63_pac::cldo_crg::RegisterBlock) };
            let (reg, bit) = self.peripheral.cken_info();
            if matches!(self.peripheral, Peripheral::Pwm) {
                let bits = cken.cken_ctl0().read();
                cken.cken_ctl0().write(|w| unsafe { w.bits(bits.bits() & !(0x1FF << 2)) });
            } else if reg == 0 {
                let bits = cken.cken_ctl0().read();
                cken.cken_ctl0().write(|w| unsafe { w.bits(bits.bits() & !(1 << bit)) });
            } else {
                let bits = cken.cken_ctl1().read();
                cken.cken_ctl1().write(|w| unsafe { w.bits(bits.bits() & !(1 << bit)) });
            }
        }
    }
}

// ── ClockControl ──────────────────────────────────────────────────

/// Clock control peripheral wrapper.
pub struct ClockControl<'d> {
    cldo_crg: CldoCrg<'d>,
}

impl<'d> ClockControl<'d> {
    /// Configure the system clocks with defaults (240MHz system, 240MHz peripheral bus).
    pub fn configure_system(system: System<'d>) -> Self {
        Self { cldo_crg: system.cldo_crg }
    }

    /// Freeze the clock configuration, returning the resolved [`Clocks`].
    pub fn freeze(self) -> Clocks {
        Clocks::default()
    }

    /// Get a reference to the CLDO_CRG.
    pub fn cldo_crg(&self) -> &CldoCrg<'d> {
        &self.cldo_crg
    }

    /// Create a RAII peripheral clock guard.
    ///
    /// Uses atomic fetch_add to avoid TOCTOU race with interrupt handlers.
    pub fn peripheral_guard(&self, peripheral: Peripheral) -> PeripheralGuard<'_> {
        let idx = peripheral as usize;
        let old = REF_COUNTS[idx].fetch_add(1, Ordering::Relaxed);
        if old == 0 {
            // PWM has 9 contiguous bits (2:10) — bulk-enable all of them
            match peripheral {
                Peripheral::Pwm => self.enable_pwm(),
                _ => {
                    let (reg, bit) = peripheral.cken_info();
                    self.write_cken_bit(reg, bit, true);
                }
            }
        }
        let cldo_ptr = self.cldo_crg.register_block() as *const ws63_pac::cldo_crg::RegisterBlock as *const ();
        PeripheralGuard { peripheral, cldo_crg: cldo_ptr, _marker: PhantomData }
    }

    // ── Private: consolidated clock register access ────────────────

    /// Write a clock enable bit to cken_ctl0 or cken_ctl1.
    fn write_cken_bit(&self, reg: u8, bit: u8, set: bool) {
        let cken = self.cldo_crg.register_block();
        let bits = if reg == 0 { cken.cken_ctl0().read().bits() } else { cken.cken_ctl1().read().bits() };
        let val = if set { bits | (1 << bit) } else { bits & !(1 << bit) };
        if reg == 0 {
            unsafe { cken.cken_ctl0().write(|w| w.bits(val)) };
        } else {
            unsafe { cken.cken_ctl1().write(|w| w.bits(val)) };
        }
    }

    fn read_cken_bit(&self, reg: u8, bit: u8) -> bool {
        let cken = self.cldo_crg.register_block();
        let bits = if reg == 0 { cken.cken_ctl0().read().bits() } else { cken.cken_ctl1().read().bits() };
        (bits & (1 << bit)) != 0
    }

    // ── Individual clock enable methods ────────────────────────────

    pub fn enable_uart(&self, uart_idx: usize) {
        let bit = match uart_idx {
            0 => 18,
            1 => 19,
            2 => 20,
            _ => unreachable!(),
        };
        self.write_cken_bit(1, bit, true);
    }

    pub fn enable_i2c(&self, i2c_idx: usize) {
        let bit = match i2c_idx {
            0 => 18,
            1 => 19,
            _ => unreachable!(),
        };
        self.write_cken_bit(0, bit, true);
    }

    /// Enable the clock gate for SPI0.
    pub fn enable_spi0(&self) { self.write_cken_bit(1, 25, true); }

    /// Enable the clock gate for SPI1.
    pub fn enable_spi1(&self) { self.write_cken_bit(1, 26, true); }

    /// Enable the clock gate for both SPI0 and SPI1.
    pub fn enable_spi(&self) { self.enable_spi0(); self.enable_spi1(); }

    pub fn enable_pwm(&self) {
        // PWM has 9 contiguous bits (2:10) — needs bulk write
        let cken = self.cldo_crg.register_block();
        let bits = cken.cken_ctl0().read();
        cken.cken_ctl0().write(|w| unsafe { w.bits(bits.bits() | (0x1FF << 2)) });
    }

    pub fn enable_timer(&self) {
        self.write_cken_bit(0, 21, true);
    }
    pub fn enable_lsadc(&self) {
        self.write_cken_bit(0, 22, true);
    }
    pub fn enable_tsensor(&self) {
        self.write_cken_bit(0, 23, true);
    }
    pub fn enable_i2s(&self) {
        self.write_cken_bit(0, 24, true);
    }
    pub fn enable_dma(&self) {
        self.write_cken_bit(1, 22, true);
    }
    pub fn enable_sdma(&self) {
        self.write_cken_bit(1, 23, true);
    }
    pub fn enable_sfc(&self) {
        self.write_cken_bit(1, 24, true);
    }
    pub fn enable_trng(&self) {
        self.write_cken_bit(0, 25, true);
    }
    pub fn enable_security(&self) {
        self.write_cken_bit(0, 26, true);
    }

    /// Disable the clock gate for a specific peripheral.
    ///
    /// Only disables the clock if no PeripheralGuard references are active.
    /// If guards exist, this is a no-op to avoid corrupting the RAII system.
    pub fn disable_peripheral(&self, peripheral: Peripheral) {
        let idx = peripheral as usize;
        // Use load-acquire to ensure all prior guard operations are visible.
        // Interrupt-racing guard creation between load and disable is a known
        // TOCTOU — callers must ensure no concurrent guard creation.
        let count = REF_COUNTS[idx].load(Ordering::Acquire);
        if count > 0 {
            return; // Guards are active, do not force-disable
        }
        if matches!(peripheral, Peripheral::Pwm) {
            // Disable all 9 PWM clock bits
            let cken = self.cldo_crg.register_block();
            let bits = cken.cken_ctl0().read();
            cken.cken_ctl0().write(|w| unsafe { w.bits(bits.bits() & !(0x1FF << 2)) });
        } else {
            let (reg, bit) = peripheral.cken_info();
            self.write_cken_bit(reg, bit, false);
        }
    }

    /// Trigger a soft reset for a specific peripheral via CLDO_CRG.
    ///
    /// Power-cycles the peripheral clock (disable → delay → enable).
    /// Does NOT check ref-count — use with caution.
    pub fn reset_peripheral(&self, peripheral: Peripheral) {
        // PWM has 9 contiguous bits (2:10) — bulk-toggle all of them
        if matches!(peripheral, Peripheral::Pwm) {
            let cken = self.cldo_crg.register_block();
            let bits = cken.cken_ctl0().read();
            let pwm_orig = bits.bits() & (0x1FF << 2); // save original PWM enable state
            cken.cken_ctl0().write(|w| unsafe { w.bits(bits.bits() & !(0x1FF << 2)) });
            for _ in 0..100 {
                core::hint::spin_loop();
            }
            // Restore only the originally-enabled PWM bits, preserving other fields
            let current = cken.cken_ctl0().read();
            cken.cken_ctl0().write(|w| unsafe { w.bits(current.bits() | pwm_orig) });
        } else {
            let (reg, bit) = peripheral.cken_info();
            // Save original enable state
            let orig = self.read_cken_bit(reg, bit);
            self.write_cken_bit(reg, bit, false);
            for _ in 0..100 {
                core::hint::spin_loop();
            }
            // Restore original enable state, don't force-enable
            self.write_cken_bit(reg, bit, orig);
        }
    }
}

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
            Peripheral::Uart0, Peripheral::Uart1, Peripheral::Uart2,
            Peripheral::I2c0, Peripheral::I2c1,
            Peripheral::Spi0, Peripheral::Spi1,
            Peripheral::Pwm, Peripheral::Timer,
            Peripheral::Lsadc, Peripheral::Tsensor,
            Peripheral::I2s, Peripheral::Dma, Peripheral::Sdma,
            Peripheral::Sfc, Peripheral::Trng, Peripheral::SecurityGroup,
        ];
        for i in 0..variants.len() {
            for j in (i + 1)..variants.len() {
                assert_ne!(variants[i], variants[j]);
            }
        }
    }
}
