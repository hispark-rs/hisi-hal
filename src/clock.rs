//! Clock configuration for WS63.
//!
//! The WS63 uses a CLDO_CRG (Clock and Reset Generator) for peripheral clock
//! enables, dividers, and clock source selection. The system boots at 240MHz
//! and this module provides helpers for enabling/disabling peripheral clocks.

use crate::peripherals::CldoCrg;
use crate::system::{Clocks, System};
use core::marker::PhantomData;
use portable_atomic::AtomicU8;
use core::sync::atomic::Ordering;

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

static REF_COUNTS: [AtomicU8; 17] = {
    const ZERO: AtomicU8 = AtomicU8::new(0);
    [ZERO; 17]
};

/// RAII guard that enables a peripheral clock on creation.
///
/// Uses reference counting — the clock is only disabled when
/// all guards for that peripheral are dropped.
pub struct PeripheralGuard<'d> {
    peripheral: Peripheral,
    _marker: PhantomData<&'d ()>,
}

impl Drop for PeripheralGuard<'_> {
    fn drop(&mut self) {
        let idx = self.peripheral as usize;
        let count = REF_COUNTS[idx].load(Ordering::Relaxed);
        if count <= 1 {
            REF_COUNTS[idx].store(0, Ordering::Relaxed);
        } else {
            REF_COUNTS[idx].store(count - 1, Ordering::Relaxed);
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
    pub fn peripheral_guard(&self, peripheral: Peripheral) -> PeripheralGuard<'_> {
        let idx = peripheral as usize;
        let count = REF_COUNTS[idx].load(Ordering::Relaxed);
        if count == 0 {
            let cken = self.cldo_crg.register_block();
            let (reg, bit) = peripheral.cken_info();
            if reg == 0 {
                let bits = cken.cken_ctl0().read();
                cken.cken_ctl0().write(|w| unsafe { w.bits(bits.bits() | (1 << bit)) });
            } else {
                let bits = cken.cken_ctl1().read();
                cken.cken_ctl1().write(|w| unsafe { w.bits(bits.bits() | (1 << bit)) });
            }
        }
        REF_COUNTS[idx].store(count + 1, Ordering::Relaxed);
        PeripheralGuard { peripheral, _marker: PhantomData }
    }

    /// Enable the clock gate for a specific UART instance.
    pub fn enable_uart(&self, uart_idx: usize) {
        let cken = self.cldo_crg.register_block();
        let bits = cken.cken_ctl1().read();
        let bit = match uart_idx {
            0 => 18,
            1 => 19,
            2 => 20,
            _ => unreachable!(),
        };
        cken.cken_ctl1().write(|w| unsafe { w.bits(bits.bits() | (1 << bit)) });
    }

    /// Enable the clock gate for a specific I2C instance.
    pub fn enable_i2c(&self, i2c_idx: usize) {
        // I2C clock gates are in cken_ctl0 general clock enables
        let cken = self.cldo_crg.register_block();
        let bits = cken.cken_ctl0().read();
        let bit = match i2c_idx {
            0 => 18,
            1 => 19,
            _ => unreachable!(),
        };
        cken.cken_ctl0().write(|w| unsafe { w.bits(bits.bits() | (1 << bit)) });
    }

    /// Enable the clock gate for the SPI peripheral.
    pub fn enable_spi(&self) {
        let cken = self.cldo_crg.register_block();
        let bits = cken.cken_ctl1().read();
        cken.cken_ctl1().write(|w| unsafe { w.bits(bits.bits() | (1 << 25)) });
    }

    /// Enable the clock gate for PWM.
    pub fn enable_pwm(&self) {
        let cken = self.cldo_crg.register_block();
        let bits = cken.cken_ctl0().read();
        // PWM clock gates: bits 2:10
        cken.cken_ctl0().write(|w| unsafe { w.bits(bits.bits() | (0x1FF << 2)) });
    }

    /// Enable the clock gate for TIMER.
    pub fn enable_timer(&self) {
        let cken = self.cldo_crg.register_block();
        let bits0 = cken.cken_ctl0().read();
        cken.cken_ctl0().write(|w| unsafe { w.bits(bits0.bits() | (1 << 21)) });
    }

    /// Enable the clock gate for the LSADC peripheral.
    pub fn enable_lsadc(&self) {
        let cken = self.cldo_crg.register_block();
        let bits = cken.cken_ctl0().read();
        cken.cken_ctl0().write(|w| unsafe { w.bits(bits.bits() | (1 << 22)) });
    }

    /// Enable the clock gate for the TSENSOR peripheral.
    pub fn enable_tsensor(&self) {
        let cken = self.cldo_crg.register_block();
        let bits = cken.cken_ctl0().read();
        cken.cken_ctl0().write(|w| unsafe { w.bits(bits.bits() | (1 << 23)) });
    }

    /// Enable the clock gate for the I2S peripheral.
    pub fn enable_i2s(&self) {
        let cken = self.cldo_crg.register_block();
        let bits = cken.cken_ctl0().read();
        cken.cken_ctl0().write(|w| unsafe { w.bits(bits.bits() | (1 << 24)) });
    }

    /// Enable the clock gate for the DMA peripheral.
    pub fn enable_dma(&self) {
        let cken = self.cldo_crg.register_block();
        let bits = cken.cken_ctl1().read();
        cken.cken_ctl1().write(|w| unsafe { w.bits(bits.bits() | (1 << 22)) });
    }

    /// Enable the clock gate for the SDMA peripheral.
    pub fn enable_sdma(&self) {
        let cken = self.cldo_crg.register_block();
        let bits = cken.cken_ctl1().read();
        cken.cken_ctl1().write(|w| unsafe { w.bits(bits.bits() | (1 << 23)) });
    }

    /// Enable the clock gate for the SFC (SPI Flash Controller) peripheral.
    pub fn enable_sfc(&self) {
        let cken = self.cldo_crg.register_block();
        let bits = cken.cken_ctl1().read();
        cken.cken_ctl1().write(|w| unsafe { w.bits(bits.bits() | (1 << 24)) });
    }

    /// Enable the clock gate for the TRNG peripheral.
    pub fn enable_trng(&self) {
        let cken = self.cldo_crg.register_block();
        let bits = cken.cken_ctl0().read();
        cken.cken_ctl0().write(|w| unsafe { w.bits(bits.bits() | (1 << 25)) });
    }

    /// Enable the clock gate for the security acceleration (SPACC/PKE/KM) peripherals.
    pub fn enable_security(&self) {
        let cken = self.cldo_crg.register_block();
        let bits = cken.cken_ctl0().read();
        cken.cken_ctl0().write(|w| unsafe { w.bits(bits.bits() | (1 << 26)) });
    }

    /// Disable the clock gate for a specific peripheral.
    pub fn disable_peripheral(&self, peripheral: Peripheral) {
        let cken = self.cldo_crg.register_block();
        let (reg, bit) = peripheral.cken_info();
        if reg == 0 {
            let bits = cken.cken_ctl0().read();
            cken.cken_ctl0().write(|w| unsafe { w.bits(bits.bits() & !(1 << bit)) });
        } else {
            let bits = cken.cken_ctl1().read();
            cken.cken_ctl1().write(|w| unsafe { w.bits(bits.bits() & !(1 << bit)) });
        }
    }

    /// Trigger a soft reset for a specific peripheral via CLDO_CRG.
    pub fn reset_peripheral(&self, peripheral: Peripheral) {
        let cken = self.cldo_crg.register_block();
        let (reg, bit) = peripheral.cken_info();
        // Reset: disable, then re-enable clock (power-cycle style reset)
        if reg == 0 {
            let bits = cken.cken_ctl0().read();
            cken.cken_ctl0().write(|w| unsafe { w.bits(bits.bits() & !(1 << bit)) });
            // Small delay
            for _ in 0..100 { core::hint::spin_loop(); }
            cken.cken_ctl0().write(|w| unsafe { w.bits(bits.bits() | (1 << bit)) });
        } else {
            let bits = cken.cken_ctl1().read();
            cken.cken_ctl1().write(|w| unsafe { w.bits(bits.bits() & !(1 << bit)) });
            for _ in 0..100 { core::hint::spin_loop(); }
            cken.cken_ctl1().write(|w| unsafe { w.bits(bits.bits() | (1 << bit)) });
        }
    }
}
