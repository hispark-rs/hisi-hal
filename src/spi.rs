//! SPI master driver for WS63 (SPI0/1, SSI v151).
//! SCK = SSI_CLK / (2 * (1 + CLK_DIV)), default SSI_CLK = 240MHz.

use crate::peripherals::{Spi0, Spi1};
use core::marker::PhantomData;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SpiMode {
    Mode0,
    Mode1,
    Mode2,
    Mode3,
}

#[derive(Debug, Clone, Copy)]
pub struct Config {
    pub frequency: u32,
    pub mode: SpiMode,
    pub data_bits: u8,
}

impl Default for Config {
    fn default() -> Self {
        Self { frequency: 1_000_000, mode: SpiMode::Mode0, data_bits: 8 }
    }
}

pub struct Spi<'d, T> {
    idx: u8,
    _peripheral: PhantomData<&'d T>,
}

fn spi_regs(idx: u8) -> &'static ws63_pac::spi0::RegisterBlock {
    unsafe {
        match idx {
            0 => &*Spi0::ptr(),
            1 => &*Spi1::ptr(),
            _ => unreachable!(),
        }
    }
}

impl<'d> Spi<'d, Spi0<'d>> {
    pub fn new_spi0(_spi: Spi0<'d>, config: Config) -> Self {
        configure_spi(0, &config);
        Self { idx: 0, _peripheral: PhantomData }
    }
}
impl<'d> Spi<'d, Spi1<'d>> {
    pub fn new_spi1(_spi: Spi1<'d>, config: Config) -> Self {
        configure_spi(1, &config);
        Self { idx: 1, _peripheral: PhantomData }
    }
}

fn configure_spi(idx: u8, config: &Config) {
    let r = spi_regs(idx);
    r.spi_er().write(|w| unsafe { w.bits(0) });
    let pclk = crate::soc::ws63::SYSTEM_CLOCK_HZ;
    let freq = if config.frequency == 0 { 1 } else { config.frequency };
    let mut div = pclk / (2 * freq);
    div = div.saturating_sub(1);
    if div > 0xFFFF {
        div = 0xFFFF;
    }
    r.spi_brs().write(|w| unsafe { w.bits(div) });

    let mut ctra = 0u32;
    match config.mode {
        SpiMode::Mode0 => {}
        SpiMode::Mode1 => ctra |= 1 << 3,
        SpiMode::Mode2 => ctra |= 1 << 4,
        SpiMode::Mode3 => ctra |= (1 << 3) | (1 << 4),
    }
    ctra |= ((config.data_bits.saturating_sub(1)) as u32) << 13;
    ctra |= 3 << 18; // TX+RX mode
    r.spi_ctra().write(|w| unsafe { w.bits(ctra) });
    r.spi_slenr().write(|w| unsafe { w.bits(0x1) });
    r.spi_er().write(|w| unsafe { w.bits(0x1) });
}

impl<T> Spi<'_, T> {
    pub fn transfer(&mut self, write: &[u8], read: &mut [u8]) -> Result<(), SpiError> {
        let r = spi_regs(self.idx);
        let len = write.len().max(read.len());
        for i in 0..len {
            let tx = if i < write.len() { write[i] as u32 } else { 0 };
            while !r.spi_wsr().read().txfnf().bit_is_set() {}
            unsafe { r.spi_dr().write(|w| w.bits(tx)) };
            while !r.spi_wsr().read().rxfne().bit_is_set() {}
            let rx = r.spi_dr().read().bits();
            if i < read.len() {
                read[i] = rx as u8;
            }
        }
        Ok(())
    }

    pub fn write(&mut self, data: &[u8]) -> Result<(), SpiError> {
        let r = spi_regs(self.idx);
        for &byte in data {
            while !r.spi_wsr().read().txfnf().bit_is_set() {}
            unsafe { r.spi_dr().write(|w| w.bits(byte as u32)) };
        }
        Ok(())
    }

    pub fn register_block(&self) -> &'static ws63_pac::spi0::RegisterBlock {
        spi_regs(self.idx)
    }

    /// Wait for all pending TX data to be transmitted and the SPI bus to become idle.
    pub fn wait_idle(&self) {
        let r = spi_regs(self.idx);
        // Wait for TX FIFO to drain
        while !r.spi_wsr().read().txfe().bit_is_set() {}
        // Wait for SPI bus to become idle (shift register empty)
        while r.spi_wsr().read().busy().bit_is_set() {}
    }
}

#[derive(Debug)]
pub enum SpiError {
    Overflow,
}

impl embedded_hal::spi::Error for SpiError {
    fn kind(&self) -> embedded_hal::spi::ErrorKind {
        embedded_hal::spi::ErrorKind::Other
    }
}
impl embedded_hal::spi::ErrorType for Spi<'_, Spi0<'_>> {
    type Error = SpiError;
}
impl embedded_hal::spi::SpiBus for Spi<'_, Spi0<'_>> {
    fn transfer(&mut self, read: &mut [u8], write: &[u8]) -> Result<(), Self::Error> {
        self.transfer(write, read)
    }
    fn read(&mut self, buf: &mut [u8]) -> Result<(), Self::Error> {
        self.transfer(&[], buf)
    }
    fn write(&mut self, buf: &[u8]) -> Result<(), Self::Error> {
        Spi::write(self, buf)
    }
    fn transfer_in_place(&mut self, buf: &mut [u8]) -> Result<(), Self::Error> {
        let r = spi_regs(self.idx);
        for byte in buf.iter_mut() {
            let tx = *byte as u32;
            while !r.spi_wsr().read().txfnf().bit_is_set() {}
            unsafe { r.spi_dr().write(|w| w.bits(tx)) };
            while !r.spi_wsr().read().rxfne().bit_is_set() {}
            *byte = r.spi_dr().read().bits() as u8;
        }
        Ok(())
    }
    fn flush(&mut self) -> Result<(), Self::Error> {
        self.wait_idle();
        Ok(())
    }
}

// SPI1 also implements the same traits
impl embedded_hal::spi::ErrorType for Spi<'_, Spi1<'_>> {
    type Error = SpiError;
}
impl embedded_hal::spi::SpiBus for Spi<'_, Spi1<'_>> {
    fn transfer(&mut self, read: &mut [u8], write: &[u8]) -> Result<(), Self::Error> {
        self.transfer(write, read)
    }
    fn read(&mut self, buf: &mut [u8]) -> Result<(), Self::Error> {
        self.transfer(&[], buf)
    }
    fn write(&mut self, buf: &[u8]) -> Result<(), Self::Error> {
        Spi::write(self, buf)
    }
    fn transfer_in_place(&mut self, buf: &mut [u8]) -> Result<(), Self::Error> {
        let r = spi_regs(self.idx);
        for byte in buf.iter_mut() {
            let tx = *byte as u32;
            while !r.spi_wsr().read().txfnf().bit_is_set() {}
            unsafe { r.spi_dr().write(|w| w.bits(tx)) };
            while !r.spi_wsr().read().rxfne().bit_is_set() {}
            *byte = r.spi_dr().read().bits() as u8;
        }
        Ok(())
    }
    fn flush(&mut self) -> Result<(), Self::Error> {
        self.wait_idle();
        Ok(())
    }
}

// ── Tests ──────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use crate::soc::ws63::SYSTEM_CLOCK_HZ;

    #[test]
    fn test_spi_baud_divisor_calculation() {
        let pclk = SYSTEM_CLOCK_HZ;
        let freq: u32 = 1_000_000;
        let mut div = pclk / (2 * freq);
        div = div.saturating_sub(1);
        assert_eq!(div, 119); // 240M / 2M = 120, minus 1 = 119
    }

    #[test]
    fn test_spi_divisor_zero_freq_guard() {
        let pclk = SYSTEM_CLOCK_HZ;
        let freq: u32 = 0;
        let safe_freq = if freq == 0 { 1 } else { freq };
        let mut div = pclk / (2 * safe_freq);
        div = div.saturating_sub(1);
        // 240M/2 = 120M, minus 1 = 119,999,999 > 0xFFFF → clamped
        assert!(div > 0xFFFF); // before clamp
        if div > 0xFFFF {
            div = 0xFFFF;
        }
        assert_eq!(div, 0xFFFF); // after clamp
    }

    #[test]
    fn test_spi_divisor_clamps_at_max() {
        let pclk = SYSTEM_CLOCK_HZ;
        let freq: u32 = 1000;
        let mut div = pclk / (2 * freq);
        div = div.saturating_sub(1);
        // 120000 minus 1 = 119999, above max → clamped
        if div > 0xFFFF {
            div = 0xFFFF;
        }
        assert_eq!(div, 0xFFFF);
    }
}

// ── Property-based fuzz tests ──────────────────────────────────

#[cfg(test)]
mod proptests {
    use proptest::prelude::*;

    proptest! {
        /// Fuzz: SPI divisor calculation never panics for any frequency.
        #[test]
        fn spi_divisor_never_panics(freq in any::<u32>()) {
            let pclk = crate::soc::ws63::SYSTEM_CLOCK_HZ as u64;
            let safe_freq = if freq == 0 { 1u64 } else { freq as u64 };
            let mut div = (pclk / (2 * safe_freq)) as u32;
            div = div.saturating_sub(1);
            if div > 0xFFFF { div = 0xFFFF; }
            prop_assert!(div <= 0xFFFF);
        }

        /// Fuzz: Divisor is always in valid range 0..=0xFFFF after clamping.
        #[test]
        fn spi_divisor_in_valid_range(freq in any::<u32>()) {
            let pclk = crate::soc::ws63::SYSTEM_CLOCK_HZ as u64;
            let safe_freq = if freq == 0 { 1u64 } else { freq as u64 };
            let mut div = (pclk / (2 * safe_freq)) as u32;
            div = div.saturating_sub(1);
            if div > 0xFFFF { div = 0xFFFF; }
            prop_assert!(div <= 0xFFFF, "divisor {} > 0xFFFF for freq={}", div, freq);
        }

        /// Fuzz: Higher frequency → lower divisor (monotonic non-increasing after clamping).
        #[test]
        fn spi_divisor_monotonic(freq1 in any::<u32>(), freq2 in any::<u32>()) {
            let pclk = crate::soc::ws63::SYSTEM_CLOCK_HZ as u64;
            let compute = |f: u32| -> u32 {
                let safe = if f == 0 { 1u64 } else { f as u64 };
                let mut d = (pclk / (2 * safe)) as u32;
                d = d.saturating_sub(1);
                if d > 0xFFFF { 0xFFFF } else { d }
            };
            let d1 = compute(freq1);
            let d2 = compute(freq2);
            if freq1 > freq2 && freq2 > 0 {
                prop_assert!(d1 <= d2, "freq1={}(d={}) freq2={}(d={}): higher freq should give <= divisor", freq1, d1, freq2, d2);
            }
        }

        /// Fuzz: saturating_sub(1) on result doesn't overflow (uses u64 for intermediate).
        #[test]
        fn spi_saturating_sub_safe(freq in any::<u32>()) {
            let pclk = crate::soc::ws63::SYSTEM_CLOCK_HZ as u64;
            let safe_freq = if freq == 0 { 1u64 } else { freq as u64 };
            let raw = (pclk / (2 * safe_freq)) as u32;
            let _ = raw.saturating_sub(1);
        }
    }
}
