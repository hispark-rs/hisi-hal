//! SPI master driver for WS63 (SPI0/1, SSI v151).
//! SCK = SSI_CLK / (2 * (1 + CLK_DIV)), default SSI_CLK = 240MHz.

use core::marker::PhantomData;
use crate::peripherals::{Spi0, Spi1};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SpiMode { Mode0, Mode1, Mode2, Mode3 }

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
    let mut div = pclk / (2 * config.frequency);
    if div > 0 { div -= 1; }
    if div > 0xFFFF { div = 0xFFFF; }
    r.spi_brs().write(|w| unsafe { w.bits(div as u32) });

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
            if i < read.len() { read[i] = rx as u8; }
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

    pub fn register_block(&self) -> &'static ws63_pac::spi0::RegisterBlock { spi_regs(self.idx) }
}

#[derive(Debug)]
pub enum SpiError { Overflow }

impl embedded_hal::spi::Error for SpiError {
    fn kind(&self) -> embedded_hal::spi::ErrorKind { embedded_hal::spi::ErrorKind::Other }
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
        for i in 0..buf.len() {
            let tx = buf[i] as u32;
            while !r.spi_wsr().read().txfnf().bit_is_set() {}
            unsafe { r.spi_dr().write(|w| w.bits(tx)) };
            while !r.spi_wsr().read().rxfne().bit_is_set() {}
            buf[i] = r.spi_dr().read().bits() as u8;
        }
        Ok(())
    }
    fn flush(&mut self) -> Result<(), Self::Error> { Ok(()) }
}
