//! I2C master driver for WS63 (I2C0/1, FIFO-capable).
//!
//! I2C clock = PCLK / (scl_h + scl_l) where each value*2 = actual period.

use crate::peripherals::{I2c0, I2c1};
use core::marker::PhantomData;

pub struct I2c<'d, T> {
    idx: u8,
    _peripheral: PhantomData<&'d T>,
}

fn i2c_regs(idx: u8) -> &'static ws63_pac::i2c0::RegisterBlock {
    unsafe {
        match idx {
            0 => &*I2c0::ptr(),
            1 => &*I2c1::ptr(),
            _ => unreachable!(),
        }
    }
}

impl<'d> I2c<'d, I2c0<'d>> {
    pub fn new_i2c0(_i2c: I2c0<'d>, freq: u32) -> Self {
        configure_i2c(0, freq);
        Self { idx: 0, _peripheral: PhantomData }
    }
}

impl<'d> I2c<'d, I2c1<'d>> {
    pub fn new_i2c1(_i2c: I2c1<'d>, freq: u32) -> Self {
        configure_i2c(1, freq);
        Self { idx: 1, _peripheral: PhantomData }
    }
}

fn configure_i2c(idx: u8, freq: u32) {
    let r = i2c_regs(idx);
    let pclk = crate::soc::ws63::SYSTEM_CLOCK_HZ;
    let period = pclk / (2 * freq);
    let half = period / 2;
    r.i2c_scl_h().write(|w| unsafe { w.bits(half) });
    r.i2c_scl_l().write(|w| unsafe { w.bits(half) });
    // Enable I2C in FIFO mode
    r.i2c_ctrl().write(|w| unsafe {
        w.bits(0);
        w.i2c_en().set_bit();
        w.mode_ctrl().set_bit()
    });
}

impl<T> I2c<'_, T> {
    #[allow(dead_code)]
    fn wait_not_busy(&self) {
        let r = i2c_regs(self.idx);
        while r.i2c_sr().read().bus_busy().bit_is_set() {}
    }

    fn clear_interrupts(&self) {
        let r = i2c_regs(self.idx);
        unsafe { r.i2c_icr().write(|w| w.bits(0x7FF)) };
    }

    pub fn write(&mut self, addr: u8, data: &[u8]) -> Result<(), I2cError> {
        let r = i2c_regs(self.idx);
        self.clear_interrupts();

        // Write address + R/W=0 to TXR
        r.i2c_txr().write(|w| unsafe { w.bits((addr as u32) << 1) });
        // Start + Write
        r.i2c_com().write(|w| unsafe { w.bits(0) });
        r.i2c_com().write(|w| {
            w.op_start().set_bit();
            w.op_we().set_bit()
        });

        // Wait for TX ready
        while !r.i2c_sr().read().int_tx().bit_is_set() {}
        self.clear_interrupts();

        for &byte in data {
            r.i2c_txr().write(|w| unsafe { w.bits(byte as u32) });
            r.i2c_com().write(|w| w.op_we().set_bit());
            while !r.i2c_sr().read().int_tx().bit_is_set() {}
            self.clear_interrupts();
        }

        // Stop
        r.i2c_com().write(|w| w.op_stop().set_bit());
        while !r.i2c_sr().read().int_stop().bit_is_set() {}
        self.clear_interrupts();

        Ok(())
    }

    pub fn read(&mut self, addr: u8, buf: &mut [u8]) -> Result<(), I2cError> {
        let r = i2c_regs(self.idx);
        self.clear_interrupts();

        // Write address + R/W=1
        r.i2c_txr().write(|w| unsafe { w.bits(((addr as u32) << 1) | 1) });
        r.i2c_com().write(|w| {
            w.op_start().set_bit();
            w.op_we().set_bit()
        });

        while !r.i2c_sr().read().int_tx().bit_is_set() {}
        self.clear_interrupts();

        // Read bytes
        let buf_len = buf.len();
        for (i, byte) in buf.iter_mut().enumerate() {
            let is_last = i == buf_len - 1;
            let mut com = r.i2c_com().read().bits();
            com |= 1 << 2; // op_rd
            if is_last {
                com |= 1 << 4;
            } // op_ack (NACK on last)
            r.i2c_com().write(|w| unsafe { w.bits(com) });

            while !r.i2c_sr().read().int_rx().bit_is_set() {}
            *byte = r.i2c_rxr().read().bits() as u8;
            self.clear_interrupts();
        }

        r.i2c_com().write(|w| w.op_stop().set_bit());
        while !r.i2c_sr().read().int_stop().bit_is_set() {}
        self.clear_interrupts();

        Ok(())
    }

    pub fn write_read(&mut self, addr: u8, wr_buf: &[u8], rd_buf: &mut [u8]) -> Result<(), I2cError> {
        if !wr_buf.is_empty() {
            self.write(addr, wr_buf)?;
        }
        if !rd_buf.is_empty() {
            self.read(addr, rd_buf)?;
        }
        Ok(())
    }
}

#[derive(Debug)]
pub enum I2cError {
    Ack,
    BusError,
    Timeout,
}

impl embedded_hal::i2c::Error for I2cError {
    fn kind(&self) -> embedded_hal::i2c::ErrorKind {
        match self {
            I2cError::Ack => {
                embedded_hal::i2c::ErrorKind::NoAcknowledge(embedded_hal::i2c::NoAcknowledgeSource::Unknown)
            }
            I2cError::BusError => embedded_hal::i2c::ErrorKind::Bus,
            I2cError::Timeout => embedded_hal::i2c::ErrorKind::Other,
        }
    }
}

fn _i2c_op(_addr: u8) {}
