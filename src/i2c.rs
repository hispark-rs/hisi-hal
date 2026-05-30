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
    let freq = if freq == 0 { 1 } else { freq };
    let period = pclk / (2 * freq);
    let half = period / 2;
    r.i2c_scl_h().write(|w| unsafe { w.bits(half) });
    r.i2c_scl_l().write(|w| unsafe { w.bits(half) });
    // Enable I2C in FIFO mode with all interrupts unmasked
    r.i2c_ctrl().write(|w| unsafe {
        w.bits(0);
        w.i2c_en().set_bit();
        w.mode_ctrl().set_bit();
        // Unmask ACK error interrupt so we can detect NACK
        w.int_ack_err_mask().set_bit()
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
        // Write 1 to each bit to clear: done, arb_loss, ack_err, rx, tx, stop, start, rxtide, txtide
        unsafe { r.i2c_icr().write(|w| w.bits(0x7FF)) };
    }

    /// Check for ACK error (NACK from slave). Reads I2C_SR bit[2] per fbb_ws63.
    fn check_ack(&self) -> Result<(), I2cError> {
        let r = i2c_regs(self.idx);
        if r.i2c_sr().read().int_ack_err().bit_is_set() {
            return Err(I2cError::Ack);
        }
        Ok(())
    }

    /// Wait for TX ready and check for ACK error.
    fn wait_tx_ack(&self) -> Result<(), I2cError> {
        let r = i2c_regs(self.idx);
        while !r.i2c_sr().read().int_tx().bit_is_set() {}
        // Check ACK after each TX (address and data bytes)
        self.check_ack()
    }

    /// Send START + address + R/W bit via I2C_COM register.
    /// Uses direct write (not RMW) to avoid restoring auto-cleared bits.
    fn send_start(&self, addr_byte: u32, is_read: bool) -> Result<(), I2cError> {
        let r = i2c_regs(self.idx);
        self.clear_interrupts();

        // Load address into TXR
        r.i2c_txr().write(|w| unsafe { w.bits(addr_byte) });

        // Direct write to COM register (bits[3:0] auto-clear after operation)
        unsafe {
            r.i2c_com().write(|w| w.bits(0));
        }
        let mut com: u32 = 0;
        com |= 1 << 3; // op_start
        com |= 1 << 1; // op_we (write address byte)
        if is_read {
            // For read, after START+WE sends address+R, we need START+RD
            // The direction bit is encoded in addr_byte, so for address+R/W=1
            // we just send START+WE; the read command comes separately
        }
        unsafe {
            r.i2c_com().write(|w| w.bits(com));
        }

        self.wait_tx_ack()
    }

    pub fn write(&mut self, addr: u8, data: &[u8]) -> Result<(), I2cError> {
        let r = i2c_regs(self.idx);

        // Start + address (R/W=0)
        self.send_start((addr as u32) << 1, false)?;

        // Write data bytes
        for &byte in data {
            r.i2c_txr().write(|w| unsafe { w.bits(byte as u32) });
            // Direct write to COM: op_we
            unsafe { r.i2c_com().write(|w| w.bits(1 << 1)) };
            self.wait_tx_ack()?;
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

        // Start + address (R/W=1)
        self.send_start(((addr as u32) << 1) | 1, true)?;

        // Read bytes
        let buf_len = buf.len();
        for (i, byte) in buf.iter_mut().enumerate() {
            let is_last = i == buf_len - 1;
            // Direct write to COM: op_rd, optionally op_ack (NACK on last byte)
            let mut com: u32 = 1 << 2; // op_rd
            if is_last {
                com |= 1 << 4; // op_ack (host sends NACK on last byte)
            }
            unsafe { r.i2c_com().write(|w| w.bits(com)) };

            while !r.i2c_sr().read().int_rx().bit_is_set() {}
            *byte = r.i2c_rxr().read().bits() as u8;
            self.clear_interrupts();
        }

        // Stop
        r.i2c_com().write(|w| w.op_stop().set_bit());
        while !r.i2c_sr().read().int_stop().bit_is_set() {}
        self.clear_interrupts();

        Ok(())
    }

    /// Combined write-then-read with repeated START (Sr) between operations,
    /// matching the I2C specification for register-based device access.
    pub fn write_read(&mut self, addr: u8, wr_buf: &[u8], rd_buf: &mut [u8]) -> Result<(), I2cError> {
        let r = i2c_regs(self.idx);

        if !wr_buf.is_empty() {
            // Start + address (R/W=0)
            self.send_start((addr as u32) << 1, false)?;

            // Write register address / data bytes
            for &byte in wr_buf {
                r.i2c_txr().write(|w| unsafe { w.bits(byte as u32) });
                unsafe { r.i2c_com().write(|w| w.bits(1 << 1)) }; // op_we
                self.wait_tx_ack()?;
                self.clear_interrupts();
            }
            // NO STOP here — go directly to repeated START
        }

        if !rd_buf.is_empty() {
            // Repeated START + address (R/W=1)
            self.send_start(((addr as u32) << 1) | 1, true)?;

            let buf_len = rd_buf.len();
            for (i, byte) in rd_buf.iter_mut().enumerate() {
                let is_last = i == buf_len - 1;
                let mut com: u32 = 1 << 2; // op_rd
                if is_last {
                    com |= 1 << 4; // op_ack (NACK on last)
                }
                unsafe { r.i2c_com().write(|w| w.bits(com)) };
                while !r.i2c_sr().read().int_rx().bit_is_set() {}
                *byte = r.i2c_rxr().read().bits() as u8;
                self.clear_interrupts();
            }
        }

        // Stop (at end of combined transaction)
        r.i2c_com().write(|w| w.op_stop().set_bit());
        while !r.i2c_sr().read().int_stop().bit_is_set() {}
        self.clear_interrupts();

        Ok(())
    }

    /// Core transaction implementation with repeated START support.
    ///
    /// Uses repeated START (Sr) between operations as required by the
    /// embedded-hal I2c trait contract. Only emits STOP after the last operation.
    fn transaction_impl(
        &mut self,
        address: u8,
        operations: &mut [embedded_hal::i2c::Operation<'_>],
    ) -> Result<(), I2cError> {
        let r = i2c_regs(self.idx);
        let addr_w = (address as u32) << 1;      // R/W=0 for write
        let addr_r = ((address as u32) << 1) | 1; // R/W=1 for read

        for op in operations.iter_mut() {
            match op {
                embedded_hal::i2c::Operation::Write(data) => {
                    self.send_start(addr_w, false)?;
                    self.clear_interrupts();

                    for &byte in data.iter() {
                        r.i2c_txr().write(|w| unsafe { w.bits(byte as u32) });
                        unsafe { r.i2c_com().write(|w| w.bits(1 << 1)) }; // op_we
                        self.wait_tx_ack()?;
                        self.clear_interrupts();
                    }
                    // NO STOP between operations — next START will be repeated START
                }
                embedded_hal::i2c::Operation::Read(buf) => {
                    self.send_start(addr_r, true)?;
                    self.clear_interrupts();

                    let buf_len = buf.len();
                    for (i, byte) in buf.iter_mut().enumerate() {
                        let is_last = i == buf_len - 1;
                        let mut com: u32 = 1 << 2; // op_rd
                        if is_last {
                            com |= 1 << 4; // op_ack (host sends NACK on last byte)
                        }
                        unsafe { r.i2c_com().write(|w| w.bits(com)) };
                        while !r.i2c_sr().read().int_rx().bit_is_set() {}
                        *byte = r.i2c_rxr().read().bits() as u8;
                        self.clear_interrupts();
                    }
                    // NO STOP between operations — next START will be repeated START
                }
            }
        }

        // STOP at end of all operations
        r.i2c_com().write(|w| w.op_stop().set_bit());
        while !r.i2c_sr().read().int_stop().bit_is_set() {}
        self.clear_interrupts();

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

// ── embedded-hal I2c trait ──────────────────────────────────────

impl embedded_hal::i2c::ErrorType for I2c<'_, I2c0<'_>> {
    type Error = I2cError;
}

impl embedded_hal::i2c::I2c for I2c<'_, I2c0<'_>> {
    fn transaction(
        &mut self,
        address: u8,
        operations: &mut [embedded_hal::i2c::Operation<'_>],
    ) -> Result<(), Self::Error> {
        self.transaction_impl(address, operations)
    }
}

impl embedded_hal::i2c::ErrorType for I2c<'_, I2c1<'_>> {
    type Error = I2cError;
}

impl embedded_hal::i2c::I2c for I2c<'_, I2c1<'_>> {
    fn transaction(
        &mut self,
        address: u8,
        operations: &mut [embedded_hal::i2c::Operation<'_>],
    ) -> Result<(), Self::Error> {
        self.transaction_impl(address, operations)
    }
}

// ── Tests ──────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    #[test]
    fn test_i2c_address_write_encoding() {
        // I2C write address = addr << 1 (R/W=0)
        assert_eq!((0x50u32 << 1), 0xA0);
        assert_eq!((0x50u32 << 1) & 0xFE, 0xA0); // Write bit is 0
    }

    #[test]
    fn test_i2c_address_read_encoding() {
        // I2C read address = (addr << 1) | 1 (R/W=1)
        assert_eq!(((0x50u32 << 1) | 1), 0xA1);
    }

    #[test]
    fn test_i2c_address_write_read_differ_by_one_bit() {
        let addr_w = (0x48u32) << 1;       // 0x90
        let addr_r = ((0x48u32) << 1) | 1; // 0x91
        assert_eq!(addr_r, addr_w | 1);
        assert_eq!(addr_w & 0x01, 0); // Write bit cleared
        assert_eq!(addr_r & 0x01, 1); // Read bit set
    }

    #[test]
    fn test_i2c_10bit_high_address_encoding() {
        // 10-bit addressing uses 0x78-0x7B range
        let addr: u32 = 0x78;
        let addr_w = addr << 1;
        assert_eq!(addr_w, 0xF0); // Address fits in 7 bits
    }
}
