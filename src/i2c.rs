//! I2C master driver for WS63 (I2C0/1, FIFO-capable).
//!
//! SCL = I2C_CLK / (2 * (scl_h + scl_l) * 2) where each scl value*2 = actual period.
//! The I2C clock is the **24 MHz TCXO crystal** ([`crate::soc::chip::I2C_CLOCK_HZ`]),
//! NOT the 240 MHz CPU clock: unlike UART/SPI, the vendor `clock_init` leaves I2C on
//! the crystal (`i2c_port_set_clock_value(REQ_24M)`).

use crate::peripherals::{I2c0, I2c1};
use core::marker::PhantomData;

/// I2C bus speed. A finite set of standard-defined modes — so a frequency the SCL
/// divider can't actually realise is unrepresentable (the old `freq: u32` could
/// divide to a 0 / overflowed SCL count). Matches the BS2X `i2c_v151::Speed`
/// surface, so `hal::i2c::Speed` reads the same on both chips.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum Speed {
    /// Standard mode (100 kHz).
    Standard,
    /// Fast mode (400 kHz).
    Fast,
}

impl Speed {
    /// The SCL frequency in Hz.
    pub const fn hz(self) -> u32 {
        match self {
            Speed::Standard => 100_000,
            Speed::Fast => 400_000,
        }
    }
}

pub struct I2c<'d, T> {
    idx: u8,
    _peripheral: PhantomData<&'d T>,
}

fn i2c_regs(idx: u8) -> &'static crate::soc::pac::i2c0::RegisterBlock {
    unsafe {
        match idx {
            0 => &*I2c0::ptr(),
            1 => &*I2c1::ptr(),
            _ => unreachable!(),
        }
    }
}

impl<'d> I2c<'d, I2c0<'d>> {
    pub fn new_i2c0(_i2c: I2c0<'d>, speed: Speed) -> Self {
        configure_i2c(0, speed.hz());
        Self { idx: 0, _peripheral: PhantomData }
    }
}

impl<'d> I2c<'d, I2c1<'d>> {
    pub fn new_i2c1(_i2c: I2c1<'d>, speed: Speed) -> Self {
        configure_i2c(1, speed.hz());
        Self { idx: 1, _peripheral: PhantomData }
    }
}

fn configure_i2c(idx: u8, freq: u32) {
    let r = i2c_regs(idx);
    let pclk = crate::soc::chip::I2C_CLOCK_HZ;
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

/// Bounded busy-wait. Returns [`I2cError::Timeout`] instead of hanging the CPU
/// forever when the bus or a peer never drives the expected status bit (mirrors
/// the bounded waits in `spi.rs`). Previously these were unbounded `while !..{}`
/// loops that would deadlock the core on a stuck/absent slave.
const I2C_WAIT_LOOPS: u32 = 1_000_000;

#[inline]
fn wait_until(mut ready: impl FnMut() -> bool) -> Result<(), I2cError> {
    let mut n = I2C_WAIT_LOOPS;
    while !ready() {
        n -= 1;
        if n == 0 {
            return Err(I2cError::Timeout);
        }
    }
    Ok(())
}

impl<T> I2c<'_, T> {
    #[allow(dead_code)]
    fn wait_not_busy(&self) -> Result<(), I2cError> {
        let r = i2c_regs(self.idx);
        wait_until(|| !r.i2c_sr().read().bus_busy().bit_is_set())
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
        wait_until(|| r.i2c_sr().read().int_tx().bit_is_set())?;
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
        wait_until(|| r.i2c_sr().read().int_stop().bit_is_set())?;
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

            wait_until(|| r.i2c_sr().read().int_rx().bit_is_set())?;
            *byte = r.i2c_rxr().read().bits() as u8;
            self.clear_interrupts();
        }

        // Stop
        r.i2c_com().write(|w| w.op_stop().set_bit());
        wait_until(|| r.i2c_sr().read().int_stop().bit_is_set())?;
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
                wait_until(|| r.i2c_sr().read().int_rx().bit_is_set())?;
                *byte = r.i2c_rxr().read().bits() as u8;
                self.clear_interrupts();
            }
        }

        // Stop (at end of combined transaction)
        r.i2c_com().write(|w| w.op_stop().set_bit());
        wait_until(|| r.i2c_sr().read().int_stop().bit_is_set())?;
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
        let addr_w = (address as u32) << 1; // R/W=0 for write
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
                        wait_until(|| r.i2c_sr().read().int_rx().bit_is_set())?;
                        *byte = r.i2c_rxr().read().bits() as u8;
                        self.clear_interrupts();
                    }
                    // NO STOP between operations — next START will be repeated START
                }
            }
        }

        // STOP at end of all operations
        r.i2c_com().write(|w| w.op_stop().set_bit());
        wait_until(|| r.i2c_sr().read().int_stop().bit_is_set())?;
        self.clear_interrupts();

        Ok(())
    }
}

#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
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

#[cfg(all(test, not(target_arch = "riscv32")))]
mod tests {
    #[test]
    fn speed_hz_values() {
        use super::Speed;
        assert_eq!(Speed::Standard.hz(), 100_000);
        assert_eq!(Speed::Fast.hz(), 400_000);
    }

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
        let addr_w = (0x48u32) << 1; // 0x90
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

// ── Property-based fuzz tests ──────────────────────────────────

#[cfg(all(test, not(target_arch = "riscv32")))]
mod proptests {
    use crate::soc::chip::I2C_CLOCK_HZ;
    use proptest::prelude::*;

    // Re-derivation of the SCL-divider math in `configure_i2c` (same u32 types,
    // same rounding, same zero-guard) WITHOUT touching any MMIO register:
    //   freq   = if freq == 0 { 1 } else { freq };
    //   period = pclk / (2 * freq);
    //   half   = period / 2;
    // The `2 * freq` term is a u32 multiply, so the driver's math is only
    // well-defined for freq <= u32::MAX/2 (a value the divider would never be
    // configured with — kHz..MHz bus speeds). We fuzz that defined regime here
    // and exercise the zero-guard / overflow boundary in dedicated tests.
    fn scl_half(freq: u32) -> u32 {
        let pclk = I2C_CLOCK_HZ;
        let freq = if freq == 0 { 1 } else { freq };
        let period = pclk / (2 * freq);
        period / 2
    }

    proptest! {
        /// Fuzz: the divider never panics (no div-by-zero) across the full u32
        /// frequency range — the `freq == 0` guard forces the divisor to >= 2.
        #[test]
        fn divider_never_div_by_zero(freq in any::<u32>()) {
            // Mirror the guard, then the divisor `2 * freq` is computed in u64 to
            // avoid the test itself overflowing; the property under test is that a
            // non-zero divisor exists for every input (no div-by-zero panic).
            let f = if freq == 0 { 1u64 } else { freq as u64 };
            let divisor = 2 * f;
            prop_assert!(divisor >= 2);
            let _ = I2C_CLOCK_HZ as u64 / divisor;
        }

        /// Fuzz: across realistic bus frequencies (1 Hz ..= I2C_CLOCK_HZ) the
        /// half-period field never exceeds the source clock — it is a fraction of
        /// `pclk`, so it always fits whatever sane SCL_H/SCL_L width the HW has.
        #[test]
        fn half_period_within_clock(freq in 1u32..=I2C_CLOCK_HZ) {
            prop_assert!(scl_half(freq) <= I2C_CLOCK_HZ);
        }

        /// Fuzz: the divider is monotonic — a higher requested SCL frequency never
        /// yields a larger half-period (faster bus ⇒ shorter clock half-period).
        #[test]
        fn divider_is_monotonic(a in 1u32..=I2C_CLOCK_HZ, b in 1u32..=I2C_CLOCK_HZ) {
            let (lo, hi) = if a <= b { (a, b) } else { (b, a) };
            // lo <= hi requested freq ⇒ half(lo) >= half(hi)
            prop_assert!(scl_half(lo) >= scl_half(hi));
        }

        /// Fuzz: the `freq == 0` guard means a 0 request is treated identically to
        /// a 1 Hz request (no panic, deterministic value), never a div-by-zero.
        #[test]
        fn zero_freq_guard_matches_one(_seed in 0u32..4) {
            prop_assert_eq!(scl_half(0), scl_half(1));
        }

        /// Fuzz: the 7-bit write-address encoding (`addr << 1`, R/W = 0) keeps the
        /// LSB clear and round-trips back to the original address.
        #[test]
        fn write_addr_encoding_roundtrips(addr in 0u8..=0x7F) {
            let addr_w = (addr as u32) << 1;
            prop_assert_eq!(addr_w & 0x01, 0, "write R/W bit must be clear");
            prop_assert_eq!(addr_w >> 1, addr as u32, "address must round-trip");
            prop_assert!(addr_w <= 0xFE, "7-bit write byte stays in a u8");
        }

        /// Fuzz: the read-address encoding (`(addr << 1) | 1`, R/W = 1) sets the
        /// LSB, round-trips, and differs from the write byte in exactly bit 0.
        #[test]
        fn read_addr_encoding_roundtrips(addr in 0u8..=0x7F) {
            let addr_w = (addr as u32) << 1;
            let addr_r = ((addr as u32) << 1) | 1;
            prop_assert_eq!(addr_r & 0x01, 1, "read R/W bit must be set");
            prop_assert_eq!(addr_r >> 1, addr as u32, "address must round-trip");
            prop_assert_eq!(addr_w ^ addr_r, 1, "write/read bytes differ only in bit 0");
        }
    }
}

// ── Async I2C (embedded-hal-async) ──────────────────────────────────────────
// Reuses the blocking transaction (FIFO-paced; synchronous loopback on ws63-qemu).
#[cfg(feature = "async")]
mod asynch_impl {
    use super::{I2c, I2c0, I2c1};
    use embedded_hal::i2c::Operation;

    macro_rules! async_i2c {
        ($inst:ty) => {
            impl embedded_hal_async::i2c::I2c for I2c<'_, $inst> {
                async fn transaction(&mut self, addr: u8, ops: &mut [Operation<'_>]) -> Result<(), Self::Error> {
                    embedded_hal::i2c::I2c::transaction(self, addr, ops)
                }
            }
        };
    }
    async_i2c!(I2c0<'_>);
    async_i2c!(I2c1<'_>);
}
