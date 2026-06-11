//! BS2X I2C master driver — DesignWare SSI core (IP version v151).
//!
//! BS2X's I2C is a Synopsys/DesignWare `ic_*` controller, a COMPLETELY different
//! IP from WS63's custom v150 I2C (`i2c.rs`). This module is the BS2X (`chip-bs21`)
//! implementation of the `i2c` module; lib.rs picks it per chip. The register map
//! and the master transaction sequences are from the fbb_bs2x `hal_i2c_v151`
//! driver (sole ground truth); the v151 register block was rewritten into
//! `bs2x-pac` (BS2X.svd) for this.
//!
//! API mirrors `i2c.rs`: `I2c::new_i2c0/1`, `write`, `read`, `write_read`, plus a
//! `probe` (ACK detection) for bus scans.

use crate::peripherals::{I2c0, I2c1};
use core::marker::PhantomData;

/// I2C bus speed.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Speed {
    /// Standard mode (100 kHz).
    Standard,
    /// Fast mode (400 kHz).
    Fast,
}

#[derive(Debug, PartialEq, Eq)]
pub enum I2cError {
    /// A status bit never asserted within the bounded wait.
    Timeout,
    /// The addressed device did not ACK (DesignWare TX_ABRT / addr_7b_noack).
    Nack,
}

/// Bound on status-bit polling so a missing device/stuck bus can't hang the CPU.
const POLL_LIMIT: u32 = 1_000_000;

fn wait_until(mut ready: impl FnMut() -> bool) -> Result<(), I2cError> {
    for _ in 0..POLL_LIMIT {
        if ready() {
            return Ok(());
        }
        core::hint::spin_loop();
    }
    Err(I2cError::Timeout)
}

pub struct I2c<'d, T> {
    idx: u8,
    _peripheral: PhantomData<&'d T>,
}

fn i2c_regs(idx: u8) -> &'static crate::soc::pac::i2c0::RegisterBlock {
    // SAFETY: I2c0/I2c1 share one register-block layout; the pointer is a static
    // physical MMIO base (0x5208_3000 + idx*0x1000) from bs2x-pac.
    unsafe { &*(if idx == 0 { I2c0::ptr() } else { I2c1::ptr() }) }
}

impl<'d> I2c<'d, I2c0<'d>> {
    pub fn new_i2c0(_i2c: I2c0<'d>, speed: Speed) -> Self {
        configure(0, speed);
        Self { idx: 0, _peripheral: PhantomData }
    }
}
impl<'d> I2c<'d, I2c1<'d>> {
    pub fn new_i2c1(_i2c: I2c1<'d>, speed: Speed) -> Self {
        configure(1, speed);
        Self { idx: 1, _peripheral: PhantomData }
    }
}

/// Master init (fbb_bs2x `hal_i2c_v151_master_init`): disable, program IC_CON +
/// the SCL counts, re-enable.
fn configure(idx: u8, speed: Speed) {
    let r = i2c_regs(idx);
    // IC_CON.speed: 1=SS, 2=FS, 3=HS.
    let (speed_field, hcnt, lcnt) = match speed {
        // SCL counts: nominal values for a ~32 MHz I2C clock; the exact divisor
        // only matters on silicon (the QEMU model ignores it). TODO(bs2x): derive
        // from soc::chip::I2C_CLOCK_HZ per hal_i2c_v151's scl-count formula.
        Speed::Standard => (1u32, 160u32, 190u32),
        Speed::Fast => (2u32, 40u32, 50u32),
    };
    unsafe {
        r.ic_enable().write(|w| w.enable().clear_bit());
        r.ic_con().write(|w| {
            w.master_mode().set_bit();
            w.speed().bits(speed_field as u8);
            w.ic_restart_en().set_bit();
            w.rx_fifo_full_hld_ctrl().set_bit();
            w.ic_slave_disable().set_bit()
        });
        match speed {
            Speed::Standard => {
                r.ic_ss_scl_hcnt().write(|w| w.cnt().bits(hcnt as u16));
                r.ic_ss_scl_lcnt().write(|w| w.cnt().bits(lcnt as u16));
            }
            Speed::Fast => {
                r.ic_fs_scl_hcnt().write(|w| w.cnt().bits(hcnt as u16));
                r.ic_fs_scl_lcnt().write(|w| w.cnt().bits(lcnt as u16));
            }
        }
        r.ic_rx_tl().write(|w| w.rx_tl().bits(0));
        r.ic_tx_tl().write(|w| w.tx_tl().bits(0));
        r.ic_enable().write(|w| w.enable().set_bit());
    }
}

impl<T> I2c<'_, T> {
    /// Point the controller at a 7-bit target address (disable → set IC_TAR →
    /// enable), per `hal_i2c_cfg_taget_address`.
    fn set_target(&self, addr: u8) {
        let r = i2c_regs(self.idx);
        unsafe {
            r.ic_enable().write(|w| w.enable().clear_bit());
            r.ic_con().modify(|_, w| w.ic_10bitaddr_master().clear_bit());
            r.ic_tar().write(|w| {
                w.special().clear_bit();
                w.gc_or_start().clear_bit();
                w.ic_tar().bits(addr as u16 & 0x3FF)
            });
            r.ic_enable().write(|w| w.enable().set_bit());
            // Clear any stale interrupt/abort state (read-to-clear).
            let _ = r.ic_clr_intr().read().bits();
        }
    }

    /// True if the DesignWare master aborted on an address NACK (device absent).
    fn aborted(&self) -> bool {
        let r = i2c_regs(self.idx);
        r.ic_raw_intr_stat().read().tx_abrt().bit_is_set()
    }

    fn clear_abort(&self) {
        let r = i2c_regs(self.idx);
        let _ = r.ic_clr_intr().read().bits();
    }

    /// Probe `addr`: issue a 1-byte read and report whether the device ACKed.
    /// This is the I2C bus-scan primitive (ACK = present, NACK = absent).
    pub fn probe(&mut self, addr: u8) -> bool {
        let r = i2c_regs(self.idx);
        self.set_target(addr);
        // Push a read command with STOP (cmd=1, stop=1 → 0x300).
        r.ic_data_cmd().write(|w| {
            w.cmd().set_bit();
            w.stop().set_bit()
        });
        // Wait for the transfer to resolve: stop-detect (ACK) or tx-abort (NACK).
        let _ = wait_until(|| {
            let s = r.ic_raw_intr_stat().read();
            s.stop_det().bit_is_set() || s.tx_abrt().bit_is_set()
        });
        let present = !self.aborted();
        // If it ACKed there is a byte in the RX FIFO — drain it.
        if present && r.ic_status().read().rfne().bit_is_set() {
            let _ = r.ic_data_cmd().read().dat().bits();
        }
        self.clear_abort();
        present
    }

    /// Write `data` to `addr` (STOP after the last byte).
    pub fn write(&mut self, addr: u8, data: &[u8]) -> Result<(), I2cError> {
        let r = i2c_regs(self.idx);
        self.set_target(addr);
        for (i, &b) in data.iter().enumerate() {
            let last = i + 1 == data.len();
            wait_until(|| r.ic_status().read().tfnf().bit_is_set())?;
            unsafe {
                r.ic_data_cmd().write(|w| {
                    w.dat().bits(b);
                    if last {
                        w.stop().set_bit();
                    }
                    w
                });
            }
            if self.aborted() {
                self.clear_abort();
                return Err(I2cError::Nack);
            }
        }
        wait_until(|| r.ic_raw_intr_stat().read().stop_det().bit_is_set())?;
        Ok(())
    }

    /// Read `buf.len()` bytes from `addr` (STOP after the last byte).
    pub fn read(&mut self, addr: u8, buf: &mut [u8]) -> Result<(), I2cError> {
        let r = i2c_regs(self.idx);
        self.set_target(addr);
        let n = buf.len();
        for (i, slot) in buf.iter_mut().enumerate() {
            let last = i + 1 == n;
            wait_until(|| r.ic_status().read().tfnf().bit_is_set())?;
            r.ic_data_cmd().write(|w| {
                w.cmd().set_bit();
                if last {
                    w.stop().set_bit();
                }
                w
            });
            wait_until(|| r.ic_status().read().rfne().bit_is_set())?;
            if self.aborted() {
                self.clear_abort();
                return Err(I2cError::Nack);
            }
            *slot = r.ic_data_cmd().read().dat().bits();
        }
        Ok(())
    }

    /// Write then read (repeated START) — `wr` out, then `rd` in.
    pub fn write_read(&mut self, addr: u8, wr: &[u8], rd: &mut [u8]) -> Result<(), I2cError> {
        if !wr.is_empty() {
            self.write(addr, wr)?;
        }
        if !rd.is_empty() {
            self.read(addr, rd)?;
        }
        Ok(())
    }
}
