//! eFuse (OTP Controller) driver for WS63.
//!
//! The WS63 eFuse controller manages access to one-time-programmable (OTP)
//! memory used for storing chip configuration, calibration data, and
//! security keys.
//!
//! # Read/write control
//!
//! The eFuse is controlled via a 16-bit control data field plus a
//! read/write direction bit. The actual OTP array access is managed
//! by the hardware with specific timing requirements.

use crate::peripherals::Efuse;

/// eFuse controller driver.
pub struct EfuseDriver<'d> {
    _efuse: Efuse<'d>,
}

impl<'d> EfuseDriver<'d> {
    /// Create a new eFuse driver.
    pub fn new(efuse: Efuse<'d>) -> Self {
        Self { _efuse: efuse }
    }

    fn regs(&self) -> &'static ws63_pac::efuse::RegisterBlock {
        unsafe { &*Efuse::ptr() }
    }

    /// Set the eFuse clock period.
    ///
    /// Controls the timing of eFuse read/write operations.
    pub fn set_clock_period(&mut self, period: u8) {
        unsafe {
            self.regs()
                .efuse_clk_period()
                .write(|w| w.bits(period as u32));
        }
    }

    /// Set the control data word (lower 16 bits).
    ///
    /// The control data selects which eFuse word is being accessed.
    pub fn set_control(&mut self, ctl: u16) {
        let current = self.regs().efuse_ctl_data().read().bits();
        // Preserve wr_rd bit, update control field
        let wr_rd = current & 0x10000;
        unsafe {
            self.regs()
                .efuse_ctl_data()
                .write(|w| w.bits(wr_rd | (ctl as u32)));
        }
    }

    /// Set the read/write direction: `true` = write, `false` = read.
    pub fn set_write_mode(&mut self, write: bool) {
        let current = self.regs().efuse_ctl_data().read().bits();
        let ctl = current & 0xFFFF;
        unsafe {
            self.regs()
                .efuse_ctl_data()
                .write(|w| w.bits(ctl | (if write { 0x10000 } else { 0 })));
        }
    }

    /// Read the current control data register value.
    pub fn read_control_data(&self) -> u32 {
        self.regs().efuse_ctl_data().read().bits()
    }

    /// Control the AVDD power switch for eFuse programming.
    ///
    /// * `enable` — `true` to enable AVDD for programming, `false` to disable.
    pub fn set_avdd(&mut self, enable: bool) {
        unsafe {
            self.regs()
                .efuse_avdd_ctl()
                .write(|w| w.bits(if enable { 1 } else { 0 }));
        }
    }

    /// Check the eFuse status.
    ///
    /// Returns a tuple of:
    /// - Manufacturing status (bits 0:1)
    /// - Boot0 done
    /// - Boot1 done
    /// - Boot2 done
    pub fn status(&self) -> EfuseStatus {
        let sts = self.regs().efuse_sts().read().bits();
        EfuseStatus {
            man_status: (sts & 0x03) as u8,
            boot0_done: (sts & 0x04) != 0,
            boot1_done: (sts & 0x08) != 0,
            boot2_done: (sts & 0x10) != 0,
        }
    }

    /// Read the chip ID from eFuse.
    ///
    /// The chip ID is stored in the eFuse control data register
    /// and contains manufacturing information.
    pub fn read_chip_id(&self) -> u32 {
        self.regs().efuse_ctl_data().read().bits()
    }

    /// Read a raw eFuse word at the given control data value.
    ///
    /// * `ctl` — Control data value specifying which eFuse word to read.
    pub fn read_raw(&mut self, ctl: u16) -> u32 {
        self.set_control(ctl);
        self.set_write_mode(false);
        self.regs().efuse_ctl_data().read().bits()
    }
}

/// eFuse status information.
#[derive(Debug, Clone, Copy)]
pub struct EfuseStatus {
    /// Manufacturing status (2-bit field).
    pub man_status: u8,
    /// Boot stage 0 completed.
    pub boot0_done: bool,
    /// Boot stage 1 completed.
    pub boot1_done: bool,
    /// Boot stage 2 completed.
    pub boot2_done: bool,
}

impl EfuseStatus {
    /// Returns true if all boot stages completed successfully.
    pub fn boot_complete(&self) -> bool {
        self.boot0_done && self.boot1_done && self.boot2_done
    }
}
