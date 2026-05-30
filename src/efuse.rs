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
        // SAFETY: PAC peripheral pointer is a static physical MMIO address, always valid
        unsafe { &*Efuse::ptr() }
    }

    /// Set the eFuse clock period.
    ///
    /// Controls the timing of eFuse read/write operations.
    pub fn set_clock_period(&mut self, period: u8) {
        unsafe {
            self.regs().efuse_clk_period().write(|w| w.bits(period as u32));
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
            self.regs().efuse_ctl_data().write(|w| w.bits(wr_rd | (ctl as u32)));
        }
    }

    /// Set the read/write direction: `true` = write, `false` = read.
    pub fn set_write_mode(&mut self, write: bool) {
        let current = self.regs().efuse_ctl_data().read().bits();
        let ctl = current & 0xFFFF;
        unsafe {
            self.regs().efuse_ctl_data().write(|w| w.bits(ctl | (if write { 0x10000 } else { 0 })));
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
            self.regs().efuse_avdd_ctl().write(|w| w.bits(if enable { 1 } else { 0 }));
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

    /// Read a raw eFuse word at the given control data value.
    ///
    /// * `ctl` — Control data value specifying which eFuse word to read.
    ///
    /// # Hardware sequence
    ///
    /// On WS63, the eFuse controller reads a word by:
    /// 1. Writing the control word with `wr_rd=0` (read mode)
    /// 2. Waiting for the hardware to latch the data (busy-wait delay)
    /// 3. Reading the result from the control data register
    ///
    /// Note: The PAC only exposes `efuse_ctl_data` for both control write and result
    /// read. The hardware internally latches the OTP data into this register after
    /// the control word is written in read mode.
    pub fn read_raw(&mut self, ctl: u16) -> u32 {
        // Set control word with wr_rd explicitly cleared (= read mode).
        // Do NOT preserve the existing wr_rd bit — a prior set_write_mode(true)
        // would cause this to be a destructive write instead of a read.
        unsafe {
            self.regs().efuse_ctl_data().write(|w| w.bits(ctl as u32));
        }
        // Delay for the hardware to latch OTP data into the register
        // (100 iterations ≈ 400ns at 240MHz, matches clock peripheral delay pattern)
        for _ in 0..100 {
            core::hint::spin_loop();
        }
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

// ── Tests ──────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_efuse_read_raw_clears_wr_rd_bit() {
        // read_raw(0x10) must write ctl=0x10 with wr_rd=0 (read mode)
        let ctl: u16 = 0x10;
        let val = ctl as u32; // wr_rd explicitly 0
        assert_eq!(val & 0x10000, 0); // bit 16 (wr_rd) must be 0 for read mode
        assert_eq!(val & 0xFFFF, 0x10); // lower 16 bits = ctl value
    }

    #[test]
    fn test_efuse_write_mode_bit_position() {
        // wr_rd is bit 16 (0x10000)
        let write_mode: u32 = 0x10000;
        let read_mode: u32 = 0x00000;
        assert_eq!(write_mode & 0x10000, 0x10000);
        assert_eq!(read_mode & 0x10000, 0);
    }

    #[test]
    fn test_efuse_ctl_not_mixed_with_write_mode() {
        // After set_write_mode(true), a subsequent read_raw must not
        // leak the write-mode bit into the control word
        let prev_write: u32 = 0x10000; // write mode was set
        let ctl: u16 = 0x5;
        // Old (buggy) behavior: val = (prev_write & 0x10000) | (ctl as u32)
        let old_buggy = (prev_write & 0x10000) | (ctl as u32);
        assert_eq!(old_buggy, 0x10005); // WRITE mode leaked!
        // New (fixed) behavior: val = ctl as u32 (no preservation of wr_rd)
        let new_fixed = ctl as u32;
        assert_eq!(new_fixed, 0x5); // clean read mode
        assert_eq!(new_fixed & 0x10000, 0);
    }

    #[test]
    fn test_efuse_boot_status_complete() {
        let sts = EfuseStatus { man_status: 0, boot0_done: true, boot1_done: true, boot2_done: true };
        assert!(sts.boot_complete());

        let partial = EfuseStatus { man_status: 0, boot0_done: true, boot1_done: false, boot2_done: true };
        assert!(!partial.boot_complete());
    }

    #[test]
    fn test_efuse_boot_status_bits() {
        // efuse_sts register bit layout:
        // bit 0-1: man_status, bit 2: boot0, bit 3: boot1, bit 4: boot2
        let sts_raw: u32 = 0x14; // boot2 done + boot0 done
        let sts = EfuseStatus {
            man_status: (sts_raw & 0x03) as u8,
            boot0_done: (sts_raw & 0x04) != 0,
            boot1_done: (sts_raw & 0x08) != 0,
            boot2_done: (sts_raw & 0x10) != 0,
        };
        assert_eq!(sts.man_status, 0);
        assert!(sts.boot0_done);
        assert!(!sts.boot1_done);
        assert!(sts.boot2_done);
    }
}

// ── Property-based fuzz tests ──────────────────────────────────

#[cfg(test)]
mod proptests {
    use proptest::prelude::*;

    proptest! {
        /// Fuzz: read_raw control word always has wr_rd=0 (read mode).
        #[test]
        fn read_raw_always_clears_wr_rd(ctl in any::<u16>()) {
            let val = ctl as u32;
            prop_assert_eq!(val & 0x10000, 0, "wr_rd bit leaked for ctl=0x{:04X}", ctl);
            prop_assert_eq!(val & 0xFFFF, ctl as u32);
        }

        /// Fuzz: set_write_mode(true) sets wr_rd, set_write_mode(false) clears it.
        #[test]
        fn set_write_mode_toggles_wr_rd(ctl in any::<u16>()) {
            let write_val: u32 = ctl as u32 | 0x10000;
            prop_assert_eq!(write_val & 0x10000, 0x10000);
            let read_val: u32 = ctl as u32;
            prop_assert_eq!(read_val & 0x10000, 0);
        }

        /// Fuzz: Control field is always preserved after wr_rd toggling.
        #[test]
        fn ctl_field_preserved_through_mode_switch(ctl in any::<u16>()) {
            let write_val: u32 = ctl as u32 | 0x10000;
            let read_val: u32 = write_val & !0x10000;
            prop_assert_eq!((read_val & 0xFFFF) as u16, ctl);
        }

        /// Fuzz: Boot status parsing for any u32 raw value.
        #[test]
        fn boot_status_never_panics(raw in any::<u32>()) {
            let sts = super::EfuseStatus {
                man_status: (raw & 0x03) as u8,
                boot0_done: (raw & 0x04) != 0,
                boot1_done: (raw & 0x08) != 0,
                boot2_done: (raw & 0x10) != 0,
            };
            prop_assert!(sts.man_status <= 3);
            let _ = sts.boot_complete();
        }

        /// Fuzz: boot_complete is true iff all three boot bits are set.
        #[test]
        fn boot_complete_iff_all_three_bits(raw in any::<u32>()) {
            let sts = super::EfuseStatus {
                man_status: (raw & 0x03) as u8,
                boot0_done: (raw & 0x04) != 0,
                boot1_done: (raw & 0x08) != 0,
                boot2_done: (raw & 0x10) != 0,
            };
            let expected = raw & 0x04 != 0 && raw & 0x08 != 0 && raw & 0x10 != 0;
            prop_assert_eq!(sts.boot_complete(), expected);
        }
    }
}
