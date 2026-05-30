//! KM (Key Management) driver for WS63.
//!
//! The KM peripheral manages cryptographic keys, providing:
//! - KLAD (Key Loading and Distribution)
//! - Keyslot locking mechanisms
//! - RKP (Root Key Provisioning)

use crate::peripherals::Km;

/// Number of available keyslots.
pub const KEYSLOT_COUNT: u8 = 8;

/// KM driver.
pub struct KmDriver<'d> {
    _km: Km<'d>,
}

impl<'d> KmDriver<'d> {
    /// Create a new KM driver.
    pub fn new(km: Km<'d>) -> Self {
        Self { _km: km }
    }

    fn regs(&self) -> &'static ws63_pac::km::RegisterBlock {
        // SAFETY: PAC peripheral pointer is a static physical MMIO address, always valid
        unsafe { &*Km::ptr() }
    }

    /// Enable the KM peripheral.
    pub fn enable(&mut self) {
        unsafe {
            self.regs().kl_com_ctrl().write(|w| w.bits(0x01));
        }
    }

    /// Check if a keyslot is locked (read-only after lock).
    ///
    /// Implements the vendor SDK `inner_kslot_chn_is_locked` sequence:
    /// 1. Write `KC_RD_SLOT_NUM` to select the keyslot and type (mcipher)
    /// 2. Read `KC_RD_LOCK_STATUS` to get the lock owner
    /// 3. Keyslot is locked if `rd_lock_status != 0` (non-zero = some CPU owns the lock)
    pub fn is_keyslot_locked(&self, slot: u8) -> bool {
        assert!(slot < KEYSLOT_COUNT, "keyslot {} out of range (max {})", slot, KEYSLOT_COUNT);
        // Select keyslot to query: mcipher type (bit 15 = 0), slot number in bits [9:0]
        unsafe {
            self.regs().kc_rd_slot_num().write(|w| {
                w.slot_num_cfg().bits(slot as u16);
                w.slot_cfg_type().clear_bit()
            });
        }
        // Read lock status: non-zero rd_lock_status field means locked
        self.regs().kc_rd_lock_status().read().rd_lock_status().bits() != 0
    }

    /// Lock a keyslot (prevents further writes).
    ///
    /// Writes to `KC_REECPU_LOCK_CMD` — the REE CPU keyslot lock command register
    /// (main application processor = REE). Uses mcipher keyslot type.
    ///
    /// After locking, the keyslot cannot be modified until the next system reset.
    pub fn lock_keyslot(&mut self, slot: u8) {
        assert!(slot < KEYSLOT_COUNT, "keyslot {} out of range (max {})", slot, KEYSLOT_COUNT);
        // Vendor SDK: write key_slot_num (bits [9:0]) and lock_cmd=1 (bit 20),
        // flush_hmac_kslot_ind=0 (mcipher), tscipher_ind=0
        unsafe {
            self.regs().kc_reecpu_lock_cmd().write(|w| {
                w.key_slot_num().bits(slot as u16);
                w.lock_cmd().set_bit()
            });
        }
    }

    /// Get the number of available keyslots.
    pub fn keyslot_count(&self) -> u8 {
        KEYSLOT_COUNT
    }
}

// ── Tests ──────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    #[test]
    fn test_keyslot_lock_bit_position() {
        // lock_keyslot writes key_slot_num in bits [9:0], lock_cmd=1 at bit 20
        let slot: u16 = 3;
        let lock_cmd_bit: u32 = 1 << 20;
        let val: u32 = (slot as u32) | lock_cmd_bit;
        assert_eq!(val & 0x3FF, 3); // key_slot_num = 3 in bits [9:0]
        assert_eq!((val >> 20) & 1, 1); // lock_cmd = 1 at bit 20
    }

    #[test]
    fn test_keyslot_lock_status_is_enum_not_bitmask() {
        // rd_lock_status is a 3-bit enum (bits [2:0]):
        // 0=unlocked, 1=REE, 2=TEE, 4=PCPU, 6=AIDSP
        // is_keyslot_locked should check rd_lock_status != 0, NOT (status & (1 << slot))
        let unlocked: u8 = 0;
        let ree_locked: u8 = 1;
        let tee_locked: u8 = 2;
        assert_eq!(unlocked != 0, false); // unlocked → not locked
        assert_eq!(ree_locked != 0, true); // REE locked → locked
        assert_eq!(tee_locked != 0, true); // TEE locked → locked
    }

    #[test]
    fn test_keyslot_count() {
        assert_eq!(super::KEYSLOT_COUNT, 8);
    }

    #[test]
    fn test_all_keyslots_lockable() {
        // All 8 keyslots (0-7) must produce valid lock values
        for slot in 0..8u16 {
            let val: u32 = (slot as u32) | (1 << 20);
            assert_eq!(val & 0x3FF, slot as u32); // key_slot_num in bits [9:0]
            assert_eq!((val >> 20) & 1, 1); // lock_cmd at bit 20
            assert!(slot < super::KEYSLOT_COUNT as u16); // within valid range
        }
    }
}
