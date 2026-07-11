//! eFuse (OTP) driver for WS63 — v151 controller.
//!
//! Register map and access sequence are cross-checked against the WS63 C SDK
//! (`hal_efuse_v151.c`, `hal_efuse_v151_reg_op.h`, `efuse_porting.c`):
//!
//! - **Status** `EFUSE_STS` at base+0x2C (boot-done flags, read-only).
//! - **Control block** at base+0x30: `EFUSE_CTL_DATA` (mode select), +0x34
//!   `EFUSE_CLK_PERIOD`, +0x3C `EFUSE_AVDD_CTL` (program-voltage switch).
//! - **Data window** at base+0x800: 128 × 32-bit words, each packing two eFuse
//!   bytes (even byte address in `[7:0]`, odd in `[15:8]`). Word index =
//!   `byte_addr / 2`.
//!
//! An access is *armed* by writing a 16-bit magic to `EFUSE_CTL_DATA`:
//! `0x5A5A` = read mode, `0xA5A5` = program mode (`HAL_EFUSE_READ_MODE` /
//! `HAL_EFUSE_WRITE_MODE`). A read then loads the latched word from the window;
//! a program raises AVDD, writes the byte, and lowers AVDD with timing delays.
//!
//! # Safety / status
//!
//! This driver has **not been validated on silicon**. Programming an eFuse is a
//! one-time, irreversible operation; [`EfuseDriver::write_byte`] is provided for
//! completeness but should be treated as experimental.

use crate::peripherals::Efuse;

/// Magic written to `EFUSE_CTL_DATA` to arm a read (`HAL_EFUSE_READ_MODE`).
const EFUSE_READ_MAGIC: u32 = 0x5A5A;
/// Magic written to `EFUSE_CTL_DATA` to arm a program (`HAL_EFUSE_WRITE_MODE`).
const EFUSE_WRITE_MAGIC: u32 = 0xA5A5;
/// eFuse array size for one region: 2048 bits = 256 bytes
/// (`EFUSE_REGION_MAX_BITS` in the SDK).
pub const EFUSE_MAX_BYTES: u16 = 256;
/// Settle delay around a program pulse (`HAL_EFUSE_DELAY_US`).
const EFUSE_PROGRAM_DELAY_US: u32 = 100;

/// Word index into the data window for a given eFuse byte address.
///
/// Each 32-bit window word holds two consecutive eFuse bytes, so the word
/// index is `byte_addr / 2` (the SDK computes `(offset >> 1) << 2` as a byte
/// offset, i.e. `word_index * 4`).
#[inline]
const fn word_index(byte_addr: u16) -> usize {
    (byte_addr / 2) as usize
}

/// Extract one eFuse byte from a window word: even address → low byte
/// (`[7:0]`), odd address → high byte (`[15:8]`).
#[inline]
const fn extract_byte(word: u32, byte_addr: u16) -> u8 {
    if byte_addr & 1 != 0 { ((word >> 8) & 0xFF) as u8 } else { (word & 0xFF) as u8 }
}

/// Pack one eFuse byte into the window word for programming: even address →
/// low byte, odd address → high byte (matches `hal_efuse_write_operation`).
#[inline]
const fn pack_byte(value: u8, byte_addr: u16) -> u32 {
    if byte_addr & 1 != 0 { (value as u32) << 8 } else { value as u32 }
}

/// eFuse access error.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum EfuseError {
    /// Byte address is outside the eFuse array (`>= EFUSE_MAX_BYTES`).
    OutOfRange,
}

/// A validated byte address inside the eFuse array.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub struct EfuseByteAddress(u16);

impl EfuseByteAddress {
    /// Build an eFuse byte address, rejecting values outside the 256-byte array.
    pub const fn from_byte(byte_addr: u16) -> Option<Self> {
        if byte_addr < EFUSE_MAX_BYTES { Some(Self(byte_addr)) } else { None }
    }

    /// The raw byte address inside the eFuse array.
    pub const fn byte(self) -> u16 {
        self.0
    }
}

/// eFuse controller driver.
pub struct EfuseDriver<'d> {
    _efuse: Efuse<'d>,
}

impl<'d> EfuseDriver<'d> {
    /// Create a new eFuse driver.
    pub fn new(efuse: Efuse<'d>) -> Self {
        let mut driver = Self { _efuse: efuse };
        driver.set_clock_period_raw(default_clock_period());
        driver
    }

    #[cfg(feature = "unstable")]
    pub(crate) fn into_inner(self) -> Efuse<'d> {
        self._efuse
    }

    fn regs() -> &'static crate::soc::pac::efuse::RegisterBlock {
        // SAFETY: PAC peripheral pointer is a static physical MMIO address, always valid.
        unsafe { &*Efuse::ptr() }
    }

    fn set_clock_period_raw(&mut self, period: u8) {
        unsafe {
            Self::regs().efuse_clk_period().write(|w| w.bits(period as u32));
        }
    }

    /// Override the eFuse clock period (cycles). The SDK uses `0x29` @ 24 MHz TCXO
    /// and `0x19` @ 40 MHz; [`new`](Self::new) programs the detected board default.
    #[instability::unstable]
    pub fn set_clock_period(&mut self, period: u8) {
        self.set_clock_period_raw(period);
    }

    /// Read the boot-done status register.
    #[instability::unstable]
    pub fn status(&self) -> EfuseStatus {
        let sts = Self::regs().efuse_sts().read();
        EfuseStatus {
            man_status: sts.man_sts().bits(),
            boot0_done: sts.boot0_done().bit_is_set(),
            boot1_done: sts.boot1_done().bit_is_set(),
            boot2_done: sts.boot2_done().bit_is_set(),
        }
    }

    /// Read a single eFuse byte at `byte_addr`.
    ///
    /// Arms read mode (`0x5A5A`), then loads the latched word from the data
    /// window and extracts the requested byte. No delay is required for reads
    /// (matches `hal_efuse_read_byte`).
    pub fn read_byte(&mut self, byte_addr: EfuseByteAddress) -> u8 {
        // SAFETY: `self` owns the unique eFuse token.
        unsafe { Self::read_byte_unchecked(byte_addr) }
    }

    /// Read one byte while ownership of the eFuse token is enforced externally.
    ///
    /// This supports C-ABI adapters whose callback shape cannot carry a Rust
    /// driver reference. Prefer [`read_byte`](Self::read_byte) in normal code.
    /// The read-mode arm and data-window load are serialized in one short
    /// critical section; no polling or delay occurs while interrupts are off.
    ///
    /// # Safety
    ///
    /// The caller must own the unique [`Efuse`] peripheral token for the entire
    /// call and prevent any concurrent eFuse operation outside this function.
    #[instability::unstable]
    pub unsafe fn read_byte_unchecked(byte_addr: EfuseByteAddress) -> u8 {
        critical_section::with(|_| {
            let byte_addr = byte_addr.byte();
            // SAFETY: the caller owns the eFuse token. This is a complete write
            // to the modeled control register, followed by a PAC read.
            unsafe {
                Self::regs().efuse_ctl_data().write(|w| w.bits(EFUSE_READ_MAGIC));
            }
            let word = Self::regs().efuse_data(word_index(byte_addr)).read().bits();
            extract_byte(word, byte_addr)
        })
    }

    /// Read `buf.len()` consecutive eFuse bytes starting at `start_byte`.
    #[instability::unstable]
    pub fn read_buffer(&mut self, start_byte: EfuseByteAddress, buf: &mut [u8]) -> Result<(), EfuseError> {
        let start_byte = start_byte.byte();
        for (i, slot) in buf.iter_mut().enumerate() {
            if i > u16::MAX as usize {
                return Err(EfuseError::OutOfRange);
            }
            let addr =
                start_byte.checked_add(i as u16).and_then(EfuseByteAddress::from_byte).ok_or(EfuseError::OutOfRange)?;
            *slot = self.read_byte(addr);
        }
        Ok(())
    }

    /// Program a single eFuse byte (**one-time, irreversible**).
    ///
    /// Sequence (per `hal_efuse_write_operation`): arm write mode (`0xA5A5`),
    /// raise AVDD, settle, write the packed byte to the window, lower AVDD,
    /// settle. eFuse bits can only be burned 0→1; this does not erase.
    ///
    /// Not validated on silicon — gated behind `unstable`.
    #[instability::unstable]
    pub fn write_byte(&mut self, byte_addr: EfuseByteAddress, value: u8) {
        let byte_addr = byte_addr.byte();
        let delay = crate::delay::Delay::new();
        unsafe {
            Self::regs().efuse_ctl_data().write(|w| w.bits(EFUSE_WRITE_MAGIC));
            Self::regs().efuse_avdd_ctl().write(|w| w.bits(1));
        }
        delay.delay_micros(EFUSE_PROGRAM_DELAY_US);
        unsafe {
            Self::regs().efuse_data(word_index(byte_addr)).write(|w| w.bits(pack_byte(value, byte_addr)));
            Self::regs().efuse_avdd_ctl().write(|w| w.bits(0));
        }
        delay.delay_micros(EFUSE_PROGRAM_DELAY_US);
    }
}

fn default_clock_period() -> u8 {
    if crate::soc::chip::uart_boot_clock_hz() == crate::soc::chip::UART_BOOT_CLOCK_24M_HZ { 0x29 } else { 0x19 }
}

/// eFuse status information.
#[derive(Debug, Clone, Copy)]
#[instability::unstable]
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

#[allow(dead_code)]
impl EfuseStatus {
    /// Returns true if all boot stages completed successfully.
    pub fn boot_complete(&self) -> bool {
        self.boot0_done && self.boot1_done && self.boot2_done
    }
}

// ── Tests ──────────────────────────────────────────────────────

#[cfg(all(test, not(target_arch = "riscv32")))]
mod tests {
    use super::*;

    #[test]
    fn test_magic_values() {
        // Mode-select magics must match the SDK exactly.
        assert_eq!(EFUSE_READ_MAGIC, 0x5A5A);
        assert_eq!(EFUSE_WRITE_MAGIC, 0xA5A5);
    }

    #[test]
    fn test_word_index() {
        // Two bytes per 32-bit window word.
        assert_eq!(word_index(0), 0);
        assert_eq!(word_index(1), 0);
        assert_eq!(word_index(2), 1);
        assert_eq!(word_index(3), 1);
        assert_eq!(word_index(255), 127);
    }

    #[test]
    fn test_extract_byte_even_odd() {
        let word = 0xABCD;
        assert_eq!(extract_byte(word, 0), 0xCD); // even → low byte
        assert_eq!(extract_byte(word, 1), 0xAB); // odd  → high byte
        assert_eq!(extract_byte(word, 2), 0xCD); // even (word index differs)
    }

    #[test]
    fn test_pack_byte_even_odd() {
        assert_eq!(pack_byte(0xCD, 0), 0x00CD); // even → low byte
        assert_eq!(pack_byte(0xAB, 1), 0xAB00); // odd  → high byte
    }

    #[test]
    fn test_pack_extract_roundtrip() {
        // Packing then extracting the same byte address must recover the value.
        for addr in [0u16, 1, 42, 255] {
            for v in [0u8, 1, 0x5A, 0xFF] {
                assert_eq!(extract_byte(pack_byte(v, addr), addr), v);
            }
        }
    }

    #[test]
    fn byte_address_accepts_only_array_range() {
        assert_eq!(EfuseByteAddress::from_byte(0).unwrap().byte(), 0);
        assert_eq!(EfuseByteAddress::from_byte(EFUSE_MAX_BYTES - 1).unwrap().byte(), EFUSE_MAX_BYTES - 1);
        assert_eq!(EfuseByteAddress::from_byte(EFUSE_MAX_BYTES), None);
        assert_eq!(EfuseByteAddress::from_byte(u16::MAX), None);
    }

    #[test]
    fn test_efuse_boot_status_complete() {
        let sts = EfuseStatus { man_status: 0, boot0_done: true, boot1_done: true, boot2_done: true };
        assert!(sts.boot_complete());

        let partial = EfuseStatus { man_status: 0, boot0_done: true, boot1_done: false, boot2_done: true };
        assert!(!partial.boot_complete());
    }
}

// ── Property-based fuzz tests ──────────────────────────────────

#[cfg(all(test, not(target_arch = "riscv32")))]
mod proptests {
    use super::*;
    use proptest::prelude::*;

    proptest! {
        /// Fuzz: word index is always byte_addr/2 and within the 128-word window.
        #[test]
        fn word_index_in_range(addr in 0u16..EFUSE_MAX_BYTES) {
            let idx = word_index(addr);
            prop_assert_eq!(idx, (addr / 2) as usize);
            prop_assert!(idx < 128);
        }

        /// Fuzz: pack/extract is a faithful round-trip for any byte and address.
        #[test]
        fn pack_extract_roundtrip(addr in any::<u16>(), v in any::<u8>()) {
            prop_assert_eq!(extract_byte(pack_byte(v, addr), addr), v);
        }

        /// Fuzz: extract never reads beyond the addressed byte lane.
        #[test]
        fn extract_byte_lane(word in any::<u32>(), addr in any::<u16>()) {
            let b = extract_byte(word, addr);
            let expected = if addr & 1 != 0 { ((word >> 8) & 0xFF) as u8 } else { (word & 0xFF) as u8 };
            prop_assert_eq!(b, expected);
        }
    }
}
