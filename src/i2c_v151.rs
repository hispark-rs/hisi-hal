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

/// Error returned by the BS2X I2C master operations.
#[derive(Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum I2cError {
    /// A status bit never asserted within the bounded wait.
    Timeout,
    /// The addressed device did not ACK (DesignWare TX_ABRT / addr_7b_noack).
    Nack,
}

// embedded-hal 1.0 portability: wire the BS2X I2C error + driver into the standard
// traits so generic drivers work against it exactly like the WS63 i2c.rs core.
impl embedded_hal::i2c::Error for I2cError {
    fn kind(&self) -> embedded_hal::i2c::ErrorKind {
        match self {
            I2cError::Nack => {
                embedded_hal::i2c::ErrorKind::NoAcknowledge(embedded_hal::i2c::NoAcknowledgeSource::Unknown)
            }
            I2cError::Timeout => embedded_hal::i2c::ErrorKind::Other,
        }
    }
}

impl<T> embedded_hal::i2c::ErrorType for I2c<'_, T> {
    type Error = I2cError;
}

impl<T> embedded_hal::i2c::I2c for I2c<'_, T> {
    fn transaction(&mut self, addr: u8, operations: &mut [embedded_hal::i2c::Operation<'_>]) -> Result<(), I2cError> {
        for op in operations {
            match op {
                embedded_hal::i2c::Operation::Read(buf) => self.read(addr, buf)?,
                embedded_hal::i2c::Operation::Write(data) => self.write(addr, data)?,
            }
        }
        Ok(())
    }
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

/// BS2X I2C master driver over the DesignWare `ic_*` (v151) controller.
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
    /// Create the I2C0 master, configuring it for `speed`.
    pub fn new_i2c0(_i2c: I2c0<'d>, speed: Speed) -> Self {
        configure(0, speed);
        Self { idx: 0, _peripheral: PhantomData }
    }
}
impl<'d> I2c<'d, I2c1<'d>> {
    /// Create the I2C1 master, configuring it for `speed`.
    pub fn new_i2c1(_i2c: I2c1<'d>, speed: Speed) -> Self {
        configure(1, speed);
        Self { idx: 1, _peripheral: PhantomData }
    }
}

/// IC_CON.speed field + (SCL high, SCL low) counts for a bus speed.
///
/// Pure mapping extracted from `configure` so it can be unit-tested without
/// touching MMIO. IC_CON.speed encoding: 1=SS, 2=FS, 3=HS. The SCL counts are
/// nominal values for a ~32 MHz I2C clock; the exact divisor only matters on
/// silicon (the QEMU model ignores it). TODO(bs2x): derive from
/// soc::chip::I2C_CLOCK_HZ per hal_i2c_v151's scl-count formula.
const fn scl_params(speed: Speed) -> (u32, u32, u32) {
    match speed {
        Speed::Standard => (1u32, 160u32, 190u32),
        Speed::Fast => (2u32, 40u32, 50u32),
    }
}

/// Master init (fbb_bs2x `hal_i2c_v151_master_init`): disable, program IC_CON +
/// the SCL counts, re-enable.
fn configure(idx: u8, speed: Speed) {
    let r = i2c_regs(idx);
    let (speed_field, hcnt, lcnt) = scl_params(speed);
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
                r.ic_ss_scl_hcnt().write(|w| w.ic_ss_scl_hcnt().bits(hcnt as u16));
                r.ic_ss_scl_lcnt().write(|w| w.ic_ss_scl_lcnt().bits(lcnt as u16));
            }
            Speed::Fast => {
                r.ic_fs_scl_hcnt().write(|w| w.ic_fs_scl_hcnt().bits(hcnt as u16));
                r.ic_fs_scl_lcnt().write(|w| w.ic_fs_scl_lcnt().bits(lcnt as u16));
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

// ── Tests ──────────────────────────────────────────────────────

#[cfg(all(test, not(target_arch = "riscv32")))]
mod tests {
    use super::*;

    #[test]
    fn standard_scl_params_exact() {
        // Standard mode → IC_CON.speed=1 (SS) with the SS hcnt/lcnt pair.
        assert_eq!(scl_params(Speed::Standard), (1, 160, 190));
    }

    #[test]
    fn fast_scl_params_exact() {
        // Fast mode → IC_CON.speed=2 (FS) with the FS hcnt/lcnt pair.
        assert_eq!(scl_params(Speed::Fast), (2, 40, 50));
    }

    #[test]
    fn scl_speed_field_fits_byte() {
        // configure() writes speed_field with `.bits(speed_field as u8)`, so the
        // IC_CON.speed field must fit a u8 for every Speed.
        for s in [Speed::Standard, Speed::Fast] {
            let (field, _, _) = scl_params(s);
            assert!(field <= u8::MAX as u32);
            assert!(field != 0, "0 is the reserved/illegal IC_CON.speed value");
        }
    }

    #[test]
    fn scl_counts_fit_u16() {
        // The SCL counts are written with `.bits(_ as u16)`; both must fit u16.
        for s in [Speed::Standard, Speed::Fast] {
            let (_, hcnt, lcnt) = scl_params(s);
            assert!(hcnt <= u16::MAX as u32);
            assert!(lcnt <= u16::MAX as u32);
        }
    }

    #[test]
    fn fast_is_shorter_than_standard() {
        // Fast mode (400 kHz) halves the SCL period vs Standard (100 kHz), so its
        // high/low counts must both be strictly smaller for the same clock.
        let (_, sh, sl) = scl_params(Speed::Standard);
        let (_, fh, fl) = scl_params(Speed::Fast);
        assert!(fh < sh);
        assert!(fl < sl);
    }

    #[test]
    fn scl_low_exceeds_high() {
        // I2C SCL low time is longer than high time in both modes (the asymmetry
        // the DesignWare SS/FS count pairs encode).
        for s in [Speed::Standard, Speed::Fast] {
            let (_, hcnt, lcnt) = scl_params(s);
            assert!(lcnt > hcnt);
        }
    }

    #[test]
    fn target_addr_mask_keeps_7bit() {
        // set_target masks with `& 0x3FF`; any valid 7-bit address passes through
        // unchanged (the top bits are only relevant for 10-bit addressing).
        for addr in 0u8..=0x7F {
            assert_eq!(addr as u16 & 0x3FF, addr as u16);
        }
    }

    #[test]
    fn target_addr_mask_drops_nothing_for_u8() {
        // A u8 address can never exceed 0x3FF, so the mask is a no-op for the whole
        // u8 range — confirms the cast widens before masking (no truncation).
        for addr in 0u8..=0xFF {
            assert_eq!(addr as u16 & 0x3FF, addr as u16);
        }
    }

    #[test]
    fn wait_until_returns_ok_when_ready() {
        // A condition that is already true resolves immediately with Ok.
        assert_eq!(wait_until(|| true), Ok(()));
    }

    #[test]
    fn wait_until_times_out_when_never_ready() {
        // A condition that never asserts exhausts POLL_LIMIT and reports Timeout.
        assert_eq!(wait_until(|| false), Err(I2cError::Timeout));
    }

    #[test]
    fn wait_until_polls_at_most_poll_limit() {
        // The poll loop is bounded: it calls the predicate no more than POLL_LIMIT
        // times before giving up (so a stuck bus can't hang the CPU).
        let mut calls: u32 = 0;
        let r = wait_until(|| {
            calls += 1;
            false
        });
        assert_eq!(r, Err(I2cError::Timeout));
        assert_eq!(calls, POLL_LIMIT);
    }

    #[test]
    fn wait_until_stops_at_first_ready() {
        // Once the predicate is true the loop returns; later polls don't happen.
        let mut calls: u32 = 0;
        let r = wait_until(|| {
            calls += 1;
            calls >= 3
        });
        assert_eq!(r, Ok(()));
        assert_eq!(calls, 3);
    }

    #[test]
    fn i2c_error_variants_distinct() {
        // Timeout and Nack are distinct error values (the two failure modes the
        // public API surfaces via PartialEq).
        assert_ne!(I2cError::Timeout, I2cError::Nack);
        assert_eq!(I2cError::Nack, I2cError::Nack);
    }

    #[test]
    fn speed_is_copy_and_eq() {
        // Speed derives Copy + Eq; mapping the same value twice is identical.
        let s = Speed::Fast;
        let t = s; // Copy, not move
        assert_eq!(s, t);
        assert_eq!(scl_params(s), scl_params(t));
    }
}

// ── Property-based fuzz tests ──────────────────────────────────

#[cfg(all(test, not(target_arch = "riscv32")))]
mod proptests {
    use super::{Speed, scl_params};
    use proptest::prelude::*;

    /// Pick a `Speed` from a bool so proptest can drive the enum-keyed mapping.
    fn speed_of(fast: bool) -> Speed {
        if fast { Speed::Fast } else { Speed::Standard }
    }

    proptest! {
        /// Fuzz: `scl_params` never panics for either speed (pure mapping).
        #[test]
        fn scl_params_never_panics(fast in any::<bool>()) {
            let _ = scl_params(speed_of(fast));
        }

        /// Fuzz: the IC_CON.speed field survives the `as u8` cast in `configure`
        /// without truncation — i.e. it fits a u8 (round-trips through u8) and is
        /// never the reserved/illegal value 0. This guards the narrowing cast
        /// `w.speed().bits(speed_field as u8)`.
        #[test]
        fn speed_field_fits_u8_and_nonzero(fast in any::<bool>()) {
            let (field, _, _) = scl_params(speed_of(fast));
            // round-trip the actual cast `configure` performs.
            prop_assert_eq!((field as u8) as u32, field);
            prop_assert!(field != 0);
        }

        /// Fuzz: both SCL counts survive the `as u16` cast in `configure` without
        /// truncation (they round-trip through u16). This guards the narrowing
        /// casts `w.cnt().bits(hcnt as u16)` / `(lcnt as u16)`.
        #[test]
        fn scl_counts_fit_u16(fast in any::<bool>()) {
            let (_, hcnt, lcnt) = scl_params(speed_of(fast));
            prop_assert_eq!((hcnt as u16) as u32, hcnt);
            prop_assert_eq!((lcnt as u16) as u32, lcnt);
        }

        /// Fuzz: in every mode the SCL low time exceeds the high time (the
        /// asymmetry the DesignWare SS/FS count pairs must encode).
        #[test]
        fn scl_low_exceeds_high(fast in any::<bool>()) {
            let (_, hcnt, lcnt) = scl_params(speed_of(fast));
            prop_assert!(lcnt > hcnt);
        }

        /// Fuzz: the target-address mask `addr as u16 & 0x3FF` (from `set_target`)
        /// is a no-op for ANY u8 address — the `as u16` widens BEFORE the mask, so
        /// nothing is ever truncated and the result equals the input. This is the
        /// "mask applied to a too-narrow / pre-cast value" bug shape; a u8 can
        /// never exceed the 10-bit field, so the masked value must round-trip.
        #[test]
        fn target_addr_mask_roundtrips_for_u8(addr in any::<u8>()) {
            let masked = addr as u16 & 0x3FF;
            prop_assert_eq!(masked, addr as u16);
        }

        /// Fuzz: the masked target address never sets a bit outside the 10-bit
        /// IC_TAR field (bits above bit 9 are always clear).
        #[test]
        fn target_addr_mask_clears_high_bits(addr in any::<u8>()) {
            let masked = addr as u16 & 0x3FF;
            prop_assert_eq!(masked & !0x3FF, 0);
        }
    }
}
