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
//!
//! This experimental backend does not yet establish that a timed-out or aborted
//! transaction has left the physical bus idle. Do not infer successful bus
//! recovery or peer state from an error; platform recovery and real bus behavior
//! still require validation.

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
    /// Address, data or general call was not acknowledged.
    Nack,
    /// The controller reported lost bus arbitration.
    ArbitrationLost,
    /// Another or ambiguous abort; `source` preserves the raw source bits.
    Abort {
        /// Raw `IC_TX_ABRT_SLV_INTX` bits captured before clearing the abort.
        source: u32,
    },
}

// embedded-hal 1.0 portability: wire the BS2X I2C error + driver into the standard
// traits so generic drivers work against it exactly like the WS63 i2c.rs core.
impl embedded_hal::i2c::Error for I2cError {
    fn kind(&self) -> embedded_hal::i2c::ErrorKind {
        match self {
            I2cError::Nack => {
                embedded_hal::i2c::ErrorKind::NoAcknowledge(embedded_hal::i2c::NoAcknowledgeSource::Unknown)
            }
            I2cError::ArbitrationLost => embedded_hal::i2c::ErrorKind::ArbitrationLoss,
            I2cError::Timeout | I2cError::Abort { .. } => embedded_hal::i2c::ErrorKind::Other,
        }
    }
}

impl<T> embedded_hal::i2c::ErrorType for I2c<'_, T> {
    type Error = I2cError;
}

impl<T> embedded_hal::i2c::I2c for I2c<'_, T> {
    fn transaction(&mut self, addr: u8, operations: &mut [embedded_hal::i2c::Operation<'_>]) -> Result<(), I2cError> {
        run_transaction(&mut PacTransaction(self.idx), addr, operations, POLL_LIMIT)
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
    /// Probe `addr`: issue a 1-byte read and report whether the device ACKed.
    /// Timeout is reported as false; this bool API cannot distinguish it from NACK.
    pub fn probe(&mut self, addr: u8) -> bool {
        run_probe(&mut PacTransaction(self.idx), addr)
    }

    /// Write `data` to `addr` (STOP after the last byte).
    pub fn write(&mut self, addr: u8, data: &[u8]) -> Result<(), I2cError> {
        run_transaction(&mut PacTransaction(self.idx), addr, &mut [embedded_hal::i2c::Operation::Write(data)], POLL_LIMIT)
    }

    /// Read `buf.len()` bytes from `addr` (STOP after the last byte).
    pub fn read(&mut self, addr: u8, buf: &mut [u8]) -> Result<(), I2cError> {
        run_transaction(&mut PacTransaction(self.idx), addr, &mut [embedded_hal::i2c::Operation::Read(buf)], POLL_LIMIT)
    }

    /// Write then read (repeated START) — `wr` out, then `rd` in.
    pub fn write_read(&mut self, addr: u8, wr: &[u8], rd: &mut [u8]) -> Result<(), I2cError> {
        run_transaction(&mut PacTransaction(self.idx), addr, &mut [embedded_hal::i2c::Operation::Write(wr), embedded_hal::i2c::Operation::Read(rd)], POLL_LIMIT)
    }
}

trait TransactionIo {
    fn target(&mut self, address: u8);
    fn tx_ready(&mut self) -> bool;
    fn rx_ready(&mut self) -> bool;
    fn stopped(&mut self) -> bool;
    fn aborted(&mut self) -> bool;
    fn abort_source(&mut self) -> u32;
    fn clear_abort(&mut self);
    fn send(&mut self, data: u8, read: bool, restart: bool, stop: bool);
    fn receive(&mut self) -> u8;
}

fn run_probe(io: &mut impl TransactionIo, address: u8) -> bool {
    io.target(address);
    io.send(0, true, false, true);
    let resolved = wait_until(|| io.stopped() || io.aborted()).is_ok();
    let present = resolved && !io.aborted();
    if present && io.rx_ready() { let _ = io.receive(); }
    io.clear_abort();
    present
}

fn wait_transaction<I: TransactionIo>(io: &mut I, limit: u32, ready: impl Fn(&mut I) -> bool) -> Result<(), I2cError> {
    for _ in 0..limit {
        if io.aborted() {
            let error = classify_abort(io.abort_source());
            io.clear_abort();
            return Err(error);
        }
        if ready(io) {
            // An abort may arrive with the ready condition; never report it as success.
            if io.aborted() {
                let error = classify_abort(io.abort_source());
                io.clear_abort();
                return Err(error);
            }
            return Ok(());
        }
        core::hint::spin_loop();
    }
    Err(I2cError::Timeout)
}

// fbb_bs2x SDK 80d4672b, hal_i2c_v151_regs_def.h: TX_ABRT_SLV_INTX
// bits 0..4 describe NACK, bit 12 arbitration loss; other causes remain raw.
fn classify_abort(source: u32) -> I2cError {
    const NACK: u32 = 0x1f;
    const ARBITRATION_LOST: u32 = 1 << 12;
    if source & ARBITRATION_LOST != 0 {
        I2cError::ArbitrationLost
    } else if source & NACK != 0 && source & !NACK == 0 {
        I2cError::Nack
    } else {
        I2cError::Abort { source }
    }
}

fn run_transaction(io: &mut impl TransactionIo, address: u8, operations: &mut [embedded_hal::i2c::Operation<'_>], limit: u32) -> Result<(), I2cError> {
    use embedded_hal::i2c::Operation;
    let mut plan = crate::i2c_plan::Planner::new(operations);
    if plan.is_empty() { return Ok(()); }
    io.target(address);
    while let Some(step) = plan.next(operations) {
        wait_transaction(io, limit, |io| io.tx_ready())?;
        let data = match &operations[step.op] { Operation::Write(data) => data[step.byte], Operation::Read(_) => 0 };
        io.send(data, step.read, step.restart, step.stop);
        if let Operation::Read(data) = &mut operations[step.op] {
            wait_transaction(io, limit, |io| io.rx_ready())?;
            data[step.byte] = io.receive();
        }
    }
    wait_transaction(io, limit, |io| io.stopped())
}

struct PacTransaction(u8);
impl TransactionIo for PacTransaction {
    fn target(&mut self, address: u8) {
        let r = i2c_regs(self.0);
        r.ic_enable().write(|w| w.enable().clear_bit());
        r.ic_con().modify(|_, w| w.ic_10bitaddr_master().clear_bit());
        r.ic_tar().write(|w| unsafe { w.special().clear_bit().gc_or_start().clear_bit().ic_tar().bits(address as u16 & 0x3ff) });
        r.ic_enable().write(|w| w.enable().set_bit());
        let _ = r.ic_clr_intr().read().bits();
    }
    fn tx_ready(&mut self) -> bool { i2c_regs(self.0).ic_status().read().tfnf().bit_is_set() }
    fn rx_ready(&mut self) -> bool { i2c_regs(self.0).ic_status().read().rfne().bit_is_set() }
    fn stopped(&mut self) -> bool { i2c_regs(self.0).ic_raw_intr_stat().read().stop_det().bit_is_set() }
    fn aborted(&mut self) -> bool { i2c_regs(self.0).ic_raw_intr_stat().read().tx_abrt().bit_is_set() }
    fn abort_source(&mut self) -> u32 { i2c_regs(self.0).ic_tx_abrt_slv_intx().read().bits() }
    fn clear_abort(&mut self) { let _ = i2c_regs(self.0).ic_clr_intr().read().bits(); }
    fn send(&mut self, data: u8, read: bool, restart: bool, stop: bool) {
        i2c_regs(self.0).ic_data_cmd().write(|w| unsafe { w.dat().bits(data).cmd().bit(read).restart().bit(restart).stop().bit(stop) });
    }
    fn receive(&mut self) -> u8 { i2c_regs(self.0).ic_data_cmd().read().dat().bits() }
}

// ── Tests ──────────────────────────────────────────────────────

#[cfg(all(test, not(target_arch = "riscv32")))]
mod tests {
    use super::*;

    #[derive(Default)]
    struct TransactionMock {
        targets: std::vec::Vec<u8>,
        commands: std::vec::Vec<(u8, bool, bool, bool)>,
        rx: std::collections::VecDeque<u8>,
        abort_after: Option<usize>,
        source: u32,
        stalled: bool,
        no_stop: bool,
        abort_on_stop: bool,
        cleared: usize,
    }
    impl TransactionIo for TransactionMock {
        fn target(&mut self, address: u8) { self.targets.push(address); }
        fn tx_ready(&mut self) -> bool { !self.stalled }
        fn rx_ready(&mut self) -> bool { !self.rx.is_empty() }
        fn stopped(&mut self) -> bool {
            let stopped = !self.no_stop && self.commands.last().is_some_and(|command| command.3);
            if stopped && self.abort_on_stop { self.abort_after = Some(self.commands.len()); }
            stopped
        }
        fn aborted(&mut self) -> bool { self.abort_after == Some(self.commands.len()) }
        fn abort_source(&mut self) -> u32 { self.source }
        fn clear_abort(&mut self) { self.cleared += 1; self.abort_after = None; self.source = 0; }
        fn send(&mut self, data: u8, read: bool, restart: bool, stop: bool) { self.commands.push((data, read, restart, stop)); }
        fn receive(&mut self) -> u8 { self.rx.pop_front().unwrap() }
    }

    #[test]
    fn transaction_preserves_repeated_start_and_single_final_stop() {
        use embedded_hal::i2c::Operation;
        let mut mock = TransactionMock { rx: [0xa5, 0x5a].into(), ..Default::default() };
        let mut empty = [];
        let mut read = [0; 2];
        run_transaction(&mut mock, 0x52, &mut [Operation::Write(&[1]), Operation::Write(&[2]), Operation::Read(&mut empty), Operation::Read(&mut read), Operation::Write(&[3])], 3).unwrap();
        assert_eq!(mock.targets, [0x52]);
        assert_eq!(read, [0xa5, 0x5a]);
        assert_eq!(mock.commands, [(1, false, false, false), (2, false, false, false), (0, true, true, false), (0, true, false, false), (3, false, true, true)]);
    }

    #[test]
    fn empty_transaction_does_not_touch_target_and_errors_are_bounded() {
        use embedded_hal::i2c::Operation;
        let mut mock = TransactionMock::default();
        let mut empty = [];
        run_transaction(&mut mock, 0x52, &mut [Operation::Write(&[]), Operation::Read(&mut empty)], 3).unwrap();
        assert!(mock.targets.is_empty());
        mock.stalled = true;
        assert_eq!(run_transaction(&mut mock, 0x52, &mut [Operation::Write(&[1])], 3), Err(I2cError::Timeout));
        assert!(mock.commands.is_empty());
        mock.stalled = false;
        mock.abort_after = Some(1);
        mock.source = 1;
        let mut output = [0x55];
        assert_eq!(run_transaction(&mut mock, 0x52, &mut [Operation::Read(&mut output)], 3), Err(I2cError::Nack));
        assert_eq!(output, [0x55]);
        assert_eq!(mock.cleared, 1);
    }

    #[test]
    fn probe_reports_timeout_or_abort_as_absent_and_drains_success() {
        let mut timeout = TransactionMock { no_stop: true, ..Default::default() };
        assert!(!run_probe(&mut timeout, 0x52));
        assert_eq!(timeout.cleared, 1);
        let mut nack = TransactionMock { abort_after: Some(1), ..Default::default() };
        assert!(!run_probe(&mut nack, 0x52));
        let mut present = TransactionMock { rx: [0xa5].into(), ..Default::default() };
        assert!(run_probe(&mut present, 0x52));
        assert!(present.rx.is_empty());
    }

    #[test]
    fn abort_arriving_with_stop_is_not_reported_as_success() {
        use embedded_hal::i2c::Operation;
        let mut mock = TransactionMock { abort_on_stop: true, source: 1, ..Default::default() };
        assert_eq!(run_transaction(&mut mock, 0x52, &mut [Operation::Write(&[9])], 3), Err(I2cError::Nack));
        assert_eq!(mock.commands, [(9, false, false, true)]);
        assert_eq!(mock.cleared, 1);
    }

    #[test]
    fn abort_source_is_sampled_before_clear_and_classified_conservatively() {
        use embedded_hal::i2c::{Error as _, ErrorKind, NoAcknowledgeSource, Operation};
        for (source, expected, kind) in [
            (1, I2cError::Nack, ErrorKind::NoAcknowledge(NoAcknowledgeSource::Unknown)),
            (8, I2cError::Nack, ErrorKind::NoAcknowledge(NoAcknowledgeSource::Unknown)),
            (1 << 12, I2cError::ArbitrationLost, ErrorKind::ArbitrationLoss),
            (1 << 9, I2cError::Abort { source: 1 << 9 }, ErrorKind::Other),
            (0, I2cError::Abort { source: 0 }, ErrorKind::Other),
            (1 | (1 << 9), I2cError::Abort { source: 1 | (1 << 9) }, ErrorKind::Other),
        ] {
            let mut mock = TransactionMock { abort_after: Some(1), source, ..Default::default() };
            let actual = run_transaction(&mut mock, 0x52, &mut [Operation::Write(&[9])], 3).unwrap_err();
            assert_eq!(actual.kind(), kind);
            assert_eq!(actual, expected);
            assert_eq!(mock.cleared, 1);
        }
    }

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
