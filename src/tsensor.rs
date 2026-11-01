//! Temperature Sensor (TSENSOR) driver for WS63.
//!
//! The WS63 temperature sensor provides a 10-bit digital temperature code
//! covering approximately -40°C to +125°C. It supports automatic refresh,
//! high/low temperature thresholds, and over-temperature detection.
//!
//! # Temperature calculation
//!
//! Temperature code range: 114 to 896 (approximate).
//! Temperature (°C) can be derived from the code using chip-specific
//! calibration data. Without calibration, the raw code is provided.
//!
//! # Interrupts
//!
//! - Conversion done interrupt
//! - Out-of-threshold interrupt (temperature outside [low, high])
//! - Over-temperature interrupt

use crate::peripherals::Tsensor;

/// Temperature sensor driver.
pub struct TempSensor<'d> {
    _tsensor: Tsensor<'d>,
}

/// Temperature code range constants.
/// Minimum valid 10-bit temperature code (approximate low end of the range).
pub const TEMP_CODE_MIN: u16 = 114;
/// Maximum valid 10-bit temperature code (approximate high end of the range).
pub const TEMP_CODE_MAX: u16 = 896;

impl<'d> TempSensor<'d> {
    /// Create a new temperature sensor driver.
    pub fn new(tsensor: Tsensor<'d>) -> Self {
        Self { _tsensor: tsensor }
    }

    fn regs(&self) -> &'static crate::soc::pac::tsensor::RegisterBlock {
        // SAFETY: PAC peripheral pointer is a static physical MMIO address, always valid
        unsafe { &*Tsensor::ptr() }
    }

    /// Enable the temperature sensor.
    pub fn enable(&mut self) {
        let r = self.regs();
        let ctrl = r.tsensor_ctrl().read().bits();
        unsafe {
            r.tsensor_ctrl().write(|w| w.bits(ctrl | 0x01));
        }
    }

    /// Disable the temperature sensor.
    #[instability::unstable]
    pub fn disable(&mut self) {
        let r = self.regs();
        let ctrl = r.tsensor_ctrl().read().bits();
        unsafe {
            r.tsensor_ctrl().write(|w| w.bits(ctrl & !0x01));
        }
    }

    /// Set the operating mode.
    ///
    /// * `mode` — Mode value (0-3). The exact meaning depends on the chip
    ///   configuration.
    #[instability::unstable]
    pub fn set_mode(&mut self, mode: u8) {
        let r = self.regs();
        let ctrl = r.tsensor_ctrl().read().bits();
        let ctrl = (ctrl & !0x06) | (((mode as u32) & 0x03) << 1);
        unsafe {
            r.tsensor_ctrl().write(|w| w.bits(ctrl));
        }
    }

    /// Trigger a single temperature reading (software start).
    ///
    /// Writes 1 to the start register to refresh the temperature value.
    pub fn start_conversion(&mut self) {
        unsafe {
            self.regs().tsensor_start().write(|w| w.bits(0x01));
        }
    }

    /// Check if a temperature reading is ready.
    pub fn data_ready(&self) -> bool {
        self.regs().tsensor_sts().read().bits() & 0x02 != 0
    }

    /// Read the raw temperature code (10-bit, range 114-896).
    ///
    /// Returns `None` if data is not ready.
    pub fn read_raw(&self) -> Option<u16> {
        if !self.data_ready() {
            return None;
        }
        let sts = self.regs().tsensor_sts().read().bits();
        Some(((sts >> 2) & 0x3FF) as u16)
    }

    /// Read the temperature code, blocking until ready.
    #[instability::unstable]
    pub fn read_blocking(&self) -> u16 {
        while !self.data_ready() {}
        let sts = self.regs().tsensor_sts().read().bits();
        ((sts >> 2) & 0x3FF) as u16
    }

    /// Clear the temperature sensor status.
    #[instability::unstable]
    pub fn clear_status(&mut self) {
        let r = self.regs();
        let sts = r.tsensor_sts().read().bits();
        unsafe {
            r.tsensor_sts().write(|w| w.bits(sts | 0x01));
        }
    }

    /// Set the high temperature limit.
    ///
    /// Interrupt triggers when temperature code exceeds this value.
    #[instability::unstable]
    pub fn set_high_limit(&mut self, code: u16) {
        unsafe {
            self.regs().tsensor_temp_high_limit().write(|w| w.bits((code & 0x3FF) as u32));
        }
    }

    /// Set the low temperature limit.
    ///
    /// Interrupt triggers when temperature code falls below this value.
    #[instability::unstable]
    pub fn set_low_limit(&mut self, code: u16) {
        unsafe {
            self.regs().tsensor_temp_low_limit().write(|w| w.bits((code & 0x3FF) as u32));
        }
    }

    /// Set the over-temperature threshold.
    #[instability::unstable]
    pub fn set_over_temp_threshold(&mut self, code: u16, enable_interrupt: bool) {
        let mut val = (code & 0x3FF) as u32;
        if enable_interrupt {
            val |= 1 << 10; // over_temp_en
        }
        unsafe {
            self.regs().tsensor_over_temp().write(|w| w.bits(val));
        }
    }

    /// Enable specific temperature interrupts.
    ///
    /// * `done_int` — Conversion done interrupt
    /// * `out_thresh_int` — Out-of-threshold interrupt
    /// * `overtemp_int` — Over-temperature interrupt
    #[instability::unstable]
    pub fn enable_interrupts(&mut self, done_int: bool, out_thresh_int: bool, overtemp_int: bool) {
        let mut val: u32 = 0;
        if done_int {
            val |= 0x01;
        }
        if out_thresh_int {
            val |= 0x02;
        }
        if overtemp_int {
            val |= 0x04;
        }
        unsafe {
            self.regs().tsensor_temp_int_en().write(|w| w.bits(val));
        }
    }

    /// Disable all temperature interrupts.
    #[instability::unstable]
    pub fn disable_all_interrupts(&mut self) {
        unsafe {
            self.regs().tsensor_temp_int_en().write(|w| w.bits(0));
        }
    }

    /// Check interrupt status.
    ///
    /// Returns `(done, out_thresh, overtemp)`.
    #[instability::unstable]
    pub fn interrupt_status(&self) -> (bool, bool, bool) {
        let sts = self.regs().tsensor_temp_int_sts().read().bits();
        ((sts & 0x01) != 0, (sts & 0x02) != 0, (sts & 0x04) != 0)
    }

    /// Clear specific temperature interrupts.
    #[instability::unstable]
    pub fn clear_interrupts(&mut self, done: bool, out_thresh: bool, overtemp: bool) {
        let mut val: u32 = 0;
        if done {
            val |= 0x01;
        }
        if out_thresh {
            val |= 0x02;
        }
        if overtemp {
            val |= 0x04;
        }
        unsafe {
            self.regs().tsensor_temp_int_clr().write(|w| w.bits(val));
        }
    }

    /// Configure automatic temperature refresh.
    ///
    /// * `period` — Refresh period in 32kHz clock cycles.
    /// * `enable` — Enable auto refresh.
    #[instability::unstable]
    pub fn configure_auto_refresh(&mut self, period: u16, enable: bool) {
        unsafe {
            self.regs().tsensor_auto_refresh_period().write(|w| w.bits(period as u32));
            self.regs().tsensor_auto_refresh_cfg().write(|w| w.bits(if enable { 1 } else { 0 }));
        }
    }

    /// Enable temperature calibration.
    #[instability::unstable]
    pub fn enable_calibration(&mut self) {
        let ctrl1 = self.regs().tsensor_ctrl1().read().bits();
        unsafe {
            self.regs().tsensor_ctrl1().write(|w| w.bits(ctrl1 | 0x01));
        }
    }

    /// Disable temperature calibration.
    #[instability::unstable]
    pub fn disable_calibration(&mut self) {
        let ctrl1 = self.regs().tsensor_ctrl1().read().bits();
        unsafe {
            self.regs().tsensor_ctrl1().write(|w| w.bits(ctrl1 & !0x01));
        }
    }
}

// ── Tests ──────────────────────────────────────────────────────
//
// These exercise the pure bit-manipulation that the register-writing methods
// perform (code extraction, bitfield insertion, interrupt mask pack/unpack).
// The formulas are re-derived here rather than refactored out of the driver so
// the production code stays a thin MMIO layer; the masks/shifts mirror the ones
// used in `read_raw`, `set_mode`, `set_over_temp_threshold`, `enable_interrupts`,
// `interrupt_status`, etc. No MMIO is touched.

#[cfg(all(test, not(target_arch = "riscv32")))]
mod tests {
    use super::*;

    /// Extract the 10-bit temperature code from a status word, exactly as
    /// `read_raw`/`read_blocking` do: `((sts >> 2) & 0x3FF) as u16`.
    fn code_from_sts(sts: u32) -> u16 {
        ((sts >> 2) & 0x3FF) as u16
    }

    /// Pack the (done, out_thresh, overtemp) interrupt mask like
    /// `enable_interrupts`/`clear_interrupts`.
    fn pack_int(done: bool, out_thresh: bool, overtemp: bool) -> u32 {
        let mut val: u32 = 0;
        if done {
            val |= 0x01;
        }
        if out_thresh {
            val |= 0x02;
        }
        if overtemp {
            val |= 0x04;
        }
        val
    }

    /// Unpack the interrupt status word like `interrupt_status`.
    fn unpack_int(sts: u32) -> (bool, bool, bool) {
        ((sts & 0x01) != 0, (sts & 0x02) != 0, (sts & 0x04) != 0)
    }

    #[test]
    fn code_range_constants_within_10_bits() {
        // The documented code range must fit in the 10-bit field the driver reads.
        const {
            assert!(TEMP_CODE_MIN < TEMP_CODE_MAX);
        }
        const {
            assert!(TEMP_CODE_MAX <= 0x3FF);
        }
        assert_eq!(TEMP_CODE_MIN, 114);
        assert_eq!(TEMP_CODE_MAX, 896);
    }

    #[test]
    fn read_raw_extracts_bits_2_through_11() {
        // The code lives in bits [11:2] of the status word; lower 2 bits and any
        // bits above 11 must be ignored.
        assert_eq!(code_from_sts(0), 0);
        // 0x3FF placed at bit 2 → full-scale code, with junk in low/high bits.
        assert_eq!(code_from_sts((0x3FF << 2) | 0b11 | (0xF << 12)), 0x3FF);
        // A mid-range code 896 (TEMP_CODE_MAX) round-trips through the field.
        assert_eq!(code_from_sts((TEMP_CODE_MAX as u32) << 2), TEMP_CODE_MAX);
    }

    #[test]
    fn read_raw_never_exceeds_10_bit_field() {
        // Whatever garbage is in the status word, the extracted code is masked.
        assert_eq!(code_from_sts(u32::MAX), 0x3FF);
        assert!(code_from_sts(u32::MAX) <= TEMP_CODE_MAX.max(0x3FF));
    }

    #[test]
    fn set_mode_inserts_into_bits_2_1_only() {
        // set_mode: (ctrl & !0x06) | (((mode & 0x03) << 1)).
        let apply = |ctrl: u32, mode: u8| (ctrl & !0x06) | (((mode as u32) & 0x03) << 1);
        // Mode 0 clears the field, preserving the enable bit (bit 0).
        assert_eq!(apply(0x01, 0), 0x01);
        // Mode 3 sets both field bits, still preserving bit 0 and other bits.
        assert_eq!(apply(0x01, 3), 0x01 | 0x06);
        // Mode is masked to 2 bits: mode 0xFF behaves as mode 3.
        assert_eq!(apply(0, 0xFF), 0x06);
        // Pre-existing field bits are replaced, not OR'd in.
        assert_eq!(apply(0x06, 1), 0x02);
    }

    #[test]
    fn set_mode_preserves_unrelated_bits() {
        // Only bits 2:1 may change; every other bit is untouched.
        let apply = |ctrl: u32, mode: u8| (ctrl & !0x06) | (((mode as u32) & 0x03) << 1);
        let ctrl = 0xFFFF_FFFF;
        let out = apply(ctrl, 0);
        // Bits 2:1 cleared, everything else stays set.
        assert_eq!(out, !0x06);
    }

    #[test]
    fn over_temp_threshold_masks_code_and_sets_enable_bit() {
        // set_over_temp_threshold: (code & 0x3FF) | (en << 10).
        let apply = |code: u16, en: bool| {
            let mut val = (code & 0x3FF) as u32;
            if en {
                val |= 1 << 10;
            }
            val
        };
        // Disabled: just the masked code, bit 10 clear.
        assert_eq!(apply(896, false), 896);
        assert_eq!(apply(896, false) & (1 << 10), 0);
        // Enabled: code plus the over_temp_en bit (bit 10).
        assert_eq!(apply(896, true), 896 | (1 << 10));
        // Code above 10 bits is masked; the enable bit is unaffected by it.
        assert_eq!(apply(0xFFFF, true), 0x3FF | (1 << 10));
    }

    #[test]
    fn limit_setters_mask_to_10_bits() {
        // set_high_limit / set_low_limit both do (code & 0x3FF).
        let apply = |code: u16| (code & 0x3FF) as u32;
        assert_eq!(apply(0x3FF), 0x3FF);
        assert_eq!(apply(0x400), 0); // bit 10 dropped
        assert_eq!(apply(0xFFFF), 0x3FF);
        assert_eq!(apply(TEMP_CODE_MIN), TEMP_CODE_MIN as u32);
    }

    #[test]
    fn interrupt_mask_pack_known_bits() {
        // Each interrupt maps to a fixed bit: done=0, out_thresh=1, overtemp=2.
        assert_eq!(pack_int(false, false, false), 0);
        assert_eq!(pack_int(true, false, false), 0x01);
        assert_eq!(pack_int(false, true, false), 0x02);
        assert_eq!(pack_int(false, false, true), 0x04);
        assert_eq!(pack_int(true, true, true), 0x07);
    }

    #[test]
    fn interrupt_status_unpack_known_bits() {
        // interrupt_status reads the same bit layout it would write.
        assert_eq!(unpack_int(0), (false, false, false));
        assert_eq!(unpack_int(0x01), (true, false, false));
        assert_eq!(unpack_int(0x02), (false, true, false));
        assert_eq!(unpack_int(0x04), (false, false, true));
        assert_eq!(unpack_int(0x07), (true, true, true));
        // Higher bits in the status word are ignored.
        assert_eq!(unpack_int(0xFFFF_FFF8), (false, false, false));
    }

    #[test]
    fn interrupt_pack_unpack_round_trip() {
        // pack then unpack restores the original three flags for all 8 combos.
        for done in [false, true] {
            for out_thresh in [false, true] {
                for overtemp in [false, true] {
                    let packed = pack_int(done, out_thresh, overtemp);
                    assert_eq!(unpack_int(packed), (done, out_thresh, overtemp));
                }
            }
        }
    }
}

// ── Property-based fuzz tests ──────────────────────────────────

#[cfg(all(test, not(target_arch = "riscv32")))]
mod proptests {
    use proptest::prelude::*;

    fn pack_int(done: bool, out_thresh: bool, overtemp: bool) -> u32 {
        let mut val: u32 = 0;
        if done {
            val |= 0x01;
        }
        if out_thresh {
            val |= 0x02;
        }
        if overtemp {
            val |= 0x04;
        }
        val
    }

    fn unpack_int(sts: u32) -> (bool, bool, bool) {
        ((sts & 0x01) != 0, (sts & 0x02) != 0, (sts & 0x04) != 0)
    }

    proptest! {
        /// Fuzz: the extracted temperature code always fits in 10 bits, for any
        /// status word the hardware could present (read_raw's masking invariant).
        #[test]
        fn code_extract_always_10_bits(sts in any::<u32>()) {
            let code = ((sts >> 2) & 0x3FF) as u16;
            prop_assert!(code <= 0x3FF);
        }

        /// Fuzz: pack(flags) → unpack round-trips for any flag triple, and the
        /// packed mask never sets a bit above bit 2.
        #[test]
        fn int_pack_unpack_round_trip(done in any::<bool>(), out in any::<bool>(), over in any::<bool>()) {
            let packed = pack_int(done, out, over);
            prop_assert!(packed <= 0x07);
            prop_assert_eq!(unpack_int(packed), (done, out, over));
        }
    }
}
