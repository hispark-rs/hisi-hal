//! Low-Speed ADC (LSADC) driver for WS63 — v154 controller.
//!
//! Register map and bit fields are cross-checked against the WS63 C SDK
//! (`hal_adc_v154_regs_def.h`, `hal_adc_v154_regs_op.{c,h}`, `hal_adc_v154.c`).
//! The control bank is the contiguous `adc_regs_t` struct at base `0x4400_C000`:
//!
//! | Register      | Offset | Purpose                                            |
//! |---------------|--------|----------------------------------------------------|
//! | `LSADC_CTRL_0`  | 0x00 | scan config: per-channel enable + sample timing    |
//! | `LSADC_CTRL_1`  | 0x04 | FIFO status (`rne`/`rff`/`bsy`) + waterline        |
//! | `LSADC_CTRL_2`  | 0x08 | interrupt mask/status                              |
//! | `LSADC_CTRL_8`  | 0x1C | scan start/stop                                    |
//! | `LSADC_CTRL_9`  | 0x20 | FIFO read data (`data[13:0]`, `channel[16:14]`)    |
//! | `LSADC_CTRL_11` | 0x24 | analog enable (`da_lsadc_en`) + reset (`rstn`@16)  |
//! | `CFG_DATA_SEL`  | 0xDC | data output select                                 |
//! | `CFG_OFFSET`    | 0xE0 | offset correction                                  |
//! | `CFG_GAIN`      | 0xE4 | gain correction                                    |
//! | `CFG_CIC_FILTER_EN` | 0xE8 | CIC filter enable                              |
//! | `CFG_CIC_OSR`   | 0xEC | CIC oversampling ratio                             |
//!
//! # Status
//!
//! Not validated on silicon. The full analog power-up sequence (the SDK's
//! `hal_adc_simulation_cfg` magic writes to `da_lsadc_en` and the `da_lsadc_rwreg`
//! registers, plus offset/cap/gain calibration) is **not** implemented here;
//! [`LsAdc::set_analog_enable`] exposes `da_lsadc_en` for callers that port it.

use crate::peripherals::Lsadc;

/// LSADC channel (0-5).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AdcChannel {
    Channel0 = 0,
    Channel1 = 1,
    Channel2 = 2,
    Channel3 = 3,
    Channel4 = 4,
    Channel5 = 5,
}

impl AdcChannel {
    /// Convert a channel index (0-5) to an `AdcChannel`.
    pub fn from_index(idx: u8) -> Option<Self> {
        match idx {
            0 => Some(Self::Channel0),
            1 => Some(Self::Channel1),
            2 => Some(Self::Channel2),
            3 => Some(Self::Channel3),
            4 => Some(Self::Channel4),
            5 => Some(Self::Channel5),
            _ => None,
        }
    }
}

/// Averaging mode (`equ_model_sel`, `LSADC_CTRL_0[7:6]`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Averaging {
    /// 1 sample averaged.
    One = 0,
    /// 2 samples averaged.
    Two = 1,
    /// 4 samples averaged.
    Four = 2,
    /// 8 samples averaged.
    Eight = 3,
}

/// Scan-mode sample timing (`LSADC_CTRL_0`). Defaults match the SDK's
/// `hal_adc_auto_scan_mode_set` (8× averaging, `sample_cnt=8`, `start_cnt=0x18`).
#[derive(Debug, Clone, Copy)]
pub struct AdcConfig {
    /// Averaging mode (`equ_model_sel`, 2-bit).
    pub averaging: Averaging,
    /// Sample count (`sample_cnt`, 5-bit).
    pub sample_count: u8,
    /// Start count (`start_cnt` / SDK `satrt_cnt`, 8-bit).
    pub start_count: u8,
    /// Cast count (`cast_cnt`, 7-bit).
    pub cast_count: u8,
}

impl Default for AdcConfig {
    fn default() -> Self {
        Self { averaging: Averaging::Eight, sample_count: 0x8, start_count: 0x18, cast_count: 0x0 }
    }
}

/// ADC conversion result.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AdcSample {
    /// 14-bit conversion code.
    pub data: u16,
    /// Channel that produced this sample (0-5).
    pub channel: u8,
}

/// Parse a raw `LSADC_CTRL_9` word into a sample: `data` = bits `[13:0]`,
/// `channel` = bits `[16:14]` (matches `adc_fifo_data_str` in the SDK).
#[inline]
const fn parse_sample(raw: u32) -> AdcSample {
    AdcSample { data: (raw & 0x3FFF) as u16, channel: ((raw >> 14) & 0x07) as u8 }
}

/// LSADC driver.
pub struct LsAdc<'d> {
    _lsadc: Lsadc<'d>,
}

impl<'d> LsAdc<'d> {
    /// Create a new LSADC driver from the LSADC peripheral.
    pub fn new(lsadc: Lsadc<'d>) -> Self {
        Self { _lsadc: lsadc }
    }

    fn regs(&self) -> &'static ws63_pac::lsadc::RegisterBlock {
        // SAFETY: PAC peripheral pointer is a static physical MMIO address, always valid.
        unsafe { &*Lsadc::ptr() }
    }

    /// Release the analog reset (`LSADC_CTRL_11.da_lsadc_rstn = 1`, active-low).
    pub fn enable(&mut self) {
        self.regs().lsadc_ctrl_11().modify(|_, w| w.da_lsadc_rstn().set_bit());
    }

    /// Assert the analog reset (`LSADC_CTRL_11.da_lsadc_rstn = 0`).
    pub fn disable(&mut self) {
        self.regs().lsadc_ctrl_11().modify(|_, w| w.da_lsadc_rstn().clear_bit());
    }

    /// Write the 16-bit analog-enable field (`LSADC_CTRL_11.da_lsadc_en`).
    ///
    /// The SDK power-up sequence ORs in `0x7000`, `0xE7F`, `0x100`, `0x80`
    /// across several steps; this exposes the raw field for porting it.
    pub fn set_analog_enable(&mut self, bits: u16) {
        self.regs().lsadc_ctrl_11().modify(|_, w| unsafe { w.da_lsadc_en().bits(bits) });
    }

    /// Enable a channel and program the scan timing (`LSADC_CTRL_0`).
    ///
    /// Sets the per-channel enable bit (preserving any already-enabled channels)
    /// and the averaging/sample/start/cast counts, matching
    /// `hal_adc_auto_scan_mode_set`.
    pub fn configure_scan(&mut self, channel: AdcChannel, config: &AdcConfig) {
        self.regs().lsadc_ctrl_0().modify(|r, w| {
            let ch = r.channel().bits() | (1 << (channel as u8));
            unsafe {
                w.channel().bits(ch);
                w.equ_model_sel().bits(config.averaging as u8);
                w.sample_cnt().bits(config.sample_count & 0x1F);
                w.start_cnt().bits(config.start_count);
                w.cast_cnt().bits(config.cast_count & 0x7F)
            }
        });
    }

    /// Start an ADC scan (`LSADC_CTRL_8.lsadc_start = 1`).
    pub fn start_scan(&mut self) {
        self.regs().lsadc_ctrl_8().write(|w| w.lsadc_start().set_bit());
    }

    /// Stop an ADC scan (`LSADC_CTRL_8.lsadc_stop = 1`).
    pub fn stop_scan(&mut self) {
        self.regs().lsadc_ctrl_8().write(|w| w.lsadc_stop().set_bit());
    }

    /// Set the RX-FIFO interrupt waterline (`LSADC_CTRL_1.rxintsize`, 3-bit).
    pub fn set_fifo_waterline(&mut self, level: u8) {
        self.regs().lsadc_ctrl_1().modify(|_, w| unsafe { w.rxintsize().bits(level & 0x07) });
    }

    /// True if the RX FIFO holds at least one sample (`LSADC_CTRL_1.rne`).
    ///
    /// This is the reliable empty check — read it before [`Self::read_sample`].
    pub fn data_ready(&self) -> bool {
        self.regs().lsadc_ctrl_1().read().rne().bit_is_set()
    }

    /// Read one sample from the FIFO (`LSADC_CTRL_9`).
    ///
    /// Returns `None` when the FIFO is empty (checked via `rne`), so a genuine
    /// 0-code reading is **not** mistaken for "no data".
    pub fn read_sample(&self) -> Option<AdcSample> {
        if !self.data_ready() {
            return None;
        }
        Some(parse_sample(self.regs().lsadc_ctrl_9().read().bits()))
    }

    /// Enable the CIC filter with the given oversampling ratio.
    pub fn enable_cic_filter(&mut self, oversampling_ratio: u8) {
        let r = self.regs();
        unsafe {
            r.cfg_cic_osr().write(|w| w.cic_osr().bits(oversampling_ratio));
        }
        r.cfg_cic_filter_en().write(|w| w.cic_filter_en().set_bit());
    }

    /// Disable the CIC filter.
    pub fn disable_cic_filter(&mut self) {
        self.regs().cfg_cic_filter_en().write(|w| w.cic_filter_en().clear_bit());
    }

    /// Set the ADC offset correction value (`CFG_OFFSET`).
    pub fn set_offset(&mut self, offset: u16) {
        self.regs().cfg_offset().write(|w| unsafe { w.offset().bits(offset) });
    }

    /// Set the ADC gain correction value (`CFG_GAIN`).
    pub fn set_gain(&mut self, gain: u16) {
        self.regs().cfg_gain().write(|w| unsafe { w.gain().bits(gain) });
    }

    /// Select the data source: `true` = post-processed, `false` = raw ADC code.
    pub fn set_data_select(&mut self, processed: bool) {
        self.regs().cfg_data_sel().write(|w| w.data_sel().bit(processed));
    }
}

// ── Tests ──────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_parse_sample_data_and_channel() {
        // channel 3, code 0x0AAA  ->  raw = (3 << 14) | 0x0AAA
        let raw: u32 = (3 << 14) | 0x0AAA;
        let s = parse_sample(raw);
        assert_eq!(s.data, 0x0AAA);
        assert_eq!(s.channel, 3);
    }

    #[test]
    fn test_parse_sample_max_code() {
        let s = parse_sample(0x3FFF); // all data bits, channel 0
        assert_eq!(s.data, 0x3FFF);
        assert_eq!(s.channel, 0);
    }

    #[test]
    fn test_parse_sample_channel_field_is_3_bits() {
        // bits above [16:14] must not leak into channel
        let raw: u32 = 0xFFFF_FFFF;
        let s = parse_sample(raw);
        assert_eq!(s.channel, 0x07);
        assert_eq!(s.data, 0x3FFF);
    }

    #[test]
    fn test_channel_from_index() {
        assert_eq!(AdcChannel::from_index(0), Some(AdcChannel::Channel0));
        assert_eq!(AdcChannel::from_index(5), Some(AdcChannel::Channel5));
        assert_eq!(AdcChannel::from_index(6), None);
        assert_eq!(AdcChannel::from_index(255), None);
    }

    #[test]
    fn test_default_config_matches_sdk() {
        let c = AdcConfig::default();
        assert_eq!(c.averaging as u8, 3); // AVERAGE_OF_EIGHT_SAMPLES
        assert_eq!(c.sample_count, 0x8);
        assert_eq!(c.start_count, 0x18);
        assert_eq!(c.cast_count, 0x0);
    }
}

// ── Property-based fuzz tests ──────────────────────────────────

#[cfg(test)]
mod proptests {
    use super::*;
    use proptest::prelude::*;

    proptest! {
        /// Fuzz: channel field is always within 3 bits regardless of input.
        #[test]
        fn parse_channel_always_3_bits(raw in any::<u32>()) {
            prop_assert!(parse_sample(raw).channel <= 0x07);
        }

        /// Fuzz: data field is always within 14 bits.
        #[test]
        fn parse_data_always_14_bits(raw in any::<u32>()) {
            prop_assert!(parse_sample(raw).data <= 0x3FFF);
        }

        /// Fuzz: data and channel are extracted from the correct, disjoint lanes.
        #[test]
        fn parse_sample_lanes(raw in any::<u32>()) {
            let s = parse_sample(raw);
            prop_assert_eq!(s.data, (raw & 0x3FFF) as u16);
            prop_assert_eq!(s.channel, ((raw >> 14) & 0x07) as u8);
        }

        /// Fuzz: AdcChannel::from_index returns Some for 0-5, None otherwise.
        #[test]
        fn channel_from_index_coverage(idx in any::<u8>()) {
            prop_assert_eq!(AdcChannel::from_index(idx).is_some(), idx <= 5);
        }
    }
}
