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
    /// Channel 0 (`LSADC_CTRL_0.channel` bit 0).
    Channel0 = 0,
    /// Channel 1 (`LSADC_CTRL_0.channel` bit 1).
    Channel1 = 1,
    /// Channel 2 (`LSADC_CTRL_0.channel` bit 2).
    Channel2 = 2,
    /// Channel 3 (`LSADC_CTRL_0.channel` bit 3).
    Channel3 = 3,
    /// Channel 4 (`LSADC_CTRL_0.channel` bit 4).
    Channel4 = 4,
    /// Channel 5 (`LSADC_CTRL_0.channel` bit 5).
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

/// A validated `sample_cnt` (`LSADC_CTRL_0`, 5-bit). [`SampleCount::new`] returns
/// `None` for `> 31`, so [`AdcConfig`] can never silently `& 0x1F`-truncate it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct SampleCount(u8);

impl SampleCount {
    /// Construct from a raw count. `None` if `> 31` (the 5-bit field maximum).
    pub const fn new(n: u8) -> Option<Self> {
        if n <= 0x1F { Some(Self(n)) } else { None }
    }
    /// The raw 5-bit count.
    pub const fn bits(self) -> u8 {
        self.0
    }
}

/// A validated `cast_cnt` (`LSADC_CTRL_0`, 7-bit). [`CastCount::new`] returns `None`
/// for `> 127`, so [`AdcConfig`] can never silently `& 0x7F`-truncate it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct CastCount(u8);

impl CastCount {
    /// Construct from a raw count. `None` if `> 127` (the 7-bit field maximum).
    pub const fn new(n: u8) -> Option<Self> {
        if n <= 0x7F { Some(Self(n)) } else { None }
    }
    /// The raw 7-bit count.
    pub const fn bits(self) -> u8 {
        self.0
    }
}

/// A validated RX-FIFO interrupt waterline (`LSADC_CTRL_1.rxintsize`, 3-bit).
/// [`FifoWaterline::new`] returns `None` for `> 7`, so the currently unstable
/// `LsAdc::set_fifo_waterline` can never silently `& 0x07`-truncate it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct FifoWaterline(u8);

impl FifoWaterline {
    /// Construct from a raw level. `None` if `> 7` (the 3-bit field maximum).
    pub const fn new(level: u8) -> Option<Self> {
        if level <= 0x07 { Some(Self(level)) } else { None }
    }
    /// The raw 3-bit level.
    pub const fn bits(self) -> u8 {
        self.0
    }
}

/// Scan-mode sample timing (`LSADC_CTRL_0`). Defaults match the SDK's
/// `hal_adc_auto_scan_mode_set` (8× averaging, `sample_cnt=8`, `start_cnt=0x18`).
///
/// The count fields are validated newtypes ([`SampleCount`]/[`CastCount`]) so an
/// out-of-range value is rejected at construction instead of silently masked here.
#[derive(Debug, Clone, Copy)]
pub struct AdcConfig {
    /// Averaging mode (`equ_model_sel`, 2-bit).
    pub averaging: Averaging,
    /// Sample count (`sample_cnt`, validated 5-bit).
    pub sample_count: SampleCount,
    /// Start count (`start_cnt` / SDK `satrt_cnt`, 8-bit — the full `u8` field).
    pub start_count: u8,
    /// Cast count (`cast_cnt`, validated 7-bit).
    pub cast_count: CastCount,
}

impl Default for AdcConfig {
    fn default() -> Self {
        Self {
            averaging: Averaging::Eight,
            // 0x8 and 0x0 are in range, so these unwraps never fire.
            sample_count: SampleCount::new(0x8).unwrap(),
            start_count: 0x18,
            cast_count: CastCount::new(0x0).unwrap(),
        }
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

    fn regs(&self) -> &'static crate::soc::pac::lsadc::RegisterBlock {
        // SAFETY: PAC peripheral pointer is a static physical MMIO address, always valid.
        unsafe { &*Lsadc::ptr() }
    }

    /// Release the analog reset (`LSADC_CTRL_11.da_lsadc_rstn = 1`, active-low).
    #[instability::unstable]
    pub fn enable(&mut self) {
        self.regs().lsadc_ctrl_11().modify(|_, w| w.da_lsadc_rstn().set_bit());
    }

    /// Assert the analog reset (`LSADC_CTRL_11.da_lsadc_rstn = 0`).
    #[instability::unstable]
    pub fn disable(&mut self) {
        self.regs().lsadc_ctrl_11().modify(|_, w| w.da_lsadc_rstn().clear_bit());
    }

    /// Write the 16-bit analog-enable field (`LSADC_CTRL_11.da_lsadc_en`).
    ///
    /// The SDK power-up sequence ORs in `0x7000`, `0xE7F`, `0x100`, `0x80`
    /// across several steps; this exposes the raw field for porting it.
    #[instability::unstable]
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
                // Pre-validated by the newtypes — no silent mask needed.
                w.sample_cnt().bits(config.sample_count.bits());
                w.start_cnt().bits(config.start_count);
                w.cast_cnt().bits(config.cast_count.bits())
            }
        });
    }

    /// Start an ADC scan (`LSADC_CTRL_8.lsadc_start = 1`).
    #[instability::unstable]
    pub fn start_scan(&mut self) {
        self.regs().lsadc_ctrl_8().write(|w| w.lsadc_start().set_bit());
    }

    /// Stop an ADC scan (`LSADC_CTRL_8.lsadc_stop = 1`).
    #[instability::unstable]
    pub fn stop_scan(&mut self) {
        self.regs().lsadc_ctrl_8().write(|w| w.lsadc_stop().set_bit());
    }

    /// Set the RX-FIFO interrupt waterline (`LSADC_CTRL_1.rxintsize`, 3-bit). The
    /// level is a validated [`FifoWaterline`] so an out-of-range value is rejected
    /// at construction rather than silently masked here.
    #[instability::unstable]
    pub fn set_fifo_waterline(&mut self, level: FifoWaterline) {
        self.regs().lsadc_ctrl_1().modify(|_, w| unsafe { w.rxintsize().bits(level.bits()) });
    }

    /// True if the RX FIFO holds at least one sample (`LSADC_CTRL_1.rne`).
    ///
    /// This is the reliable empty check — read it before [`Self::read_sample`].
    #[instability::unstable]
    pub fn data_ready(&self) -> bool {
        self.regs().lsadc_ctrl_1().read().rne().bit_is_set()
    }

    /// Read one sample from the FIFO (`LSADC_CTRL_9`).
    ///
    /// Returns `None` when the FIFO is empty (checked via `rne`), so a genuine
    /// 0-code reading is **not** mistaken for "no data".
    #[instability::unstable]
    pub fn read_sample(&self) -> Option<AdcSample> {
        if !self.data_ready() {
            return None;
        }
        Some(parse_sample(self.regs().lsadc_ctrl_9().read().bits()))
    }

    /// Enable the CIC filter with the given oversampling ratio.
    #[instability::unstable]
    pub fn enable_cic_filter(&mut self, oversampling_ratio: u8) {
        let r = self.regs();
        unsafe {
            r.cfg_cic_osr().write(|w| w.cic_osr().bits(oversampling_ratio));
        }
        r.cfg_cic_filter_en().write(|w| w.cic_filter_en().set_bit());
    }

    /// Disable the CIC filter.
    #[instability::unstable]
    pub fn disable_cic_filter(&mut self) {
        self.regs().cfg_cic_filter_en().write(|w| w.cic_filter_en().clear_bit());
    }

    /// Set the ADC offset correction value (`CFG_OFFSET`).
    #[instability::unstable]
    pub fn set_offset(&mut self, offset: u16) {
        self.regs().cfg_offset().write(|w| unsafe { w.offset().bits(offset) });
    }

    /// Set the ADC gain correction value (`CFG_GAIN`).
    #[instability::unstable]
    pub fn set_gain(&mut self, gain: u16) {
        self.regs().cfg_gain().write(|w| unsafe { w.gain().bits(gain) });
    }

    /// Select the data source: `true` = post-processed, `false` = raw ADC code.
    #[instability::unstable]
    pub fn set_data_select(&mut self, processed: bool) {
        self.regs().cfg_data_sel().write(|w| w.data_sel().bit(processed));
    }
}

// ── Tests ──────────────────────────────────────────────────────

#[cfg(all(test, not(target_arch = "riscv32")))]
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
        assert_eq!(c.sample_count.bits(), 0x8);
        assert_eq!(c.start_count, 0x18);
        assert_eq!(c.cast_count.bits(), 0x0);
    }

    #[test]
    fn count_newtypes_reject_out_of_range() {
        // 5-bit sample_cnt, 7-bit cast_cnt, 3-bit waterline — boundaries enforced.
        assert!(SampleCount::new(0x1F).is_some());
        assert!(SampleCount::new(0x20).is_none());
        assert!(CastCount::new(0x7F).is_some());
        assert!(CastCount::new(0x80).is_none());
        assert!(FifoWaterline::new(7).is_some());
        assert!(FifoWaterline::new(8).is_none());
    }
}

// ── Property-based fuzz tests ──────────────────────────────────

#[cfg(all(test, not(target_arch = "riscv32")))]
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

// ── Async LSADC (bespoke; LSADC_INTR = IRQ 72) ──────────────────────────────
#[cfg(all(feature = "chip-ws63", feature = "async", feature = "unstable"))]
mod asynch_impl {
    use super::{AdcSample, LsAdc};
    use crate::asynch::IrqSignal;
    use crate::interrupt::{self, Interrupt};
    use core::future::Future;
    use core::pin::Pin;
    use core::task::{Context, Poll};

    static LSADC_SIGNAL: IrqSignal = IrqSignal::new();

    /// LSADC trap hook (IRQ 72): wake the awaiting conversion. The sample is
    /// read-cleared by `read_sample`, so the ISR just signals + clears pending.
    pub fn on_interrupt() {
        LSADC_SIGNAL.signal();
        interrupt::clear_pending(Interrupt::LSADC_INTR);
    }

    struct ConvFuture;
    impl Future for ConvFuture {
        type Output = ();
        fn poll(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<()> {
            LSADC_SIGNAL.poll_wait(cx)
        }
    }

    impl LsAdc<'_> {
        /// Start a scan and await conversion-complete (IRQ 72), returning a sample.
        /// On hardware this parks until the IRQ; the WS63 model fills the FIFO
        /// synchronously, so the fast path returns without parking.
        pub async fn read_async(&mut self) -> Option<AdcSample> {
            LSADC_SIGNAL.reset();
            // SAFETY: enabling a known, fixed WS63 IRQ line.
            unsafe { interrupt::enable(Interrupt::LSADC_INTR) };
            self.start_scan();
            if !self.data_ready() {
                ConvFuture.await;
            }
            self.read_sample()
        }
    }
}

#[cfg(all(feature = "chip-ws63", feature = "async", feature = "unstable"))]
pub use asynch_impl::on_interrupt;
