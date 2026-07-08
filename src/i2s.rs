//! I2S (Inter-IC Sound) / PCM audio interface driver for WS63.
//!
//! The WS63 I2S peripheral (vendor "SIO v151") supports both I2S and PCM modes,
//! master and slave operation, 2/4/8/16 channels, 16/18/20/24/32-bit samples, and
//! separate TX/RX FIFOs with threshold interrupts.
//!
//! # Typed config — "if it compiles, it runs on silicon"
//!
//! The role is encoded in the **type** ([`Master`] / [`Slave`]) via two fused
//! constructors — [`I2sDriver::new_master`] / [`I2sDriver::new_slave`] — so there
//! is no separate `configure()` step that could be skipped or mis-ordered. A
//! **master generates BCLK and FS**, so its clock dividers are *derived* from the
//! `(data_width, channels)` pair exactly as the vendor `sio_porting` does, never
//! supplied raw:
//!
//! * `FS_DIV_NUM      = data_width · channels`
//! * `FS_DIV_RATIO    = data_width · channels / 2`  (50 % duty)
//! * `BCLK_DIV_NUM    = round(I2S_MCLK / (32 · data_width · channels))`
//!
//! Because the dividers are derived from non-zero enums, a **zero-divider master is
//! unrepresentable** and every divider is provably in field range (see the
//! exhaustive `derived_dividers_in_range` host test) — there is no silent `& mask`
//! truncation of a user-supplied value. A **slave** receives BCLK/FS externally, so
//! [`SlaveConfig`] carries no dividers at all. `new_master` also self-enables the
//! I2S clock tree (CMU + CKEN bus/clk gates), the class-C "construct → clocked"
//! guarantee.
//!
//! Register bit layouts follow the authoritative vendor `hal_sio_v151_regs_def.h`
//! (the ws63-pac SVD encodes a fabricated textbook layout for `mode`/`i2s_crg`/
//! `data_width_set`; the driver writes the correct bits via raw `.bits()`), and the
//! [`DataWidth`]/[`ChannelCount`] enums are tabled to the silicon codes (16/18/20/
//! 24/32-bit; 2/4/8/16-channel — there is no 8/10/12/14-bit or 6-channel mode).
//!
//! On the current HIL board no audio codec is wired, so only register read-back and
//! the IP `version` are validated on silicon; the master audio waveform itself is
//! not codec-verified.

use crate::peripherals::I2s;
use core::marker::PhantomData;

/// I2S operating mode (`mode` register bit 0).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum I2sMode {
    /// I2S (Philips) mode.
    I2s,
    /// PCM mode.
    Pcm,
}

/// Clock edge selection (`mode` register bit 6). Meaningful for a slave; a master
/// drives the vendor-default falling edge.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[instability::unstable]
pub enum ClockEdge {
    /// Rising edge.
    Rising,
    /// Falling edge.
    Falling,
}

/// Number of audio channels (`mode.chn_num`, bits [5:4]).
///
/// The field codes 0..3 map to **2/4/8/16** channels — there is no 6-channel mode
/// (the previous `Six` table entry was wrong).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ChannelCount {
    /// 2 channels (stereo).
    Two = 0,
    /// 4 channels.
    Four = 1,
    /// 8 channels.
    Eight = 2,
    /// 16 channels.
    Sixteen = 3,
}

impl ChannelCount {
    /// The actual channel count (2/4/8/16), used to derive the master dividers.
    pub const fn count(self) -> u32 {
        match self {
            ChannelCount::Two => 2,
            ChannelCount::Four => 4,
            ChannelCount::Eight => 8,
            ChannelCount::Sixteen => 16,
        }
    }
}

/// TX/RX sample width (`data_width_set.tx_mode`/`rx_mode`, 3-bit fields).
///
/// The field codes are the silicon enum: `1=16, 2=18, 3=20, 4=24, 5=32` bits (code
/// 0 is reserved). The previous `Bits8..Bits24 = 0..7` table did not exist on the
/// hardware.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum DataWidth {
    /// 16-bit samples.
    Bits16 = 1,
    /// 18-bit samples.
    Bits18 = 2,
    /// 20-bit samples.
    Bits20 = 3,
    /// 24-bit samples.
    Bits24 = 4,
    /// 32-bit samples.
    Bits32 = 5,
}

impl DataWidth {
    /// The sample width in bits (16/18/20/24/32), used to derive the master dividers.
    pub const fn bits(self) -> u32 {
        match self {
            DataWidth::Bits16 => 16,
            DataWidth::Bits18 => 18,
            DataWidth::Bits20 => 20,
            DataWidth::Bits24 => 24,
            DataWidth::Bits32 => 32,
        }
    }
}

// ── Master clock-divider derivation (vendor `sio_porting`) ──────────────────────
//
// I2S_MCLK_RATE = 12288 (kHz units), FREQ_OF_NEED = 32, I2S_DUTY_CYCLE = 2.
// All three dividers are derived from (data_width · channels), so a master never
// takes a raw divider and can never present a zero/out-of-range one.

/// `I2S_MCLK_RATE` in the same kHz units the vendor uses for the BCLK divider.
const I2S_MCLK: u32 = 12288;
/// `FREQ_OF_NEED` — the BCLK derivation's per-sample reference factor.
const FREQ_OF_NEED: u32 = 32;

/// The three derived master dividers `(bclk_div_num, fs_div_num, fs_div_ratio_num)`
/// for a `(data_width, channels)` pair, matching the vendor derivation exactly:
///   fs_div_num   = dw·ch                    (≤ 512, fits the 10-bit field)
///   fs_div_ratio = dw·ch / 2  (50 % duty)   (≤ 256, fits the 11-bit field)
///   bclk_div_num = round(MCLK / (32·dw·ch)) (∈ [1, 12], fits the 7-bit field)
/// `round` is round-half-up: `(2·N + D) / (2·D)` integer division.
const fn derive_dividers(dw: DataWidth, ch: ChannelCount) -> (u32, u32, u32) {
    let prod = dw.bits() * ch.count(); // dw·ch, always even and ≥ 32
    let fs_div_num = prod;
    let fs_div_ratio = prod / 2;
    let d = FREQ_OF_NEED * prod; // denominator
    // round-half-up(MCLK / d) = (2·MCLK + d) / (2·d); never 0 for these inputs.
    let bclk = (2 * I2S_MCLK + d) / (2 * d);
    let bclk = if bclk == 0 { 1 } else { bclk };
    (bclk, fs_div_num, fs_div_ratio)
}

// ── Role type-state ─────────────────────────────────────────────────────────────

mod sealed {
    /// Sealed marker for the I2S role type-state.
    pub trait Role {}
}

/// Type-state marker: the I2S block **generates** BCLK and FS (clock master).
#[derive(Debug)]
pub struct Master;
/// Type-state marker: the I2S block **receives** BCLK and FS externally (clock slave).
#[derive(Debug)]
#[instability::unstable]
pub struct Slave;
impl sealed::Role for Master {}
impl sealed::Role for Slave {}

/// Master-mode configuration. Carries **no dividers** — they are derived from
/// `(data_width, channels)` so a zero-divider master cannot be expressed.
#[derive(Debug, Clone, Copy)]
pub struct MasterConfig {
    /// I2S or PCM framing.
    pub mode: I2sMode,
    /// Number of channels.
    pub channels: ChannelCount,
    /// Sample width (applied to both TX and RX, as the vendor driver does).
    pub data_width: DataWidth,
    /// TX FIFO threshold (8-bit field).
    pub tx_fifo_threshold: u8,
    /// RX FIFO threshold (8-bit field).
    pub rx_fifo_threshold: u8,
    /// Internal TX→RX loopback (the `version.loop` bit), for self-test.
    pub loopback: bool,
}

impl Default for MasterConfig {
    fn default() -> Self {
        Self {
            mode: I2sMode::I2s,
            channels: ChannelCount::Two,
            data_width: DataWidth::Bits16,
            tx_fifo_threshold: 8,
            rx_fifo_threshold: 8,
            loopback: false,
        }
    }
}

/// Slave-mode configuration. A slave receives BCLK/FS externally, so there are no
/// dividers; the externally-clocked sampling edge is selectable.
#[derive(Debug, Clone, Copy)]
#[instability::unstable]
pub struct SlaveConfig {
    /// I2S or PCM framing.
    pub mode: I2sMode,
    /// Number of channels.
    pub channels: ChannelCount,
    /// Sample width (applied to both TX and RX).
    pub data_width: DataWidth,
    /// Sampling clock edge.
    pub clock_edge: ClockEdge,
    /// TX FIFO threshold (8-bit field).
    pub tx_fifo_threshold: u8,
    /// RX FIFO threshold (8-bit field).
    pub rx_fifo_threshold: u8,
    /// Internal TX→RX loopback (the `version.loop` bit), for self-test.
    pub loopback: bool,
}

impl Default for SlaveConfig {
    fn default() -> Self {
        Self {
            mode: I2sMode::I2s,
            channels: ChannelCount::Two,
            data_width: DataWidth::Bits16,
            clock_edge: ClockEdge::Rising,
            tx_fifo_threshold: 8,
            rx_fifo_threshold: 8,
            loopback: false,
        }
    }
}

/// I2S audio interface driver, parameterised by clock role ([`Master`] / [`Slave`]).
pub struct I2sDriver<'d, R> {
    _i2s: I2s<'d>,
    _role: PhantomData<R>,
}

/// Build the `mode` register value per the vendor `hal_sio_v151_regs_def.h`:
///   bit0 mode (0=i2s,1=pcm) · bits[5:4] chn_num · bit6 clk_edge · bit7 ms_mode_sel.
const fn mode_bits(mode: I2sMode, channels: ChannelCount, edge: ClockEdge, master: bool) -> u32 {
    let mut m = 0u32;
    if matches!(mode, I2sMode::Pcm) {
        m |= 1 << 0;
    }
    m |= ((channels as u32) & 0x3) << 4;
    if matches!(edge, ClockEdge::Falling) {
        m |= 1 << 6;
    }
    if master {
        m |= 1 << 7;
    }
    m
}

/// Build the `data_width_set` value: `tx_mode` bits[2:0], `rx_mode` bits[5:3], both
/// set to the same width (matching the vendor `hal_sio_v151_data_width_set`).
const fn data_width_bits(dw: DataWidth) -> u32 {
    let code = (dw as u32) & 0x7;
    code | (code << 3)
}

impl<'d> I2sDriver<'d, Master> {
    /// Create and fully configure an I2S **master**. Self-enables the I2S clock
    /// tree, programs the framing/width/FIFO registers, and derives + programs the
    /// BCLK/FS dividers from `(data_width, channels)`.
    pub fn new_master(i2s: I2s<'d>, config: &MasterConfig) -> Self {
        enable_i2s_clock();
        let driver = Self { _i2s: i2s, _role: PhantomData };
        let r = driver.regs();

        // mode: master, vendor-default falling edge.
        let mode = mode_bits(config.mode, config.channels, ClockEdge::Falling, true);
        // Derived dividers (provably in field range for every enum combination).
        let (bclk, fs_num, fs_ratio) = derive_dividers(config.data_width, config.channels);
        unsafe {
            r.mode().write(|w| w.bits(mode));
            r.i2s_fs_div_num().write(|w| w.bits(fs_num));
            r.i2s_fs_div_ratio_num().write(|w| w.bits(fs_ratio));
            r.i2s_bclk_div_num().write(|w| w.bits(bclk));
            // i2s_crg: bclk_div_en (bit0) + crg_clken (bit1); phase bits 0.
            r.i2s_crg().write(|w| w.bits(0b11));
        }
        driver.apply_common(config.data_width, config.tx_fifo_threshold, config.rx_fifo_threshold, config.loopback);
        driver
    }
}

impl<'d> I2sDriver<'d, Slave> {
    /// Create and fully configure an I2S **slave** (BCLK/FS supplied externally;
    /// no dividers). Self-enables the bus clock so the FIFOs/registers are reachable.
    #[instability::unstable]
    pub fn new_slave(i2s: I2s<'d>, config: &SlaveConfig) -> Self {
        enable_i2s_clock();
        let driver = Self { _i2s: i2s, _role: PhantomData };
        let r = driver.regs();

        let mode = mode_bits(config.mode, config.channels, config.clock_edge, false);
        unsafe {
            r.mode().write(|w| w.bits(mode));
            // Slave does not generate clocks: leave crg dividers off (phase bits 0).
            r.i2s_crg().write(|w| w.bits(0));
        }
        driver.apply_common(config.data_width, config.tx_fifo_threshold, config.rx_fifo_threshold, config.loopback);
        driver
    }
}

impl<'d, R: sealed::Role> I2sDriver<'d, R> {
    fn regs(&self) -> &'static crate::soc::pac::i2s::RegisterBlock {
        // SAFETY: PAC peripheral pointer is a static physical MMIO address, always valid
        unsafe { &*I2s::ptr() }
    }

    /// Shared tail of `new_master`/`new_slave`: data width, FIFO thresholds, sign
    /// extension, loopback, and FIFO reset — the role-independent register writes.
    fn apply_common(&self, data_width: DataWidth, tx_thresh: u8, rx_thresh: u8, loopback: bool) {
        let r = self.regs();
        unsafe {
            r.data_width_set().write(|w| w.bits(data_width_bits(data_width)));
            let thresh = (tx_thresh as u32) | ((rx_thresh as u32) << 8);
            r.fifo_threshold().write(|w| w.bits(thresh));
            // version.loop (bit 8): internal TX→RX loopback.
            r.version().write(|w| w.bits(if loopback { 1 << 8 } else { 0 }));
            // Signed sample extension for correct audio handling.
            r.signed_ext().write(|w| w.bits(0x01));
            // Reset both FIFOs (ct_set bits 2/3).
            r.ct_set().write(|w| w.bits(0x0C));
        }
    }

    /// Enable TX.
    #[instability::unstable]
    pub fn enable_tx(&mut self) {
        unsafe {
            self.regs().ct_set().write(|w| w.bits(0x01)); // tx_en
        }
    }

    /// Enable RX.
    #[instability::unstable]
    pub fn enable_rx(&mut self) {
        unsafe {
            self.regs().ct_set().write(|w| w.bits(0x02)); // rx_en
        }
    }

    /// Disable TX.
    #[instability::unstable]
    pub fn disable_tx(&mut self) {
        unsafe {
            self.regs().ct_clr().write(|w| w.bits(0x01));
        }
    }

    /// Disable RX.
    #[instability::unstable]
    pub fn disable_rx(&mut self) {
        unsafe {
            self.regs().ct_clr().write(|w| w.bits(0x02));
        }
    }

    /// Reset TX FIFO.
    #[instability::unstable]
    pub fn reset_tx(&mut self) {
        unsafe {
            self.regs().ct_set().write(|w| w.bits(0x04));
        }
    }

    /// Reset RX FIFO.
    #[instability::unstable]
    pub fn reset_rx(&mut self) {
        unsafe {
            self.regs().ct_set().write(|w| w.bits(0x08));
        }
    }

    /// Enable TX interrupt.
    #[instability::unstable]
    pub fn enable_tx_interrupt(&mut self) {
        unsafe {
            self.regs().ct_set().write(|w| w.bits(0x10));
        }
    }

    /// Enable RX interrupt.
    #[instability::unstable]
    pub fn enable_rx_interrupt(&mut self) {
        unsafe {
            self.regs().ct_set().write(|w| w.bits(0x20));
        }
    }

    /// Write a sample to the left TX channel.
    #[instability::unstable]
    pub fn write_left(&mut self, data: u32) {
        unsafe {
            self.regs().left_tx().write(|w| w.bits(data));
        }
    }

    /// Write a sample to the right TX channel.
    #[instability::unstable]
    pub fn write_right(&mut self, data: u32) {
        unsafe {
            self.regs().right_tx().write(|w| w.bits(data));
        }
    }

    /// Read a sample from the left RX channel.
    #[instability::unstable]
    pub fn read_left(&self) -> u32 {
        self.regs().left_rx().read().bits()
    }

    /// Read a sample from the right RX channel.
    #[instability::unstable]
    pub fn read_right(&self) -> u32 {
        self.regs().right_rx().read().bits()
    }

    /// Get TX FIFO depth (left channel).
    #[instability::unstable]
    pub fn tx_fifo_left_depth(&self) -> u8 {
        (self.regs().tx_sta().read().bits() & 0xFF) as u8
    }

    /// Get TX FIFO depth (right channel).
    #[instability::unstable]
    pub fn tx_fifo_right_depth(&self) -> u8 {
        ((self.regs().tx_sta().read().bits() >> 8) & 0xFF) as u8
    }

    /// Get RX FIFO depth (left channel).
    #[instability::unstable]
    pub fn rx_fifo_left_depth(&self) -> u8 {
        (self.regs().rx_sta().read().bits() & 0xFF) as u8
    }

    /// Get RX FIFO depth (right channel).
    #[instability::unstable]
    pub fn rx_fifo_right_depth(&self) -> u8 {
        ((self.regs().rx_sta().read().bits() >> 8) & 0xFF) as u8
    }

    /// Check interrupt status.
    ///
    /// Returns `(rx_int, tx_int, rx_overflow, tx_underflow)`.
    #[instability::unstable]
    pub fn interrupt_status(&self) -> (bool, bool, bool, bool) {
        let sts = self.regs().intstatus().read().bits();
        ((sts & 0x01) != 0, (sts & 0x02) != 0, (sts & 0x04) != 0, (sts & 0x08) != 0)
    }

    /// Clear all interrupts.
    #[instability::unstable]
    pub fn clear_interrupts(&mut self) {
        unsafe {
            self.regs().intclr().write(|w| w.bits(0x0F));
        }
    }

    /// Set the interrupt mask.
    ///
    /// * `rx_int_mask` — Mask RX interrupt
    /// * `tx_int_mask` — Mask TX interrupt
    /// * `rx_overflow_mask` — Mask RX overflow interrupt
    /// * `tx_underflow_mask` — Mask TX underflow interrupt
    #[instability::unstable]
    pub fn set_interrupt_mask(
        &mut self,
        rx_int_mask: bool,
        tx_int_mask: bool,
        rx_overflow_mask: bool,
        tx_underflow_mask: bool,
    ) {
        let mut val: u32 = 0;
        if rx_int_mask {
            val |= 0x01;
        }
        if tx_int_mask {
            val |= 0x02;
        }
        if rx_overflow_mask {
            val |= 0x04;
        }
        if tx_underflow_mask {
            val |= 0x08;
        }
        unsafe {
            self.regs().intmask().write(|w| w.bits(val));
        }
    }

    /// Get the I2S IP version (low byte of the `version` register).
    pub fn version(&self) -> u8 {
        (self.regs().version().read().bits() & 0xFF) as u8
    }
}

/// Bring up the I2S clock tree the way the vendor `sio_porting_clock_enable` does on
/// WS63: release the CMU divider reset-sync, then enable the CKEN_CTL0 clk (bit 12)
/// and bus (bit 11) gates. WITHOUT this the SIO registers do not latch — the class-C
/// "construct → clocked" guarantee.
///
/// The addresses are WS63-specific (CMU/CLDO_CRG), so the body is gated to
/// `chip-ws63`; on BS2X this is a no-op (their SIO clock tree differs).
#[cfg(feature = "chip-ws63")]
fn enable_i2s_clock() {
    let cmu = unsafe { &*crate::soc::pac::Cmu::ptr() };
    let cldo = unsafe { &*crate::peripherals::CldoCrg::ptr() };
    cmu.cmu_new_cfg0().modify(|_, w| w.cmu_div_ad_rstn_sync().set_bit());
    cldo.cken_ctl0().modify(|_, w| w.i2s_cken().set_bit().i2s_bus_cken().set_bit());
}

#[cfg(not(feature = "chip-ws63"))]
fn enable_i2s_clock() {}

// ── Tests ──────────────────────────────────────────────────────

#[cfg(all(test, not(target_arch = "riscv32")))]
mod tests {
    use super::*;

    #[test]
    fn channel_count_field_codes() {
        // chn_num field codes 0..3 → 2/4/8/16 channels (no 6-channel mode).
        assert_eq!(ChannelCount::Two as u32, 0);
        assert_eq!(ChannelCount::Four as u32, 1);
        assert_eq!(ChannelCount::Eight as u32, 2);
        assert_eq!(ChannelCount::Sixteen as u32, 3);
        assert_eq!(ChannelCount::Two.count(), 2);
        assert_eq!(ChannelCount::Four.count(), 4);
        assert_eq!(ChannelCount::Eight.count(), 8);
        assert_eq!(ChannelCount::Sixteen.count(), 16);
    }

    #[test]
    fn data_width_field_codes() {
        // tx/rx_mode codes: 1=16, 2=18, 3=20, 4=24, 5=32 (code 0 reserved).
        assert_eq!(DataWidth::Bits16 as u32, 1);
        assert_eq!(DataWidth::Bits18 as u32, 2);
        assert_eq!(DataWidth::Bits20 as u32, 3);
        assert_eq!(DataWidth::Bits24 as u32, 4);
        assert_eq!(DataWidth::Bits32 as u32, 5);
        assert_eq!(DataWidth::Bits16.bits(), 16);
        assert_eq!(DataWidth::Bits32.bits(), 32);
    }

    #[test]
    fn mode_encoding_master_default() {
        // I2S + 2ch + falling (master default) + master bit = bit6 | bit7 = 0xC0.
        let m = mode_bits(I2sMode::I2s, ChannelCount::Two, ClockEdge::Falling, true);
        assert_eq!(m, (1 << 6) | (1 << 7));
    }

    #[test]
    fn mode_encoding_fields_disjoint() {
        // chn_num occupies bits[5:4]; pcm=bit0; edge=bit6; master=bit7 — no overlap.
        assert_eq!(mode_bits(I2sMode::Pcm, ChannelCount::Two, ClockEdge::Rising, false), 1 << 0);
        assert_eq!(mode_bits(I2sMode::I2s, ChannelCount::Sixteen, ClockEdge::Rising, false), 0b11 << 4);
        assert_eq!(mode_bits(I2sMode::I2s, ChannelCount::Two, ClockEdge::Falling, false), 1 << 6);
        assert_eq!(mode_bits(I2sMode::I2s, ChannelCount::Two, ClockEdge::Rising, true), 1 << 7);
        // Eight channels = code 2 → bits[5:4] = 0b10 = 0x20.
        assert_eq!(mode_bits(I2sMode::I2s, ChannelCount::Eight, ClockEdge::Rising, false), 2 << 4);
    }

    #[test]
    fn data_width_set_encoding() {
        // tx_mode bits[2:0], rx_mode bits[5:3], both the same width code.
        // 16-bit (code 1): 0b001 | (0b001 << 3) = 0x09.
        assert_eq!(data_width_bits(DataWidth::Bits16), 0x09);
        // 32-bit (code 5): 0b101 | (0b101 << 3) = 0x2D.
        assert_eq!(data_width_bits(DataWidth::Bits32), 0x2D);
    }

    #[test]
    fn derived_dividers_known_value() {
        // 16-bit stereo: dw·ch = 32. fs_num=32, fs_ratio=16,
        // bclk = round(12288 / (32·32)) = round(12.0) = 12.
        let (bclk, fs_num, fs_ratio) = derive_dividers(DataWidth::Bits16, ChannelCount::Two);
        assert_eq!(fs_num, 32);
        assert_eq!(fs_ratio, 16);
        assert_eq!(bclk, 12);
    }

    /// Exhaustive proof: for EVERY (data_width, channels) pair the derived dividers
    /// fit their hardware field widths — bclk 7-bit, fs_num 10-bit, fs_ratio 11-bit —
    /// and are all non-zero. This is what makes a zero/overflowing master divider
    /// unrepresentable instead of silently `& mask`-truncated.
    #[test]
    fn derived_dividers_in_range() {
        let widths = [DataWidth::Bits16, DataWidth::Bits18, DataWidth::Bits20, DataWidth::Bits24, DataWidth::Bits32];
        let chans = [ChannelCount::Two, ChannelCount::Four, ChannelCount::Eight, ChannelCount::Sixteen];
        for &dw in &widths {
            for &ch in &chans {
                let (bclk, fs_num, fs_ratio) = derive_dividers(dw, ch);
                assert!(bclk >= 1 && bclk <= 0x7F, "bclk {bclk} out of 7-bit range for {dw:?}/{ch:?}");
                assert!(fs_num >= 1 && fs_num <= 0x3FF, "fs_num {fs_num} out of 10-bit range");
                assert!(fs_ratio >= 1 && fs_ratio <= 0x7FF, "fs_ratio {fs_ratio} out of 11-bit range");
            }
        }
    }

    // ── Interrupt mask/status encoding (operational, unchanged layout) ──

    fn encode_int_mask(rx: bool, tx: bool, rx_ovf: bool, tx_unf: bool) -> u32 {
        let mut val: u32 = 0;
        if rx {
            val |= 0x01;
        }
        if tx {
            val |= 0x02;
        }
        if rx_ovf {
            val |= 0x04;
        }
        if tx_unf {
            val |= 0x08;
        }
        val
    }
    fn decode_int_status(sts: u32) -> (bool, bool, bool, bool) {
        ((sts & 0x01) != 0, (sts & 0x02) != 0, (sts & 0x04) != 0, (sts & 0x08) != 0)
    }

    #[test]
    fn interrupt_mask_encoding() {
        assert_eq!(encode_int_mask(false, false, false, false), 0x00);
        assert_eq!(encode_int_mask(true, false, false, false), 0x01);
        assert_eq!(encode_int_mask(false, true, false, false), 0x02);
        assert_eq!(encode_int_mask(false, false, true, false), 0x04);
        assert_eq!(encode_int_mask(false, false, false, true), 0x08);
        assert_eq!(encode_int_mask(true, true, true, true), 0x0F);
    }

    #[test]
    fn interrupt_status_decode_roundtrips_mask() {
        for bits in 0u32..16 {
            let (rx, tx, ovf, unf) = decode_int_status(bits);
            assert_eq!(encode_int_mask(rx, tx, ovf, unf), bits);
        }
    }

    #[test]
    fn fifo_threshold_packing() {
        // tx threshold low byte, rx threshold second byte.
        let tx: u8 = 0x12;
        let rx: u8 = 0x34;
        let thresh = (tx as u32) | ((rx as u32) << 8);
        assert_eq!(thresh, 0x3412);
    }

    #[test]
    fn default_master_config_is_16bit_stereo_i2s() {
        let c = MasterConfig::default();
        assert_eq!(c.mode, I2sMode::I2s);
        assert_eq!(c.channels, ChannelCount::Two);
        assert_eq!(c.data_width, DataWidth::Bits16);
        assert!(!c.loopback);
    }
}

// ── Property-based fuzz tests ──────────────────────────────────

#[cfg(all(test, not(target_arch = "riscv32")))]
mod proptests {
    use proptest::prelude::*;

    proptest! {
        /// Fuzz: the FIFO-threshold packing keeps tx in the low byte and rx in the
        /// second byte with no cross-contamination, and is exactly recoverable.
        #[test]
        fn fifo_threshold_roundtrip(tx in any::<u8>(), rx in any::<u8>()) {
            let thresh = (tx as u32) | ((rx as u32) << 8);
            prop_assert_eq!((thresh & 0xFF) as u8, tx);
            prop_assert_eq!(((thresh >> 8) & 0xFF) as u8, rx);
            prop_assert_eq!(thresh & !0xFFFF, 0);
        }
    }
}
