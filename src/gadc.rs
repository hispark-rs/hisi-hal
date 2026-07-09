//! BS2X GADC — 13-bit general-purpose ADC (IP version v153).
//!
//! BS2X-only (`chip-bs21`). The WS63 has a different ADC (LSADC v154, the `lsadc`
//! module); this is a ground-up driver for the BS2X GADC, whose register layout
//! shares nothing with the LSADC.
//!
//! A single conversion touches **three** hardware blocks (from the fbb_bs2x
//! `hal_adc_v153` driver, the sole ground truth):
//! - the GADC digital block @ `0x5703_6000` (in `bs2x-pac` as `Gadc`): reset/clock,
//!   channel select (`cfg_amux_1`), the done flag (`rpt_gadc_data_3` bit0) and the
//!   result (`rpt_gadc_data_2`);
//! - the ANA/LDO sub-block @ `0x5703_63D0` (GADC base + 0x3D0, modeled in the
//!   same `Gadc` PAC register block): the AFE/ADC/VREF LDO power-up;
//! - the PMU AFE sub-block @ `0x5700_8700` (in `bs2x-pac` as `AdcPmuAfe`):
//!   MTCMOS power / isolation / reset / clock and the `afe_gadc_cfg` enable
//!   handshake; plus `AonAfe.afe_iso[10]` @ `0x5702_C230`.
//!
//! The exact analog timing/trim magic values are the SDK defaults; they are
//! reproduced where they matter for control flow and marked `TODO(bs2x)` where
//! only silicon cares (the QEMU GADC model returns a fixed sample regardless of
//! the analog config, so this driver's control path — power-up → channel select
//! → trigger → poll done → read — is what is exercised on `-M bs21/bs22/bs20`).

use crate::peripherals::Gadc as GadcPeriph;
use core::marker::PhantomData;

/// Rough microsecond busy-wait (the analog settling delays). Sized for the 64 MHz
/// BS2X app core; only matters on silicon (QEMU ignores it).
#[inline]
fn delay_us(us: u32) {
    let cycles = us.saturating_mul(64);
    for _ in 0..cycles {
        core::hint::spin_loop();
    }
}

/// GADC input channels (AIN0..AIN7, single-ended; n-side tied to VSSAFE1).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum AdcChannel {
    /// Analog input 0 (p-side one-hot `BIT(0)` in `cfg_amux_1`).
    Ain0 = 0,
    /// Analog input 1 (p-side one-hot `BIT(1)` in `cfg_amux_1`).
    Ain1 = 1,
    /// Analog input 2 (p-side one-hot `BIT(2)` in `cfg_amux_1`).
    Ain2 = 2,
    /// Analog input 3 (p-side one-hot `BIT(3)` in `cfg_amux_1`).
    Ain3 = 3,
    /// Analog input 4 (p-side one-hot `BIT(4)` in `cfg_amux_1`).
    Ain4 = 4,
    /// Analog input 5 (p-side one-hot `BIT(5)` in `cfg_amux_1`).
    Ain5 = 5,
    /// Analog input 6 (p-side one-hot `BIT(6)` in `cfg_amux_1`).
    Ain6 = 6,
    /// Analog input 7 (p-side one-hot `BIT(7)` in `cfg_amux_1`).
    Ain7 = 7,
}

const VSSAFE1_BIT: u32 = 9; // n-side reference channel (cfg_amux_1 amuxn)

/// Bounded spin limit for the conversion-done poll (`rpt_gadc_data_3` bit 0). A
/// single GADC conversion latches within microseconds at the SDK clock divider;
/// this cap is several orders above that so a wedged AFE/LDO (e.g. analog never
/// powered) yields [`GadcError::ConversionTimeout`] instead of hanging forever.
const GADC_DONE_POLL_LIMIT: u32 = 1_000_000;

/// GADC conversion error.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum GadcError {
    /// The conversion-done flag did not assert within `GADC_DONE_POLL_LIMIT`
    /// spins — the analog front-end is unpowered or the trigger handshake stalled.
    ConversionTimeout,
}

/// The BS2X GADC driver (single instance).
pub struct Gadc<'d> {
    _gadc: PhantomData<GadcPeriph<'d>>,
    first_sample: bool,
}

impl<'d> Gadc<'d> {
    fn regs(&self) -> &'static crate::soc::pac::gadc::RegisterBlock {
        // SAFETY: static physical MMIO address from bs2x-pac.
        unsafe { &*GadcPeriph::ptr() }
    }

    fn pmu_regs(&self) -> &'static crate::soc::pac::adc_pmu_afe::RegisterBlock {
        // SAFETY: static physical MMIO address from bs2x-pac.
        unsafe { &*crate::soc::pac::AdcPmuAfe::ptr() }
    }

    fn aon_afe_regs(&self) -> &'static crate::soc::pac::aon_afe::RegisterBlock {
        // SAFETY: static physical MMIO address from bs2x-pac.
        unsafe { &*crate::soc::pac::AonAfe::ptr() }
    }

    fn write_gadc_cfg(
        pmu: &'static crate::soc::pac::adc_pmu_afe::RegisterBlock,
        en: bool,
        mux_en: bool,
        iso_en: bool,
    ) {
        pmu.afe_gadc_cfg().write(|w| {
            let w = if en { w.s2d_gadc_en().set_bit() } else { w.s2d_gadc_en().clear_bit() };
            let w = if mux_en {
                w.s2d_gadc_mux_en().set_bit()
            } else {
                w.s2d_gadc_mux_en().clear_bit()
            };
            if iso_en { w.s2d_gadc_iso_en().set_bit() } else { w.s2d_gadc_iso_en().clear_bit() }
        });
    }

    /// Power up the GADC and bring it live. Mirrors `hal_adc_v153_init` ->
    /// `_power_on` -> `_enable` -> `hal_gafe_enable` (fbb_bs2x).
    ///
    /// # Preconditions (analog)
    ///
    /// This drives the GADC digital block, the ANA/LDO sub-block, and the PMU AFE
    /// (MTCMOS power / isolation / reset / clock) through the vendor power-up
    /// sequence with the SDK's settling delays. It assumes the **board's analog
    /// supply and 32 MHz reference are up** (the AON `XO32M` path); the efuse LDO
    /// trim is skipped (`TODO(bs2x)`). Without a live AFE, a conversion never
    /// completes — [`Gadc::read`] then returns [`GadcError::ConversionTimeout`]
    /// rather than hanging. Not validated on silicon (QEMU returns a fixed sample).
    pub fn new(_gadc: GadcPeriph<'d>) -> Self {
        let this = Self { _gadc: PhantomData, first_sample: true };
        let r = this.regs();
        let pmu = this.pmu_regs();
        let aon = this.aon_afe_regs();
        // Phase A — block bring-up.
        aon.afe_iso().modify(|_, w| w.afe_iso_en().set_bit()); // AFE iso enable (AON)
        pmu.afe_iso_cfg().write(|w| unsafe { w.bits(0) }); // release XO32M iso
        delay_us(30);
        // MTCMOS power-on (pwr_en|iso_en).
        pmu.afe_dig_pwr_en().write(|w| w.afe_pwr_en().set_bit().afe_iso_en().set_bit());
        delay_us(50);
        pmu.afe_dig_pwr_en().modify(|_, w| w.afe_iso_en().clear_bit()); // release iso
        pmu.afe_adc_rst_n().write(|w| w.afe_adc_rst_n().set_bit()); // ana rstn release
        pmu.afe_clk_en().write(|w| w.afe_clk_en().set_bit()); // dig clk release
        pmu.afe_soft_rst().write(|w| w.afe_soft_rst().set_bit()); // dig apb rstn release
        r.cfg_clken().reset();
        r.cfg_rstn().reset();
        r.cfg_clken().write(|w| {
            w.cfg_clken_tst()
                .set_bit()
                .cfg_gadc_clken_bc()
                .set_bit()
                .cfg_gadc_clken_fc()
                .set_bit()
                .cfg_gadc_clken_byp()
                .set_bit()
                .cfg_gadc_clken_prechg()
                .set_bit()
                .cfg_gadc_clken_ctrl()
                .set_bit()
        }); // tst/bc/fc/byp/prechg/ctrl clocks on

        // Phase B — LDO power-on (ANA sub-block). TODO(bs2x): efuse LDO trim skipped.
        r.cfg_ana_6().write(|w| w.cfg_afe_afeldo_en().set_bit()); // afeldo
        delay_us(150);
        r.cfg_ana_7().modify(|_, w| w.cfg_afe_afeldo_en_dly().set_bit());
        r.cfg_ana_4().write(|w| w.cfg_afe_adcldo_en().set_bit()); // adcldo
        delay_us(150);
        r.cfg_ana_5().modify(|_, w| w.cfg_afe_adcldo_en_dly().set_bit());
        r.cfg_ana_0().modify(|_, w| w.cfg_vrefldo_en().set_bit()); // vrefldo_en
        delay_us(150);

        // Phase C — common + GADC analog enable.
        r.cfg_clk_div_0().write(|w| unsafe { w.cfg_adc_ana_div_th().bits(0x27) }); // TODO(bs2x): COMMON_DEFAULT clk div
        r.cfg_ana_1().modify(|_, w| w.cfg_bufp_en().set_bit().cfg_bufn_en().set_bit());
        r.cfg_freg_5().reset();
        r.cfg_freg_9().write(|w| unsafe { w.bits(2) });
        r.cfg_tst_1().write(|w| unsafe { w.diag_node().bits(2) }); // ACCUMULATED_AVERAGE_OUTPUT diag node
        r.cfg_rstn().write(|w| {
            w.cfg_rstn_tst()
                .set_bit()
                .cfg_gadc_rstn_bc()
                .set_bit()
                .cfg_gadc_rstn_fc()
                .set_bit()
                .cfg_gadc_rstn_data()
                .set_bit()
                .cfg_gadc_rstn_ana()
                .set_bit()
        }); // release all GADC resets
        r.cfg_iso().write(|w| unsafe { w.bits(0) });

        // hal_gafe_enable: afe_gadc_cfg power-up handshake.
        Self::write_gadc_cfg(pmu, false, true, true);
        delay_us(30);
        Self::write_gadc_cfg(pmu, false, true, false);
        delay_us(30);
        Self::write_gadc_cfg(pmu, true, true, false);
        this
    }

    /// Select `channel`, run one conversion, and return the raw 18-bit signed
    /// accumulated sample (`rpt_gadc_data_2`, sign-extended at bit 17). Mirrors
    /// `hal_adc_v153_auto_sample`.
    ///
    /// # Errors
    ///
    /// [`GadcError::ConversionTimeout`] if the conversion-done flag never asserts
    /// within the bounded poll (an unpowered AFE/LDO) — instead of spinning forever.
    pub fn read(&mut self, channel: AdcChannel) -> Result<i32, GadcError> {
        let r = self.regs();
        // Channel select: p-side one-hot = BIT(channel), n-side = BIT(VSSAFE1),
        // both divide-disable. cfg_amux_1: amuxn[10:0], amuxp[22:12].
        let amuxp = 1u16 << (channel as u16);
        let amuxn = 1u16 << VSSAFE1_BIT;
        r.cfg_amux_1().write(|w| unsafe {
            w.amuxn_sensor_ch_sel()
                .bits(amuxn)
                .amuxn_devide_disable()
                .set_bit()
                .amuxp_sensor_ch_sel()
                .bits(amuxp)
                .amuxp_devide_disable()
                .set_bit()
        });
        r.cfg_amux_2().reset();

        // The very first sample after power-up is discarded.
        if self.first_sample {
            self.convert_once()?;
            self.first_sample = false;
        }
        self.convert_once()
    }

    /// Trigger one conversion, poll done (bounded), read + sign-extend the result.
    fn convert_once(&self) -> Result<i32, GadcError> {
        let r = self.regs();
        let pmu = self.pmu_regs();
        // Trigger (hal_gadc_iso_on): un-isolate + enable -> free-running scan.
        delay_us(5);
        pmu.afe_gadc_cfg().modify(|_, w| w.s2d_gadc_mux_en().set_bit());
        pmu.afe_gadc_cfg().modify(|_, w| w.s2d_gadc_iso_en().clear_bit());
        delay_us(5);
        pmu.afe_gadc_cfg().modify(|_, w| w.s2d_gadc_en().set_bit());

        // Poll done: rpt_gadc_data_3 bit0 (GADC block). Bounded so an unpowered
        // AFE returns ConversionTimeout instead of hanging.
        let mut done = false;
        for _ in 0..GADC_DONE_POLL_LIMIT {
            if r.rpt_gadc_data_3().read().single_sample_done().bit_is_set() {
                done = true;
                break;
            }
            core::hint::spin_loop();
        }

        // Stop / re-isolate (hal_gadc_iso_off) — always, even on timeout, so the
        // AFE is left isolated rather than free-running.
        pmu.afe_gadc_cfg().modify(|_, w| w.s2d_gadc_en().clear_bit());
        delay_us(5);
        pmu.afe_gadc_cfg().modify(|_, w| {
            w.s2d_gadc_mux_en().clear_bit();
            w.s2d_gadc_iso_en().set_bit()
        });

        if !done {
            return Err(GadcError::ConversionTimeout);
        }

        // Read result + sign-extend (18-bit signed, sign bit 17).
        let raw = r.rpt_gadc_data_2().read().sample_data().bits();
        Ok(sign_extend18(raw))
    }
}

/// Sign-extend an 18-bit GADC sample to `i32` (sign bit = 17).
#[inline]
fn sign_extend18(raw: u32) -> i32 {
    let v = raw & 0x3FFFF;
    if v & (1 << 17) != 0 { (v as i32) - 0x4_0000 } else { v as i32 }
}

// ── Tests ──────────────────────────────────────────────────────

#[cfg(all(test, not(target_arch = "riscv32")))]
mod tests {
    use super::*;

    /// Re-derivation of the `cfg_amux_1` packing inlined in `Gadc::read`: p-side
    /// one-hot `BIT(channel)`, n-side one-hot `BIT(VSSAFE1)`, both divide-disable.
    /// Layout: amuxn[10:0], div-n bit11, amuxp[22:12], div-p bit23.
    fn amux_1(channel: AdcChannel) -> u32 {
        let amuxp = 1u32 << (channel as u32);
        let amuxn = 1u32 << VSSAFE1_BIT;
        (amuxn & 0x7FF) | (1 << 11) | ((amuxp & 0x7FF) << 12) | (1 << 23)
    }

    #[test]
    fn channel_discriminants_are_index() {
        // AdcChannel is repr(u8) numbered 0..=7; `channel as u32` is the AIN index.
        assert_eq!(AdcChannel::Ain0 as u32, 0);
        assert_eq!(AdcChannel::Ain3 as u32, 3);
        assert_eq!(AdcChannel::Ain7 as u32, 7);
    }

    #[test]
    fn sign_extend_positive_passthrough() {
        // Values with bit17 clear are non-negative and unchanged.
        assert_eq!(sign_extend18(0), 0);
        assert_eq!(sign_extend18(1), 1);
        // 0x1FFFF = largest positive 18-bit sample (bit17 clear) = 131071.
        assert_eq!(sign_extend18(0x1FFFF), 131_071);
    }

    #[test]
    fn sign_extend_negative_values() {
        // bit17 set → negative. 0x20000 is the most-negative sample (-2^17).
        assert_eq!(sign_extend18(0x2_0000), -131_072);
        // 0x3FFFF (all 18 bits set) = -1.
        assert_eq!(sign_extend18(0x3_FFFF), -1);
        // 0x3FFFE = -2.
        assert_eq!(sign_extend18(0x3_FFFE), -2);
    }

    #[test]
    fn sign_extend_ignores_high_bits() {
        // Only bits [17:0] are significant; junk above bit17 is masked off.
        assert_eq!(sign_extend18(0xFFFC_0000), 0);
        // High junk plus the bit17 sign + low bits round-trips to the masked value.
        assert_eq!(sign_extend18(0xFFFF_FFFF), -1);
        assert_eq!(sign_extend18(0xDEAD_0000 | 0x1FFFF), 131_071);
    }

    #[test]
    fn sign_extend_range_is_18bit_signed() {
        // The output is always within the signed 18-bit range [-2^17, 2^17).
        for raw in [0u32, 0x1, 0x1_FFFF, 0x2_0000, 0x3_FFFF, 0xFFFF_FFFF] {
            let v = sign_extend18(raw);
            assert!((-131_072..=131_071).contains(&v), "raw={raw:#x} -> {v}");
        }
    }

    #[test]
    fn amux_n_side_fixed_to_vssafe1() {
        // The n-side is always BIT(VSSAFE1)=BIT(9) regardless of channel, and both
        // divide-disable bits (11 and 23) are always set.
        for ch in [AdcChannel::Ain0, AdcChannel::Ain4, AdcChannel::Ain7] {
            let v = amux_1(ch);
            assert_eq!(v & 0x7FF, 1 << VSSAFE1_BIT, "amuxn for {ch:?}");
            assert_ne!(v & (1 << 11), 0, "div-n bit for {ch:?}");
            assert_ne!(v & (1 << 23), 0, "div-p bit for {ch:?}");
        }
    }

    #[test]
    fn amux_p_side_is_channel_onehot() {
        // The p-side field [22:12] is a one-hot of the selected channel index.
        assert_eq!((amux_1(AdcChannel::Ain0) >> 12) & 0x7FF, 1 << 0);
        assert_eq!((amux_1(AdcChannel::Ain5) >> 12) & 0x7FF, 1 << 5);
        assert_eq!((amux_1(AdcChannel::Ain7) >> 12) & 0x7FF, 1 << 7);
    }

    #[test]
    fn amux_known_value_ain0() {
        // Exact word for AIN0: amuxn=BIT(9), div-n=BIT(11), amuxp=BIT(12), div-p=BIT(23).
        let expected = (1 << 9) | (1 << 11) | (1 << 12) | (1 << 23);
        assert_eq!(amux_1(AdcChannel::Ain0), expected);
    }

    #[test]
    fn delay_us_saturates() {
        // delay_us computes a cycle count `us * 64` with a saturating multiply, so
        // a huge µs request clamps to u32::MAX rather than wrapping.
        assert_eq!(0u32.saturating_mul(64), 0);
        assert_eq!(1u32.saturating_mul(64), 64);
        assert_eq!(u32::MAX.saturating_mul(64), u32::MAX);
    }
}

// ── Property-based fuzz tests ──────────────────────────────────

#[cfg(all(test, not(target_arch = "riscv32")))]
mod proptests {
    use super::*;
    use proptest::prelude::*;

    proptest! {
        /// Fuzz: sign_extend18 output is always within the signed 18-bit range and
        /// depends only on the low 18 bits of the input (high bits are masked).
        #[test]
        fn sign_extend_in_range_and_masked(raw in any::<u32>()) {
            let v = sign_extend18(raw);
            prop_assert!((-131_072..=131_071).contains(&v));
            // Masking the input to 18 bits gives the identical result.
            prop_assert_eq!(v, sign_extend18(raw & 0x3FFFF));
        }

        /// Fuzz: re-encoding the sign-extended value back to 18 bits round-trips
        /// to the masked raw (two's-complement is bijective over 18 bits).
        #[test]
        fn sign_extend_roundtrips(raw in any::<u32>()) {
            let v = sign_extend18(raw);
            let reencoded = (v as u32) & 0x3FFFF;
            prop_assert_eq!(reencoded, raw & 0x3FFFF);
        }

        /// Fuzz: the delay_us cycle count saturates and never panics for any input.
        #[test]
        fn delay_cycles_never_panic(us in any::<u32>()) {
            let cycles = us.saturating_mul(64);
            prop_assert!(cycles <= u32::MAX);
            // Saturation: result is either the exact product (as u64) or u32::MAX.
            let exact = us as u64 * 64;
            if exact <= u32::MAX as u64 {
                prop_assert_eq!(cycles as u64, exact);
            } else {
                prop_assert_eq!(cycles, u32::MAX);
            }
        }
    }
}
