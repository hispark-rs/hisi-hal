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
//! - the ANA/LDO sub-block @ `0x5703_63D0` (GADC base + 0x3D0, NOT in `bs2x-pac`):
//!   the AFE/ADC/VREF LDO power-up;
//! - the PMU AFE sub-block @ `0x5700_8700` (NOT in `bs2x-pac`): MTCMOS power /
//!   isolation / reset / clock and the `afe_gadc_cfg` enable handshake; plus an
//!   AON isolation bit @ `0x5702_C230[10]`.
//!
//! The sub-blocks that `bs2x-pac` does not model are reached via raw pointer
//! constants below (BS2X-correct addresses from `adc_porting.h`). The exact analog
//! timing/trim magic values are the SDK defaults; they are reproduced where they
//! matter for control flow and marked `TODO(bs2x)` where only silicon cares (the
//! QEMU GADC model returns a fixed sample regardless of the analog config, so this
//! driver's control path — power-up → channel select → trigger → poll done → read
//! — is what is exercised on `-M bs21/bs22/bs20`).

use crate::peripherals::Gadc as GadcPeriph;
use core::marker::PhantomData;

// ── Sub-block base addresses not covered by bs2x-pac's `gadc` block ──────────
const GADC_BASE: usize = 0x5703_6000; // GADC digital (also covers ANA at +0x3D0)
const PMU_AFE_BASE: usize = 0x5700_8700; // PMU AFE (power/iso/rst/clk + afe_gadc_cfg)
const AON_AFE_ISO: usize = 0x5702_C230; // AON: bit10 = AFE iso enable

// PMU AFE register offsets (from GADC base 0x5700_8700)
const PMU_AFE_ADC_RST_N: usize = 0x00;
const PMU_AFE_GADC_CFG: usize = 0x08; // [0]s2d_en [1]s2d_mux_en [2]s2d_iso_en [3]done? [4]done(RO)
const PMU_AFE_DIG_PWR_EN: usize = 0x20; // [0]pwr_en [1]iso_en [2]pwr_ack(RO)
const PMU_AFE_SOFT_RST: usize = 0x24;
const PMU_AFE_CLK_EN: usize = 0x28;
const PMU_AFE_ISO_CFG: usize = 0x30;

// ANA/LDO register offsets (GADC base + these; the ANA sub-block starts at +0x3D0)
const ANA_CFG_ANA_0: usize = 0x3D4; // vrefldo
const ANA_CFG_ANA_1: usize = 0x3D8; // bufp/bufn
const ANA_CFG_ANA_4: usize = 0x3E4; // adcldo
const ANA_CFG_ANA_5: usize = 0x3E8; // adcldo_en_dly
const ANA_CFG_ANA_6: usize = 0x3EC; // afeldo
const ANA_CFG_ANA_7: usize = 0x3F0; // afeldo_en_dly
const ANA_CFG_FREG_5: usize = 0x408;
const ANA_CFG_FREG_9: usize = 0x418;

// afe_gadc_cfg bits
const S2D_GADC_EN: u32 = 1 << 0;
const S2D_GADC_MUX_EN: u32 = 1 << 1;
const S2D_GADC_ISO_EN: u32 = 1 << 2;

#[inline]
unsafe fn rd(addr: usize) -> u32 {
    unsafe { core::ptr::read_volatile(addr as *const u32) }
}
#[inline]
unsafe fn wr(addr: usize, v: u32) {
    unsafe { core::ptr::write_volatile(addr as *mut u32, v) }
}
#[inline]
unsafe fn pmu_wr(off: usize, v: u32) {
    unsafe { wr(PMU_AFE_BASE + off, v) }
}
#[inline]
unsafe fn pmu_rmw(off: usize, clear: u32, set: u32) {
    unsafe {
        let v = (rd(PMU_AFE_BASE + off) & !clear) | set;
        wr(PMU_AFE_BASE + off, v);
    }
}

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
    Ain0 = 0,
    Ain1 = 1,
    Ain2 = 2,
    Ain3 = 3,
    Ain4 = 4,
    Ain5 = 5,
    Ain6 = 6,
    Ain7 = 7,
}

const VSSAFE1_BIT: u32 = 9; // n-side reference channel (cfg_amux_1 amuxn)

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

    /// Power up the GADC and bring it live. Mirrors `hal_adc_v153_init` ->
    /// `_power_on` -> `_enable` -> `hal_gafe_enable` (fbb_bs2x).
    pub fn new(_gadc: GadcPeriph<'d>) -> Self {
        let r = unsafe { &*GadcPeriph::ptr() };
        unsafe {
            // Phase A — block bring-up.
            wr(AON_AFE_ISO, rd(AON_AFE_ISO) | (1 << 10)); // AFE iso enable (AON)
            pmu_wr(PMU_AFE_ISO_CFG, 0); // release XO32M iso
            delay_us(30);
            pmu_wr(PMU_AFE_DIG_PWR_EN, 0x3); // MTCMOS power-on (pwr_en|iso_en)
            delay_us(50);
            pmu_rmw(PMU_AFE_DIG_PWR_EN, 1 << 1, 0); // release iso
            pmu_wr(PMU_AFE_ADC_RST_N, 1); // ana rstn release
            pmu_wr(PMU_AFE_CLK_EN, 1); // dig clk release
            pmu_wr(PMU_AFE_SOFT_RST, 1); // dig apb rstn release
            r.cfg_clken().write(|w| w.bits(0));
            r.cfg_rstn().write(|w| w.bits(0));
            r.cfg_clken().write(|w| w.bits(0x0011_1111)); // tst/bc/fc/byp/prechg/ctrl clocks on

            // Phase B — LDO power-on (ANA sub-block). TODO(bs2x): efuse LDO trim skipped.
            wr(GADC_BASE + ANA_CFG_ANA_6, 0x1); // afeldo
            delay_us(150);
            wr(GADC_BASE + ANA_CFG_ANA_7, rd(GADC_BASE + ANA_CFG_ANA_7) | 0x1);
            wr(GADC_BASE + ANA_CFG_ANA_4, 0x1); // adcldo
            delay_us(150);
            wr(GADC_BASE + ANA_CFG_ANA_5, rd(GADC_BASE + ANA_CFG_ANA_5) | 0x1);
            wr(GADC_BASE + ANA_CFG_ANA_0, rd(GADC_BASE + ANA_CFG_ANA_0) | 0x1); // vrefldo_en
            delay_us(150);

            // Phase C — common + GADC analog enable.
            r.cfg_clk_div_0().write(|w| w.bits(0x27)); // TODO(bs2x): COMMON_DEFAULT clk div
            wr(GADC_BASE + ANA_CFG_ANA_1, rd(GADC_BASE + ANA_CFG_ANA_1) | 0x3); // bufp|bufn
            wr(GADC_BASE + ANA_CFG_FREG_5, 0);
            wr(GADC_BASE + ANA_CFG_FREG_9, 0x2);
            r.cfg_tst_1().write(|w| w.bits(2)); // ACCUMULATED_AVERAGE_OUTPUT diag node
            r.cfg_rstn().write(|w| w.bits(0x0001_1111)); // release all GADC resets
            r.cfg_iso().write(|w| w.bits(0));

            // hal_gafe_enable: afe_gadc_cfg power-up handshake.
            pmu_wr(PMU_AFE_GADC_CFG, S2D_GADC_ISO_EN | S2D_GADC_MUX_EN);
            delay_us(30);
            pmu_rmw(PMU_AFE_GADC_CFG, S2D_GADC_ISO_EN, 0);
            delay_us(30);
            pmu_rmw(PMU_AFE_GADC_CFG, 0, S2D_GADC_EN);
        }
        let _ = r;
        Self { _gadc: PhantomData, first_sample: true }
    }

    /// Select `channel`, run one conversion, and return the raw 18-bit signed
    /// accumulated sample (`rpt_gadc_data_2`, sign-extended at bit 17). Mirrors
    /// `hal_adc_v153_auto_sample`.
    pub fn read(&mut self, channel: AdcChannel) -> i32 {
        let r = self.regs();
        // Channel select: p-side one-hot = BIT(channel), n-side = BIT(VSSAFE1),
        // both divide-disable. cfg_amux_1: amuxn[10:0], amuxp[22:12].
        let amuxp = 1u32 << (channel as u32);
        let amuxn = 1u32 << VSSAFE1_BIT;
        let v = (amuxn & 0x7FF) | (1 << 11) | ((amuxp & 0x7FF) << 12) | (1 << 23);
        unsafe {
            r.cfg_amux_1().write(|w| w.bits(v));
            r.cfg_amux_2().write(|w| w.bits(0));
        }

        // The very first sample after power-up is discarded.
        if self.first_sample {
            self.convert_once();
            self.first_sample = false;
        }
        self.convert_once()
    }

    /// Trigger one conversion, poll done, read + sign-extend the result.
    fn convert_once(&self) -> i32 {
        let r = self.regs();
        unsafe {
            // Trigger (hal_gadc_iso_on): un-isolate + enable -> free-running scan.
            delay_us(5);
            pmu_rmw(PMU_AFE_GADC_CFG, 0, S2D_GADC_MUX_EN);
            pmu_rmw(PMU_AFE_GADC_CFG, S2D_GADC_ISO_EN, 0);
            delay_us(5);
            pmu_rmw(PMU_AFE_GADC_CFG, 0, S2D_GADC_EN);

            // Poll done: rpt_gadc_data_3 bit0 (GADC block).
            while r.rpt_gadc_data_3().read().bits() & 0x1 == 0 {
                core::hint::spin_loop();
            }

            // Stop / re-isolate (hal_gadc_iso_off).
            pmu_rmw(PMU_AFE_GADC_CFG, S2D_GADC_EN, 0);
            delay_us(5);
            pmu_rmw(PMU_AFE_GADC_CFG, S2D_GADC_MUX_EN, S2D_GADC_ISO_EN);

            // Read result + sign-extend (18-bit signed, sign bit 17).
            let raw = r.rpt_gadc_data_2().read().bits();
            sign_extend18(raw)
        }
    }
}

/// Sign-extend an 18-bit GADC sample to `i32` (sign bit = 17).
#[inline]
fn sign_extend18(raw: u32) -> i32 {
    let v = raw & 0x3FFFF;
    if v & (1 << 17) != 0 { (v as i32) - 0x4_0000 } else { v as i32 }
}
