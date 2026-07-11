//! WS63 RF analog-power and clock preparation.
//!
//! The mask ROM and vendor Wi-Fi objects own calibration, FRW dispatch, and the
//! radio state machines. They do not perform the application-level power-up
//! sequence: the vendor SDK calls `open_rf_power()` and the RF/Wi-Fi portion of
//! `switch_clock()` from `hw_init()` before entering the Wi-Fi stack. This
//! module provides that narrow missing board-independent step through PAC fields.

use embedded_hal::delay::DelayNs;

use crate::peripherals::{CldoCrg, Cmu};

const XO_TRIM_REVERSE_SELECT: u32 = 0x3_0000;
const XO_TRIM_COARSE: u32 = 0x3_083c;
const LDO_ENABLE: u16 = 0x4070;
const LDO_ENABLE_DELAYED: u16 = 0x6070;
const PLL_LDO_ENABLE: u16 = 0x4080;
const PLL_LDO_ENABLE_DELAYED: u16 = 0x6080;
const VCO_LDO_ENABLE: u16 = 0x4000;
const VCO_LDO_ENABLE_DELAYED: u16 = 0x6000;
const TX_LDO_ENABLE: u16 = 0x4030;
const TX_LDO_ENABLE_DELAYED: u16 = 0x6030;
const RELEASE_ADDA_ISOLATION: u8 = 0x03;
const ENABLE_RF_PLL_REFERENCE: u8 = 0x05;

/// Exclusive controller for the WS63 application-owned RF power sequence.
///
/// Construction consumes the CMU and CLDO_CRG singleton tokens, preventing a
/// second context from changing the same analog-power and clock registers while
/// the sequence is running. [`enable`](Self::enable) returns CLDO_CRG so it can
/// subsequently be moved into [`crate::System`].
pub struct RfPower<'d> {
    _cmu: Cmu<'d>,
    cldo_crg: CldoCrg<'d>,
}

impl<'d> RfPower<'d> {
    /// Create the RF power controller from its exclusive peripheral tokens.
    pub fn new(cmu: Cmu<'d>, cldo_crg: CldoCrg<'d>) -> Self {
        Self { _cmu: cmu, cldo_crg }
    }

    /// Run the vendor-attested analog-power sequence and select PLL radio clocks.
    ///
    /// This reproduces only the RF/Wi-Fi operations from the SDK's
    /// `open_rf_power()` and `switch_clock()`. UART, TRNG, and temperature-sensor
    /// clock setup is deliberately left untouched. Calibration and all runtime
    /// radio logic remain in the mask ROM/vendor objects.
    pub fn enable(self, delay: &mut impl DelayNs) -> CldoCrg<'d> {
        let cmu = unsafe { &*Cmu::ptr() };
        let cldo = unsafe { &*CldoCrg::ptr() };

        cmu.xo_signal().write(|w| unsafe { w.control().bits(XO_TRIM_REVERSE_SELECT) });
        cmu.xo_signal().write(|w| unsafe { w.control().bits(XO_TRIM_COARSE) });

        cmu.rf_rx_ldo().write(|w| unsafe { w.control().bits(LDO_ENABLE) });
        delay.delay_us(120);
        cmu.rf_rx_ldo().write(|w| unsafe { w.control().bits(LDO_ENABLE_DELAYED) });
        delay.delay_us(10);

        cmu.rf_pll_ldo().write(|w| unsafe { w.control().bits(PLL_LDO_ENABLE) });
        delay.delay_us(120);
        cmu.rf_pll_ldo().write(|w| unsafe { w.control().bits(PLL_LDO_ENABLE_DELAYED) });
        delay.delay_us(10);

        cmu.rf_lna_ldo().write(|w| unsafe { w.control().bits(LDO_ENABLE) });
        delay.delay_us(120);
        cmu.rf_lna_ldo().write(|w| unsafe { w.control().bits(LDO_ENABLE_DELAYED) });
        delay.delay_us(10);

        cmu.rf_vco_ldo().write(|w| unsafe { w.control().bits(VCO_LDO_ENABLE) });
        delay.delay_us(120);
        cmu.rf_vco_ldo().write(|w| unsafe { w.control().bits(VCO_LDO_ENABLE_DELAYED) });
        delay.delay_us(10);

        cmu.rf_tx_ldo().write(|w| unsafe { w.control().bits(TX_LDO_ENABLE) });
        delay.delay_us(120);
        cmu.rf_tx_ldo().write(|w| unsafe { w.control().bits(TX_LDO_ENABLE_DELAYED) });
        delay.delay_us(10);

        cmu.rf_abb_ldo().write(|w| unsafe { w.control().bits(LDO_ENABLE) });
        delay.delay_us(120);
        cmu.rf_abb_ldo().write(|w| unsafe { w.control().bits(LDO_ENABLE_DELAYED) });
        cmu.rf_adda_iso().write(|w| unsafe { w.control().bits(RELEASE_ADDA_ISOLATION) });

        cmu.rf_pll_reference()
            .write(|w| unsafe { w.control().bits(ENABLE_RF_PLL_REFERENCE) });
        delay.delay_us(10);

        cldo.clk_sel().modify(|_, w| w.wifi_mac_clk_sel().set_bit());
        let wifi_gates = cldo.cken_ctl1().read().wifi_cken().bits();
        cldo
            .cken_ctl1()
            .modify(|_, w| unsafe { w.wifi_cken().bits(wifi_gates & !0x01) });
        delay.delay_us(1);
        cldo.clk_sel().modify(|_, w| w.wifi_phy_clk_sel().set_bit());
        delay.delay_us(1);
        cldo
            .cken_ctl1()
            .modify(|_, w| unsafe { w.wifi_cken().bits(wifi_gates | 0x01) });
        cldo.clk_sel().modify(|_, w| w.rf_ctl_clk_sel().set_bit());

        self.cldo_crg
    }
}
