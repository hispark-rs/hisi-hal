//! WS63 RF analog-power and clock preparation.
//!
//! The mask ROM and vendor Wi-Fi objects own calibration, FRW dispatch, and the
//! radio state machines. They do not perform the application-level power-up
//! sequence: the vendor SDK calls `open_rf_power()` and the RF/Wi-Fi portion of
//! `switch_clock()` from `hw_init()` before entering the Wi-Fi stack. This
//! module provides that narrow missing board-independent step through PAC fields.

use embedded_hal::delay::DelayNs;

use crate::efuse::{EfuseByteAddress, EfuseDriver};
use crate::peripherals::{CldoCrg, Cmu, Efuse};

const DEFAULT_XO_TRIM_FINE: u8 = 0x3c;
const DEFAULT_XO_TRIM_COARSE: u8 = 0x08;
const XO_TRIM_IDS: [u16; 3] = [144, 162, 180];
const XO_TRIM_LOCK_IDS: [u16; 3] = [160, 178, 196];
const XO_TRIM_LOCK_BIT: u8 = 7;
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

/// Factory XO trim selected from the WS63 eFuse redundancy groups.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FactoryXoTrim {
    /// No programmed group was locked; the SDK default remains active.
    Default,
    /// The newest locked group supplied a board-specific trim code.
    Applied {
        /// One-based redundancy group number (1 through 3).
        group: u8,
        /// Fine trim field written to `CMU_XO_SIG[7:0]`.
        fine: u8,
        /// Coarse trim field written to `CMU_XO_SIG[11:8]`.
        coarse: u8,
    },
}

/// RF-ready resources after power, clock selection, and factory XO trim.
pub struct RfReady<'d> {
    cldo_crg: CldoCrg<'d>,
    efuse: Efuse<'d>,
    factory_xo_trim: FactoryXoTrim,
}

impl<'d> RfReady<'d> {
    /// Report whether a programmed factory trim group was applied.
    pub fn factory_xo_trim(&self) -> FactoryXoTrim {
        self.factory_xo_trim
    }

    /// Return resources needed by the system-clock and Wi-Fi adapters.
    pub fn into_parts(self) -> (CldoCrg<'d>, Efuse<'d>) {
        (self.cldo_crg, self.efuse)
    }
}

impl<'d> RfPower<'d> {
    /// Create the RF power controller from its exclusive peripheral tokens.
    pub fn new(cmu: Cmu<'d>, cldo_crg: CldoCrg<'d>) -> Self {
        Self { _cmu: cmu, cldo_crg }
    }

    /// Run the vendor-attested power/clock sequence and apply factory XO trim.
    ///
    /// This reproduces only the RF/Wi-Fi operations from the SDK's
    /// `open_rf_power()`, the RF portion of `switch_clock()`, and
    /// `cmu_xo_trim_init()`. UART, TRNG, and temperature-sensor clock setup is
    /// deliberately left untouched. Calibration and all runtime radio logic
    /// remain in the mask ROM/vendor objects.
    pub fn enable(self, efuse: Efuse<'d>, delay: &mut impl DelayNs) -> RfReady<'d> {
        let cmu = unsafe { &*Cmu::ptr() };
        let cldo = unsafe { &*CldoCrg::ptr() };

        cmu.xo_signal().write(|w| {
            w.trim_fine_select().set_bit();
            w.trim_coarse_select().set_bit()
        });
        cmu.xo_signal().write(|w| {
            unsafe {
                w.trim_fine().bits(DEFAULT_XO_TRIM_FINE);
                w.trim_coarse().bits(DEFAULT_XO_TRIM_COARSE);
            }
            w.trim_fine_select().set_bit();
            w.trim_coarse_select().set_bit()
        });

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

        let mut efuse = EfuseDriver::new(efuse);
        let factory_xo_trim = read_factory_xo_trim(&mut efuse);
        if let FactoryXoTrim::Applied { fine, coarse, .. } = factory_xo_trim {
            cmu.xo_signal().modify(|_, w| {
                unsafe {
                    w.trim_fine().bits(fine);
                    w.trim_coarse().bits(coarse);
                }
                w.trim_fine_select().set_bit();
                w.trim_coarse_select().set_bit()
            });
        }

        RfReady {
            cldo_crg: self.cldo_crg,
            efuse: efuse.into_inner(),
            factory_xo_trim,
        }
    }
}

fn read_factory_xo_trim(efuse: &mut EfuseDriver<'_>) -> FactoryXoTrim {
    for index in (0..XO_TRIM_IDS.len()).rev() {
        let lock_addr = EfuseByteAddress::from_byte(XO_TRIM_LOCK_IDS[index])
            .expect("factory XO lock address is inside the WS63 eFuse array");
        if efuse.read_byte(lock_addr) & (1 << XO_TRIM_LOCK_BIT) == 0 {
            continue;
        }

        let trim_addr = EfuseByteAddress::from_byte(XO_TRIM_IDS[index])
            .expect("factory XO trim address is inside the WS63 eFuse array");
        let mut bytes = [0; 2];
        efuse
            .read_buffer(trim_addr, &mut bytes)
            .expect("factory XO trim occupies two in-range eFuse bytes");
        return decode_factory_xo_trim(index as u8 + 1, bytes);
    }

    FactoryXoTrim::Default
}

const fn decode_factory_xo_trim(group: u8, bytes: [u8; 2]) -> FactoryXoTrim {
    let value = u16::from_le_bytes(bytes) & 0x0fff;
    FactoryXoTrim::Applied {
        group,
        fine: value as u8,
        coarse: ((value >> 8) & 0x0f) as u8,
    }
}

#[cfg(all(test, not(target_arch = "riscv32")))]
mod tests {
    use super::{FactoryXoTrim, decode_factory_xo_trim};

    #[test]
    fn decodes_vendor_little_endian_12_bit_trim() {
        assert_eq!(
            decode_factory_xo_trim(3, [0xbc, 0xfa]),
            FactoryXoTrim::Applied {
                group: 3,
                fine: 0xbc,
                coarse: 0x0a,
            }
        );
    }
}
