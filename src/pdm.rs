//! BS2X PDM — PDM-microphone audio front-end (IP v150).
//!
//! BS2X-only (`chip-bs21`); no WS63 analogue. The PDM block decimates a PDM mic
//! bitstream (CIC + compensation + sample-rate conversion) into PCM, which is
//! drained from the UP FIFO **by DMA** — there is no CPU-readable sample register.
//! So this driver covers configuration/bring-up (clock + channel enable) and reads
//! back the `version` register for presence; full audio capture is a DMA-fed path
//! (future work). Register map from fbb_bs2x `hal_pdm_v150`; bs2x-pac `Pdm`
//! @ 0x5208_e000.

use crate::peripherals::Pdm as PdmPeriph;
use core::marker::PhantomData;

pub struct Pdm<'d> {
    _p: PhantomData<PdmPeriph<'d>>,
}

impl<'d> Pdm<'d> {
    fn regs(&self) -> &'static crate::soc::pac::pdm::RegisterBlock {
        // SAFETY: static physical MMIO base (0x5208_e000) from bs2x-pac.
        unsafe { &*PdmPeriph::ptr() }
    }

    /// Bring up the PDM block: release the datapath reset and enable the mic /
    /// UP-FIFO clocks + the up channel (`clk_rst_en`).
    pub fn new(_p: PdmPeriph<'d>) -> Self {
        let this = Self { _p: PhantomData };
        this.regs().clk_rst_en().write(|w| {
            w.pdm_dp_rst_n().set_bit();
            w.dmic_clken().set_bit();
            w.up_fifo_clken().set_bit();
            w.func_up_en().set_bit();
            w.func_up_ch_en_0().set_bit()
        });
        this
    }

    /// Read the PDM IP version register (presence/bring-up check).
    pub fn version(&self) -> u32 {
        self.regs().version().read().bits()
    }
}
