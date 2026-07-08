//! BS2X PDM — PDM-microphone audio front-end (IP v150).
//!
//! BS2X-only (`chip-bs21`); no WS63 analogue. The PDM block decimates a PDM mic
//! bitstream (CIC + compensation + sample-rate conversion) into 32-bit PCM words
//! that queue in the UP FIFO. The FIFO is a fixed (non-incrementing) MMIO data
//! window at `0x5208_E080` — the vendor drains it by DMA, but it is also directly
//! CPU-readable, gated on the UP-FIFO status (`up_fifo_st` @ 0x30, bit2 = empty).
//! Register map + sequence from fbb_bs2x `hal_pdm_v150`; bs2x-pac `Pdm`
//! @ 0x5208_e000 models the config registers, status register, and FIFO data
//! window.

use crate::peripherals::Pdm as PdmPeriph;
use core::marker::PhantomData;

const POLL_LIMIT: u32 = 1_000_000;

/// PDM-microphone audio front-end driver (BS2X IP v150): decimates the DMIC PDM
/// bitstream into PCM words drained from the UP FIFO.
pub struct Pdm<'d> {
    _p: PhantomData<PdmPeriph<'d>>,
}

impl<'d> Pdm<'d> {
    fn regs(&self) -> &'static crate::soc::pac::pdm::RegisterBlock {
        // SAFETY: static physical MMIO base (0x5208_e000) from bs2x-pac.
        unsafe { &*PdmPeriph::ptr() }
    }

    /// Bring up the full DMIC-CH0 audio path (CIC + SRC + UP FIFO) and start it
    /// (`hal_pdm_v150_set_attr` + `_start`).
    pub fn new(_p: PdmPeriph<'d>) -> Self {
        let this = Self { _p: PhantomData };
        let r = this.regs();
        unsafe {
            // CIC decimation filter: gain + enable for channel 0.
            r.cic_ctrl().write(|w| {
                w.cic_gain_0().bits(0x14);
                w.cic_en_0().set_bit()
            });
            // Sample-rate down-conversion mode for channel 0 (0 = 2x / 32->16 kHz).
            r.srcdn_ctrl().write(|w| w.srcdn_mode_0().bits(0));
            // UP FIFO thresholds (almost-empty / almost-full).
            r.up_fifo_ctrl().write(|w| {
                w.up_fifo_aempty_th().bits(3);
                w.up_fifo_afull_th().bits(16)
            });
            // Clocks: DMIC + UP-FIFO + the channel-0 up path.
            r.clk_rst_en().write(|w| {
                w.dmic_clken().set_bit();
                w.up_fifo_clken().set_bit();
                w.func_up_en().set_bit();
                w.func_up_ch_en_0().set_bit()
            });
            // Start: release the datapath reset (active-low run bit).
            r.clk_rst_en().modify(|_, w| w.pdm_dp_rst_n().set_bit());
        }
        this
    }

    /// Read the PDM IP version register (presence/bring-up check).
    pub fn version(&self) -> u32 {
        self.regs().version().read().bits()
    }

    /// Capture PCM samples from the UP FIFO into `buf` (CPU-polled). Each entry is
    /// a 32-bit FIFO word (the meaningful PCM is the upper 16 bits). Returns the
    /// number of samples actually captured (fewer than `buf.len()` only on
    /// timeout).
    pub fn capture(&self, buf: &mut [u32]) -> usize {
        let r = self.regs();
        let mut n = 0;
        for slot in buf.iter_mut() {
            // Wait for the FIFO to be non-empty.
            let mut spins = 0;
            while r.up_fifo_st().read().up_fifo_empty_int().bit_is_set() {
                spins += 1;
                if spins > POLL_LIMIT {
                    return n;
                }
                core::hint::spin_loop();
            }
            *slot = r.up_fifo_data().read().pcm_word().bits();
            n += 1;
        }
        n
    }
}
