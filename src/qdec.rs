//! BS2X QDEC — quadrature decoder (IP v150).
//!
//! BS2X-only (`chip-bs21`); no WS63 analogue. An nRF-style quadrature decoder: it
//! samples a 2-bit A/B gray code and accumulates SIGNED motion into ACC. The
//! "count" is the signed `qdec_acc_data` value (negative = reverse). Register map
//! from fbb_bs2x `hal_qdec_v150`; bs2x-pac's `qdec` block is a faithful match (the
//! ACC field is unsigned in the PAC, so this driver sign-extends).

use crate::peripherals::Qdec as QdecPeriph;
use core::marker::PhantomData;

pub struct Qdec<'d> {
    _q: PhantomData<QdecPeriph<'d>>,
}

impl<'d> Qdec<'d> {
    fn regs(&self) -> &'static crate::soc::pac::qdec::RegisterBlock {
        // SAFETY: static physical MMIO base (0x5200_0200) from bs2x-pac.
        unsafe { &*QdecPeriph::ptr() }
    }

    /// Enable the decoder and begin sampling/accumulation (`hal_qdec_v150_enable`
    /// + `QDEC_TASK_START`).
    pub fn new(_q: QdecPeriph<'d>) -> Self {
        let this = Self { _q: PhantomData };
        let r = this.regs();
        r.qdec_enable().write(|w| w.en().set_bit());
        r.qdec_task_start().write(|w| w.start().set_bit());
        this
    }

    /// Read the live signed accumulated quadrature count (`qdec_acc_data`,
    /// sign-extended from 16 bits). Negative = reverse rotation.
    pub fn read_count(&self) -> i16 {
        self.regs().qdec_acc_data().read().acc_rd_val().bits() as i16
    }

    /// Read the double/invalid-transition error count (`qdec_accdbl_data`).
    pub fn read_error_count(&self) -> u8 {
        self.regs().qdec_accdbl_data().read().dbl_rd_val().bits()
    }

    /// Stop sampling (counter state retained).
    pub fn stop(&self) {
        self.regs().qdec_task_stop().write(|w| w.stop().set_bit());
    }
}
