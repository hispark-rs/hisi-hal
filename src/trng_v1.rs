//! BS2X TRNG — true random number generator (IP v1).
//!
//! BS2X-only (`chip-bs21`) implementation of the `trng` module (WS63 has a
//! different TRNG, `trng.rs`). Register map from fbb_bs2x `hal_trng_v1` (sole
//! ground truth): enable the ring oscillators (`trng_ring_en`), poll the FIFO
//! ready+done bits, read a 32-bit random word from `trng_fifo_data` (the FIFO
//! auto-advances per read). bs2x-pac `Trng` @ 0x5200_9000.

use crate::peripherals::Trng as TrngPeriph;
use core::marker::PhantomData;

const POLL_LIMIT: u32 = 1_000_000;

#[derive(Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
/// Errors returned by the BS2X TRNG driver.
pub enum TrngError {
    /// The FIFO never signalled ready+done within the bounded wait.
    Timeout,
}

/// BS2X true random number generator (IP v1) driver.
pub struct Trng<'d> {
    _trng: PhantomData<TrngPeriph<'d>>,
}

impl<'d> Trng<'d> {
    fn regs(&self) -> &'static crate::soc::pac::trng::RegisterBlock {
        // SAFETY: static physical MMIO base (0x5200_9000) from bs2x-pac.
        unsafe { &*TrngPeriph::ptr() }
    }

    /// Enable the entropy ring oscillators (`hal_trng_v1_start`).
    ///
    /// NB: the vendor also enables a TRNG clock/ring/reset in the M_CTL fabric
    /// (0x5200_0xxx) before this; that is the clock-tree's job. TODO(bs2x): wire
    /// the M_CTL clock-enable when bringing TRNG up on silicon (the QEMU model
    /// reports ready regardless).
    pub fn new(_trng: TrngPeriph<'d>) -> Self {
        let this = Self { _trng: PhantomData };
        this.regs().trng_ring_en().write(|w| {
            w.ro_en().set_bit();
            w.tero_en().set_bit();
            w.fro_en().set_bit()
        });
        this
    }

    /// Read one 32-bit random word (`hal_trng_v1_get`): wait for the FIFO to be
    /// ready AND done, then pop `trng_fifo_data`.
    pub fn next_u32(&self) -> Result<u32, TrngError> {
        let r = self.regs();
        for _ in 0..POLL_LIMIT {
            let rdy = r.trng_fifo_ready().read();
            if rdy.trng_ready().bit_is_set() && rdy.trng_done().bit_is_set() {
                return Ok(r.trng_fifo_data().read().trng_data().bits());
            }
            core::hint::spin_loop();
        }
        Err(TrngError::Timeout)
    }
}
