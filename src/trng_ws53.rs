//! WS53 unified-security TRNG (SDK 163d9150), experimental configured handoff.
//!
//! The vendor application also supplies random buffers to the control core.
//! A singleton Rust token alone does not exclude that service. IPC18 image
//! dispatch belongs to the control-core cooperation layer; live peer handoff, entropy
//! qualification, platform clock/source initialization and cryptographic RNG
//! traits are not provided here. `start_sdk_ring` covers only the enabled SDK
//! application's ring-enable/200us path. The two-word filter is not a continuous
//! health test: identical accepted words on separate calls remain possible.
use crate::{peripherals::Trng, soc::pac};
use core::num::NonZeroU16;
use embedded_hal::delay::DelayNs;

/// Ring startup readback failure. The running/stopped state is now unknown;
/// no rollback, FIFO access or automatic ring disable is attempted.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum RingStartError {
    /// Expected `before | 1` after the settling interval, preserving all other
    /// handed-off control bits. Readback is not proof of entropy/source health.
    ControlChanged {
        /// Complete handed-off word before enabling.
        before: u16,
        /// Complete observed word after the settling interval.
        after: u16,
    },
}

/// Failed startup retaining its exclusive peripheral reservation. Dropping
/// this value does not alter the hardware or discharge service obligations.
pub struct RingStartFailure<'d> {
    token: Trng<'d>,
    error: RingStartError,
}
impl<'d> RingStartFailure<'d> {
    /// Readback error, without releasing or reconfiguring the source.
    pub const fn error(&self) -> RingStartError {
        self.error
    }
    /// Return reservation and error for platform recovery. The source may be
    /// running; preserve its clock/power/exclusivity until recovery/handback.
    pub fn into_parts(self) -> (Trng<'d>, RingStartError) {
        (self.token, self.error)
    }
}
impl core::fmt::Debug for RingStartFailure<'_> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("RingStartFailure").field("error", &self.error).finish_non_exhaustive()
    }
}
#[cfg(feature = "defmt")]
impl defmt::Format for RingStartFailure<'_> {
    fn format(&self, f: defmt::Formatter<'_>) {
        defmt::write!(f, "RingStartFailure({})", self.error);
    }
}

/// Bounded read failure; no rejected candidate is returned as successful data.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    /// The shared ring is disabled. No FIFO read occurs in this attempt.
    RingDisabled,
    /// No acceptable word within the total attempt budget. Retry is permitted.
    /// Rejected words may have been consumed; no time or entropy bound is implied.
    Timeout,
}

/// A fill stopped after writing a prefix. The remaining bytes are unchanged.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct FillError {
    written: usize,
    error: Error,
}
impl FillError {
    /// Number of bytes successfully written into this call's destination.
    pub const fn written(self) -> usize {
        self.written
    }
    /// Failure which stopped the fill.
    pub const fn error(self) -> Error {
        self.error
    }
}

/// One externally reserved consumer of the shared security FIFO.
/// Reads require mutable access; waiting never masks global interrupts.
///
/// An unclaimed reservation cannot be passed directly to the driver:
/// ```compile_fail
/// let p = hisi_hal::Peripherals::take().unwrap();
/// let _ = unsafe { hisi_hal::trng::TrngDriver::from_configured(p.TRNG) };
/// ```
/// Configuration/service handoff cannot be assumed by a safe constructor:
/// ```compile_fail
/// let p = hisi_hal::Peripherals::take().unwrap();
/// let token = unsafe { p.TRNG.assume_application_ownership() };
/// let _ = hisi_hal::trng::TrngDriver::from_configured(token);
/// ```
pub struct TrngDriver<'d> {
    token: Trng<'d>,
    engine: Engine<PacRegisters>,
}
impl<'d> TrngDriver<'d> {
    /// Enable the SDK application's shared ring, wait at least200us, and verify
    /// the complete16-bit control word. Only bit0 is modified; even an already
    /// enabled ring receives the settling delay. Does not consume FIFO data.
    ///
    /// This is not full SoC or entropy-source initialization: it does not select
    /// FRO, configure a source divider, enable unknown upstream clocks, reset
    /// security hardware, negotiate IPC or qualify randomness. The pinned
    /// ws53_liteos_app profile enables RING_ENABLE with FRO/CLK_OPEN disabled.
    /// Drop/release leave the ring running. Failure retains the reservation.
    ///
    /// # Safety
    /// Hand off exclusive access to both TRNG/FIFO and the complete shared ring
    /// control word. Stop/coordinate all other CPU, IRQ, DMA and crypto-service
    /// users. Independently establish accessible power, reset, upstream clock
    /// and entropy-source configuration, and keep them stable until handback.
    /// No other writer may change this control word, including reserved bits.
    /// A Rust token or observed bit alone establishes none of these facts.
    ///
    /// Qualify the SDK double-ready/two-read sequence for the installed source
    /// profile, as for `from_configured`. `delay` must meet DelayNs minimum time
    /// and progress independently without this source or its interrupts. No
    /// software bound can recover a stalled bus access. On readback failure,
    /// keep all peers quiescent and use the platform's proven recovery path;
    /// do not resume an overwritten application or assume the source is off.
    pub unsafe fn start_sdk_ring(token: Trng<'d>, delay: &mut impl DelayNs) -> Result<Self, RingStartFailure<'d>> {
        let mut io = PacRegisters;
        if let Err(error) = start_ring(&mut io, delay) {
            return Err(RingStartFailure { token, error });
        }
        Ok(Self { token, engine: Engine { io } })
    }

    /// Inherit an already configured and settled unified-security TRNG.
    /// This constructor does not access registers or enable the ring.
    ///
    /// # Safety
    /// Coordinate and stop all competing application/control-core random-service
    /// clients, IRQ handlers and security users. Keep the source clock, reset,
    /// entropy-source selection and shared ring stable and accessible for this
    /// driver's lifetime. Complete the source-required settling interval (SDK
    /// init waits 200us) before handoff. This driver does not implement that
    /// service negotiation, board initialization or optional FRO/CLK_OPEN setup.
    ///
    /// Independently qualify that observing DATA_READY and DONE twice admits
    /// two consecutive FIFO_DATA reads without another consumer taking either
    /// word. This is the SDK sequence, not proven silicon FIFO-depth behavior.
    /// After release or Drop the ring and generator remain running; preserve
    /// their service obligations until an external owner takes responsibility.
    /// Entropy quality requires separate qualification even after these duties.
    pub unsafe fn from_configured(token: Trng<'d>) -> Self {
        Self { token, engine: Engine { io: PacRegisters } }
    }

    /// Attempt one candidate: check ring, check both ready flags twice, consume
    /// candidate and comparison, and reject zero/all-ones/equal candidates.
    /// `None` means not ready or rejected, without exposing the candidate.
    /// At most two status reads and two destructive FIFO reads are performed.
    pub fn try_read_word(&mut self) -> Result<Option<u32>, Error> {
        self.engine.try_read()
    }

    /// Read one accepted word within `attempts` polling attempts. Wait 1us only
    /// between unsuccessful attempts; this is not a wall-clock deadline. The
    /// delay implementation must progress independently of this peripheral.
    /// A rejected pair consumes an attempt just like a not-ready observation.
    pub fn read_word(&mut self, attempts: NonZeroU16, delay: &mut impl DelayNs) -> Result<u32, Error> {
        self.engine.read(&mut attempts.get(), delay)
    }

    /// Fill bytes in explicit little-endian word order using one total attempt
    /// budget for the entire call. Empty slices succeed without any MMIO/delay.
    /// On failure, report the written prefix; the remaining bytes are unchanged.
    /// A partial final word discards its unused bytes. Neither success nor an
    /// error claims entropy quality or reinitializes the shared generator.
    pub fn fill_bytes(
        &mut self,
        bytes: &mut [u8],
        attempts: NonZeroU16,
        delay: &mut impl DelayNs,
    ) -> Result<(), FillError> {
        self.engine.fill(bytes, attempts, delay)
    }

    /// Return the reservation without disabling/reconfiguring the shared ring.
    /// The receiver inherits the running source and service obligations.
    pub fn release_without_stopping(self) -> Trng<'d> {
        self.token
    }
}

trait Registers {
    fn ring_enabled(&mut self) -> bool;
    fn ready(&mut self) -> bool;
    fn consume(&mut self) -> u32;
}
struct Engine<R: Registers> {
    io: R,
}
impl<R: Registers> Engine<R> {
    fn try_read(&mut self) -> Result<Option<u32>, Error> {
        if !self.io.ring_enabled() {
            return Err(Error::RingDisabled);
        }
        if !self.io.ready() || !self.io.ready() {
            return Ok(None);
        }
        let candidate = self.io.consume();
        let comparison = self.io.consume();
        Ok((candidate != 0 && candidate != u32::MAX && candidate != comparison).then_some(candidate))
    }

    fn read(&mut self, remaining: &mut u16, delay: &mut impl DelayNs) -> Result<u32, Error> {
        while *remaining != 0 {
            *remaining -= 1;
            if let Some(word) = self.try_read()? {
                return Ok(word);
            }
            if *remaining != 0 {
                delay.delay_us(1);
            }
        }
        Err(Error::Timeout)
    }

    fn fill(&mut self, bytes: &mut [u8], attempts: NonZeroU16, delay: &mut impl DelayNs) -> Result<(), FillError> {
        let mut remaining = attempts.get();
        let mut written = 0;
        for chunk in bytes.chunks_mut(4) {
            let word = self.read(&mut remaining, delay).map_err(|error| FillError { written, error })?;
            chunk.copy_from_slice(&word.to_le_bytes()[..chunk.len()]);
            written += chunk.len();
        }
        Ok(())
    }
}
struct PacRegisters;
trait RingRegisters {
    fn control(&mut self) -> u16;
    fn enable_bit(&mut self);
    fn fence(&mut self);
}
fn start_ring(io: &mut impl RingRegisters, delay: &mut impl DelayNs) -> Result<(), RingStartError> {
    let before = io.control();
    if before & 1 == 0 {
        io.enable_bit();
    }
    io.fence();
    delay.delay_us(200);
    let after = io.control();
    if after != before | 1 {
        return Err(RingStartError::ControlChanged { before, after });
    }
    Ok(())
}
impl RingRegisters for PacRegisters {
    fn control(&mut self) -> u16 {
        unsafe { &*pac::MCrg::PTR }.trng_ring_control().read().bits()
    }
    fn enable_bit(&mut self) {
        // This RW view has no read side effect. The field writer preserves the
        // other15 bits; exclusive full-word handoff is required by the caller.
        unsafe { &*pac::MCrg::PTR }.trng_ring_control().modify(|_, w| w.ring_enable().set_bit());
    }
    fn fence(&mut self) {
        #[cfg(target_arch = "riscv32")]
        unsafe {
            core::arch::asm!("fence iorw,iorw", options(nostack));
        }
        #[cfg(not(target_arch = "riscv32"))]
        core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);
    }
}
impl Registers for PacRegisters {
    fn ring_enabled(&mut self) -> bool {
        unsafe { &*pac::MCrg::PTR }.trng_ring_control().read().ring_enable().bit_is_set()
    }
    fn ready(&mut self) -> bool {
        let value = unsafe { &*pac::Trng::PTR }.fifo_ready().read();
        value.data_ready().bit_is_set() && value.done().bit_is_set()
    }
    fn consume(&mut self) -> u32 {
        unsafe { &*pac::Trng::PTR }.fifo_data().read().value().bits()
    }
}

#[cfg(all(test, not(target_arch = "riscv32")))]
#[path = "trng_ws53_tests.rs"]
mod tests;
