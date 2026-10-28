//! Experimental WS53 AES/SM4 full-block transfers. Source-profile control and
//! descriptor ABI are separate from algorithm, silicon and DMA-drain acceptance.
//! Uses a real KM lease, never an unchecked integer slot or direct key write.
//!
//! The whole SPACC controller, key and static allocations remain reserved on
//! success, failure, cancellation, Drop and forget. Completion returns metadata;
//! reading output memory requires a separate unsafe physical-quiescence handback.
//! Partial stream blocks, AEAD/MAC, IRQ/async and initialization are not yet here.
use crate::{km, peripherals::Spacc, soc::pac};
use core::num::NonZeroU16;
use embedded_hal::delay::DelayNs;

#[path = "symc_ws53/config.rs"]
mod config;
#[path = "symc_ws53/engine.rs"]
mod engine;
#[path = "symc_ws53/memory.rs"]
mod memory;
use config::Configuration;
pub use config::{BlockLength, Channel, Direction, Mode, PrepareError};
use engine::{Engine, PacRegisters, State};
pub use memory::{Buffer, DescriptorStorage, MemoryError, Table};

/// Complete diagnostic words, not a speculative error-clear interface.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct HardwareError {
    /// TEE SYMC cause, including unknown bits.
    pub tee_cause: u32,
    /// TEE SYMC status.
    pub tee_status: u32,
    /// REE SYMC cause, including unknown bits.
    pub ree_cause: u32,
    /// REE SYMC status.
    pub ree_status: u32,
}
impl HardwareError {
    fn present(self) -> bool {
        self.tee_cause != 0 || self.ree_cause != 0
    }
}
/// Transfer failure. Terminal failures cache the first error and stop MMIO.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    /// No start was made; no MMIO.
    NotStarted,
    /// Repeated start rejected; no MMIO.
    AlreadyStarted,
    /// Channel busy or REE claim rejected.
    Busy,
    /// Channel ownership changed after its claim.
    OwnershipLost,
    /// KM lease health/ownership check failed.
    Key(km::Error),
    /// Hardware error cause is nonzero. Full status is retained.
    Hardware(HardwareError),
    /// Channel clear did not finish within the attempt budget.
    ClearTimeout,
    /// Selected acknowledgement did not read back cleared.
    Acknowledge,
    /// Ring did not start empty or pointers no longer match this submission.
    Ring,
    /// No usable completion within the attempt budget.
    Timeout,
    /// Locally abandoned, not physically aborted.
    Cancelled,
}
/// A protocol completion; not proof of memory/key handback or authenticity.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Completion {
    /// Exact submitted byte count.
    pub bytes: u32,
    /// Four source-profile little-endian IV state words, copied as 16 bytes.
    /// ECB does not promise a meaningful updated IV.
    pub iv: [u8; 16],
}
/// Unpublished resources, or resources returned after external physical drain.
/// Input and output are distinct allocations; this API has no in-place path.
pub struct Resources<'k> {
    /// Entire shared SPACC reservation, including HASH and IRQ59.
    pub spacc: Spacc<'static>,
    /// Noncopyable loaded key lease, consumed by the transfer.
    pub key: km::LoadedKey<'k>,
    /// Source allocation; only the requested prefix is submitted.
    pub input: Buffer,
    /// Destination allocation, not exposed while DMA may write it.
    pub output: Buffer,
    /// Combined two-entry input and output tables.
    pub descriptors: Table,
}
/// One-shot transfer retaining all resources, including the consumed key lease.
/// No safe completion/cancellation/drop path releases memory or key ownership.
/// ```compile_fail
/// use hisi_hal::symc::Transfer;
/// fn release(t: Transfer<'_>) { t.release_after_external_quiescence(); }
/// ```
/// The input/output cannot be recovered by accessing the transfer directly:
/// ```compile_fail
/// use hisi_hal::symc::Transfer;
/// fn output(t: Transfer<'_>) { let data = t.output.into_bytes(); }
/// ```
/// Preparing consumes the key as part of the resources, even before start:
/// ```compile_fail
/// use hisi_hal::symc::{Resources, Transfer, Channel, Mode, Direction, BlockLength};
/// fn retain(r: Resources<'_>) {
///     let key = &r.key;
///     let transfer = unsafe { Transfer::from_configured(r, Channel::One, Mode::Ecb,
///         Direction::Encrypt, BlockLength::from_bytes(16).unwrap()) };
///     key.validate().unwrap();
/// }
/// ```
pub struct Transfer<'k> {
    spacc: Spacc<'static>,
    input: Buffer,
    output: Buffer,
    descriptors: Table,
    engine: Engine<PacRegisters<'static>, km::LoadedKey<'k>>,
}
impl<'k> Transfer<'k> {
    /// Validate key metadata and disjoint capacities, then fill unpublished
    /// descriptors. No MMIO; on failure return every resource unchanged.
    ///
    /// # Safety
    /// Negotiate exclusive ownership of the entire shared SPACC block, including
    /// HASH, SYMC, error/lock/IRQ59 words and all prior control/security CPU users.
    /// Independently stop and physically drain previous requests before entry.
    /// Keep inherited clocks/reset, REE identity, little-endian descriptor ABI,
    /// coherent/uncached RAM and bus mapping valid; prevent reset/sleep and races.
    /// Qualify selected-bit clear/IRQ acknowledgement, empty two-entry rings
    /// starting at zero, and the full-block mode/IV profile on the actual target.
    /// This is not performed or proved by this constructor or the SDK source.
    /// Both tables are complete before output publication; once either address
    /// is published, all reservations and KM obligations survive Drop/forget
    /// until independently proven physical quiescence. No cache maintenance,
    /// cross-core handoff, security/global IRQ initialization is supplied.
    #[allow(clippy::result_large_err)] // no heap; failed validation must return all reservations
    pub unsafe fn from_configured(
        mut resources: Resources<'k>,
        channel: Channel,
        mode: Mode,
        direction: Direction,
        length: BlockLength,
    ) -> Result<Self, (PrepareError, Resources<'k>)> {
        let key = &resources.key;
        let config = match Configuration::new(key.slot(), key.algorithm(), key.parity(), key.usage(), direction, mode) {
            Ok(c) => c,
            Err(e) => return Err((e, resources)),
        };
        if let Err(e) =
            memory::validate(resources.input.range, resources.output.range, resources.descriptors.range, length)
        {
            return Err((e, resources));
        }
        resources.descriptors.fill(resources.input.range.start, resources.output.range.start, length, config);
        let engine = Engine {
            // SAFETY: exclusive configured peripheral is an explicit entry obligation.
            io: PacRegisters { registers: unsafe { &*pac::Spacc::PTR } },
            key: resources.key,
            channel,
            config,
            length,
            table: resources.descriptors.range.start,
            state: State::New,
        };
        Ok(Self {
            spacc: resources.spacc,
            input: resources.input,
            output: resources.output,
            descriptors: resources.descriptors,
            engine,
        })
    }
    /// Recheck key health, claim and clear with a bounded wait, configure and
    /// publish output then input. Delays occur only between unsuccessful polls.
    pub fn start(&mut self, attempts: NonZeroU16, delay: &mut impl DelayNs) -> Result<(), Error> {
        self.engine.start(attempts, delay)
    }
    /// One completion inspection. No output DMA memory is read here. Cached
    /// success/failure returns without additional SPACC or KM access.
    pub fn poll(&mut self) -> Result<Option<Completion>, Error> {
        self.engine.poll()
    }
    /// Finite attempts, 1us between unsuccessful polls; not a wall-clock deadline.
    pub fn wait(&mut self, attempts: NonZeroU16, delay: &mut impl DelayNs) -> Result<Completion, Error> {
        self.engine.wait(attempts, delay)
    }
    /// Local abandonment, with no MMIO, key release or DMA-stop claim.
    pub fn cancel(&mut self) {
        self.engine.cancel();
    }
    /// Return reservations without MMIO, unlock, key erasure or cache maintenance.
    ///
    /// # Safety
    /// Independently stop all requests/IRQ users and physically drain all input
    /// fetches, output writes, key consumers and posted bus transactions. Resolve
    /// cache aliases and shared ownership before CPU memory access or reuse.
    /// Completion/IV, equal pointers, CPU halt, timeout and Drop are insufficient.
    /// This does not release the KM slot: its returned lease still needs its own
    /// explicit quiescence contract and checked release protocol.
    pub unsafe fn release_after_external_quiescence(self) -> Resources<'k> {
        Resources {
            spacc: self.spacc,
            key: self.engine.key,
            input: self.input,
            output: self.output,
            descriptors: self.descriptors,
        }
    }
}

#[cfg(all(test, not(target_arch = "riscv32")))]
#[path = "symc_ws53/tests.rs"]
mod tests;
