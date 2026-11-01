//! Experimental WS53 shared keyslot and CPU clear-key loader.
//!
//! This does not encrypt, derive root keys, program OTP or configure permanent
//! policy. Eight cipher slots and two HMAC slots share one serialized loader.
//! A dropped/forgotten key stays reserved. Terminal faults stop further MMIO
//! until an external recovery; timeout and dropping Rust state prove no erasure.
use crate::{peripherals::Km, soc::pac};
use core::{marker::PhantomData, num::NonZeroU32};
use embedded_hal::delay::DelayNs;

#[path = "km_ws53/config.rs"]
mod config;
#[path = "km_ws53/engine.rs"]
mod engine;
use config::Material;
pub use config::{
    Algorithm, CipherKey, CipherSlot, HmacAlgorithm, HmacKey, HmacSlot, KeyLengthError, Parity, Slot, Usage,
};
use engine::{Core, Lease, PacRegisters};

/// Complete raw error words; no speculative error-clear command is issued.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct HardwareError {
    /// KL_ERROR, all bits retained.
    pub loader: u32,
    /// KC_ERROR, all bits retained.
    pub controller: u32,
}
/// A failure does not return a hardware keyslot or imply a stopped loader.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    /// Odd-only selection is invalid for a 192/256-bit key; no MMIO performed.
    InvalidParity,
    /// Another call holds the shared loader/selector transaction; no MMIO.
    Borrowed,
    /// This slot was already reserved, including a dropped/forgotten key.
    Reserved,
    /// An unclaimed slot has a hardware owner; no claim was issued.
    SlotBusy(u8),
    /// REE ownership was not observed after a claim or was subsequently lost.
    Ownership(u8),
    /// A prior operation left a hardware error word nonzero.
    Hardware(HardwareError),
    /// A REE flush is already in progress or reported unlock/timeout error.
    FlushStatus(u32),
    /// A clear route is already active before submission.
    LoaderBusy,
    /// An inherited TS-cipher selector cannot be treated as a normal slot.
    UnsupportedSelection,
    /// Unowned/unknown interrupt bits make a whole-word acknowledgement unsafe.
    UnexpectedInterrupt(u32),
    /// The selected completion bit did not clear after acknowledgement.
    Acknowledge,
    /// Clear route did not complete within the caller's poll budget.
    LoadTimeout,
    /// Release did not complete within the caller's poll budget.
    ReleaseTimeout,
    /// Flush reported idle but the selected owner did not become zero.
    ReleaseUnconfirmed(u8),
    /// Internal slot state does not denote a fully loaded key.
    NotLoaded,
}

/// Local bookkeeping only; this is not a hardware or key-erasure report.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Snapshot {
    /// Bit0..7 cipher, bit8..9 HMAC; includes dropped/forgotten keys.
    pub reserved: u16,
    /// Slots whose last load completed the configured protocol.
    pub loaded: u16,
    /// First terminal error. Once present, later operations perform no MMIO.
    pub fault: Option<Error>,
}

/// Whole KM controller, confined to the application CPU. The non-Send/non-Sync
/// marker prevents cross-thread/core selector races; RefCell rejects reentry.
///
/// A live lease prevents handing the manager back, even through its unsafe API:
/// ```compile_fail
/// use hisi_hal::km::{KeyManager, CipherSlot, CipherKey, Usage, Parity};
/// use core::num::NonZeroU32;
/// fn handback(manager: KeyManager, delay: &mut impl embedded_hal::delay::DelayNs) {
///     let key = manager.load_cipher(CipherSlot::Zero, CipherKey::Aes128(&[0; 16]),
///         Usage::Both, Parity::Even, NonZeroU32::new(1).unwrap(), delay).unwrap();
///     let token = unsafe { manager.release_after_external_quiescence() };
///     key.validate().unwrap();
/// }
/// ```
pub struct KeyManager {
    token: Km<'static>,
    inner: Core<PacRegisters<'static>>,
    _core: PhantomData<*mut ()>,
}
impl KeyManager {
    /// Reserve an already handed-over controller without MMIO.
    ///
    /// # Safety
    /// Negotiate exclusive ownership of the entire KM block, every slot,
    /// selector and IRQ61 against control/security firmware and all handlers.
    /// Stop prior root/common/clear routes and all previous key consumers,
    /// including live Flash decryption. This does not perform that handoff.
    /// Keep KM clocks/reset, REE identity and MMIO ordering valid; prevent sleep,
    /// security changes and other users until external handback. Verify the
    /// source profile: 8 cipher/2 HMAC slots, REE owner1, little-endian staging,
    /// idle START0 permitting subsequent staging writes, and raw bit14 ACK
    /// clearing the event without replay. Unknown pending causes are rejected.
    /// Verify that completed clear commands copy staging words into the chosen
    /// slot and that flush idle plus owner0 means the slot is no longer in use.
    /// These are unqualified integration preconditions, not silicon evidence.
    /// The controller is application-CPU-only and uses no common/root lock as
    /// a substitute for cross-core exclusion. Delay providers must not require
    /// this peripheral to progress. No clock, OTP or security setup is made.
    pub unsafe fn from_configured(token: Km<'static>) -> Self {
        Self { token, inner: Core::new(PacRegisters { registers: unsafe { &*pac::Km::PTR } }), _core: PhantomData }
    }
    /// Claim one cipher slot and synchronously load a size-checked AES/SM4 key.
    /// The supplied bytes are borrowed only for this call; no DMA points to
    /// them. They are not logged, stored in the driver, or erased by this API.
    /// Temporary CPU/stack copies have no guaranteed zeroization.
    /// Each stage has at most `attempts` polls with 1us between unsuccessful polls.
    pub fn load_cipher(
        &self,
        slot: CipherSlot,
        key: CipherKey<'_>,
        usage: Usage,
        parity: Parity,
        attempts: NonZeroU32,
        delay: &mut impl DelayNs,
    ) -> Result<LoadedKey<'_>, Error> {
        self.load(Slot::Cipher(slot), Material::Cipher(key, usage, parity), attempts, delay)
    }
    /// Claim a HMAC slot and load a validated block-sized-or-shorter key. Long
    /// HMAC keys must be hashed by the caller according to the chosen algorithm.
    /// This only loads a key; it does not provide a HASH/HMAC computation API.
    pub fn load_hmac(
        &self,
        slot: HmacSlot,
        key: HmacKey<'_>,
        attempts: NonZeroU32,
        delay: &mut impl DelayNs,
    ) -> Result<LoadedKey<'_>, Error> {
        self.load(Slot::Hmac(slot), Material::Hmac(key), attempts, delay)
    }
    fn load(
        &self,
        slot: Slot,
        material: Material<'_>,
        attempts: NonZeroU32,
        delay: &mut impl DelayNs,
    ) -> Result<LoadedKey<'_>, Error> {
        Ok(LoadedKey { lease: self.inner.load(slot, material, attempts, delay)? })
    }
    /// Inspect local reservations without reading hardware or key material.
    pub fn snapshot(&self) -> Result<Snapshot, Error> {
        self.inner.snapshot()
    }
    /// Recover the singleton only after independently reconciling all users.
    /// Does not read/write, clear, reset, unlock or erase anything.
    ///
    /// # Safety
    /// Stop/drain all loader/flush transactions and all key consumers, including
    /// forgotten transfers/keys. Independently erase/reconcile slots and shared
    /// state according to the intended policy. No other core/handler may retain
    /// a reference. Dropping Rust values, START0, equal ring pointers or a poll
    /// timeout alone cannot establish this. Keep that contract through handback.
    pub unsafe fn release_after_external_quiescence(self) -> Km<'static> {
        self.token
    }
}

/// A non-copyable slot lease tied to its manager. Drop/forget deliberately do
/// not free or reuse the hardware slot, and never perform MMIO.
/// ```compile_fail
/// use hisi_hal::km::LoadedKey;
/// fn copy_key(key: LoadedKey<'_>) { let other = key; let duplicate = key; }
/// ```
/// Releasing a slot requires an explicit unsafe quiescence assertion:
/// ```compile_fail
/// use hisi_hal::km::LoadedKey;
/// fn release(key: LoadedKey<'_>, delay: &mut impl embedded_hal::delay::DelayNs) {
///     key.release_after_external_quiescence(core::num::NonZeroU32::new(1).unwrap(), delay);
/// }
/// ```
pub struct LoadedKey<'a> {
    lease: Lease<'a, PacRegisters<'static>>,
}
impl LoadedKey<'_> {
    /// Typed slot identity; not a grant to use raw MMIO or release the slot.
    pub fn slot(&self) -> Slot {
        self.lease.slot
    }
    /// Algorithm bound when loading this lease; contains no key bytes.
    pub fn algorithm(&self) -> Algorithm {
        self.lease.algorithm
    }
    /// Cipher half selection; Even is returned for longer keys and HMAC.
    pub fn parity(&self) -> Parity {
        self.lease.parity
    }
    /// Direction permissions loaded with a cipher key; HMAC returns `None`.
    /// Metadata only: it does not read or change hardware policy.
    pub fn usage(&self) -> Option<Usage> {
        self.lease.usage
    }
    /// Recheck manager health and REE ownership. A successful check is not a
    /// cipher completion, DMA-drain or safe slot-erasure proof.
    pub fn validate(&self) -> Result<(), Error> {
        self.lease.validate()
    }
    /// Flush/unlock with bounded polling and owner reread. The lease is consumed
    /// even on error; its reservation remains in the faulted manager. Success
    /// frees the local reservation, but does not promise physical key erasure.
    ///
    /// # Safety
    /// Independently quiesce all SPACC/other consumers of this slot and retire
    /// their posted transactions before calling, including any forgotten DMA.
    /// A copied slot ID or successful loader status is not that proof.
    pub unsafe fn release_after_external_quiescence(
        self,
        attempts: NonZeroU32,
        delay: &mut impl DelayNs,
    ) -> Result<(), Error> {
        self.lease.release(attempts, delay)
    }
}

#[cfg(all(test, not(target_arch = "riscv32")))]
#[path = "km_ws53/tests.rs"]
mod tests;
