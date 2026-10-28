//! Experimental WS53 unkeyed HASH. SDK 163d9150 control/layout/message evidence
//! is the source; algorithm results and DMA/ownership behavior require separate
//! model and silicon acceptance. No HMAC, keyslot, SYMC, PKE or IRQ API is exposed.
//!
//! A transfer retains its static input, descriptors and whole SPACC reservation
//! on success, timeout, cancellation, Drop and forget. Completion is not a DMA
//! drain proof. No safe memory handback or implicit channel unlock is provided.
//! This deliberately avoids the SDK's ignored clear timeout and premature free.
use crate::{peripherals::Spacc, soc::pac};
use core::num::NonZeroU16;
use embedded_hal::delay::DelayNs;

#[path = "hash_ws53/format.rs"]
mod format;
#[path = "hash_ws53/memory.rs"]
mod memory;
pub use format::{Algorithm, Digest, FormatError};
pub use memory::{DescriptorStorage, Input, MemoryError, PreparedInput, Table};

/// Source-audited channel identity (PAC array indices are 0/1).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Channel {
    /// SDK HASH channel 1.
    One,
    /// SDK HASH channel 2; polling avoids the SDK IRQ dispatch omission.
    Two,
}
impl Channel {
    const fn index(self) -> usize {
        match self {
            Self::One => 0,
            Self::Two => 1,
        }
    }
}

/// Raw error-bank snapshots. Errors are not cleared by this driver because the
/// SDK path does not establish a writable error-clear protocol.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct HardwareError {
    /// TEE cause register (all bits retained, including unknown ones).
    pub tee_cause: u32,
    /// TEE channel/status register.
    pub tee_status: u32,
    /// REE cause register.
    pub ree_cause: u32,
    /// REE channel/status register.
    pub ree_status: u32,
}
impl HardwareError {
    fn present(self) -> bool {
        self.tee_cause != 0 || self.ree_cause != 0
    }
}

/// Failures retain controller and allocations. Terminal failures perform no
/// additional MMIO on later poll/wait calls; external recovery is required.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Error {
    /// `start` has not been called; this is not terminal.
    NotStarted,
    /// A second start was rejected without MMIO; current state is retained.
    AlreadyStarted,
    /// Selected owner was busy or did not accept the REE claim.
    Busy,
    /// Selected hardware owner no longer matches REE.
    OwnershipLost,
    /// Clear handshake did not complete within the caller's attempt budget.
    ClearTimeout,
    /// A selected completion/clear acknowledgement did not read back low.
    Acknowledge,
    /// Clear did not leave the expected empty ring, or submission changed.
    Ring,
    /// An error bank was nonzero; no error/status reset was attempted.
    Hardware(HardwareError),
    /// Completion did not become usable within the wait budget.
    Timeout,
    /// Locally abandoned; cancellation does not stop or drain the hardware.
    Cancelled,
}

/// One statically reserved unkeyed operation. Memory cannot be recovered by a
/// safe call, even after a digest was copied out.
/// ```compile_fail
/// use hisi_hal::hash::Transfer;
/// fn release(t: Transfer) { t.release_after_external_quiescence(); }
/// ```
pub struct Transfer {
    token: Spacc<'static>,
    input: PreparedInput,
    table: Table,
    engine: Engine<PacRegisters<'static>>,
}
impl Transfer {
    /// Prepare a one-shot transfer without touching MMIO or starting DMA.
    /// Descriptors are filled while unpublished. The whole shared SPACC token
    /// is retained; choosing a channel does not split ownership of the block.
    ///
    /// # Safety
    /// Independently stop/drain all prior HASH/SYMC/core/DMA/IRQ59 users and
    /// negotiate exclusive access to the entire SPACC block, including shared
    /// lock, interrupt and error words. No other user may RMW these words. Keep
    /// its inherited clocks/reset, REE bus identity, little-endian descriptor
    /// profile and CPU mappings valid. Prevent sleep/reset/security changes.
    /// Verify that the SDK clear/completion acknowledgements clear selected
    /// bits, and a cleared two-entry ring can begin at pointer zero. These are
    /// source-derived preconditions, not claimed hardware qualifications.
    /// Input/table contracts remain in force after Drop/forget. Once addresses
    /// have been published, future owners must not reuse either allocation or
    /// reconfigure the block until independent physical quiescence is proved.
    /// This call implements neither IPC negotiation nor security/clock setup.
    pub unsafe fn from_configured(
        token: Spacc<'static>,
        channel: Channel,
        input: PreparedInput,
        mut table: Table,
    ) -> Self {
        table.fill(&input);
        let engine = Engine {
            io: PacRegisters { registers: unsafe { &*pac::Spacc::PTR } },
            channel,
            algorithm: input.algorithm,
            table_address: table.range.start,
            state: State::New,
        };
        Self { token, input, table, engine }
    }
    /// Claim, mask the selected HASH IRQ, clear with a bounded wait, initialize
    /// the unkeyed state and submit one descriptor. Global IRQ gates and other
    /// channels are preserved. At most `attempts` clear polls; delay 1us only
    /// between unsuccessful polls. No wall-clock deadline is implied.
    pub fn start(&mut self, attempts: NonZeroU16, delay: &mut impl DelayNs) -> Result<(), Error> {
        self.engine.start(attempts, delay)
    }
    /// Inspect one completion. Cached success is returned without further MMIO;
    /// no digest is read after a hardware error, lost ownership or timeout.
    pub fn poll(&mut self) -> Result<Option<Digest>, Error> {
        self.engine.poll()
    }
    /// Bounded polling, with 1us only between unsuccessful attempts. The delay
    /// provider must progress independently of SPACC and its interrupt state.
    pub fn wait(&mut self, attempts: NonZeroU16, delay: &mut impl DelayNs) -> Result<Digest, Error> {
        self.engine.wait(attempts, delay)
    }
    /// Retain all resources without touching MMIO. Does not abort a bus master.
    pub fn cancel(&mut self) {
        self.engine.cancel();
    }
    /// Recover reservations after a separately performed physical handback.
    /// This method does not acknowledge, stop, reset, unlock or access memory.
    ///
    /// # Safety
    /// Externally disable requests and IRQ users, stop and drain all descriptor
    /// fetches and data accesses, and retire every posted transaction. Reconcile
    /// cache aliases and channel ownership/locks without disturbing another
    /// core. Completion flags, equal ring pointers, CPU halt, timeout and Drop
    /// alone are not proof. Preserve allocation ownership until these duties
    /// have completed; only then may the returned memory be accessed/reused.
    pub unsafe fn release_after_external_quiescence(self) -> (Spacc<'static>, Input, Table) {
        (self.token, self.input.input, self.table)
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum State {
    New,
    Running,
    Complete(Digest),
    Retained(Error),
}
trait Registers {
    fn owner(&mut self, channel: Channel) -> u8;
    fn claim(&mut self, channel: Channel);
    fn mask_irq(&mut self, channel: Channel);
    fn clear_done(&mut self, channel: Channel) -> bool;
    fn ack_clear(&mut self, channel: Channel);
    fn request_clear(&mut self, channel: Channel);
    fn done(&mut self, channel: Channel) -> bool;
    fn ack_done(&mut self, channel: Channel);
    fn ring(&mut self, channel: Channel) -> (u8, u8, u32);
    fn errors(&mut self) -> HardwareError;
    fn configure(&mut self, channel: Channel, algorithm: Algorithm, table: u32);
    fn submit(&mut self, channel: Channel);
    fn state(&mut self, channel: Channel, words: &mut [u32]);
}
struct Engine<R> {
    io: R,
    channel: Channel,
    algorithm: Algorithm,
    table_address: u32,
    state: State,
}
impl<R: Registers> Engine<R> {
    fn retain<T>(&mut self, error: Error) -> Result<T, Error> {
        self.state = State::Retained(error);
        Err(error)
    }
    fn healthy(&mut self) -> Result<(), Error> {
        if self.io.owner(self.channel) != 1 {
            return self.retain(Error::OwnershipLost);
        }
        let errors = self.io.errors();
        if errors.present() {
            return self.retain(Error::Hardware(errors));
        }
        Ok(())
    }
    fn start(&mut self, attempts: NonZeroU16, delay: &mut impl DelayNs) -> Result<(), Error> {
        if self.state != State::New {
            return Err(Error::AlreadyStarted);
        }
        let errors = self.io.errors();
        if errors.present() {
            return self.retain(Error::Hardware(errors));
        }
        if self.io.owner(self.channel) != 0 {
            return self.retain(Error::Busy);
        }
        self.io.claim(self.channel);
        if self.io.owner(self.channel) != 1 {
            return self.retain(Error::Busy);
        }
        self.io.mask_irq(self.channel);
        if self.io.clear_done(self.channel) {
            self.io.ack_clear(self.channel);
            if self.io.clear_done(self.channel) {
                return self.retain(Error::Acknowledge);
            }
        }
        self.io.request_clear(self.channel);
        let mut cleared = false;
        for i in 0..attempts.get() {
            self.healthy()?;
            if self.io.clear_done(self.channel) {
                cleared = true;
                break;
            }
            if i + 1 < attempts.get() {
                delay.delay_us(1);
            }
        }
        if !cleared {
            return self.retain(Error::ClearTimeout);
        }
        self.io.ack_clear(self.channel);
        if self.io.clear_done(self.channel) {
            return self.retain(Error::Acknowledge);
        }
        if self.io.done(self.channel) {
            self.io.ack_done(self.channel);
            if self.io.done(self.channel) {
                return self.retain(Error::Acknowledge);
            }
        }
        if self.io.ring(self.channel) != (0, 0, 0) {
            return self.retain(Error::Ring);
        }
        self.healthy()?;
        memory::publish_fence();
        self.io.configure(self.channel, self.algorithm, self.table_address);
        self.healthy()?;
        memory::publish_fence();
        self.io.submit(self.channel);
        self.state = State::Running;
        Ok(())
    }
    fn poll(&mut self) -> Result<Option<Digest>, Error> {
        match self.state {
            State::New => return Err(Error::NotStarted),
            State::Retained(error) => return Err(error),
            State::Complete(digest) => return Ok(Some(digest)),
            State::Running => (),
        }
        self.healthy()?;
        if !self.io.done(self.channel) {
            return Ok(None);
        }
        let (write, read, length) = self.io.ring(self.channel);
        if write != 1 {
            return self.retain(Error::Ring);
        }
        if read != 1 || length != 0 {
            return Ok(None);
        }
        self.io.ack_done(self.channel);
        if self.io.done(self.channel) {
            return self.retain(Error::Acknowledge);
        }
        let mut words = [0; 16];
        self.healthy()?;
        self.io.state(self.channel, &mut words[..self.algorithm.state_words()]);
        self.healthy()?;
        let digest = Digest::from_state(self.algorithm, words);
        self.state = State::Complete(digest);
        Ok(Some(digest))
    }
    fn wait(&mut self, attempts: NonZeroU16, delay: &mut impl DelayNs) -> Result<Digest, Error> {
        for i in 0..attempts.get() {
            if let Some(digest) = self.poll()? {
                return Ok(digest);
            }
            if i + 1 < attempts.get() {
                delay.delay_us(1);
            }
        }
        self.retain(Error::Timeout)
    }
    fn cancel(&mut self) {
        if matches!(self.state, State::New | State::Running) {
            self.state = State::Retained(Error::Cancelled);
        }
    }
}

struct PacRegisters<'a> {
    registers: &'a pac::spacc::RegisterBlock,
}
impl PacRegisters<'_> {
    fn regs(&self) -> &pac::spacc::RegisterBlock {
        self.registers
    }
}
impl Registers for PacRegisters<'_> {
    fn owner(&mut self, channel: Channel) -> u8 {
        let r = self.regs().hash_lock().read();
        match channel {
            Channel::One => r.channel1_owner().bits(),
            Channel::Two => r.channel2_owner().bits(),
        }
    }
    fn claim(&mut self, channel: Channel) {
        self.regs().hash_lock().modify(|_, w| unsafe {
            match channel {
                Channel::One => w.channel1_owner().bits(1),
                Channel::Two => w.channel2_owner().bits(1),
            }
        });
    }
    fn mask_irq(&mut self, channel: Channel) {
        self.regs().hash_interrupt_enable().modify(|_, w| match channel {
            Channel::One => w.channel1().clear_bit(),
            Channel::Two => w.channel2().clear_bit(),
        });
    }
    fn clear_done(&mut self, channel: Channel) -> bool {
        let r = self.regs().hash_clear_status().read();
        match channel {
            Channel::One => r.channel1().bit(),
            Channel::Two => r.channel2().bit(),
        }
    }
    fn ack_clear(&mut self, channel: Channel) {
        unsafe {
            self.regs().hash_clear_ack().write_with_zero(|w| match channel {
                Channel::One => w.channel1().set_bit(),
                Channel::Two => w.channel2().set_bit(),
            });
        }
    }
    fn request_clear(&mut self, channel: Channel) {
        unsafe {
            self.regs().hash_clear_request().write_with_zero(|w| match channel {
                Channel::One => w.channel1().set_bit(),
                Channel::Two => w.channel2().set_bit(),
            });
        }
    }
    fn done(&mut self, channel: Channel) -> bool {
        let r = self.regs().hash_interrupt_status().read();
        match channel {
            Channel::One => r.channel1().bit(),
            Channel::Two => r.channel2().bit(),
        }
    }
    fn ack_done(&mut self, channel: Channel) {
        unsafe {
            self.regs().hash_interrupt_ack().write_with_zero(|w| match channel {
                Channel::One => w.channel1().set_bit(),
                Channel::Two => w.channel2().set_bit(),
            });
        }
    }
    fn ring(&mut self, channel: Channel) -> (u8, u8, u32) {
        let r = self.regs().hash_channel(channel.index());
        (
            r.write_pointer().read().pointer().bits(),
            r.read_pointer().read().pointer().bits(),
            r.data_length().read().value().bits(),
        )
    }
    fn errors(&mut self) -> HardwareError {
        let r = self.regs();
        HardwareError {
            tee_cause: r.hash_error(0).cause().read().bits(),
            tee_status: r.hash_error(0).status().read().bits(),
            ree_cause: r.hash_error(1).cause().read().bits(),
            ree_status: r.hash_error(1).status().read().bits(),
        }
    }
    fn configure(&mut self, channel: Channel, algorithm: Algorithm, table: u32) {
        let bank = self.regs();
        let r = bank.hash_channel(channel.index());
        let (alg, mode) = algorithm.hardware();
        r.control().modify(|_, w| unsafe { w.security().bits(10).enable().set_bit() });
        r.key_control()
            .modify(|_, w| unsafe { w.keyslot().bits(0).hmac().clear_bit().algorithm().bits(alg).mode().bits(mode) });
        let state = bank.hash_state(channel.index());
        for (index, word) in algorithm.initial_words().iter().take(algorithm.state_words()).enumerate() {
            unsafe {
                state.index().write_with_zero(|w| w.value().bits(index as u32));
                state.data().write_with_zero(|w| w.value().bits(*word));
            }
        }
        unsafe {
            r.node_base_high().write_with_zero(|w| w.value().bits(0));
            r.node_base_low().write_with_zero(|w| w.address().bits(table));
            r.node_depth().write_with_zero(|w| w.value().bits(2));
        }
    }
    fn submit(&mut self, channel: Channel) {
        let r = self.regs().hash_channel(channel.index());
        let previous = r.write_pointer().read().bits();
        // SDK updates only the low 8-bit pointer; preserve the other read bits.
        unsafe {
            r.submit().write_with_zero(|w| w.value().bits((previous & !0xff) | 1));
        }
    }
    fn state(&mut self, channel: Channel, words: &mut [u32]) {
        let r = self.regs().hash_state(channel.index());
        for (index, word) in words.iter_mut().enumerate() {
            unsafe {
                r.index().write_with_zero(|w| w.value().bits(index as u32));
            }
            *word = r.data().read().value().bits();
        }
    }
}

#[cfg(all(test, not(target_arch = "riscv32")))]
#[path = "hash_ws53/tests.rs"]
mod tests;
