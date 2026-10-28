//! Serialized source-derived protocol. Fakes and QEMU must not be confused with
//! the externally qualified completion/exclusion contract required on silicon.
use super::*;
use core::cell::RefCell;

pub(super) struct Core<R> {
    pub(super) inner: RefCell<Engine<R>>,
}
impl<R: Registers> Core<R> {
    pub(super) fn new(io: R) -> Self {
        Self { inner: RefCell::new(Engine::new(io)) }
    }
    pub(super) fn load(
        &self,
        slot: Slot,
        material: Material<'_>,
        attempts: NonZeroU32,
        delay: &mut impl DelayNs,
    ) -> Result<Lease<'_, R>, Error> {
        let algorithm = material.algorithm();
        let parity = material.parity();
        let usage = material.usage();
        self.inner.try_borrow_mut().map_err(|_| Error::Borrowed)?.load(slot, material, attempts, delay)?;
        Ok(Lease { owner: self, slot, algorithm, parity, usage })
    }
    pub(super) fn snapshot(&self) -> Result<Snapshot, Error> {
        Ok(self.inner.try_borrow().map_err(|_| Error::Borrowed)?.snapshot())
    }
}
pub(super) struct Lease<'a, R> {
    owner: &'a Core<R>,
    pub(super) slot: Slot,
    pub(super) algorithm: Algorithm,
    pub(super) parity: Parity,
    pub(super) usage: Option<Usage>,
}
impl<R: Registers> Lease<'_, R> {
    pub(super) fn validate(&self) -> Result<(), Error> {
        self.owner.inner.try_borrow_mut().map_err(|_| Error::Borrowed)?.validate(self.slot)
    }
    // Only called by the public unsafe handback after consumer quiescence.
    pub(super) fn release(self, attempts: NonZeroU32, delay: &mut impl DelayNs) -> Result<(), Error> {
        self.owner.inner.try_borrow_mut().map_err(|_| Error::Borrowed)?.release(self.slot, attempts, delay)
    }
}

pub(super) trait Registers {
    fn select(&mut self, slot: Slot) -> Result<(), Error>;
    fn owner(&mut self) -> u8;
    fn slot_command(&mut self, slot: Slot, lock: bool) -> Result<(), Error>;
    fn flush_status(&mut self) -> u32;
    fn errors(&mut self) -> HardwareError;
    fn clear_control(&mut self) -> u32;
    fn interrupt_status(&mut self) -> u32;
    fn acknowledge(&mut self, value: u32);
    fn configure(&mut self, material: &Material<'_>);
    fn address(&mut self, slot: Slot, block: usize);
    fn data(&mut self, index: usize, word: u32);
    fn submit(&mut self, value: u32);
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum State {
    Free,
    Reserved,
    Loaded,
}
pub(super) struct Engine<R> {
    pub(super) io: R,
    slots: [State; 10],
    fault: Option<Error>,
}
impl<R: Registers> Engine<R> {
    pub(super) fn new(io: R) -> Self {
        Self { io, slots: [State::Free; 10], fault: None }
    }
    pub(super) fn snapshot(&self) -> Snapshot {
        let mut reserved = 0;
        let mut loaded = 0;
        for (index, state) in self.slots.iter().enumerate() {
            if *state != State::Free {
                reserved |= 1 << index;
            }
            if *state == State::Loaded {
                loaded |= 1 << index;
            }
        }
        Snapshot { reserved, loaded, fault: self.fault }
    }
    fn fail<T>(&mut self, error: Error) -> Result<T, Error> {
        self.fault = Some(error);
        Err(error)
    }
    fn available(&self) -> Result<(), Error> {
        self.fault.map_or(Ok(()), Err)
    }
    fn errors(&mut self) -> Result<(), Error> {
        let errors = self.io.errors();
        if errors.loader != 0 || errors.controller != 0 {
            return self.fail(Error::Hardware(errors));
        }
        Ok(())
    }
    fn flush_idle(&mut self) -> Result<(), Error> {
        let value = self.io.flush_status();
        if value & 7 != 0 {
            return self.fail(Error::FlushStatus(value));
        }
        Ok(())
    }
    fn select(&mut self, slot: Slot) -> Result<(), Error> {
        if let Err(error) = self.io.select(slot) {
            return self.fail(error);
        }
        Ok(())
    }
    fn command(&mut self, slot: Slot, lock: bool) -> Result<(), Error> {
        if let Err(error) = self.io.slot_command(slot, lock) {
            return self.fail(error);
        }
        Ok(())
    }
    fn owned(&mut self, slot: Slot) -> Result<(), Error> {
        self.select(slot)?;
        let owner = self.io.owner();
        if owner != 1 {
            return self.fail(Error::Ownership(owner));
        }
        Ok(())
    }
    fn acknowledge(&mut self) -> Result<(), Error> {
        let raw = self.io.interrupt_status();
        // Bit14 is our clear-route event; bits4..8 are the source's index, not
        // another completion. Never replay common-route/unknown pending bits.
        if raw & !0x41f0 != 0 {
            return self.fail(Error::UnexpectedInterrupt(raw));
        }
        self.io.acknowledge(raw | (1 << 14));
        let after = self.io.interrupt_status();
        if after & !0x41f0 != 0 {
            return self.fail(Error::UnexpectedInterrupt(after));
        }
        if after & (1 << 14) != 0 {
            return self.fail(Error::Acknowledge);
        }
        Ok(())
    }
    pub(super) fn load(
        &mut self,
        slot: Slot,
        material: Material<'_>,
        attempts: NonZeroU32,
        delay: &mut impl DelayNs,
    ) -> Result<(), Error> {
        self.available()?;
        if !material.valid_parity() {
            return Err(Error::InvalidParity);
        }
        if self.slots[slot.index()] != State::Free {
            return Err(Error::Reserved);
        }
        self.errors()?;
        self.flush_idle()?;
        if self.io.clear_control() & 1 != 0 {
            return self.fail(Error::LoaderBusy);
        }
        self.select(slot)?;
        let owner = self.io.owner();
        if owner != 0 {
            return Err(Error::SlotBusy(owner));
        }
        // Reserve before command publication; any failure thereafter retains it.
        self.slots[slot.index()] = State::Reserved;
        self.command(slot, true)?;
        self.owned(slot)?;
        self.acknowledge()?;
        self.io.configure(&material);
        for block in 0..material.blocks() {
            self.errors()?;
            self.owned(slot)?;
            let previous = self.io.clear_control();
            if previous & 1 != 0 {
                return self.fail(Error::LoaderBusy);
            }
            self.io.address(slot, material.half(block));
            for word in 0..4 {
                self.io.data(word, material.word(block, word));
            }
            let command = if slot.hmac() {
                ((block as u32) << 13) | 1
            } else {
                (previous & !13) | (material.size_code() << 2) | 1
            };
            self.io.submit(command);
            let mut complete = false;
            for attempt in 0..attempts.get() {
                self.errors()?;
                if self.io.clear_control() & 1 == 0 {
                    complete = true;
                    break;
                }
                if attempt + 1 < attempts.get() {
                    delay.delay_us(1);
                }
            }
            if !complete {
                return self.fail(Error::LoadTimeout);
            }
            self.errors()?;
            self.owned(slot)?;
            self.acknowledge()?;
        }
        // Unlike the SDK, no staging writes at all are made after an error or
        // timeout. Only qualified idle/ACK/ownership permits this success cleanup.
        self.errors()?;
        self.owned(slot)?;
        for word in 0..4 {
            self.io.data(word, 0);
        }
        self.slots[slot.index()] = State::Loaded;
        Ok(())
    }
    pub(super) fn validate(&mut self, slot: Slot) -> Result<(), Error> {
        self.available()?;
        if self.slots[slot.index()] != State::Loaded {
            return Err(Error::NotLoaded);
        }
        self.errors()?;
        self.flush_idle()?;
        self.owned(slot)
    }
    pub(super) fn release(&mut self, slot: Slot, attempts: NonZeroU32, delay: &mut impl DelayNs) -> Result<(), Error> {
        self.validate(slot)?;
        if self.io.clear_control() & 1 != 0 {
            return self.fail(Error::LoaderBusy);
        }
        self.slots[slot.index()] = State::Reserved;
        self.command(slot, false)?;
        for attempt in 0..attempts.get() {
            self.errors()?;
            let status = self.io.flush_status();
            if status & 6 != 0 {
                return self.fail(Error::FlushStatus(status));
            }
            if status & 1 == 0 {
                self.select(slot)?;
                let owner = self.io.owner();
                if owner != 0 {
                    return self.fail(Error::ReleaseUnconfirmed(owner));
                }
                self.errors()?;
                self.slots[slot.index()] = State::Free;
                return Ok(());
            }
            if attempt + 1 < attempts.get() {
                delay.delay_us(1);
            }
        }
        self.fail(Error::ReleaseTimeout)
    }
}

pub(super) struct PacRegisters<'a> {
    pub(super) registers: &'a pac::km::RegisterBlock,
}
impl Registers for PacRegisters<'_> {
    fn select(&mut self, slot: Slot) -> Result<(), Error> {
        if self.registers.slot_select().read().bits() & (1 << 16) != 0 {
            return Err(Error::UnsupportedSelection);
        }
        self.registers
            .slot_select()
            .modify(|_, w| unsafe { w.slot().bits(slot.number() as u16).hmac().bit(slot.hmac()) });
        Ok(())
    }
    fn owner(&mut self) -> u8 {
        self.registers.slot_owner().read().owner().bits()
    }
    fn slot_command(&mut self, slot: Slot, lock: bool) -> Result<(), Error> {
        let previous = self.registers.ree_slot_read().read().bits();
        if previous & (1 << 16) != 0 {
            return Err(Error::UnsupportedSelection);
        }
        let value = (previous & !0x10_83ff) | slot.number() | (u32::from(slot.hmac()) << 15) | (u32::from(lock) << 20);
        unsafe {
            self.registers.ree_slot_command().write_with_zero(|w| w.value().bits(value));
        }
        Ok(())
    }
    fn flush_status(&mut self) -> u32 {
        self.registers.ree_flush_status().read().bits()
    }
    fn errors(&mut self) -> HardwareError {
        HardwareError {
            loader: self.registers.loader_error().read().bits(),
            controller: self.registers.controller_error().read().bits(),
        }
    }
    fn clear_control(&mut self) -> u32 {
        self.registers.clear_control().read().bits()
    }
    fn interrupt_status(&mut self) -> u32 {
        self.registers.interrupt_status().read().bits()
    }
    fn acknowledge(&mut self, value: u32) {
        unsafe {
            self.registers.interrupt_ack().write_with_zero(|w| w.value().bits(value));
        }
    }
    fn configure(&mut self, material: &Material<'_>) {
        let (engine, encrypt, decrypt) = material.config();
        self.registers.key_config().modify(|_, w| unsafe {
            w.port().bits(1).engine().bits(engine).encrypt().bit(encrypt).decrypt().bit(decrypt)
        });
        // Application REE keys and ordinary nonsecure buffers only. This is
        // working-key policy; no global security/OTP/permanent control is touched.
        self.registers.key_security().modify(|_, w| {
            w.secure_key()
                .clear_bit()
                .same_master_only()
                .clear_bit()
                .source_nonsecure()
                .set_bit()
                .source_secure()
                .clear_bit()
                .destination_nonsecure()
                .set_bit()
                .destination_secure()
                .clear_bit()
        });
    }
    fn address(&mut self, slot: Slot, block: usize) {
        let address = if slot.hmac() { slot.number() } else { slot.number() * 2 + block as u32 };
        self.registers.key_address().modify(|_, w| unsafe { w.address().bits(address as u16) });
    }
    fn data(&mut self, index: usize, word: u32) {
        unsafe {
            self.registers.data_in(index).write_with_zero(|w| w.value().bits(word));
        }
    }
    fn submit(&mut self, value: u32) {
        unsafe {
            self.registers.clear_submit().write_with_zero(|w| w.value().bits(value));
        }
    }
}
