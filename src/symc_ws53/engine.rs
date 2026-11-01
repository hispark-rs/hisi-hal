use super::*;

pub(super) trait Key {
    fn validate(&self) -> Result<(), km::Error>;
}
impl Key for km::LoadedKey<'_> {
    fn validate(&self) -> Result<(), km::Error> {
        self.validate()
    }
}
pub(super) trait Registers {
    fn owner(&mut self, ch: Channel) -> u8;
    fn claim(&mut self, ch: Channel);
    fn errors(&mut self) -> HardwareError;
    fn mask_irq(&mut self, ch: Channel);
    fn clear_done(&mut self, ch: Channel) -> bool;
    fn ack_clear(&mut self, ch: Channel);
    fn request_clear(&mut self, ch: Channel);
    fn done(&mut self, ch: Channel, output: bool) -> bool;
    fn ack_done(&mut self, ch: Channel, output: bool);
    fn ring(&mut self, ch: Channel) -> [u8; 4];
    fn configure(&mut self, ch: Channel, config: Configuration, table: u32);
    fn submit(&mut self, ch: Channel, output: bool);
    fn iv(&mut self, ch: Channel) -> [u8; 16];
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) enum State {
    New,
    Running,
    Complete(Completion),
    Retained(Error),
}
pub(super) struct Engine<R, K> {
    pub io: R,
    pub key: K,
    pub channel: Channel,
    pub config: Configuration,
    pub length: BlockLength,
    pub table: u32,
    pub state: State,
}
impl<R: Registers, K: Key> Engine<R, K> {
    fn retain<T>(&mut self, error: Error) -> Result<T, Error> {
        self.state = State::Retained(error);
        Err(error)
    }
    fn key_healthy(&mut self) -> Result<(), Error> {
        if let Err(e) = self.key.validate() {
            return self.retain(Error::Key(e));
        }
        Ok(())
    }
    fn errors(&mut self) -> Result<(), Error> {
        let errors = self.io.errors();
        if errors.present() {
            return self.retain(Error::Hardware(errors));
        }
        Ok(())
    }
    fn healthy(&mut self) -> Result<(), Error> {
        if self.io.owner(self.channel) != 1 {
            return self.retain(Error::OwnershipLost);
        }
        self.errors()?;
        self.key_healthy()
    }
    fn ack_clear(&mut self) -> Result<(), Error> {
        self.io.ack_clear(self.channel);
        if self.io.clear_done(self.channel) {
            return self.retain(Error::Acknowledge);
        }
        Ok(())
    }
    fn ack_pending(&mut self, output: bool) -> Result<(), Error> {
        if self.io.done(self.channel, output) {
            self.io.ack_done(self.channel, output);
            if self.io.done(self.channel, output) {
                return self.retain(Error::Acknowledge);
            }
        }
        Ok(())
    }
    pub fn start(&mut self, attempts: NonZeroU16, delay: &mut impl DelayNs) -> Result<(), Error> {
        if self.state != State::New {
            return Err(Error::AlreadyStarted);
        }
        self.key_healthy()?;
        self.errors()?;
        if self.io.owner(self.channel) != 0 {
            return self.retain(Error::Busy);
        }
        self.io.claim(self.channel);
        if self.io.owner(self.channel) != 1 {
            return self.retain(Error::Busy);
        }
        self.io.mask_irq(self.channel);
        if self.io.clear_done(self.channel) {
            self.ack_clear()?;
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
        self.ack_clear()?;
        self.ack_pending(false)?;
        self.ack_pending(true)?;
        if self.io.ring(self.channel) != [0; 4] {
            return self.retain(Error::Ring);
        }
        self.healthy()?;
        memory::publish_fence();
        self.io.configure(self.channel, self.config, self.table);
        self.healthy()?;
        memory::publish_fence();
        self.io.submit(self.channel, true);
        // Output publication alone may already permit a fetch: never unwind
        // ownership on an error here, and never publish input after key loss.
        self.healthy()?;
        memory::publish_fence();
        self.io.submit(self.channel, false);
        self.state = State::Running;
        Ok(())
    }
    pub fn poll(&mut self) -> Result<Option<Completion>, Error> {
        match self.state {
            State::New => return Err(Error::NotStarted),
            State::Retained(e) => return Err(e),
            State::Complete(c) => return Ok(Some(c)),
            State::Running => (),
        }
        self.healthy()?;
        if !self.io.done(self.channel, true) {
            return Ok(None);
        }
        let [iw, ir, ow, or] = self.io.ring(self.channel);
        if iw != 1 || ow != 1 || ir > 1 || or > 1 {
            return self.retain(Error::Ring);
        }
        if ir != 1 || or != 1 {
            return Ok(None);
        }
        self.healthy()?;
        self.ack_pending(true)?;
        self.ack_pending(false)?;
        self.healthy()?;
        let iv = self.io.iv(self.channel);
        self.healthy()?;
        let completion = Completion { bytes: self.length.bytes(), iv };
        self.state = State::Complete(completion);
        Ok(Some(completion))
    }
    pub fn wait(&mut self, attempts: NonZeroU16, delay: &mut impl DelayNs) -> Result<Completion, Error> {
        for i in 0..attempts.get() {
            if let Some(c) = self.poll()? {
                return Ok(c);
            }
            if i + 1 < attempts.get() {
                delay.delay_us(1);
            }
        }
        self.retain(Error::Timeout)
    }
    pub fn cancel(&mut self) {
        if matches!(self.state, State::New | State::Running) {
            self.state = State::Retained(Error::Cancelled);
        }
    }
}

pub(super) struct PacRegisters<'a> {
    pub registers: &'a pac::spacc::RegisterBlock,
}
impl Registers for PacRegisters<'_> {
    fn owner(&mut self, ch: Channel) -> u8 {
        let r = self.registers.symc_lock().read();
        match ch {
            Channel::One => r.channel1_owner().bits(),
            Channel::Two => r.channel2_owner().bits(),
        }
    }
    fn claim(&mut self, ch: Channel) {
        self.registers.symc_lock().modify(|_, w| {
            // SAFETY: fixed REE owner1, shared RMW excluded by constructor.
            unsafe {
                match ch {
                    Channel::One => w.channel1_owner().bits(1),
                    Channel::Two => w.channel2_owner().bits(1),
                }
            }
        });
    }
    fn errors(&mut self) -> HardwareError {
        let r = self.registers;
        HardwareError {
            tee_cause: r.hash_error(0).symc_cause().read().bits(),
            tee_status: r.hash_error(0).symc_status().read().bits(),
            ree_cause: r.hash_error(1).symc_cause().read().bits(),
            ree_status: r.hash_error(1).symc_status().read().bits(),
        }
    }
    fn mask_irq(&mut self, ch: Channel) {
        self.registers.symc_output_interrupt_enable().modify(|_, w| match ch {
            Channel::One => w.channel1().clear_bit(),
            Channel::Two => w.channel2().clear_bit(),
        });
        self.registers.symc_input_interrupt_enable().modify(|_, w| match ch {
            Channel::One => w.channel1().clear_bit(),
            Channel::Two => w.channel2().clear_bit(),
        });
    }
    fn clear_done(&mut self, ch: Channel) -> bool {
        self.registers.symc_clear_status().read().bits() & ch.mask() != 0
    }
    fn ack_clear(&mut self, ch: Channel) {
        // SAFETY: qualified selected-bit command, no replay of other channels.
        unsafe {
            self.registers.symc_clear_ack().write_with_zero(|w| w.value().bits(ch.mask()));
        }
    }
    fn request_clear(&mut self, ch: Channel) {
        // SAFETY: source command view, selected channel only, not an ordinary RMW.
        unsafe {
            self.registers.symc_clear_request().write_with_zero(|w| w.value().bits(ch.mask()));
        }
    }
    fn done(&mut self, ch: Channel, output: bool) -> bool {
        let r = self.registers;
        (if output {
            r.symc_output_interrupt_status().read().bits()
        } else {
            r.symc_input_interrupt_status().read().bits()
        }) & ch.mask()
            != 0
    }
    fn ack_done(&mut self, ch: Channel, output: bool) {
        // SAFETY: explicit selected-bit ACK contract, no inferred writable status.
        unsafe {
            if output {
                self.registers.symc_output_interrupt_ack().write_with_zero(|w| w.value().bits(ch.mask()));
            } else {
                self.registers.symc_input_interrupt_ack().write_with_zero(|w| w.value().bits(ch.mask()));
            }
        }
    }
    fn ring(&mut self, ch: Channel) -> [u8; 4] {
        let i = self.registers.symc_channel(ch.index());
        let o = self.registers.symc_output(ch.index());
        [
            i.write_pointer().read().pointer().bits(),
            i.read_pointer().read().pointer().bits(),
            o.write_pointer().read().pointer().bits(),
            o.read_pointer().read().pointer().bits(),
        ]
    }
    fn configure(&mut self, ch: Channel, c: Configuration, table: u32) {
        let i = self.registers.symc_channel(ch.index());
        let o = self.registers.symc_output(ch.index());
        // SAFETY: closed validated configuration; table spans checked 96-byte RAM
        // allocation. Only known fields change and reserved bits are preserved.
        unsafe {
            i.control().modify(|_, w| w.source_security().bits(10).destination_security().bits(10).enable().set_bit());
            i.output_control().modify(|_, w| w.copy_mode().bits(0));
            i.key_control().modify(|_, w| {
                w.keyslot()
                    .bits(c.slot as u16)
                    .algorithm()
                    .bits(c.algorithm)
                    .mode()
                    .bits(c.mode.code())
                    .key_length()
                    .bits(c.key_length)
                    .data_width()
                    .bits(0)
                    .decrypt()
                    .bit(c.direction == Direction::Decrypt)
            });
            i.node_base_high().write_with_zero(|w| w.value().bits(0));
            i.node_base_low().write_with_zero(|w| w.address().bits(table));
            i.node_depth().modify(|_, w| w.length().bits(2));
            o.node_base_high().write_with_zero(|w| w.value().bits(0));
            o.node_base_low().write_with_zero(|w| w.address().bits(table + 64));
            o.node_depth().modify(|_, w| w.length().bits(2));
        }
    }
    fn submit(&mut self, ch: Channel, output: bool) {
        // SAFETY: empty two-entry ring checked before configure, one descriptor
        // published. Preserve high reserved read bits, never replay a pointer.
        unsafe {
            if output {
                let r = self.registers.symc_output(ch.index());
                let prior = r.write_pointer().read().bits();
                r.submit().write_with_zero(|w| w.value().bits((prior & !0xff) | 1));
            } else {
                let r = self.registers.symc_channel(ch.index());
                let prior = r.write_pointer().read().bits();
                r.submit().write_with_zero(|w| w.value().bits((prior & !0xff) | 1));
            }
        }
    }
    fn iv(&mut self, ch: Channel) -> [u8; 16] {
        let r = self.registers.symc_iv(ch.index());
        let words = [
            r.word0().read().value().bits(),
            r.word1().read().value().bits(),
            r.word2().read().value().bits(),
            r.word3().read().value().bits(),
        ];
        let mut bytes = [0; 16];
        for (word, dest) in words.iter().zip(bytes.as_chunks_mut::<4>().0) {
            dest.copy_from_slice(&word.to_le_bytes());
        }
        bytes
    }
}
