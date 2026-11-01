//! Host protocol tests, not cryptographic computations or silicon DMA proof.
use super::*;
use engine::{Key, Registers};
use km::{Algorithm, CipherSlot, HmacAlgorithm, HmacSlot, Parity, Slot, Usage};
use std::{cell::Cell, rc::Rc, vec, vec::Vec};

// A positive implementation makes inference ambiguous and compilation fail.
macro_rules! absent {
    ($ty:ty, $bound:path) => {
        const _: fn() = || {
            trait Ambiguous<A> {
                fn check() {}
            }
            impl<T: ?Sized> Ambiguous<()> for T {}
            struct Present;
            impl<T: ?Sized + $bound> Ambiguous<Present> for T {}
            let _ = <$ty as Ambiguous<_>>::check;
        };
    };
}
absent!(Transfer<'static>, Copy);
absent!(Transfer<'static>, Clone);
absent!(Transfer<'static>, Send);
absent!(Transfer<'static>, Sync);
absent!(Resources<'static>, Copy);
absent!(Resources<'static>, Clone);
absent!(Resources<'static>, Send);
absent!(Resources<'static>, Sync);
absent!(Buffer, Copy);
absent!(Buffer, Clone);
absent!(Buffer, Send);
absent!(Buffer, Sync);
absent!(Table, Copy);
absent!(Table, Clone);
absent!(Table, Send);
absent!(Table, Sync);

const CLEAN: HardwareError = HardwareError { tee_cause: 0, tee_status: 0, ree_cause: 0, ree_status: 0 };
#[derive(Default)]
struct Delay(Vec<u32>);
impl DelayNs for Delay {
    fn delay_ns(&mut self, ns: u32) {
        self.0.push(ns);
    }
}
struct FakeKey {
    calls: Rc<Cell<usize>>,
    failure: Rc<Cell<Option<km::Error>>>,
}
impl Key for FakeKey {
    fn validate(&self) -> Result<(), km::Error> {
        self.calls.set(self.calls.get() + 1);
        self.failure.get().map_or(Ok(()), Err)
    }
}
struct Fake {
    owner: [u8; 2],
    clear: u8,
    done: [u8; 2],
    irq: [u8; 2],
    rings: [[u8; 4]; 2],
    errors: HardwareError,
    clear_pending: bool,
    clear_after: usize,
    clear_polls: usize,
    reject_claim: bool,
    clear_ack_stuck: bool,
    done_ack_stuck: [bool; 2],
    fault_at: Option<(&'static str, u8)>,
    key_failure: Rc<Cell<Option<km::Error>>>,
    log: Vec<(&'static str, usize)>,
    config: Option<Configuration>,
}
impl Fake {
    fn mark(&mut self, event: &'static str, ch: Channel) -> usize {
        let i = ch.index();
        self.log.push((event, i));
        if let Some((at, kind)) = self.fault_at
            && event == at
        {
            match kind {
                0 => self.owner[i] = 2,
                1 => self.errors.ree_cause = 0x80000000,
                2 => self.key_failure.set(Some(km::Error::Ownership(2))),
                _ => unreachable!(),
            }
        }
        i
    }
    fn count(&self, event: &str) -> usize {
        self.log.iter().filter(|(e, _)| *e == event).count()
    }
    fn complete(&mut self, ch: Channel) {
        self.rings[ch.index()] = [1; 4];
        self.done = [3, 3];
    }
}
impl Registers for Fake {
    fn owner(&mut self, ch: Channel) -> u8 {
        let i = self.mark("owner", ch);
        self.owner[i]
    }
    fn claim(&mut self, ch: Channel) {
        let i = self.mark("claim", ch);
        if !self.reject_claim {
            self.owner[i] = 1;
        }
    }
    fn errors(&mut self) -> HardwareError {
        self.log.push(("errors", 0));
        self.errors
    }
    fn mask_irq(&mut self, ch: Channel) {
        let i = self.mark("mask", ch);
        for irq in &mut self.irq {
            *irq &= !(1 << i);
        }
    }
    fn clear_done(&mut self, ch: Channel) -> bool {
        let i = self.mark("clear?", ch);
        if self.clear_pending {
            self.clear_polls += 1;
            if self.clear_after > 0 && self.clear_polls >= self.clear_after {
                self.clear |= 1 << i;
            }
        }
        self.clear & (1 << i) != 0
    }
    fn ack_clear(&mut self, ch: Channel) {
        let i = self.mark("ack-clear", ch);
        if !self.clear_ack_stuck {
            self.clear &= !(1 << i);
            self.clear_pending = false;
        }
    }
    fn request_clear(&mut self, ch: Channel) {
        self.mark("request-clear", ch);
        self.clear_pending = true;
    }
    fn done(&mut self, ch: Channel, output: bool) -> bool {
        let i = self.mark(if output { "out?" } else { "in?" }, ch);
        self.done[output as usize] & (1 << i) != 0
    }
    fn ack_done(&mut self, ch: Channel, output: bool) {
        let i = self.mark(if output { "ack-out" } else { "ack-in" }, ch);
        if !self.done_ack_stuck[output as usize] {
            self.done[output as usize] &= !(1 << i);
        }
    }
    fn ring(&mut self, ch: Channel) -> [u8; 4] {
        let i = self.mark("ring", ch);
        self.rings[i]
    }
    fn configure(&mut self, ch: Channel, config: Configuration, table: u32) {
        self.mark("configure", ch);
        assert_eq!(table, 0x20050000);
        self.config = Some(config);
    }
    fn submit(&mut self, ch: Channel, output: bool) {
        let i = self.mark(if output { "submit-out" } else { "submit-in" }, ch);
        self.rings[i][if output { 2 } else { 0 }] = 1;
    }
    fn iv(&mut self, ch: Channel) -> [u8; 16] {
        self.mark("iv", ch);
        [0x5a; 16]
    }
}
fn config() -> Configuration {
    Configuration::new(
        Slot::Cipher(CipherSlot::Zero),
        Algorithm::Aes256,
        Parity::Even,
        Some(Usage::Both),
        Direction::Encrypt,
        Mode::Cbc([0x17; 16]),
    )
    .unwrap()
}
fn engine(ch: Channel) -> Engine<Fake, FakeKey> {
    let failure = Rc::new(Cell::new(None));
    Engine {
        io: Fake {
            owner: [0; 2],
            clear: 0,
            done: [0; 2],
            irq: [0xff; 2],
            rings: [[0; 4]; 2],
            errors: CLEAN,
            clear_pending: false,
            clear_after: 1,
            clear_polls: 0,
            reject_claim: false,
            clear_ack_stuck: false,
            done_ack_stuck: [false; 2],
            fault_at: None,
            key_failure: failure.clone(),
            log: vec![],
            config: None,
        },
        key: FakeKey { calls: Rc::new(Cell::new(0)), failure },
        channel: ch,
        config: config(),
        length: BlockLength::from_bytes(48).unwrap(),
        table: 0x20050000,
        state: State::New,
    }
}
fn attempts(n: u16) -> NonZeroU16 {
    NonZeroU16::new(n).unwrap()
}
fn retained(e: &mut Engine<Fake, FakeKey>, error: Error) {
    let log = e.io.log.clone();
    let calls = e.key.calls.get();
    let mut delay = Delay::default();
    assert_eq!(e.poll(), Err(error));
    assert_eq!(e.wait(attempts(4), &mut delay), Err(error));
    assert_eq!(e.start(attempts(4), &mut delay), Err(Error::AlreadyStarted));
    e.cancel();
    assert_eq!(e.io.log, log);
    assert_eq!(e.key.calls.get(), calls);
    assert!(delay.0.is_empty());
}
#[test]
fn block_lengths_and_key_compatibility_are_checked_without_truncation() {
    for n in 0..=0x50010 {
        assert_eq!(BlockLength::from_bytes(n).is_some(), n > 0 && n <= 0x50000 && n % 16 == 0);
    }
    assert!(BlockLength::from_bytes(usize::MAX).is_none());
    let modes = [Mode::Ecb, Mode::Cbc([1; 16]), Mode::Ctr([2; 16]), Mode::Ofb([3; 16]), Mode::Cfb128([4; 16])];
    let algorithms = [Algorithm::Aes128, Algorithm::Aes192, Algorithm::Aes256, Algorithm::Sm4];
    let mut valid = 0;
    for slot in CipherSlot::ALL {
        for alg in algorithms {
            for parity in [Parity::Even, Parity::Odd] {
                for usage in [Usage::Encrypt, Usage::Decrypt, Usage::Both] {
                    for direction in [Direction::Encrypt, Direction::Decrypt] {
                        for mode in modes {
                            let got = Configuration::new(Slot::Cipher(slot), alg, parity, Some(usage), direction, mode);
                            if parity == Parity::Odd && matches!(alg, Algorithm::Aes192 | Algorithm::Aes256) {
                                assert_eq!(got, Err(PrepareError::KeyParity));
                            } else if matches!(
                                (usage, direction),
                                (Usage::Encrypt, Direction::Decrypt) | (Usage::Decrypt, Direction::Encrypt)
                            ) {
                                assert_eq!(got, Err(PrepareError::KeyUsage));
                            } else {
                                let c = got.unwrap();
                                assert_eq!(c.slot, slot);
                                valid += 1;
                            }
                        }
                    }
                }
            }
        }
    }
    assert_eq!(valid, 960);
    for slot in [Slot::Cipher(CipherSlot::Zero), Slot::Hmac(HmacSlot::Zero), Slot::Hmac(HmacSlot::One)] {
        assert_eq!(
            Configuration::new(
                slot,
                Algorithm::Hmac(HmacAlgorithm::Sha256),
                Parity::Even,
                None,
                Direction::Encrypt,
                Mode::Ecb
            ),
            Err(PrepareError::KeyType)
        );
    }
    assert_eq!(
        Configuration::new(
            Slot::Cipher(CipherSlot::Zero),
            Algorithm::Aes128,
            Parity::Even,
            None,
            Direction::Encrypt,
            Mode::Ecb
        ),
        Err(PrepareError::KeyUsage)
    );
}
#[test]
fn both_rings_and_events_complete_before_cached_metadata() {
    for ch in [Channel::One, Channel::Two] {
        let mut e = engine(ch);
        let mut delay = Delay::default();
        let other = 1 - ch.index();
        e.io.owner[other] = 2;
        e.io.clear = 3;
        e.io.done = [3; 2];
        assert_eq!(e.poll(), Err(Error::NotStarted));
        assert!(e.io.log.is_empty());
        e.start(attempts(3), &mut delay).unwrap();
        assert_eq!(e.io.owner[other], 2);
        assert_eq!(e.io.irq, [!(1 << ch.index()); 2]);
        assert_eq!(e.io.clear, 1 << other);
        assert_eq!(e.io.done, [1 << other; 2]);
        assert_eq!(e.io.config, Some(config()));
        let order: Vec<_> = e.io.log.iter().filter(|(n, _)| n.starts_with("submit")).map(|(n, _)| *n).collect();
        assert_eq!(order, ["submit-out", "submit-in"]);
        assert_eq!(e.poll(), Ok(None));
        e.io.done[1] = 3;
        for partial in [[1, 0, 1, 0], [1, 1, 1, 0], [1, 0, 1, 1]] {
            e.io.rings[ch.index()] = partial;
            assert_eq!(e.poll(), Ok(None));
        }
        e.io.complete(ch);
        let c = Completion { bytes: 48, iv: [0x5a; 16] };
        assert_eq!(e.poll(), Ok(Some(c)));
        assert_eq!(e.io.count("iv"), 1);
        assert_eq!(e.io.done, [1 << other; 2]);
        assert_eq!(e.io.owner[ch.index()], 1);
        let log = e.io.log.clone();
        let calls = e.key.calls.get();
        assert_eq!(e.wait(attempts(4), &mut delay), Ok(c));
        e.cancel();
        assert_eq!(e.io.log, log);
        assert_eq!(e.key.calls.get(), calls);
        assert!(delay.0.is_empty());
    }
}
#[test]
fn clear_budget_and_timeout_preserve_owner_without_publication() {
    for ch in [Channel::One, Channel::Two] {
        for budget in [1, 2, 5] {
            for ready in [0, 1, 2, 5, 6] {
                let mut e = engine(ch);
                let mut delay = Delay::default();
                e.io.clear_after = ready;
                let result = e.start(attempts(budget), &mut delay);
                let polls = if ready == 0 { budget as usize } else { ready.min(budget as usize) };
                assert_eq!(delay.0, vec![1000; polls - 1]);
                if ready > 0 && ready <= budget as usize {
                    assert_eq!(result, Ok(()));
                } else {
                    assert_eq!(result, Err(Error::ClearTimeout));
                    assert_eq!(e.io.owner[ch.index()], 1);
                    assert_eq!(e.io.count("configure"), 0);
                    assert_eq!(e.io.count("submit-out"), 0);
                    retained(&mut e, Error::ClearTimeout);
                }
            }
        }
    }
}
#[test]
fn key_fault_and_busy_claim_never_touch_submission() {
    for ch in [Channel::One, Channel::Two] {
        let mut e = engine(ch);
        e.key.failure.set(Some(km::Error::Reserved));
        assert_eq!(e.start(attempts(2), &mut Delay::default()), Err(Error::Key(km::Error::Reserved)));
        assert!(e.io.log.is_empty());
        retained(&mut e, Error::Key(km::Error::Reserved));
        for owner in 0..16 {
            let mut e = engine(ch);
            e.io.owner[ch.index()] = owner;
            e.io.reject_claim = true;
            assert_eq!(e.start(attempts(2), &mut Delay::default()), Err(Error::Busy));
            assert_eq!(e.io.count("claim"), usize::from(owner == 0));
            assert_eq!(e.io.count("mask"), 0);
            retained(&mut e, Error::Busy);
        }
    }
}
#[test]
fn stale_ack_and_nonempty_rings_stop_before_configure() {
    for ch in [Channel::One, Channel::Two] {
        for scenario in 0..8 {
            let mut e = engine(ch);
            let expected = match scenario {
                0 | 1 => {
                    e.io.clear = if scenario == 0 { 3 } else { 0 };
                    e.io.clear_ack_stuck = true;
                    Error::Acknowledge
                }
                2 | 3 => {
                    let i = scenario - 2;
                    e.io.done[i] = 3;
                    e.io.done_ack_stuck[i] = true;
                    Error::Acknowledge
                }
                _ => {
                    e.io.rings[ch.index()][scenario - 4] = 1;
                    Error::Ring
                }
            };
            assert_eq!(e.start(attempts(2), &mut Delay::default()), Err(expected));
            assert_eq!(e.io.count("configure"), 0);
            retained(&mut e, expected);
        }
    }
}
#[test]
fn faults_at_publication_and_iv_boundaries_never_continue_writing() {
    for ch in [Channel::One, Channel::Two] {
        for at in ["configure", "submit-out", "iv"] {
            for kind in 0..3 {
                let mut e = engine(ch);
                e.io.fault_at = Some((at, kind));
                let expected = match kind {
                    0 => Error::OwnershipLost,
                    1 => Error::Hardware(HardwareError { ree_cause: 0x80000000, ..CLEAN }),
                    _ => Error::Key(km::Error::Ownership(2)),
                };
                if at == "iv" {
                    e.start(attempts(1), &mut Delay::default()).unwrap();
                    e.io.complete(ch);
                    assert_eq!(e.poll(), Err(expected));
                } else {
                    assert_eq!(e.start(attempts(1), &mut Delay::default()), Err(expected));
                    assert_eq!(e.io.count("submit-in"), 0);
                    assert_eq!(e.io.count("submit-out"), usize::from(at == "submit-out"));
                }
                retained(&mut e, expected);
            }
        }
    }
}
#[test]
fn completion_faults_and_wait_boundaries_never_return_output_memory() {
    for ch in [Channel::One, Channel::Two] {
        for scenario in 0..10 {
            let mut e = engine(ch);
            e.start(attempts(1), &mut Delay::default()).unwrap();
            e.io.complete(ch);
            let expected = match scenario {
                0 | 1 => {
                    e.io.done_ack_stuck[scenario] = true;
                    Error::Acknowledge
                }
                2..=5 => {
                    e.io.rings[ch.index()][scenario - 2] = 7;
                    Error::Ring
                }
                6 => {
                    e.io.owner[ch.index()] = 2;
                    Error::OwnershipLost
                }
                7 => {
                    e.key.failure.set(Some(km::Error::Ownership(2)));
                    Error::Key(km::Error::Ownership(2))
                }
                _ => {
                    if scenario == 8 {
                        e.io.errors.tee_cause = 1;
                    } else {
                        e.io.errors.ree_cause = 1;
                    }
                    Error::Hardware(e.io.errors)
                }
            };
            assert_eq!(e.poll(), Err(expected));
            assert_eq!(e.io.count("iv"), 0);
            retained(&mut e, expected);
        }
    }
    for n in [1, 2, 7] {
        let mut e = engine(Channel::One);
        let mut delay = Delay::default();
        e.start(attempts(1), &mut delay).unwrap();
        assert_eq!(e.wait(attempts(n), &mut delay), Err(Error::Timeout));
        assert_eq!(delay.0, vec![1000; n as usize - 1]);
        retained(&mut e, Error::Timeout);
    }
}
#[test]
fn cancel_drop_and_forget_do_not_call_either_controller() {
    for started in [false, true] {
        let mut e = engine(Channel::One);
        if started {
            e.start(attempts(1), &mut Delay::default()).unwrap();
        }
        let log = e.io.log.clone();
        let calls = e.key.calls.clone();
        let prior = calls.get();
        e.cancel();
        assert_eq!(e.io.log, log);
        retained(&mut e, Error::Cancelled);
        drop(e);
        assert_eq!(calls.get(), prior);
        let mut e = engine(Channel::Two);
        if started {
            e.start(attempts(1), &mut Delay::default()).unwrap();
        }
        let calls = e.key.calls.clone();
        let prior = calls.get();
        core::mem::forget(e);
        assert_eq!(calls.get(), prior);
    }
}

#[test]
fn pac_changes_only_selected_fields_and_uses_distinct_table_formats() {
    for ch in [Channel::One, Channel::Two] {
        let mut words = vec![0u32; 0x10000 / 4];
        // SAFETY: aligned local RAM backing is large enough for this PAC block;
        // no physical device is accessed. Read-only status views are local bytes.
        let r = unsafe { &*(words.as_mut_ptr().cast::<pac::spacc::RegisterBlock>()) };
        let mut io = PacRegisters { registers: r };
        unsafe {
            r.symc_lock().write_with_zero(|w| w.bits(0xa5a50000));
            r.symc_input_interrupt_enable().write_with_zero(|w| w.bits(u32::MAX));
            r.symc_output_interrupt_enable().write_with_zero(|w| w.bits(u32::MAX));
        }
        io.claim(ch);
        assert_eq!(io.owner(ch), 1);
        assert_eq!(r.symc_lock().read().bits(), 0xa5a50000 | (1 << ((ch.index() + 1) * 4)));
        io.mask_irq(ch);
        assert_eq!(r.symc_input_interrupt_enable().read().bits(), !ch.mask());
        assert_eq!(r.symc_output_interrupt_enable().read().bits(), !ch.mask());
        let i = r.symc_channel(ch.index());
        let o = r.symc_output(ch.index());
        unsafe {
            i.control().write_with_zero(|w| w.bits(u32::MAX));
            i.output_control().write_with_zero(|w| w.bits(u32::MAX));
            i.key_control().write_with_zero(|w| w.bits(u32::MAX));
            i.node_depth().write_with_zero(|w| w.bits(0x123456ff));
            o.node_depth().write_with_zero(|w| w.bits(0x876543ff));
        }
        let mut c = config();
        c.slot = CipherSlot::Seven;
        c.direction = Direction::Decrypt;
        io.configure(ch, c, 0x20050000);
        assert_eq!(i.control().read().bits(), !0xff00 | 0xaa00);
        assert_eq!(i.output_control().read().bits(), !3);
        let mask = 0x1fff01ff;
        let value = 7 | (2 << 16) | (3 << 20) | (3 << 24) | (1 << 28);
        assert_eq!(i.key_control().read().bits(), !mask | value);
        assert_eq!(i.node_base_high().read().bits(), 0);
        assert_eq!(o.node_base_high().read().bits(), 0);
        assert_eq!(i.node_base_low().read().bits(), 0x20050000);
        assert_eq!(o.node_base_low().read().bits(), 0x20050040);
        assert_eq!(i.node_depth().read().bits(), 0x12345602);
        assert_eq!(o.node_depth().read().bits(), 0x87654302);
        unsafe {
            i.submit().write_with_zero(|w| w.value().bits(0xa5a50000));
            o.submit().write_with_zero(|w| w.value().bits(0x5a5a0000));
        }
        io.submit(ch, true);
        io.submit(ch, false);
        assert_eq!(i.write_pointer().read().bits(), 0xa5a50001);
        assert_eq!(o.write_pointer().read().bits(), 0x5a5a0001);
        io.request_clear(ch);
        io.ack_clear(ch);
        io.ack_done(ch, false);
        io.ack_done(ch, true);
        // Verify emitted command words as ordinary RAM, not fictitious ACK semantics.
        assert_eq!(words[0x60 / 4], ch.mask());
        assert_eq!(words[0x8 / 4], ch.mask());
        assert_eq!(words[0x8610 / 4], ch.mask());
        assert_eq!(words[0xc000 / 4], ch.mask());
    }
}
