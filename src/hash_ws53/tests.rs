//! Control-flow fake, not an algorithm model or physical DMA-drain proof.
use super::*;
use std::{vec, vec::Vec};

#[derive(Default)]
struct Delay {
    calls: Vec<u32>,
}
impl DelayNs for Delay {
    fn delay_ns(&mut self, ns: u32) {
        self.calls.push(ns);
    }
}
const CLEAN: HardwareError = HardwareError { tee_cause: 0, tee_status: 0, ree_cause: 0, ree_status: 0 };
struct Fake {
    owners: [u8; 2],
    irq: u8,
    clear: u8,
    done: u8,
    ring: [(u8, u8, u32); 2],
    errors: HardwareError,
    clear_after: usize,
    clear_polls: usize,
    clear_pending: bool,
    reject_claim: bool,
    stuck_clear_ack: bool,
    stuck_done_ack: bool,
    lose_after_configure: bool,
    values: [u32; 16],
    log: Vec<(&'static str, usize)>,
}
impl Default for Fake {
    fn default() -> Self {
        Self {
            owners: [0; 2],
            irq: 0xff,
            clear: 0,
            done: 0,
            ring: [(0, 0, 0); 2],
            errors: CLEAN,
            clear_after: 1,
            clear_polls: 0,
            clear_pending: false,
            reject_claim: false,
            stuck_clear_ack: false,
            stuck_done_ack: false,
            lose_after_configure: false,
            values: [0x67452301; 16],
            log: vec![],
        }
    }
}
impl Fake {
    fn mark(&mut self, name: &'static str, channel: Channel) -> usize {
        let index = channel.index();
        self.log.push((name, index));
        index
    }
    fn complete(&mut self, channel: Channel) {
        self.done |= 1 << channel.index();
        self.ring[channel.index()] = (1, 1, 0);
    }
    fn writes(&self, name: &str) -> usize {
        self.log.iter().filter(|(n, _)| *n == name).count()
    }
}
impl Registers for Fake {
    fn owner(&mut self, ch: Channel) -> u8 {
        let i = self.mark("owner", ch);
        self.owners[i]
    }
    fn claim(&mut self, ch: Channel) {
        let i = self.mark("claim", ch);
        if !self.reject_claim {
            self.owners[i] = 1;
        }
    }
    fn mask_irq(&mut self, ch: Channel) {
        let i = self.mark("mask", ch);
        self.irq &= !(1 << i);
    }
    fn clear_done(&mut self, ch: Channel) -> bool {
        let i = self.mark("clear?", ch);
        if self.clear_pending {
            self.clear_polls += 1;
            if self.clear_after != 0 && self.clear_polls >= self.clear_after {
                self.clear |= 1 << i;
            }
        }
        self.clear & (1 << i) != 0
    }
    fn ack_clear(&mut self, ch: Channel) {
        let i = self.mark("ack-clear", ch);
        if !self.stuck_clear_ack {
            self.clear &= !(1 << i);
            self.clear_pending = false;
        }
    }
    fn request_clear(&mut self, ch: Channel) {
        self.mark("request-clear", ch);
        self.clear_pending = true;
    }
    fn done(&mut self, ch: Channel) -> bool {
        let i = self.mark("done?", ch);
        self.done & (1 << i) != 0
    }
    fn ack_done(&mut self, ch: Channel) {
        let i = self.mark("ack-done", ch);
        if !self.stuck_done_ack {
            self.done &= !(1 << i);
        }
    }
    fn ring(&mut self, ch: Channel) -> (u8, u8, u32) {
        let i = self.mark("ring", ch);
        self.ring[i]
    }
    fn errors(&mut self) -> HardwareError {
        self.log.push(("errors", 0));
        self.errors
    }
    fn configure(&mut self, ch: Channel, algorithm: Algorithm, table: u32) {
        let i = self.mark("configure", ch);
        assert_eq!(algorithm, Algorithm::Sha256);
        assert_eq!(table, 0x20050000);
        if self.lose_after_configure {
            self.owners[i] = 2;
        }
    }
    fn submit(&mut self, ch: Channel) {
        let i = self.mark("submit", ch);
        self.ring[i].0 = 1;
    }
    fn state(&mut self, ch: Channel, words: &mut [u32]) {
        self.mark("state", ch);
        words.copy_from_slice(&self.values[..words.len()]);
    }
}
fn engine(channel: Channel) -> Engine<Fake> {
    Engine { io: Fake::default(), channel, algorithm: Algorithm::Sha256, table_address: 0x20050000, state: State::New }
}
fn attempts(n: u16) -> NonZeroU16 {
    NonZeroU16::new(n).unwrap()
}
fn retained(e: &mut Engine<Fake>, error: Error) {
    let log = e.io.log.clone();
    let mut delay = Delay::default();
    assert_eq!(e.poll(), Err(error));
    assert_eq!(e.wait(attempts(5), &mut delay), Err(error));
    assert_eq!(e.start(attempts(5), &mut delay), Err(Error::AlreadyStarted));
    e.cancel();
    assert_eq!(e.io.log, log);
    assert!(delay.calls.is_empty());
}

#[test]
fn claim_clear_submit_and_cached_result_on_each_channel() {
    for ch in [Channel::One, Channel::Two] {
        let mut e = engine(ch);
        let mut delay = Delay::default();
        let other = 1 - ch.index();
        e.io.owners[other] = 2;
        e.io.clear = 3;
        e.io.done = 3;
        assert_eq!(e.start(attempts(3), &mut delay), Ok(()));
        assert_eq!(e.io.owners[other], 2);
        assert_eq!(e.io.irq, !(1 << ch.index()));
        assert_eq!(e.io.clear, 1 << other);
        assert_eq!(e.io.done, 1 << other);
        assert_eq!(e.io.writes("configure"), 1);
        assert_eq!(e.io.writes("submit"), 1);
        assert!(delay.calls.is_empty());
        assert_eq!(e.poll(), Ok(None));
        e.io.complete(ch);
        let digest = e.poll().unwrap().unwrap();
        assert_eq!(digest.as_bytes(), [1, 0x23, 0x45, 0x67].repeat(8));
        let log = e.io.log.clone();
        assert_eq!(e.poll(), Ok(Some(digest)));
        assert_eq!(e.wait(attempts(1), &mut delay), Ok(digest));
        e.cancel();
        assert_eq!(e.io.log, log);
        assert_eq!(e.io.owners[ch.index()], 1);
        assert_eq!(e.io.done, 1 << other);
        assert_eq!(e.io.writes("state"), 1);
    }
}
#[test]
fn clear_budget_boundaries_never_publish_after_timeout() {
    for ch in [Channel::One, Channel::Two] {
        for budget in [1, 2, 5] {
            for ready in [0, 1, 2, 5, 6] {
                let mut e = engine(ch);
                let mut delay = Delay::default();
                e.io.clear_after = ready;
                let result = e.start(attempts(budget), &mut delay);
                let count = if ready == 0 { budget as usize } else { ready.min(budget as usize) };
                assert_eq!(delay.calls, vec![1000; count - 1]);
                if ready > 0 && ready <= budget as usize {
                    assert_eq!(result, Ok(()));
                } else {
                    assert_eq!(result, Err(Error::ClearTimeout));
                    assert_eq!(e.io.writes("submit"), 0);
                    assert_eq!(e.io.writes("configure"), 0);
                    assert_eq!(e.io.writes("state"), 0);
                    assert_eq!(e.io.owners[ch.index()], 1);
                    retained(&mut e, Error::ClearTimeout);
                }
            }
        }
    }
}
#[test]
fn busy_and_raced_claim_do_not_mask_or_clear() {
    for ch in [Channel::One, Channel::Two] {
        for owner in 0..16 {
            let mut e = engine(ch);
            e.io.owners[ch.index()] = owner;
            e.io.reject_claim = true;
            assert_eq!(e.start(attempts(1), &mut Delay::default()), Err(Error::Busy));
            assert_eq!(e.io.writes("claim"), usize::from(owner == 0));
            assert_eq!(e.io.writes("mask"), 0);
            assert_eq!(e.io.writes("request-clear"), 0);
            retained(&mut e, Error::Busy);
        }
    }
}
#[test]
fn stale_clear_ack_and_nonempty_ring_stop_before_publication() {
    for ch in [Channel::One, Channel::Two] {
        for scenario in 0..5 {
            let mut e = engine(ch);
            let expected = match scenario {
                0 => {
                    e.io.clear = 3;
                    e.io.stuck_clear_ack = true;
                    Error::Acknowledge
                }
                1 => {
                    e.io.stuck_clear_ack = true;
                    Error::Acknowledge
                }
                2 => {
                    e.io.done = 3;
                    e.io.stuck_done_ack = true;
                    Error::Acknowledge
                }
                3 => {
                    e.io.ring[ch.index()] = (0, 1, 0);
                    Error::Ring
                }
                _ => {
                    e.io.ring[ch.index()] = (0, 0, 1);
                    Error::Ring
                }
            };
            assert_eq!(e.start(attempts(2), &mut Delay::default()), Err(expected));
            assert_eq!(e.io.writes("configure"), 0);
            assert_eq!(e.io.writes("submit"), 0);
            retained(&mut e, expected);
        }
    }
}
#[test]
fn errors_before_claim_and_while_running_never_read_digest() {
    for ch in [Channel::One, Channel::Two] {
        for bank in 0..2 {
            for running in [false, true] {
                let mut e = engine(ch);
                if running {
                    e.start(attempts(1), &mut Delay::default()).unwrap();
                    e.io.complete(ch);
                }
                if bank == 0 {
                    e.io.errors.tee_cause = 1 << 13;
                    e.io.errors.tee_status = 1 << 16;
                } else {
                    e.io.errors.ree_cause = 1 << 31;
                    e.io.errors.ree_status = 2 << 16;
                }
                let error = Error::Hardware(e.io.errors);
                if running {
                    assert_eq!(e.poll(), Err(error));
                } else {
                    assert_eq!(e.start(attempts(1), &mut Delay::default()), Err(error));
                    assert_eq!(e.io.writes("claim"), 0);
                }
                assert_eq!(e.io.writes("state"), 0);
                retained(&mut e, error);
            }
        }
    }
}
#[test]
fn ownership_loss_prevents_submission_and_completion_ack() {
    for ch in [Channel::One, Channel::Two] {
        let mut e = engine(ch);
        e.io.lose_after_configure = true;
        assert_eq!(e.start(attempts(1), &mut Delay::default()), Err(Error::OwnershipLost));
        assert_eq!(e.io.writes("submit"), 0);
        retained(&mut e, Error::OwnershipLost);
        let mut e = engine(ch);
        e.start(attempts(1), &mut Delay::default()).unwrap();
        e.io.complete(ch);
        e.io.owners[ch.index()] = 2;
        assert_eq!(e.poll(), Err(Error::OwnershipLost));
        assert_eq!(e.io.writes("ack-done"), 0);
        assert_eq!(e.io.writes("state"), 0);
        retained(&mut e, Error::OwnershipLost);
    }
}
#[test]
fn pending_ring_or_length_does_not_read_or_ack_a_result() {
    for ch in [Channel::One, Channel::Two] {
        for ring in [(1, 0, 0), (1, 1, 64)] {
            let mut e = engine(ch);
            e.start(attempts(1), &mut Delay::default()).unwrap();
            e.io.complete(ch);
            e.io.ring[ch.index()] = ring;
            let mut delay = Delay::default();
            assert_eq!(e.wait(attempts(3), &mut delay), Err(Error::Timeout));
            assert_eq!(delay.calls, vec![1000; 2]);
            assert_eq!(e.io.writes("state"), 0);
            assert_eq!(e.io.writes("ack-done"), 0);
            retained(&mut e, Error::Timeout);
        }
    }
}
#[test]
fn changed_submission_and_stuck_completion_ack_are_terminal() {
    for ch in [Channel::One, Channel::Two] {
        for stuck in [false, true] {
            let mut e = engine(ch);
            e.start(attempts(1), &mut Delay::default()).unwrap();
            e.io.complete(ch);
            let expected = if stuck {
                e.io.stuck_done_ack = true;
                Error::Acknowledge
            } else {
                e.io.ring[ch.index()].0 = 2;
                Error::Ring
            };
            assert_eq!(e.poll(), Err(expected));
            assert_eq!(e.io.writes("state"), 0);
            retained(&mut e, expected);
        }
    }
}
#[test]
fn not_started_cancel_and_duplicate_start_have_no_hidden_io() {
    for ch in [Channel::One, Channel::Two] {
        for running in [false, true] {
            let mut e = engine(ch);
            let mut delay = Delay::default();
            if running {
                e.start(attempts(1), &mut delay).unwrap();
                let log = e.io.log.clone();
                assert_eq!(e.start(attempts(2), &mut delay), Err(Error::AlreadyStarted));
                assert_eq!(e.io.log, log);
            } else {
                assert_eq!(e.poll(), Err(Error::NotStarted));
                assert_eq!(e.wait(attempts(2), &mut delay), Err(Error::NotStarted));
                assert!(e.io.log.is_empty());
            }
            let log = e.io.log.clone();
            e.cancel();
            assert_eq!(e.io.log, log);
            retained(&mut e, Error::Cancelled);
        }
    }
}

fn hex(s: &str) -> Vec<u8> {
    s.as_bytes()
        .as_chunks::<2>()
        .0
        .iter()
        .map(|p| u8::from_str_radix(core::str::from_utf8(p).unwrap(), 16).unwrap())
        .collect()
}
fn algorithm(name: &str) -> Algorithm {
    match name {
        "Sha1" => Algorithm::Sha1,
        "Sha224" => Algorithm::Sha224,
        "Sha256" => Algorithm::Sha256,
        "Sha384" => Algorithm::Sha384,
        "Sha512" => Algorithm::Sha512,
        "Sm3" => Algorithm::Sm3,
        _ => panic!("unknown source profile"),
    }
}
#[test]
fn original_c_message_vectors_match_all_six_profiles_and_512_tails() {
    let (mut configs, mut tails) = (0, 0);
    for line in include_str!("../../tests/fixtures/ws53-hash-message.txt").lines() {
        let words: Vec<_> = line.split_whitespace().collect();
        let alg = algorithm(words[1]);
        if words[0] == "IV" {
            assert_eq!(alg.block_size(), words[2].parse::<usize>().unwrap());
            assert_eq!(alg.digest_size(), words[3].parse::<usize>().unwrap());
            assert_eq!(alg.suffix_size(), words[4].parse::<usize>().unwrap());
            assert_eq!(alg.hardware(), (words[5].parse().unwrap(), words[6].parse().unwrap()));
            let initial: Vec<u8> =
                alg.initial_words()[..alg.state_words()].iter().flat_map(|w| w.to_le_bytes()).collect();
            assert_eq!(initial, hex(words[7]));
            assert_eq!(Digest::from_state(alg, alg.initial_words()).as_bytes(), &initial[..alg.digest_size()]);
            configs += 1;
        } else {
            assert_eq!(words[0], "PAD");
            let tail = words[2].parse::<usize>().unwrap();
            let total = words[3].parse::<usize>().unwrap();
            let mut message = vec![0x5a; total];
            for i in 0..tail {
                message[total - tail + i] = (i * 37 + 11) as u8;
            }
            let expected = hex(words[4]);
            let mut output = vec![0xa5; alg.padded_len(total).unwrap() + 32];
            let used = alg.encode(&message, &mut output).unwrap();
            assert_eq!(&output[..total], message);
            assert_eq!(&output[2 * alg.block_size()..used], expected);
            assert_eq!(&output[used..], &[0xa5; 32]);
            tails += 1;
        }
    }
    assert_eq!((configs, tails), (6, 512));
}
#[test]
fn padding_boundaries_empty_input_capacity_and_overflow() {
    for alg in
        [Algorithm::Sha1, Algorithm::Sha224, Algorithm::Sha256, Algorithm::Sha384, Algorithm::Sha512, Algorithm::Sm3]
    {
        let block = alg.block_size();
        assert_eq!(alg.prepared_len(alg.max_message_size()), Ok(256));
        assert_eq!(alg.prepared_len(alg.max_message_size() + 1), Err(FormatError::ProfileLength));
        let boundary = block - 1 - alg.suffix_size();
        assert_eq!(alg.padded_len(boundary), Ok(block));
        assert_eq!(alg.padded_len(boundary + 1), Ok(2 * block));
        assert_eq!(alg.padded_len(usize::MAX), Err(FormatError::Length));
        assert_eq!(alg.padded_len(u32::MAX as usize), Err(FormatError::Length));
        let mut output = [0xa5; 256];
        assert_eq!(alg.encode(&[], &mut output), Ok(block));
        assert_eq!(output[0], 0x80);
        assert_eq!(&output[1..block], vec![0; block - 1]);
        let mut short = [0xa5; 32];
        assert_eq!(alg.encode(&[], &mut short), Err(FormatError::Capacity));
        assert_eq!(short, [0xa5; 32]);
        // Representable >512MiB padding sizing is allowed; actual static input
        // remains bounded by the much smaller mode-1 allocation envelope.
        assert!(alg.padded_len(0x20000000).is_ok());
    }
}

#[test]
fn pac_backend_preserves_neighbors_and_uses_selected_command_views() {
    // Local RAM validates emitted field values only. It does not emulate ACK,
    // an indexed state window, DMA, or a silicon reset value.
    for ch in [Channel::One, Channel::Two] {
        let registers: pac::spacc::RegisterBlock = unsafe { core::mem::zeroed() };
        let mut io = PacRegisters { registers: &registers };
        for seed in [0, u32::MAX, 0xa55a5aa5] {
            unsafe {
                registers.hash_lock().write_with_zero(|w| w.bits(seed));
                registers.hash_interrupt_enable().write_with_zero(|w| w.bits(seed));
                registers.interrupt_enable().write_with_zero(|w| w.bits(seed));
            }
            let shift = (ch.index() + 1) * 4;
            io.claim(ch);
            assert_eq!(registers.hash_lock().read().bits(), (seed & !(15 << shift)) | (1 << shift));
            assert_eq!(io.owner(ch), 1);
            io.mask_irq(ch);
            assert_eq!(registers.hash_interrupt_enable().read().bits(), seed & !(1 << (ch.index() + 1)));
            assert_eq!(registers.interrupt_enable().read().bits(), seed);
            io.request_clear(ch);
            io.ack_clear(ch);
            io.ack_done(ch);
            assert_eq!(registers.hash_clear_status().read().bits(), 1 << (ch.index() + 1));
            assert_eq!(registers.hash_interrupt_status().read().bits(), 1 << (ch.index() + 1));
            assert!(io.clear_done(ch));
            assert!(io.done(ch));
            // The WO submission alias seeds the readable ring pointer in RAM.
            unsafe {
                registers.hash_channel(ch.index()).submit().write_with_zero(|w| w.bits(seed));
            }
            io.submit(ch);
            assert_eq!(registers.hash_channel(ch.index()).write_pointer().read().bits(), (seed & !255) | 1);
        }
    }
}
#[test]
fn pac_configures_unkeyed_profiles_without_global_gate_or_other_channel_changes() {
    for ch in [Channel::One, Channel::Two] {
        for alg in [
            Algorithm::Sha1,
            Algorithm::Sha224,
            Algorithm::Sha256,
            Algorithm::Sha384,
            Algorithm::Sha512,
            Algorithm::Sm3,
        ] {
            let registers: pac::spacc::RegisterBlock = unsafe { core::mem::zeroed() };
            let selected = registers.hash_channel(ch.index());
            let other = registers.hash_channel(1 - ch.index());
            unsafe {
                selected.control().write_with_zero(|w| w.bits(u32::MAX));
                selected.key_control().write_with_zero(|w| w.bits(u32::MAX));
                other.control().write_with_zero(|w| w.bits(0xa55a));
                other.key_control().write_with_zero(|w| w.bits(0x1234));
                registers.interrupt_enable().write_with_zero(|w| w.bits(0x111));
            }
            let mut io = PacRegisters { registers: &registers };
            io.configure(ch, alg, 0x20050000);
            assert_eq!(selected.control().read().bits(), !0xf00 | 0xa00);
            let key = selected.key_control().read();
            let (algorithm, mode) = alg.hardware();
            assert_eq!(key.algorithm().bits(), algorithm);
            assert_eq!(key.mode().bits(), mode);
            assert_eq!(key.keyslot().bits(), 0);
            assert!(!key.hmac().bit());
            let field_mask = 0xf | (1 << 9) | (0xff << 16);
            assert_eq!(key.bits() & !field_mask, !field_mask);
            assert_eq!(selected.node_base_low().read().bits(), 0x20050000);
            assert_eq!(selected.node_base_high().read().bits(), 0);
            assert_eq!(selected.node_depth().read().bits(), 2);
            assert_eq!(other.control().read().bits(), 0xa55a);
            assert_eq!(other.key_control().read().bits(), 0x1234);
            assert_eq!(registers.interrupt_enable().read().bits(), 0x111);
            assert_eq!(
                registers.hash_state(ch.index()).data().read().bits(),
                alg.initial_words()[alg.state_words() - 1]
            );
        }
    }
}
