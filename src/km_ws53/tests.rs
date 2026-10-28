//! Protocol fault injection, not an algorithm model or silicon ownership proof.
use super::{engine::Registers, *};
use std::{vec, vec::Vec};

// A matching positive impl makes the inference ambiguous and compilation fail.
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
absent!(KeyManager, Send);
absent!(KeyManager, Sync);
absent!(LoadedKey<'static>, Send);
absent!(LoadedKey<'static>, Sync);
absent!(LoadedKey<'static>, Copy);
absent!(LoadedKey<'static>, Clone);
absent!(CipherKey<'static>, core::fmt::Debug);
absent!(HmacKey<'static>, core::fmt::Debug);
#[cfg(feature = "defmt")]
absent!(CipherKey<'static>, defmt::Format);
#[cfg(feature = "defmt")]
absent!(HmacKey<'static>, defmt::Format);

const SLOT: Slot = Slot::Cipher(CipherSlot::Zero);
const CLEAN: HardwareError = HardwareError { loader: 0, controller: 0 };
const KEY: [u8; 16] = [0x5a; 16]; // Public fixture bytes only.
fn material() -> Material<'static> {
    Material::Cipher(CipherKey::Aes128(&KEY), Usage::Both, Parity::Even)
}

#[test]
fn leases_keep_cipher_permissions_and_distinguish_hmac() {
    for usage in [Usage::Encrypt, Usage::Decrypt, Usage::Both] {
        let core = Core::new(Fake::default());
        let lease = core
            .load(SLOT, Material::Cipher(CipherKey::Aes128(&KEY), usage, Parity::Odd), budget(), &mut Delay::default())
            .unwrap();
        assert_eq!(lease.usage, Some(usage));
        assert_eq!(lease.parity, Parity::Odd);
        assert_eq!(lease.algorithm, Algorithm::Aes128);
    }
    let core = Core::new(Fake::default());
    let lease = core
        .load(
            Slot::Hmac(HmacSlot::One),
            Material::Hmac(HmacKey::new(HmacAlgorithm::Sha256, &KEY).unwrap()),
            budget(),
            &mut Delay::default(),
        )
        .unwrap();
    assert_eq!(lease.usage, None);
}
fn budget() -> NonZeroU32 {
    NonZeroU32::new(3).unwrap()
}

#[derive(Default)]
struct Delay(Vec<u32>);
impl DelayNs for Delay {
    fn delay_ns(&mut self, ns: u32) {
        self.0.push(ns);
    }
}
#[derive(Debug)]
struct Frame {
    slot: Slot,
    half: usize,
    data: [u32; 4],
    command: u32,
}
struct Fake {
    owners: [u8; 10],
    selected: Slot,
    half: usize,
    staging: [u32; 4],
    control: u32,
    irq: u32,
    errors: HardwareError,
    flush: u32,
    frames: Vec<Frame>,
    configs: Vec<(u8, bool, bool)>,
    log: Vec<(&'static str, u32)>,
    polls: usize,
    complete_after: usize,
    timeout_stage: Option<usize>,
    error_stage: Option<usize>,
    lose_stage: Option<usize>,
    reject_claim: bool,
    fail_command: bool,
    ack_count: usize,
    stuck_ack: Option<usize>,
    release_active: bool,
    release_polls: usize,
    release_after: usize,
    release_status: u32,
    release_owner: u8,
}
impl Default for Fake {
    fn default() -> Self {
        Self {
            owners: [0; 10],
            selected: SLOT,
            half: 0,
            staging: [0; 4],
            control: 0,
            irq: 0x140,
            errors: CLEAN,
            flush: 0,
            frames: vec![],
            configs: vec![],
            log: vec![],
            polls: 0,
            complete_after: 2,
            timeout_stage: None,
            error_stage: None,
            lose_stage: None,
            reject_claim: false,
            fail_command: false,
            ack_count: 0,
            stuck_ack: None,
            release_active: false,
            release_polls: 0,
            release_after: 2,
            release_status: 0,
            release_owner: 0,
        }
    }
}
impl Fake {
    fn count(&self, operation: &str) -> usize {
        self.log.iter().filter(|(name, _)| *name == operation).count()
    }
}
impl Registers for Fake {
    fn select(&mut self, slot: Slot) -> Result<(), Error> {
        self.log.push(("select", slot.index() as u32));
        self.selected = slot;
        Ok(())
    }
    fn owner(&mut self) -> u8 {
        self.log.push(("owner", self.selected.index() as u32));
        self.owners[self.selected.index()]
    }
    fn slot_command(&mut self, slot: Slot, lock: bool) -> Result<(), Error> {
        self.log.push((if lock { "claim" } else { "release" }, slot.index() as u32));
        if self.fail_command {
            return Err(Error::UnsupportedSelection);
        }
        if lock {
            if !self.reject_claim {
                self.owners[slot.index()] = 1;
            }
        } else {
            self.release_active = true;
            self.release_polls = 0;
        }
        Ok(())
    }
    fn flush_status(&mut self) -> u32 {
        self.log.push(("flush", 0));
        if self.release_active {
            self.release_polls += 1;
            if self.release_status != 0 {
                return self.release_status;
            }
            if self.release_polls < self.release_after {
                return 1;
            }
            self.owners[self.selected.index()] = self.release_owner;
            self.release_active = false;
        }
        self.flush
    }
    fn errors(&mut self) -> HardwareError {
        self.log.push(("errors", 0));
        self.errors
    }
    fn clear_control(&mut self) -> u32 {
        self.log.push(("control", 0));
        if self.control & 1 != 0 && !self.frames.is_empty() {
            self.polls += 1;
            let stage = self.frames.len() - 1;
            if self.timeout_stage != Some(stage) && self.polls >= self.complete_after {
                self.control &= !1;
                self.irq |= 1 << 14;
                if self.lose_stage == Some(stage) {
                    self.owners[self.selected.index()] = 2;
                }
            }
        }
        self.control
    }
    fn interrupt_status(&mut self) -> u32 {
        self.log.push(("irq", 0));
        self.irq
    }
    fn acknowledge(&mut self, value: u32) {
        self.log.push(("ack", value));
        let index = self.ack_count;
        self.ack_count += 1;
        if self.stuck_ack != Some(index) {
            self.irq &= !(value & (1 << 14));
        }
    }
    fn configure(&mut self, material: &Material<'_>) {
        self.log.push(("configure", 0));
        self.configs.push(material.config());
    }
    fn address(&mut self, slot: Slot, block: usize) {
        assert_eq!(self.selected, slot);
        self.log.push(("address", block as u32));
        self.half = block;
    }
    fn data(&mut self, index: usize, word: u32) {
        self.log.push(("data", word));
        self.staging[index] = word;
    }
    fn submit(&mut self, value: u32) {
        self.log.push(("submit", value));
        self.control = value;
        self.polls = 0;
        self.frames.push(Frame { slot: self.selected, half: self.half, data: self.staging, command: value });
        if self.error_stage == Some(self.frames.len() - 1) {
            self.errors = HardwareError { loader: 0x8000_0000, controller: 0x1234 };
        }
    }
}
fn frozen(core: &Core<Fake>, expected: Error) {
    let count = core.inner.borrow().io.log.len();
    assert_eq!(core.load(SLOT, material(), budget(), &mut Delay::default()).err(), Some(expected));
    assert_eq!(core.inner.borrow_mut().validate(SLOT), Err(expected));
    assert_eq!(core.inner.borrow_mut().release(SLOT, budget(), &mut Delay::default()), Err(expected));
    assert_eq!(core.snapshot().unwrap().fault, Some(expected));
    assert_eq!(core.inner.borrow().io.log.len(), count);
}

#[test]
fn all_slots_coexist_and_drop_or_forget_never_recycles() {
    let core = Core::new(Fake::default());
    let mut leases = vec![];
    for slot in CipherSlot::ALL {
        leases.push(core.load(Slot::Cipher(slot), material(), budget(), &mut Delay::default()).unwrap());
    }
    for slot in HmacSlot::ALL {
        leases.push(
            core.load(
                Slot::Hmac(slot),
                Material::Hmac(HmacKey::new(HmacAlgorithm::Sha256, &KEY).unwrap()),
                budget(),
                &mut Delay::default(),
            )
            .unwrap(),
        );
    }
    assert_eq!(core.snapshot().unwrap(), Snapshot { reserved: 1023, loaded: 1023, fault: None });
    for key in &leases {
        key.validate().unwrap();
    }
    let count = core.inner.borrow().io.log.len();
    // No Drop implementation is intentional: neither operation may do MMIO.
    #[allow(clippy::forget_non_drop)]
    core::mem::forget(leases.pop().unwrap());
    drop(leases);
    for slot in CipherSlot::ALL {
        assert_eq!(
            core.load(Slot::Cipher(slot), material(), budget(), &mut Delay::default()).err(),
            Some(Error::Reserved)
        );
    }
    assert_eq!(core.inner.borrow().io.log.len(), count);
    assert_eq!(core.snapshot().unwrap().reserved, 1023);
}

#[test]
fn explicit_release_reclaims_only_selected_slot_after_owner_reread() {
    let core = Core::new(Fake::default());
    let first = core.load(SLOT, material(), budget(), &mut Delay::default()).unwrap();
    let second = core.load(Slot::Cipher(CipherSlot::One), material(), budget(), &mut Delay::default()).unwrap();
    let mut delay = Delay::default();
    first.release(budget(), &mut delay).unwrap();
    assert_eq!(delay.0, [1000]);
    assert_eq!(core.snapshot().unwrap().reserved, 2);
    second.validate().unwrap();
    core.load(SLOT, material(), budget(), &mut Delay::default()).unwrap();
    assert_eq!(core.inner.borrow().io.count("claim"), 3);
    assert_eq!(core.inner.borrow().io.count("release"), 1);
}

#[test]
fn cipher_bytes_halves_sizes_permissions_and_success_cleanup() {
    let bytes: [u8; 32] = core::array::from_fn(|i| i as u8 + 1);
    for kind in 0..4 {
        for usage in [Usage::Encrypt, Usage::Decrypt, Usage::Both] {
            for parity in [Parity::Even, Parity::Odd] {
                let key = match kind {
                    0 => CipherKey::Aes128(bytes[..16].try_into().unwrap()),
                    1 => CipherKey::Aes192(bytes[..24].try_into().unwrap()),
                    2 => CipherKey::Aes256(&bytes),
                    _ => CipherKey::Sm4(bytes[..16].try_into().unwrap()),
                };
                let length = [16, 24, 32, 16][kind];
                let core = Core::new(Fake { control: 0xa500_6002, ..Fake::default() });
                let result = core.load(SLOT, Material::Cipher(key, usage, parity), budget(), &mut Delay::default());
                if length > 16 && parity == Parity::Odd {
                    assert_eq!(result.err(), Some(Error::InvalidParity));
                    assert!(core.inner.borrow().io.log.is_empty());
                    continue;
                }
                let lease = result.unwrap();
                assert_eq!(lease.parity, parity);
                assert_eq!(
                    lease.algorithm,
                    [Algorithm::Aes128, Algorithm::Aes192, Algorithm::Aes256, Algorithm::Sm4][kind]
                );
                let inner = core.inner.borrow();
                let fake = &inner.io;
                assert_eq!(
                    fake.configs,
                    [(if kind == 3 { 0x50 } else { 0x20 }, usage != Usage::Decrypt, usage != Usage::Encrypt)]
                );
                let actual: Vec<u8> = fake
                    .frames
                    .iter()
                    .flat_map(|frame| frame.data.iter().flat_map(|word| word.to_le_bytes()))
                    .collect();
                assert_eq!(&actual[..length], &bytes[..length]);
                assert!(actual[length..].iter().all(|byte| *byte == 0));
                for (index, frame) in fake.frames.iter().enumerate() {
                    assert_eq!(frame.slot, SLOT);
                    assert_eq!(frame.half, if parity == Parity::Odd { 1 } else { index });
                    assert_eq!(frame.command, 0xa500_6003 | ([1, 2, 3, 1][kind] << 2));
                }
                assert_eq!(fake.staging, [0; 4]);
                assert_eq!(fake.count("data"), fake.frames.len() * 4 + 4);
                assert!(fake.log.iter().filter(|(name, _)| *name == "ack").all(|(_, value)| *value == 0x4140));
            }
        }
    }
}

#[test]
fn hmac_every_length_is_padded_without_alignment_or_silent_truncation() {
    let bytes: [u8; 130] = core::array::from_fn(|i| i as u8);
    for (algorithm, maximum, engine) in [
        (HmacAlgorithm::Sha1, 64, 0xa0),
        (HmacAlgorithm::Sha224, 64, 0xa1),
        (HmacAlgorithm::Sha256, 64, 0xa1),
        (HmacAlgorithm::Sha384, 128, 0xa1),
        (HmacAlgorithm::Sha512, 128, 0xa1),
        (HmacAlgorithm::Sm3, 64, 0xa2),
    ] {
        for length in 0..=maximum + 1 {
            let key = HmacKey::new(algorithm, &bytes[1..1 + length]);
            if length > maximum {
                assert_eq!(key.err(), Some(KeyLengthError { actual: length, maximum }));
                continue;
            }
            let core = Core::new(Fake::default());
            let slot = Slot::Hmac(HmacSlot::One);
            let lease = core.load(slot, Material::Hmac(key.unwrap()), budget(), &mut Delay::default()).unwrap();
            assert_eq!(lease.algorithm, Algorithm::Hmac(algorithm));
            let inner = core.inner.borrow();
            let fake = &inner.io;
            assert_eq!(fake.configs, [(engine, true, false)]);
            let actual: Vec<u8> =
                fake.frames.iter().flat_map(|frame| frame.data.iter().flat_map(|word| word.to_le_bytes())).collect();
            assert_eq!(actual.len(), maximum);
            assert_eq!(&actual[..length], &bytes[1..1 + length]);
            assert!(actual[length..].iter().all(|byte| *byte == 0));
            for (index, frame) in fake.frames.iter().enumerate() {
                assert_eq!(frame.command, 1 | ((index as u32) << 13));
                assert_eq!(frame.slot, slot);
            }
        }
    }
}

#[test]
fn failures_at_each_multistage_submission_retain_staging_and_slot() {
    for hmac in [false, true] {
        for stage in 0..if hmac { 8 } else { 2 } {
            for failure in 0..3 {
                let mut fake = Fake::default();
                let expected = match failure {
                    0 => {
                        fake.timeout_stage = Some(stage);
                        Error::LoadTimeout
                    }
                    1 => {
                        fake.error_stage = Some(stage);
                        Error::Hardware(HardwareError { loader: 0x8000_0000, controller: 0x1234 })
                    }
                    _ => {
                        fake.lose_stage = Some(stage);
                        Error::Ownership(2)
                    }
                };
                let core = Core::new(fake);
                let bytes = [0x9a; 128];
                let (slot, key) = if hmac {
                    (Slot::Hmac(HmacSlot::Zero), Material::Hmac(HmacKey::new(HmacAlgorithm::Sha512, &bytes).unwrap()))
                } else {
                    (
                        SLOT,
                        Material::Cipher(CipherKey::Aes256(bytes[..32].try_into().unwrap()), Usage::Both, Parity::Even),
                    )
                };
                assert_eq!(core.load(slot, key, budget(), &mut Delay::default()).err(), Some(expected));
                assert_eq!(core.snapshot().unwrap().reserved, 1 << slot.index());
                assert_eq!(core.snapshot().unwrap().loaded, 0);
                let inner = core.inner.borrow();
                assert_eq!(inner.io.frames.len(), stage + 1);
                assert_eq!(inner.io.staging, [0x9a9a_9a9a; 4]);
                assert_eq!(inner.io.count("data"), (stage + 1) * 4);
                assert_eq!(inner.io.count("release"), 0);
                drop(inner);
                frozen(&core, expected);
            }
        }
    }
}

#[test]
fn poll_budgets_are_finite_and_do_not_delay_after_final_attempt() {
    for attempts in 1..=5 {
        let core = Core::new(Fake { timeout_stage: Some(0), ..Fake::default() });
        let mut delay = Delay::default();
        assert_eq!(
            core.load(SLOT, material(), NonZeroU32::new(attempts).unwrap(), &mut delay).err(),
            Some(Error::LoadTimeout)
        );
        assert_eq!(core.inner.borrow().io.polls, attempts as usize);
        assert_eq!(delay.0, vec![1000; attempts as usize - 1]);
    }
}

#[test]
fn foreign_slot_is_nonterminal_and_never_claimed() {
    for owner in 1..8 {
        let mut fake = Fake::default();
        fake.owners[0] = owner;
        let core = Core::new(fake);
        assert_eq!(core.load(SLOT, material(), budget(), &mut Delay::default()).err(), Some(Error::SlotBusy(owner)));
        assert_eq!(core.inner.borrow().io.count("claim"), 0);
        assert_eq!(core.snapshot().unwrap(), Snapshot { reserved: 0, loaded: 0, fault: None });
        core.load(Slot::Cipher(CipherSlot::One), material(), budget(), &mut Delay::default()).unwrap();
    }
}

#[test]
fn claim_rejection_or_unrepresentable_command_keeps_reservation() {
    for reject_claim in [false, true] {
        let core = Core::new(Fake { reject_claim, fail_command: !reject_claim, ..Fake::default() });
        let error = if reject_claim { Error::Ownership(0) } else { Error::UnsupportedSelection };
        assert_eq!(core.load(SLOT, material(), budget(), &mut Delay::default()).err(), Some(error));
        assert_eq!(core.snapshot().unwrap().reserved, 1);
        assert_eq!(core.inner.borrow().io.count("data"), 0);
        frozen(&core, error);
    }
}

#[test]
fn preexisting_busy_and_full_error_words_stop_before_claim() {
    for (fake, error) in [
        (Fake { control: 1, ..Fake::default() }, Error::LoaderBusy),
        (Fake { flush: 1, ..Fake::default() }, Error::FlushStatus(1)),
        (Fake { flush: 0x8000_0004, ..Fake::default() }, Error::FlushStatus(0x8000_0004)),
        (
            Fake { errors: HardwareError { loader: 0, controller: 0x8000_0000 }, ..Fake::default() },
            Error::Hardware(HardwareError { loader: 0, controller: 0x8000_0000 }),
        ),
    ] {
        let core = Core::new(fake);
        assert_eq!(core.load(SLOT, material(), budget(), &mut Delay::default()).err(), Some(error));
        assert_eq!(core.inner.borrow().io.count("claim"), 0);
        assert_eq!(core.snapshot().unwrap().reserved, 0);
        frozen(&core, error);
    }
}

#[test]
fn foreign_interrupts_and_stuck_ack_never_replay_or_clear_staging() {
    for bit in 0..32 {
        if 0x41f0 & (1 << bit) != 0 {
            continue;
        }
        let core = Core::new(Fake { irq: 1 << bit, ..Fake::default() });
        let error = Error::UnexpectedInterrupt(1 << bit);
        assert_eq!(core.load(SLOT, material(), budget(), &mut Delay::default()).err(), Some(error));
        assert_eq!(core.inner.borrow().io.count("ack"), 0);
        assert_eq!(core.inner.borrow().io.count("data"), 0);
        frozen(&core, error);
    }
    for stuck in 0..=1 {
        let core = Core::new(Fake { irq: 1 << 14, stuck_ack: Some(stuck), ..Fake::default() });
        assert_eq!(core.load(SLOT, material(), budget(), &mut Delay::default()).err(), Some(Error::Acknowledge));
        assert_eq!(core.inner.borrow().io.count("data"), stuck * 4);
        frozen(&core, Error::Acknowledge);
    }
}

#[test]
fn release_timeout_error_bit_and_nonzero_owner_never_free_slot() {
    for (status, owner, after, error) in [
        (1, 0, 2, Error::ReleaseTimeout),
        (2, 0, 2, Error::FlushStatus(2)),
        (4, 0, 2, Error::FlushStatus(4)),
        (0, 1, 1, Error::ReleaseUnconfirmed(1)),
        (0, 2, 1, Error::ReleaseUnconfirmed(2)),
    ] {
        let core =
            Core::new(Fake { release_status: status, release_owner: owner, release_after: after, ..Fake::default() });
        let lease = core.load(SLOT, material(), budget(), &mut Delay::default()).unwrap();
        let mut delay = Delay::default();
        assert_eq!(lease.release(budget(), &mut delay), Err(error));
        assert_eq!(delay.0.len(), if status == 1 { 2 } else { 0 });
        assert_eq!(core.snapshot().unwrap().reserved, 1);
        assert_eq!(core.snapshot().unwrap().loaded, 0);
        frozen(&core, error);
    }
}

#[test]
fn lost_loaded_owner_never_issues_another_users_unlock() {
    let core = Core::new(Fake::default());
    let lease = core.load(SLOT, material(), budget(), &mut Delay::default()).unwrap();
    core.inner.borrow_mut().io.owners[0] = 2;
    assert_eq!(lease.release(budget(), &mut Delay::default()), Err(Error::Ownership(2)));
    assert_eq!(core.inner.borrow().io.count("release"), 0);
    frozen(&core, Error::Ownership(2));
}

#[test]
fn delay_reentry_is_rejected_without_panicking_or_nested_mmio() {
    struct Reenter<'a>(&'a Core<Fake>, usize);
    impl DelayNs for Reenter<'_> {
        fn delay_ns(&mut self, _: u32) {
            self.1 += 1;
            assert_eq!(self.0.snapshot(), Err(Error::Borrowed));
            assert_eq!(self.0.load(SLOT, material(), budget(), &mut Delay::default()).err(), Some(Error::Borrowed));
        }
    }
    let core = Core::new(Fake::default());
    let mut delay = Reenter(&core, 0);
    let lease = core.load(SLOT, material(), budget(), &mut delay).unwrap();
    assert_eq!(delay.1, 1);
    assert_eq!(core.inner.borrow().io.count("claim"), 1);
    lease.validate().unwrap();
}

#[test]
fn pac_transport_preserves_reserved_fields_and_rejects_ts_namespace() {
    // Real generated PAC over ordinary host storage; no silicon emulation.
    let registers: pac::km::RegisterBlock = unsafe { core::mem::zeroed() };
    let mut io = PacRegisters { registers: &registers };
    for seed in [0, 0xfffe_ffff, 0xa5a4_5a5a] {
        for slot in CipherSlot::ALL.into_iter().map(Slot::Cipher).chain(HmacSlot::ALL.into_iter().map(Slot::Hmac)) {
            unsafe {
                registers.slot_select().write_with_zero(|w| w.bits(seed));
                registers.ree_slot_command().write_with_zero(|w| w.bits(seed));
                registers.key_address().write_with_zero(|w| w.bits(seed));
                registers.key_config().write_with_zero(|w| w.bits(seed));
                registers.key_security().write_with_zero(|w| w.bits(seed));
            }
            io.select(slot).unwrap();
            assert_eq!(
                registers.slot_select().read().bits(),
                (seed & !0x83ff) | slot.number() | (u32::from(slot.hmac()) << 15)
            );
            io.slot_command(slot, true).unwrap();
            assert_eq!(
                registers.ree_slot_read().read().bits(),
                (seed & !0x10_83ff) | slot.number() | (u32::from(slot.hmac()) << 15) | (1 << 20)
            );
            io.slot_command(slot, false).unwrap();
            assert_eq!(registers.ree_slot_read().read().bits() & (1 << 20), 0);
            io.address(slot, 1);
            assert_eq!(
                registers.key_address().read().bits(),
                (seed & !1023) | if slot.hmac() { slot.number() } else { slot.number() * 2 + 1 }
            );
            io.configure(&material());
            assert_eq!(registers.key_config().read().bits(), (seed & !0x3_0ff7) | 0x3_0201);
            assert_eq!(registers.key_security().read().bits(), (seed & !63) | 10);
        }
    }
    unsafe {
        registers.slot_select().write_with_zero(|w| w.bits(0x10001));
        registers.ree_slot_command().write_with_zero(|w| w.bits(0x10001));
    }
    assert_eq!(io.select(SLOT), Err(Error::UnsupportedSelection));
    assert_eq!(io.slot_command(SLOT, true), Err(Error::UnsupportedSelection));
    assert_eq!(registers.slot_select().read().bits(), 0x10001);
    assert_eq!(registers.ree_slot_read().read().bits(), 0x10001);
    for (index, word) in [0x1122_3344, 0x5566_7788, 0x99aa_bbcc, 0xddee_ff00].into_iter().enumerate() {
        io.data(index, word);
    }
    // These pointers address the local RAM fixture only, never real WO MMIO.
    assert_eq!(unsafe { registers.data_in(2).as_ptr().read() }, 0x99aa_bbcc);
    assert_eq!(registers.tee_slot_read().read().bits(), 0);
    assert_eq!(unsafe { registers.common_lock().as_ptr().read() }, 0);
    assert_eq!(unsafe { registers.common_unlock().as_ptr().read() }, 0);
}
