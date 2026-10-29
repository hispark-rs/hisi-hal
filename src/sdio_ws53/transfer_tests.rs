use super::*;
use std::{collections::VecDeque, vec::Vec};

pub(super) struct Model {
    pub(super) clock: bool,
    pub(super) ready: bool,
    pub(super) events: VecDeque<EventSnapshot>,
    argument: Cmd53Argument,
    block: Option<BlockSize>,
    pub(super) trace: Vec<&'static str>,
    pub(super) published: Vec<u32>,
    pub(super) acked: Vec<EventSnapshot>,
    send_lengths: Vec<u32>,
}
impl Registers for Model {
    fn clocks(&mut self) -> bool {
        self.trace.push("clock");
        self.clock
    }
    fn configured(&mut self) -> bool {
        self.trace.push("ready");
        self.ready
    }
    fn mask(&mut self) {
        self.trace.push("mask");
    }
    fn snapshot(&mut self) -> EventSnapshot {
        self.trace.push("snapshot");
        self.events.pop_front().unwrap_or_default()
    }
    fn request(&mut self) -> (Cmd53Argument, Option<BlockSize>) {
        self.trace.push("request");
        (self.argument, self.block)
    }
    fn prepare(&mut self, _: Range, _: Range) {
        self.trace.push("prepare");
    }
    fn acknowledge(&mut self, e: EventSnapshot) {
        self.trace.push("ack");
        if pending(e) {
            self.acked.push(e);
        }
    }
    fn send_length(&mut self, bytes: u32) {
        self.trace.push("length");
        self.send_lengths.push(bytes);
    }
    fn publish(&mut self, addr: u32) {
        self.trace.push("publish");
        self.published.push(addr);
    }
    fn announce_read(&mut self) {
        self.trace.push("announce");
    }
    #[cfg(feature = "async")]
    fn listen(&mut self) {
        self.trace.push("listen");
    }
}
pub(super) fn new(direction: Direction) -> Engine<Model> {
    Engine {
        io: Model {
            clock: true,
            ready: true,
            events: VecDeque::new(),
            argument: Cmd53Argument::from_bits(0x10000008 | (u32::from(direction == Direction::HostWrite) << 31)),
            block: None,
            trace: Vec::new(),
            published: Vec::new(),
            acked: Vec::new(),
            send_lengths: Vec::new(),
        },
        state: State::New,
        direction,
        data: Range::new(0x20020000, 8).unwrap(),
        table: Range::new(0x20010000, core::mem::size_of::<AdmaStorage>()).unwrap(),
    }
}
pub(super) fn event(bits: u32) -> EventSnapshot {
    EventSnapshot { global: 4, function1: Function1Events::from_bits(bits), ..Default::default() }
}
pub(super) fn running(direction: Direction) -> Engine<Model> {
    let mut e = new(direction);
    e.start().unwrap();
    e.io.events.push_back(event(if direction == Direction::HostWrite { 0x10 } else { 8 }));
    assert_eq!(e.poll().unwrap().progress, TransferProgress::Running);
    e
}
fn inert(e: &mut Engine<Model>) {
    let trace = e.io.trace.clone();
    assert_eq!(e.poll(), Err(TransferError::Retained));
    e.cancel();
    assert_eq!(e.start(), Err(TransferError::AlreadyStarted));
    assert_eq!(e.io.trace, trace);
}
#[derive(Default)]
struct Delay(Vec<u32>);
impl DelayNs for Delay {
    fn delay_ns(&mut self, n: u32) {
        self.0.push(n);
    }
}

#[test]
fn allocation_envelope_rejects_overflow_alignment_and_overlap() {
    for (start, length) in [(0, 4), (0x20010001, 4), (0x2000fffc, 8), (0x2005fffc, 8), (usize::MAX - 3, 8)] {
        assert_eq!(Range::new(start, length), Err(MemoryError::Range));
    }
    assert_eq!(Range::new(0x20010000, 0), Err(MemoryError::Length));
    assert_eq!(Range::new(0x20010000, 0x200000), Err(MemoryError::Length));
    let first = Range::new(0x20010000, 8).unwrap();
    assert!(first.overlaps(Range::new(0x20010004, 8).unwrap()));
    assert!(!first.overlaps(Range::new(0x20010008, 4).unwrap()));
    assert!(Range::new(0x2005fffc, 4).is_ok());
}

#[test]
fn ordinary_descriptors_preserve_lengths_alignment_and_retained_tail() {
    assert_eq!(core::mem::size_of::<AdmaStorage>(), 1036);
    assert_eq!(core::mem::offset_of!(AdmaStorage, trailer), 1032);
    for length in [1, 3, 4, 65531, 65532, 65533, 65535, 65536, 131064, 327680] {
        let mut storage = AdmaStorage::new();
        fill_storage(&mut storage, Range::new(0x20010000, length).unwrap(), 0x20050000);
        let mut total = 0;
        let mut records = 0;
        while total < length {
            let raw = storage.entries[records];
            assert_eq!(&raw[..2], &[0x21, 0]);
            let bytes = u16::from_le_bytes(raw[2..4].try_into().unwrap()) as usize;
            let address = u32::from_le_bytes(raw[4..8].try_into().unwrap());
            assert!(bytes > 0 && bytes <= 65532 && address & 3 == 0);
            assert_eq!(address as usize, 0x20010000 + total);
            total += bytes;
            records += 1;
        }
        assert_eq!(total, length);
        assert_eq!(storage.entries[records], [0x21, 0, 4, 0, 8, 4, 5, 0x20]);
        assert_eq!(storage.trailer, [0; 4]);
        assert!(storage.entries[records + 1..].iter().all(|r| *r == [0; 8]));
    }
}

#[test]
fn publication_follows_fresh_request_ack_and_exact_send_length() {
    for direction in [Direction::HostRead, Direction::HostWrite] {
        let mut e = new(direction);
        assert_eq!(e.poll(), Err(TransferError::NotStarted));
        assert!(e.io.trace.is_empty());
        e.start().unwrap();
        assert_eq!(
            e.io.trace,
            if direction == Direction::HostRead {
                std::vec!["clock", "ready", "mask", "snapshot", "prepare", "length", "announce"]
            } else {
                std::vec!["clock", "ready", "mask", "snapshot", "prepare"]
            }
        );
        assert!(e.io.published.is_empty());
        assert_eq!(e.poll().unwrap().progress, TransferProgress::WaitingForRequest);
        e.io.events.push_back(event(if direction == Direction::HostWrite { 0x10 } else { 8 }));
        assert_eq!(e.poll().unwrap().progress, TransferProgress::Running);
        assert_eq!(e.io.published, [0x20010000]);
        assert_eq!(e.io.send_lengths, if direction == Direction::HostRead { std::vec![8, 8] } else { Vec::new() });
        assert!(e.io.trace.ends_with(if direction == Direction::HostRead {
            &["request", "ack", "length", "publish"]
        } else {
            &["request", "ack", "publish"]
        }));
    }
}

#[test]
fn invalid_requests_never_publish_or_ack() {
    for (argument, direction) in [
        (0x00000008, Direction::HostRead),
        (0x90000008, Direction::HostRead),
        (0x14000008, Direction::HostRead),
        (0x10000208, Direction::HostRead),
        (0x18000000, Direction::HostRead),
        (0x10000008, Direction::HostWrite),
    ] {
        let mut e = new(direction);
        e.start().unwrap();
        e.io.argument = Cmd53Argument::from_bits(argument);
        e.io.events.push_back(event(if direction == Direction::HostWrite { 0x10 } else { 8 }));
        assert!(matches!(e.poll(), Err(TransferError::Request { .. })));
        assert!(e.io.published.is_empty() && e.io.acked.is_empty());
        inert(&mut e);
    }
    let mut e = new(Direction::HostRead);
    e.start().unwrap();
    e.io.argument = Cmd53Argument::from_bits(0x10000007);
    e.io.events.push_back(event(8));
    assert_eq!(e.poll(), Err(TransferError::LengthMismatch { requested: 7, reserved: 8, events: event(8) }));
    assert!(e.io.published.is_empty() && e.io.acked.is_empty());
    inert(&mut e);
}

#[test]
fn dirty_clock_and_config_failures_stop_without_unsafe_bus_access() {
    let mut e = new(Direction::HostRead);
    e.io.clock = false;
    assert_eq!(e.start(), Err(TransferError::ClockGate));
    assert_eq!(e.io.trace, ["clock"]);
    inert(&mut e);
    let mut e = new(Direction::HostRead);
    e.io.events.push_back(event(8));
    assert_eq!(e.start(), Err(TransferError::DirtyStart(event(8))));
    assert!(!e.io.trace.contains(&"prepare"));
    inert(&mut e);
    let mut e = running(Direction::HostWrite);
    e.io.clock = false;
    let before = e.io.trace.len();
    assert_eq!(e.poll(), Err(TransferError::ClockGate));
    assert_eq!(&e.io.trace[before..], ["clock"]);
    inert(&mut e);
    let mut e = running(Direction::HostWrite);
    e.io.ready = false;
    assert_eq!(e.poll(), Err(TransferError::Configuration));
    inert(&mut e);
}

#[test]
fn completion_and_errors_keep_status_and_disallow_rearming() {
    for direction in [Direction::HostRead, Direction::HostWrite] {
        let mut e = running(direction);
        let before = e.io.acked.len();
        let done = event(0x20 | if direction == Direction::HostWrite { 1 } else { 2 });
        e.io.events.push_back(done);
        assert_eq!(e.poll(), Ok(TransferPoll { progress: TransferProgress::CompletionObserved, events: done }));
        assert_eq!(e.io.acked.len(), before);
        inert(&mut e);
        for bits in [0x80, 4, 0x81, 0x82] {
            let mut e = running(direction);
            let before = e.io.acked.len();
            e.io.events.push_back(event(bits));
            assert_eq!(e.poll(), Err(TransferError::DeviceError(event(bits))));
            assert_eq!(e.io.acked.len(), before);
            inert(&mut e);
        }
    }
}

#[test]
fn unknown_reset_suspend_and_opposite_start_are_retained() {
    for bit in [4, 8, 10, 11, 13, 14, 31] {
        let mut e = new(Direction::HostRead);
        e.start().unwrap();
        let event = event((1 << bit) | 8);
        e.io.events.push_back(event);
        assert_eq!(e.poll(), Err(TransferError::Unsupported(event)));
        assert!(e.io.published.is_empty() && e.io.acked.is_empty());
        inert(&mut e);
    }
}

#[test]
fn a_single_budget_covers_request_and_completion_and_cancellation_retains() {
    let mut e = new(Direction::HostRead);
    e.start().unwrap();
    e.io.events = [EventSnapshot::default(), event(8), event(0x40)].into();
    let mut delay = Delay::default();
    let mut events = Vec::new();
    assert_eq!(e.wait(NonZeroU16::new(3).unwrap(), &mut delay, |v| events.push(v)), Err(TransferError::Timeout));
    assert_eq!(delay.0, [1000, 1000]);
    assert_eq!(events, [event(8), event(0x40)]);
    inert(&mut e);
    for started in [false, true] {
        let mut e = if started { running(Direction::HostWrite) } else { new(Direction::HostWrite) };
        e.cancel();
        inert(&mut e);
        assert!(!e.io.trace.contains(&"reset"));
    }
}
