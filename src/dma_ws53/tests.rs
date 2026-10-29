use super::*;
use std::{cell::RefCell, rc::Rc, vec::Vec};
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) enum Event {
    Configure,
    Start,
    Status,
    Stop,
    Delay,
    Listen(InterruptEvents, bool),
    Acknowledge(InterruptEvents),
}
pub(super) struct Mock {
    events: Rc<RefCell<Vec<Event>>>,
    running: bool,
    valid: bool,
    stale: bool,
    busy: bool,
    pub(super) ready: usize,
    polls: usize,
    pub(super) error: bool,
    pub(super) bad_done: bool,
    observed: bool,
    wrong_address: bool,
    source: Range,
    destination: Range,
}
impl Registers for Mock {
    fn memory_readback(&self) -> MemoryReadback {
        if self.observed { MemoryReadback::ObservedIdleZeroLastAddress } else { MemoryReadback::SdkModel }
    }
    fn status(&mut self) -> Status {
        self.events.borrow_mut().push(Event::Status);
        if !self.running {
            return Status {
                complete: self.stale,
                enabled: self.busy,
                remaining: if self.observed { 0 } else { self.source.count },
                source: self.source.start,
                destination: self.destination.start,
                ..Status::default()
            };
        }
        self.polls += 1;
        let complete = self.polls >= self.ready;
        Status {
            complete,
            error: self.error,
            remaining: if complete && !self.bad_done { 0 } else { self.source.count },
            enabled: !complete,
            active: !complete,
            source: if complete {
                self.source.end - if self.observed && !self.wrong_address { u32::from(self.source.width) } else { 0 }
            } else {
                self.source.start
            },
            destination: if complete {
                self.destination.end
                    - if self.observed && !self.wrong_address { u32::from(self.destination.width) } else { 0 }
            } else {
                self.destination.start
            },
        }
    }
    fn configure(&mut self, s: Range, d: Range) {
        self.source = s;
        self.destination = d;
        self.events.borrow_mut().push(Event::Configure);
    }
    fn valid(&mut self, _: Range) -> bool {
        self.valid
    }
    fn start(&mut self) {
        self.running = true;
        self.events.borrow_mut().push(Event::Start);
    }
    fn stop(&mut self) {
        self.events.borrow_mut().push(Event::Stop);
    }
}
pub(super) struct Delay(pub(super) Rc<RefCell<Vec<Event>>>);
impl DelayNs for Delay {
    fn delay_ns(&mut self, _: u32) {
        self.0.borrow_mut().push(Event::Delay);
    }
}
pub(super) fn engine() -> (Engine<Mock>, Delay) {
    let events = Rc::new(RefCell::new(Vec::new()));
    let source = Range::new(0x20020000, 3, 4).unwrap();
    let destination = Range::new(0x20040000, 3, 4).unwrap();
    (
        Engine {
            io: Mock {
                events: events.clone(),
                running: false,
                valid: true,
                stale: false,
                busy: false,
                ready: 2,
                polls: 0,
                error: false,
                bad_done: false,
                observed: false,
                wrong_address: false,
                source,
                destination,
            },
            source,
            destination,
            state: State::Ready,
            stop_on_drop: false,
        },
        Delay(events),
    )
}
fn budget(n: u16) -> NonZeroU16 {
    NonZeroU16::new(n).unwrap()
}
#[test]
fn observed_memory_accepts_idle_zero_and_requires_last_element_address() {
    for width in [1, 2, 4] {
        for count in [1, 17, 4095] {
            let (mut e, mut delay) = engine();
            e.io.observed = true;
            e.source = Range::new(0x20020000, count, width).unwrap();
            e.destination = Range::new(0x20040000, count, width).unwrap();
            e.start().unwrap();
            assert_eq!(e.wait(budget(3), &mut delay).unwrap().elements(), count as u16);
            assert!(e.state == State::Retained);
            let (mut e, mut delay) = engine();
            e.io.observed = true;
            e.io.wrong_address = true;
            e.source = Range::new(0x20020000, count, width).unwrap();
            e.destination = Range::new(0x20040000, count, width).unwrap();
            e.start().unwrap();
            assert_eq!(e.wait(budget(3), &mut delay), Err(Error::CompletionState));
        }
    }
}
#[test]
fn all_width_count_endpoints_and_invalid_ranges() {
    for width in [1, 2, 4] {
        for count in 1..=4095 {
            let end = 0x20060000;
            let start = end - count * width as usize;
            let r = Range::new(start, count, width).unwrap();
            assert_eq!(r.end, end as u32);
            assert_eq!(r.count as usize, count);
            assert_eq!(Range::new(start + width as usize, count, width), Err(BufferError::Range));
        }
        for count in [0, 4096, usize::MAX] {
            assert_eq!(Range::new(0x20010000, count, width), Err(BufferError::Length));
        }
        assert_eq!(Range::new(0x20010000 - width as usize, 1, width), Err(BufferError::Range));
        assert_eq!(Range::new(usize::MAX - 3, 2, width), Err(BufferError::Range));
    }
    assert_eq!(Range::new(0x20010001, 1, 2), Err(BufferError::Range));
    assert_eq!(Range::new(0x20010002, 1, 4), Err(BufferError::Range));
    assert_eq!(Range::new(0x20010000, 1, 8), Err(BufferError::Range));
}
#[test]
fn touching_ranges_are_disjoint_but_partial_or_contained_aliases_rejected() {
    let r = Range::new(0x20020000, 4, 4).unwrap();
    for (offset, overlap) in [(0, true), (4, true), (12, true), (16, false), (20, false)] {
        let other = Range::new(r.start as usize + offset, 1, 4).unwrap();
        assert_eq!(r.overlaps(other), overlap);
        assert_eq!(other.overlaps(r), overlap);
    }
}
#[test]
fn tokens_cover_all_eight_distinct_bits() {
    let ids = [
        ChannelId::Channel0,
        ChannelId::Channel1,
        ChannelId::Channel2,
        ChannelId::Channel3,
        ChannelId::Channel4,
        ChannelId::Channel5,
        ChannelId::Channel6,
        ChannelId::Channel7,
    ];
    let mut mask = 0;
    for (n, id) in ids.into_iter().enumerate() {
        assert_eq!(id.mask(), 1 << n);
        mask |= id.mask();
    }
    assert_eq!(mask, 255);
}
#[test]
fn controller_checks_idle_before_mutation_and_verifies_enable() {
    struct C {
        idle: bool,
        valid: bool,
        calls: usize,
    }
    impl ControllerRegisters for C {
        fn idle(&mut self) -> bool {
            self.idle
        }
        fn enable(&mut self) {
            self.calls += 1;
        }
        fn valid(&mut self) -> bool {
            self.valid
        }
    }
    let mut c = C { idle: false, valid: true, calls: 0 };
    assert_eq!(initialize(&mut c), Err(Error::Occupied));
    assert_eq!(c.calls, 0);
    c.idle = true;
    c.valid = false;
    assert_eq!(initialize(&mut c), Err(Error::Configuration));
    assert_eq!(c.calls, 1);
    c.valid = true;
    assert_eq!(initialize(&mut c), Ok(()));
    assert_eq!(c.calls, 2);
}
#[test]
fn busy_or_stale_never_arms_and_cannot_be_silently_retried() {
    for busy in [false, true] {
        let (mut e, d) = engine();
        e.io.busy = busy;
        e.io.stale = !busy;
        assert_eq!(e.start(), Err(if busy { Error::Occupied } else { Error::StaleEvent }));
        assert_eq!(e.start(), Err(Error::NotReady));
        drop(e);
        assert!(!d.0.borrow().contains(&Event::Start));
        assert_eq!(d.0.borrow().contains(&Event::Stop), !busy);
    }
}
#[test]
fn observed_completion_is_not_automatic_reuse_or_buffer_release() {
    let (mut e, mut d) = engine();
    e.start().unwrap();
    assert_eq!(e.wait(budget(2), &mut d), Ok(Completion { elements: 3 }));
    assert_eq!(e.start(), Err(Error::NotReady));
    assert_eq!(e.wait(budget(1), &mut d), Err(Error::NotReady));
    e.stop();
    drop(e);
    assert_eq!(d.0.borrow().iter().filter(|v| **v == Event::Stop).count(), 1);
    assert_eq!(d.0.borrow().iter().filter(|v| **v == Event::Delay).count(), 1);
}
#[test]
fn timeout_is_bounded_and_preserves_retained_state() {
    let (mut e, mut d) = engine();
    e.io.ready = usize::MAX;
    e.start().unwrap();
    assert_eq!(e.wait(budget(3), &mut d), Err(Error::Timeout));
    assert_eq!(e.io.polls, 3);
    assert_eq!(d.0.borrow().iter().filter(|v| **v == Event::Delay).count(), 2);
    assert_eq!(e.start(), Err(Error::NotReady));
}
#[test]
fn raw_error_wins_over_simultaneous_completion_and_configuration_failure() {
    let (mut e, mut d) = engine();
    e.io.ready = 1;
    e.start().unwrap();
    e.io.error = true;
    e.io.valid = false;
    assert_eq!(e.wait(budget(1), &mut d), Err(Error::Bus));
    assert_eq!(e.start(), Err(Error::NotReady));
}
#[test]
fn wrong_completion_state_is_not_accepted() {
    let (mut e, mut d) = engine();
    e.io.ready = 1;
    e.io.bad_done = true;
    e.start().unwrap();
    assert_eq!(e.wait(budget(1), &mut d), Err(Error::CompletionState));
}
#[test]
fn configuration_failure_before_or_after_start_requests_stop_once() {
    for before in [false, true] {
        let (mut e, mut d) = engine();
        if before {
            e.io.valid = false;
            assert_eq!(e.start(), Err(Error::Configuration));
        } else {
            e.start().unwrap();
            e.io.valid = false;
            assert_eq!(e.wait(budget(1), &mut d), Err(Error::Configuration));
        }
        drop(e);
        assert_eq!(d.0.borrow().iter().filter(|v| **v == Event::Stop).count(), 1);
    }
}
#[test]
fn drop_cancels_only_running_owner_and_forget_cannot_be_a_drain_mechanism() {
    let (e, d) = engine();
    drop(e);
    assert!(d.0.borrow().is_empty());
    let (mut e, d) = engine();
    e.start().unwrap();
    drop(e);
    assert_eq!(d.0.borrow().last(), Some(&Event::Stop));
    let (mut e, d) = engine();
    e.start().unwrap();
    core::mem::forget(e);
    assert!(!d.0.borrow().contains(&Event::Stop));
}
#[test]
fn delay_panic_keeps_buffers_reserved_until_owner_drops() {
    struct PanicDelay;
    impl DelayNs for PanicDelay {
        fn delay_ns(&mut self, _: u32) {
            panic!("clock lost");
        }
    }
    let (mut e, d) = engine();
    e.start().unwrap();
    assert!(std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| e.wait(budget(3), &mut PanicDelay))).is_err());
    assert_eq!(e.start(), Err(Error::NotReady));
    drop(e);
    assert_eq!(d.0.borrow().last(), Some(&Event::Stop));
}

impl InterruptRegisters for Mock {
    fn listen(&mut self, events: InterruptEvents, running: bool) {
        self.events.borrow_mut().push(Event::Listen(events, running));
    }
    fn acknowledge(&mut self, events: InterruptEvents) {
        self.events.borrow_mut().push(Event::Acknowledge(events));
    }
}
#[test]
fn interrupts_preserve_running_evidence_and_retained_ownership() {
    for event in [InterruptEvents::None, InterruptEvents::Completion, InterruptEvents::Error, InterruptEvents::All] {
        let (mut e, mut delay) = engine();
        assert_eq!(e.acknowledge(event), Err(Error::NotReady));
        e.listen(event).unwrap();
        assert_eq!(e.io.events.borrow().last(), Some(&Event::Listen(event, false)));
        e.start().unwrap();
        e.listen(event).unwrap();
        assert_eq!(e.io.events.borrow().last(), Some(&Event::Listen(event, true)));
        let before = e.io.events.borrow().len();
        assert_eq!(e.acknowledge(event), Err(Error::NotReady));
        assert_eq!(e.io.events.borrow().len(), before);
        e.wait(budget(3), &mut delay).unwrap();
        e.acknowledge(event).unwrap();
        assert_eq!(e.io.events.borrow().last(), Some(&Event::Acknowledge(event)));
        assert_eq!(e.listen(event), Err(Error::NotReady));
        assert!(e.state == State::Retained);
    }
}
