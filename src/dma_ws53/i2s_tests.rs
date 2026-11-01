use super::super::Status;
use super::*;
use std::{cell::RefCell, rc::Rc, vec::Vec};
type Events = Rc<RefCell<Vec<&'static str>>>;
struct Dma {
    events: Events,
    rx: bool,
    running: bool,
    busy: bool,
    valid: bool,
    error: bool,
    late_error: bool,
    ready: usize,
    polls: usize,
    remaining: Option<u16>,
    source: Range,
    destination: Range,
}
impl Registers for Dma {
    fn status(&mut self) -> Status {
        if !self.running {
            return Status {
                enabled: self.busy,
                remaining: self.source.count,
                source: self.source.start,
                destination: self.destination.start,
                ..Status::default()
            };
        }
        self.polls += 1;
        let done = self.polls >= self.ready;
        Status {
            complete: done,
            error: self.error || self.late_error && self.polls >= 2,
            enabled: !done,
            active: !done,
            remaining: self.remaining.unwrap_or(if done { 0 } else { self.source.count }),
            source: if done { self.source.end } else { self.source.start },
            destination: if done { self.destination.end } else { self.destination.start },
        }
    }
    fn configure(&mut self, s: Range, d: Range) {
        self.source = s;
        self.destination = d;
        self.events.borrow_mut().push(if self.rx { "rx-config" } else { "tx-config" });
    }
    fn valid(&mut self, _: Range) -> bool {
        self.valid
    }
    fn start(&mut self) {
        self.running = true;
        self.events.borrow_mut().push(if self.rx { "rx-start" } else { "tx-start" });
    }
    fn stop(&mut self) {
        self.events.borrow_mut().push(if self.rx { "rx-stop" } else { "tx-stop" });
    }
}
struct Peripheral {
    events: Events,
    snapshot: Snapshot,
    valid: bool,
    tail_valid: bool,
    prefill: bool,
    level: u8,
}
impl I2sRegisters for Peripheral {
    fn snapshot(&mut self) -> Snapshot {
        self.snapshot
    }
    fn prepare(&mut self, level: u8) {
        self.events.borrow_mut().push("prepare");
        self.level = level;
        self.snapshot.clock = true;
        self.snapshot.merge = true;
        if self.prefill {
            self.snapshot.tx = [1, 1];
        }
    }
    fn valid(&mut self, level: u8) -> bool {
        self.valid && self.level == level && (level != 0 || self.tail_valid)
    }
    fn start(&mut self, tx: bool, rx: bool) {
        self.events.borrow_mut().push(match (tx, rx) {
            (true, true) => "duplex",
            (true, false) => "transmit",
            (false, true) => "receive",
            _ => panic!(),
        });
    }
    fn rx_level(&mut self, level: u8) {
        self.events.borrow_mut().push("tail");
        self.level = level;
    }
}
#[derive(Default)]
struct Delay(usize);
impl DelayNs for Delay {
    fn delay_ns(&mut self, _: u32) {
        self.0 += 1;
    }
}
fn polls(n: u16) -> NonZeroU16 {
    NonZeroU16::new(n).unwrap()
}
fn pair(tx: bool, rx: bool) -> (Pair<Dma, Peripheral>, Events) {
    let events = Rc::new(RefCell::new(Vec::new()));
    let engine = |receive| {
        let mem = Range::new(if receive { 0x20030000 } else { 0x20020000 }, 8, 4).unwrap();
        let addr = if receive { 0x520300a0 } else { 0x520300c0 };
        let per = Range { start: addr, end: addr, count: 8, width: 4 };
        let (source, destination) = if receive { (per, mem) } else { (mem, per) };
        Engine {
            io: Dma {
                events: events.clone(),
                rx: receive,
                running: false,
                busy: false,
                valid: true,
                error: false,
                late_error: false,
                ready: 3,
                polls: 0,
                remaining: None,
                source,
                destination,
            },
            source,
            destination,
            state: State::Ready,
            stop_on_drop: false,
        }
    };
    (
        Pair {
            tx: tx.then(|| engine(false)),
            rx: rx.then(|| engine(true)),
            io: Peripheral {
                events: events.clone(),
                snapshot: Snapshot {
                    configured: true,
                    clock: false,
                    merge: false,
                    tx: [0, 0],
                    rx: [0, 0],
                    error: None,
                },
                valid: true,
                tail_valid: true,
                prefill: true,
                level: 0,
            },
            state: State::Ready,
            serial_started: false,
            level: 3,
            burst: Burst::Eight,
        },
        events,
    )
}
#[test]
fn sample_pairs_and_widths_reject_truncation() {
    for width in [DataWidth::Bits16, DataWidth::Bits18, DataWidth::Bits20, DataWidth::Bits24, DataWidth::Bits32] {
        let max = u32::MAX >> (32 - width.bits());
        assert!(valid_samples(&[0, max], width));
        assert!(!valid_samples(&[], width));
        assert!(!valid_samples(&[max], width));
        if width != DataWidth::Bits32 {
            assert!(!valid_samples(&[0, max + 1], width));
        }
    }
    let a = Range::new(0x20020000, 8, 4).unwrap();
    let b = Range::new(0x20030000, 8, 4).unwrap();
    assert_eq!(validate_ranges(Some(a), Some(b)), Ok(()));
    assert_eq!(validate_ranges(None, None), Err(Error::I2sFraming));
    assert_eq!(validate_ranges(Some(a), Some(a)), Err(Error::Overlap));
    let odd = Range::new(0x20030000, 7, 4).unwrap();
    assert_eq!(validate_ranges(None, Some(odd)), Err(Error::I2sFraming));
    let short = Range::new(0x20030000, 6, 4).unwrap();
    assert_eq!(validate_ranges(Some(a), Some(short)), Ok(()));
}
#[test]
fn i2s_routes_burst_and_protection_are_source_specific() {
    for (burst, code, count, level) in
        [(Burst::One, 0, 1, 0), (Burst::Four, 1, 4, 1), (Burst::Eight, 2, 8, 3), (Burst::Sixteen, 3, 16, 7)]
    {
        assert_eq!((burst.encoding(), burst.samples(), burst.level()), (code, count, level));
        for receive in [true, false] {
            let m = Mode::I2s { receive, burst };
            assert_eq!(m.requests(), if receive { (13, 0) } else { (0, 12) });
            assert_eq!(m.flow(), if receive { 2 } else { 1 });
            assert_eq!(m.protection(), 1);
            assert_eq!(m.burst(), code);
            assert_eq!(m.increments(), (!receive, receive));
        }
    }
}
#[test]
fn optional_directions_arm_rx_first_and_complete_without_wire_claim() {
    for (tx, rx, label) in [(true, true, "duplex"), (true, false, "transmit"), (false, true, "receive")] {
        let (mut p, e) = pair(tx, rx);
        p.start().unwrap();
        let before = e.borrow().clone();
        assert_eq!(before[0], "prepare");
        if tx && rx {
            assert_eq!(&before[1..], &["rx-config", "rx-start", "tx-config", "tx-start"]);
        }
        let done = p.wait(polls(5), &mut Delay::default()).unwrap();
        assert_eq!((done.tx_samples(), done.rx_samples()), (if tx { 8 } else { 0 }, if rx { 8 } else { 0 }));
        assert!(e.borrow().contains(&label));
        let after = e.borrow().len();
        p.stop();
        drop(p);
        assert_eq!(e.borrow().len(), after);
        // The fake exposes no stop-clock/reset/flush operation: completion cannot silently call one.
    }
}
#[test]
fn serializer_waits_for_a_complete_prefill_pair() {
    let (mut p, e) = pair(true, true);
    p.io.prefill = false;
    p.start().unwrap();
    p.io.snapshot.tx = [1, 0];
    let mut delay = Delay::default();
    assert_eq!(p.wait(polls(4), &mut delay), Err(Error::Timeout));
    assert_eq!(delay.0, 3);
    assert!(!e.borrow().contains(&"duplex"));
    assert_eq!(p.start(), Err(Error::NotReady));
}
#[test]
fn busy_or_bad_handoff_never_mutates_peripheral_or_channels() {
    for case in 0..7 {
        let (mut p, e) = pair(true, true);
        let expected = match case {
            0 => {
                p.tx.as_mut().unwrap().io.busy = true;
                Error::Occupied
            }
            1 => {
                p.rx.as_mut().unwrap().io.busy = true;
                Error::Occupied
            }
            2 => {
                p.io.snapshot.configured = false;
                Error::I2sConfiguration
            }
            3 => {
                p.io.snapshot.clock = true;
                Error::I2sBusy
            }
            4 => {
                p.io.snapshot.rx = [0, 1];
                Error::I2sBusy
            }
            5 => {
                p.io.snapshot.tx = [9, 0];
                Error::I2sConfiguration
            }
            _ => {
                p.io.snapshot.error = Some(Error::I2sHardware { raw: 8 });
                Error::I2sHardware { raw: 8 }
            }
        };
        assert_eq!(p.start(), Err(expected));
        drop(p);
        assert!(e.borrow().is_empty());
    }
}
#[test]
fn failed_configuration_stops_only_attempted_dma_channels() {
    for case in 0..3 {
        let (mut p, e) = pair(true, true);
        match case {
            0 => p.io.valid = false,
            1 => p.rx.as_mut().unwrap().io.valid = false,
            _ => p.tx.as_mut().unwrap().io.valid = false,
        };
        assert_eq!(p.start(), Err(if case == 0 { Error::I2sConfiguration } else { Error::Configuration }));
        drop(p);
        assert_eq!(e.borrow().iter().filter(|&&x| x == "tx-stop").count(), usize::from(case == 2));
        assert_eq!(e.borrow().iter().filter(|&&x| x == "rx-stop").count(), usize::from(case > 0));
    }
}
#[test]
fn one_budget_requires_both_dma_completions_and_prioritizes_bus_errors() {
    for receive in [false, true] {
        let (mut p, _) = pair(true, true);
        p.start().unwrap();
        let e = if receive { p.rx.as_mut().unwrap() } else { p.tx.as_mut().unwrap() };
        e.io.ready = 100;
        assert_eq!(p.wait(polls(4), &mut Delay::default()), Err(Error::Timeout));
    }
    let (mut p, _) = pair(true, true);
    p.start().unwrap();
    p.tx.as_mut().unwrap().io.error = true;
    p.io.snapshot.error = Some(Error::I2sHardware { raw: 32 });
    assert_eq!(p.wait(polls(1), &mut Delay::default()), Err(Error::Bus));
    let (mut p, _) = pair(true, true);
    p.start().unwrap();
    p.rx.as_mut().unwrap().io.late_error = true;
    assert_eq!(p.wait(polls(1), &mut Delay::default()), Err(Error::Bus));
}
#[test]
fn short_tail_lowering_is_once_checked_and_keeps_resources() {
    for valid in [true, false] {
        let (mut p, e) = pair(false, true);
        p.start().unwrap();
        p.rx.as_mut().unwrap().io.ready = 100;
        p.rx.as_mut().unwrap().io.remaining = Some(2);
        p.io.tail_valid = valid;
        assert_eq!(
            p.wait(polls(3), &mut Delay::default()),
            Err(if valid { Error::Timeout } else { Error::I2sConfiguration })
        );
        assert_eq!(e.borrow().iter().filter(|&&x| x == "tail").count(), 1);
    }
}
#[test]
fn peripheral_faults_clock_drift_and_invalid_progress_are_errors() {
    for case in 0..4 {
        let (mut p, _) = pair(true, true);
        p.start().unwrap();
        let expected = match case {
            0 => {
                p.io.snapshot.error = Some(Error::I2sHardware { raw: 4 });
                Error::I2sHardware { raw: 4 }
            }
            1 => {
                p.io.snapshot.clock = false;
                Error::I2sConfiguration
            }
            2 => {
                p.io.snapshot.rx = [0, 9];
                Error::I2sConfiguration
            }
            _ => {
                p.rx.as_mut().unwrap().io.remaining = Some(9);
                Error::CompletionState
            }
        };
        assert_eq!(p.wait(polls(3), &mut Delay::default()), Err(expected));
    }
}
#[test]
fn drop_forget_and_unwind_never_reclaim_or_restart() {
    let (mut p, e) = pair(true, true);
    p.start().unwrap();
    drop(p);
    assert!(e.borrow().ends_with(&["tx-stop", "rx-stop"]));
    let (mut p, e) = pair(true, true);
    p.start().unwrap();
    let n = e.borrow().len();
    core::mem::forget(p);
    assert_eq!(e.borrow().len(), n);
    struct Panic;
    impl DelayNs for Panic {
        fn delay_ns(&mut self, _: u32) {
            panic!("delay");
        }
    }
    let (mut p, e) = pair(true, true);
    p.start().unwrap();
    let r = std::panic::catch_unwind(std::panic::AssertUnwindSafe(move || {
        let _ = p.wait(polls(5), &mut Panic);
    }));
    assert!(r.is_err());
    assert!(e.borrow().ends_with(&["tx-stop", "rx-stop"]));
}
