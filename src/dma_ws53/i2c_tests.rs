use super::super::Status;
use super::*;
use std::{
    cell::{Cell, RefCell},
    rc::Rc,
    vec::Vec,
};
type Events = Rc<RefCell<Vec<&'static str>>>;
struct Dma {
    events: Events,
    rx: bool,
    running: bool,
    busy: bool,
    valid: bool,
    error: bool,
    late_error: bool,
    bad_done: bool,
    ready: usize,
    polls: usize,
    src: Range,
    dst: Range,
    tail: bool,
    lowered: Rc<Cell<bool>>,
}
impl Registers for Dma {
    fn status(&mut self) -> Status {
        if !self.running {
            return Status {
                enabled: self.busy,
                remaining: self.src.count,
                source: self.src.start,
                destination: self.dst.start,
                ..Status::default()
            };
        }
        self.polls += 1;
        let tail = self.tail && !self.lowered.get();
        let done = self.polls >= self.ready && !tail;
        Status {
            complete: done,
            error: self.error || self.late_error && self.polls >= 2,
            enabled: !done,
            active: !done,
            remaining: if tail {
                1
            } else if done && !self.bad_done {
                0
            } else {
                self.src.count
            },
            source: if done { self.src.end } else { self.src.start },
            destination: if done { self.dst.end } else { self.dst.start },
        }
    }
    fn configure(&mut self, s: Range, d: Range) {
        self.src = s;
        self.dst = d;
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
    enable_valid: bool,
    tail_valid: bool,
    lowered: Rc<Cell<bool>>,
}
impl I2cRegisters for Peripheral {
    fn snapshot(&mut self) -> Snapshot {
        self.snapshot
    }
    fn prepare(&mut self, _: u32) {
        self.events.borrow_mut().push("isolate");
    }
    fn requests(&mut self, tx: bool, rx: bool) {
        self.events.borrow_mut().push(if tx { if rx { "duplex-on" } else { "write-on" } } else { "off" });
    }
    fn rx_level(&mut self, level: u32) {
        assert_eq!(level, 0);
        self.lowered.set(true);
        self.events.borrow_mut().push("tail");
    }
    fn valid(&mut self, on: bool, _: bool, _: u32) -> bool {
        self.valid && (!on || self.enable_valid) && (!self.lowered.get() || self.tail_valid)
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
fn pair(receive: bool) -> (Pair<Dma, Peripheral>, Events) {
    let events = Rc::new(RefCell::new(Vec::new()));
    let lowered = Rc::new(Cell::new(false));
    let engine = |rx| {
        let width = if rx { 1 } else { 2 };
        let count = 17;
        let mem = Range::new(if rx { 0x20030000 } else { 0x20020000 }, count, width).unwrap();
        let peripheral = Range { start: 0x5208301c, end: 0x5208301c, count: count as u16, width };
        let (src, dst) = if rx { (peripheral, mem) } else { (mem, peripheral) };
        Engine {
            io: Dma {
                events: events.clone(),
                rx,
                running: false,
                busy: false,
                valid: true,
                error: false,
                late_error: false,
                bad_done: false,
                ready: 2,
                polls: 0,
                src,
                dst,
                tail: false,
                lowered: lowered.clone(),
            },
            source: src,
            destination: dst,
            state: State::Ready,
            stop_on_drop: false,
        }
    };
    (
        Pair {
            tx: engine(false),
            rx: receive.then(|| engine(true)),
            io: Peripheral {
                events: events.clone(),
                snapshot: Snapshot { configured: true, idle: true, stop: true, rx_count: 0, requests: 0, error: None },
                valid: true,
                enable_valid: true,
                tail_valid: true,
                lowered,
            },
            state: State::Ready,
            requests_owned: false,
            rx_level: 3,
        },
        events,
    )
}
fn buffer(n: usize) -> Buffer<u16> {
    let words = std::boxed::Box::leak(std::vec![0x5a5au16;n].into_boxed_slice());
    Buffer {
        pointer: core::ptr::NonNull::new(words.as_mut_ptr()).unwrap(),
        range: Range::new(0x20020000, n, 2).unwrap(),
        _owned: core::marker::PhantomData,
    }
}
#[test]
fn command_builders_validate_before_mutation_and_frame_only_final_stop() {
    for n in [1, 3, 4, 17, 4095] {
        let c = Commands::read(buffer(n));
        assert_eq!(usize::from(c.reads), n);
        let words = c.into_buffer().into_words();
        assert_eq!(words[n - 1], 0x300);
        assert!(words[..n - 1].iter().all(|v| *v == 0x100));
    }
    let c = Commands::write(buffer(3), &[0x20, 0x12, 0x34]).ok().unwrap();
    assert_eq!(c.reads, 0);
    assert_eq!(c.into_buffer().into_words(), &[0x20, 0x12, 0x234]);
    let c = Commands::write_read(buffer(5), &[0x20, 0x12], polls(3)).ok().unwrap();
    assert_eq!(c.reads, 3);
    assert_eq!(c.into_buffer().into_words(), &[0x20, 0x12, 0x500, 0x100, 0x300]);
    let c = Commands::write_read(buffer(2), &[0x20], polls(1)).ok().unwrap();
    assert_eq!(c.into_buffer().into_words(), &[0x20, 0x700]);
    for which in 0..3 {
        let b = buffer(3);
        let result = match which {
            0 => Commands::write(b, &[1]),
            1 => Commands::write_read(b, &[], polls(3)),
            _ => Commands::write_read(b, &[1], polls(3)),
        };
        let (e, b) = result.err().unwrap();
        assert_eq!(e, Error::LengthMismatch);
        assert_eq!(b.into_words(), &[0x5a5a; 3]);
    }
}
#[test]
fn route_width_protection_burst_are_i2c_specific() {
    for (port, tx) in [(Port::Zero, 0), (Port::One, 2)] {
        for rx in [false, true] {
            for burst in [Burst::One, Burst::Four] {
                let m = Mode::I2c { port, receive: rx, burst };
                assert_eq!(m.flow(), if rx { 2 } else { 1 });
                assert_eq!(m.requests(), if rx { (tx + 1, 0) } else { (0, tx) });
                assert_eq!(m.protection(), if rx { 2 } else { 1 });
                assert_eq!(m.burst(), u8::from(burst == Burst::Four));
                assert_eq!(m.increments(), (!rx, rx));
            }
        }
    }
    assert_eq!(Mode::Memory.burst(), 0);
}
#[test]
fn pure_write_and_duplex_arm_only_owned_channels_and_stop_once() {
    for rx in [false, true] {
        let (mut p, e) = pair(rx);
        p.start().unwrap();
        assert_eq!(
            &*e.borrow(),
            if rx {
                &["isolate", "rx-config", "rx-start", "tx-config", "tx-start", "duplex-on"][..]
            } else {
                &["isolate", "tx-config", "tx-start", "write-on"][..]
            }
        );
        assert_eq!(p.wait(polls(4), &mut Delay::default()).unwrap().elements(), 17);
        p.stop();
        drop(p);
        let e = e.borrow();
        assert_eq!(e.iter().filter(|x| **x == "off").count(), 1);
        assert_eq!(e.iter().filter(|x| **x == "tx-stop").count(), 1);
        assert_eq!(e.iter().filter(|x| **x == "rx-stop").count(), usize::from(rx));
    }
}
#[test]
fn busy_or_invalid_handoff_does_not_mutate_other_owners() {
    for case in 0..7 {
        let (mut p, e) = pair(true);
        let expected = match case {
            0 => {
                p.tx.io.busy = true;
                Error::Occupied
            }
            1 => {
                p.rx.as_mut().unwrap().io.busy = true;
                Error::Occupied
            }
            2 => {
                p.io.snapshot.configured = false;
                Error::I2cConfiguration
            }
            3 => {
                p.io.snapshot.idle = false;
                Error::I2cBusy
            }
            4 => {
                p.io.snapshot.rx_count = 1;
                Error::I2cBusy
            }
            5 => {
                p.io.snapshot.requests = 1;
                Error::I2cBusy
            }
            _ => {
                p.io.snapshot.error = Some(Error::I2cHardware { raw: 64, source: 0, detail: 1 });
                p.io.snapshot.error.unwrap()
            }
        };
        assert_eq!(p.start(), Err(expected));
        assert_eq!(p.start(), Err(Error::NotReady));
        drop(p);
        assert!(e.borrow().is_empty());
    }
}
#[test]
fn setup_failures_close_requests_and_stop_only_attempted_channels() {
    for rx in [false, true] {
        let (mut p, e) = pair(true);
        if rx {
            p.rx.as_mut().unwrap().io.valid = false;
        } else {
            p.tx.io.valid = false;
        }
        assert_eq!(p.start(), Err(Error::Configuration));
        drop(p);
        let e = e.borrow();
        assert!(!e.contains(&"duplex-on"));
        assert_eq!(e.iter().filter(|x| **x == "off").count(), 1);
        assert_eq!(e.iter().filter(|x| **x == "rx-stop").count(), 1);
        assert_eq!(e.iter().filter(|x| **x == "tx-stop").count(), usize::from(!rx));
    }
    let (mut p, e) = pair(true);
    p.io.enable_valid = false;
    assert_eq!(p.start(), Err(Error::I2cConfiguration));
    assert_eq!(&e.borrow()[6..], &["off", "tx-stop", "rx-stop"]);
}
#[test]
fn short_tail_changes_threshold_once_and_checks_readback() {
    for valid in [false, true] {
        let (mut p, e) = pair(true);
        p.rx.as_mut().unwrap().io.tail = true;
        p.io.tail_valid = valid;
        p.start().unwrap();
        let result = p.wait(polls(4), &mut Delay::default());
        if valid {
            assert_eq!(result.unwrap().elements(), 17);
        } else {
            assert_eq!(result, Err(Error::I2cConfiguration));
        }
        assert_eq!(e.borrow().iter().filter(|x| **x == "tail").count(), 1);
    }
}
#[test]
fn either_dma_error_wins_over_complete_and_second_progress_error_is_not_ignored() {
    for which in 0..3 {
        let (mut p, _) = pair(true);
        p.start().unwrap();
        match which {
            0 => p.tx.io.error = true,
            1 => p.rx.as_mut().unwrap().io.error = true,
            _ => p.rx.as_mut().unwrap().io.late_error = true,
        };
        assert_eq!(p.wait(polls(1), &mut Delay::default()), Err(Error::Bus));
    }
}
#[test]
fn completion_requires_both_channels_clean_fifo_idle_and_fresh_stop() {
    for case in 0..7 {
        let (mut p, _) = pair(true);
        p.start().unwrap();
        let expected = match case {
            0 => {
                p.rx.as_mut().unwrap().io.ready = usize::MAX;
                Error::Timeout
            }
            1 => {
                p.tx.io.bad_done = true;
                Error::CompletionState
            }
            2 => {
                p.io.snapshot.rx_count = 1;
                Error::I2cResidualReceive
            }
            3 => {
                p.io.snapshot.idle = false;
                Error::TransmitTimeout
            }
            4 => {
                p.io.snapshot.stop = false;
                Error::TransmitTimeout
            }
            5 => {
                p.io.snapshot.error = Some(Error::I2cHardware { raw: 64, source: 0, detail: 8 });
                p.io.snapshot.error.unwrap()
            }
            _ => {
                p.io.snapshot.configured = false;
                Error::I2cConfiguration
            }
        };
        assert_eq!(p.wait(polls(3), &mut Delay::default()), Err(expected));
        assert_eq!(p.wait(polls(1), &mut Delay::default()), Err(Error::NotReady));
    }
}
#[test]
fn drop_and_forget_never_reclaim_running_resources() {
    let (p, e) = pair(true);
    drop(p);
    assert!(e.borrow().is_empty());
    let (mut p, e) = pair(true);
    p.start().unwrap();
    drop(p);
    assert_eq!(&e.borrow()[6..], &["off", "tx-stop", "rx-stop"]);
    let (mut p, e) = pair(true);
    p.start().unwrap();
    core::mem::forget(p);
    assert_eq!(e.borrow().len(), 6);
}
#[test]
fn wait_is_finite_and_unwind_keeps_owners_until_drop() {
    let (mut p, _) = pair(true);
    p.rx.as_mut().unwrap().io.ready = usize::MAX;
    p.start().unwrap();
    let mut d = Delay::default();
    assert_eq!(p.wait(polls(5), &mut d), Err(Error::Timeout));
    assert_eq!(d.0, 4);
    struct Panic;
    impl DelayNs for Panic {
        fn delay_ns(&mut self, _: u32) {
            panic!("delay");
        }
    }
    let (mut p, e) = pair(true);
    p.rx.as_mut().unwrap().io.ready = usize::MAX;
    p.start().unwrap();
    assert!(std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| p.wait(polls(3), &mut Panic))).is_err());
    drop(p);
    assert_eq!(&e.borrow()[6..], &["off", "tx-stop", "rx-stop"]);
}
