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
    bad_done: bool,
    ready: usize,
    polls: usize,
    src: Range,
    dst: Range,
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
        let done = self.polls >= self.ready;
        Status {
            complete: done,
            error: self.error,
            enabled: !done,
            active: !done,
            remaining: if done && !self.bad_done { 0 } else { self.src.count },
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
}
impl SpiRegisters for Peripheral {
    fn snapshot(&mut self) -> Snapshot {
        self.snapshot
    }
    fn prepare(&mut self) {
        self.events.borrow_mut().push("isolate");
    }
    fn requests(&mut self, on: bool) {
        self.events.borrow_mut().push(if on { "requests-on" } else { "requests-off" });
    }
    fn valid(&mut self, on: bool) -> bool {
        self.valid && (!on || self.enable_valid)
    }
}
#[derive(Default)]
struct Delay(usize);
impl DelayNs for Delay {
    fn delay_ns(&mut self, _: u32) {
        self.0 += 1;
    }
}
fn pair() -> (Pair<Dma, Peripheral>, Events) {
    let events = Rc::new(RefCell::new(Vec::new()));
    let src = Range::new(0x20020000, 17, 4).unwrap();
    let dst = Range::new(0x20030000, 17, 4).unwrap();
    let engine = |rx| Engine {
        io: Dma {
            events: events.clone(),
            rx,
            running: false,
            busy: false,
            valid: true,
            error: false,
            bad_done: false,
            ready: 2,
            polls: 0,
            src,
            dst,
        },
        source: src,
        destination: dst,
        state: State::Ready,
        stop_on_drop: false,
    };
    (
        Pair {
            tx: engine(false),
            rx: engine(true),
            io: Peripheral {
                events: events.clone(),
                snapshot: Snapshot { configured: true, idle: true, rx_count: 0, requests: 0, error: None },
                valid: true,
                enable_valid: true,
            },
            state: State::Ready,
            requests_owned: false,
        },
        events,
    )
}
fn polls(n: u16) -> NonZeroU16 {
    NonZeroU16::new(n).unwrap()
}
#[test]
fn routes_and_protection_follow_spi_source_not_uart_aliases() {
    for (port, tx) in [(Port::Spi, 6), (Port::Qspi, 4)] {
        for rx in [false, true] {
            let m = Mode::Spi { port, receive: rx };
            assert_eq!(m.requests(), if rx { (tx + 1, 0) } else { (0, tx) });
            assert_eq!(m.flow(), if rx { 2 } else { 1 });
            assert_eq!(m.protection(), 2);
            assert_eq!(m.increments(), (!rx, rx));
        }
    }
}
#[test]
fn one_failure_domain_arms_receive_then_transmit_before_requests() {
    let (mut p, e) = pair();
    p.start().unwrap();
    assert_eq!(&*e.borrow(), &["isolate", "rx-config", "rx-start", "tx-config", "tx-start", "requests-on"]);
    let mut d = Delay::default();
    assert_eq!(p.wait(polls(3), &mut d).unwrap().elements(), 17);
    assert_eq!(d.0, 1);
    assert_eq!(&e.borrow()[6..], &["requests-off", "tx-stop", "rx-stop"]);
    p.stop();
    drop(p);
    assert_eq!(e.borrow().len(), 9);
}
#[test]
fn initial_busy_channel_never_touches_spi_or_other_channel() {
    for rx in [false, true] {
        let (mut p, e) = pair();
        if rx {
            p.rx.io.busy = true;
        } else {
            p.tx.io.busy = true;
        }
        assert_eq!(p.start(), Err(Error::Occupied));
        assert_eq!(p.start(), Err(Error::NotReady));
        drop(p);
        assert!(e.borrow().is_empty());
    }
}
#[test]
fn peripheral_handoff_rejects_modes_queues_requests_and_errors_without_mutation() {
    for which in 0..5 {
        let (mut p, e) = pair();
        let s = &mut p.io.snapshot;
        let expected = match which {
            0 => {
                s.configured = false;
                Error::SpiConfiguration
            }
            1 => {
                s.idle = false;
                Error::SpiBusy
            }
            2 => {
                s.rx_count = 1;
                Error::SpiBusy
            }
            3 => {
                s.requests = 1;
                Error::SpiBusy
            }
            _ => {
                s.error = Some(Error::SpiHardware { raw: 8 });
                Error::SpiHardware { raw: 8 }
            }
        };
        assert_eq!(p.start(), Err(expected));
        drop(p);
        assert!(e.borrow().is_empty());
    }
}
#[test]
fn failures_during_either_channel_setup_close_requests_and_every_attempted_channel() {
    for rx in [false, true] {
        let (mut p, e) = pair();
        if rx {
            p.rx.io.valid = false;
        } else {
            p.tx.io.valid = false;
        }
        assert_eq!(p.start(), Err(Error::Configuration));
        drop(p);
        let ev = e.borrow();
        assert!(!ev.contains(&"requests-on"));
        assert_eq!(ev.iter().filter(|v| **v == "rx-stop").count(), 1);
        assert_eq!(ev.iter().filter(|v| **v == "tx-stop").count(), usize::from(!rx));
        assert_eq!(ev.iter().filter(|v| **v == "requests-off").count(), 1);
    }
}
#[test]
fn request_readback_failure_stops_both_without_releasing_resources() {
    let (mut p, e) = pair();
    p.io.enable_valid = false;
    assert_eq!(p.start(), Err(Error::SpiConfiguration));
    assert_eq!(&e.borrow()[6..], &["requests-off", "tx-stop", "rx-stop"]);
    assert_eq!(p.start(), Err(Error::NotReady));
}
#[test]
fn either_channel_bus_error_wins_over_completion_and_peripheral_fault() {
    for rx in [false, true] {
        let (mut p, _) = pair();
        p.start().unwrap();
        p.io.snapshot.error = Some(Error::SpiHardware { raw: 2 });
        if rx {
            p.rx.io.error = true;
        } else {
            p.tx.io.error = true;
        }
        assert_eq!(p.wait(polls(1), &mut Delay::default()), Err(Error::Bus));
    }
}
#[test]
fn completion_requires_both_channels_valid_progress_no_residual_and_idle_wire() {
    for which in 0..5 {
        let (mut p, _) = pair();
        p.start().unwrap();
        let expected = match which {
            0 => {
                p.rx.io.ready = usize::MAX;
                Error::Timeout
            }
            1 => {
                p.tx.io.bad_done = true;
                Error::CompletionState
            }
            2 => {
                p.io.snapshot.rx_count = 1;
                Error::SpiResidualReceive
            }
            3 => {
                p.io.snapshot.idle = false;
                Error::TransmitTimeout
            }
            _ => {
                p.io.snapshot.error = Some(Error::SpiHardware { raw: 2 });
                Error::SpiHardware { raw: 2 }
            }
        };
        let mut d = Delay::default();
        assert_eq!(p.wait(polls(3), &mut d), Err(expected));
        assert!(d.0 <= 2);
        assert_eq!(p.wait(polls(1), &mut d), Err(Error::NotReady));
    }
}
#[test]
fn drop_cancels_pair_but_forget_retains_resources_and_running_requests() {
    let (p, e) = pair();
    drop(p);
    assert!(e.borrow().is_empty());
    let (mut p, e) = pair();
    p.start().unwrap();
    drop(p);
    assert_eq!(&e.borrow()[6..], &["requests-off", "tx-stop", "rx-stop"]);
    let (mut p, e) = pair();
    p.start().unwrap();
    core::mem::forget(p);
    assert_eq!(e.borrow().len(), 6);
}
#[test]
fn wait_has_one_finite_budget_and_panic_leaves_pair_owned_for_drop() {
    let (mut p, e) = pair();
    p.rx.io.ready = usize::MAX;
    p.start().unwrap();
    let mut d = Delay::default();
    assert_eq!(p.wait(polls(5), &mut d), Err(Error::Timeout));
    assert_eq!(d.0, 4);
    assert_eq!(p.tx.io.polls, 5);
    assert_eq!(p.rx.io.polls, 5);
    drop(p);
    assert_eq!(e.borrow().len(), 9);
    struct Panic;
    impl DelayNs for Panic {
        fn delay_ns(&mut self, _: u32) {
            panic!("clock failed");
        }
    }
    let (mut p, e) = pair();
    p.start().unwrap();
    assert!(std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| p.wait(polls(3), &mut Panic))).is_err());
    assert_eq!(p.start(), Err(Error::NotReady));
    drop(p);
    assert_eq!(e.borrow().len(), 9);
}
