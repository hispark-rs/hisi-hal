use super::*;
use std::{cell::RefCell, collections::VecDeque, rc::Rc, vec::Vec};
#[derive(Debug, PartialEq, Eq)]
enum Op {
    Clock(bool),
    Isolate,
    Enable(bool),
    Clear,
    Configure(Timing),
    Target(Address),
    Send(Command),
    Receive,
    Delay(u32),
}
type Trace = Rc<RefCell<Vec<Op>>>;
struct Mock {
    trace: Trace,
    enabled: bool,
    active: bool,
    stop: bool,
    rx: VecDeque<u8>,
    commands: Vec<Command>,
    fault: Option<Failure>,
    fail_after: Option<usize>,
    refuse_enable: bool,
    refuse_disable: bool,
    tx_stall: bool,
    rx_stall: bool,
    stop_stall: bool,
    late_abort: bool,
    fault_on_stop_sample: bool,
    clear_fault_on_disable: bool,
    activity_stall: bool,
}
impl Registers for Mock {
    fn clock(&mut self, on: bool) {
        self.trace.borrow_mut().push(Op::Clock(on));
    }
    fn isolate(&mut self) {
        self.trace.borrow_mut().push(Op::Isolate);
    }
    fn enable(&mut self, on: bool) {
        self.trace.borrow_mut().push(Op::Enable(on));
        if on {
            if !self.refuse_enable {
                self.enabled = true;
            }
        } else if !self.refuse_disable {
            self.enabled = false;
            self.active = false;
            if self.clear_fault_on_disable {
                self.fault = None;
            }
            if self.late_abort && !self.commands.is_empty() {
                self.fault = Some(nack());
            }
        }
    }
    fn status(&mut self) -> Status {
        if self.stop && self.fault_on_stop_sample {
            self.fault = Some(nack());
            self.fault_on_stop_sample = false;
        }
        Status {
            enabled: self.enabled,
            active: self.active || self.activity_stall,
            tx_ready: !self.tx_stall,
            tx_empty: true,
            rx_ready: !self.rx_stall && !self.rx.is_empty(),
            stop: self.stop && !self.stop_stall,
        }
    }
    fn fault(&mut self) -> Option<Failure> {
        self.fault
    }
    fn clear(&mut self) {
        self.trace.borrow_mut().push(Op::Clear);
        self.fault = None;
        self.stop = false;
    }
    fn configure(&mut self, t: Timing) {
        assert!(!self.enabled);
        self.trace.borrow_mut().push(Op::Configure(t));
    }
    fn target(&mut self, a: Address) {
        assert!(!self.enabled);
        self.trace.borrow_mut().push(Op::Target(a));
    }
    fn send(&mut self, c: Command) {
        assert!(self.enabled);
        self.trace.borrow_mut().push(Op::Send(c));
        self.commands.push(c);
        self.active = !c.stop;
        self.stop = c.stop;
        if self.fail_after == Some(self.commands.len()) {
            self.fault = Some(nack());
        }
    }
    fn receive(&mut self) -> u8 {
        self.trace.borrow_mut().push(Op::Receive);
        self.rx.pop_front().unwrap()
    }
}
struct Delay(Trace);
impl DelayNs for Delay {
    fn delay_ns(&mut self, ns: u32) {
        self.0.borrow_mut().push(Op::Delay(ns));
    }
}
fn engine() -> Engine<Mock, Delay> {
    let trace = Trace::default();
    let io = Mock {
        trace: trace.clone(),
        enabled: false,
        active: false,
        stop: false,
        rx: VecDeque::new(),
        commands: Vec::new(),
        fault: None,
        fail_after: None,
        refuse_enable: false,
        refuse_disable: false,
        tx_stall: false,
        rx_stall: false,
        stop_stall: false,
        late_abort: false,
        fault_on_stop_sample: false,
        clear_fault_on_disable: false,
        activity_stall: false,
    };
    let mut e =
        Engine::new(io, Delay(trace), Config::new(Timing::from_hz(100_000).unwrap(), NonZeroU16::new(3).unwrap()));
    e.recover().unwrap();
    e.io.trace.borrow_mut().clear();
    e
}
fn addr() -> Address {
    Address::seven(0x52).unwrap()
}
fn nack() -> Failure {
    Failure::Abort { source: 0, detail: 1 }
}
#[test]
fn timing_rejects_truncation_and_preserves_sdk_three_speed_profiles() {
    for (hz, h, l, mode) in [(100_000, 153, 159, 1), (400_000, 32, 40, 2), (800_000, 9, 23, 3)] {
        let t = Timing::from_hz(hz).unwrap();
        assert_eq!(t.counts(), (h, l));
        assert_eq!(t.mode, mode);
        assert_eq!(t.requested_hz(), hz);
    }
    for hz in [0, 1, 244, 914_286, 1_000_000, 1_000_001, u32::MAX] {
        assert!(Timing::from_hz(hz).is_none());
    }
    assert!(Timing::from_hz(245).is_some());
    for hz in 245..=914_285 {
        let t = Timing::from_hz(hz).unwrap();
        assert!(t.high >= 6 && t.low >= 12);
        assert!(u64::from(t.high) + u64::from(t.low) + 9 >= (32_000_000u64 + u64::from(hz)) / u64::from(hz));
    }
}
#[test]
fn address_width_is_explicit_and_never_truncated() {
    for a in 0..=255 {
        assert_eq!(Address::seven(a).is_some(), a < 128);
    }
    assert!(Address::ten(1023).is_some());
    assert!(Address::ten(1024).is_none());
    assert_ne!(Address::ten(42), Address::seven(42));
}
#[test]
fn adjacent_operations_have_exactly_one_stop_and_restart_only_on_direction_change() {
    let mut e = engine();
    e.io.rx = VecDeque::from([1, 2, 3]);
    let mut a = [0; 2];
    let mut b = [0; 1];
    let mut empty = [];
    e.transaction(
        addr(),
        &mut [
            Operation::Write(&[9]),
            Operation::Write(&[8, 7]),
            Operation::Read(&mut empty),
            Operation::Read(&mut a),
            Operation::Read(&mut b),
            Operation::Write(&[6]),
            Operation::Write(&[]),
        ],
    )
    .unwrap();
    assert_eq!(a, [1, 2]);
    assert_eq!(b, [3]);
    assert_eq!(
        e.io.commands.iter().map(|c| (c.data, c.read, c.restart, c.stop)).collect::<Vec<_>>(),
        [
            (9, false, false, false),
            (8, false, false, false),
            (7, false, false, false),
            (0, true, true, false),
            (0, true, false, false),
            (0, true, false, false),
            (6, false, true, true)
        ]
    );
    assert_eq!(e.state, State::Ready);
    assert!(!e.io.enabled);
    assert_eq!(e.io.trace.borrow().iter().filter(|o| matches!(o, Op::Target(_))).count(), 1);
}
#[test]
fn reads_longer_than_fifo_drain_each_command_before_enqueuing_the_next() {
    let mut e = engine();
    e.io.rx = (0..100).collect();
    let mut bytes = [0; 100];
    e.transaction(Address::ten(17).unwrap(), &mut [Operation::Read(&mut bytes)]).unwrap();
    assert_eq!(bytes, core::array::from_fn(|i| i as u8));
    let filtered =
        e.io.trace
            .borrow()
            .iter()
            .filter(|op| matches!(op, Op::Send(_) | Op::Receive))
            .map(|op| matches!(op, Op::Receive))
            .collect::<Vec<_>>();
    assert_eq!(filtered, (0..200).map(|i| i % 2 == 1).collect::<Vec<_>>());
    assert!(e.io.commands.last().unwrap().stop);
}
#[test]
fn empty_operations_never_touch_mmio_and_poisoned_noops_do_not_hide_faults() {
    let mut e = engine();
    let mut empty = [];
    e.transaction(addr(), &mut []).unwrap();
    e.transaction(addr(), &mut [Operation::Write(&[]), Operation::Read(&mut empty)]).unwrap();
    assert!(e.io.trace.borrow().is_empty());
    e.state = State::Poisoned;
    assert_eq!(e.transaction(addr(), &mut []), Err(Error::Poisoned));
}
#[test]
fn abort_during_write_preserves_first_reason_and_returns_to_ready_after_disable() {
    let mut e = engine();
    e.io.fail_after = Some(1);
    assert_eq!(e.transaction(addr(), &mut [Operation::Write(&[1, 2, 3])]), Err(Error::Failure(nack())));
    assert_eq!(e.io.commands.len(), 1);
    assert_eq!(e.state, State::Ready);
    assert!(!e.io.enabled);
    e.io.fail_after = None;
    e.transaction(addr(), &mut [Operation::Write(&[4])]).unwrap();
}
#[test]
fn late_abort_is_sampled_after_disable_before_clearing() {
    let mut e = engine();
    e.io.late_abort = true;
    assert_eq!(e.transaction(addr(), &mut [Operation::Write(&[1])]), Err(Error::Failure(nack())));
    assert_eq!(e.state, State::Ready);
}
#[test]
fn enable_tx_rx_and_stop_timeouts_are_bounded_and_cleanup_runs() {
    for stage in [Wait::Enable, Wait::Transmit, Wait::Receive, Wait::Stop] {
        let mut e = engine();
        match stage {
            Wait::Enable => e.io.refuse_enable = true,
            Wait::Transmit => e.io.tx_stall = true,
            Wait::Receive => e.io.rx_stall = true,
            Wait::Stop => e.io.stop_stall = true,
            _ => unreachable!(),
        }
        e.io.rx.push_back(7);
        let mut b = [0];
        let result = if stage == Wait::Receive {
            e.transaction(addr(), &mut [Operation::Read(&mut b)])
        } else {
            e.transaction(addr(), &mut [Operation::Write(&[1])])
        };
        assert_eq!(result, Err(Error::Failure(Failure::Timeout(stage))));
        assert_eq!(e.state, State::Ready);
        assert_eq!(e.io.trace.borrow().iter().filter(|o| matches!(o, Op::Delay(1000))).count(), 2);
        assert!(!e.io.enabled);
    }
}
#[test]
fn failed_cleanup_keeps_original_abort_and_blocks_reuse_until_recovery() {
    let mut e = engine();
    e.io.fail_after = Some(1);
    e.io.refuse_disable = true;
    assert_eq!(e.transaction(addr(), &mut [Operation::Write(&[1])]), Err(Error::RecoveryFailed(nack())));
    assert_eq!(e.state, State::Poisoned);
    e.io.trace.borrow_mut().clear();
    assert_eq!(e.transaction(addr(), &mut [Operation::Write(&[2])]), Err(Error::Poisoned));
    assert!(e.io.trace.borrow().is_empty());
    e.io.refuse_disable = false;
    e.io.fail_after = None;
    e.recover().unwrap();
    e.transaction(addr(), &mut [Operation::Write(&[3])]).unwrap();
}
#[test]
fn successful_transfer_with_failed_shutdown_is_not_reported_as_success() {
    let mut e = engine();
    e.io.refuse_disable = true;
    assert_eq!(
        e.transaction(addr(), &mut [Operation::Write(&[1])]),
        Err(Error::RecoveryFailed(Failure::Timeout(Wait::Disable)))
    );
    assert_eq!(e.state, State::Poisoned);
}
#[test]
fn close_gates_clock_only_after_inactivity_and_is_idempotent() {
    let mut e = engine();
    e.io.enabled = true;
    e.io.refuse_disable = true;
    assert_eq!(e.close(), Err(Error::Failure(Failure::Timeout(Wait::Disable))));
    assert!(!e.io.trace.borrow().contains(&Op::Clock(false)));
    e.io.refuse_disable = false;
    e.close().unwrap();
    assert_eq!(e.state, State::Closed);
    assert_eq!(e.io.trace.borrow().last(), Some(&Op::Clock(false)));
    e.io.trace.borrow_mut().clear();
    e.close().unwrap();
    assert!(e.io.trace.borrow().is_empty());
}
#[test]
fn error_kinds_distinguish_address_data_and_arbitration_with_priority() {
    use embedded_hal::i2c::Error as _;
    for (detail, kind) in [
        (1, ErrorKind::NoAcknowledge(NoAcknowledgeSource::Address)),
        (8, ErrorKind::NoAcknowledge(NoAcknowledgeSource::Data)),
        (0x1009, ErrorKind::ArbitrationLoss),
        (0x8000, ErrorKind::Other),
    ] {
        assert_eq!(Error::RecoveryFailed(Failure::Abort { source: 0, detail }).kind(), kind);
    }
    assert_eq!(Error::Failure(Failure::Fifo(2)).kind(), ErrorKind::Overrun);
    assert_eq!(Error::Failure(Failure::Fifo(1)).kind(), ErrorKind::Other);
}

#[test]
fn partial_read_failure_leaves_unreceived_buffer_bytes_unchanged() {
    let mut e = engine();
    e.io.rx = VecDeque::from([1, 2, 3, 4]);
    e.io.fail_after = Some(3);
    let mut bytes = [99; 4];
    assert_eq!(e.transaction(addr(), &mut [Operation::Read(&mut bytes)]), Err(Error::Failure(nack())));
    assert_eq!(bytes, [1, 2, 99, 99]);
    assert_eq!(e.io.commands.len(), 3);
    assert_eq!(e.state, State::Ready);
}

#[test]
fn initial_disable_timeout_never_programs_timing_or_claims_ready() {
    let mut e = engine();
    e.io.enabled = true;
    e.io.refuse_disable = true;
    assert_eq!(e.recover(), Err(Error::Failure(Failure::Timeout(Wait::Disable))));
    assert_eq!(e.state, State::Poisoned);
    assert!(!e.io.trace.borrow().iter().any(|o| matches!(o, Op::Configure(_) | Op::Target(_) | Op::Clear)));
}

#[test]
fn abort_between_final_fault_and_stop_reads_is_preserved_before_disable() {
    let mut e = engine();
    e.io.fault_on_stop_sample = true;
    e.io.clear_fault_on_disable = true;
    assert_eq!(e.transaction(addr(), &mut [Operation::Write(&[1])]), Err(Error::Failure(nack())));
    assert_eq!(e.state, State::Ready);
}

#[test]
fn disabled_status_alone_does_not_prove_bus_inactive() {
    let mut e = engine();
    e.io.activity_stall = true;
    assert_eq!(e.close(), Err(Error::Failure(Failure::Timeout(Wait::Disable))));
    assert_eq!(e.state, State::Poisoned);
    assert!(!e.io.trace.borrow().contains(&Op::Clock(false)));
}
