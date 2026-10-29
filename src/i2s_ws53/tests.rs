use super::*;
use std::{cell::RefCell, collections::VecDeque, rc::Rc, vec::Vec};
#[derive(Clone, Debug, PartialEq, Eq)]
enum Op {
    Clocks,
    Stop,
    Configure(Config),
    Clock(bool),
    Start(StreamMode),
    Push(Frame),
    Pop(Frame),
    Delay(u32),
}
type Trace = Rc<RefCell<Vec<Op>>>;
struct Mock {
    trace: Trace,
    tx: VecDeque<Frame>,
    rx: VecDeque<Frame>,
    running: bool,
    clock: bool,
    configured: bool,
    faults: Faults,
    snapshots: usize,
    retain: bool,
    refuse_config: bool,
    refuse_clock: bool,
    bad_depth: bool,
    asymmetric_rx: bool,
    advance: bool,
    shift_busy: bool,
}
impl Mock {
    fn new(trace: Trace) -> Self {
        Self {
            trace,
            tx: VecDeque::new(),
            rx: VecDeque::new(),
            running: false,
            clock: false,
            configured: false,
            faults: Faults::default(),
            snapshots: 0,
            retain: false,
            refuse_config: false,
            refuse_clock: false,
            bad_depth: false,
            asymmetric_rx: false,
            advance: false,
            shift_busy: false,
        }
    }
}
impl Registers for Mock {
    fn enable_clocks(&mut self) {
        self.trace.borrow_mut().push(Op::Clocks);
    }
    fn stop_request(&mut self) {
        self.trace.borrow_mut().push(Op::Stop);
        self.clock = false;
        self.running = false;
    }
    fn reset_configure(&mut self, c: Config) {
        self.trace.borrow_mut().push(Op::Configure(c));
        self.configured = !self.refuse_config;
        if !self.retain {
            self.tx.clear();
            self.rx.clear();
            self.faults = Faults::default();
            self.shift_busy = false;
        }
    }
    fn snapshot(&mut self, _: Config) -> Snapshot {
        self.snapshots += 1;
        if self.advance && self.running && self.clock && self.tx.pop_front().is_some() {
            self.shift_busy = true;
        }
        let tx = if self.bad_depth { [9, 0] } else { [self.tx.len() as u8; 2] };
        let rx = if self.asymmetric_rx { [self.rx.len() as u8, 0] } else { [self.rx.len() as u8; 2] };
        Snapshot { tx, rx, clock: self.clock, configured: self.configured, faults: self.faults }
    }
    fn clock(&mut self, on: bool) {
        self.trace.borrow_mut().push(Op::Clock(on));
        self.clock = on && !self.refuse_clock;
    }
    fn start(&mut self, mode: StreamMode) {
        self.trace.borrow_mut().push(Op::Start(mode));
        self.running = true;
    }
    fn push(&mut self, f: Frame) {
        assert!(self.tx.len() < 8);
        self.trace.borrow_mut().push(Op::Push(f));
        self.tx.push_back(f);
    }
    fn pop(&mut self) -> Frame {
        assert!(!self.asymmetric_rx);
        let f = self.rx.pop_front().unwrap();
        self.trace.borrow_mut().push(Op::Pop(f));
        f
    }
}
struct Delay(Trace);
impl DelayNs for Delay {
    fn delay_ns(&mut self, ns: u32) {
        self.0.borrow_mut().push(Op::Delay(ns));
    }
}
fn config() -> Config {
    Config::new(DataWidth::Bits16, ClockMode::Master(BclkDivider::new(40).unwrap()), NonZeroU16::new(3).unwrap())
}
fn engine() -> Engine<Mock, Delay> {
    let t = Trace::default();
    Engine::new(Mock::new(t.clone()), Delay(t), config())
}
fn frame(n: u32) -> Frame {
    Frame { left: n, right: n + 1 }
}
fn running(mode: StreamMode) -> Engine<Mock, Delay> {
    let mut e = engine();
    e.recover().unwrap();
    if mode.tx() {
        e.start(mode, &[frame(0)]).unwrap();
    } else {
        e.start(mode, &[]).unwrap();
    }
    e
}
#[test]
fn divider_rejects_every_out_of_range_u16() {
    for n in 0..=u16::MAX {
        assert_eq!(BclkDivider::new(n).map(BclkDivider::get), (1..=127).contains(&n).then_some(n as u8));
    }
}
#[test]
fn sample_encodings_and_stereo_dividers_match_source_not_legacy_prose() {
    for (width, bits, code) in [
        (DataWidth::Bits16, 16, 1),
        (DataWidth::Bits18, 18, 2),
        (DataWidth::Bits20, 20, 3),
        (DataWidth::Bits24, 24, 4),
        (DataWidth::Bits32, 32, 5),
    ] {
        let c = Config::new(width, ClockMode::Slave, NonZeroU16::new(1).unwrap());
        assert_eq!((width.bits(), width as u8, c.frame_bits()), (bits, code, u16::from(bits) * 2));
        let mask = u32::MAX >> (32 - bits);
        assert!(width.contains(Frame { left: mask, right: mask }));
        if bits < 32 {
            assert!(!width.contains(Frame { left: mask + 1, right: 0 }));
            assert!(!width.contains(Frame { left: 0, right: mask + 1 }));
        }
    }
}
#[test]
fn recovery_sequences_clocks_stop_and_configuration() {
    let mut e = engine();
    e.recover().unwrap();
    assert_eq!(e.state, State::Ready);
    assert!(e.accessed);
    assert_eq!(*e.io.trace.borrow(), [Op::Clocks, Op::Stop, Op::Configure(config())]);
}
#[test]
fn prefill_precedes_clock_setup_delay_and_start() {
    let e = running(StreamMode::Duplex);
    assert_eq!(
        &e.io.trace.borrow()[3..],
        &[Op::Push(frame(0)), Op::Clock(true), Op::Delay(25000), Op::Start(StreamMode::Duplex)]
    );
}
#[test]
fn rejected_prefill_and_samples_do_not_touch_hardware_or_poison() {
    let mut e = engine();
    e.recover().unwrap();
    let start = e.io.trace.borrow().len();
    let reads = e.io.snapshots;
    assert_eq!(e.start(StreamMode::Transmit, &[]), Err(Error::InvalidPrefill));
    assert_eq!(e.start(StreamMode::Duplex, &[frame(0); 9]), Err(Error::InvalidPrefill));
    assert_eq!(e.start(StreamMode::Receive, &[frame(0)]), Err(Error::InvalidPrefill));
    assert_eq!(e.start(StreamMode::Transmit, &[frame(0x10000)]), Err(Error::InvalidSample));
    assert_eq!(e.io.trace.borrow().len(), start);
    assert_eq!(e.io.snapshots, reads);
    assert_eq!(e.state, State::Ready);
}
#[test]
fn wrong_direction_empty_calls_and_double_start_are_explicit() {
    let mut e = running(StreamMode::Receive);
    assert_eq!(e.try_queue(frame(1)), Err(Error::WrongDirection));
    assert_eq!(e.queue(&[]), Err(TransferError { cause: Error::WrongDirection, frames: 0 }));
    assert_eq!(e.start(StreamMode::Receive, &[]), Err(Error::NotReady));
    assert_eq!(e.read(&mut []), Ok(()));
    let mut e = running(StreamMode::Transmit);
    assert_eq!(e.try_read(), Err(Error::WrongDirection));
    assert_eq!(e.queue(&[]), Ok(()));
}
#[test]
fn queue_timeout_preserves_partial_count_and_requires_explicit_recovery() {
    let mut e = running(StreamMode::Transmit);
    assert_eq!(e.queue(&[frame(2); 9]), Err(TransferError { cause: Error::Timeout(Wait::TxSpace), frames: 7 }));
    assert_eq!(e.state, State::Poisoned);
    assert!(!e.io.running);
    assert_eq!(e.io.tx.len(), 8);
    assert_eq!(e.try_queue(frame(9)), Err(Error::Poisoned));
    e.recover().unwrap();
    assert_eq!(e.state, State::Ready);
    assert!(e.io.tx.is_empty());
    e.start(StreamMode::Transmit, &[frame(3)]).unwrap();
}
#[test]
fn invalid_later_sample_is_rejected_before_any_queue_progress() {
    let mut e = running(StreamMode::Transmit);
    let count = e.io.tx.len();
    let reads = e.io.snapshots;
    assert_eq!(e.queue(&[frame(1), frame(0x10000)]), Err(TransferError { cause: Error::InvalidSample, frames: 0 }));
    assert_eq!(e.io.tx.len(), count);
    assert_eq!(e.io.snapshots, reads);
    assert_eq!(e.state, State::Running(StreamMode::Transmit));
}
#[test]
fn nonblocking_full_fifo_does_not_wait_or_poison() {
    let mut e = running(StreamMode::Transmit);
    e.queue(&[frame(2); 7]).unwrap();
    let n = e.io.trace.borrow().len();
    assert_eq!(e.try_queue(frame(9)), Ok(false));
    assert_eq!(e.io.trace.borrow().len(), n);
    assert_eq!(e.state, State::Running(StreamMode::Transmit));
}
#[test]
fn short_rx_burst_below_irq_threshold_is_returned_and_raw_bits_preserved() {
    let mut e = running(StreamMode::Receive);
    let f = Frame { left: 0xdeadbeef, right: 0xaabbccdd };
    e.io.rx.push_back(f);
    assert_eq!(e.try_read(), Ok(Some(f)));
    assert_eq!(e.try_read(), Ok(None));
    assert_eq!(e.state, State::Running(StreamMode::Receive));
}
#[test]
fn rx_timeout_reports_prefix_and_does_not_overwrite_remaining_buffer() {
    let mut e = running(StreamMode::Receive);
    e.io.rx.extend([frame(1), frame(4)]);
    let mut out = [frame(88); 3];
    assert_eq!(e.read(&mut out), Err(TransferError { cause: Error::Timeout(Wait::RxData), frames: 2 }));
    assert_eq!(out, [frame(1), frame(4), frame(88)]);
    assert_eq!(e.state, State::Poisoned);
}
#[test]
fn unpaired_rx_never_consumes_a_single_side() {
    let mut e = running(StreamMode::Receive);
    e.io.rx.push_back(frame(7));
    e.io.asymmetric_rx = true;
    assert_eq!(e.try_read(), Ok(None));
    let mut out = [frame(0)];
    assert_eq!(e.read(&mut out), Err(TransferError { cause: Error::Timeout(Wait::RxData), frames: 0 }));
    assert_eq!(e.io.rx.len(), 1);
    assert!(!e.io.trace.borrow().iter().any(|v| matches!(v, Op::Pop(_))));
}
#[test]
fn fifo_empty_is_not_wire_completion_and_does_not_stop_stream() {
    let mut e = running(StreamMode::Transmit);
    e.io.advance = true;
    e.wait(Wait::TxEmpty).unwrap();
    assert!(e.io.shift_busy);
    assert!(e.io.running);
    assert_eq!(e.state, State::Running(StreamMode::Transmit));
}
#[test]
fn each_reported_active_error_poisons_without_acknowledging_it() {
    for faults in [
        Faults { rx_left: true, ..Faults::default() },
        Faults { rx_right: true, ..Faults::default() },
        Faults { tx_left: true, ..Faults::default() },
        Faults { tx_right: true, ..Faults::default() },
    ] {
        let mut e = running(StreamMode::Duplex);
        e.io.faults = faults;
        assert!(matches!(e.inspect(),Err(Error::Hardware(f)) if f==faults));
        assert_eq!(e.io.faults, faults);
        assert_eq!(e.state, State::Poisoned);
        assert!(!e.io.running);
    }
}
#[test]
fn inactive_direction_fault_does_not_mask_useful_rx() {
    let mut e = running(StreamMode::Receive);
    e.io.faults.tx_left = true;
    e.io.rx.push_back(frame(5));
    assert_eq!(e.try_read(), Ok(Some(frame(5))));
}
#[test]
fn clock_loss_config_change_and_invalid_depth_stop_before_fifo_access() {
    for cause in [Error::ClockStopped, Error::ConfigurationChanged, Error::InvalidDepth] {
        let mut e = running(StreamMode::Duplex);
        let n = e.io.tx.len();
        match cause {
            Error::ClockStopped => e.io.clock = false,
            Error::ConfigurationChanged => e.io.configured = false,
            _ => e.io.bad_depth = true,
        }
        assert_eq!(e.try_queue(frame(9)), Err(cause));
        assert_eq!(e.io.tx.len(), n);
        assert_eq!(e.state, State::Poisoned);
    }
}
#[test]
fn failed_clock_start_and_stuck_reset_are_bounded_and_recoverable() {
    let mut e = engine();
    e.recover().unwrap();
    e.io.refuse_clock = true;
    assert_eq!(e.start(StreamMode::Transmit, &[frame(1)]), Err(Error::ClockStopped));
    e.io.refuse_clock = false;
    e.io.retain = true;
    assert_eq!(e.recover(), Err(Error::Timeout(Wait::Configure)));
    assert_eq!(e.state, State::Poisoned);
    e.io.retain = false;
    e.recover().unwrap();
    assert_eq!(e.state, State::Ready);
}
#[test]
fn refused_configuration_uses_exact_poll_budget_and_only_between_attempt_delays() {
    let mut e = engine();
    e.io.refuse_config = true;
    assert_eq!(e.recover(), Err(Error::Timeout(Wait::Configure)));
    assert_eq!(e.io.snapshots, 3);
    assert_eq!(e.io.trace.borrow().iter().filter(|v| **v == Op::Delay(1000)).count(), 2);
    assert_eq!(e.io.trace.borrow().last(), Some(&Op::Stop));
}
#[test]
fn streaming_progress_handles_more_than_one_fifo_capacity() {
    let mut e = running(StreamMode::Transmit);
    e.io.advance = true;
    e.queue(&[frame(7); 100]).unwrap();
    assert_eq!(e.io.trace.borrow().iter().filter(|v| matches!(v, Op::Push(_))).count(), 101);
    assert_eq!(e.state, State::Running(StreamMode::Transmit));
}

#[test]
fn mismatched_pin_clock_direction_returns_resources_and_drop_performs_no_mmio() {
    use crate::soc::ws53::pins::Pad;
    // Host-only metadata: the invalid role must return before any physical MMIO,
    // including during recovery retry and Drop of the returned driver.
    let pins = unsafe {
        Pins::from_configured(
            PadControl::claim(Pad::Mgpio17).unwrap(),
            PadControl::claim(Pad::Mgpio18).unwrap(),
            PadControl::claim(Pad::Mgpio19).unwrap(),
            PadControl::claim(Pad::Mgpio20).unwrap(),
            ClockMode::Slave,
        )
    };
    let token = unsafe { I2s::steal() };
    let Err((err, mut stream)) = I2sStream::new(token, pins, config(), Delay(Trace::default())) else {
        panic!("role must fail")
    };
    assert_eq!(err, Error::PinClockDirection);
    assert!(!stream.engine.accessed);
    assert!(stream.token.is_some() && stream.pins.is_some());
    assert_eq!(stream.discard_and_recover(), Err(Error::PinClockDirection));
    drop(stream);
}
