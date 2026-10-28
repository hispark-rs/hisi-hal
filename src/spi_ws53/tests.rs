use super::*;
use std::{cell::RefCell, collections::VecDeque, rc::Rc, vec::Vec};
#[derive(Clone, Debug, PartialEq, Eq)]
enum Op {
    Clocks,
    Isolate,
    Enable(bool),
    Select(bool),
    Configure(u16, u8),
    Clear,
    Send(u32),
    Receive(u32),
    Delay(u32),
}
type Trace = Rc<RefCell<Vec<Op>>>;
#[derive(Default)]
struct Mock {
    trace: Trace,
    enabled: bool,
    selected: bool,
    busy: bool,
    pending: bool,
    rx: VecDeque<u32>,
    tx_stall: bool,
    rx_stall: bool,
    idle_stall: bool,
    refuse_enable: bool,
    refuse_disable: bool,
    retain_tx: bool,
    retain_busy: bool,
    sticky_rx: bool,
    fail_on_send: Option<usize>,
    sent: usize,
    received: usize,
    fault: Option<Failure>,
    late_fault: bool,
    fault_on_idle: bool,
    clear_fault_on_disable: bool,
    extra_rx_on_idle: bool,
}
fn fault() -> Failure {
    Failure::Hardware { raw: 0x82, kind: embedded_hal::spi::ErrorKind::Overrun }
}
impl Registers for Mock {
    fn enable_clocks(&mut self) {
        self.trace.borrow_mut().push(Op::Clocks);
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
            self.pending = false;
            self.busy = false;
            if self.clear_fault_on_disable {
                self.fault = None;
            }
            if self.late_fault && self.sent > 0 {
                self.fault = Some(fault());
            }
        }
    }
    fn select(&mut self, on: bool) {
        self.selected = on;
        self.trace.borrow_mut().push(Op::Select(on));
    }
    fn configure(&mut self, c: Config, dfs: u8) {
        assert!(!self.enabled && !self.selected && self.rx.is_empty());
        self.trace.borrow_mut().push(Op::Configure(c.frequency.divider(), dfs));
    }
    fn status(&mut self) -> Status {
        if self.enabled && self.received > 0 && !self.pending {
            if self.fault_on_idle {
                self.fault = Some(fault());
                self.fault_on_idle = false;
            }
            if self.extra_rx_on_idle {
                self.rx.push_back(123);
                self.extra_rx_on_idle = false;
            }
        }
        Status {
            enabled: self.enabled,
            busy: self.busy || (self.idle_stall && self.enabled) || self.retain_busy,
            tx_ready: !self.tx_stall,
            tx_empty: !self.pending && !self.retain_tx,
            rx_ready: self.sticky_rx || !self.rx.is_empty(),
        }
    }
    fn fault(&mut self) -> Option<Failure> {
        self.fault
    }
    fn clear(&mut self) {
        self.fault = None;
        self.trace.borrow_mut().push(Op::Clear);
    }
    fn send(&mut self, word: u32) {
        assert!(self.enabled && self.selected && !self.pending && self.rx.is_empty());
        self.sent += 1;
        self.pending = true;
        self.busy = true;
        self.trace.borrow_mut().push(Op::Send(word));
        if !self.rx_stall {
            self.rx.push_back(word ^ 0xa5a5_a5a5);
        }
        if self.fail_on_send == Some(self.sent) {
            self.fault = Some(fault());
        }
    }
    fn receive(&mut self) -> u32 {
        let v = if self.sticky_rx { self.rx.front().copied().unwrap_or(99) } else { self.rx.pop_front().unwrap() };
        self.pending = false;
        self.busy = false;
        self.received += 1;
        self.trace.borrow_mut().push(Op::Receive(v));
        v
    }
}
struct Delay(Trace);
impl DelayNs for Delay {
    fn delay_ns(&mut self, ns: u32) {
        self.0.borrow_mut().push(Op::Delay(ns));
    }
}
fn engine<W: Word>() -> Engine<Mock, Delay, W> {
    let trace = Trace::default();
    let io = Mock { trace: trace.clone(), ..Mock::default() };
    let mut e = Engine::new(
        io,
        Delay(trace),
        Config::new(SpiHz::try_from_hz(10_000_000).unwrap(), MODE_0, NonZeroU16::new(3).unwrap()),
    );
    e.recover().unwrap();
    e.io.trace.borrow_mut().clear();
    e
}
fn sent<W: Word>(e: &Engine<Mock, Delay, W>) -> Vec<u32> {
    e.io.trace.borrow().iter().filter_map(|x| if let Op::Send(v) = x { Some(*v) } else { None }).collect()
}
#[test]
fn divider_bounds_never_exceed_requested_rate_or_truncate() {
    for hz in [0, 1, 488, 16_000_001, u32::MAX] {
        assert!(SpiHz::try_from_hz(hz).is_none());
    }
    for (hz, divider, actual) in [
        (489, 65440, 488),
        (1_000_000, 32, 1_000_000),
        (10_000_000, 4, 8_000_000),
        (12_000_000, 4, 8_000_000),
        (16_000_000, 2, 16_000_000),
    ] {
        let f = SpiHz::try_from_hz(hz).unwrap();
        assert_eq!((f.hz(), f.divider(), f.actual_hz()), (hz, divider, actual));
    }
    for hz in (489..=16_000_000).step_by(17) {
        let f = SpiHz::try_from_hz(hz).unwrap();
        let div = u64::from(f.divider());
        assert_eq!(div % 2, 0);
        assert!((2..=65534).contains(&div));
        assert!(u64::from(hz) * div >= 32_000_000);
        if div > 2 {
            assert!(u64::from(hz) * (div - 2) < 32_000_000);
        }
    }
}
#[test]
fn word_width_and_sample_timing_do_not_silently_change_configuration() {
    let mut e = engine::<u32>();
    e.recover().unwrap();
    assert!(e.io.trace.borrow().contains(&Op::Configure(4, 31)));
    let mut e = engine::<u8>();
    e.recover().unwrap();
    assert!(e.io.trace.borrow().contains(&Op::Configure(4, 7)));
    let c = e.config.with_sample_timing(255, 254);
    assert_eq!((c.rx_delay, c.tx_edge), (255, 254));
}
#[test]
fn unequal_transfers_clock_maximum_length_pad_zero_and_discard_extra_rx() {
    let mut e = engine::<u8>();
    let mut read = [0; 4];
    e.transfer(&mut read, &[1, 2]).unwrap();
    assert_eq!(read, [0xa4, 0xa7, 0xa5, 0xa5]);
    assert_eq!(sent(&e), [1, 2, 0, 0]);
    e.io.trace.borrow_mut().clear();
    let mut read = [0; 1];
    e.transfer(&mut read, &[3, 4, 5]).unwrap();
    assert_eq!(read, [0xa6]);
    assert_eq!(sent(&e), [3, 4, 5]);
    assert!(e.io.rx.is_empty());
}
#[test]
fn long_transfer_always_drains_rx_before_next_send() {
    let mut e = engine::<u32>();
    let input = [0x1234_5678; 100];
    let mut output = [0; 100];
    e.transfer(&mut output, &input).unwrap();
    assert_eq!(output, [0xb791_f3dd; 100]);
    let data: Vec<_> =
        e.io.trace.borrow().iter().filter(|x| matches!(x, Op::Send(_) | Op::Receive(_))).cloned().collect();
    assert_eq!(data.len(), 200);
    for pair in data.as_chunks::<2>().0 {
        assert!(matches!(pair, [Op::Send(_), Op::Receive(_)]));
    }
    assert!(!e.io.enabled && !e.io.selected);
    assert_eq!(e.state, State::Ready);
}
#[test]
fn in_place_preserves_unreceived_tail_on_error() {
    let mut e = engine::<u8>();
    e.io.fail_on_send = Some(3);
    let mut words = [1, 2, 3, 4];
    assert_eq!(e.transfer_in_place(&mut words), Err(Error::Failure(fault())));
    assert_eq!(words, [0xa4, 0xa7, 3, 4]);
    assert_eq!(sent(&e), [1, 2, 3]);
    assert_eq!(e.state, State::Ready);
    e.io.fail_on_send = None;
    e.transfer_in_place(&mut words[2..]).unwrap();
    assert_eq!(words, [0xa4, 0xa7, 0xa6, 0xa1]);
}
#[test]
fn empty_operations_do_not_touch_bus_and_cannot_hide_poison() {
    let mut e = engine::<u8>();
    e.transfer(&mut [], &[]).unwrap();
    e.transfer_in_place(&mut []).unwrap();
    assert!(e.io.trace.borrow().is_empty());
    e.state = State::Poisoned;
    assert_eq!(e.transfer(&mut [], &[]), Err(Error::Poisoned));
    assert_eq!(e.transfer_in_place(&mut []), Err(Error::Poisoned));
    assert_eq!(e.execute(false, |_| Ok(())), Err(Error::Poisoned));
    assert!(e.io.trace.borrow().is_empty());
}
#[test]
fn flush_checks_idle_without_enabling_or_clocking_data() {
    let mut e = engine::<u8>();
    e.execute(false, |_| Ok(())).unwrap();
    assert!(!e.io.trace.borrow().contains(&Op::Enable(true)));
    assert!(!e.io.trace.borrow().contains(&Op::Select(true)));
    assert!(sent(&e).is_empty());
    assert_eq!(e.state, State::Ready);
}
#[test]
fn every_transfer_wait_is_bounded_and_cleanup_allows_reuse() {
    for wait in [Wait::Enable, Wait::Transmit, Wait::Receive, Wait::Idle] {
        let mut e = engine::<u8>();
        match wait {
            Wait::Enable => e.io.refuse_enable = true,
            Wait::Transmit => e.io.tx_stall = true,
            Wait::Receive => e.io.rx_stall = true,
            Wait::Idle => e.io.idle_stall = true,
            _ => unreachable!(),
        }
        assert_eq!(e.transfer(&mut [], &[3]), Err(Error::Failure(Failure::Timeout(wait))));
        assert_eq!(e.state, State::Ready);
        assert_eq!(e.io.trace.borrow().iter().filter(|x| matches!(x, Op::Delay(1000))).count(), 2);
    }
}
#[test]
fn failed_cleanup_retains_original_fault_and_blocks_reuse_until_recovery() {
    let mut e = engine::<u8>();
    e.io.refuse_disable = true;
    e.io.fail_on_send = Some(1);
    assert_eq!(e.transfer(&mut [], &[3]), Err(Error::RecoveryFailed(fault())));
    assert_eq!(e.state, State::Poisoned);
    assert!(!e.io.trace.borrow().contains(&Op::Clear));
    assert_eq!(e.transfer(&mut [], &[4]), Err(Error::Poisoned));
    assert!(e.recover().is_err());
    e.io.refuse_disable = false;
    e.io.fail_on_send = None;
    e.recover().unwrap();
    e.transfer(&mut [], &[4]).unwrap();
    assert_eq!(e.state, State::Ready);
}
#[test]
fn successful_transfer_with_failed_disable_is_not_reported_successful() {
    let mut e = engine::<u8>();
    e.io.refuse_disable = true;
    assert_eq!(e.transfer(&mut [], &[3]), Err(Error::RecoveryFailed(Failure::Timeout(Wait::Disable))));
    assert_eq!(e.state, State::Poisoned);
}
#[test]
fn late_faults_are_sampled_before_and_after_disable() {
    for after_disable in [false, true] {
        let mut e = engine::<u8>();
        e.io.clear_fault_on_disable = true;
        e.io.fault_on_idle = !after_disable;
        e.io.late_fault = after_disable;
        assert_eq!(e.transfer(&mut [], &[3]), Err(Error::Failure(fault())));
        assert_eq!(e.state, State::Ready);
    }
}
#[test]
fn unexpected_receive_data_is_not_misattributed_to_next_request() {
    let mut e = engine::<u8>();
    e.io.rx.push_back(99);
    let mut out = [0x42];
    assert_eq!(e.transfer(&mut out, &[1]), Err(Error::Failure(Failure::UnexpectedReceive)));
    assert_eq!(out, [0x42]);
    assert!(sent(&e).is_empty());
    e.io.extra_rx_on_idle = true;
    assert_eq!(e.transfer(&mut out, &[1]), Err(Error::Failure(Failure::UnexpectedReceive)));
    assert_eq!(e.state, State::Ready);
    assert!(e.io.rx.is_empty());
}
#[test]
fn recovery_cannot_claim_idle_from_enable_readback_alone() {
    for busy in [false, true] {
        let mut e = engine::<u8>();
        e.io.retain_busy = busy;
        e.io.retain_tx = !busy;
        e.io.trace.borrow_mut().clear();
        assert_eq!(e.recover(), Err(Error::Failure(Failure::Timeout(Wait::Disable))));
        assert_eq!(e.state, State::Poisoned);
        assert!(!e.io.trace.borrow().iter().any(|x| matches!(x, Op::Configure(..) | Op::Clear)));
    }
}
#[test]
fn drain_has_a_bound_without_inventing_fifo_capacity() {
    let mut e = engine::<u8>();
    e.io.sticky_rx = true;
    assert_eq!(e.recover(), Err(Error::Failure(Failure::Timeout(Wait::Drain))));
    assert_eq!(e.io.trace.borrow().iter().filter(|x| matches!(x, Op::Receive(_))).count(), 3);
    e.io.sticky_rx = false;
    e.io.rx = VecDeque::from([1, 2, 3]);
    e.recover().unwrap();
    assert!(e.io.rx.is_empty());
}
#[test]
fn close_is_idempotent_and_failed_close_keeps_ownership_state_poisoned() {
    let mut e = engine::<u8>();
    e.io.enabled = true;
    e.io.refuse_disable = true;
    assert!(e.close().is_err());
    assert_eq!(e.state, State::Poisoned);
    e.io.refuse_disable = false;
    e.close().unwrap();
    assert_eq!(e.state, State::Closed);
    e.io.trace.borrow_mut().clear();
    e.close().unwrap();
    assert!(e.io.trace.borrow().is_empty());
}
#[test]
fn error_kind_preserves_normalized_hardware_cause() {
    use embedded_hal::spi::{Error as _, ErrorKind};
    assert_eq!(Error::Failure(fault()).kind(), ErrorKind::Overrun);
    assert_eq!(
        Error::RecoveryFailed(Failure::Hardware { raw: 0x8000, kind: ErrorKind::ModeFault }).kind(),
        ErrorKind::ModeFault
    );
    assert_eq!(Error::Poisoned.kind(), ErrorKind::Other);
}
