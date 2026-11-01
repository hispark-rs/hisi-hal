use super::super::{Registers, Status};
use super::*;
use std::{cell::RefCell, rc::Rc};

fn snapshot() -> Snapshot {
    Snapshot { capability: true, configured: true, tx_idle: true, tx_count: 0, rx_count: 0, lines: 0 }
}
#[test]
fn capability_and_8n1_mode_are_required_not_silently_enabled() {
    let mut s = snapshot();
    assert_eq!(s.guard(), Ok(()));
    s.capability = false;
    assert_eq!(s.guard(), Err(Error::UartCapability));
    s.configured = false;
    assert_eq!(s.guard(), Err(Error::UartCapability));
    s.capability = true;
    assert_eq!(s.guard(), Err(Error::UartConfiguration));
}
#[test]
fn inherited_data_and_active_shifter_require_external_recovery() {
    for count in 1..=127 {
        let mut s = snapshot();
        s.tx_count = count;
        assert_eq!(s.guard(), Err(Error::UartBusy));
        s.tx_count = 0;
        s.rx_count = count;
        assert_eq!(s.guard(), Err(Error::UartBusy));
    }
    let mut s = snapshot();
    s.tx_idle = false;
    assert_eq!(s.guard(), Err(Error::UartBusy));
}
#[test]
fn all_line_flags_including_fifo_summary_are_preserved_and_take_priority_over_busy() {
    for flags in 1..32 {
        let mut s = snapshot();
        s.lines = flags;
        s.rx_count = 1;
        let e = LineErrors::from_flags(flags).unwrap();
        assert_eq!(s.guard(), Err(Error::UartLine(e)));
        assert_eq!(e.fifo_error(), flags & 1 != 0);
        assert_eq!(e.framing(), flags & 2 != 0);
        assert_eq!(e.parity(), flags & 4 != 0);
        assert_eq!(e.overrun(), flags & 8 != 0);
        assert_eq!(e.break_detected(), flags & 16 != 0);
    }
    assert_eq!(LineErrors::from_flags(0xe0), None);
}
#[test]
fn sealed_ports_select_actual_request_pairs_and_fixed_peripheral_progress() {
    assert_eq!(Port::of::<Uart0<'static>>(), Port::Uart0);
    assert_eq!(Port::of::<Uart1<'static>>(), Port::Uart1);
    for (port, tx, rx) in [(Port::Uart0, 10, 11), (Port::Uart1, 8, 9)] {
        let m = Mode::Uart { port, receive: false };
        assert_eq!(m.flow(), 1);
        assert_eq!(m.requests(), (0, tx));
        assert_eq!(m.increments(), (true, false));
        let m = Mode::Uart { port, receive: true };
        assert_eq!(m.flow(), 2);
        assert_eq!(m.requests(), (rx, 0));
        assert_eq!(m.increments(), (false, true));
    }
    assert_eq!(Mode::Memory.requests(), (0, 0));
    assert_eq!(Mode::Memory.increments(), (true, true));
}
#[derive(Default)]
struct Delay(usize);
impl DelayNs for Delay {
    fn delay_ns(&mut self, _: u32) {
        self.0 += 1;
    }
}
struct Lines {
    calls: usize,
    ready: usize,
    flags: u8,
}
impl LineRegisters for Lines {
    fn observe(&mut self) -> (bool, u8) {
        self.calls += 1;
        (self.calls >= self.ready, self.flags)
    }
}
#[test]
fn wire_idle_has_an_independent_finite_budget_not_a_fifo_only_completion() {
    for ready in [1, 3, 4] {
        let mut l = Lines { calls: 0, ready, flags: 0 };
        let mut d = Delay::default();
        assert_eq!(
            wait_idle(&mut l, NonZeroU16::new(3).unwrap(), &mut d),
            if ready <= 3 { Ok(()) } else { Err(Error::TransmitTimeout) }
        );
        assert_eq!(l.calls, ready.min(3));
        assert_eq!(d.0, ready.min(3) - 1);
    }
}
#[test]
fn line_error_wins_even_when_shift_register_reports_empty() {
    let mut l = Lines { calls: 0, ready: 1, flags: 4 };
    let mut d = Delay::default();
    assert_eq!(wait_idle(&mut l, NonZeroU16::new(3).unwrap(), &mut d), Err(Error::UartLine(LineErrors(4))));
    assert_eq!(d.0, 0);
}
#[derive(Default)]
struct Counters {
    configured: usize,
    started: usize,
    stopped: usize,
    line_reads: usize,
}
struct Io {
    counts: Rc<RefCell<Counters>>,
    prepare: Option<Error>,
    line: Option<Error>,
    dma_error: bool,
    source: Range,
    dest: Range,
}
impl Registers for Io {
    fn prepare(&mut self) -> Result<(), Error> {
        self.prepare.map_or(Ok(()), Err)
    }
    fn peripheral_error(&mut self) -> Option<Error> {
        self.counts.borrow_mut().line_reads += 1;
        self.line
    }
    fn configure(&mut self, s: Range, d: Range) {
        self.source = s;
        self.dest = d;
        self.counts.borrow_mut().configured += 1;
    }
    fn valid(&mut self, _: Range) -> bool {
        true
    }
    fn status(&mut self) -> Status {
        let started = self.counts.borrow().started != 0;
        Status {
            complete: started,
            error: started && self.dma_error,
            remaining: if started { 0 } else { self.source.count },
            source: if started { self.source.end } else { self.source.start },
            destination: if started { self.dest.end } else { self.dest.start },
            ..Status::default()
        }
    }
    fn start(&mut self) {
        self.counts.borrow_mut().started += 1;
    }
    fn stop(&mut self) {
        self.counts.borrow_mut().stopped += 1;
    }
}
fn engine() -> (Engine<Io>, Rc<RefCell<Counters>>) {
    let counts = Rc::new(RefCell::new(Counters::default()));
    let source = Range::new(0x20020000, 3, 1).unwrap();
    let dest = Range { start: 0x52081004, end: 0x52081004, count: 3, width: 1 };
    (
        Engine {
            io: Io { counts: counts.clone(), prepare: None, line: None, dma_error: false, source, dest },
            source,
            destination: dest,
            state: State::Ready,
            stop_on_drop: false,
        },
        counts,
    )
}
#[test]
fn failing_uart_prepare_never_configures_or_arms_dma_and_drop_stops_once() {
    let (mut e, c) = engine();
    e.io.prepare = Some(Error::UartCapability);
    assert_eq!(e.start(), Err(Error::UartCapability));
    assert_eq!(e.start(), Err(Error::NotReady));
    drop(e);
    let c = c.borrow();
    assert_eq!(c.configured, 0);
    assert_eq!(c.started, 0);
    assert_eq!(c.stopped, 1);
}
#[test]
fn peripheral_fault_retains_after_dma_completion_and_dma_error_has_priority() {
    for bus_error in [false, true] {
        let (mut e, c) = engine();
        e.io.line = Some(Error::UartLine(LineErrors(4)));
        e.io.dma_error = bus_error;
        e.start().unwrap();
        let mut d = Delay::default();
        assert_eq!(
            e.wait(NonZeroU16::new(1).unwrap(), &mut d),
            Err(if bus_error { Error::Bus } else { Error::UartLine(LineErrors(4)) })
        );
        assert_eq!(e.start(), Err(Error::NotReady));
        drop(e);
        assert_eq!(c.borrow().stopped, 1);
        assert_eq!(c.borrow().line_reads, usize::from(!bus_error));
    }
}
#[test]
fn observed_parameter_profile_is_not_the_cpu_register_access_width() {
    // UART_PARAMETER=0xD04, independently read on all three physical ports.
    assert!(supported_capability(true, 4, 1));
    assert!(!supported_capability(false, 4, 1));
    for width in [0, 2, 3] {
        assert!(!supported_capability(true, 4, width));
    }
    for depth in [0, 1, 2, 3, 5, 6, 7, 255] {
        assert!(!supported_capability(true, depth, 1));
    }
}
