use super::*;
use core::num::NonZeroU32;
use std::{cell::RefCell, collections::VecDeque, rc::Rc, vec, vec::Vec};

#[derive(Debug, Clone, PartialEq, Eq)]
enum Event {
    LocalEnabled,
    LocalPending,
    ResetReleased,
    Clock(u16),
    Enable,
    Fence,
    Delay(u32),
    Configure(LineConfig, SignalPath),
}
type Trace = Rc<RefCell<Vec<Event>>>;
struct Mock {
    enabled: bool,
    pending: bool,
    released: bool,
    values: VecDeque<u16>,
    error: Option<ResetError>,
    trace: Trace,
}
impl Registers for Mock {
    fn local_enabled(&mut self) -> bool {
        self.trace.borrow_mut().push(Event::LocalEnabled);
        self.enabled
    }
    fn local_pending(&mut self) -> bool {
        self.trace.borrow_mut().push(Event::LocalPending);
        self.pending
    }
    fn reset_released(&mut self) -> bool {
        self.trace.borrow_mut().push(Event::ResetReleased);
        self.released
    }
    fn clock(&mut self) -> u16 {
        let v = self.values.pop_front().expect("unexpected clock read");
        self.trace.borrow_mut().push(Event::Clock(v));
        v
    }
    fn enable_h0(&mut self) {
        self.trace.borrow_mut().push(Event::Enable)
    }
    fn fence(&mut self) {
        self.trace.borrow_mut().push(Event::Fence)
    }
    fn configure(&mut self, c: LineConfig, p: SignalPath, _: &mut impl DelayNs) -> Result<(), ResetError> {
        self.trace.borrow_mut().push(Event::Configure(c, p));
        self.error.map_or(Ok(()), Err)
    }
}
struct Delay(Trace);
impl DelayNs for Delay {
    fn delay_ns(&mut self, n: u32) {
        self.0.borrow_mut().push(Event::Delay(n))
    }
}
fn config() -> LineConfig {
    LineConfig::new(
        NonZeroU32::new(32_000_000).unwrap(),
        115200,
        super::super::DataBits::Eight,
        super::super::Parity::None,
        super::super::StopDuration::One,
    )
    .unwrap()
}
fn fixture(values: &[u16]) -> (Mock, Delay, Trace) {
    let trace = Rc::new(RefCell::new(Vec::new()));
    (
        Mock {
            enabled: false,
            pending: false,
            released: true,
            values: values.iter().copied().collect(),
            error: None,
            trace: trace.clone(),
        },
        Delay(trace.clone()),
        trace,
    )
}

#[test]
fn first_uart_access_follows_whole_word_admission_and_independent_delay() {
    let (mut io, mut delay, trace) = fixture(&[0xf729, 0xf739, 0xf739]);
    assert_eq!(initialize(&mut io, config(), SignalPath::InternalLoopback, &mut delay), Ok(()));
    assert_eq!(
        *trace.borrow(),
        vec![
            Event::LocalEnabled,
            Event::LocalPending,
            Event::ResetReleased,
            Event::Clock(0xf729),
            Event::Enable,
            Event::Fence,
            Event::Delay(10_000),
            Event::Clock(0xf739),
            Event::Configure(config(), SignalPath::InternalLoopback),
            Event::Clock(0xf739)
        ]
    );
    assert!(io.values.is_empty());
}

#[test]
fn every_original_halfword_preserves_all_other_bits_and_never_rewrites_enabled_gate() {
    for before in 0..=u16::MAX {
        let after = before | 0x10;
        let (mut io, mut delay, trace) = fixture(&[before, after, after]);
        assert_eq!(initialize(&mut io, config(), SignalPath::Pins, &mut delay), Ok(()));
        let t = trace.borrow();
        assert_eq!(t.iter().filter(|x| **x == Event::Enable).count(), usize::from(before & 16 == 0));
        assert_eq!(t.iter().filter(|x| **x == Event::Delay(10_000)).count(), 1);
        assert_eq!(t.iter().filter(|x| matches!(x, Event::Configure(_, SignalPath::Pins))).count(), 1);
        assert!(io.values.is_empty());
    }
}

#[test]
fn enabled_local_irq_refuses_without_clock_reset_or_uart_mmio() {
    let (mut io, mut delay, trace) = fixture(&[]);
    io.enabled = true;
    assert_eq!(initialize(&mut io, config(), SignalPath::Pins, &mut delay), Err(H0InitError::InterruptsEnabled));
    assert_eq!(*trace.borrow(), vec![Event::LocalEnabled]);
}
#[test]
fn pending_local_irq_is_not_cleared_and_clock_is_not_touched() {
    let (mut io, mut delay, trace) = fixture(&[]);
    io.pending = true;
    assert_eq!(initialize(&mut io, config(), SignalPath::Pins, &mut delay), Err(H0InitError::PendingInterrupt));
    assert_eq!(*trace.borrow(), vec![Event::LocalEnabled, Event::LocalPending]);
}
#[test]
fn held_reset_is_not_probed_through_uart() {
    let (mut io, mut delay, trace) = fixture(&[]);
    io.released = false;
    assert_eq!(initialize(&mut io, config(), SignalPath::Pins, &mut delay), Err(H0InitError::ResetAsserted));
    assert_eq!(*trace.borrow(), vec![Event::LocalEnabled, Event::LocalPending, Event::ResetReleased]);
}

#[test]
fn every_readback_bit_fault_before_configure_refuses_without_uart_or_rollback() {
    for original in [0, 0xffff, 0xf729, 0x11] {
        for bit in 0..16 {
            let expected = original | 16;
            let observed = expected ^ (1 << bit);
            let (mut io, mut delay, trace) = fixture(&[original, observed]);
            assert_eq!(
                initialize(&mut io, config(), SignalPath::Pins, &mut delay),
                Err(H0InitError::ClockReadback { expected, observed })
            );
            let t = trace.borrow();
            assert!(!t.iter().any(|x| matches!(x, Event::Configure(..))));
            assert_eq!(t.iter().filter(|x| **x == Event::Enable).count(), usize::from(original & 16 == 0));
            assert!(io.values.is_empty());
        }
    }
}
#[test]
fn a_clock_change_during_configuration_is_reported_without_rollback() {
    for bit in 0..16 {
        let expected = 0xf739;
        let observed = expected ^ (1 << bit);
        let (mut io, mut delay, trace) = fixture(&[expected, expected, observed]);
        assert_eq!(
            initialize(&mut io, config(), SignalPath::Pins, &mut delay),
            Err(H0InitError::ClockReadback { expected, observed })
        );
        assert_eq!(trace.borrow().iter().filter(|x| matches!(x, Event::Configure(..))).count(), 1);
        assert!(!trace.borrow().contains(&Event::Enable));
    }
}
#[test]
fn reset_or_configuration_failure_stops_without_more_clock_access() {
    for error in [
        ResetError::AlreadyAsserted,
        ResetError::InterruptsEnabled,
        ResetError::TransmitBusy,
        ResetError::ResetReadback,
        ResetError::ConfigurationReadback,
    ] {
        let (mut io, mut delay, trace) = fixture(&[0, 16]);
        io.error = Some(error);
        assert_eq!(initialize(&mut io, config(), SignalPath::Pins, &mut delay), Err(H0InitError::Configuration(error)));
        assert_eq!(trace.borrow().last(), Some(&Event::Configure(config(), SignalPath::Pins)));
        assert!(io.values.is_empty());
    }
}
#[test]
fn failure_and_success_drop_have_no_implicit_hardware_cleanup() {
    assert!(!core::mem::needs_drop::<H0InitFailure<'_>>());
    assert!(!core::mem::needs_drop::<Uart<'_, Uart1<'_>>>());
}
