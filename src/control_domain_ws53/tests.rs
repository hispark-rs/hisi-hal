use super::*;
use std::{cell::RefCell, collections::VecDeque, rc::Rc, vec, vec::Vec};

#[derive(Clone, Debug, PartialEq, Eq)]
enum Event {
    Status(u32),
    ReleaseCommand(u32),
    Fence,
    DelayNs(u32),
}

type Trace = Rc<RefCell<Vec<Event>>>;
struct Mock {
    values: VecDeque<u32>,
    trace: Trace,
}
impl Registers for Mock {
    fn reset_status(&mut self) -> u32 {
        let value = self.values.pop_front().expect("unexpected extra public-status read");
        self.trace.borrow_mut().push(Event::Status(value));
        value
    }
    fn release_logic_clock_only(&mut self) {
        self.trace.borrow_mut().push(Event::ReleaseCommand(3));
    }
    fn fence(&mut self) {
        self.trace.borrow_mut().push(Event::Fence);
    }
}
struct IndependentDelay {
    trace: Trace,
    panic_after_request: bool,
}
impl DelayNs for IndependentDelay {
    fn delay_ns(&mut self, ns: u32) {
        self.trace.borrow_mut().push(Event::DelayNs(ns));
        assert!(!self.panic_after_request, "injected unavailable independent timebase");
    }
}
fn fixture(values: &[u32]) -> (Mock, IndependentDelay, Trace) {
    let trace = Rc::new(RefCell::new(Vec::new()));
    (
        Mock { values: values.iter().copied().collect(), trace: trace.clone() },
        IndependentDelay { trace: trace.clone(), panic_after_request: false },
        trace,
    )
}

#[test]
fn exact_zero_releases_once_fences_waits_five_microseconds_then_checks_whole_word() {
    let (mut registers, mut delay, trace) = fixture(&[0, 3]);
    {
        let _lease = release_with(&mut registers, &mut delay).unwrap();
        assert_eq!(
            *trace.borrow(),
            vec![Event::Status(0), Event::ReleaseCommand(3), Event::Fence, Event::DelayNs(5_000), Event::Status(3)]
        );
        assert!(registers.values.is_empty());
    }
    // No implicit reset/clock gating, even when the lifetime ends immediately.
    assert_eq!(trace.borrow().len(), 5);
    assert!(!core::mem::needs_drop::<LogicClockLease>());
}

#[test]
fn every_nonzero_low_byte_is_rejected_without_command_fence_delay_or_second_read() {
    for initial in 1..=255 {
        let (mut registers, mut delay, trace) = fixture(&[initial]);
        assert!(matches!(release_with(&mut registers, &mut delay),
            Err(Error::UnsupportedInitialState(value)) if value == initial));
        assert_eq!(*trace.borrow(), vec![Event::Status(initial)]);
    }
}

#[test]
fn upper_bits_cannot_masquerade_as_zero_or_an_already_released_domain() {
    for initial in (3..32).flat_map(|bit| [1_u32 << bit, (1_u32 << bit) | 3, (1_u32 << bit) | 7]).chain([
        u32::MAX,
        0x8000_0000,
        0x8000_0003,
    ]) {
        let (mut registers, mut delay, trace) = fixture(&[initial]);
        assert!(matches!(release_with(&mut registers, &mut delay),
            Err(Error::UnsupportedInitialState(value)) if value == initial));
        assert_eq!(*trace.borrow(), vec![Event::Status(initial)]);
    }
}

#[test]
fn already_released_three_and_cpu_released_seven_are_not_attached_or_rewritten() {
    for initial in [3, 7] {
        let (mut registers, mut delay, trace) = fixture(&[initial]);
        assert!(matches!(release_with(&mut registers, &mut delay), Err(Error::UnsupportedInitialState(_))));
        assert_eq!(*trace.borrow(), vec![Event::Status(initial)]);
    }
}

#[test]
fn failed_readback_never_retries_asserts_reset_or_probes_another_register() {
    for observed in (0..=255).filter(|value| *value != 3).chain([0x8000_0003, u32::MAX]) {
        let (mut registers, mut delay, trace) = fixture(&[0, observed]);
        assert!(matches!(release_with(&mut registers, &mut delay),
            Err(Error::ReadbackMismatch(value)) if value == observed));
        assert_eq!(
            *trace.borrow(),
            vec![
                Event::Status(0),
                Event::ReleaseCommand(3),
                Event::Fence,
                Event::DelayNs(5_000),
                Event::Status(observed)
            ]
        );
        assert!(registers.values.is_empty());
    }
}

#[test]
fn delay_failure_keeps_unknown_state_and_does_not_rollback_or_read_held_peripherals() {
    let (mut registers, mut delay, trace) = fixture(&[0, 3]);
    delay.panic_after_request = true;
    let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
        let _ = release_with(&mut registers, &mut delay);
    }));
    assert!(result.is_err());
    assert_eq!(*trace.borrow(), vec![Event::Status(0), Event::ReleaseCommand(3), Event::Fence, Event::DelayNs(5_000)]);
    assert_eq!(registers.values, VecDeque::from([3]));
}
