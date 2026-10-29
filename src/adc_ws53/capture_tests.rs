use super::*;
use std::{cell::RefCell, rc::Rc, vec::Vec};
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Event {
    Configure,
    Start,
    Status,
    Stop,
    Delay,
}
struct Mock {
    events: Rc<RefCell<Vec<Event>>>,
    idle: bool,
    valid: bool,
    stale: bool,
    started: bool,
    ready: usize,
    polls: usize,
    address: u32,
}
impl CaptureRegisters for Mock {
    fn idle(&mut self) -> bool {
        self.idle
    }
    fn configure(&mut self, _: Range, _: u8) {
        self.events.borrow_mut().push(Event::Configure);
    }
    fn valid(&mut self, _: Range, _: u8, _: bool) -> bool {
        self.valid
    }
    fn status(&mut self) -> (bool, u32) {
        self.events.borrow_mut().push(Event::Status);
        if !self.started {
            return (self.stale, 0);
        }
        self.polls += 1;
        (self.polls >= self.ready, self.address)
    }
    fn start(&mut self) {
        self.started = true;
        self.events.borrow_mut().push(Event::Start);
    }
    fn stop(&mut self) {
        self.events.borrow_mut().push(Event::Stop);
    }
}
struct Delay(Rc<RefCell<Vec<Event>>>);
impl DelayNs for Delay {
    fn delay_ns(&mut self, _: u32) {
        self.0.borrow_mut().push(Event::Delay);
    }
}
fn engine() -> (Engine<Mock>, Delay) {
    let events = Rc::new(RefCell::new(Vec::new()));
    (
        Engine {
            io: Mock {
                events: events.clone(),
                idle: true,
                valid: true,
                stale: false,
                started: false,
                ready: 2,
                polls: 0,
                address: 0x20050004,
            },
            range: Range::new(0x20050000, 2).unwrap(),
            channels: 1,
            state: State::Ready,
            stop_on_drop: false,
        },
        Delay(events),
    )
}
fn polls(n: u16) -> NonZeroU16 {
    NonZeroU16::new(n).unwrap()
}
#[test]
fn dtcm_bounds_checked_without_truncation_or_wrapping() {
    assert_eq!(Range::new(0x20010000, 0), Err(BufferError::Empty));
    for (start, length) in [
        (0x2000fffc, 1),
        (0x20060000, 1),
        (0x20010001, 1),
        (0x20010000, 0x14001),
        (0x20010000, usize::MAX),
        (usize::MAX - 3, 2),
    ] {
        assert_eq!(Range::new(start, length), Err(BufferError::Range));
    }
    assert_eq!(Range::new(0x20010000, 0x14000), Ok(Range { start: 0x20010000, end: 0x2005ffff, words: 0x14000 }));
    assert_eq!(Range::new(0x2005fffc, 1).unwrap().end, 0x2005ffff);
}
#[test]
fn all_source_channels_map_to_owned_canonical_pads() {
    let pads =
        [Pad::Agpio1, Pad::Agpio3, Pad::Agpio4, Pad::Agpio6, Pad::Mgpio22, Pad::Mgpio8, Pad::Mgpio9, Pad::Agpio0];
    let mut inputs = None;
    for (i, pad) in pads.into_iter().enumerate() {
        let p = unsafe { PadControl::claim(pad).unwrap() };
        let input = unsafe { Input::from_configured(p).ok().unwrap() };
        assert_eq!(input.channel() as usize, i);
        if let Some(set) = inputs.as_mut() {
            let set: &mut Inputs<'_> = set;
            assert!(set.add(input).is_ok());
        } else {
            inputs = Some(Inputs::new(input));
        }
    }
    let set = inputs.unwrap();
    assert_eq!(set.mask, 255);
    for input in set.into_inputs() {
        let i = input.unwrap();
        assert_eq!(pad_channel(i.pad.pad()), Some(i.channel()));
    }
    let bad = unsafe { PadControl::claim(Pad::Mgpio0).unwrap() };
    assert!(unsafe { Input::from_configured(bad) }.is_err());
}
#[test]
fn uncalibrated_and_occupied_never_configure_or_stop_another_owner() {
    let (mut e, d) = engine();
    assert_eq!(e.start(false), Err(Error::NotCalibrated));
    e.io.idle = false;
    assert_eq!(e.start(true), Err(Error::Occupied));
    assert_eq!(e.start(true), Err(Error::NotReady));
    drop(e);
    assert!(d.0.borrow().is_empty());
}
#[test]
fn configuration_and_stale_fail_before_arm_but_request_cleanup() {
    for stale in [false, true] {
        let (mut e, d) = engine();
        e.io.stale = stale;
        e.io.valid = stale;
        assert_eq!(e.start(true), Err(if stale { Error::StaleCompletion } else { Error::Configuration }));
        assert!(!d.0.borrow().contains(&Event::Start));
        assert_eq!(e.start(true), Err(Error::NotReady));
        drop(e);
        assert_eq!(d.0.borrow().iter().filter(|v| **v == Event::Stop).count(), 1);
    }
}
#[test]
fn completion_stops_but_never_makes_driver_ready_or_releases_memory() {
    let (mut e, mut d) = engine();
    e.start(true).unwrap();
    assert_eq!(e.wait(polls(2), &mut d), Ok(Completion { last_address: 0x20050004 }));
    assert_eq!(e.wait(polls(2), &mut d), Err(Error::NotReady));
    assert_eq!(e.start(true), Err(Error::NotReady));
    assert_eq!(
        *d.0.borrow(),
        [Event::Configure, Event::Status, Event::Start, Event::Status, Event::Delay, Event::Status, Event::Stop]
    );
    e.stop();
    drop(e);
    assert_eq!(d.0.borrow().iter().filter(|v| **v == Event::Stop).count(), 1);
}
#[test]
fn timeout_has_exact_budget_and_no_implicit_retry() {
    let (mut e, mut d) = engine();
    e.io.ready = usize::MAX;
    e.start(true).unwrap();
    assert_eq!(e.wait(polls(3), &mut d), Err(Error::Timeout));
    assert_eq!(e.io.polls, 3);
    assert_eq!(d.0.borrow().iter().filter(|v| **v == Event::Delay).count(), 2);
    assert_eq!(e.start(true), Err(Error::NotReady));
}
#[test]
fn changed_configuration_and_wrong_completion_address_are_failures() {
    let (mut e, mut d) = engine();
    e.start(true).unwrap();
    e.io.valid = false;
    assert_eq!(e.wait(polls(2), &mut d), Err(Error::Configuration));
    let (mut e, mut d) = engine();
    e.io.ready = 1;
    e.io.address = 0x20050000;
    e.start(true).unwrap();
    assert_eq!(e.wait(polls(2), &mut d), Err(Error::CompletionAddress));
    assert_eq!(e.start(true), Err(Error::NotReady));
}
#[test]
fn dropping_running_driver_requests_stop_once_without_waiting() {
    let (mut e, d) = engine();
    e.start(true).unwrap();
    drop(e);
    assert_eq!(d.0.borrow().last(), Some(&Event::Stop));
    assert!(!d.0.borrow().contains(&Event::Delay));
}
#[test]
fn cancellation_before_start_and_forget_do_not_arm_or_free() {
    let (mut e, d) = engine();
    e.stop();
    assert_eq!(e.start(true), Err(Error::NotReady));
    drop(e);
    assert!(d.0.borrow().is_empty());
    let (mut e, d) = engine();
    e.start(true).unwrap();
    core::mem::forget(e);
    // Forget cannot be relied on for stopping a peripheral; the public API's
    // static memory owner is why this cannot invalidate a stack allocation.
    assert!(!d.0.borrow().contains(&Event::Stop));
}
#[test]
fn delay_panic_keeps_running_resources_until_drop_cleanup() {
    struct PanicDelay;
    impl DelayNs for PanicDelay {
        fn delay_ns(&mut self, _: u32) {
            panic!("independent delay failed");
        }
    }
    let (mut e, d) = engine();
    e.start(true).unwrap();
    let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
        let _ = e.wait(polls(3), &mut PanicDelay);
    }));
    assert!(result.is_err());
    assert_eq!(e.start(true), Err(Error::NotReady));
    drop(e);
    assert_eq!(d.0.borrow().last(), Some(&Event::Stop));
}
