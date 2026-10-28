use super::*;
use std::{cell::RefCell, rc::Rc, vec::Vec};
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Event {
    Clocks,
    Reset(bool),
    AnalogFirst,
    AnalogRelease,
    Prepare(Stage, u8),
    Enable(Stage),
    Disable,
    Finish,
    Delay(u32),
    Status(Stage),
}
struct Mock {
    events: Rc<RefCell<Vec<Event>>>,
    stage: Stage,
    started: bool,
    poll: u16,
    ready_after: u16,
    timeout: Option<Stage>,
    stale: Option<Stage>,
    cap_error: bool,
    valid: bool,
    offset: u8,
}
impl Registers for Mock {
    fn clocks(&mut self) {
        self.events.borrow_mut().push(Event::Clocks);
    }
    fn reset(&mut self, b: bool) {
        self.events.borrow_mut().push(Event::Reset(b));
    }
    fn analog_first(&mut self) {
        self.events.borrow_mut().push(Event::AnalogFirst);
    }
    fn analog_release(&mut self) {
        self.events.borrow_mut().push(Event::AnalogRelease);
    }
    fn configuration_valid(&mut self) -> bool {
        self.valid
    }
    fn prepare(&mut self, s: Stage, m: u8) {
        self.stage = s;
        self.poll = 0;
        self.started = false;
        self.events.borrow_mut().push(Event::Prepare(s, m));
    }
    fn enable(&mut self, s: Stage) {
        assert_eq!(self.stage, s);
        assert!(!self.started);
        self.started = true;
        self.events.borrow_mut().push(Event::Enable(s));
    }
    fn status(&mut self, s: Stage) -> Status {
        assert_eq!(self.stage, s);
        self.events.borrow_mut().push(Event::Status(s));
        if !self.started {
            return Status { finished: self.stale == Some(s), error: false };
        }
        self.poll += 1;
        Status {
            finished: self.timeout != Some(s) && self.poll >= self.ready_after,
            error: s == Stage::Capacitor && self.cap_error,
        }
    }
    fn offset(&mut self) -> u8 {
        self.offset
    }
    fn disable(&mut self) {
        self.started = false;
        self.events.borrow_mut().push(Event::Disable);
    }
    fn finish(&mut self) {
        self.events.borrow_mut().push(Event::Finish);
    }
}
struct Delay(Rc<RefCell<Vec<Event>>>);
impl DelayNs for Delay {
    fn delay_ns(&mut self, ns: u32) {
        self.0.borrow_mut().push(Event::Delay(ns));
    }
}
fn engine() -> (Engine<Mock>, Delay) {
    let events = Rc::new(RefCell::new(Vec::new()));
    (
        Engine {
            io: Mock {
                events: events.clone(),
                stage: Stage::OffsetAuto,
                started: false,
                poll: 0,
                ready_after: 1,
                timeout: None,
                stale: None,
                cap_error: false,
                valid: true,
                offset: 10,
            },
            result: None,
        },
        Delay(events),
    )
}
fn budget(n: u16) -> NonZeroU16 {
    NonZeroU16::new(n).unwrap()
}
#[test]
fn all_capture_words_match_original_c_format_and_preserve_reserved_bits() {
    let channels = [
        Channel::Channel0,
        Channel::Channel1,
        Channel::Channel2,
        Channel::Channel3,
        Channel::Channel4,
        Channel::Channel5,
        Channel::Channel6,
        Channel::Channel7,
    ];
    for (n, ch) in channels.into_iter().enumerate() {
        for code in 0..4096 {
            let word = 0xfffe0003 | code << 2 | (n as u32) << 14;
            let sample = Sample::from_capture_word(word);
            assert_eq!(sample.raw(), word);
            assert_eq!(sample.code(), code as u16);
            assert_eq!(sample.channel(), ch);
        }
    }
}
#[test]
fn source_piecewise_correction_all_six_bit_inputs() {
    for n in 0..64 {
        let expected = if n <= 3 {
            0
        } else if n <= 31 {
            n - 3
        } else if n <= 59 {
            n + 3
        } else {
            63
        };
        assert_eq!(corrected_offset(n), expected);
    }
    assert_eq!((corrected_offset(31), corrected_offset(32)), (28, 35));
}
#[test]
fn source_sequence_and_explicit_manual_phase_clear() {
    let (mut e, mut d) = engine();
    assert_eq!(e.calibrate(budget(1), &mut d), Ok(Calibration { automatic_offset: 10, manual_offset: 7 }));
    assert_eq!(e.result, Some(Calibration { automatic_offset: 10, manual_offset: 7 }));
    assert_eq!(
        *d.0.borrow(),
        [
            Event::Clocks,
            Event::Reset(false),
            Event::Delay(1000),
            Event::Reset(true),
            Event::Delay(1000),
            Event::AnalogFirst,
            Event::Delay(5000),
            Event::AnalogRelease,
            Event::Prepare(Stage::OffsetAuto, 0),
            Event::Status(Stage::OffsetAuto),
            Event::Enable(Stage::OffsetAuto),
            Event::Delay(100000),
            Event::Status(Stage::OffsetAuto),
            Event::Disable,
            Event::Prepare(Stage::OffsetManual, 7),
            Event::Status(Stage::OffsetManual),
            Event::Enable(Stage::OffsetManual),
            Event::Delay(128000),
            Event::Status(Stage::OffsetManual),
            Event::Disable,
            Event::Prepare(Stage::Capacitor, 7),
            Event::Status(Stage::Capacitor),
            Event::Enable(Stage::Capacitor),
            Event::Delay(160000),
            Event::Status(Stage::Capacitor),
            Event::Disable,
            Event::Finish,
            Event::Disable
        ]
    );
}
#[test]
fn polling_budget_includes_final_attempt_and_no_extra_delay() {
    let (mut e, mut d) = engine();
    e.io.ready_after = 3;
    assert!(e.calibrate(budget(3), &mut d).is_ok());
    let events = d.0.borrow();
    for stage in [Stage::OffsetAuto, Stage::OffsetManual, Stage::Capacitor] {
        // One clear check plus exactly three completion polls.
        assert_eq!(events.iter().filter(|v| **v == Event::Status(stage)).count(), 4);
    }
    // Two reset delays plus two polling delays for each of three stages.
    assert_eq!(events.iter().filter(|v| **v == Event::Delay(1000)).count(), 8);
}
#[test]
fn every_phase_times_out_and_disables_without_later_phase_or_success() {
    for stage in [Stage::OffsetAuto, Stage::OffsetManual, Stage::Capacitor] {
        let (mut e, mut d) = engine();
        e.io.timeout = Some(stage);
        assert_eq!(e.calibrate(budget(3), &mut d), Err(Error::Timeout(stage)));
        assert_eq!(e.result, None);
        assert!(!e.io.started);
        let events = d.0.borrow();
        assert_eq!(events.iter().filter(|v| **v == Event::Status(stage)).count(), 4);
        assert_eq!(events.last(), Some(&Event::Disable));
        assert!(!events.contains(&Event::Finish));
        if stage == Stage::OffsetAuto {
            assert!(!events.contains(&Event::Prepare(Stage::OffsetManual, 7)));
        }
        if stage == Stage::OffsetManual {
            assert!(!events.contains(&Event::Prepare(Stage::Capacitor, 7)));
        }
    }
}
#[test]
fn stale_status_never_counts_as_fresh_completion_in_any_phase() {
    for stage in [Stage::OffsetAuto, Stage::OffsetManual, Stage::Capacitor] {
        let (mut e, mut d) = engine();
        e.io.stale = Some(stage);
        assert_eq!(e.calibrate(budget(3), &mut d), Err(Error::StaleStatus(stage)));
        assert!(!d.0.borrow().contains(&Event::Enable(stage)));
        assert_eq!(e.result, None);
        assert!(!e.io.started);
    }
}
#[test]
fn capacitor_error_wins_over_finished_and_prevents_success() {
    let (mut e, mut d) = engine();
    e.io.cap_error = true;
    assert_eq!(e.calibrate(budget(1), &mut d), Err(Error::Capacitor));
    assert_eq!(e.result, None);
    assert!(!e.io.started);
    assert!(!d.0.borrow().contains(&Event::Finish));
}
#[test]
fn capacitor_error_is_reported_even_when_not_finished() {
    let (mut e, mut d) = engine();
    e.io.cap_error = true;
    e.io.ready_after = 2;
    assert_eq!(e.calibrate(budget(3), &mut d), Err(Error::Capacitor));
    assert!(!e.io.started);
}
#[test]
fn invalid_configuration_fails_before_enabling_calibration() {
    let (mut e, mut d) = engine();
    e.io.valid = false;
    assert_eq!(e.calibrate(budget(3), &mut d), Err(Error::Configuration));
    assert!(!d.0.borrow().iter().any(|e| matches!(e, Event::Enable(_))));
    assert!(!e.io.started);
}
#[test]
fn failure_invalidates_old_report_and_explicit_retry_can_recover() {
    let (mut e, mut d) = engine();
    assert!(e.calibrate(budget(1), &mut d).is_ok());
    e.io.timeout = Some(Stage::OffsetManual);
    assert!(e.calibrate(budget(1), &mut d).is_err());
    assert_eq!(e.result, None);
    e.io.timeout = None;
    e.io.offset = 63;
    assert_eq!(e.calibrate(budget(1), &mut d), Ok(Calibration { automatic_offset: 63, manual_offset: 63 }));
}
