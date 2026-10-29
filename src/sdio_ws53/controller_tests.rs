use super::*;
use std::{collections::VecDeque, vec::Vec};

#[derive(Default)]
struct Model {
    gate_failure: bool,
    clocks: bool,
    clock: bool,
    ready: bool,
    ready_failure: bool,
    events: VecDeque<EventSnapshot>,
    acks: Vec<EventSnapshot>,
    trace: Vec<&'static str>,
}
impl Registers for Model {
    fn enable_clocks(&mut self) {
        self.trace.push("enable-clocks");
        self.clocks = !self.gate_failure;
    }
    fn clocks_enabled(&mut self) -> bool {
        self.trace.push("read-clock-gates");
        self.clocks
    }
    fn mask_global(&mut self) {
        self.trace.push("mask-global");
    }
    fn ready(&mut self) -> bool {
        self.trace.push("read-ready");
        self.ready
    }
    fn host_clock(&mut self) -> bool {
        self.trace.push("read-host-clock");
        self.clock
    }
    fn snapshot(&mut self) -> EventSnapshot {
        self.trace.push("snapshot");
        self.events.pop_front().unwrap_or_default()
    }
    fn configure(&mut self) {
        self.trace.push("configure");
    }
    fn acknowledge(&mut self, events: EventSnapshot) {
        self.trace.push("acknowledge");
        if events.pending() {
            self.acks.push(events);
        }
    }
    fn set_ready(&mut self) {
        self.trace.push("set-ready");
        if !self.ready_failure {
            self.ready = true;
        }
    }
}
fn initialized() -> Engine<Model> {
    let mut engine = Engine::new(Model::default());
    engine.begin().unwrap();
    engine
}
fn configured() -> Engine<Model> {
    let mut engine = initialized();
    engine.io.clock = true;
    assert_eq!(engine.poll().unwrap().progress, InitProgress::WaitingFunctionEnable);
    engine
}
fn enabled_event() -> EventSnapshot {
    EventSnapshot { global: 4, function1: Function1Events(1 << 13), ..EventSnapshot::default() }
}
fn poisoned_without_more_io(e: &mut Engine<Model>) {
    let before = e.io.trace.clone();
    for _ in 0..3 {
        assert_eq!(e.poll(), Err(InitError::Poisoned));
    }
    assert_eq!(e.io.trace, before);
}
#[derive(Default)]
struct Delay(Vec<u32>);
impl DelayNs for Delay {
    fn delay_ns(&mut self, ns: u32) {
        self.0.push(ns);
    }
}

#[test]
fn init_requires_clock_then_a_fresh_enable_event() {
    let mut e = initialized();
    for _ in 0..5 {
        assert_eq!(e.poll().unwrap().progress, InitProgress::WaitingHostClock);
    }
    assert!(!e.io.trace.contains(&"configure"));
    e.io.clock = true;
    assert_eq!(e.poll().unwrap().progress, InitProgress::WaitingFunctionEnable);
    // READY alone cannot replace a newly captured FunctionEnable event.
    e.io.ready = true;
    assert_eq!(e.poll().unwrap().progress, InitProgress::WaitingFunctionEnable);
    let event = EventSnapshot { host_message: Some(0x12345678), function1: Function1Events(0x2200), ..enabled_event() };
    e.io.events.push_back(event);
    assert_eq!(e.poll(), Ok(PollResult { progress: InitProgress::FunctionReady, events: event }));
    assert_eq!(e.io.acks, [event]);
    assert_eq!(e.io.trace.iter().filter(|s| **s == "configure").count(), 1);
    assert!(e.io.trace.ends_with(&["acknowledge", "set-ready", "read-ready"]));
}

#[test]
fn initial_pending_or_ready_requires_external_recovery() {
    for ready in [false, true] {
        for status in [0, 0x2000, 0x80000000] {
            if !ready && status == 0 {
                continue;
            }
            let event = EventSnapshot { function1: Function1Events(status), ..Default::default() };
            let mut e = Engine::new(Model { ready, events: [event].into(), ..Default::default() });
            assert_eq!(e.begin(), Err(InitError::DirtyStart { ready, events: event }));
            assert!(e.io.acks.is_empty());
            assert!(!e.io.trace.contains(&"configure"));
            poisoned_without_more_io(&mut e);
        }
    }
}

#[test]
fn gate_failure_never_touches_the_sdio_window() {
    let mut e = Engine::new(Model { gate_failure: true, ..Default::default() });
    assert_eq!(e.begin(), Err(InitError::ClockGateReadback));
    assert_eq!(e.io.trace, ["enable-clocks", "read-clock-gates"]);
    poisoned_without_more_io(&mut e);
    let mut e = configured();
    e.io.clocks = false;
    let before = e.io.trace.len();
    assert_eq!(e.poll(), Err(InitError::ClockGateReadback));
    assert_eq!(&e.io.trace[before..], ["read-clock-gates"]);
    poisoned_without_more_io(&mut e);
}

#[test]
fn unsupported_events_are_retained_without_ack_or_ready_write() {
    for domain in 0..3 {
        for bit in 0..32 {
            let supported = match domain {
                0 => (1 << bit) & 5 != 0,
                1 => (1 << bit) & 0xffc != 0,
                _ => (1 << bit) & 0x3e00 != 0,
            };
            if supported {
                continue;
            }
            let mut event = enabled_event(); // Mixed enable + error must fail atomically.
            match domain {
                0 => event.global |= 1 << bit,
                1 => event.function0 |= 1 << bit,
                _ => event.function1.0 |= 1 << bit,
            }
            let mut e = configured();
            e.io.events.push_back(event);
            assert_eq!(e.poll(), Err(InitError::UnsupportedEvents(event)));
            assert!(e.io.acks.is_empty());
            assert!(!e.io.trace.contains(&"set-ready"));
            poisoned_without_more_io(&mut e);
        }
    }
}

#[test]
fn status_before_our_profile_cannot_count_as_fresh_enable() {
    let mut e = initialized();
    e.io.clock = true;
    let event = enabled_event();
    e.io.events.push_back(event);
    assert_eq!(e.poll(), Err(InitError::DirtyStart { ready: false, events: event }));
    assert!(!e.io.trace.contains(&"configure") && e.io.acks.is_empty());
    poisoned_without_more_io(&mut e);
}

#[test]
fn host_clock_loss_and_ready_readback_failure_retain_owner() {
    let mut e = configured();
    e.io.clock = false;
    assert_eq!(e.poll(), Err(InitError::HostClockLost));
    assert!(e.io.acks.is_empty());
    poisoned_without_more_io(&mut e);
    let mut e = configured();
    e.io.ready_failure = true;
    e.io.events.push_back(enabled_event());
    assert_eq!(e.poll(), Err(InitError::ReadyReadback(enabled_event())));
    assert_eq!(e.io.acks, [enabled_event()]);
    poisoned_without_more_io(&mut e);
}

#[test]
fn total_budget_is_finite_even_with_repeated_notifications() {
    for waiting_host in [false, true] {
        let mut e = if waiting_host { initialized() } else { configured() };
        let notification = EventSnapshot { function0: 4, global: 1, ..Default::default() };
        if !waiting_host {
            e.io.events = [notification; 7].into();
        }
        let mut delay = Delay::default();
        let mut observed = Vec::new();
        let progress = e.progress;
        assert_eq!(
            e.wait(NonZeroU16::new(7).unwrap(), &mut delay, |event| observed.push(event)),
            Err(InitError::Timeout(progress))
        );
        assert_eq!(delay.0, [1000; 6]);
        assert_eq!(observed.len(), if waiting_host { 0 } else { 7 });
        assert_eq!(e.io.events.len(), 0);
        poisoned_without_more_io(&mut e);
    }
}

#[test]
fn last_attempt_enable_succeeds_without_trailing_delay() {
    let mut e = configured();
    e.io.events = [EventSnapshot::default(), enabled_event()].into();
    let mut delay = Delay::default();
    let mut observed = Vec::new();
    e.wait(NonZeroU16::new(2).unwrap(), &mut delay, |event| observed.push(event)).unwrap();
    assert_eq!(observed, [enabled_event()]);
    assert_eq!(delay.0, [1000]);
}

#[test]
fn cancellation_performs_no_reset_and_followups_are_inert() {
    let mut e = configured();
    let before = e.io.trace.len();
    assert_eq!(e.fail(InitError::Cancelled), InitError::Cancelled);
    assert_eq!(&e.io.trace[before..], ["mask-global"]);
    poisoned_without_more_io(&mut e);
}

#[test]
fn production_pac_profile_matches_all_original_rom_observations() {
    // Generated from unmodified ROM execution, not from the Rust profile.
    // See rom_init_observations.txt header for source and observation identity.
    let fixture = include_str!("rom_init_observations.txt");
    let mut rows = 0;
    for row in fixture.lines().filter(|line| !line.starts_with('#') && !line.is_empty()) {
        let values: Vec<u32> = row.split_whitespace().map(|v| u32::from_str_radix(v, 16).unwrap()).collect();
        assert_eq!(values.len(), 193);
        let seed = values[0];
        let mut memory = [seed; 192];
        let r = unsafe { &*memory.as_mut_ptr().cast::<pac::sdio::RegisterBlock>() };
        configure_profile(r);
        let mut expected = values[1..].to_vec();
        // Deliberate polling policy: don't unmask global IRQs. ROM full init
        // sets CONFIG_30 after the audited common initializer returns.
        expected[0x18 / 4] = seed;
        expected[0x30 / 4] = seed | 1;
        assert_eq!(memory.as_slice(), expected, "seed {seed:#x}");
        rows += 1;
    }
    assert_eq!(rows, 36);
}
