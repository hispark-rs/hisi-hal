use super::*;
use std::{cell::RefCell, rc::Rc, vec, vec::Vec};

#[derive(Debug, PartialEq, Eq)]
enum Op {
    Clock(bool),
    Divider(ClockDivider),
    DisableIrqs,
    Events(EventEnable),
    DmaOff,
    Select(Group, u8),
    Enable(Channel, bool),
    Configure(Channel, Waveform),
    Commit(Group, bool),
    Delay(u32),
    Ack(Events),
}
type Trace = Rc<RefCell<Vec<Op>>>;
struct Mock {
    trace: Trace,
    pending: Events,
    expire_on_commit: u8,
    expire_on_first_commit: u8,
}
impl Registers for Mock {
    fn clock_divider(&mut self, divider: ClockDivider) {
        self.trace.borrow_mut().push(Op::Divider(divider));
    }
    fn clock(&mut self, on: bool) {
        self.trace.borrow_mut().push(Op::Clock(on));
    }
    fn disable_irqs(&mut self) {
        self.trace.borrow_mut().push(Op::DisableIrqs);
    }
    fn event_enable(&mut self, e: EventEnable) {
        self.trace.borrow_mut().push(Op::Events(e));
    }
    fn dma_off(&mut self) {
        self.trace.borrow_mut().push(Op::DmaOff);
    }
    fn select(&mut self, g: Group, m: u8) {
        self.trace.borrow_mut().push(Op::Select(g, m));
    }
    fn enable(&mut self, c: Channel, on: bool) {
        self.trace.borrow_mut().push(Op::Enable(c, on));
    }
    fn configure(&mut self, c: Channel, w: Waveform) {
        self.trace.borrow_mut().push(Op::Configure(c, w));
    }
    fn commit(&mut self, g: Group, clear: bool) {
        self.trace.borrow_mut().push(Op::Commit(g, clear));
        if clear {
            self.pending.configuration_loaded |= self.expire_on_first_commit;
        } else {
            self.pending.configuration_loaded |= self.expire_on_commit;
        }
    }
    fn errors(&self) -> (u16, u16) {
        (self.pending.group_errors, self.pending.count_errors)
    }
    fn period_flag(&self, c: Channel) -> bool {
        self.pending.configuration_loaded & (1 << c.index()) != 0
    }
    fn ack(&mut self, e: Events) {
        self.trace.borrow_mut().push(Op::Ack(e));
        self.pending.group_errors &= !e.group_errors;
        self.pending.count_errors &= !e.count_errors;
        self.pending.configuration_loaded &= !e.configuration_loaded;
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
    Engine::new(
        Mock { trace: trace.clone(), pending: Events::default(), expire_on_commit: 0, expire_on_first_commit: 0 },
        Delay(trace),
    )
}

#[test]
fn late_finite_staging_stops_whole_group_without_disturbing_other_groups() {
    let mut e = engine();
    let mixed = GroupConfig::new(&[
        (Channel::C0, wave(PulseMode::Finite(block()))),
        (Channel::C1, wave(PulseMode::Repeating(block()))),
    ])
    .unwrap();
    e.configure(Group::G0, mixed, 255).unwrap();
    e.configure(Group::G1, config(Channel::C7, PulseMode::Repeating(block())), 255).unwrap();
    e.start(Group::G1).unwrap();
    e.io.expire_on_commit = 1;
    e.io.pending.configuration_loaded = 128;
    assert_eq!(e.start(Group::G0), Err(Error::FiniteDeadlineMissed));
    assert!(!e.running[0] && e.running[1]);
    assert_eq!(e.io.pending.configuration_loaded, 128);
    assert!(e.io.trace.borrow().ends_with(&[
        Op::Enable(Channel::C0, false),
        Op::Enable(Channel::C1, false),
        Op::Commit(Group::G0, true),
        Op::Delay(1000),
        Op::Ack(Events { group_errors: 3, count_errors: 3, configuration_loaded: 3 }),
    ]));
    // A repeating neighbor's event must not cause a false finite deadline error.
    e.io.expire_on_commit = 2;
    e.start(Group::G0).unwrap();
    assert!(e.running[0] && e.running[1]);
}
fn wave(mode: PulseMode) -> Waveform {
    Waveform::new(150, 100, 0, mode).unwrap()
}
fn block() -> PulseCount {
    PulseCount::new(7).unwrap()
}
fn config(ch: Channel, mode: PulseMode) -> GroupConfig {
    GroupConfig::new(&[(ch, wave(mode))]).unwrap()
}

#[test]
fn configured_indices_and_count_limits_reject_sdk_capacity_and_overflow() {
    for i in 0..=u8::MAX {
        assert_eq!(Channel::from_index(i).is_some(), i < 8);
        assert_eq!(Group::from_index(i).is_some(), i < 4);
    }
    for count in [0, 32768, u32::MAX] {
        assert_eq!(PulseCount::new(count), None);
    }
    assert_eq!(PulseCount::new(32767).unwrap().count(), 32767);
    for (low, high, offset) in [(0, 0, 0), (200, 56, 0), (150, 100, 151), (u32::MAX, 1, 0), (1, u32::MAX, 0)] {
        assert!(Waveform::new(low, high, offset, PulseMode::Continuous).is_none());
    }
    assert_eq!(Waveform::new(200, 55, 200, PulseMode::Continuous).unwrap().period(), 255);
}
proptest::proptest! {
    #[test]
    fn accepted_waveforms_preserve_wide_arithmetic_bounds(low in proptest::prop_oneof![0u32..=256,proptest::num::u32::ANY],
        high in proptest::prop_oneof![0u32..=256,proptest::num::u32::ANY],
        offset in proptest::prop_oneof![0u32..=256,proptest::num::u32::ANY]) {
        let total=u64::from(low)+u64::from(high);
        let parsed=Waveform::new(low,high,offset,PulseMode::Continuous);
        proptest::prop_assert_eq!(parsed.is_some(),total>0 && total<=255 && offset<=low);
        if let Some(w)=parsed {
            proptest::prop_assert_eq!(u64::from(w.period()),total);
            proptest::prop_assert_eq!(u32::from(w.high()),high);
            proptest::prop_assert_eq!(u32::from(w.offset()),offset);
        }
    }
}
#[test]
fn group_rejects_empty_and_duplicate_channels() {
    assert!(matches!(GroupConfig::new(&[]), Err(Error::EmptyGroup)));
    let w = wave(PulseMode::Continuous);
    assert!(matches!(GroupConfig::new(&[(Channel::C0, w), (Channel::C0, w)]), Err(Error::DuplicateChannel)));
    let all: Vec<_> = channels(255).map(|c| (c, w)).collect();
    assert_eq!(GroupConfig::new(&all).unwrap().mask(), 255);
}
#[test]
fn invalid_handoff_overlap_and_running_reconfiguration_do_not_write() {
    let mut e = engine();
    let c = config(Channel::C0, PulseMode::Continuous);
    assert_eq!(e.configure(Group::G0, c, 0), Err(Error::MissingRoute));
    assert!(e.io.trace.borrow().is_empty());
    e.configure(Group::G0, c, 1).unwrap();
    e.io.trace.borrow_mut().clear();
    assert_eq!(e.configure(Group::G1, c, 255), Err(Error::ChannelInUse));
    assert!(e.io.trace.borrow().is_empty());
    e.start(Group::G0).unwrap();
    e.io.trace.borrow_mut().clear();
    assert_eq!(e.configure(Group::G0, c, 255), Err(Error::GroupRunning));
    assert_eq!(e.start(Group::G0), Err(Error::GroupRunning));
    assert!(e.io.trace.borrow().is_empty());
}
#[test]
fn finite_stop_staging_is_per_channel_even_with_repeating_neighbor() {
    for group in [Group::G0, Group::G1, Group::G2, Group::G3] {
        let mut e = engine();
        let c = GroupConfig::new(&[
            (Channel::C0, wave(PulseMode::Finite(block()))),
            (Channel::C1, wave(PulseMode::Repeating(block()))),
            (Channel::C7, wave(PulseMode::Continuous)),
        ])
        .unwrap();
        e.configure(group, c, 255).unwrap();
        e.io.trace.borrow_mut().clear();
        e.start(group).unwrap();
        assert_eq!(
            *e.io.trace.borrow(),
            vec![
                Op::Ack(Events { group_errors: 131, count_errors: 131, configuration_loaded: 131 }),
                Op::Enable(Channel::C0, true),
                Op::Enable(Channel::C1, true),
                Op::Enable(Channel::C7, true),
                Op::Commit(group, true),
                Op::Delay(10000),
                Op::Ack(Events { configuration_loaded: 1, ..Events::default() }),
                Op::Enable(Channel::C0, false),
                Op::Delay(1000),
                Op::Commit(group, false)
            ]
        );
    }
}
#[test]
fn configuring_another_group_does_not_change_repeat_policy() {
    let mut e = engine();
    e.configure(Group::G0, config(Channel::C0, PulseMode::Finite(block())), 255).unwrap();
    e.configure(Group::G1, config(Channel::C1, PulseMode::Repeating(block())), 255).unwrap();
    e.io.trace.borrow_mut().clear();
    e.start(Group::G0).unwrap();
    assert!(e.io.trace.borrow().contains(&Op::Enable(Channel::C0, false)));
    assert!(e.io.trace.borrow().contains(&Op::Delay(1000)));
    e.io.trace.borrow_mut().clear();
    e.start(Group::G1).unwrap();
    assert!(!e.io.trace.borrow().iter().any(|o| matches!(o, Op::Delay(_) | Op::Enable(_, false))));
}
#[test]
fn stop_preserves_other_group_and_shared_irq_enable() {
    let mut e = engine();
    e.configure(Group::G0, config(Channel::C0, PulseMode::Continuous), 255).unwrap();
    e.configure(Group::G1, config(Channel::C7, PulseMode::Continuous), 255).unwrap();
    e.start(Group::G0).unwrap();
    e.start(Group::G1).unwrap();
    e.io.trace.borrow_mut().clear();
    e.stop(Group::G0);
    assert_eq!(
        *e.io.trace.borrow(),
        vec![
            Op::Enable(Channel::C0, false),
            Op::Commit(Group::G0, true),
            Op::Delay(1000),
            Op::Ack(Events { group_errors: 1, count_errors: 1, configuration_loaded: 1 })
        ]
    );
    assert!(!e.running[0] && e.running[1]);
    assert_eq!(e.group_for(Channel::C7), Some(Group::G1));
}
#[test]
fn duty_rejection_has_no_partial_writes_and_keeps_phase_constraint() {
    let mut e = engine();
    let w = Waveform::new(100, 100, 50, PulseMode::Continuous).unwrap();
    e.configure(Group::G2, GroupConfig::new(&[(Channel::C4, w)]).unwrap(), 255).unwrap();
    e.io.trace.borrow_mut().clear();
    for high in [151, 201, u16::MAX] {
        assert_eq!(e.set_duty(Group::G2, Channel::C4, high), Err(Error::InvalidDuty));
    }
    assert!(e.io.trace.borrow().is_empty());
    e.set_duty(Group::G2, Channel::C4, 150).unwrap();
    let updated = e.groups[2].unwrap().channels[4].unwrap();
    assert_eq!(updated.high(), 150);
    assert_eq!(updated.offset(), 50);
    assert_eq!(*e.io.trace.borrow(), vec![Op::Configure(Channel::C4, updated), Op::Commit(Group::G2, true)]);
    e.io.trace.borrow_mut().clear();
    e.set_duty(Group::G2, Channel::C4, 0).unwrap();
    assert_eq!(e.groups[2].unwrap().channels[4].unwrap().high(), 0);
}
#[test]
fn shutdown_masks_irqs_before_stop_and_gates_clock_last() {
    let mut e = engine();
    e.initialize();
    assert_eq!(e.io.trace.borrow().first(), Some(&Op::Clock(true)));
    e.configure(Group::G3, config(Channel::C7, PulseMode::Finite(block())), 255).unwrap();
    e.start(Group::G3).unwrap();
    e.io.trace.borrow_mut().clear();
    e.shutdown();
    let t = e.io.trace.borrow();
    assert_eq!(&t[..3], &[Op::Events(EventEnable::Disabled), Op::DisableIrqs, Op::DmaOff]);
    assert_eq!(t.last(), Some(&Op::Clock(false)));
    assert!(t.contains(&Op::Commit(Group::G3, true)) && t.contains(&Op::Delay(1000)));
    assert_eq!(e.running, [false; 4]);
}
#[test]
fn status_retains_all_error_bits_and_reads_all_eight_end_flags_without_ack() {
    let mut e = engine();
    e.io.pending = Events { group_errors: 0x8012, count_errors: 0x4071, configuration_loaded: 0xa5 };
    assert_eq!(events(&e.io), e.io.pending);
    assert!(e.io.trace.borrow().is_empty());
}

#[test]
fn divider_rejects_unmeasured_encodings_without_clamping() {
    for value in 0..=255 {
        let parsed = ClockDivider::from_divisor(value);
        assert_eq!(parsed.is_some(), [1, 2, 4, 8, 16].contains(&value));
        if let Some(divider) = parsed {
            assert_eq!(divider.divisor(), value);
        }
    }
}

#[test]
fn divider_requires_all_groups_stopped_even_with_a_completion_flag() {
    for group in [Group::G0, Group::G1, Group::G2, Group::G3] {
        let mut e = engine();
        e.configure(group, config(Channel::C0, PulseMode::Continuous), 255).unwrap();
        e.start(group).unwrap();
        // A latched event cannot prove quiescence or permit clock changes.
        e.io.pending.configuration_loaded = 1;
        e.io.trace.borrow_mut().clear();
        for value in [1, 2, 4, 8, 16] {
            assert_eq!(e.set_clock_divider(ClockDivider::from_divisor(value).unwrap()), Err(Error::GroupRunning));
        }
        assert!(e.io.trace.borrow().is_empty());
        e.stop(group);
        e.io.trace.borrow_mut().clear();
        for value in [1, 2, 4, 8, 16] {
            let divider = ClockDivider::from_divisor(value).unwrap();
            e.set_clock_divider(divider).unwrap();
            assert_eq!(e.io.trace.borrow_mut().pop(), Some(Op::Divider(divider)));
            assert!(e.io.trace.borrow().is_empty());
        }
    }
}

#[test]
fn initial_configuration_load_is_acknowledged_without_truncating_finite_output() {
    let mut e = engine();
    e.configure(Group::G0, config(Channel::C0, PulseMode::Finite(block())), 255).unwrap();
    e.io.trace.borrow_mut().clear();
    e.io.expire_on_first_commit = 1;
    assert_eq!(e.start(Group::G0), Ok(()));
    let trace = e.io.trace.borrow();
    assert!(trace.contains(&Op::Delay(10000)));
    assert!(trace.contains(&Op::Commit(Group::G0, false)));
    assert!(e.running[0]);
    assert_eq!(e.io.pending.configuration_loaded, 0);
}
