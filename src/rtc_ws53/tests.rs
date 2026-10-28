use super::*;
use std::{cell::RefCell, rc::Rc, vec, vec::Vec};

#[derive(Debug, PartialEq, Eq)]
enum Op {
    Mask(bool),
    DisableLocal,
    Enable(bool),
    Delay(u32),
    Ack,
    ClearLocal,
    Configure(RtcMode, u32),
    Request,
    Poll,
    Read,
}
type Trace = Rc<RefCell<Vec<Op>>>;
struct Mock {
    trace: Trace,
    enabled: bool,
    ready_at: u32,
    polls: u32,
    value: u32,
    pending: bool,
    one_shot: bool,
}
impl Registers for Mock {
    fn enable(&mut self, enabled: bool) {
        self.enabled = enabled;
        self.trace.borrow_mut().push(Op::Enable(enabled));
    }
    fn enabled(&mut self) -> bool {
        self.enabled
    }
    fn mask(&mut self, masked: bool) {
        self.trace.borrow_mut().push(Op::Mask(masked));
    }
    fn configure(&mut self, mode: RtcMode, ticks: RtcTicks) {
        self.trace.borrow_mut().push(Op::Configure(mode, ticks.count()));
    }
    fn request(&mut self) {
        self.trace.borrow_mut().push(Op::Request);
    }
    fn valid(&mut self) -> bool {
        self.trace.borrow_mut().push(Op::Poll);
        self.polls += 1;
        self.polls >= self.ready_at
    }
    fn count(&mut self) -> u32 {
        self.trace.borrow_mut().push(Op::Read);
        self.value
    }
    fn acknowledge(&mut self) {
        self.trace.borrow_mut().push(Op::Ack);
    }
    fn disable_local(&mut self) {
        self.trace.borrow_mut().push(Op::DisableLocal);
    }
    fn clear_local(&mut self) {
        self.trace.borrow_mut().push(Op::ClearLocal);
    }
    fn pending(&self) -> bool {
        self.pending
    }
    fn one_shot(&self) -> bool {
        self.one_shot
    }
}
struct Delay(Trace);
impl DelayNs for Delay {
    fn delay_ns(&mut self, ns: u32) {
        self.0.borrow_mut().push(Op::Delay(ns));
    }
}
fn mock(enabled: bool, ready_at: u32, value: u32) -> Mock {
    Mock { trace: Trace::default(), enabled, ready_at, polls: 0, value, pending: false, one_shot: true }
}
#[test]
fn start_quiesces_before_load_and_waits_for_both_synchronizations() {
    for mode in [RtcMode::OneShot, RtcMode::Periodic, RtcMode::FreeRunning] {
        let mut io = mock(true, 1, 0);
        let mut delay = Delay(io.trace.clone());
        start(&mut io, &mut delay, mode, RtcTicks::from_count(u32::MAX).unwrap());
        assert_eq!(
            *io.trace.borrow(),
            vec![
                Op::Mask(true),
                Op::DisableLocal,
                Op::Enable(false),
                Op::Delay(32_000),
                Op::Ack,
                Op::ClearLocal,
                Op::Configure(mode, u32::MAX),
                Op::Enable(true),
                Op::Delay(65_000)
            ]
        );
        assert!(io.enabled);
    }
}
#[test]
fn stop_does_not_reload_or_restart_and_clears_pending_after_ack() {
    let mut io = mock(true, 1, 0);
    let mut delay = Delay(io.trace.clone());
    stop(&mut io, &mut delay);
    assert_eq!(
        *io.trace.borrow(),
        vec![Op::Mask(true), Op::DisableLocal, Op::Enable(false), Op::Delay(32_000), Op::Ack, Op::ClearLocal]
    );
    assert!(!io.enabled);
}
#[test]
fn disabled_snapshot_does_not_touch_request_or_data() {
    let mut io = mock(false, 1, 123);
    assert_eq!(snapshot(&mut io, NonZeroU16::new(1).unwrap()), (Err(RtcError::Disabled), false));
    assert!(io.trace.borrow().is_empty());
}
#[test]
fn timeout_never_consumes_stale_counter() {
    let mut io = mock(true, 4, 123);
    assert_eq!(snapshot(&mut io, NonZeroU16::new(3).unwrap()), (Err(RtcError::SnapshotTimeout), false));
    assert_eq!(*io.trace.borrow(), vec![Op::Request, Op::Poll, Op::Poll, Op::Poll]);
}
#[test]
fn valid_zero_and_last_budget_poll_remain_successful() {
    for count in [0, 1, u32::MAX] {
        let mut io = mock(true, 3, count);
        assert_eq!(snapshot(&mut io, NonZeroU16::new(3).unwrap()), (Ok(count), false));
        assert_eq!(*io.trace.borrow(), vec![Op::Request, Op::Poll, Op::Poll, Op::Poll, Op::Read]);
    }
}
#[test]
fn one_shot_expiry_undoes_rearm_without_consuming_notification() {
    for ready_at in [1, 10] {
        let mut io = mock(true, ready_at, 123);
        io.pending = true;
        assert_eq!(snapshot(&mut io, NonZeroU16::new(3).unwrap()), (Err(RtcError::Expired), true));
        assert!(!io.enabled && io.pending);
        assert_eq!(io.trace.borrow().last(), Some(&Op::Enable(false)));
        assert!(!io.trace.borrow().contains(&Op::Ack));
        assert!(!io.trace.borrow().contains(&Op::ClearLocal));
    }
}
#[test]
fn periodic_pending_event_does_not_stop_or_invalidate_sample() {
    let mut io = mock(true, 1, 123);
    io.one_shot = false;
    io.pending = true;
    assert_eq!(snapshot(&mut io, NonZeroU16::new(3).unwrap()), (Ok(123), false));
    assert!(io.enabled && io.pending);
    assert_eq!(*io.trace.borrow(), vec![Op::Request, Op::Poll, Op::Read]);
}
#[test]
fn duration_rejects_zero_overflow_and_rounds_up_subtick_intervals() {
    assert_eq!(RtcTicks::from_count(0), None);
    assert_eq!(RtcTicks::from_duration(Duration::ZERO), None);
    assert_eq!(RtcTicks::from_duration(Duration::from_nanos(1)).unwrap().count(), 1);
    assert_eq!(RtcTicks::from_duration(Duration::from_secs(1)).unwrap().count(), 32768);
    assert_eq!(RtcTicks::from_duration(Duration::from_secs(131072)), None); // 2^32 ticks
    assert_eq!(RtcTicks::from_duration(Duration::MAX), None);
    assert_eq!(RtcTicks::from_count(u32::MAX).unwrap().count(), u32::MAX);
}
proptest::proptest! {
    #[test]
    fn accepted_duration_is_never_rounded_early(ns in proptest::prop_oneof![0u64..=131_072_000_000_000, proptest::num::u64::ANY]) {
        let scaled = u128::from(ns) * 32768;
        match RtcTicks::from_duration(Duration::from_nanos(ns)) {
            Some(ticks) => {
                let count = u128::from(ticks.count());
                proptest::prop_assert!(count * 1_000_000_000 >= scaled);
                proptest::prop_assert!((count - 1) * 1_000_000_000 < scaled);
            }
            None => proptest::prop_assert!(ns == 0 || scaled > u128::from(u32::MAX) * 1_000_000_000),
        }
    }
}
#[test]
fn constructors_select_owned_channel_without_mmio_or_delays() {
    // Tokens are synthetic host-only values. ManuallyDrop prevents real MMIO
    // cleanup; this checks constructor purity, not physical resource handoff.
    let trace = Trace::default();
    let a = core::mem::ManuallyDrop::new(RtcDriver::new(
        unsafe { Rtc0::steal() },
        Delay(trace.clone()),
        NonZeroU16::new(1).unwrap(),
    ));
    let b = core::mem::ManuallyDrop::new(RtcDriver::new(
        unsafe { Rtc1::steal() },
        Delay(trace.clone()),
        NonZeroU16::new(1).unwrap(),
    ));
    assert_eq!(a.channel().interrupt(), Interrupt::RTC_0);
    assert_eq!(b.channel().interrupt(), Interrupt::RTC_1);
    assert!(trace.borrow().is_empty());
}
