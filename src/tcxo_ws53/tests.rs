use super::*;

#[derive(Default)]
struct Mock {
    enabled: bool,
    requests: u32,
    polls: u32,
    ready_at: u32,
    data_reads: u32,
    words: [u16; 4],
}
impl Registers for Mock {
    fn enabled(&mut self) -> bool {
        self.enabled
    }
    fn request(&mut self) {
        self.requests += 1;
    }
    fn valid(&mut self) -> bool {
        self.polls += 1;
        self.polls >= self.ready_at
    }
    fn words_high_to_low(&mut self) -> [u16; 4] {
        self.data_reads += 1;
        self.words
    }
}
#[test]
fn disabled_does_not_request_or_consume_data() {
    let mut io = Mock::default();
    assert_eq!(snapshot(&mut io, NonZeroU16::new(1).unwrap(), &mut false), Err(TcxoError::Disabled));
    assert_eq!((io.requests, io.polls, io.data_reads), (0, 0, 0));
}
#[test]
fn timeout_does_not_read_stale_words_or_report_zero() {
    let mut io = Mock { enabled: true, ready_at: 4, ..Mock::default() };
    assert_eq!(snapshot(&mut io, NonZeroU16::new(3).unwrap(), &mut false), Err(TcxoError::SnapshotTimeout));
    assert_eq!((io.requests, io.polls, io.data_reads), (1, 3, 0));
}
#[test]
fn valid_zero_and_last_poll_are_successful_samples() {
    let mut io = Mock { enabled: true, ready_at: 3, ..Mock::default() };
    assert_eq!(snapshot(&mut io, NonZeroU16::new(3).unwrap(), &mut false).unwrap().ticks(), 0);
    assert_eq!((io.requests, io.polls, io.data_reads), (1, 3, 1));
}
#[test]
fn four_chunks_form_full_width_counter() {
    let mut io = Mock { enabled: true, ready_at: 1, words: [0xfedc, 0xba98, 0x7654, 0x3210], ..Mock::default() };
    assert_eq!(snapshot(&mut io, NonZeroU16::new(1).unwrap(), &mut false).unwrap().ticks(), 0xfedcba9876543210);
}
#[test]
fn frequency_conversion_has_no_tick_multiplier_overflow() {
    let max = CounterTicks(u64::MAX);
    assert_eq!(max.as_duration(NonZeroU32::new(1).unwrap()), Duration::from_secs(u64::MAX));
    assert_eq!(CounterTicks(32).as_duration(NonZeroU32::new(32_000_000).unwrap()), Duration::from_micros(1));
    assert_eq!(CounterTicks(1).as_duration(NonZeroU32::new(32_000_000).unwrap()).as_nanos(), 31);
}
#[test]
fn backwards_time_is_not_silently_saturated_to_zero() {
    let frequency = NonZeroU32::new(32_000_000).unwrap();
    assert_eq!(CounterTicks(1).checked_duration_since(CounterTicks(2), frequency), None);
    assert_eq!(CounterTicks(33).checked_duration_since(CounterTicks(1), frequency), Some(Duration::from_micros(1)));
}
proptest::proptest! {
    #[test]
    fn conversion_matches_wide_reference(ticks in proptest::num::u64::ANY, hz in 1u32..=u32::MAX) {
        let duration = CounterTicks(ticks).as_duration(NonZeroU32::new(hz).unwrap());
        proptest::prop_assert_eq!(duration.as_nanos(), u128::from(ticks) * 1_000_000_000 / u128::from(hz));
    }
}

#[test]
fn pac_backend_does_not_replay_clear_or_include_reserved_data_halves() {
    // Local RAM encoding check, not an MMIO device or latch-behavior model.
    // Build all five slots before creating the PAC view; no backing reference
    // is used after conversion. Layout is already validated against the SDK.
    let backing =
        std::boxed::Box::leak(std::boxed::Box::new([0xa5000016u32, 0xaaaadef0, 0xbbbb9abc, 0xcccc5678, 0xdddd1234]));
    assert_eq!(core::mem::size_of_val(backing), core::mem::size_of::<crate::soc::pac::tcxo::RegisterBlock>());
    let regs = unsafe { &*backing.as_mut_ptr().cast::<crate::soc::pac::tcxo::RegisterBlock>() };
    let mut io = PacRegisters(regs);
    io.request();
    assert_eq!(regs.tcxo_status().read().bits(), 0xa5000015);
    assert_eq!(io.words_high_to_low(), [0x1234, 0x5678, 0x9abc, 0xdef0]);
}

#[test]
fn cancelled_pending_request_cannot_be_overwritten_or_read_early() {
    let mut io = Mock { enabled: true, ready_at: 10, ..Mock::default() };
    let mut pending = false;
    begin(&mut io, &mut pending).unwrap();
    assert!(pending);
    assert_eq!(poll(&mut io, &mut pending), Err(nb::Error::WouldBlock));
    assert_eq!(begin(&mut io, &mut pending), Err(TcxoError::SnapshotBusy));
    assert_eq!(discard(&mut io, &mut pending), Err(nb::Error::WouldBlock));
    assert_eq!((io.requests, io.data_reads), (1, 0));
    io.ready_at = 0;
    assert_eq!(discard(&mut io, &mut pending), Ok(()));
    assert!(!pending);
    assert_eq!(io.data_reads, 0);
    begin(&mut io, &mut pending).unwrap();
    io.words = [1, 2, 3, 4];
    assert_eq!(poll(&mut io, &mut pending).unwrap().ticks(), 0x0001000200030004);
    assert_eq!((io.requests, io.data_reads), (2, 1));
    assert!(!pending);
}
#[test]
fn timeout_retains_pending_until_late_completion_can_be_discarded() {
    let mut io = Mock { enabled: true, ready_at: 100, ..Mock::default() };
    let mut pending = false;
    assert_eq!(snapshot(&mut io, NonZeroU16::new(3).unwrap(), &mut pending), Err(TcxoError::SnapshotTimeout));
    assert!(pending);
    assert_eq!(snapshot(&mut io, NonZeroU16::new(3).unwrap(), &mut pending), Err(TcxoError::SnapshotBusy));
    assert_eq!((io.requests, io.data_reads), (1, 0));
    io.ready_at = 0;
    assert!(snapshot(&mut io, NonZeroU16::new(3).unwrap(), &mut pending).is_ok());
    assert_eq!((io.requests, io.data_reads), (2, 1));
    assert!(!pending);
}
#[test]
fn discarded_completed_snapshot_is_never_returned_as_new_request() {
    let mut io = Mock { enabled: true, ready_at: 0, words: [0, 0, 0, 7], ..Mock::default() };
    let mut pending = false;
    begin(&mut io, &mut pending).unwrap();
    // Cancel without polling a completed snapshot; next begin must discard it.
    begin(&mut io, &mut pending).unwrap();
    assert_eq!((io.requests, io.data_reads), (2, 0));
    io.words = [0, 0, 0, 19];
    assert_eq!(poll(&mut io, &mut pending).unwrap().ticks(), 19);
    let polls = io.polls;
    assert_eq!(discard(&mut io, &mut pending), Ok(()));
    assert_eq!(io.polls, polls);
}
