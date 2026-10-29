use super::*;
use std::collections::VecDeque;
use std::{cell::RefCell, rc::Rc};

struct RingFake {
    word: u16,
    forced_after: Option<u16>,
    reads: usize,
    writes: usize,
    trace: Rc<RefCell<Vec<&'static str>>>,
}
impl RingRegisters for RingFake {
    fn control(&mut self) -> u16 {
        self.trace.borrow_mut().push("control");
        self.reads += 1;
        if self.reads == 1 { self.word } else { self.forced_after.unwrap_or(self.word) }
    }
    fn enable_bit(&mut self) {
        self.trace.borrow_mut().push("enable");
        self.word |= 1;
        self.writes += 1;
    }
    fn fence(&mut self) {
        self.trace.borrow_mut().push("fence");
    }
}
struct RingDelay(Rc<RefCell<Vec<&'static str>>>);
impl DelayNs for RingDelay {
    fn delay_ns(&mut self, ns: u32) {
        assert_eq!(ns, 200_000);
        self.0.borrow_mut().push("settle200us");
    }
}

#[test]
fn ring_start_preserves_every_other_bit_and_always_settles_before_readback() {
    for before in 0..=u16::MAX {
        let trace = Rc::new(RefCell::new(Vec::new()));
        let mut io = RingFake { word: before, forced_after: None, reads: 0, writes: 0, trace: trace.clone() };
        assert_eq!(start_ring(&mut io, &mut RingDelay(trace.clone())), Ok(()));
        assert_eq!(io.word, before | 1);
        assert_eq!(io.reads, 2);
        assert_eq!(io.writes, usize::from(before & 1 == 0));
        let expected = if before & 1 == 0 {
            vec!["control", "enable", "fence", "settle200us", "control"]
        } else {
            vec!["control", "fence", "settle200us", "control"]
        };
        assert_eq!(*trace.borrow(), expected);
    }
}

#[test]
fn ring_start_failure_is_not_retried_or_disabled_and_records_full_readback() {
    for (before, after) in [(0, 0), (0, 3), (0xa5a4, 1), (1, 0), (0xffff, 0xfffd)] {
        let trace = Rc::new(RefCell::new(Vec::new()));
        let mut io = RingFake { word: before, forced_after: Some(after), reads: 0, writes: 0, trace: trace.clone() };
        assert_eq!(
            start_ring(&mut io, &mut RingDelay(trace.clone())),
            Err(RingStartError::ControlChanged { before, after })
        );
        assert_eq!(io.reads, 2);
        assert_eq!(io.writes, usize::from(before & 1 == 0));
        assert_eq!(io.word, before | 1);
        assert_eq!(trace.borrow().iter().filter(|s| **s == "settle200us").count(), 1);
    }
}

#[cfg(feature = "defmt")]
#[test]
fn ring_start_failure_types_support_defmt() {
    fn supported<T: defmt::Format>() {}
    supported::<RingStartError>();
    supported::<RingStartFailure<'static>>();
}

struct Fake {
    ring: VecDeque<bool>,
    ready: VecDeque<bool>,
    words: VecDeque<u32>,
    trace: Vec<&'static str>,
}
impl Fake {
    fn new(ready: &[bool], words: &[u32]) -> Self {
        Self {
            ring: VecDeque::new(),
            ready: ready.iter().copied().collect(),
            words: words.iter().copied().collect(),
            trace: Vec::new(),
        }
    }
}
impl Registers for Fake {
    fn ring_enabled(&mut self) -> bool {
        self.trace.push("ring");
        self.ring.pop_front().unwrap_or(true)
    }
    fn ready(&mut self) -> bool {
        self.trace.push("ready");
        self.ready.pop_front().expect("read beyond scripted budget")
    }
    fn consume(&mut self) -> u32 {
        self.trace.push("consume");
        self.words.pop_front().expect("unscripted FIFO consumption")
    }
}
#[derive(Default)]
struct Delay(Vec<u32>);
impl DelayNs for Delay {
    fn delay_ns(&mut self, ns: u32) {
        self.0.push(ns);
    }
}
fn budget(n: u16) -> NonZeroU16 {
    NonZeroU16::new(n).unwrap()
}

#[test]
fn ring_failure_never_touches_fifo_or_delays() {
    let mut e = Engine { io: Fake::new(&[], &[]) };
    e.io.ring.push_back(false);
    let mut d = Delay::default();
    assert_eq!(e.read(&mut 5, &mut d), Err(Error::RingDisabled));
    assert_eq!(e.io.trace, ["ring"]);
    assert!(d.0.is_empty());
}

#[test]
fn ready_race_and_not_ready_do_not_consume() {
    for flags in [&[false][..], &[true, false][..]] {
        let mut e = Engine { io: Fake::new(flags, &[9, 10]) };
        assert_eq!(e.try_read(), Ok(None));
        assert_eq!(e.io.words.len(), 2);
        assert!(!e.io.trace.contains(&"consume"));
    }
}

#[test]
fn candidate_filter_matches_executed_sdk_contract() {
    for (word, comparison, expected) in [
        (0, 1, None),
        (u32::MAX, 1, None),
        (7, 7, None),
        (1, 0, Some(1)),
        (1, u32::MAX, Some(1)),
        (0x12345678, 8, Some(0x12345678)),
    ] {
        let mut e = Engine { io: Fake::new(&[true, true], &[word, comparison]) };
        assert_eq!(e.try_read(), Ok(expected));
        assert_eq!(e.io.trace, ["ring", "ready", "ready", "consume", "consume"]);
    }
}

#[test]
fn accepted_repeated_words_are_not_an_entropy_health_claim() {
    let mut e = Engine { io: Fake::new(&[true; 4], &[7, 8, 7, 9]) };
    assert_eq!(e.try_read(), Ok(Some(7)));
    assert_eq!(e.try_read(), Ok(Some(7)));
}

#[test]
fn rejected_pairs_share_budget_and_only_failures_delay() {
    let mut e = Engine { io: Fake::new(&[true; 6], &[0, 1, 7, 7, 8, 9]) };
    let mut d = Delay::default();
    let mut remaining = 3;
    assert_eq!(e.read(&mut remaining, &mut d), Ok(8));
    assert_eq!(remaining, 0);
    assert_eq!(d.0, [1000, 1000]);
}

#[test]
fn timeout_does_not_return_rejected_output_and_retry_can_succeed() {
    let mut e = Engine { io: Fake::new(&[true; 4], &[u32::MAX, 1, 8, 9]) };
    let mut d = Delay::default();
    assert_eq!(e.read(&mut 1, &mut d), Err(Error::Timeout));
    assert_eq!(e.read(&mut 1, &mut d), Ok(8));
    assert!(d.0.is_empty());
}

#[test]
fn fill_zero_and_word_tails_preserve_guards_and_use_little_endian() {
    for n in 0usize..=17 {
        let count = n.div_ceil(4);
        let mut words = Vec::new();
        for _ in 0..count {
            words.extend([0x12345678, 0xabcdef01]);
        }
        let mut e = Engine { io: Fake::new(&vec![true; count * 2], &words) };
        let mut data = [0xa5; 20];
        assert_eq!(e.fill(&mut data[1..1 + n], budget(5), &mut Delay::default()), Ok(()));
        for i in 0..n {
            assert_eq!(data[i + 1], [0x78, 0x56, 0x34, 0x12][i % 4]);
        }
        assert_eq!(data[0], 0xa5);
        assert!(data[n + 1..].iter().all(|&b| b == 0xa5));
        assert!(e.io.words.is_empty());
        if n == 0 {
            assert!(e.io.trace.is_empty());
        }
    }
}

#[test]
fn fill_budget_is_total_not_reset_per_word() {
    let mut e = Engine { io: Fake::new(&[true; 2], &[0x12345678, 9]) };
    let mut data = [0xa5; 9];
    let error = e.fill(&mut data, budget(1), &mut Delay::default()).unwrap_err();
    assert_eq!((error.written(), error.error()), (4, Error::Timeout));
    assert_eq!(data, [0x78, 0x56, 0x34, 0x12, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5]);
}

#[test]
fn partial_failure_reports_prefix_without_writing_rejected_candidate() {
    for ring_off in [false, true] {
        let mut e = Engine { io: Fake::new(&[true; 4], &[0x12345678, 9, 0, 1]) };
        e.io.ring.extend([true, !ring_off]);
        let mut data = [0xa5; 8];
        let mut d = Delay::default();
        let error = e.fill(&mut data, budget(2), &mut d).unwrap_err();
        assert_eq!(error.written(), 4);
        assert_eq!(error.error(), if ring_off { Error::RingDisabled } else { Error::Timeout });
        assert_eq!(data[4..], [0xa5; 4]);
        assert!(d.0.is_empty());
    }
}

#[test]
fn zero_budget_internal_read_is_inert_and_does_not_underflow() {
    let mut e = Engine { io: Fake::new(&[], &[]) };
    let mut d = Delay::default();
    assert_eq!(e.read(&mut 0, &mut d), Err(Error::Timeout));
    assert!(e.io.trace.is_empty() && d.0.is_empty());
}
