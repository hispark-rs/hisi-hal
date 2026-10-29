use super::*;
use std::collections::VecDeque;

const READY: State = State { boot_done: true, period: 0x1f, mode: 0xcafe5a5a };
struct Fake {
    overrides: VecDeque<State>,
    state_reads: usize,
    commands: Vec<u32>,
    reads: Vec<(usize, bool)>,
}
impl Fake {
    fn new() -> Self {
        Self { overrides: VecDeque::new(), state_reads: 0, commands: Vec::new(), reads: Vec::new() }
    }
}
impl Registers for Fake {
    fn state(&mut self) -> State {
        self.state_reads += 1;
        self.overrides.pop_front().unwrap_or(READY)
    }
    fn read_command(&mut self, upper: u32) {
        assert_eq!(upper, 0xcafe0000);
        self.commands.push(upper | 0x5a5a);
    }
    fn data(&mut self, slot: usize, odd: bool) -> u8 {
        assert!(slot < 128);
        self.reads.push((slot, odd));
        ((slot * 2 + usize::from(odd)) * 73 + 19) as u8
    }
}
fn address(n: u16) -> EfuseByteAddress {
    EfuseByteAddress::new(n).unwrap()
}

#[test]
fn typed_address_rejects_entire_out_of_region_domain() {
    for n in 0..=u16::MAX {
        assert_eq!(EfuseByteAddress::new(n).is_some(), n < 256);
    }
}
#[test]
fn all_bytes_preserve_mode_upper_and_select_correct_slot_half() {
    let mut e = Engine { io: Fake::new() };
    for n in 0..256 {
        assert_eq!(e.read(address(n)), Ok((n.wrapping_mul(73) + 19) as u8));
        assert_eq!(e.io.reads[usize::from(n)], (usize::from(n) / 2, n & 1 != 0));
    }
    assert_eq!(e.io.commands, vec![0xcafe5a5a; 256]);
    assert_eq!(e.io.state_reads, 768);
}
#[test]
fn guard_failure_performs_no_command_or_data_read() {
    for (state, error) in [
        (State { boot_done: false, ..READY }, EfuseError::NotReady),
        (State { period: 0, ..READY }, EfuseError::TimingMismatch(0)),
        (State { mode: 0xcafea5a5, ..READY }, EfuseError::ModeMismatch(0xa5a5)),
        (State { mode: 0xcafe1234, ..READY }, EfuseError::ModeMismatch(0x1234)),
    ] {
        let mut e = Engine { io: Fake::new() };
        e.io.overrides.push_back(state);
        assert_eq!(e.read(address(0)), Err(error));
        assert!(e.io.commands.is_empty() && e.io.reads.is_empty());
    }
}
#[test]
fn command_readback_failure_does_not_touch_data() {
    let mut e = Engine { io: Fake::new() };
    e.io.overrides.extend([READY, State { mode: 0xcafe0000, ..READY }]);
    assert_eq!(e.read(address(0)), Err(EfuseError::ModeMismatch(0)));
    assert_eq!(e.io.commands.len(), 1);
    assert!(e.io.reads.is_empty());
}
#[test]
fn changed_profile_after_data_read_does_not_publish_byte() {
    let mut e = Engine { io: Fake::new() };
    e.io.overrides.extend([READY, READY, State { boot_done: false, ..READY }]);
    let mut data = [0xa5; 1];
    let error = e.read_buffer(address(0), &mut data).unwrap_err();
    assert_eq!((error.completed(), error.error()), (0, EfuseError::NotReady));
    assert_eq!(data, [0xa5]);
    assert_eq!(e.io.reads.len(), 1);
}
#[test]
fn every_valid_buffer_range_matches_original_layout() {
    for start in 0..256 {
        for count in 1..=256 - usize::from(start) {
            let mut e = Engine { io: Fake::new() };
            let mut data = [0xa5; 258];
            assert_eq!(e.read_buffer(address(start), &mut data[1..1 + count]), Ok(()));
            for n in 0..count {
                assert_eq!(data[1 + n], ((usize::from(start) + n) * 73 + 19) as u8);
            }
            assert_eq!(data[0], 0xa5);
            assert!(data[1 + count..].iter().all(|&b| b == 0xa5));
            assert_eq!(e.io.reads.len(), count);
        }
    }
}
#[test]
fn empty_and_bad_range_have_no_mmio_and_leave_output_untouched() {
    let mut e = Engine { io: Fake::new() };
    let mut data = [0xa5; 257];
    assert_eq!(e.read_buffer(address(255), &mut []), Ok(()));
    for (start, len) in [(0, 257), (1, 256), (255, 2)] {
        assert_eq!(
            e.read_buffer(address(start), &mut data[..len]),
            Err(ReadError { completed: 0, error: EfuseError::InvalidRange })
        );
    }
    assert_eq!(data, [0xa5; 257]);
    assert_eq!(e.io.state_reads, 0);
}
#[test]
fn mid_buffer_failure_reports_prefix_and_allows_retry() {
    for prefix in 0..16 {
        let mut e = Engine { io: Fake::new() };
        e.io.overrides.extend(core::iter::repeat_n(READY, prefix * 3));
        e.io.overrides.push_back(State { period: 7, ..READY });
        let mut data = [0xa5; 16];
        let error = e.read_buffer(address(240), &mut data).unwrap_err();
        assert_eq!((error.completed(), error.error()), (prefix, EfuseError::TimingMismatch(7)));
        for (n, &value) in data[..prefix].iter().enumerate() {
            assert_eq!(value, ((240 + n) * 73 + 19) as u8);
        }
        assert!(data[prefix..].iter().all(|&b| b == 0xa5));
        assert_eq!(e.read(address(255)), Ok((255u16 * 73 + 19) as u8));
    }
}
