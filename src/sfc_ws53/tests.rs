use super::*;
use std::{collections::VecDeque, vec, vec::Vec};

fn budget(ticks: u64, polls: u32) -> Budget {
    Budget::new(NonZeroU64::new(ticks).unwrap(), NonZeroU32::new(polls).unwrap())
}
fn address(n: u32) -> FlashAddress {
    FlashAddress::try_new(n).unwrap()
}
fn length(n: usize) -> ReadLength {
    ReadLength::try_new(n).unwrap()
}
fn region() -> ReadRegion {
    ReadRegion::try_new(address(0x100), NonZeroU32::new(0x100).unwrap()).unwrap()
}
fn command() -> Command {
    Command::Read(address(0x100), length(7))
}

#[derive(Default)]
struct Clock {
    tick: u64,
    step: u64,
    calls: usize,
    fail_at: Option<usize>,
    backwards_at: Option<usize>,
}
impl Clock {
    fn running() -> Self {
        Self { step: 1, ..Self::default() }
    }
}
impl TickSource for Clock {
    fn now(&mut self) -> Option<u64> {
        self.calls += 1;
        if self.fail_at == Some(self.calls) {
            return None;
        }
        if self.backwards_at == Some(self.calls) {
            return Some(self.tick.saturating_sub(2));
        }
        let result = self.tick;
        self.tick += self.step;
        Some(result)
    }
}

/// Independent command model: output initially contains stale sentinel bytes;
/// fresh data is published only by completion after the configured START polls.
struct Mock {
    snapshot: Snapshot,
    starts: Vec<(u8, Option<u32>, usize)>,
    reads: Vec<usize>,
    snapshot_reads: usize,
    poll_reads: usize,
    pending_polls: u32,
    latency: u32,
    stalled: bool,
    pending: Option<(u8, Option<u32>, usize)>,
    data: [u8; 64],
    statuses: VecDeque<u8>,
    status3: u8,
    mutate_on_data: bool,
    late_clock_change: bool,
    array_data_read: bool,
}
impl Default for Mock {
    fn default() -> Self {
        Self {
            snapshot: Snapshot { global: 0xa5a5_0000, dma: 0x5a00, command: 0, clock: 0x5a5a_000d },
            starts: Vec::new(),
            reads: Vec::new(),
            snapshot_reads: 0,
            poll_reads: 0,
            pending_polls: 0,
            latency: 2,
            stalled: false,
            pending: None,
            data: [0xee; 64],
            statuses: VecDeque::new(),
            status3: 0xa7,
            mutate_on_data: false,
            late_clock_change: false,
            array_data_read: false,
        }
    }
}
impl Registers for Mock {
    fn snapshot(&mut self) -> Snapshot {
        self.snapshot_reads += 1;
        if self.late_clock_change && self.array_data_read {
            self.snapshot.clock ^= 1 << 20;
        }
        self.snapshot
    }
    fn start(&mut self, command: Command) {
        assert!(self.pending.is_none(), "driver started over an outstanding command");
        let shape = command.shape();
        self.starts.push(shape);
        self.pending = Some(shape);
        self.pending_polls = self.latency;
        self.snapshot.command |= 1;
    }
    fn busy(&mut self) -> bool {
        self.poll_reads += 1;
        assert!(self.pending.is_some());
        if self.stalled {
            return true;
        }
        self.pending_polls = self.pending_polls.saturating_sub(1);
        if self.pending_polls != 0 {
            return true;
        }
        let (opcode, address, _) = self.pending.take().unwrap();
        self.data.fill(0xee);
        match opcode {
            0x9f => self.data[..3].copy_from_slice(&[0x7f, 0x53, 0x01]),
            0x05 => self.data[0] = self.statuses.pop_front().unwrap_or(0),
            0x35 => self.data[0] = 0x42,
            0x15 => self.data[0] = self.status3,
            0x03 => {
                let base = address.unwrap();
                for (i, byte) in self.data.iter_mut().enumerate() {
                    *byte = (base as u8).wrapping_add(i as u8) ^ 0x69;
                }
            }
            _ => panic!("non-read opcode"),
        }
        self.snapshot.command &= !1;
        false
    }
    fn data_word(&mut self, index: usize) -> u32 {
        assert!(self.pending.is_none(), "stale data before completion");
        self.reads.push(index);
        self.array_data_read |= self.starts.last().unwrap().0 == 0x03;
        if self.mutate_on_data && self.array_data_read {
            self.snapshot.dma |= 1;
        }
        u32::from_le_bytes(self.data[index * 4..index * 4 + 4].try_into().unwrap())
    }
}
fn engine() -> Engine<Mock> {
    Engine::attach(Mock::default()).unwrap()
}

#[test]
fn typed_lengths_addresses_and_ranges_reject_overflow() {
    for n in 1..=64 {
        assert_eq!(length(n).get(), n);
    }
    for n in [0, 65, 255, usize::MAX] {
        assert!(ReadLength::try_new(n).is_none());
    }
    assert_eq!(address(0xffffff).get(), 0xffffff);
    for n in [0x1000000, u32::MAX] {
        assert!(FlashAddress::try_new(n).is_none());
    }
    assert!(ReadRegion::try_new(address(0xffffff), NonZeroU32::new(2).unwrap()).is_none());
    assert!(ReadRegion::try_new(address(1), NonZeroU32::new(u32::MAX).unwrap()).is_none());
    let all = ReadRegion::try_new(address(0), NonZeroU32::new(0x1000000).unwrap()).unwrap();
    assert!(all.contains(address(0xffffff), length(1)));
    assert!(!all.contains(address(0xffffff), length(2)));
    assert!(region().contains(address(0x1c0), length(64)));
    assert!(!region().contains(address(0x1c1), length(64)));
    assert!(!region().contains(address(0xff), length(1)));
}

#[test]
fn every_admission_failure_has_no_writes_or_data_reads() {
    let good = Mock::default().snapshot;
    let cases = [
        (Snapshot { command: 1, ..good }, AdmissionError::CommandBusy),
        (Snapshot { dma: 1, ..good }, AdmissionError::DmaBusy),
        (Snapshot { clock: 0xc, ..good }, AdmissionError::ClockDisabled),
        (Snapshot { clock: 3, ..good }, AdmissionError::ClockSource),
        (Snapshot { global: 1, ..good }, AdmissionError::SpiMode),
        (Snapshot { global: 4, ..good }, AdmissionError::AddressMode),
    ];
    for (snapshot, expected) in cases {
        assert_eq!(snapshot.validate(), Err(expected));
    }
    for code in 0..8 {
        let snapshot = Snapshot { clock: 1 | code << 1, ..good };
        assert_eq!(snapshot.validate().is_ok(), matches!(code, 0 | 6));
    }
    // Attach takes only one snapshot and has no start/write hook on failure.
    let mut e = engine();
    e.io.snapshot = cases[0].0;
    assert_eq!(
        e.run(Command::Jedec, budget(1000, 100), &mut Clock::running()),
        Err(Error::Admission(AdmissionError::CommandBusy))
    );
    assert!(e.io.starts.is_empty() && e.io.reads.is_empty());
    assert!(e.poisoned);
}

#[test]
fn named_identity_and_status_commands_return_fresh_exact_bytes() {
    let mut e = engine();
    let mut clock = Clock::running();
    let b = budget(1000, 100);
    assert_eq!(&e.run(Command::Jedec, b, &mut clock).unwrap()[..3], &[0x7f, 0x53, 1]);
    for (status, expected) in
        [(StatusRegister::Status1, 0), (StatusRegister::Status2, 0x42), (StatusRegister::Status3, 0xa7)]
    {
        assert_eq!(e.run(Command::Status(status), b, &mut clock).unwrap()[0], expected);
    }
    assert_eq!(
        e.io.starts,
        vec![(5, None, 1), (0x9f, None, 3), (5, None, 1), (5, None, 1), (0x35, None, 1), (5, None, 1), (0x15, None, 1)]
    );
    assert_eq!(e.io.reads, vec![0; 7]);
    assert!(!e.poisoned);
}

#[test]
fn all_lengths_pack_partial_words_after_fresh_wip_ready() {
    for count in 1..=64 {
        let mut e = engine();
        e.io.statuses.extend([1, 1, 0]);
        let mut output = vec![0xab; count];
        e.read(region(), address(0x101), length(count), &mut output, budget(1000, 100), &mut Clock::running()).unwrap();
        assert_eq!(output, (0..count).map(|n| (1u8).wrapping_add(n as u8) ^ 0x69).collect::<Vec<_>>());
        assert_eq!(e.io.starts, vec![(5, None, 1), (5, None, 1), (5, None, 1), (3, Some(0x101), count)]);
        assert_eq!(e.io.reads, [vec![0; 3], (0..count.div_ceil(4)).collect()].concat());
        assert_eq!(e.io.poll_reads, 8);
    }
}

#[test]
fn invalid_slice_or_range_does_not_touch_controller_clock_or_buffer() {
    for (addr, len, mut output, expected) in [
        (address(0x100), length(2), vec![0xab], Error::BufferLength),
        (address(0x100), length(1), vec![], Error::BufferLength),
        (address(0xff), length(2), vec![0xab; 2], Error::OutsideRegion),
        (address(0x1ff), length(2), vec![0xab; 2], Error::OutsideRegion),
    ] {
        let before = output.clone();
        let mut e = engine();
        let mut clock = Clock::running();
        assert_eq!(e.read(region(), addr, len, &mut output, budget(1000, 100), &mut clock), Err(expected));
        assert_eq!(output, before);
        assert_eq!(e.io.snapshot_reads, 1);
        assert!(e.io.starts.is_empty() && e.io.reads.is_empty());
        assert_eq!(clock.calls, 0);
        assert!(!e.poisoned);
    }
}

fn assert_poison_has_no_access(e: &mut Engine<Mock>) {
    let before = (e.io.starts.len(), e.io.reads.len(), e.io.snapshot_reads, e.io.poll_reads);
    let mut clock = Clock::running();
    assert_eq!(e.run(Command::Jedec, budget(1000, 100), &mut clock), Err(Error::Poisoned));
    let mut buffer = [0x55; 7];
    assert_eq!(
        e.read(region(), address(0x100), length(7), &mut buffer, budget(1000, 100), &mut clock),
        Err(Error::Poisoned)
    );
    assert_eq!(buffer, [0x55; 7]);
    assert_eq!(before, (e.io.starts.len(), e.io.reads.len(), e.io.snapshot_reads, e.io.poll_reads));
    assert_eq!(clock.calls, 0);
}

#[test]
fn stopped_clock_hits_poll_limit_without_clearing_pending_start() {
    let mut e = engine();
    e.io.stalled = true;
    let mut output = [0xab; 7];
    assert_eq!(
        e.read(region(), address(0x100), length(7), &mut output, budget(100, 3), &mut Clock::default()),
        Err(Error::PollLimit(Stage::NorReady))
    );
    assert_eq!(output, [0xab; 7]);
    assert_eq!(e.io.poll_reads, 3);
    assert_eq!(e.io.starts.len(), 1);
    assert_eq!(e.io.snapshot.command & 1, 1);
    assert!(e.io.reads.is_empty());
    assert_poison_has_no_access(&mut e);
}

#[test]
fn running_deadline_rejects_stall_before_poll_limit() {
    let mut e = engine();
    e.io.stalled = true;
    assert_eq!(
        e.run(Command::Status(StatusRegister::Status1), budget(3, 100), &mut Clock::running()),
        Err(Error::Deadline(Stage::Command))
    );
    assert_eq!(e.io.poll_reads, 1);
    assert!(e.io.reads.is_empty());
    assert_eq!(e.io.snapshot.command & 1, 1);
    assert_poison_has_no_access(&mut e);
}

#[test]
fn nor_busy_uses_fresh_status_with_one_budget_and_never_reads_array() {
    let mut e = engine();
    e.io.latency = 1;
    e.io.statuses.extend([1; 10]);
    let mut output = [0xab; 7];
    assert_eq!(
        e.read(region(), address(0x100), length(7), &mut output, budget(1000, 3), &mut Clock::default()),
        Err(Error::PollLimit(Stage::NorReady))
    );
    assert_eq!(output, [0xab; 7]);
    assert_eq!(e.io.starts, vec![(5, None, 1); 3]);
    assert_eq!(e.io.reads, vec![0; 3]);
    assert!(e.io.pending.is_none());
    assert_poison_has_no_access(&mut e);
}

#[test]
fn shared_budget_cannot_restart_for_array_command() {
    let mut e = engine();
    e.io.latency = 1;
    let mut output = [0xab; 7];
    assert_eq!(
        e.read(region(), address(0x100), length(7), &mut output, budget(1000, 1), &mut Clock::default()),
        Err(Error::PollLimit(Stage::Command))
    );
    assert_eq!(output, [0xab; 7]);
    assert_eq!(e.io.starts, vec![(5, None, 1)]);
    assert_poison_has_no_access(&mut e);
}

#[test]
fn failed_or_backwards_time_poison_and_preserve_buffer() {
    for failure in 1..=12 {
        let mut e = engine();
        let mut clock = Clock { fail_at: Some(failure), ..Clock::running() };
        let mut output = [0xab; 7];
        let result = e.read(region(), address(0x100), length(7), &mut output, budget(1000, 100), &mut clock);
        if result.is_err() {
            assert_eq!(result, Err(Error::TimeSource));
            assert_eq!(output, [0xab; 7]);
            assert_poison_has_no_access(&mut e);
        } else {
            assert!(failure > clock.calls);
        }
    }
    let mut e = engine();
    let mut clock = Clock { backwards_at: Some(4), ..Clock::running() };
    assert_eq!(e.run(command(), budget(1000, 100), &mut clock), Err(Error::TimeSource));
    assert_poison_has_no_access(&mut e);
}

#[test]
fn data_read_or_config_change_never_commits_partial_output() {
    for dma in [false, true] {
        let mut e = engine();
        e.io.mutate_on_data = dma;
        e.io.late_clock_change = !dma;
        let mut output = [0xab; 7];
        let expected = if dma { AdmissionError::DmaBusy } else { AdmissionError::ConfigurationChanged };
        assert_eq!(
            e.read(region(), address(0x100), length(7), &mut output, budget(1000, 100), &mut Clock::running()),
            Err(Error::Admission(expected))
        );
        assert_eq!(output, [0xab; 7]);
        assert_eq!(e.io.reads, vec![0, 0, 1], "fresh status plus both staged array words");
        assert_poison_has_no_access(&mut e);
    }
}

#[test]
fn clock_deadline_at_completion_is_not_success() {
    let mut e = engine();
    e.io.latency = 1;
    // start=0, pre-submit=1, poll=2, post-data=3: completion at deadline fails.
    assert_eq!(
        e.run(Command::Status(StatusRegister::Status1), budget(3, 100), &mut Clock::running()),
        Err(Error::Deadline(Stage::Command))
    );
    assert_eq!(e.io.reads, vec![0]);
    assert_poison_has_no_access(&mut e);
}

#[test]
fn read_at_last_byte_does_not_wrap_or_assume_four_megabytes() {
    let all = ReadRegion::try_new(address(0), NonZeroU32::new(0x1000000).unwrap()).unwrap();
    let mut e = engine();
    let mut byte = [0xab];
    e.read(all, address(0xffffff), length(1), &mut byte, budget(1000, 100), &mut Clock::running()).unwrap();
    assert_eq!(byte, [0xff ^ 0x69]);
    assert_eq!(e.io.starts.last(), Some(&(3, Some(0xffffff), 1)));
}

#[test]
fn busy_nor_never_launches_identity_or_other_status_commands() {
    for command in [Command::Jedec, Command::Status(StatusRegister::Status2), Command::Status(StatusRegister::Status3)]
    {
        let mut e = engine();
        e.io.latency = 1;
        e.io.statuses.extend([1; 8]);
        assert_eq!(e.run(command, budget(1000, 3), &mut Clock::default()), Err(Error::PollLimit(Stage::NorReady)));
        assert_eq!(e.io.starts, vec![(5, None, 1); 3]);
        assert_poison_has_no_access(&mut e);
    }
}

#[test]
fn production_pac_adapter_matches_sdk_complete_command_encoding() {
    // Local RAM objects only: tests real production field writers without
    // dereferencing any numeric MMIO address or claiming reset values.
    let sfc: pac::sfc_cfg::RegisterBlock = unsafe { core::mem::zeroed() };
    let clocks: pac::m_crg::RegisterBlock = unsafe { core::mem::zeroed() };
    unsafe {
        clocks.xip_sfc_clock_word().write_with_zero(|w| w.bits(0xa55a_000d));
        sfc.global_config().write_with_zero(|w| w.bits(0x5aa5_0000));
        sfc.bus_dma_ctrl().write_with_zero(|w| w.bits(0xa500));
        sfc.cmd_ins().write_with_zero(|w| w.bits(0xa5a5_5500));
        sfc.cmd_addr().write_with_zero(|w| w.bits(0x654321));
    }
    let mut io = PacRegisters { sfc: &sfc, clocks: &clocks };
    let expected_snapshot = Snapshot { global: 0x5aa5_0000, dma: 0xa500, command: 0, clock: 0xa55a_000d };
    assert_eq!(io.snapshot(), expected_snapshot);
    for n in 1..=64 {
        let addr = address(0xffffc0);
        io.start(Command::Read(addr, length(n)));
        // Original SDK C bitfields: START0, CS1, ADDR_EN3, DATA_EN7,
        // READ8, (n-1)14:9; all interface/dummy/DDR flags stay clear.
        assert_eq!(sfc.cmd_config().read().bits(), 0x18b | ((n as u32 - 1) << 9));
        assert_eq!(sfc.cmd_ins().read().bits(), 0xa5a5_5503);
        assert_eq!(sfc.cmd_addr().read().bits(), 0xffffc0);
        assert!(io.busy());
    }
    for (cmd, opcode, count) in [
        (Command::Jedec, 0x9f, 3),
        (Command::Status(StatusRegister::Status1), 0x05, 1),
        (Command::Status(StatusRegister::Status2), 0x35, 1),
        (Command::Status(StatusRegister::Status3), 0x15, 1),
    ] {
        io.start(cmd);
        assert_eq!(sfc.cmd_config().read().bits(), 0x183 | ((count - 1) << 9));
        assert_eq!(sfc.cmd_ins().read().bits(), 0xa5a5_5500 | opcode);
        assert_eq!(sfc.cmd_addr().read().bits(), 0xffffc0, "no-address command must not alter address");
    }
    assert_eq!(sfc.global_config().read().bits(), expected_snapshot.global);
    assert_eq!(sfc.bus_dma_ctrl().read().bits(), expected_snapshot.dma);
    assert_eq!(clocks.xip_sfc_clock_word().read().bits(), expected_snapshot.clock);
    unsafe {
        sfc.cmd_databuf_0().write_with_zero(|w| w.bits(0x44332211));
        sfc.cmd_databuf_15().write_with_zero(|w| w.bits(0x88776655));
    }
    assert_eq!(io.data_word(0), 0x44332211);
    assert_eq!(io.data_word(15), 0x88776655);
}
