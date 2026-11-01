use super::*;
use std::vec::Vec;

struct Mock {
    released: bool,
    interrupts: bool,
    idle: bool,
    ignore_assert: bool,
    ignore_release: bool,
    writes: Vec<bool>,
    interrupt_reads: usize,
    status_reads: usize,
}
impl Default for Mock {
    fn default() -> Self {
        Self {
            released: true,
            interrupts: false,
            idle: true,
            ignore_assert: false,
            ignore_release: false,
            writes: Vec::new(),
            interrupt_reads: 0,
            status_reads: 0,
        }
    }
}
impl ResetRegisters for Mock {
    fn released(&mut self) -> bool {
        self.released
    }
    fn set_released(&mut self, value: bool) {
        self.writes.push(value);
        if !(if value { self.ignore_release } else { self.ignore_assert }) {
            self.released = value;
        }
    }
    fn interrupts_enabled(&mut self) -> bool {
        self.interrupt_reads += 1;
        self.interrupts
    }
    fn transmit_idle(&mut self) -> bool {
        self.status_reads += 1;
        self.idle
    }
}
#[derive(Default)]
struct Delay(Vec<u32>);
impl DelayNs for Delay {
    fn delay_ns(&mut self, ns: u32) {
        self.0.push(ns);
    }
}

#[test]
fn held_reset_never_reads_uart_mmio() {
    let mut io = Mock { released: false, ..Mock::default() };
    let mut delay = Delay::default();
    assert_eq!(pulse(&mut io, &mut delay), Err(ResetError::AlreadyAsserted));
    assert_eq!((io.interrupt_reads, io.status_reads), (0, 0));
    assert!(io.writes.is_empty() && delay.0.is_empty());
}
#[test]
fn enabled_interrupts_do_not_consume_status_or_change_reset() {
    let mut io = Mock { interrupts: true, ..Mock::default() };
    let mut delay = Delay::default();
    assert_eq!(pulse(&mut io, &mut delay), Err(ResetError::InterruptsEnabled));
    assert_eq!(io.status_reads, 0);
    assert!(io.writes.is_empty() && delay.0.is_empty());
}
#[test]
fn busy_transmitter_is_not_aborted() {
    let mut io = Mock { idle: false, ..Mock::default() };
    let mut delay = Delay::default();
    assert_eq!(pulse(&mut io, &mut delay), Err(ResetError::TransmitBusy));
    assert!(io.writes.is_empty() && delay.0.is_empty());
}
#[test]
fn failed_assertion_still_attempts_release_and_reports_failure() {
    let mut io = Mock { ignore_assert: true, ..Mock::default() };
    let mut delay = Delay::default();
    assert_eq!(pulse(&mut io, &mut delay), Err(ResetError::ResetReadback));
    assert_eq!(io.writes, [false, true]);
    assert!(io.released);
}
#[test]
fn failed_release_is_not_reported_as_ready() {
    let mut io = Mock { ignore_release: true, ..Mock::default() };
    let mut delay = Delay::default();
    assert_eq!(pulse(&mut io, &mut delay), Err(ResetError::ResetReadback));
    assert_eq!(io.writes, [false, true]);
    assert!(!io.released);
}
#[test]
fn successful_pulse_releases_and_observes_before_configuring() {
    let mut io = Mock::default();
    let mut delay = Delay::default();
    assert_eq!(pulse(&mut io, &mut delay), Ok(()));
    assert_eq!(io.writes, [false, true]);
    assert_eq!(delay.0, [10_000]);
    assert!(io.released);
}
