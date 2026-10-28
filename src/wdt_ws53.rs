//! WS53 application watchdog v151 (experimental, SDK source profile).
//!
//! Requires an explicit clock/reset/resource handoff. The driver never changes
//! shared clock registers or NMI routing. The application must arrange the
//! selected NMI hook before starting InterruptThenReset mode. No physical reset
//! or exact expiry timing has been validated. Dropping the driver does not stop
//! the watchdog; call `stop` explicitly while its clock remains active.
//!
//! Evidence: SDK 163d9150, hal_watchdog_v151.c and watchdog_porting.c;
//! register semantics are provided by hisi-registers / ws53-pac. In particular,
//! LOAD programs only counter bits 31:8. The initial low byte and second-stage
//! timing are unknown: the SDK seconds conversion is not an exact timing promise.
//!
//! A-board testing found that CNT consumption clears validity, but a request
//! issued soon after consumption can be lost while BUSY remains zero. Snapshots
//! therefore wait at least 200us and two declared input-clock periods before
//! issuing a request, rounded up to microseconds. The 200us floor also covers
//! command settlement: a 63us guard avoided timeouts but returned stale counts
//! after start/restart on A-board. This is an empirical software guard, not
//! a statement of exact silicon latency. Other clocks and low-power transitions
//! still need qualification. The caller must provide the actual stable clock
//! and an independently progressing delay. Read each valid snapshot only once.
use crate::peripherals::Wdt;
use core::num::NonZeroU32;
use embedded_hal::delay::DelayNs;

const KEY: u32 = 0x5a5a_5a5a;
const WAIT_US: u32 = 2_000;

/// SDK watchdog expiry policy (reset output remains enabled in either mode).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum WatchdogMode {
    /// Request reset at the first expiry.
    Reset,
    /// Assert the peripheral interrupt first; reset on the subsequent expiry.
    /// The WS53 application source profile routes this through NMI source bit 1.
    InterruptThenReset,
}

/// Validated SDK seconds-to-LOAD conversion for an externally established clock.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Config {
    clock: NonZeroU32,
    seconds: NonZeroU32,
    load: u32,
    mode: WatchdogMode,
}
impl Config {
    /// Reproduce the SDK range check and right shift, without u32 overflow.
    ///
    /// `clock` is the actual watchdog input frequency, not the CPU or TCXO
    /// frequency. The reference application uses 32000 Hz (FPGA: 31902 Hz).
    /// Accepts products 255..=u32::MAX. The low eight bits are deliberately not
    /// programmed; their initial value and exact expiry time remain unverified.
    pub const fn from_seconds(clock: NonZeroU32, seconds: u32, mode: WatchdogMode) -> Option<Self> {
        let Some(seconds) = NonZeroU32::new(seconds) else { return None };
        let ticks = clock.get() as u64 * seconds.get() as u64;
        if ticks < 255 || ticks > u32::MAX as u64 {
            return None;
        }
        Some(Self { clock, seconds, load: (ticks >> 8) as u32, mode })
    }
    /// Declared input frequency; ownership handoff must establish this clock.
    pub const fn clock_hz(self) -> NonZeroU32 {
        self.clock
    }
    /// Requested SDK timeout in seconds; not a measured expiry interval.
    pub const fn requested_seconds(self) -> NonZeroU32 {
        self.seconds
    }
    /// Value written to the 24-bit LOAD field (counter bits 31:8).
    pub const fn load_upper_bits(self) -> u32 {
        self.load
    }
    /// Selected expiry policy.
    pub const fn mode(self) -> WatchdogMode {
        self.mode
    }
}

/// A bounded operation failed. Neither error means the counter reached zero.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum WatchdogError {
    /// The bank is locked; no kick or snapshot command was issued.
    Locked,
    /// Unlock status did not clear within 2000 one-microsecond delays.
    UnlockTimeout,
    /// The hardware did not acknowledge the counter snapshot request.
    SnapshotTimeout,
}

/// A valid hardware snapshot; zero is distinct from failure to sample.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct CounterSnapshot {
    ticks: u32,
    clock: NonZeroU32,
}
impl CounterSnapshot {
    /// Captured counter value, including the low eight bits.
    pub const fn ticks(self) -> u32 {
        self.ticks
    }
    /// Floor-converted counter duration. This does not include any second stage
    /// and is not a promise of time until chip reset. u64 avoids overflow at low clocks.
    pub const fn micros(self) -> u64 {
        self.ticks as u64 * 1_000_000 / self.clock.get() as u64
    }
}

/// Owns a handed-over watchdog. Construction performs no MMIO.
pub struct WatchdogDriver<'d> {
    _token: Wdt<'d>,
    config: Config,
    io: PacRegisters,
}
impl<'d> WatchdogDriver<'d> {
    /// Attach after establishing clock/reset and exclusive NMI/resource ownership.
    /// The clock must match Config and remain active for the driver's lifetime.
    pub fn new(token: Wdt<'d>, config: Config) -> Self {
        Self { _token: token, config, io: PacRegisters }
    }
    /// Stop, configure and start using the SDK ordering and reset pulse code 7.
    ///
    /// The NMI hook/routing for InterruptThenReset must already be established.
    /// Unlock failure returns before touching LOAD/control/EOI. An already
    /// running watchdog may still expire during this operation; this is not a
    /// way to take over an active watchdog whose deadline is unknown.
    pub fn start(&mut self, delay: &mut impl DelayNs) -> Result<(), WatchdogError> {
        start(&mut self.io, self.config, delay)
    }
    /// Clear pending peripheral status and disable counting, as in the SDK.
    /// Does not gate the shared clock or release NMI/resource ownership.
    pub fn stop(&mut self, delay: &mut impl DelayNs) -> Result<(), WatchdogError> {
        unlock(&mut self.io, delay)?;
        stop(&mut self.io);
        Ok(())
    }
    /// Reload the counter with the SDK magic key. No EOI is consumed here.
    /// Returns Locked without issuing a kick if the bank has not been unlocked.
    pub fn feed(&mut self) -> Result<(), WatchdogError> {
        feed(&mut self.io)
    }
    /// Wait at least 200us and two declared input-clock periods, then request a
    /// counter sample, with at most 2000 further one-microsecond polling delays.
    /// A locked bank returns before waiting or issuing a request.
    /// Consumes the snapshot and clears hardware validity. Clock/delay must meet
    /// the module's handoff contract; no precise sample timestamp is promised.
    pub fn counter_snapshot(&mut self, delay: &mut impl DelayNs) -> Result<CounterSnapshot, WatchdogError> {
        snapshot(&mut self.io, self.config.clock, delay)
    }
    /// Raw peripheral pending state; does not acknowledge it.
    pub fn interrupt_pending(&self) -> bool {
        PacRegisters::regs().wdt_raw_intr().read().wdt_raw_intr().bit_is_set()
    }
    /// Peripheral pending state after its local mask, before the NMI aggregator.
    pub fn masked_interrupt_pending(&self) -> bool {
        PacRegisters::regs().wdt_intr().read().wdt_intr().bit_is_set()
    }
    /// Clear the watchdog source via the SDK-documented read side effect.
    /// This does not prove the reset stage has restarted or acknowledge other NMI sources.
    ///
    /// # Safety
    /// Call only in this watchdog owner's NMI hook or after proving NMI exclusion.
    /// The resource and clock must remain owned/active; do not borrow or alias the
    /// main-context driver. Return from NMI only after verifying the complete
    /// source-clear protocol. Ordinary critical sections do not exclude NMI.
    pub unsafe fn acknowledge_from_nmi() {
        PacRegisters.clear_interrupt();
    }
    /// Issue a kick from the owning NMI hook without accessing the driver object.
    ///
    /// # Safety
    /// Same ownership/exclusion requirements as `acknowledge_from_nmi`. The bank
    /// must be unlocked. This alone does not prove that the NMI source is cleared.
    pub unsafe fn feed_from_nmi() {
        PacRegisters.restart();
    }
}

// Private operation boundary lets host tests inject delayed/failed handshakes
// and inspect ordering. All real MMIO still goes through field-level PAC access.
trait Registers {
    fn unlock(&mut self);
    fn locked(&mut self) -> bool;
    fn clear_interrupt(&mut self);
    fn enable(&mut self, enabled: bool);
    fn configure(&mut self);
    fn mode(&mut self, mode: WatchdogMode);
    fn load(&mut self, value: u32);
    fn restart(&mut self);
    fn request_snapshot(&mut self);
    fn snapshot_valid(&mut self) -> bool;
    fn count(&mut self) -> u32;
}
struct PacRegisters;
impl PacRegisters {
    fn regs() -> &'static crate::soc::pac::wdt::RegisterBlock {
        // All callers require a live, handed-over Wdt token (or the documented NMI contract).
        unsafe { &*Wdt::ptr() }
    }
}
impl Registers for PacRegisters {
    fn unlock(&mut self) {
        unsafe {
            Self::regs().lock_command().write_with_zero(|w| w.key().bits(KEY));
        }
    }
    fn locked(&mut self) -> bool {
        Self::regs().lock_status().read().locked().bit_is_set()
    }
    fn clear_interrupt(&mut self) {
        // Deliberate consuming read. Do not copy the SDK's read-then-write of EOI:
        // an independent write-clear effect has not been established in the RDL.
        let _ = Self::regs().wdt_eoi().read();
    }
    fn enable(&mut self, enabled: bool) {
        Self::regs().wdt_cr().modify(|_, w| w.wdt_en().bit(enabled));
    }
    fn configure(&mut self) {
        Self::regs().wdt_cr().modify(|_, w| unsafe { w.rst_en().set_bit().wdt_imsk().clear_bit().rst_pl().bits(7) });
    }
    fn mode(&mut self, mode: WatchdogMode) {
        Self::regs().wdt_cr().modify(|_, w| match mode {
            WatchdogMode::Reset => w.wdt_mode().reset_on_first(),
            WatchdogMode::InterruptThenReset => w.wdt_mode().reset_on_second(),
        });
    }
    fn load(&mut self, value: u32) {
        // Config has already bounded the 24-bit field. Preserve reserved low bits.
        Self::regs().wdt_load().modify(|_, w| unsafe { w.wdt_load().bits(value) });
    }
    fn restart(&mut self) {
        unsafe {
            Self::regs().wdt_restart().write_with_zero(|w| w.wdt_restart().bits(KEY));
        }
    }
    fn request_snapshot(&mut self) {
        unsafe {
            Self::regs().ccvr_request().write_with_zero(|w| w.request().set_bit());
        }
    }
    fn snapshot_valid(&mut self) -> bool {
        Self::regs().ccvr_status().read().valid().bit_is_set()
    }
    fn count(&mut self) -> u32 {
        // Consuming read: the generated PAC describes its effect on CCVR validity.
        Self::regs().wdt_cnt().read().wdt_cnt().bits()
    }
}
fn wait(delay: &mut impl DelayNs, mut ready: impl FnMut() -> bool) -> bool {
    for elapsed in 0..=WAIT_US {
        if ready() {
            return true;
        }
        if elapsed == WAIT_US {
            break;
        }
        delay.delay_us(1);
    }
    false
}
fn unlock(io: &mut impl Registers, delay: &mut impl DelayNs) -> Result<(), WatchdogError> {
    io.unlock();
    if wait(delay, || !io.locked()) { Ok(()) } else { Err(WatchdogError::UnlockTimeout) }
}
fn stop(io: &mut impl Registers) {
    io.clear_interrupt();
    io.enable(false);
}
fn start(io: &mut impl Registers, config: Config, delay: &mut impl DelayNs) -> Result<(), WatchdogError> {
    unlock(io, delay)?;
    stop(io);
    io.configure();
    io.load(config.load);
    io.restart();
    stop(io);
    io.mode(config.mode);
    io.clear_interrupt();
    io.enable(true);
    io.restart();
    Ok(())
}
fn snapshot(
    io: &mut impl Registers,
    clock: NonZeroU32,
    delay: &mut impl DelayNs,
) -> Result<CounterSnapshot, WatchdogError> {
    if io.locked() {
        return Err(WatchdogError::Locked);
    }
    // Policy guard, not a guessed BUSY handshake. CNT consumption releases the
    // APB-visible valid flag before the source-clock request path can rearm.
    // The 200us floor additionally covers observed stale command results.
    // u64 arithmetic also handles a declared 1Hz clock without overflow.
    let guard_us = (2_000_000u64.div_ceil(u64::from(clock.get())) as u32).max(200);
    delay.delay_us(guard_us);
    io.request_snapshot();
    if wait(delay, || io.snapshot_valid()) {
        Ok(CounterSnapshot { ticks: io.count(), clock })
    } else {
        Err(WatchdogError::SnapshotTimeout)
    }
}

fn feed(io: &mut impl Registers) -> Result<(), WatchdogError> {
    if io.locked() {
        return Err(WatchdogError::Locked);
    }
    io.restart();
    Ok(())
}

#[cfg(all(test, not(target_arch = "riscv32")))]
#[path = "wdt_ws53/tests.rs"]
mod tests;
