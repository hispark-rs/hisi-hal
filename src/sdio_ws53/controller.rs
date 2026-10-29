//! Source profile: ROM base init and function1 handler, SDK163d9150 clock patch.
//! W1C behavior still requires silicon qualification; acknowledgements below
//! use separate WO PAC command views and never read/modify a command register.
use crate::{peripherals::Sdio, soc::pac};
use core::num::NonZeroU16;
use embedded_hal::delay::DelayNs;

/// Function1 events decoded by the audited WS53 ROM handler.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum Function1Event {
    /// Host write completion notification; not a DMA drain guarantee.
    HostWriteComplete = 0,
    /// Host read completion notification; not a DMA drain guarantee.
    HostReadComplete = 1,
    /// Host read failed.
    HostReadError = 2,
    /// Host asks to read data; requires a separately owned ADMA transfer.
    HostReadStart = 3,
    /// Host asks to write data; requires a separately owned ADMA transfer.
    HostWriteStart = 4,
    /// ADMA end notification; no buffer release is implied.
    AdmaEnd = 5,
    /// ADMA interrupt notification.
    AdmaInterrupt = 6,
    /// ADMA error notification.
    AdmaError = 7,
    /// Function reset requires a separate recovery sequence.
    FunctionReset = 8,
    /// A host message is available.
    HostMessage = 9,
    /// Suspend notification.
    Suspend = 10,
    /// Resume notification.
    Resume = 11,
    /// Device message acknowledgement.
    DeviceMessageAck = 12,
    /// Host enabled Function1; ROM then sets FUNCTION1_READY.
    FunctionEnable = 13,
}

/// Complete captured function1 status; unknown bits are never discarded.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Function1Events(u32);
impl Function1Events {
    /// Decode an observation without acknowledging anything.
    pub const fn from_bits(bits: u32) -> Self {
        Self(bits)
    }
    /// Original status word.
    pub const fn bits(self) -> u32 {
        self.0
    }
    /// Whether this source event was present.
    pub const fn contains(self, event: Function1Event) -> bool {
        self.0 & (1 << event as u8) != 0
    }
    /// Unclassified upper status bits, in their original bit positions.
    pub const fn unknown_bits(self) -> u32 {
        self.0 & !0x3fff
    }
}

/// One bounded status observation. This is not an atomic multi-register read.
/// A later event may require another poll; a snapshot is not a count of events.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct EventSnapshot {
    /// Complete global status word.
    pub global: u32,
    /// Complete Function0 status word; bits2..11 have no established names.
    pub function0: u32,
    /// Complete Function1 status word.
    pub function1: Function1Events,
    /// Message read before acknowledgement when HostMessage was captured.
    pub host_message: Option<u32>,
}
impl EventSnapshot {
    fn pending(self) -> bool {
        self.global != 0 || self.function0 != 0 || self.function1.bits() != 0
    }
    fn unsupported(self) -> bool {
        // Function0 reset/voltage switch and Function1 transfer/reset operations
        // require protocols not implemented by an initialization session.
        self.global & !5 != 0 || self.function0 & !0xffc != 0 || self.function1.bits() & !0x3e00 != 0
    }
}

/// Progress of this initialization session, not full host/card enumeration.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum InitProgress {
    /// Waiting for an externally supplied SDIO clock.
    WaitingHostClock,
    /// Source profile is written; waiting for a fresh FunctionEnable event.
    WaitingFunctionEnable,
    /// A fresh enable event was acknowledged and READY read back as set.
    /// CIS, data transfer and local IRQ delivery have not been qualified.
    FunctionReady,
}

/// Retained initialization failure; none of these promises DMA drain or reset.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum InitError {
    /// Either word-sized SDIO clock gate failed readback.
    ClockGateReadback,
    /// READY was already set or pending events existed before this session.
    DirtyStart {
        /// READY observed before this session could establish freshness.
        ready: bool,
        /// Pending observations retained without acknowledgement.
        events: EventSnapshot,
    },
    /// The host clock disappeared after the source profile was applied.
    HostClockLost,
    /// Unknown, memory, reset, voltage-switch or transfer event. No event ACK
    /// or READY write is performed for the rejected snapshot.
    UnsupportedEvents(EventSnapshot),
    /// READY did not latch after a fresh FunctionEnable event.
    ReadyReadback(EventSnapshot),
    /// The single total attempt budget expired, with last known progress.
    Timeout(InitProgress),
    /// Explicit cancellation; known global sources are masked, clocks retained.
    Cancelled,
    /// The session already failed; no further MMIO is performed.
    Poisoned,
}

/// One poll's progress and captured events. Notifications are reported even if
/// they do not advance initialization. No transfer descriptor is published.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PollResult {
    /// Current readiness stage.
    pub progress: InitProgress,
    /// Events observed and acknowledged by this call.
    pub events: EventSnapshot,
}

/// Polling initialization owner. Global SDIO source gates stay masked; the
/// caller must also keep local IRQ46 masked. Drop performs no MMIO or reset.
///
/// ```compile_fail
/// let p = hisi_hal::Peripherals::take().unwrap();
/// let _ = unsafe { hisi_hal::sdio::Initialization::begin(p.SDIO) };
/// ```
/// ```compile_fail
/// let p = hisi_hal::Peripherals::take().unwrap();
/// let token = unsafe { p.SDIO.assume_application_ownership() };
/// let _ = hisi_hal::sdio::Initialization::begin(token);
/// ```
pub struct Initialization<'d> {
    token: Sdio<'d>,
    engine: Engine<PacRegisters>,
}
impl<'d> Initialization<'d> {
    /// Enable both SDK word-sized gates, mask known global sources and require
    /// a clean initial status/READY state. Failure returns the retained owner.
    /// No software reset is guessed and no old ADMA table is read or written.
    ///
    /// # Safety
    /// Own SDIO, IRQ46, both complete M_CLKEN0/1 word transactions, configured
    /// pads, reset and power. Serialize clock RMW against other cores, IRQs and
    /// halfword users. Keep the upstream source/MMIO accessible and local IRQ46
    /// masked. Stop and physically drain all previous SDIO ADMA users before
    /// handoff, independently of MDMA. Coordinate the external host so it sends
    /// no data transfers during this session; this API has no DMA buffers or
    /// CIS implementation. Establish the fresh hardware state externally;
    /// neither the generic ownership token nor clearing software queues proves
    /// it. The host supplies the clock and later enables Function1.
    ///
    /// The source profile advertises SDK capabilities, not implemented Rust
    /// transfer support. READY alone does not complete host enumeration. Keep
    /// host/clock/IRQ obligations after Drop or failure until external recovery
    /// or handoff. This API does not switch voltage, reset, gate off or drain.
    pub unsafe fn begin(token: Sdio<'d>) -> Result<Self, (InitError, Self)> {
        let mut session = Self { token, engine: Engine::new(PacRegisters) };
        match session.engine.begin() {
            Ok(()) => Ok(session),
            Err(error) => Err((error, session)),
        }
    }
    /// Perform one bounded step. Known notifications are returned; unknown or
    /// unsupported events fail without ACK. After any failure, future calls
    /// return Poisoned without touching registers. No local IRQ is enabled.
    pub fn poll(&mut self) -> Result<PollResult, InitError> {
        self.engine.poll()
    }
    /// Wait using one total poll budget across both stages, with 1us delays
    /// between unfinished steps. This is an attempt bound, not a wall-clock
    /// deadline. The delay must progress independently of this peripheral.
    /// Each observed notification is delivered to `event` before another poll.
    /// Timeout poisons the session and retains the token and clock state.
    pub fn wait_bounded(
        &mut self,
        attempts: NonZeroU16,
        delay: &mut impl DelayNs,
        event: impl FnMut(EventSnapshot),
    ) -> Result<(), InitError> {
        self.engine.wait(attempts, delay, event)
    }
    /// Cancel once; mask known global sources without claiming to stop DMA.
    /// Repeated cancellation is a no-op and performs no additional MMIO.
    pub fn cancel(&mut self) {
        if !self.engine.poisoned {
            self.engine.fail(InitError::Cancelled);
        }
    }
    /// Hand back the retained controller token without reset, gating or drain.
    /// The receiver inherits the host/IRQ/clock obligations and pending state.
    pub fn release_without_stopping(self) -> Sdio<'d> {
        self.token
    }
}

trait Registers {
    fn enable_clocks(&mut self);
    fn clocks_enabled(&mut self) -> bool;
    fn mask_global(&mut self);
    fn ready(&mut self) -> bool;
    fn host_clock(&mut self) -> bool;
    fn snapshot(&mut self) -> EventSnapshot;
    fn configure(&mut self);
    fn acknowledge(&mut self, events: EventSnapshot);
    fn set_ready(&mut self);
}
struct Engine<R> {
    io: R,
    progress: InitProgress,
    poisoned: bool,
}
impl<R: Registers> Engine<R> {
    fn new(io: R) -> Self {
        Self { io, progress: InitProgress::WaitingHostClock, poisoned: false }
    }
    fn fail(&mut self, error: InitError) -> InitError {
        self.io.mask_global();
        self.poisoned = true;
        error
    }
    fn begin(&mut self) -> Result<(), InitError> {
        self.io.enable_clocks();
        if !self.io.clocks_enabled() {
            // Do not touch the SDIO window if its bus gate did not latch.
            self.poisoned = true;
            return Err(InitError::ClockGateReadback);
        }
        self.io.mask_global();
        let ready = self.io.ready();
        let events = self.io.snapshot();
        if ready || events.pending() {
            return Err(self.fail(InitError::DirtyStart { ready, events }));
        }
        Ok(())
    }
    fn poll(&mut self) -> Result<PollResult, InitError> {
        if self.poisoned {
            return Err(InitError::Poisoned);
        }
        if !self.io.clocks_enabled() {
            self.poisoned = true;
            return Err(InitError::ClockGateReadback);
        }
        let host_clock = self.io.host_clock();
        if self.progress != InitProgress::WaitingHostClock && !host_clock {
            return Err(self.fail(InitError::HostClockLost));
        }
        let events = self.io.snapshot();
        if events.unsupported() {
            return Err(self.fail(InitError::UnsupportedEvents(events)));
        }
        if self.progress == InitProgress::WaitingHostClock {
            // No event may become evidence of enable before our profile exists.
            if events.pending() {
                let ready = self.io.ready();
                return Err(self.fail(InitError::DirtyStart { ready, events }));
            }
            if host_clock {
                self.io.configure();
                self.progress = InitProgress::WaitingFunctionEnable;
            }
        } else {
            self.io.acknowledge(events);
            if events.function1.contains(Function1Event::FunctionEnable) {
                self.io.set_ready();
                if !self.io.ready() {
                    return Err(self.fail(InitError::ReadyReadback(events)));
                }
                self.progress = InitProgress::FunctionReady;
            }
        }
        Ok(PollResult { progress: self.progress, events })
    }
    fn wait(
        &mut self,
        attempts: NonZeroU16,
        delay: &mut impl DelayNs,
        mut event: impl FnMut(EventSnapshot),
    ) -> Result<(), InitError> {
        for attempt in 0..attempts.get() {
            let result = self.poll()?;
            if result.events.pending() {
                event(result.events);
            }
            if result.progress == InitProgress::FunctionReady {
                return Ok(());
            }
            if attempt + 1 < attempts.get() {
                delay.delay_us(1);
            }
        }
        Err(self.fail(InitError::Timeout(self.progress)))
    }
}

struct PacRegisters;
impl Registers for PacRegisters {
    fn enable_clocks(&mut self) {
        let c = unsafe { &*pac::MCrg::PTR };
        c.m_clken0_word().modify(|_, w| w.sdio_device_enable().set_bit());
        c.m_clken1_word().modify(|_, w| w.sdio_ahb_enable().set_bit());
    }
    fn clocks_enabled(&mut self) -> bool {
        let c = unsafe { &*pac::MCrg::PTR };
        c.m_clken0_word().read().sdio_device_enable().bit() && c.m_clken1_word().read().sdio_ahb_enable().bit()
    }
    fn mask_global(&mut self) {
        unsafe { &*pac::Sdio::PTR }
            .global_interrupt_enable()
            .modify(|_, w| w.function0().clear_bit().memory().clear_bit().function1().clear_bit());
    }
    fn ready(&mut self) -> bool {
        unsafe { &*pac::Sdio::PTR }.function1_ready().read().ready().bit()
    }
    fn host_clock(&mut self) -> bool {
        unsafe { &*pac::Sdio::PTR }.host_clock().read().ready().bit()
    }
    fn snapshot(&mut self) -> EventSnapshot {
        let r = unsafe { &*pac::Sdio::PTR };
        let global = r.global_interrupt_status().read().bits();
        let function0 = r.function0_interrupt_status().read().value().bits();
        let function1 = Function1Events::from_bits(r.function1_interrupt_status().read().bits());
        let host_message =
            function1.contains(Function1Event::HostMessage).then(|| r.host_message().read().value().bits());
        EventSnapshot { global, function0, function1, host_message }
    }
    fn configure(&mut self) {
        configure_profile(unsafe { &*pac::Sdio::PTR });
    }
    fn acknowledge(&mut self, events: EventSnapshot) {
        let r = unsafe { &*pac::Sdio::PTR };
        unsafe {
            // Source order: global snapshot first, then child status snapshots.
            // A later event is left for another bounded call. No command RMW.
            if events.global != 0 {
                r.global_interrupt_ack().write_with_zero(|w| w.value().bits(events.global));
            }
            if events.function0 != 0 {
                r.function0_interrupt_ack().write_with_zero(|w| w.value().bits(events.function0));
            }
            if events.function1.bits() != 0 {
                r.function1_interrupt_ack().write_with_zero(|w| w.value().bits(events.function1.bits()));
            }
        }
    }
    fn set_ready(&mut self) {
        unsafe { &*pac::Sdio::PTR }.function1_ready().modify(|_, w| w.ready().set_bit());
    }
}

// Verified ROM base profile. Global IRQ sources intentionally stay masked for
// polling; CONFIG_30 bit0 is the ROM's post-profile action. No ADMA table write.
fn configure_profile(r: &pac::sdio::RegisterBlock) {
    unsafe {
        r.cccr_capability().write_with_zero(|w| {
            w.cccr_revision()
                .bits(3)
                .sdio_revision()
                .bits(4)
                .sd_revision()
                .bits(3)
                .continuous_spi_interrupt()
                .clear_bit()
                .direct_command()
                .set_bit()
                .multiblock()
                .set_bit()
                .read_wait()
                .clear_bit()
                .suspend_resume()
                .set_bit()
                .interrupt_4bit()
                .set_bit()
                .low_speed()
                .clear_bit()
                .low_speed_4bit()
                .clear_bit()
                .master_power_control()
                .set_bit()
                .high_speed()
                .set_bit()
                .card_type()
                .bits(2)
        });
        r.function1_config().modify(|_, w| w.init_profile().bits(0xff8000));
        r.function0_interrupt_enable().modify(|_, w| w.events().bits(0x7ff).event11().set_bit());
        r.function1_interrupt_enable().modify(|_, w| {
            w.host_write_complete()
                .set_bit()
                .host_read_complete()
                .set_bit()
                .host_read_error()
                .set_bit()
                .host_read_start()
                .set_bit()
                .host_write_start()
                .set_bit()
                .adma_end()
                .set_bit()
                .adma_interrupt()
                .set_bit()
                .adma_error()
                .set_bit()
                .function_reset()
                .set_bit()
                .host_message()
                .set_bit()
                .suspend()
                .set_bit()
                .resume()
                .set_bit()
                .device_message_ack()
                .set_bit()
                .function_enable()
                .set_bit()
        });
        r.function1_fbr().modify(|_, w| {
            w.io_device_code().bits(7).csa_support().clear_bit().extended_io_device_code().bits(0).sps().clear_bit()
        });
        r.mode().modify(|_, w| w.init_profile().bits(0x7f));
        r.voltage_control().modify(|_, w| w.init_bit().set_bit());
        // The source performs a read before this complete opaque configuration write.
        let _ = r.config_2c().read();
        r.config_2c().write_with_zero(|w| w.value().bits(0x2f92));
        r.config_30().modify(|_, w| w.init_bit().set_bit());
    }
}

#[cfg(all(test, not(target_arch = "riscv32")))]
#[path = "controller_tests.rs"]
mod tests;
