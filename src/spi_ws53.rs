//! WS53 v151_100 SPI0/QSPI1 single-line masters, SDK 163d9150 / RDL 04ced9c.
//! Separate PAC layouts share a bounded transaction engine. No physical pin
//! mapping, native-CS continuity, FIFO depth or silicon timing is assumed.
//! Configured-master DMA is separate in [`crate::dma::spi`]. IRQ-driven,
//! dual/quad protocols and clock shutdown are not implemented here.
use crate::{
    interrupt,
    peripherals::{Interrupt, Qspi1, Spi0},
    pinctrl::PadControl,
    soc::pac,
};
use core::{marker::PhantomData, num::NonZeroU16};
use embedded_hal::delay::DelayNs;
pub use embedded_hal::spi::{MODE_0, MODE_1, MODE_2, MODE_3, Mode, Phase, Polarity};

/// Validated even divider of the handed-over 32 MHz source.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SpiHz {
    requested: u32,
    divider: u16,
}
impl SpiHz {
    /// Choose the smallest legal even divider whose exact source-profile rate
    /// does not exceed the request. Reject requests outside the source range;
    /// no truncation or minimum-divider clamp. This is not a measured rate.
    pub const fn try_from_hz(hz: u32) -> Option<Self> {
        if hz == 0 || hz > 16_000_000 {
            return None;
        }
        let ceil = 32_000_000u64.div_ceil(hz as u64);
        let even = (ceil + 1) & !1;
        if even > 65534 {
            return None;
        }
        Some(Self { requested: hz, divider: even as u16 })
    }
    /// Requested maximum rate in Hz.
    pub const fn hz(self) -> u32 {
        self.requested
    }
    /// Exact register divisor; always even and within 2..65534.
    pub const fn divider(self) -> u16 {
        self.divider
    }
    /// Source-profile rate rounded down to integral Hz, not a physical measurement.
    /// The exact fraction is 32_000_000 / divider().
    pub const fn actual_hz(self) -> u32 {
        32_000_000 / self.divider as u32
    }
}
/// Master configuration. Poll count is a per-progress bound, not a wall deadline.
#[derive(Clone, Copy, Debug)]
pub struct Config {
    frequency: SpiHz,
    mode: Mode,
    polls: NonZeroU16,
    rx_delay: u8,
    tx_edge: u8,
}
impl Config {
    /// Configure Motorola single-line, MSB-first full duplex; sample delays start at zero.
    pub const fn new(frequency: SpiHz, mode: Mode, polls: NonZeroU16) -> Self {
        Self { frequency, mode, polls, rx_delay: 0, tx_edge: 0 }
    }
    /// Supply the source-defined eight-bit sample-delay/drive-edge fields.
    /// Their electrical timing must be qualified for the board and bus rate.
    pub const fn with_sample_timing(mut self, rx_delay: u8, tx_edge: u8) -> Self {
        self.rx_delay = rx_delay;
        self.tx_edge = tx_edge;
        self
    }
}
mod sealed {
    pub trait Sealed {}
    impl Sealed for u8 {}
    impl Sealed for u32 {}
}
/// Word types enabled by the WS53 source profile: u8 or u32 only.
pub trait Word: sealed::Sealed + Copy + Default + 'static {
    /// Data frame size, encoded as bits minus one.
    const DFS: u8;
    /// Lossless conversion to one FIFO entry.
    fn into_fifo(self) -> u32;
    /// Read a FIFO entry using the configured word width.
    fn from_fifo(value: u32) -> Self;
}
impl Word for u8 {
    const DFS: u8 = 7;
    fn into_fifo(self) -> u32 {
        self.into()
    }
    fn from_fifo(value: u32) -> Self {
        value as u8
    }
}
impl Word for u32 {
    const DFS: u8 = 31;
    fn into_fifo(self) -> u32 {
        self
    }
    fn from_fifo(value: u32) -> Self {
        value
    }
}
/// Progress point that exhausted the configured polling budget.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Wait {
    /// Disabled, not busy and transmit FIFO empty were not all observed.
    Disable,
    /// Enable register did not read back set (not an independent clock acknowledgement).
    Enable,
    /// Transmit FIFO has no space.
    Transmit,
    /// The queued word has not produced a receive entry.
    Receive,
    /// Shift engine or transmit queue did not become idle.
    Idle,
    /// Disabled receive FIFO could not be drained within the bound.
    Drain,
}
/// Original operation failure; raw interrupt bits retain instance-specific positions.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Failure {
    /// Exhausted progress budget.
    Timeout(Wait),
    /// Hardware-reported cause, preserving all raw interrupt bits.
    Hardware {
        /// Raw status from the appropriate instance's PAC layout.
        raw: u32,
        /// Normalized embedded-hal error category.
        kind: embedded_hal::spi::ErrorKind,
    },
    /// Master collision observed in the read-clear status register. This
    /// observation is independent of the raw interrupt latch.
    Collision {
        /// Complete status word captured before the collision bit was cleared.
        status: u32,
        /// Raw interrupt snapshot when a recognized interrupt fault was also present.
        raw_interrupt: Option<u32>,
    },
    /// Receive data existed without a corresponding word owned by this operation.
    UnexpectedReceive,
}
/// Driver failure; failed shutdown keeps the instance poisoned until recovery.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Error {
    /// No successful initialization/recovery or shutdown verification.
    Poisoned,
    /// Operation failed; transaction cleanup succeeded. For explicit recovery or
    /// close failures, consult is_poisoned() before reuse.
    Failure(Failure),
    /// Cleanup failed too; original failure retained, no inactive-bus promise.
    RecoveryFailed(Failure),
}
impl embedded_hal::spi::Error for Error {
    fn kind(&self) -> embedded_hal::spi::ErrorKind {
        match *self {
            Self::Failure(Failure::Hardware { kind, .. }) | Self::RecoveryFailed(Failure::Hardware { kind, .. }) => {
                kind
            }
            Self::Failure(Failure::Collision { .. }) | Self::RecoveryFailed(Failure::Collision { .. }) => {
                embedded_hal::spi::ErrorKind::ModeFault
            }
            _ => embedded_hal::spi::ErrorKind::Other,
        }
    }
}
/// Exclusive SCLK/MOSI/MISO routes for a typed controller.
/// Chip select belongs to the caller's SpiDevice wrapper or board protocol.
pub struct Pins<'d, T> {
    sclk: PadControl<'d>,
    mosi: PadControl<'d>,
    miso: PadControl<'d>,
    _instance: PhantomData<T>,
}
impl<'d, T> Pins<'d, T> {
    /// Hand over configured single-line routes without changing pad registers.
    ///
    /// # Safety
    /// Own all three distinct physical pads and aliases exclusively; route them
    /// to T with valid input/output, supply and drive settings. Native hardware
    /// CS outputs must be disconnected/unrouted: this bus driver makes no native
    /// CS continuity promise across words or calls. A separately owned software
    /// CS must frame device transactions. No other CPU/DMA/peripheral may drive
    /// these routes, and clocks/routing must remain valid until shutdown.
    pub unsafe fn from_configured(sclk: PadControl<'d>, mosi: PadControl<'d>, miso: PadControl<'d>) -> Self {
        Self { sclk, mosi, miso, _instance: PhantomData }
    }
    /// Return pads after successful driver try_free().
    pub fn into_pads(self) -> (PadControl<'d>, PadControl<'d>, PadControl<'d>) {
        (self.sclk, self.mosi, self.miso)
    }
}
#[derive(Clone, Copy)]
enum Bus {
    Spi,
    Qspi,
}
impl Bus {
    fn irq(self) -> Interrupt {
        match self {
            Self::Spi => Interrupt::SPI_3,
            Self::Qspi => Interrupt::QSPI0_2CS,
        }
    }
}

// The two PAC layouts document different read-clear master-collision bits.
pub(crate) const fn collision_status(qspi: bool, status: u32) -> Option<u32> {
    let mask = if qspi { 1 << 6 } else { 1 };
    if status & mask != 0 { Some(status) } else { None }
}
/// Blocking SpiBus with fixed u8/u32 words and caller-managed chip select.
/// Every successful call waits for all transmitted bits and drains every reply.
/// Drop only attempts bounded shutdown. On failure it cannot promise released
/// bus signals; retain the driver and use try_free/recover when proof is needed.
///
/// ```compile_fail
/// use hisi_hal::{spi::{Spi,Config,Pins}, peripherals::{Spi0,Unclaimed}};
/// fn no_handoff(p:Unclaimed<Spi0<'static>>, pins:Pins<'static,Spi0<'static>>, c:Config,d:impl embedded_hal::delay::DelayNs) {
///     let _ = Spi::<_,u8,_>::new_spi0(p,pins,c,d);
/// }
/// ```
/// ```compile_fail
/// use hisi_hal::{spi::{Spi,Config,Pins}, peripherals::{Spi0,Qspi1}};
/// fn wrong_pins(p:Spi0<'static>, pins:Pins<'static,Qspi1<'static>>, c:Config,d:impl embedded_hal::delay::DelayNs) {
///     let _ = Spi::<_,u8,_>::new_spi0(p,pins,c,d);
/// }
/// ```
/// ```compile_fail
/// use hisi_hal::spi::Word;
/// fn only_source_words<W:Word>() {}
/// only_source_words::<u16>();
/// ```
pub struct Spi<'d, T, W: Word, D: DelayNs> {
    token: Option<T>,
    pins: Option<Pins<'d, T>>,
    engine: Engine<PacRegisters, D, W>,
}
macro_rules! constructors {
    ($token:ident,$ctor:ident,$bus:ident) => {
        impl<'d, W: Word, D: DelayNs> Spi<'d, $token<'d>, W, D> {
            /// Enable port clocks, isolate IRQ/DMA, verify shutdown and configure.
            /// Failure returns the still-owned driver for bounded recovery.
            pub fn $ctor(
                token: $token<'d>,
                pins: Pins<'d, $token<'d>>,
                config: Config,
                delay: D,
            ) -> Result<Self, (Error, Self)> {
                let mut driver = Self {
                    token: Some(token),
                    pins: Some(pins),
                    engine: Engine::new(PacRegisters(Bus::$bus), delay, config),
                };
                match driver.engine.recover() {
                    Ok(()) => Ok(driver),
                    Err(e) => Err((e, driver)),
                }
            }
        }
    };
}
constructors!(Spi0, new_spi0, Spi);
constructors!(Qspi1, new_qspi1, Qspi);
impl<'d, T, W: Word, D: DelayNs> Spi<'d, T, W, D> {
    /// Whether recovery is required before reuse.
    pub fn is_poisoned(&self) -> bool {
        self.engine.state == State::Poisoned
    }
    /// Retry bounded shutdown/drain and configuration; no shared reset or pin scanning.
    pub fn recover(&mut self) -> Result<(), Error> {
        self.engine.recover()
    }
    /// Return instance and pads only after disabled/idle/empty confirmation.
    /// Clock gates remain enabled: the source's delayed sleep acknowledgement is
    /// not modeled in the PAC, and shared XIP clock shutdown is not qualified.
    #[allow(clippy::type_complexity)]
    pub fn try_free(mut self) -> Result<(T, Pins<'d, T>), (Error, Self)> {
        match self.engine.close() {
            Ok(()) => Ok((self.token.take().unwrap(), self.pins.take().unwrap())),
            Err(e) => Err((e, self)),
        }
    }
}
impl<T, W: Word, D: DelayNs> Drop for Spi<'_, T, W, D> {
    fn drop(&mut self) {
        let _ = self.engine.close();
    }
}
impl<T, W: Word, D: DelayNs> embedded_hal::spi::ErrorType for Spi<'_, T, W, D> {
    type Error = Error;
}
impl<T, W: Word, D: DelayNs> embedded_hal::spi::SpiBus<W> for Spi<'_, T, W, D> {
    fn read(&mut self, words: &mut [W]) -> Result<(), Error> {
        self.engine.transfer(words, &[])
    }
    fn write(&mut self, words: &[W]) -> Result<(), Error> {
        self.engine.transfer(&mut [], words)
    }
    fn transfer(&mut self, read: &mut [W], write: &[W]) -> Result<(), Error> {
        self.engine.transfer(read, write)
    }
    fn transfer_in_place(&mut self, words: &mut [W]) -> Result<(), Error> {
        self.engine.transfer_in_place(words)
    }
    fn flush(&mut self) -> Result<(), Error> {
        self.engine.execute(false, |_| Ok(()))
    }
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum State {
    Ready,
    Poisoned,
    Closed,
}
#[derive(Clone, Copy, Default)]
struct Status {
    enabled: bool,
    busy: bool,
    tx_ready: bool,
    tx_empty: bool,
    rx_ready: bool,
    collision: Option<u32>,
}
trait Registers {
    fn enable_clocks(&mut self);
    fn isolate(&mut self);
    fn enable(&mut self, on: bool);
    fn select(&mut self, on: bool);
    fn configure(&mut self, config: Config, dfs: u8);
    fn status(&mut self) -> Status;
    fn fault(&mut self) -> Option<Failure>;
    fn clear(&mut self);
    fn send(&mut self, word: u32);
    fn receive(&mut self) -> u32;
}
struct Engine<R: Registers, D: DelayNs, W: Word> {
    io: R,
    delay: D,
    config: Config,
    state: State,
    collision: Option<u32>,
    _word: PhantomData<W>,
}
impl<R: Registers, D: DelayNs, W: Word> Engine<R, D, W> {
    fn new(io: R, delay: D, config: Config) -> Self {
        Self { io, delay, config, state: State::Poisoned, collision: None, _word: PhantomData }
    }
    fn status(&mut self) -> Status {
        let status = self.io.status();
        if self.collision.is_none() {
            self.collision = status.collision;
        }
        status
    }
    fn fault(&mut self) -> Option<Failure> {
        if let Some(status) = self.collision.take() {
            let raw_interrupt = match self.io.fault() {
                Some(Failure::Hardware { raw, .. }) => Some(raw),
                _ => None,
            };
            Some(Failure::Collision { status, raw_interrupt })
        } else {
            self.io.fault()
        }
    }
    fn ready(&self) -> Result<(), Error> {
        if self.state == State::Ready { Ok(()) } else { Err(Error::Poisoned) }
    }
    fn poll(&mut self, stage: Wait, check_fault: bool, ready: impl Fn(Status) -> bool) -> Result<(), Failure> {
        for n in 0..self.config.polls.get() {
            if check_fault && let Some(f) = self.fault() {
                return Err(f);
            }
            let status = self.status();
            if check_fault
                && self.collision.is_some()
                && let Some(f) = self.fault()
            {
                return Err(f);
            }
            if ready(status) {
                return Ok(());
            }
            if n + 1 < self.config.polls.get() {
                self.delay.delay_us(1);
            }
        }
        Err(Failure::Timeout(stage))
    }
    fn quiesce(&mut self) -> Result<(), Failure> {
        self.state = State::Poisoned;
        self.io.isolate();
        self.io.enable(false);
        self.poll(Wait::Disable, false, |s| !s.enabled && !s.busy && s.tx_empty)?;
        self.io.select(false);
        // Do not infer FIFO capacity from the 36 address aliases. Drain is
        // bounded even if a defective/stopped peripheral never clears RFNE.
        for _ in 0..self.config.polls.get() {
            if !self.status().rx_ready {
                return Ok(());
            }
            self.io.receive();
        }
        if self.status().rx_ready { Err(Failure::Timeout(Wait::Drain)) } else { Ok(()) }
    }
    fn recover(&mut self) -> Result<(), Error> {
        self.io.enable_clocks();
        self.quiesce().map_err(Error::Failure)?;
        self.io.clear();
        self.collision = None;
        self.io.configure(self.config, W::DFS);
        self.state = State::Ready;
        Ok(())
    }
    fn close(&mut self) -> Result<(), Error> {
        if self.state == State::Closed {
            return Ok(());
        }
        self.quiesce().map_err(Error::Failure)?;
        self.io.clear();
        self.collision = None;
        self.state = State::Closed;
        Ok(())
    }
    fn exchange(&mut self, word: W) -> Result<W, Failure> {
        self.poll(Wait::Transmit, true, |s| s.tx_ready)?;
        let status = self.status();
        if let Some(f) = self.fault() {
            return Err(f);
        }
        if status.rx_ready {
            return Err(Failure::UnexpectedReceive);
        }
        self.io.send(word.into_fifo());
        self.poll(Wait::Receive, true, |s| s.rx_ready)?;
        Ok(W::from_fifo(self.io.receive()))
    }
    fn execute(&mut self, start: bool, body: impl FnOnce(&mut Self) -> Result<(), Failure>) -> Result<(), Error> {
        self.ready()?;
        self.state = State::Poisoned;
        let mut result = (|| {
            if start {
                self.io.select(true);
                self.io.enable(true);
                self.poll(Wait::Enable, true, |s| s.enabled)?;
            }
            body(self)?;
            self.poll(Wait::Idle, true, |s| !s.busy && s.tx_empty)?;
            if self.status().rx_ready {
                return Err(Failure::UnexpectedReceive);
            }
            Ok(())
        })();
        if result.is_ok()
            && let Some(f) = self.fault()
        {
            result = Err(f);
        }
        let cleanup = self.quiesce();
        if result.is_ok()
            && let Some(f) = self.fault()
        {
            result = Err(f);
        }
        if cleanup.is_ok() {
            self.io.clear();
            self.collision = None;
        }
        match (result, cleanup) {
            (Ok(()), Ok(())) => {
                self.state = State::Ready;
                Ok(())
            }
            (Err(f), Ok(())) => {
                self.state = State::Ready;
                Err(Error::Failure(f))
            }
            (Err(f), Err(_)) | (Ok(()), Err(f)) => Err(Error::RecoveryFailed(f)),
        }
    }
    fn transfer(&mut self, read: &mut [W], write: &[W]) -> Result<(), Error> {
        let n = read.len().max(write.len());
        if n == 0 {
            return self.ready();
        }
        self.execute(true, |s| {
            for i in 0..n {
                let received = s.exchange(write.get(i).copied().unwrap_or_default())?;
                if let Some(slot) = read.get_mut(i) {
                    *slot = received;
                }
            }
            Ok(())
        })
    }
    fn transfer_in_place(&mut self, words: &mut [W]) -> Result<(), Error> {
        if words.is_empty() {
            return self.ready();
        }
        self.execute(true, |s| {
            for word in words {
                *word = s.exchange(*word)?;
            }
            Ok(())
        })
    }
}
struct PacRegisters(Bus);
macro_rules! regs {
    ($bus:expr,$r:ident,$body:block) => {
        match $bus {
            Bus::Spi => {
                let $r = unsafe { &*Spi0::ptr() };
                $body
            }
            Bus::Qspi => {
                let $r = unsafe { &*Qspi1::ptr() };
                $body
            }
        }
    };
}
impl Registers for PacRegisters {
    fn enable_clocks(&mut self) {
        let r = unsafe { &*pac::MCrg::PTR };
        match self.0 {
            Bus::Spi => {
                r.spi_clock().modify(|_, w| w.enable().set_bit());
                r.m_clken1().modify(|_, w| w.spi3_ms_enable().set_bit());
                r.spi_device_mode().modify(|_, w| w.master().set_bit());
            }
            Bus::Qspi => {
                r.xip_clock_enable().modify(|_, w| {
                    w.qspi1_divider_enable().set_bit().qspi1_divider_clock_enable().set_bit().qspi1_enable().set_bit()
                });
                r.xip_qspi_clock().modify(|_, w| unsafe { w.control().bits(1) });
                r.global_clken().modify(|_, w| w.xip_qspi_enable().set_bit());
            }
        }
    }
    fn isolate(&mut self) {
        unsafe {
            interrupt::disable(self.0.irq());
        }
        regs!(self.0, r, {
            unsafe {
                r.interrupt_mask().write_with_zero(|w| w);
                r.dma_control().write_with_zero(|w| w.rx_enable().clear_bit().tx_enable().clear_bit());
            }
        });
    }
    fn enable(&mut self, on: bool) {
        regs!(self.0, r, {
            r.enable().modify(|_, w| w.start().bit(on));
        });
    }
    fn select(&mut self, on: bool) {
        regs!(self.0, r, {
            r.slave_enable().modify(|_, w| unsafe { w.value().bits(u16::from(on)) });
        });
    }
    fn configure(&mut self, c: Config, dfs: u8) {
        regs!(self.0, r, {
            r.control().modify(|_, w| unsafe {
                w.slave_output_disable()
                    .clear_bit()
                    .loopback()
                    .clear_bit()
                    .select_toggle()
                    .clear_bit()
                    .phase()
                    .bit(c.mode.phase == Phase::CaptureOnSecondTransition)
                    .polarity()
                    .bit(c.mode.polarity == Polarity::IdleHigh)
                    .control_frame_size()
                    .bits(0)
                    .frame_size16()
                    .bits(0)
                    .frame_size32()
                    .bits(dfs)
                    .transfer_mode()
                    .bits(0)
                    .frame_format()
                    .bits(0)
                    .protocol()
                    .bits(0)
            });
            r.frame_count().modify(|_, w| unsafe { w.value().bits(0) });
            r.enhanced_control().modify(|_, w| unsafe {
                w.wait_cycles()
                    .bits(0)
                    .transfer_type()
                    .bits(0)
                    .rx_strobe_enable()
                    .clear_bit()
                    .address_length()
                    .bits(0)
                    .instruction_length()
                    .bits(0)
                    .instruction_ddr()
                    .clear_bit()
                    .data_ddr()
                    .clear_bit()
            });
            r.microwire().modify(|_, w| w.handshake().clear_bit().sequential().clear_bit().transmit().clear_bit());
            r.baud().modify(|_, w| unsafe { w.value().bits(c.frequency.divider) });
            r.tx_threshold().modify(|_, w| unsafe { w.value().bits(0) });
            r.rx_threshold().modify(|_, w| unsafe { w.value().bits(0) });
            r.rx_sample_delay().modify(|_, w| unsafe { w.value().bits(c.rx_delay) });
            r.tx_drive_edge().modify(|_, w| unsafe { w.value().bits(c.tx_edge) });
        });
    }
    fn status(&mut self) -> Status {
        regs!(self.0, r, {
            let s = r.status().read();
            // SDK SPI0 WSR bit0 and QSPI1 SR bit6 are distinct read-clear
            // master-collision fields. Preserve the entire observed word.
            Status {
                enabled: r.enable().read().start().bit(),
                busy: s.busy().bit(),
                tx_ready: s.tx_not_full().bit(),
                tx_empty: s.tx_empty().bit(),
                rx_ready: s.rx_not_empty().bit(),
                collision: collision_status(matches!(self.0, Bus::Qspi), s.bits()),
            }
        })
    }
    fn fault(&mut self) -> Option<Failure> {
        regs!(self.0, r, {
            let s = r.raw_interrupt().read();
            let kind = if s.multi_master().bit() {
                embedded_hal::spi::ErrorKind::ModeFault
            } else if s.rx_overflow().bit() || s.tx_overflow().bit() {
                embedded_hal::spi::ErrorKind::Overrun
            } else if s.rx_underflow().bit() {
                embedded_hal::spi::ErrorKind::Other
            } else {
                return None;
            };
            Some(Failure::Hardware { raw: s.bits(), kind })
        })
    }
    fn clear(&mut self) {
        regs!(self.0, r, {
            let _ = r.clear_all().read();
        });
        interrupt::clear_pending(self.0.irq());
    }
    fn send(&mut self, word: u32) {
        regs!(self.0, r, {
            unsafe {
                r.tx(0).write_with_zero(|w| w.data().bits(word));
            }
        });
    }
    fn receive(&mut self) -> u32 {
        regs!(self.0, r, { r.rx(0).read().data().bits() })
    }
}
#[cfg(all(test, not(target_arch = "riscv32")))]
#[path = "spi_ws53/tests.rs"]
mod tests;
