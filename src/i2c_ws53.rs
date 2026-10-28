//! WS53 application I2C v151 master, SDK source 163d9150 / RDL 01dcce5.
//!
//! Blocking transactions own the entire controller, IRQ, DMA requests and two
//! configured routes. No scan/probe is performed. Polling and delays never hold
//! a global critical section. Selective-clear helpers in the SDK are malformed;
//! only its write-one clear-all command is used. The QEMU source profile exercises
//! real PAC access; timing and recovery still need silicon qualification.
//! This module is experimental.
use crate::interrupt;
use crate::peripherals::{I2c0, I2c1, Interrupt};
use crate::pinctrl::PadControl;
use crate::soc::pac;
use core::{marker::PhantomData, num::NonZeroU16};
use embedded_hal::{
    delay::DelayNs,
    i2c::{ErrorKind, NoAcknowledgeSource, Operation},
};

/// Selected SDK clock profile: the caller establishes the upstream 32 MHz.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Timing {
    requested_hz: u32,
    high: u16,
    low: u16,
    mode: u8,
}
impl Timing {
    /// Source count formula with wide arithmetic and no silent field truncation.
    /// Accepts representable rates within the source's 1 MHz limit. Rejects
    /// requests below the count minima instead of the SDK's silent clamp (thus
    /// 1 MHz itself is not representable in this 32 MHz source profile).
    /// Mode3/Fast-plus uses HS count slots, not a measured bus frequency.
    pub const fn from_hz(hz: u32) -> Option<Self> {
        if hz == 0 || hz > 1_000_000 {
            return None;
        }
        let (mode, min_low) = if hz <= 100_000 {
            (1, 13)
        } else if hz <= 400_000 {
            (2, 16)
        } else {
            (3, 22)
        };
        let total = (32_000_000u64 + hz as u64) / hz as u64;
        if total < min_low + 14 {
            return None;
        }
        let extra = total - (min_low + 14);
        let low = min_low + extra / 2 - 1;
        let high = 14 + extra - extra / 2 - 8;
        if high > u16::MAX as u64 || low > u16::MAX as u64 {
            return None;
        }
        Some(Self { requested_hz: hz, high: high as u16, low: low as u16, mode })
    }
    /// Requested rate, not a measured result (source counts include fixed offsets).
    pub const fn requested_hz(self) -> u32 {
        self.requested_hz
    }
    /// Exact source-profile high/low register counts.
    pub const fn counts(self) -> (u16, u16) {
        (self.high, self.low)
    }
}
/// Validated addressing mode; even a small ten-bit address remains ten-bit.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Address {
    value: u16,
    ten: bool,
}
impl Address {
    pub(crate) const fn parts(self) -> (u16, bool) {
        (self.value, self.ten)
    }
    /// A seven-bit address; reserved addresses are not silently remapped.
    pub const fn seven(value: u8) -> Option<Self> {
        if value < 128 { Some(Self { value: value as u16, ten: false }) } else { None }
    }
    /// A ten-bit address, including values below 128.
    pub const fn ten(value: u16) -> Option<Self> {
        if value < 1024 { Some(Self { value, ten: true }) } else { None }
    }
}
/// Configuration and per-progress polling bound. Each unsuccessful poll waits
/// one microsecond except the final attempt. This is not a wall-clock deadline.
#[derive(Clone, Copy, Debug)]
pub struct Config {
    timing: Timing,
    polls: NonZeroU16,
}
impl Config {
    /// Use validated source-profile timing and a nonzero bounded poll count.
    pub const fn new(timing: Timing, polls: NonZeroU16) -> Self {
        Self { timing, polls }
    }
}
/// Where progress could not be observed.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Wait {
    /// Disable status and bus activity have not both cleared.
    Disable,
    /// Enable has not reached the controller clock domain.
    Enable,
    /// Transmit FIFO has no room for a command.
    Transmit,
    /// A queued read has not produced a receive entry.
    Receive,
    /// Final STOP, empty transmit FIFO and inactivity have not all arrived.
    Stop,
}
/// Original hardware failure, retained even if cleanup also times out.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Failure {
    /// No progress within the configured poll budget.
    Timeout(Wait),
    /// Raw source and detailed abort words; multiple reasons may coexist.
    Abort {
        /// IC_TX_ABRT_SOURCE word, including any unrecognized bits.
        source: u32,
        /// IC_TX_ABRT_SLV_INTX detailed reason word.
        detail: u32,
    },
    /// RX under/overflow or TX overflow bits from raw interrupt status.
    Fifo(u16),
}
/// Transaction/configuration failure. A failed cleanup requires `recover()`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Error {
    /// Trait address exceeds the selected seven/ten-bit range; no MMIO occurred.
    InvalidAddress,
    /// Initialization or recovery must succeed before another transfer.
    Poisoned,
    /// Operation failed. Transaction cleanup succeeded; for initialization or
    /// explicit recovery/shutdown failures consult `is_poisoned()` before reuse.
    Failure(Failure),
    /// Cleanup could not verify inactivity; original transfer error is retained.
    RecoveryFailed(Failure),
}
impl embedded_hal::i2c::Error for Error {
    fn kind(&self) -> ErrorKind {
        match *self {
            Self::Failure(Failure::Abort { detail, .. }) | Self::RecoveryFailed(Failure::Abort { detail, .. }) => {
                if detail & (1 << 12) != 0 {
                    ErrorKind::ArbitrationLoss
                } else if detail & 7 != 0 {
                    ErrorKind::NoAcknowledge(NoAcknowledgeSource::Address)
                } else if detail & 8 != 0 {
                    ErrorKind::NoAcknowledge(NoAcknowledgeSource::Data)
                } else {
                    ErrorKind::Other
                }
            }
            Self::Failure(Failure::Fifo(bits)) | Self::RecoveryFailed(Failure::Fifo(bits)) if bits & 0xa != 0 => {
                ErrorKind::Overrun
            }
            _ => ErrorKind::Other,
        }
    }
}
/// Exclusive SCL/SDA pad permissions for one typed controller instance.
pub struct Pins<'d, T> {
    scl: PadControl<'d>,
    sda: PadControl<'d>,
    _instance: PhantomData<T>,
}
impl<'d, T> Pins<'d, T> {
    /// Hand over externally configured routes without touching pad registers.
    ///
    /// # Safety
    /// Both distinct pads and every alias must be exclusively owned and route
    /// SCL/SDA for T, with open-drain outputs, input buffers, valid pull-ups and
    /// supply/drive settings. No old CPU/DMA/peripheral may drive them. This
    /// does not infer board MIO/AIO labels from MGPIO/AGPIO/SDK numbers. Keep
    /// routing and clocks valid until verified shutdown; avoid external conflicts.
    pub unsafe fn from_configured(scl: PadControl<'d>, sda: PadControl<'d>) -> Self {
        Self { scl, sda, _instance: PhantomData }
    }
    /// Return pads after obtaining this owner from successful `try_free()`.
    pub fn into_pads(self) -> (PadControl<'d>, PadControl<'d>) {
        (self.scl, self.sda)
    }
}
#[derive(Clone, Copy)]
enum Bus {
    Zero,
    One,
}
impl Bus {
    fn irq(self) -> Interrupt {
        match self {
            Self::Zero => Interrupt::I2C_0,
            Self::One => Interrupt::I2C_1,
        }
    }
}
/// Blocking controller. Constructor failure retains the driver/resources in the
/// error tuple; recovery can be retried without re-claiming pads or peripheral.
///
/// ```compile_fail
/// use hisi_hal::{i2c::{I2c,Config,Pins}, peripherals::{I2c0,Unclaimed}};
/// fn no_handoff(p:Unclaimed<I2c0<'static>>, pins:Pins<'static,I2c0<'static>>, c:Config, d:impl embedded_hal::delay::DelayNs) {
///     let _ = I2c::new_i2c0(p,pins,c,d);
/// }
/// ```
///
/// ```compile_fail
/// use hisi_hal::{i2c::{I2c,Config,Pins}, peripherals::{I2c0,I2c1}};
/// fn wrong_bus(p:I2c0<'static>, pins:Pins<'static,I2c1<'static>>, c:Config, d:impl embedded_hal::delay::DelayNs) {
///     let _ = I2c::new_i2c0(p,pins,c,d);
/// }
/// ```
pub struct I2c<'d, T, D: DelayNs> {
    token: Option<T>,
    pins: Option<Pins<'d, T>>,
    engine: Engine<PacRegisters, D>,
}
macro_rules! constructors {
    ($token:ident,$ctor:ident,$bus:ident) => {
        impl<'d, D: DelayNs> I2c<'d, $token<'d>, D> {
            /// Acquire the handed-over instance, enable its gate, disable IRQ/DMA,
            /// and verify inactivity before configuring. Failure returns ownership.
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
constructors!(I2c0, new_i2c0, Zero);
constructors!(I2c1, new_i2c1, One);
impl<'d, T, D: DelayNs> I2c<'d, T, D> {
    /// Execute one bus transaction. Adjacent equal-direction operations are
    /// concatenated; direction changes use repeated START, with one final STOP.
    /// Empty buffers produce no wire commands; an all-empty list is a no-op.
    pub fn transaction(&mut self, address: Address, ops: &mut [Operation<'_>]) -> Result<(), Error> {
        self.engine.transaction(address, ops)
    }
    /// Whether recovery/initialization has not yet proved the controller inactive.
    pub fn is_poisoned(&self) -> bool {
        self.engine.state == State::Poisoned
    }
    /// Retry bounded disable/status confirmation and reconfigure. This does not
    /// toggle pins, generate recovery clocks, reset shared domains or scan a bus.
    pub fn recover(&mut self) -> Result<(), Error> {
        self.engine.recover()
    }
    /// Release ownership only after confirmed shutdown; otherwise return the
    /// still-owned driver. The independent delay provider is dropped on success.
    #[allow(clippy::type_complexity)]
    pub fn try_free(mut self) -> Result<(T, Pins<'d, T>), (Error, Self)> {
        match self.engine.close() {
            Ok(()) => Ok((self.token.take().unwrap(), self.pins.take().unwrap())),
            Err(e) => Err((e, self)),
        }
    }
}
impl<T, D: DelayNs> Drop for I2c<'_, T, D> {
    fn drop(&mut self) {
        // Bounded best effort only. If inactive cannot be proven, leave the gate
        // on for external recovery; never claim that a timed-out Drop quiesced it.
        let _ = self.engine.close();
    }
}
impl<T, D: DelayNs> embedded_hal::i2c::ErrorType for I2c<'_, T, D> {
    type Error = Error;
}
impl<T, D: DelayNs> embedded_hal::i2c::I2c<u8> for I2c<'_, T, D> {
    fn transaction(&mut self, address: u8, ops: &mut [Operation<'_>]) -> Result<(), Error> {
        self.transaction(Address::seven(address).ok_or(Error::InvalidAddress)?, ops)
    }
}
impl<T, D: DelayNs> embedded_hal::i2c::I2c<u16> for I2c<'_, T, D> {
    fn transaction(&mut self, address: u16, ops: &mut [Operation<'_>]) -> Result<(), Error> {
        self.transaction(Address::ten(address).ok_or(Error::InvalidAddress)?, ops)
    }
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum State {
    Ready,
    Poisoned,
    Closed,
}
#[derive(Default, Clone, Copy)]
struct Status {
    enabled: bool,
    active: bool,
    tx_ready: bool,
    tx_empty: bool,
    rx_ready: bool,
    stop: bool,
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct Command {
    data: u8,
    read: bool,
    restart: bool,
    stop: bool,
}
trait Registers {
    fn clock(&mut self, on: bool);
    fn isolate(&mut self);
    fn enable(&mut self, on: bool);
    fn status(&mut self) -> Status;
    fn fault(&mut self) -> Option<Failure>;
    fn clear(&mut self);
    fn configure(&mut self, timing: Timing);
    fn target(&mut self, address: Address);
    fn send(&mut self, command: Command);
    fn receive(&mut self) -> u8;
}
struct Engine<R: Registers, D: DelayNs> {
    io: R,
    delay: D,
    config: Config,
    state: State,
}
impl<R: Registers, D: DelayNs> Engine<R, D> {
    fn new(io: R, delay: D, config: Config) -> Self {
        Self { io, delay, config, state: State::Poisoned }
    }
    fn poll(&mut self, stage: Wait, check_fault: bool, ready: impl Fn(Status) -> bool) -> Result<(), Failure> {
        for n in 0..self.config.polls.get() {
            if check_fault && let Some(f) = self.io.fault() {
                return Err(f);
            }
            if ready(self.io.status()) {
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
        self.poll(Wait::Disable, false, |s| !s.enabled && !s.active)?;
        Ok(())
    }
    fn recover(&mut self) -> Result<(), Error> {
        self.io.clock(true);
        self.quiesce().map_err(Error::Failure)?;
        self.io.clear();
        self.io.configure(self.config.timing);
        self.state = State::Ready;
        Ok(())
    }
    fn close(&mut self) -> Result<(), Error> {
        if self.state == State::Closed {
            return Ok(());
        }
        self.quiesce().map_err(Error::Failure)?;
        self.io.clear();
        self.io.clock(false);
        self.state = State::Closed;
        Ok(())
    }
    fn transaction(&mut self, address: Address, ops: &mut [Operation<'_>]) -> Result<(), Error> {
        if self.state != State::Ready {
            return Err(Error::Poisoned);
        }
        let last = ops.iter().rposition(|op| match op {
            Operation::Read(b) => !b.is_empty(),
            Operation::Write(b) => !b.is_empty(),
        });
        let Some(last) = last else {
            return Ok(());
        };
        // Ready always means disabled and quiesced, including after success/error.
        self.state = State::Poisoned;
        self.io.target(address);
        self.io.clear();
        self.io.enable(true);
        let mut result = self.transfer(ops, last);
        // Sample again after the final status read, before disable can change
        // error visibility. The first poll may precede the final bus event.
        if result.is_ok()
            && let Some(f) = self.io.fault()
        {
            result = Err(f);
        }
        let cleanup = self.quiesce();
        // Once disabled, no command can create a new abort. Preserve a failure
        // arriving between the last STOP poll and disable, before clearing it.
        if cleanup.is_ok() {
            if result.is_ok()
                && let Some(f) = self.io.fault()
            {
                result = Err(f);
            }
            self.io.clear();
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
            (Err(f), Err(_)) => Err(Error::RecoveryFailed(f)),
            (Ok(()), Err(f)) => Err(Error::RecoveryFailed(f)),
        }
    }
    fn transfer(&mut self, ops: &mut [Operation<'_>], last: usize) -> Result<(), Failure> {
        self.poll(Wait::Enable, true, |s| s.enabled)?;
        let mut previous = None;
        for (index, op) in ops.iter_mut().enumerate().take(last + 1) {
            let read = matches!(op, Operation::Read(_));
            let len = match op {
                Operation::Read(b) => b.len(),
                Operation::Write(b) => b.len(),
            };
            for byte in 0..len {
                self.poll(Wait::Transmit, true, |s| s.tx_ready)?;
                let data = match op {
                    Operation::Write(b) => b[byte],
                    Operation::Read(_) => 0,
                };
                self.io.send(Command {
                    data,
                    read,
                    restart: byte == 0 && previous.is_some_and(|p| p != read),
                    stop: index == last && byte + 1 == len,
                });
                if let Operation::Read(buffer) = op {
                    // At most one outstanding read: never enqueue more than the
                    // receive FIFO can hold, including across adjacent read ops.
                    self.poll(Wait::Receive, true, |s| s.rx_ready)?;
                    buffer[byte] = self.io.receive();
                }
                previous = Some(read);
            }
        }
        self.poll(Wait::Stop, true, |s| s.stop && s.tx_empty && !s.active)
    }
}

struct PacRegisters(Bus);
macro_rules! with_regs {
    ($bus:expr,$r:ident,$body:block) => {
        match $bus {
            Bus::Zero => {
                let $r = unsafe { &*I2c0::ptr() };
                $body
            }
            Bus::One => {
                let $r = unsafe { &*I2c1::ptr() };
                $body
            }
        }
    };
}
impl Registers for PacRegisters {
    fn clock(&mut self, on: bool) {
        let r = unsafe { &*pac::MCrg::PTR };
        match self.0 {
            Bus::Zero => r.m_clken1().modify(|_, w| w.i2c0_enable().bit(on)),
            Bus::One => r.m_clken1().modify(|_, w| w.i2c1_enable().bit(on)),
        };
    }
    fn isolate(&mut self) {
        unsafe {
            interrupt::disable(self.0.irq());
        }
        with_regs!(self.0, r, {
            // Disable every interrupt/DMA request owned by this instance.
            unsafe {
                r.ic_intr_mask().write_with_zero(|w| w);
            }
            unsafe {
                r.ic_dma_cr().write_with_zero(|w| w.rdmae().clear_bit().tdmae().clear_bit());
            }
        });
    }
    fn enable(&mut self, on: bool) {
        with_regs!(self.0, r, {
            unsafe {
                r.ic_enable().write_with_zero(|w| w.enable().bit(on));
            }
        });
    }
    fn status(&mut self) -> Status {
        with_regs!(self.0, r, {
            let s = r.ic_status().read();
            Status {
                enabled: r.ic_enable_status().read().ic_en().bit(),
                active: s.activity().bit(),
                tx_ready: s.tfnf().bit(),
                tx_empty: s.tfe().bit(),
                rx_ready: s.rfne().bit(),
                stop: r.ic_raw_intr_stat().read().stop_det().bit(),
            }
        })
    }
    fn fault(&mut self) -> Option<Failure> {
        with_regs!(self.0, r, {
            let s = r.ic_raw_intr_stat().read();
            if s.tx_abrt().bit() {
                Some(Failure::Abort {
                    source: r.ic_tx_abrt_source().read().bits(),
                    detail: r.ic_tx_abrt_slv_intx().read().bits(),
                })
            } else {
                let fifo = (s.bits() & 0xb) as u16;
                if fifo != 0 { Some(Failure::Fifo(fifo)) } else { None }
            }
        })
    }
    fn clear(&mut self) {
        with_regs!(self.0, r, {
            unsafe {
                r.ic_clr_int().write_with_zero(|w| w.clr_int().set_bit());
            }
        });
        interrupt::clear_pending(self.0.irq());
    }
    fn configure(&mut self, t: Timing) {
        with_regs!(self.0, r, {
            r.ic_con().modify(|_, w| unsafe {
                w.master_mode()
                    .set_bit()
                    .ic_slave_disable()
                    .set_bit()
                    .ic_restart_en()
                    .set_bit()
                    .ic_10bitaddr_master()
                    .clear_bit()
                    .rx_fifo_full_hld_ctrl()
                    .set_bit()
                    .tx_empty_ctrl()
                    .clear_bit()
                    .speed()
                    .bits(t.mode)
            });
            match t.mode {
                1 => {
                    r.ic_ss_scl_hcnt().modify(|_, w| unsafe { w.ic_ss_scl_hcnt().bits(t.high) });
                    r.ic_ss_scl_lcnt().modify(|_, w| unsafe { w.ic_ss_scl_lcnt().bits(t.low) });
                }
                2 => {
                    r.ic_fs_scl_hcnt().modify(|_, w| unsafe { w.ic_fs_scl_hcnt().bits(t.high) });
                    r.ic_fs_scl_lcnt().modify(|_, w| unsafe { w.ic_fs_scl_lcnt().bits(t.low) });
                }
                _ => {
                    r.ic_hs_scl_hcnt().modify(|_, w| unsafe { w.ic_hs_scl_hcnt().bits(t.high) });
                    r.ic_hs_scl_lcnt().modify(|_, w| unsafe { w.ic_hs_scl_lcnt().bits(t.low) });
                }
            }
            r.ic_rx_tl().modify(|_, w| unsafe { w.rx_tl().bits(0) });
            r.ic_tx_tl().modify(|_, w| unsafe { w.tx_tl().bits(0) });
        });
    }
    fn target(&mut self, a: Address) {
        with_regs!(self.0, r, {
            r.ic_con().modify(|_, w| w.ic_10bitaddr_master().bit(a.ten));
            // Match the source's disabled-controller CON addressing selection.
            // Dynamic TAR update capability is not established for this chip.
            r.ic_tar().modify(|_, w| unsafe {
                w.ic_tar()
                    .bits(a.value)
                    .special()
                    .clear_bit()
                    .gc_or_start()
                    .clear_bit()
                    .device_id()
                    .clear_bit()
                    .smbus_quick_cmd()
                    .clear_bit()
            });
        });
    }
    fn send(&mut self, c: Command) {
        with_regs!(self.0, r, {
            unsafe {
                r.ic_data_write().write_with_zero(|w| {
                    w.dat().bits(c.data).cmd().bit(c.read).restart().bit(c.restart).stop().bit(c.stop)
                });
            }
        });
    }
    fn receive(&mut self) -> u8 {
        with_regs!(self.0, r, { r.ic_data_read().read().dat().bits() })
    }
}
#[cfg(all(test, not(target_arch = "riscv32")))]
#[path = "i2c_ws53/tests.rs"]
mod tests;
