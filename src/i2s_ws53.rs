//! WS53 v151 stereo I2S polling streams, SDK163d9150 and audited WS53 PAC.
//!
//! Queueing and FIFO-empty observations are NOT wire completion. The source has
//! no evidenced shift-register BUSY flag; no flush or safe free API is invented.
//! Cancellation resets/discards the stream, retains ownership and can truncate
//! output. Shared clock gates/reset release remain enabled. Experimental merge
//! DMA has a separate `dma::i2s` adapter; multi-channel PCM, async and IRQ handlers
//! are not implemented. Physical timing remains unverified.
use crate::{
    interrupt,
    peripherals::{I2s, Interrupt},
    pinctrl::PadControl,
    soc::pac,
};
use core::num::NonZeroU16;
use embedded_hal::delay::DelayNs;

/// SDK sample-width encodings; zero is reserved.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum DataWidth {
    /// Sixteen bits, code1.
    Bits16 = 1,
    /// Eighteen bits, code2.
    Bits18 = 2,
    /// Twenty bits, code3.
    Bits20 = 3,
    /// Twenty-four bits, code4.
    Bits24 = 4,
    /// Thirty-two bits, code5.
    Bits32 = 5,
}
impl DataWidth {
    /// Number of data bits per left/right sample.
    pub const fn bits(self) -> u8 {
        match self {
            Self::Bits16 => 16,
            Self::Bits18 => 18,
            Self::Bits20 => 20,
            Self::Bits24 => 24,
            Self::Bits32 => 32,
        }
    }
    fn contains(self, frame: Frame) -> bool {
        let mask = u32::MAX >> (32 - self.bits());
        frame.left & !mask == 0 && frame.right & !mask == 0
    }
}
/// Nonzero seven-bit BCLK divisor. Upstream frequency is a handoff requirement.
///
/// ```compile_fail
/// let zero = hisi_hal::i2s::BclkDivider(0);
/// ```
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct BclkDivider(u8);
impl BclkDivider {
    /// Reject zero or values above127; never truncate or silently round.
    pub const fn new(divider: u16) -> Option<Self> {
        if divider == 0 || divider > 127 { None } else { Some(Self(divider as u8)) }
    }
    /// Hardware divisor, not a measured output frequency.
    pub const fn get(self) -> u8 {
        self.0
    }
}
/// Clock role; only master mode accepts/requires a nonzero divider.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ClockMode {
    /// Drive BCLK/WS from the handed-over audio source.
    Master(BclkDivider),
    /// Receive BCLK/WS from the external source; no internal-divider promise.
    Slave,
}
/// Source-defined data sampling edge.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ClockEdge {
    /// MODE.CLK_EDGE=0.
    Falling,
    /// MODE.CLK_EDGE=1; SDK default.
    Rising,
}
/// Stereo, standard I2S, split FIFOs, no sign extension or internal loopback.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Config {
    width: DataWidth,
    clock: ClockMode,
    edge: ClockEdge,
    polls: NonZeroU16,
}
impl Config {
    /// Finite polling count per frame/progress point, with1us between attempts.
    /// This is not a wall deadline; the delay provider must remain independent.
    pub const fn new(width: DataWidth, clock: ClockMode, polls: NonZeroU16) -> Self {
        Self { width, clock, edge: ClockEdge::Rising, polls }
    }
    /// Select the source-defined sampling edge; physical timing needs qualification.
    pub const fn with_edge(mut self, edge: ClockEdge) -> Self {
        self.edge = edge;
        self
    }
    /// Data width used by both directions.
    pub const fn data_width(self) -> DataWidth {
        self.width
    }
    /// Role and optional validated BCLK divider.
    pub const fn clock_mode(self) -> ClockMode {
        self.clock
    }
    fn frame_bits(self) -> u16 {
        u16::from(self.width.bits()) * 2
    }
}
/// One left/right sample pair. TX rejects nonzero bits above the configured width.
/// Signed samples must be explicitly represented in that width's two's complement.
/// RX preserves the hardware's raw words instead of guessing padding semantics.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Frame {
    /// Left sample bits.
    pub left: u32,
    /// Right sample bits.
    pub right: u32,
}
/// Directions active during a stream. No peripheral DMA requests are consumed.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum StreamMode {
    /// Transmit only.
    Transmit,
    /// Receive only.
    Receive,
    /// Transmit and receive with a common stereo clock.
    Duplex,
}
impl StreamMode {
    fn tx(self) -> bool {
        self != Self::Receive
    }
    fn rx(self) -> bool {
        self != Self::Transmit
    }
}
/// FIFO faults reported by the IP; no status bits are silently cleared on error.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Faults {
    /// Left receive FIFO overflow.
    pub rx_left: bool,
    /// Right receive FIFO overflow.
    pub rx_right: bool,
    /// Left transmit FIFO underflow.
    pub tx_left: bool,
    /// Right transmit FIFO underflow.
    pub tx_right: bool,
}
impl Faults {
    fn relevant(self, mode: StreamMode) -> bool {
        (mode.rx() && (self.rx_left || self.rx_right)) || (mode.tx() && (self.tx_left || self.tx_right))
    }
}
/// Progress point that exhausted its finite polling budget.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Wait {
    /// Configuration readback/empty FIFOs after block reset.
    Configure,
    /// At least one free entry in both TX FIFOs.
    TxSpace,
    /// At least one entry in both RX FIFOs, even below IRQ threshold.
    RxData,
    /// Both TX FIFOs empty; does not include the shift register.
    TxEmpty,
}
/// Driver or hardware error. Hardware/timeouts poison the stream until recovery.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Error {
    /// Handed-over SCLK/WS directions do not match the configured role.
    PinClockDirection,
    /// A previous hardware error requires discard_and_recover().
    Poisoned,
    /// Start requires the prepared/stopped state.
    NotReady,
    /// Operation requires a running stream.
    NotRunning,
    /// Requested direction was not started.
    WrongDirection,
    /// TX/duplex needs1..8 initial pairs; RX-only requires no initial pairs.
    InvalidPrefill,
    /// A TX sample has bits above the configured width; no truncation performed.
    InvalidSample,
    /// Reported FIFO depth exceeds the source profile's eight entries.
    InvalidDepth,
    /// A configuration register changed or stopped reading back the programmed value.
    ConfigurationChanged,
    /// Internal clock-enable readback was lost; not an independent clock detector.
    ClockStopped,
    /// Reported fault in an active direction.
    Hardware(Faults),
    /// No progress within the configured polling budget.
    Timeout(Wait),
}
/// Partial progress is retained; frames means queued TX pairs or returned RX pairs.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct TransferError {
    /// Failure; inspect is_poisoned() before retrying.
    pub cause: Error,
    /// Number of pairs completed before failure, never a wire-completion count.
    pub frames: usize,
}
/// Owned physical routes, handed over after mux/voltage/direction configuration.
pub struct Pins<'d> {
    sclk: PadControl<'d>,
    ws: PadControl<'d>,
    dout: PadControl<'d>,
    din: PadControl<'d>,
    master: bool,
}
impl<'d> Pins<'d> {
    /// Adopt all four distinct physical routes without changing pad MMIO.
    ///
    /// # Safety
    /// Own the pads and all aliases exclusively, with valid I2S0 mux/drive/input
    /// settings. SCLK/WS direction must match the supplied clock role. No
    /// competing driver/core/DMA may use these pads. Keep routing/power valid
    /// until external quiescence is established before reclaiming the pads.
    pub unsafe fn from_configured(
        sclk: PadControl<'d>,
        ws: PadControl<'d>,
        dout: PadControl<'d>,
        din: PadControl<'d>,
        clock: ClockMode,
    ) -> Self {
        Self { sclk, ws, dout, din, master: matches!(clock, ClockMode::Master(_)) }
    }
    /// Unpack routes after the caller has proved external quiescence.
    pub fn into_pads(self) -> (PadControl<'d>, PadControl<'d>, PadControl<'d>, PadControl<'d>) {
        (self.sclk, self.ws, self.dout, self.din)
    }
}
/// Experimental polling stereo stream. Borrowed buffers are never retained.
/// No safe flush/free claim is made without an evidenced wire-idle indication.
///
/// ```compile_fail
/// use hisi_hal::{i2s::{I2sStream,Config,Pins}, peripherals::{I2s,Unclaimed}};
/// fn handoff(p:Unclaimed<I2s<'static>>,pins:Pins<'static>,c:Config,d:impl embedded_hal::delay::DelayNs) {
///     let _ = I2sStream::new(p,pins,c,d);
/// }
/// ```
/// ```compile_fail
/// use hisi_hal::i2s::I2sStream;
/// fn no_wire_claim(mut s:I2sStream<'_,impl embedded_hal::delay::DelayNs>) { s.flush(); }
/// ```
pub struct I2sStream<'d, D: DelayNs> {
    token: Option<I2s<'d>>,
    pins: Option<Pins<'d>>,
    engine: Engine<PacRegisters, D>,
}
impl<'d, D: DelayNs> I2sStream<'d, D> {
    /// Enable clocks without asserting the shared audio reset; mask CPU IRQ67,
    /// reset/configure the owned I2S block and verify readback with a finite budget.
    /// Failure returns the driver with all resources retained for recovery.
    pub fn new(token: I2s<'d>, pins: Pins<'d>, config: Config, delay: D) -> Result<Self, (Error, Self)> {
        let mut s = Self { token: Some(token), pins: Some(pins), engine: Engine::new(PacRegisters, delay, config) };
        match s.discard_and_recover() {
            Ok(()) => Ok(s),
            Err(e) => Err((e, s)),
        }
    }
    /// Start a prepared stream. TX/duplex must prefill1..8 pairs; RX-only takes[].
    /// FIFO loading precedes clock enable, the SDK's25us setup delay, and TX/RX enable.
    pub fn start(&mut self, mode: StreamMode, initial: &[Frame]) -> Result<(), Error> {
        self.engine.start(mode, initial)
    }
    /// Queue one pair if both FIFOs have space. Success does not mean transmitted.
    pub fn try_queue(&mut self, frame: Frame) -> Result<bool, Error> {
        self.engine.try_queue(frame)
    }
    /// Queue all pairs with finite per-pair waits, reporting partial FIFO progress.
    pub fn queue_frames(&mut self, frames: &[Frame]) -> Result<(), TransferError> {
        self.engine.queue(frames)
    }
    /// Read a complete pair if available, including bursts below the IRQ threshold.
    pub fn try_read(&mut self) -> Result<Option<Frame>, Error> {
        self.engine.try_read()
    }
    /// Fill the destination with paired samples, reporting partial progress on failure.
    pub fn read_frames(&mut self, frames: &mut [Frame]) -> Result<(), TransferError> {
        self.engine.read(frames)
    }
    /// Observe both TX FIFOs empty with bounded waits. The clock/stream stays on;
    /// data may still be shifting and underflow may follow. This is NOT flush.
    pub fn wait_tx_fifo_empty(&mut self) -> Result<(), Error> {
        self.engine.direction(true)?;
        self.engine.wait(Wait::TxEmpty).map(|_| ())
    }
    /// Discard/reset any queued data, then prepare the same configuration.
    /// Can truncate an output sample; it does not establish successful transmission.
    /// Ownership stays with the driver even if recovery times out.
    pub fn discard_and_recover(&mut self) -> Result<(), Error> {
        if self.pins.as_ref().unwrap().master != matches!(self.engine.config.clock, ClockMode::Master(_)) {
            return Err(Error::PinClockDirection);
        }
        self.engine.recover()
    }
    /// Whether a hardware failure requires explicit recovery.
    pub fn is_poisoned(&self) -> bool {
        self.engine.state == State::Poisoned
    }
    /// Reclaim resources only after external proof of inactive outputs/requests.
    ///
    /// # Safety
    /// Independently establish quiescence of the block, DMA requests, IRQ source
    /// and pad drivers; FIFO-empty and software clock-bit readback are insufficient.
    /// No further driver MMIO occurs. Shared clocks remain enabled and must be
    /// handed back through the board's ownership protocol, not blindly disabled.
    pub unsafe fn release_after_external_quiescence(mut self) -> (I2s<'d>, Pins<'d>) {
        (self.token.take().unwrap(), self.pins.take().unwrap())
    }
}
impl<D: DelayNs> Drop for I2sStream<'_, D> {
    fn drop(&mut self) {
        if self.token.is_some() && self.engine.accessed {
            self.engine.io.stop_request();
        }
    }
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum State {
    Ready,
    Running(StreamMode),
    Poisoned,
}
#[derive(Clone, Copy, Default)]
struct Snapshot {
    tx: [u8; 2],
    rx: [u8; 2],
    faults: Faults,
    clock: bool,
    configured: bool,
}
impl Snapshot {
    fn valid_depths(self) -> bool {
        self.tx.into_iter().chain(self.rx).all(|v| v <= 8)
    }
    fn empty(self) -> bool {
        self.tx == [0; 2] && self.rx == [0; 2]
    }
}
trait Registers {
    fn enable_clocks(&mut self);
    fn stop_request(&mut self);
    fn reset_configure(&mut self, config: Config);
    fn snapshot(&mut self, config: Config) -> Snapshot;
    fn clock(&mut self, on: bool);
    fn start(&mut self, mode: StreamMode);
    fn push(&mut self, frame: Frame);
    fn pop(&mut self) -> Frame;
}
struct Engine<R: Registers, D: DelayNs> {
    io: R,
    delay: D,
    config: Config,
    state: State,
    accessed: bool,
}
impl<R: Registers, D: DelayNs> Engine<R, D> {
    fn new(io: R, delay: D, config: Config) -> Self {
        Self { io, delay, config, state: State::Poisoned, accessed: false }
    }
    fn fail(&mut self, error: Error) -> Error {
        self.state = State::Poisoned;
        self.io.stop_request();
        error
    }
    fn recover(&mut self) -> Result<(), Error> {
        self.state = State::Poisoned;
        self.io.enable_clocks();
        self.accessed = true;
        self.io.stop_request();
        self.io.reset_configure(self.config);
        for n in 0..self.config.polls.get() {
            let s = self.io.snapshot(self.config);
            if !s.valid_depths() {
                return Err(self.fail(Error::InvalidDepth));
            }
            if s.configured && !s.clock && s.empty() && !s.faults.relevant(StreamMode::Duplex) {
                self.state = State::Ready;
                return Ok(());
            }
            if n + 1 < self.config.polls.get() {
                self.delay.delay_us(1);
            }
        }
        Err(self.fail(Error::Timeout(Wait::Configure)))
    }
    fn direction(&self, tx: bool) -> Result<StreamMode, Error> {
        match self.state {
            State::Running(m) if if tx { m.tx() } else { m.rx() } => Ok(m),
            State::Running(_) => Err(Error::WrongDirection),
            State::Poisoned => Err(Error::Poisoned),
            State::Ready => Err(Error::NotRunning),
        }
    }
    fn inspect(&mut self) -> Result<Snapshot, Error> {
        let mode = match self.state {
            State::Running(m) => m,
            State::Poisoned => return Err(Error::Poisoned),
            _ => return Err(Error::NotRunning),
        };
        let s = self.io.snapshot(self.config);
        let err = if !s.valid_depths() {
            Some(Error::InvalidDepth)
        } else if !s.configured {
            Some(Error::ConfigurationChanged)
        } else if !s.clock {
            Some(Error::ClockStopped)
        } else if s.faults.relevant(mode) {
            Some(Error::Hardware(s.faults))
        } else {
            None
        };
        if let Some(e) = err { Err(self.fail(e)) } else { Ok(s) }
    }
    fn wait(&mut self, kind: Wait) -> Result<Snapshot, Error> {
        for n in 0..self.config.polls.get() {
            let s = self.inspect()?;
            if match kind {
                Wait::TxSpace => s.tx.iter().all(|v| *v < 8),
                Wait::RxData => s.rx.iter().all(|v| *v > 0),
                Wait::TxEmpty => s.tx == [0; 2],
                Wait::Configure => false,
            } {
                return Ok(s);
            }
            if n + 1 < self.config.polls.get() {
                self.delay.delay_us(1);
            }
        }
        Err(self.fail(Error::Timeout(kind)))
    }
    fn start(&mut self, mode: StreamMode, initial: &[Frame]) -> Result<(), Error> {
        if self.state == State::Poisoned {
            return Err(Error::Poisoned);
        }
        if self.state != State::Ready {
            return Err(Error::NotReady);
        }
        if (mode.tx() && !(1..=8).contains(&initial.len())) || (!mode.tx() && !initial.is_empty()) {
            return Err(Error::InvalidPrefill);
        }
        if initial.iter().any(|v| !self.config.width.contains(*v)) {
            return Err(Error::InvalidSample);
        }
        let s = self.io.snapshot(self.config);
        if !s.valid_depths() {
            return Err(self.fail(Error::InvalidDepth));
        }
        if !s.configured || s.clock || !s.empty() {
            return Err(self.fail(Error::ConfigurationChanged));
        }
        if s.faults.relevant(mode) {
            return Err(self.fail(Error::Hardware(s.faults)));
        }
        for frame in initial {
            self.io.push(*frame);
        }
        self.io.clock(true);
        self.delay.delay_us(25);
        self.io.start(mode);
        self.state = State::Running(mode);
        self.inspect().map(|_| ())
    }
    fn try_queue(&mut self, frame: Frame) -> Result<bool, Error> {
        self.direction(true)?;
        if !self.config.width.contains(frame) {
            return Err(Error::InvalidSample);
        }
        let s = self.inspect()?;
        if s.tx.contains(&8) {
            return Ok(false);
        }
        self.io.push(frame);
        Ok(true)
    }
    fn queue(&mut self, frames: &[Frame]) -> Result<(), TransferError> {
        self.direction(true).map_err(|cause| TransferError { cause, frames: 0 })?;
        if frames.iter().any(|f| !self.config.width.contains(*f)) {
            return Err(TransferError { cause: Error::InvalidSample, frames: 0 });
        }
        for (n, frame) in frames.iter().enumerate() {
            self.wait(Wait::TxSpace).map_err(|cause| TransferError { cause, frames: n })?;
            self.io.push(*frame);
        }
        Ok(())
    }
    fn try_read(&mut self) -> Result<Option<Frame>, Error> {
        self.direction(false)?;
        let s = self.inspect()?;
        if s.rx.contains(&0) { Ok(None) } else { Ok(Some(self.io.pop())) }
    }
    fn read(&mut self, frames: &mut [Frame]) -> Result<(), TransferError> {
        self.direction(false).map_err(|cause| TransferError { cause, frames: 0 })?;
        for (n, frame) in frames.iter_mut().enumerate() {
            self.wait(Wait::RxData).map_err(|cause| TransferError { cause, frames: n })?;
            *frame = self.io.pop();
        }
        Ok(())
    }
}
struct PacRegisters;
impl PacRegisters {
    fn regs(&self) -> &'static pac::i2s::RegisterBlock {
        unsafe { &*I2s::ptr() }
    }
}
impl Registers for PacRegisters {
    fn enable_clocks(&mut self) {
        // The ownership handoff serializes these whole-word RMW operations;
        // disabling this hart's IRQs would not exclude the control core.
        unsafe { &*pac::AudioCmu::PTR }.new_cfg0().modify(|_, w| w.audio_divider_reset_n().set_bit());
        unsafe { &*pac::MCrg::PTR }.m_clken0_word().modify(|_, w| w.i2s_enable().set_bit());
    }
    fn stop_request(&mut self) {
        unsafe {
            interrupt::disable(Interrupt::I2S);
        }
        let r = self.regs();
        unsafe {
            r.intmask().write_with_zero(|w| {
                w.rx()
                    .set_bit()
                    .tx()
                    .set_bit()
                    .rx_left_overflow()
                    .set_bit()
                    .rx_right_overflow()
                    .set_bit()
                    .tx_left_underflow()
                    .set_bit()
                    .tx_right_underflow()
                    .set_bit()
            });
            r.ct_clr().write_with_zero(|w| {
                w.tx_enable().set_bit().rx_enable().set_bit().interrupt_enable().set_bit().reset_n().set_bit()
            });
        }
        r.i2s_crg().modify(|_, w| w.clock_enable().clear_bit());
        interrupt::clear_pending(Interrupt::I2S);
    }
    fn reset_configure(&mut self, c: Config) {
        let r = self.regs();
        r.version().modify(|_, w| w.loopback().clear_bit());
        r.mode().modify(|_, w| unsafe {
            w.mode()
                .clear_bit()
                .pcm_mode()
                .clear_bit()
                .rxws_select()
                .clear_bit()
                .extended_receive()
                .clear_bit()
                .channels()
                .bits(0)
                .clk_edge()
                .bit(c.edge == ClockEdge::Rising)
                .master()
                .bit(matches!(c.clock, ClockMode::Master(_)))
        });
        let tx = match c.width {
            DataWidth::Bits16 => pac::i2s::data_width_set::TxMode::Bits16,
            DataWidth::Bits18 => pac::i2s::data_width_set::TxMode::Bits18,
            DataWidth::Bits20 => pac::i2s::data_width_set::TxMode::Bits20,
            DataWidth::Bits24 => pac::i2s::data_width_set::TxMode::Bits24,
            DataWidth::Bits32 => pac::i2s::data_width_set::TxMode::Bits32,
        };
        let rx = match c.width {
            DataWidth::Bits16 => pac::i2s::data_width_set::RxMode::Bits16,
            DataWidth::Bits18 => pac::i2s::data_width_set::RxMode::Bits18,
            DataWidth::Bits20 => pac::i2s::data_width_set::RxMode::Bits20,
            DataWidth::Bits24 => pac::i2s::data_width_set::RxMode::Bits24,
            DataWidth::Bits32 => pac::i2s::data_width_set::RxMode::Bits32,
        };
        r.data_width_set().modify(|_, w| w.tx_mode().variant(tx).rx_mode().variant(rx));
        r.i2s_crg().modify(|_, w| {
            w.clock_enable().clear_bit().bclk_div_enable().clear_bit().bclk_select().clear_bit().fs_select().clear_bit()
        });
        if let ClockMode::Master(div) = c.clock {
            r.i2s_bclk_div_num().modify(|_, w| unsafe { w.divider().bits(div.get()) });
            r.i2s_fs_div_num().modify(|_, w| unsafe { w.divider().bits(c.frame_bits()) });
            r.i2s_fs_div_ratio_num().modify(|_, w| unsafe { w.ratio().bits(c.frame_bits() / 2) });
            r.i2s_crg().modify(|_, w| w.bclk_div_enable().set_bit());
        }
        r.signed_ext().modify(|_, w| w.enable().clear_bit());
        r.i2s_pos_merge_en().modify(|_, w| w.enable().clear_bit());
        // SDK writes zero, including to the position flags; do not replay flags.
        unsafe {
            r.i2s_start_pos().write_with_zero(|w| w.read_right().clear_bit().write_right().clear_bit());
            r.i2s_pos_flag().write_with_zero(|w| w.read_right().clear_bit().write_right().clear_bit());
        }
        r.fifo_threshold().modify(|_, w| unsafe { w.tx().bits(4).rx().bits(7) });
        unsafe {
            r.ct_clr().write_with_zero(|w| {
                w.tx_merge().set_bit().rx_merge().set_bit().tx_fifo_disable().set_bit().rx_fifo_disable().set_bit()
            });
            r.ct_set().write_with_zero(|w| w.reset_n().set_bit());
            r.intclr().write_with_zero(|w| {
                w.rx()
                    .clear_bit_by_one()
                    .tx()
                    .clear_bit_by_one()
                    .rx_left_overflow()
                    .clear_bit_by_one()
                    .rx_right_overflow()
                    .clear_bit_by_one()
                    .tx_left_underflow()
                    .clear_bit_by_one()
                    .tx_right_underflow()
                    .clear_bit_by_one()
            });
            // Enable error reporting but mask threshold events; CPU IRQ stays
            // disabled. This also works if INTSTATUS is masked rather than raw.
            r.intmask().write_with_zero(|w| w.rx().set_bit().tx().set_bit());
            r.ct_set().write_with_zero(|w| w.interrupt_enable().set_bit());
        }
        interrupt::clear_pending(Interrupt::I2S);
    }
    fn snapshot(&mut self, c: Config) -> Snapshot {
        let r = self.regs();
        let tx = r.tx_sta().read();
        let rx = r.rx_sta().read();
        let flags = r.intstatus().read();
        let mode = r.mode().read();
        let width = r.data_width_set().read();
        let clock = r.i2s_crg().read();
        let divider_ok = match c.clock {
            ClockMode::Slave => !clock.bclk_div_enable().bit(),
            ClockMode::Master(div) => {
                clock.bclk_div_enable().bit()
                    && r.i2s_bclk_div_num().read().divider().bits() == div.get()
                    && r.i2s_fs_div_num().read().divider().bits() == c.frame_bits()
                    && r.i2s_fs_div_ratio_num().read().ratio().bits() == c.frame_bits() / 2
            }
        };
        Snapshot {
            tx: [tx.left().bits(), tx.right().bits()],
            rx: [rx.left().bits(), rx.right().bits()],
            clock: clock.clock_enable().bit(),
            faults: Faults {
                rx_left: flags.rx_left_overflow().bit(),
                rx_right: flags.rx_right_overflow().bit(),
                tx_left: flags.tx_left_underflow().bit(),
                tx_right: flags.tx_right_underflow().bit(),
            },
            configured: !mode.mode().bit()
                && !mode.pcm_mode().bit()
                && !mode.rxws_select().bit()
                && !mode.extended_receive().bit()
                && mode.channels().bits() == 0
                && mode.clk_edge().bit() == (c.edge == ClockEdge::Rising)
                && mode.master().bit() == matches!(c.clock, ClockMode::Master(_))
                && width.tx_mode().bits() == c.width as u8
                && width.rx_mode().bits() == c.width as u8
                && divider_ok
                && !clock.bclk_select().bit()
                && !clock.fs_select().bit()
                && !r.signed_ext().read().enable().bit()
                && !r.i2s_pos_merge_en().read().enable().bit()
                && !r.version().read().loopback().bit(),
        }
    }
    fn clock(&mut self, on: bool) {
        self.regs().i2s_crg().modify(|_, w| w.clock_enable().bit(on));
    }
    fn start(&mut self, mode: StreamMode) {
        unsafe {
            self.regs().ct_set().write_with_zero(|w| w.tx_enable().bit(mode.tx()).rx_enable().bit(mode.rx()));
        }
    }
    fn push(&mut self, frame: Frame) {
        unsafe {
            self.regs().left_tx().write_with_zero(|w| w.data().bits(frame.left));
            self.regs().right_tx().write_with_zero(|w| w.data().bits(frame.right));
        }
    }
    fn pop(&mut self) -> Frame {
        Frame { left: self.regs().left_rx().read().data().bits(), right: self.regs().right_rx().read().data().bits() }
    }
}
#[cfg(test)]
#[path = "i2s_ws53/tests.rs"]
mod tests;
