//! WS53 ROM v151 MDMA, experimental polling memory and peripheral transfers.
//!
//! Four observed memory channels or eight SDK/model slots are explicitly selected.
//! Per-channel event masks support IRQ58 notification; buffers remain static, exclusively
//! owned and explicitly coherent; timeout/cancel/Drop never reclaim them. An
//! independent unsafe bus-drain proof is required even after observed completion.
//! Memory-copy async waiting is optional; peripheral async, SDMA, LLI and cache
//! maintenance remain pending.
use crate::{interrupt, peripherals::Mdma, soc::pac};
use core::{
    marker::PhantomData,
    num::NonZeroU16,
    ptr::NonNull,
    sync::atomic::{Ordering, compiler_fence},
};
use embedded_hal::delay::DelayNs;

#[path = "dma_ws53/i2c.rs"]
pub mod i2c;
#[path = "dma_ws53/i2s.rs"]
pub mod i2s;
#[path = "dma_ws53/spi.rs"]
pub mod spi;
#[path = "dma_ws53/uart.rs"]
pub mod uart;

#[cfg(feature = "async")]
#[path = "dma_ws53/asynch.rs"]
mod asynch;
#[cfg(feature = "async")]
pub use asynch::{AsyncTransfer, on_async_interrupt};

#[derive(Clone, Copy)]
enum Mode {
    Memory,
    Uart { port: uart::Port, receive: bool },
    Spi { port: spi::Port, receive: bool },
    I2c { port: i2c::Port, receive: bool, burst: i2c::Burst },
    I2s { receive: bool, burst: i2s::Burst },
}
impl Mode {
    fn flow(self) -> u8 {
        match self {
            Self::Memory => 0,
            Self::Uart { receive: false, .. }
            | Self::Spi { receive: false, .. }
            | Self::I2c { receive: false, .. }
            | Self::I2s { receive: false, .. } => 1,
            Self::Uart { receive: true, .. }
            | Self::Spi { receive: true, .. }
            | Self::I2c { receive: true, .. }
            | Self::I2s { receive: true, .. } => 2,
        }
    }
    fn requests(self) -> (u8, u8) {
        match self {
            Self::Memory => (0, 0),
            Self::I2s { receive: true, .. } => (13, 0),
            Self::I2s { receive: false, .. } => (0, 12),
            Self::Uart { port, receive: true } => (port.request(true), 0),
            Self::Uart { port, receive: false } => (0, port.request(false)),
            Self::Spi { port, receive: true } => (port.request(true), 0),
            Self::Spi { port, receive: false } => (0, port.request(false)),
            Self::I2c { port, receive: true, .. } => (port.request(true), 0),
            Self::I2c { port, receive: false, .. } => (0, port.request(false)),
        }
    }
    fn protection(self) -> u8 {
        match self {
            Self::Spi { .. } | Self::I2c { receive: true, .. } => 2,
            Self::I2c { receive: false, .. } | Self::I2s { .. } => 1,
            _ => 0,
        }
    }
    fn burst(self) -> u8 {
        match self {
            Self::I2c { burst, .. } => burst.encoding(),
            Self::I2s { burst, .. } => burst.encoding(),
            _ => 0,
        }
    }
    fn increments(self) -> (bool, bool) {
        (self.flow() != 2, self.flow() != 1)
    }
}

/// Identity in the SDK's eight-slot MDMA layout; physical availability is separate.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum ChannelId {
    /// Hardware channel0.
    Channel0,
    /// Hardware channel1.
    Channel1,
    /// Hardware channel2.
    Channel2,
    /// Hardware channel3.
    Channel3,
    /// Hardware channel4.
    Channel4,
    /// Hardware channel5.
    Channel5,
    /// Hardware channel6.
    Channel6,
    /// Hardware channel7.
    Channel7,
}
impl ChannelId {
    fn mask(self) -> u8 {
        1 << self as u8
    }
}
/// Per-channel interrupt sources. Local CPU IRQ58 delivery is managed separately.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub enum InterruptEvents {
    /// Retain raw events without asserting the shared interrupt.
    #[default]
    None,
    /// Transfer-complete events only.
    Completion,
    /// Bus-error events only.
    Error,
    /// Both completion and bus-error events.
    All,
}
impl InterruptEvents {
    fn completion(self) -> bool {
        matches!(self, Self::Completion | Self::All)
    }
    fn error(self) -> bool {
        matches!(self, Self::Error | Self::All)
    }
}
/// Sampled event flags; neither event proves that posted bus writes have drained.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct InterruptStatus {
    /// Transfer-complete flag.
    pub complete: bool,
    /// Bus-error flag.
    pub error: bool,
}
/// Read per-channel masked status without clearing events or modifying ownership.
/// An ISR may record this snapshot and disable local IRQ58. Call `Transfer::wait`
/// in normal context before acknowledging: polling still needs the raw event.
pub fn interrupt_status(channel: ChannelId) -> InterruptStatus {
    let status = regs().interrupt_status().read();
    InterruptStatus {
        complete: status.transfer_complete().bits() & channel.mask() != 0,
        error: status.error().bits() & channel.mask() != 0,
    }
}
/// An owned channel is neither cloneable nor constructible from a raw index.
///
/// ```compile_fail
/// use hisi_hal::dma::Channel;
/// fn duplicate(c: Channel<'_>) { let _ = c.clone(); }
/// ```
pub struct Channel<'d> {
    id: ChannelId,
    memory_readback: MemoryReadback,
    _owner: PhantomData<&'d mut ()>,
}
#[derive(Clone, Copy, PartialEq, Eq)]
enum MemoryReadback {
    SdkModel,
    ObservedIdleZeroLastAddress,
}
/// Four accessible channels under the observed WS53 memory-copy profile.
/// Slots4..7 are not exposed: the tested board reads their configuration as0.
/// Peripheral request behavior and physical drain remain separately unqualified.
pub struct FourChannels<'d> {
    /// Exclusive hardware channel0.
    pub channel0: Channel<'d>,
    /// Exclusive hardware channel1.
    pub channel1: Channel<'d>,
    /// Exclusive hardware channel2.
    pub channel2: Channel<'d>,
    /// Exclusive hardware channel3.
    pub channel3: Channel<'d>,
    _controller: Mdma<'d>,
}
impl Channel<'_> {
    /// Read the hardware identity without duplicating ownership.
    pub fn id(&self) -> ChannelId {
        self.id
    }
}
/// Eight independent reservations. Dropping the container never disables the
/// shared controller or clock, including after individual channels are moved out.
pub struct Channels<'d> {
    /// Exclusive hardware channel0.
    pub channel0: Channel<'d>,
    /// Exclusive hardware channel1.
    pub channel1: Channel<'d>,
    /// Exclusive hardware channel2.
    pub channel2: Channel<'d>,
    /// Exclusive hardware channel3.
    pub channel3: Channel<'d>,
    /// Exclusive hardware channel4.
    pub channel4: Channel<'d>,
    /// Exclusive hardware channel5.
    pub channel5: Channel<'d>,
    /// Exclusive hardware channel6.
    pub channel6: Channel<'d>,
    /// Exclusive hardware channel7.
    pub channel7: Channel<'d>,
    _controller: Mdma<'d>,
}
/// Whole-controller reservation; performs no MMIO until split is requested.
///
/// ```compile_fail
/// use hisi_hal::{dma::Controller, peripherals::{Mdma, Unclaimed}};
/// fn handoff(p: Unclaimed<Mdma<'static>>) { Controller::new(p); }
/// ```
pub struct Controller<'d> {
    token: Mdma<'d>,
}
impl<'d> Controller<'d> {
    /// Accept the externally handed-over MDMA block without accessing registers.
    pub fn new(token: Mdma<'d>) -> Self {
        Self { token }
    }
    /// Guard against inherited active channels, enable the controller, disable
    /// its local IRQ and clear old raw events before handing out eight tokens.
    /// Its source/bus clocks and memory mode must satisfy the ownership handoff.
    /// Failure returns the token; clock/shared reset is never changed here.
    /// This is the eight-slot SDK/model profile, not a silicon presence probe.
    /// For the observed A-board memory-copy semantics use `split_four` instead.
    pub fn split(self) -> Result<Channels<'d>, (Error, Mdma<'d>)> {
        if let Err(error) = initialize(&mut PacController) {
            return Err((error, self.token));
        }
        fn channel<'d>(id: ChannelId) -> Channel<'d> {
            Channel { id, memory_readback: MemoryReadback::SdkModel, _owner: PhantomData }
        }
        Ok(Channels {
            channel0: channel(ChannelId::Channel0),
            channel1: channel(ChannelId::Channel1),
            channel2: channel(ChannelId::Channel2),
            channel3: channel(ChannelId::Channel3),
            channel4: channel(ChannelId::Channel4),
            channel5: channel(ChannelId::Channel5),
            channel6: channel(ChannelId::Channel6),
            channel7: channel(ChannelId::Channel7),
            _controller: self.token,
        })
    }
    /// Reserve only channels0..3 using observed idle-count0/final-element memory
    /// address readback. Does not infer peripheral transfer or bus-drain behavior.
    /// Existing source/model-only eight-slot consumers retain their explicit
    /// `split` contract; neither constructor discovers silicon revision.
    pub fn split_four(self) -> Result<FourChannels<'d>, (Error, Mdma<'d>)> {
        let mut all = self.split()?;
        for ch in [&mut all.channel0, &mut all.channel1, &mut all.channel2, &mut all.channel3] {
            ch.memory_readback = MemoryReadback::ObservedIdleZeroLastAddress;
        }
        Ok(FourChannels {
            channel0: all.channel0,
            channel1: all.channel1,
            channel2: all.channel2,
            channel3: all.channel3,
            _controller: all._controller,
        })
    }
}
mod sealed {
    pub trait Element {}
    impl Element for u8 {}
    impl Element for u16 {}
    impl Element for u32 {}
}
/// The audited equal-width memory-transfer element types: u8, u16 and u32.
/// External implementations cannot introduce unsupported widths or Rust drop types.
pub trait Element: sealed::Element + Copy + 'static {
    #[doc(hidden)]
    const WIDTH_CODE: u8;
}
impl Element for u8 {
    const WIDTH_CODE: u8 = 0;
}
impl Element for u16 {
    const WIDTH_CODE: u8 = 1;
}
impl Element for u32 {
    const WIDTH_CODE: u8 = 2;
}
/// Invalid memory-transfer allocation. No register write has occurred.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BufferError {
    /// Hardware element count must be1..4095; larger values are never truncated.
    Length,
    /// Overflow, unaligned address or allocation outside MCU_MODE_1 DTCM.
    Range,
}
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Range {
    start: u32,
    end: u32,
    count: u16,
    width: u8,
}
impl Range {
    fn new(start: usize, count: usize, width: u8) -> Result<Self, BufferError> {
        if !(1..=4095).contains(&count) {
            return Err(BufferError::Length);
        }
        if !matches!(width, 1 | 2 | 4) {
            return Err(BufferError::Range);
        }
        let end = count.checked_mul(width as usize).and_then(|n| start.checked_add(n)).ok_or(BufferError::Range)?;
        if start & (width as usize - 1) != 0 || start < 0x20010000 || end > 0x20060000 {
            return Err(BufferError::Range);
        }
        Ok(Self { start: start as u32, end: end as u32, count: count as u16, width })
    }
    fn overlaps(self, other: Self) -> bool {
        self.start < other.end && other.start < self.end
    }
}
/// Owned, coherent, static memory. Dropping it intentionally retains the
/// allocation, so forgetting an active transfer cannot free memory under DMA.
///
/// ```compile_fail
/// use hisi_hal::dma::Buffer;
/// fn local() { let mut b=[0u8;4]; unsafe { Buffer::from_coherent_static(&mut b) }; }
/// ```
pub struct Buffer<T: Element> {
    pointer: NonNull<T>,
    range: Range,
    _owned: PhantomData<&'static mut [T]>,
}
impl<T: Element> Buffer<T> {
    /// Reserve a source or destination without reading/writing its contents.
    /// Invalid allocations are returned unchanged.
    ///
    /// # Safety
    /// Entire allocation must be application-owned, accessible by MDMA master0
    /// and noncacheable or hardware coherent, in the current MCU memory mode.
    /// No dirty cache alias, CPU/control-core/trace/DMA access or overlapping
    /// cache-line owner may exist. Complete necessary CPU/bus synchronization
    /// before transfer. Keep memory attributes valid until external handback.
    /// This constructor checks bounds; it does not flush caches or negotiate IPC.
    pub unsafe fn from_coherent_static(words: &'static mut [T]) -> Result<Self, (BufferError, &'static mut [T])> {
        let pointer = words.as_mut_ptr();
        let range = match Range::new(pointer as usize, words.len(), core::mem::size_of::<T>() as u8) {
            Ok(v) => v,
            Err(e) => return Err((e, words)),
        };
        Ok(Self { pointer: NonNull::new(pointer).unwrap(), range, _owned: PhantomData })
    }
    /// Retrieve unused memory or memory returned after independent drain proof.
    pub fn into_words(self) -> &'static mut [T] {
        compiler_fence(Ordering::Acquire);
        unsafe { core::slice::from_raw_parts_mut(self.pointer.as_ptr(), self.range.count as usize) }
    }
}
/// Transfer failure; runtime failures retain the channel and both allocations.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error {
    /// Existing enabled/active channel contradicts the exclusive handoff.
    Occupied,
    /// Controller/configuration readback differs from the supported profile.
    Configuration,
    /// Source and destination lengths differ.
    LengthMismatch,
    /// Source/destination overlap; this interface is a copy, not memmove.
    Overlap,
    /// Transfer already attempted/cancelled; external recovery required.
    NotReady,
    /// Raw TC/error bits did not clear before starting.
    StaleEvent,
    /// Raw error has priority over a simultaneously observed completion.
    Bus,
    /// No completion within the finite polling budget.
    Timeout,
    /// TC observed with nonzero count, unexpected addresses or enabled/active state.
    CompletionState,
    /// UART does not advertise the audited DMA/FIFO/32-bit APB profile.
    UartCapability,
    /// UART is not in the inherited 8N1 data/FIFO profile.
    UartConfiguration,
    /// Existing queued data or an active transmitter prevents initial handoff.
    UartBusy,
    /// Observed UART line errors; this is not an exhaustive integrity certificate.
    UartLine(uart::LineErrors),
    /// DMA completion was observed but the transmitter did not become idle.
    TransmitTimeout,
    /// SPI is not in the inherited single-line, full-duplex master profile.
    SpiConfiguration,
    /// SPI has queued data, an active shifter or enabled DMA requests at handoff.
    SpiBusy,
    /// SPI latched an error; raw bits use the instance's PAC layout.
    SpiHardware {
        /// Raw interrupt snapshot from the selected SPI register layout.
        raw: u32,
    },
    /// SPI read-clear master-collision status, independently of the IRQ latch.
    SpiCollision {
        /// Complete status word sampled before its collision bit was cleared.
        status: u32,
        /// Raw interrupt word sampled alongside the status.
        raw_interrupt: u32,
    },
    /// Both DMA channels completed but additional unowned RX data remains.
    SpiResidualReceive,
    /// I2C master/address/clock/timing/request readback differs from handoff.
    I2cConfiguration,
    /// I2C has inherited FIFO contents, active bus or enabled DMA requests.
    I2cBusy,
    /// I2C raw error and abort words, preserved without read-clear operations.
    I2cHardware {
        /// Raw interrupt word.
        raw: u32,
        /// Abort source word.
        source: u32,
        /// Detailed abort word.
        detail: u32,
    },
    /// DMA completed but unexpected receive entries remain in I2C FIFO.
    I2cResidualReceive,
    /// I2S configuration, clocks or merge readback differs from handoff.
    I2sConfiguration,
    /// I2S has inherited FIFO contents or its serializer clock is running.
    I2sBusy,
    /// Stereo sample count is odd, absent or payload exceeds its sample width.
    I2sFraming,
    /// Latched I2S FIFO underflow/overflow bits; no read-clear is performed.
    I2sHardware {
        /// Raw INTSTATUS word.
        raw: u32,
    },
}
/// Status observation only, not a bus-drain or buffer-coherency proof.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Completion {
    elements: u16,
}
impl Completion {
    /// Number of elements requested by this completed observation.
    pub fn elements(self) -> u16 {
        self.elements
    }
}
#[derive(Clone, Copy, PartialEq, Eq)]
enum State {
    Ready,
    Running,
    Retained,
}
#[derive(Default, Clone, Copy)]
struct Status {
    complete: bool,
    error: bool,
    enabled: bool,
    active: bool,
    remaining: u16,
    source: u32,
    destination: u32,
}
trait ControllerRegisters {
    fn idle(&mut self) -> bool;
    fn enable(&mut self);
    fn valid(&mut self) -> bool;
}
fn initialize(io: &mut impl ControllerRegisters) -> Result<(), Error> {
    if !io.idle() {
        return Err(Error::Occupied);
    }
    io.enable();
    if !io.valid() {
        return Err(Error::Configuration);
    }
    Ok(())
}
trait Registers {
    fn memory_readback(&self) -> MemoryReadback {
        MemoryReadback::SdkModel
    }
    fn prepare(&mut self) -> Result<(), Error> {
        Ok(())
    }
    fn peripheral_error(&mut self) -> Option<Error> {
        None
    }
    fn status(&mut self) -> Status;
    fn configure(&mut self, source: Range, destination: Range);
    fn valid(&mut self, source: Range) -> bool;
    fn start(&mut self);
    fn stop(&mut self);
}
trait InterruptRegisters: Registers {
    fn listen(&mut self, events: InterruptEvents, running: bool);
    fn acknowledge(&mut self, events: InterruptEvents);
}
struct Engine<R: Registers> {
    io: R,
    source: Range,
    destination: Range,
    state: State,
    stop_on_drop: bool,
}
impl<R: Registers> Engine<R> {
    fn start(&mut self) -> Result<(), Error> {
        if self.state != State::Ready {
            return Err(Error::NotReady);
        }
        let status = self.io.status();
        if status.enabled || status.active {
            self.state = State::Retained;
            return Err(Error::Occupied);
        }
        self.state = State::Retained;
        self.stop_on_drop = true;
        if let Err(error) = self.io.prepare() {
            self.stop();
            return Err(error);
        }
        self.io.configure(self.source, self.destination);
        let status = self.io.status();
        let result = if status.enabled
            || status.active
            || !self.io.valid(self.source)
            || status.remaining
                != match self.io.memory_readback() {
                    MemoryReadback::SdkModel => self.source.count,
                    MemoryReadback::ObservedIdleZeroLastAddress => 0,
                }
            || status.source != self.source.start
            || status.destination != self.destination.start
        {
            Err(Error::Configuration)
        } else if status.complete || status.error {
            Err(Error::StaleEvent)
        } else {
            Ok(())
        };
        if let Err(e) = result {
            self.stop();
            return Err(e);
        }
        compiler_fence(Ordering::Release);
        // In particular, I2C builders write commands after the allocation
        // handoff. Order those stores and channel MMIO before ENABLE. This
        // does not flush caches or establish non-coherent bus visibility.
        #[cfg(target_arch = "riscv32")]
        unsafe {
            core::arch::asm!("fence iorw, iorw", options(nostack));
        }
        self.io.start();
        self.state = State::Running;
        Ok(())
    }
    fn stop(&mut self) {
        if self.stop_on_drop {
            self.io.stop();
            self.stop_on_drop = false;
        }
        self.state = State::Retained;
    }
    fn poll(&mut self) -> Result<Option<Completion>, Error> {
        if self.state != State::Running {
            return Err(Error::NotReady);
        }
        let s = self.io.status();
        if s.error {
            return Err(Error::Bus);
        }
        if let Some(error) = self.io.peripheral_error() {
            return Err(error);
        }
        if !self.io.valid(self.source) {
            return Err(Error::Configuration);
        }
        if !s.complete {
            return Ok(None);
        }
        if s.enabled
            || s.active
            || s.remaining != 0
            || s.source
                != self.source.end
                    - if self.io.memory_readback() == MemoryReadback::ObservedIdleZeroLastAddress {
                        u32::from(self.source.width)
                    } else {
                        0
                    }
            || s.destination
                != self.destination.end
                    - if self.io.memory_readback() == MemoryReadback::ObservedIdleZeroLastAddress {
                        u32::from(self.destination.width)
                    } else {
                        0
                    }
        {
            return Err(Error::CompletionState);
        }
        Ok(Some(Completion { elements: self.source.count }))
    }
    fn wait(&mut self, polls: NonZeroU16, delay: &mut impl DelayNs) -> Result<Completion, Error> {
        if self.state != State::Running {
            return Err(Error::NotReady);
        }
        let mut result = Err(Error::Timeout);
        for n in 0..polls.get() {
            match self.poll() {
                Ok(Some(done)) => {
                    result = Ok(done);
                    break;
                }
                Err(error) => {
                    result = Err(error);
                    break;
                }
                Ok(None) => {}
            }
            if n + 1 < polls.get() {
                delay.delay_us(1);
            }
        }
        self.stop();
        result
    }
}
impl<R: InterruptRegisters> Engine<R> {
    fn listen(&mut self, events: InterruptEvents) -> Result<(), Error> {
        if self.state == State::Retained {
            return Err(Error::NotReady);
        }
        self.io.listen(events, self.state == State::Running);
        Ok(())
    }
    fn acknowledge(&mut self, events: InterruptEvents) -> Result<(), Error> {
        // Never erase the completion/error evidence while wait still needs it.
        if self.state != State::Retained {
            return Err(Error::NotReady);
        }
        self.io.acknowledge(events);
        Ok(())
    }
}
impl<R: Registers> Drop for Engine<R> {
    fn drop(&mut self) {
        self.stop();
    }
}
/// A one-shot equal-width incrementing memory copy, with retained resources.
/// No safe sample/reference/buffer-return method is available after starting.
///
/// ```compile_fail
/// use hisi_hal::dma::Transfer;
/// fn no_safe_reclaim(t:Transfer<'_,u32>) { t.release_after_external_quiescence(); }
/// ```
/// ```compile_fail
/// use hisi_hal::dma::Transfer;
/// fn no_dma_alias(t:&Transfer<'_,u32>) { t.destination(); }
/// ```
pub struct Transfer<'d, T: Element> {
    engine: Engine<PacRegisters>,
    channel: Option<Channel<'d>>,
    source: Option<Buffer<T>>,
    destination: Option<Buffer<T>>,
}
impl<'d, T: Element> Transfer<'d, T> {
    /// Combine same-length, disjoint allocations without MMIO. Invalid inputs
    /// return all three owners untouched. Only static buffers can be retained.
    #[allow(clippy::type_complexity)]
    pub fn new(
        channel: Channel<'d>,
        source: Buffer<T>,
        destination: Buffer<T>,
    ) -> Result<Self, (Error, Channel<'d>, Buffer<T>, Buffer<T>)> {
        let error = if source.range.count != destination.range.count {
            Some(Error::LengthMismatch)
        } else if source.range.overlaps(destination.range) {
            Some(Error::Overlap)
        } else {
            None
        };
        if let Some(e) = error {
            return Err((e, channel, source, destination));
        }
        Ok(Self {
            engine: Engine {
                io: PacRegisters {
                    memory_readback: channel.memory_readback,
                    ..PacRegisters::new(channel.id, Mode::Memory)
                },
                source: source.range,
                destination: destination.range,
                state: State::Ready,
                stop_on_drop: false,
            },
            channel: Some(channel),
            source: Some(source),
            destination: Some(destination),
        })
    }
    /// Select interrupt sources before start or while running, preserving raw flags.
    /// Does not enable local IRQ58 or install a handler. An ISR should disable
    /// local delivery and record status; run `wait` and acknowledge in normal
    /// context. A retained/stopped transfer cannot be rearmed by this method.
    pub fn listen(&mut self, events: InterruptEvents) -> Result<(), Error> {
        self.engine.listen(events)
    }
    /// Acknowledge selected events only after wait/request_stop retained resources.
    /// Does not release buffers, prove drain, or alter any other channel's events.
    pub fn acknowledge(&mut self, events: InterruptEvents) -> Result<(), Error> {
        self.engine.acknowledge(events)
    }
    /// Configure and arm this channel; failure retains both buffers and the token.
    pub fn start(&mut self) -> Result<(), Error> {
        self.engine.start()
    }
    /// Poll raw status with finite attempts, then request halt/disable. No global
    /// critical section or peripheral-clock shutdown; other channels continue.
    /// Completion cannot safely expose memory until external bus-drain proof.
    pub fn wait(&mut self, polls: NonZeroU16, delay: &mut impl DelayNs) -> Result<Completion, Error> {
        self.engine.wait(polls, delay)
    }
    /// Request halt/disable once, without waiting or returning allocations.
    pub fn request_stop(&mut self) {
        self.engine.stop();
    }
    /// Return the channel and both allocations after independent external drain.
    ///
    /// # Safety
    /// Independently establish that this channel and every old request/LLI/user
    /// can issue no future read or write until explicit rearming, and all posted
    /// source/destination transactions have completed. Preserve both coherent
    /// buffer contracts and perform necessary CPU/bus synchronization. Neither
    /// TC, ACTIVE/ENABLED, CPU halt nor an elapsed delay alone proves this.
    /// This method performs no MMIO, cache maintenance or implicit restart.
    pub unsafe fn release_after_external_quiescence(mut self) -> (Channel<'d>, Buffer<T>, Buffer<T>) {
        self.engine.stop_on_drop = false;
        self.engine.state = State::Retained;
        compiler_fence(Ordering::Acquire);
        (self.channel.take().unwrap(), self.source.take().unwrap(), self.destination.take().unwrap())
    }
}
struct PacController;
fn regs() -> &'static pac::mdma::RegisterBlock {
    unsafe { &*pac::Mdma::PTR }
}
impl ControllerRegisters for PacController {
    fn idle(&mut self) -> bool {
        regs().enabled_channels().read().channels().bits() == 0
            && regs().channel_iter().all(|c| {
                let s = c.configuration().read();
                !s.enable().bit() && !s.active().bit()
            })
    }
    fn enable(&mut self) {
        unsafe {
            interrupt::disable(interrupt::Interrupt::M_DMA);
        }
        // The handoff grants whole-controller ownership and little-endian masters.
        unsafe {
            regs()
                .configuration()
                .write_with_zero(|w| w.enable().set_bit().master1().clear_bit().master2().clear_bit());
            regs().interrupt_clear().write_with_zero(|w| w.transfer_complete().bits(255).error().bits(255));
        }
    }
    fn valid(&mut self) -> bool {
        let c = regs().configuration().read();
        c.enable().bit() && !c.master1().bit() && !c.master2().bit()
    }
}
struct PacRegisters {
    id: ChannelId,
    memory_readback: MemoryReadback,
    mode: Mode,
    observed_lines: u8,
    interrupts: InterruptEvents,
}
impl PacRegisters {
    fn new(id: ChannelId, mode: Mode) -> Self {
        Self {
            id,
            mode,
            memory_readback: MemoryReadback::SdkModel,
            observed_lines: 0,
            interrupts: InterruptEvents::None,
        }
    }
    fn ch(&self) -> &'static pac::mdma::Channel {
        regs().channel(self.id as usize)
    }
}
impl InterruptRegisters for PacRegisters {
    fn listen(&mut self, events: InterruptEvents, running: bool) {
        self.interrupts = events;
        if running {
            self.ch().configuration().modify(|_, w| {
                w.error_interrupt_mask().bit(events.error()).transfer_interrupt_mask().bit(events.completion())
            });
        }
    }
    fn acknowledge(&mut self, events: InterruptEvents) {
        unsafe {
            regs().interrupt_clear().write_with_zero(|w| {
                w.transfer_complete()
                    .bits(if events.completion() { self.id.mask() } else { 0 })
                    .error()
                    .bits(if events.error() { self.id.mask() } else { 0 })
            });
        }
    }
}
impl Registers for PacRegisters {
    fn memory_readback(&self) -> MemoryReadback {
        self.memory_readback
    }
    fn prepare(&mut self) -> Result<(), Error> {
        self.prepare_uart()
    }
    fn peripheral_error(&mut self) -> Option<Error> {
        self.sample_uart_error()
    }
    fn status(&mut self) -> Status {
        let raw = regs().raw_interrupt_status().read();
        let c = self.ch();
        let cfg = c.configuration().read();
        Status {
            complete: raw.transfer_complete().bits() & self.id.mask() != 0,
            error: raw.error().bits() & self.id.mask() != 0,
            enabled: cfg.enable().bit(),
            active: cfg.active().bit(),
            remaining: c.control().read().transfer_size().bits(),
            source: c.source().read().address().bits(),
            destination: c.destination().read().address().bits(),
        }
    }
    fn configure(&mut self, source: Range, destination: Range) {
        let c = self.ch();
        let (src_request, dst_request) = self.mode.requests();
        let (src_increment, dst_increment) = self.mode.increments();
        let width = match source.width {
            1 => 0,
            2 => 1,
            _ => 2,
        };
        unsafe {
            c.configuration().write_with_zero(|w| {
                w.enable()
                    .clear_bit()
                    .source_request()
                    .bits(src_request)
                    .destination_request()
                    .bits(dst_request)
                    .flow_control()
                    .bits(self.mode.flow())
                    .error_interrupt_mask()
                    .bit(self.interrupts.error())
                    .transfer_interrupt_mask()
                    .bit(self.interrupts.completion())
                    .lock()
                    .clear_bit()
                    .halt()
                    .clear_bit()
            });
            c.linked_list().write_with_zero(|w| w.master().clear_bit().address().bits(0));
            c.source().write_with_zero(|w| w.address().bits(source.start));
            c.destination().write_with_zero(|w| w.address().bits(destination.start));
            c.control().write_with_zero(|w| {
                w.transfer_size()
                    .bits(source.count)
                    .source_burst()
                    .bits(self.mode.burst())
                    .destination_burst()
                    .bits(self.mode.burst())
                    .source_width()
                    .bits(width)
                    .destination_width()
                    .bits(width)
                    .source_master()
                    .clear_bit()
                    .destination_master()
                    .clear_bit()
                    .source_increment()
                    .bit(src_increment)
                    .destination_increment()
                    .bit(dst_increment)
                    .protection()
                    .bits(self.mode.protection())
                    .terminal_interrupt()
                    .set_bit()
            });
            regs()
                .interrupt_clear()
                .write_with_zero(|w| w.transfer_complete().bits(self.id.mask()).error().bits(self.id.mask()));
        }
    }
    fn valid(&mut self, source: Range) -> bool {
        let c = self.ch();
        let cfg = c.configuration().read();
        let ctrl = c.control().read();
        let lli = c.linked_list().read();
        let (src_request, dst_request) = self.mode.requests();
        let (src_increment, dst_increment) = self.mode.increments();
        let width = match source.width {
            1 => 0,
            2 => 1,
            _ => 2,
        };
        PacController.valid()
            && self.uart_valid()
            && !lli.master().bit()
            && lli.address().bits() == 0
            && cfg.flow_control().bits() == self.mode.flow()
            && cfg.source_request().bits() == src_request
            && cfg.destination_request().bits() == dst_request
            && !cfg.lock().bit()
            && !cfg.halt().bit()
            && cfg.error_interrupt_mask().bit() == self.interrupts.error()
            && cfg.transfer_interrupt_mask().bit() == self.interrupts.completion()
            && ctrl.source_burst().bits() == self.mode.burst()
            && ctrl.destination_burst().bits() == self.mode.burst()
            && ctrl.source_width().bits() == width
            && ctrl.destination_width().bits() == width
            && !ctrl.source_master().bit()
            && !ctrl.destination_master().bit()
            && ctrl.source_increment().bit() == src_increment
            && ctrl.destination_increment().bit() == dst_increment
            && ctrl.protection().bits() == self.mode.protection()
            && ctrl.terminal_interrupt().bit()
    }
    fn start(&mut self) {
        self.ch().configuration().modify(|_, w| w.enable().set_bit());
    }
    fn stop(&mut self) {
        self.ch().configuration().modify(|_, w| {
            w.halt()
                .set_bit()
                .enable()
                .clear_bit()
                .error_interrupt_mask()
                .clear_bit()
                .transfer_interrupt_mask()
                .clear_bit()
        });
        // Do not clear raw status here: retain the diagnostic evidence and avoid
        // any whole-controller reset/IRQ mutation that could affect another owner.
    }
}
#[cfg(all(test, not(target_arch = "riscv32")))]
#[path = "dma_ws53/tests.rs"]
mod tests;
