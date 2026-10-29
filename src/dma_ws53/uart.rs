//! UART0/1 request-paced byte DMA with retained peripheral and memory ownership.
//!
//! These are experimental, one-shot transfers (polling or optional async), not embedded-io reads or
//! writes: completion observes MDMA status and does not return memory or certify
//! serial integrity. No writable DMA_EXTRA enable is invented. Physical request
//! pacing, byte lanes, clocks and coherency must be established externally.
use super::{Buffer, Channel, Completion, Engine, Error, Mode, PacRegisters, Range, State};
use crate::{
    interrupt,
    peripherals::{Uart0, Uart1},
    soc::pac,
    uart::{UartInstance, UartPort},
};
use core::{
    marker::PhantomData,
    num::NonZeroU16,
    sync::atomic::{Ordering, compiler_fence},
};
use embedded_hal::delay::DelayNs;

#[cfg(feature = "async")]
#[path = "uart_async.rs"]
mod asynch;
#[cfg(feature = "async")]
pub use asynch::{AsyncTransmitterIdle, AsyncUartTransfer};

mod sealed {
    pub trait Instance {}
    impl Instance for super::Uart0<'_> {}
    impl Instance for super::Uart1<'_> {}
}
/// Sealed UART instances with an actual WS53 MDMA request mapping.
/// UART2 has no valid port mapping and cannot implement this trait.
pub trait Instance: UartInstance + sealed::Instance {}
impl Instance for Uart0<'_> {}
impl Instance for Uart1<'_> {}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) enum Port {
    Uart0,
    Uart1,
}
impl Port {
    pub(super) fn request(self, receive: bool) -> u8 {
        match (self, receive) {
            (Self::Uart0, false) => 10,
            (Self::Uart0, true) => 11,
            (Self::Uart1, false) => 8,
            (Self::Uart1, true) => 9,
        }
    }
    fn of<T: Instance>() -> Self {
        // The additional sealed bound excludes UART2, even though UartInstance
        // alone includes it. No raw port/channel/address is a public input.
        match T::PORT {
            UartPort::Uart0 => Self::Uart0,
            _ => Self::Uart1,
        }
    }
    fn regs(self) -> &'static pac::uart0::RegisterBlock {
        unsafe {
            &*match self {
                Self::Uart0 => Uart0::ptr(),
                Self::Uart1 => Uart1::ptr(),
            }
        }
    }
    fn interrupt(self) -> interrupt::Interrupt {
        match self {
            Self::Uart0 => interrupt::Interrupt::UART_0,
            Self::Uart1 => interrupt::Interrupt::UART_1,
        }
    }
}

/// Line flags observed and accumulated by this transfer's status reads.
/// DMA can consume a character before polling observes its error. An empty
/// report is not proof that every received byte passed framing/parity checks.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct LineErrors(u8);
impl LineErrors {
    fn from_flags(flags: u8) -> Option<Self> {
        if flags & 31 != 0 { Some(Self(flags & 31)) } else { None }
    }
    /// At least one queued FIFO character was reported erroneous.
    pub fn fifo_error(self) -> bool {
        self.0 & 1 != 0
    }
    /// A framing error was observed.
    pub fn framing(self) -> bool {
        self.0 & 2 != 0
    }
    /// A parity error was observed.
    pub fn parity(self) -> bool {
        self.0 & 4 != 0
    }
    /// Receive data was lost to FIFO overrun.
    pub fn overrun(self) -> bool {
        self.0 & 8 != 0
    }
    /// A break condition was observed.
    pub fn break_detected(self) -> bool {
        self.0 & 16 != 0
    }
}

/// Exclusive UART reservation for one direction at a time. Construction performs
/// no MMIO, and dropping/forgetting it does not produce another UART owner.
///
/// ```compile_fail
/// use hisi_hal::{dma::uart::UartDma, peripherals::Uart2};
/// fn unsupported(p:Uart2<'static>) { unsafe { UartDma::from_configured(p) }; }
/// ```
/// ```compile_fail
/// use hisi_hal::{dma::uart::UartDma, peripherals::{Uart0,Unclaimed}};
/// fn unowned(p:Unclaimed<Uart0<'static>>) { unsafe { UartDma::from_configured(p) }; }
/// ```
pub struct UartDma<'d, T: Instance> {
    token: T,
    _owner: PhantomData<&'d mut T>,
}
impl<'d, T: Instance> UartDma<'d, T> {
    /// Reserve a configured UART without changing clocks, pads or registers.
    /// Start validates capability, 8N1/FIFO mode, empty queues and idle transmitter.
    ///
    /// # Safety
    /// Own the entire UART, both request lines and local IRQ; all old CPU/ISR/DMA
    /// and control-core users must be stopped and drained. Keep pads, baud, bus
    /// and peripheral clocks valid through cleanup; prevent shared reset/sleep.
    /// Establish byte DATA access via little-endian MDMA master0/protection0,
    /// flow1/2 and single-beat FIFO pacing with TX-empty/RX-one-character trigger
    /// configuration. DMA_EXTRA is a read-only capability, not an enable knob.
    /// The reference SDK disables UART DMA and has conflicting DMA_EXTRA/HPROT
    /// helpers; the capability read alone does not prove the above behavior.
    /// Coordinate the peer: no incoming data before start's initial idle guard.
    /// Polling does not provide complete framing/parity error capture. Do not
    /// require a lossless/error-certified stream from this experimental interface.
    pub unsafe fn from_configured(token: T) -> Self {
        Self { token, _owner: PhantomData }
    }
    /// Give a static byte source and channel to one transmit operation. No MMIO.
    ///
    /// ```compile_fail
    /// use hisi_hal::{dma::{Buffer,Channel,uart::UartDma},peripherals::Uart0};
    /// fn wrong_width(u:UartDma<'static,Uart0<'static>>,c:Channel<'static>,b:Buffer<u16>) { u.transmit(c,b); }
    /// ```
    pub fn transmit(self, channel: Channel<'d>, buffer: Buffer<u8>) -> UartTransfer<'d, T> {
        self.transfer(channel, buffer, false)
    }
    /// Give a static byte destination and channel to one receive operation.
    /// FIFO-empty waits are bounded by `wait`; the peer may continue after cancel.
    pub fn receive(self, channel: Channel<'d>, buffer: Buffer<u8>) -> UartTransfer<'d, T> {
        self.transfer(channel, buffer, true)
    }
    fn transfer(self, channel: Channel<'d>, buffer: Buffer<u8>, receive: bool) -> UartTransfer<'d, T> {
        let port = Port::of::<T>();
        // DMA address exports from PAC accessors, never numeric MMIO pointers.
        let address = if receive { port.regs().rx().as_ptr() as u32 } else { port.regs().tx().as_ptr() as u32 };
        let peripheral = Range { start: address, end: address, count: buffer.range.count, width: 1 };
        let (source, destination) = if receive { (peripheral, buffer.range) } else { (buffer.range, peripheral) };
        UartTransfer {
            engine: Engine {
                io: PacRegisters::new(channel.id, Mode::Uart { port, receive }),
                source,
                destination,
                state: State::Ready,
                stop_on_drop: false,
            },
            token: Some(self.token),
            channel: Some(channel),
            buffer: Some(buffer),
            completed: false,
        }
    }
}

/// A single UART byte transfer owns the complete UART, channel and static buffer.
/// Neither completion, cancellation, Drop nor forgetting makes these reusable.
///
/// ```compile_fail
/// use hisi_hal::{dma::uart::UartTransfer,peripherals::Uart0};
/// fn no_safe_reclaim(t:UartTransfer<'static,Uart0<'static>>) { t.release_after_external_quiescence(); }
/// ```
pub struct UartTransfer<'d, T: Instance> {
    engine: Engine<PacRegisters>,
    token: Option<T>,
    channel: Option<Channel<'d>>,
    buffer: Option<Buffer<u8>>,
    completed: bool,
}
impl<'d, T: Instance> UartTransfer<'d, T> {
    /// Guard UART capability/state, disable local UART notifications, configure
    /// FIFO trigger levels without resetting either FIFO, then arm this channel.
    /// Every attempted start retains all resources, including on failure.
    pub fn start(&mut self) -> Result<(), Error> {
        self.engine.start()
    }
    /// Bounded status polling, followed by halt/disable request. Observed line
    /// errors are reported, but DMA may consume error flags between polls. A
    /// success is only element progress, never a received-data integrity proof.
    pub fn wait(&mut self, polls: NonZeroU16, delay: &mut impl DelayNs) -> Result<Completion, Error> {
        let result = self.engine.wait(polls, delay);
        self.completed |= result.is_ok();
        result
    }
    /// Request channel halt/disable once. Does not reset the UART, retract a
    /// posted peripheral write, stop the peer or return memory/peripheral tokens.
    pub fn request_stop(&mut self) {
        self.engine.stop();
    }
    /// After successful TX completion, observe FIFO and shift register empty
    /// within a finite budget. This separate observation is not bus drain: a
    /// delayed DMA beat may still arrive under an unverified physical profile.
    /// It never returns the UART or buffer. RX or uncompleted TX returns NotReady.
    pub fn wait_transmitter_idle(&mut self, polls: NonZeroU16, delay: &mut impl DelayNs) -> Result<(), Error> {
        if !self.completed || self.engine.io.mode.flow() != 1 {
            return Err(Error::NotReady);
        }
        wait_idle(&mut self.engine.io, polls, delay)
    }
    /// Return all three owners after independent physical recovery; no MMIO.
    ///
    /// # Safety
    /// Stop the peer and prove no old channel/request/LLI/controller user can
    /// issue further DATA or buffer accesses; drain all posted reads/writes,
    /// including peripheral writes, and preserve the buffer coherency contract.
    /// Reconcile the UART FIFO/shift state and any consumed line-error history
    /// before reattaching another driver. The returned raw token does not promise
    /// normal UART configuration or an intact receive stream. TC, FIFO empty,
    /// TX_EMPTY, ACTIVE/ENABLED, CPU halt or delay alone are insufficient.
    pub unsafe fn release_after_external_quiescence(mut self) -> (T, Channel<'d>, Buffer<u8>) {
        self.engine.stop_on_drop = false;
        self.engine.state = State::Retained;
        compiler_fence(Ordering::Acquire);
        (self.token.take().unwrap(), self.channel.take().unwrap(), self.buffer.take().unwrap())
    }
}

struct Snapshot {
    capability: bool,
    configured: bool,
    tx_idle: bool,
    tx_count: u8,
    rx_count: u8,
    lines: u8,
}
impl Snapshot {
    fn guard(&self) -> Result<(), Error> {
        if !self.capability {
            return Err(Error::UartCapability);
        }
        if !self.configured {
            return Err(Error::UartConfiguration);
        }
        if let Some(e) = LineErrors::from_flags(self.lines) {
            return Err(Error::UartLine(e));
        }
        if !self.tx_idle || self.tx_count != 0 || self.rx_count != 0 {
            return Err(Error::UartBusy);
        }
        Ok(())
    }
}
fn profile(port: Port) -> (bool, bool) {
    let r = port.regs();
    let p = r.uart_parameter().read();
    let c = r.uart_ctl().read();
    // All three observed WS53 ports report 0xD04: FIFO64, DMA and APB width
    // code 1 (16 bits). This parameter is not the CPU register access width,
    // and accepting it does not qualify the bridge's byte DMA transactions.
    let capability = supported_capability(p.dma_mode().bit(), p.fifo_depth().bits(), p.apb_data_width().bits());
    let configured = r.intr_id().read().fifo_en_s().bit()
        && c.dlen().is_bits8()
        && !c.div_en().bit()
        && !c.xbreak().bit()
        && !c.pen().bit()
        && !c.stp().bit()
        && r.modem_ctl().read().bits() & !2 == 0;
    (capability, configured)
}
fn supported_capability(dma: bool, depth: u8, apb_width: u8) -> bool {
    dma && depth == 4 && apb_width == 1
}
trait LineRegisters {
    fn observe(&mut self) -> (bool, u8);
}
fn wait_idle(io: &mut impl LineRegisters, polls: NonZeroU16, delay: &mut impl DelayNs) -> Result<(), Error> {
    for n in 0..polls.get() {
        let (idle, flags) = io.observe();
        if let Some(e) = LineErrors::from_flags(flags) {
            return Err(Error::UartLine(e));
        }
        if idle {
            return Ok(());
        }
        if n + 1 < polls.get() {
            delay.delay_us(1);
        }
    }
    Err(Error::TransmitTimeout)
}
impl LineRegisters for PacRegisters {
    fn observe(&mut self) -> (bool, u8) {
        let Mode::Uart { port, .. } = self.mode else {
            return (false, 0);
        };
        let r = port.regs();
        let s = r.line_status().read();
        self.observed_lines |= u8::from(s.rx_fifo_err().bit())
            | (u8::from(s.frame_err().bit()) << 1)
            | (u8::from(s.parity_err().bit()) << 2)
            | (u8::from(s.overrun_err().bit()) << 3)
            | (u8::from(s.break_intr().bit()) << 4);
        (s.tx_empty_s().bit() && r.tx_fifo_cnt().read().tx_fifo_level().bits() == 0, self.observed_lines)
    }
}
impl PacRegisters {
    pub(super) fn prepare_uart(&mut self) -> Result<(), Error> {
        let Mode::Uart { port, .. } = self.mode else {
            return Ok(());
        };
        let (capability, configured) = profile(port);
        let (tx_idle, lines) = self.observe();
        let r = port.regs();
        Snapshot {
            capability,
            configured,
            tx_idle,
            lines,
            tx_count: r.tx_fifo_cnt().read().tx_fifo_level().bits(),
            rx_count: r.rx_fifo_cnt().read().rx_fifo_level().bits(),
        }
        .guard()?;
        unsafe {
            interrupt::disable(port.interrupt());
        }
        unsafe {
            r.intr_en().write_with_zero(|w| w);
            r.fifo_ctl().write_with_zero(|w| w.fifo_en().set_bit().tx_empty_trig().empty().rx_empty_trig().char1());
        }
        if !self.uart_valid() {
            return Err(Error::UartConfiguration);
        }
        Ok(())
    }
    pub(super) fn uart_valid(&self) -> bool {
        let Mode::Uart { port, .. } = self.mode else {
            return true;
        };
        let (capability, configured) = profile(port);
        capability && configured && port.regs().intr_en().read().bits() == 0
    }
    pub(super) fn sample_uart_error(&mut self) -> Option<Error> {
        if matches!(self.mode, Mode::Memory) {
            return None;
        }
        LineErrors::from_flags(self.observe().1).map(Error::UartLine)
    }
}

#[cfg(all(test, not(target_arch = "riscv32")))]
#[path = "uart_tests.rs"]
mod tests;
