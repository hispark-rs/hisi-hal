//! Experimental single-line full-duplex SPI0/QSPI1 DMA. No physical drain proof.
//! One owner retains both DMA channels, all routes and both static buffers.
use super::{Buffer, Channel, Completion, Element, Engine, Error, Mode, PacRegisters, Range, Registers, State};
use crate::{
    interrupt,
    peripherals::{Qspi1, Spi0},
    soc::pac,
    spi::{Pins, Word},
};
use core::{
    marker::PhantomData,
    num::NonZeroU16,
    sync::atomic::{Ordering, compiler_fence},
};
use embedded_hal::delay::DelayNs;
mod sealed {
    pub trait Instance {}
    impl Instance for super::Spi0<'_> {}
    impl Instance for super::Qspi1<'_> {}
}
/// Instances with audited SPI DMA request routes. Cannot be implemented externally.
pub trait Instance: sealed::Instance {
    #[doc(hidden)]
    const QSPI: bool;
}
impl Instance for Spi0<'_> {
    const QSPI: bool = false;
}
impl Instance for Qspi1<'_> {
    const QSPI: bool = true;
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) enum Port {
    Spi,
    Qspi,
}
impl Port {
    pub(super) fn request(self, rx: bool) -> u8 {
        (if self == Self::Spi { 6 } else { 4 }) + u8::from(rx)
    }
    fn of<T: Instance>() -> Self {
        if T::QSPI { Self::Qspi } else { Self::Spi }
    }
}
macro_rules! regs {
    ($port:expr,$r:ident,$body:block) => {
        match $port {
            Port::Spi => {
                let $r = unsafe { &*Spi0::ptr() };
                $body
            }
            Port::Qspi => {
                let $r = unsafe { &*Qspi1::ptr() };
                $body
            }
        }
    };
}
/// Own an already configured, enabled, idle SPI master and its three routes.
/// Construction makes no MMIO access; this does not configure a board.
///
/// ```compile_fail
/// use hisi_hal::{dma::spi::SpiDma,peripherals::Spi0,spi::Pins};
/// fn unsupported(p:Spi0<'static>,pins:Pins<'static,Spi0<'static>>) {
///     unsafe { SpiDma::<_,u16>::from_configured(p,pins) };
/// }
/// ```
pub struct SpiDma<'d, T: Instance, W: Word + Element> {
    token: T,
    pins: Pins<'d, T>,
    _word: PhantomData<W>,
}
impl<'d, T: Instance, W: Word + Element> SpiDma<'d, T, W> {
    /// Accept exclusive ownership without starting or changing the peripheral.
    ///
    /// # Safety
    /// Own SPI, both requests and local IRQ, clock/reset controls and the three
    /// single-line routes. Stop and drain all previous CPU/ISR/DMA/control-core
    /// users; QSPI XIP must not be used. Keep the enabled master, stable clocks,
    /// selected peripheral and external software CS valid through recovery.
    /// Native CS outputs must be unrouted; this API does not promise their
    /// continuity. Establish the byte/word DATA bridge, master0/protection2,
    /// single-beat pacing and TXlevel4/RXlevel0 against the actual FIFO depth.
    /// The reference SDK disables SPI; source enums and QEMU are not silicon
    /// evidence. The peer must tolerate clocks only while this operation owns it.
    pub unsafe fn from_configured(token: T, pins: Pins<'d, T>) -> Self {
        Self { token, pins, _word: PhantomData }
    }
    /// Pair disjoint, equal-length static buffers and two unique DMA channels.
    /// No MMIO; invalid input returns every owner. No in-place aliasing is allowed.
    #[allow(clippy::type_complexity)]
    pub fn transfer(
        self,
        tx: Channel<'d>,
        rx: Channel<'d>,
        source: Buffer<W>,
        destination: Buffer<W>,
    ) -> Result<SpiTransfer<'d, T, W>, (Error, Self, Channel<'d>, Channel<'d>, Buffer<W>, Buffer<W>)> {
        let error = if source.range.count != destination.range.count {
            Some(Error::LengthMismatch)
        } else if source.range.overlaps(destination.range) {
            Some(Error::Overlap)
        } else {
            None
        };
        if let Some(e) = error {
            return Err((e, self, tx, rx, source, destination));
        }
        let port = Port::of::<T>();
        let addr = regs!(port, r, { r.tx(0).as_ptr() as u32 });
        let peripheral = Range { start: addr, end: addr, count: source.range.count, width: source.range.width };
        let engine = |id, receive, src, dst| Engine {
            io: PacRegisters::new(id, Mode::Spi { port, receive }),
            source: src,
            destination: dst,
            state: State::Ready,
            stop_on_drop: false,
        };
        Ok(SpiTransfer {
            pair: Pair {
                tx: engine(tx.id, false, source.range, peripheral),
                rx: engine(rx.id, true, peripheral, destination.range),
                io: PacSpi { port, width: source.range.width },
                state: State::Ready,
                requests_owned: false,
            },
            owner: Some(self),
            tx: Some(tx),
            rx: Some(rx),
            source: Some(source),
            destination: Some(destination),
        })
    }
    /// Return reservations after unused construction or externally proven recovery.
    pub fn into_parts(self) -> (T, Pins<'d, T>) {
        (self.token, self.pins)
    }
}
/// Full-duplex one-shot operation. Success, errors, cancellation, Drop and forget
/// retain all resources. Never safe to access either allocation during transfer.
///
/// ```compile_fail
/// use hisi_hal::{dma::spi::SpiTransfer,peripherals::Spi0};
/// fn reclaim(t:SpiTransfer<'static,Spi0<'static>,u8>) { t.release_after_external_quiescence(); }
/// ```
pub struct SpiTransfer<'d, T: Instance, W: Word + Element> {
    pair: Pair<PacRegisters, PacSpi>,
    owner: Option<SpiDma<'d, T, W>>,
    tx: Option<Channel<'d>>,
    rx: Option<Channel<'d>>,
    source: Option<Buffer<W>>,
    destination: Option<Buffer<W>>,
}
impl<'d, T: Instance, W: Word + Element> SpiTransfer<'d, T, W> {
    /// Guard both channels and idle peripheral; arm RX then TX with peripheral
    /// requests disabled, then open requests. Any failed attempt retains owners.
    pub fn start(&mut self) -> Result<(), Error> {
        self.pair.start()
    }
    /// Poll both DMA channels, latched SPI errors and wire-idle within one budget.
    /// Completion observes progress, not physical drain/coherency or CS framing.
    pub fn wait(&mut self, polls: NonZeroU16, delay: &mut impl DelayNs) -> Result<Completion, Error> {
        self.pair.wait(polls, delay)
    }
    /// Disable both peripheral requests and request both channels halted once.
    /// Does not flush FIFO, stop clocks/peer or revoke an in-flight bus access.
    pub fn request_stop(&mut self) {
        self.pair.stop();
    }
    /// Return owners after independent recovery. No register writes or reset.
    ///
    /// # Safety
    /// Stop the peer and prove that no old DMA/request/controller/CPU/ISR can
    /// access DATA, pads or buffers. Independently drain all posted reads/writes,
    /// complete serial traffic and reconcile FIFO/errors/CS. Maintain buffer
    /// coherency. TC, ACTIVE, ENABLED, idle observation or a delay is insufficient.
    /// Returned SpiDma must again satisfy its configured/idle contract before use.
    #[allow(clippy::type_complexity)]
    pub unsafe fn release_after_external_quiescence(
        mut self,
    ) -> (SpiDma<'d, T, W>, Channel<'d>, Channel<'d>, Buffer<W>, Buffer<W>) {
        self.pair.requests_owned = false;
        self.pair.tx.stop_on_drop = false;
        self.pair.rx.stop_on_drop = false;
        compiler_fence(Ordering::Acquire);
        (
            self.owner.take().unwrap(),
            self.tx.take().unwrap(),
            self.rx.take().unwrap(),
            self.source.take().unwrap(),
            self.destination.take().unwrap(),
        )
    }
}
#[derive(Clone, Copy)]
struct Snapshot {
    configured: bool,
    idle: bool,
    rx_count: u32,
    requests: u8,
    error: Option<Error>,
}
impl Snapshot {
    fn guard(self) -> Result<(), Error> {
        if !self.configured {
            return Err(Error::SpiConfiguration);
        }
        if let Some(e) = self.error {
            return Err(e);
        }
        if !self.idle || self.rx_count != 0 || self.requests != 0 {
            return Err(Error::SpiBusy);
        }
        Ok(())
    }
}
trait SpiRegisters {
    fn snapshot(&mut self) -> Snapshot;
    fn prepare(&mut self);
    fn requests(&mut self, on: bool);
    fn valid(&mut self, on: bool) -> bool;
}
struct Pair<R: Registers, P: SpiRegisters> {
    tx: Engine<R>,
    rx: Engine<R>,
    io: P,
    state: State,
    requests_owned: bool,
}
impl<R: Registers, P: SpiRegisters> Pair<R, P> {
    fn start(&mut self) -> Result<(), Error> {
        if self.state != State::Ready {
            return Err(Error::NotReady);
        }
        self.state = State::Retained;
        let tx = self.tx.io.status();
        let rx = self.rx.io.status();
        if tx.active || tx.enabled || rx.active || rx.enabled {
            return Err(Error::Occupied);
        }
        self.io.snapshot().guard()?;
        self.requests_owned = true;
        self.io.prepare();
        let result = (|| {
            if !self.io.valid(false) {
                return Err(Error::SpiConfiguration);
            }
            self.rx.start()?;
            self.tx.start()?;
            self.io.requests(true);
            if !self.io.valid(true) {
                return Err(Error::SpiConfiguration);
            }
            Ok(())
        })();
        if result.is_err() {
            self.stop();
        } else {
            self.state = State::Running;
        }
        result
    }
    fn stop(&mut self) {
        if self.requests_owned {
            self.io.requests(false);
            self.requests_owned = false;
        }
        self.tx.stop();
        self.rx.stop();
        self.state = State::Retained;
    }
    fn wait(&mut self, polls: NonZeroU16, delay: &mut impl DelayNs) -> Result<Completion, Error> {
        if self.state != State::Running {
            return Err(Error::NotReady);
        }
        let mut result = Err(Error::Timeout);
        for n in 0..polls.get() {
            let tx = self.tx.poll();
            let rx = self.rx.poll();
            let fault = match (tx, rx) {
                (Err(Error::Bus), _) | (_, Err(Error::Bus)) => Some(Error::Bus),
                (Err(e), _) | (_, Err(e)) => Some(e),
                _ => None,
            };
            if let Some(e) = fault {
                result = Err(e);
                break;
            }
            let s = self.io.snapshot();
            if let Some(e) = s.error {
                result = Err(e);
                break;
            }
            if !s.configured || !self.io.valid(true) {
                result = Err(Error::SpiConfiguration);
                break;
            }
            if let (Ok(Some(done)), Ok(Some(_))) = (tx, rx) {
                if s.rx_count != 0 {
                    result = Err(Error::SpiResidualReceive);
                    break;
                }
                if s.idle {
                    result = Ok(done);
                    break;
                }
                result = Err(Error::TransmitTimeout);
            }
            if n + 1 < polls.get() {
                delay.delay_us(1);
            }
        }
        self.stop();
        result
    }
}
impl<R: Registers, P: SpiRegisters> Drop for Pair<R, P> {
    fn drop(&mut self) {
        self.stop();
    }
}
struct PacSpi {
    port: Port,
    width: u8,
}
impl SpiRegisters for PacSpi {
    fn snapshot(&mut self) -> Snapshot {
        let clocks = unsafe { &*pac::MCrg::PTR };
        let clocked = match self.port {
            Port::Spi => {
                clocks.spi_clock().read().enable().bit()
                    && clocks.m_clken1().read().spi3_ms_enable().bit()
                    && clocks.spi_device_mode().read().master().bit()
            }
            Port::Qspi => {
                let c = clocks.xip_clock_enable().read();
                c.qspi1_divider_enable().bit()
                    && c.qspi1_divider_clock_enable().bit()
                    && c.qspi1_enable().bit()
                    && clocks.xip_qspi_clock().read().control().bits() == 1
                    && clocks.global_clken().read().xip_qspi_enable().bit()
            }
        };
        regs!(self.port, r, {
            let c = r.control().read();
            let s = r.status().read();
            let e = r.raw_interrupt().read();
            let divider = r.baud().read().value().bits();
            let dma = r.dma_control().read();
            Snapshot {
                configured: clocked
                    && r.enable().read().start().bit()
                    && r.slave_enable().read().value().bits() == 1
                    && divider >= 2
                    && divider & 1 == 0
                    && c.frame_size32().bits() == self.width * 8 - 1
                    && !c.slave_output_disable().bit()
                    && !c.loopback().bit()
                    && !c.select_toggle().bit()
                    && c.control_frame_size().bits() == 0
                    && c.frame_size16().bits() == 0
                    && c.transfer_mode().bits() == 0
                    && c.frame_format().bits() == 0
                    && c.protocol().bits() == 0
                    && r.frame_count().read().value().bits() == 0
                    && r.enhanced_control().read().bits() == 0
                    && r.microwire().read().bits() == 0,
                idle: !s.busy().bit() && s.tx_empty().bit() && r.tx_level().read().value().bits() == 0,
                rx_count: r.rx_level().read().value().bits(),
                requests: u8::from(dma.tx_enable().bit()) | u8::from(dma.rx_enable().bit()) << 1,
                error: if let Some(status) = crate::spi::collision_status(matches!(self.port, Port::Qspi), s.bits()) {
                    Some(Error::SpiCollision { status, raw_interrupt: e.bits() })
                } else if e.multi_master().bit()
                    || e.rx_overflow().bit()
                    || e.rx_underflow().bit()
                    || e.tx_overflow().bit()
                {
                    Some(Error::SpiHardware { raw: e.bits() })
                } else {
                    None
                },
            }
        })
    }
    fn prepare(&mut self) {
        unsafe {
            interrupt::disable(match self.port {
                Port::Spi => interrupt::Interrupt::SPI_3,
                Port::Qspi => interrupt::Interrupt::QSPI0_2CS,
            });
        }
        self.requests(false);
        regs!(self.port, r, {
            unsafe {
                r.interrupt_mask().write_with_zero(|w| w);
                r.dma_tx_level().write_with_zero(|w| w.value().bits(4));
                r.dma_rx_level().write_with_zero(|w| w.value().bits(0));
            }
        });
    }
    fn requests(&mut self, on: bool) {
        regs!(self.port, r, {
            unsafe {
                r.dma_control().write_with_zero(|w| w.tx_enable().bit(on).rx_enable().bit(on));
            }
        });
    }
    fn valid(&mut self, on: bool) -> bool {
        regs!(self.port, r, {
            let d = r.dma_control().read();
            d.tx_enable().bit() == on
                && d.rx_enable().bit() == on
                && r.dma_tx_level().read().value().bits() == 4
                && r.dma_rx_level().read().value().bits() == 0
                && r.interrupt_mask().read().bits() == 0
        })
    }
}

#[cfg(all(test, not(target_arch = "riscv32")))]
#[path = "spi_tests.rs"]
mod tests;
