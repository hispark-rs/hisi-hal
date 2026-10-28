//! Experimental I2C DMA command streams with unified request/channel ownership.
//! Read commands are halfwords; received data is bytes. No physical drain proof.
use super::{Buffer, Channel, Completion, Engine, Error, Mode, PacRegisters, Range, Registers, State};
use crate::{
    i2c::{Address, Pins},
    interrupt,
    peripherals::{I2c0, I2c1},
    soc::pac,
};
use core::{
    num::NonZeroU16,
    sync::atomic::{Ordering, compiler_fence},
};
use embedded_hal::delay::DelayNs;
mod sealed {
    pub trait Instance {}
    impl Instance for super::I2c0<'_> {}
    impl Instance for super::I2c1<'_> {}
}
/// The two source-confirmed I2C instances; externally implementing this is forbidden.
pub trait Instance: sealed::Instance {
    #[doc(hidden)]
    const SECOND: bool;
}
impl Instance for I2c0<'_> {
    const SECOND: bool = false;
}
impl Instance for I2c1<'_> {
    const SECOND: bool = true;
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) enum Port {
    Zero,
    One,
}
impl Port {
    fn of<T: Instance>() -> Self {
        if T::SECOND { Self::One } else { Self::Zero }
    }
    pub(super) fn request(self, rx: bool) -> u8 {
        (if self == Self::Zero { 0 } else { 2 }) + u8::from(rx)
    }
    fn irq(self) -> interrupt::Interrupt {
        match self {
            Self::Zero => interrupt::Interrupt::I2C_0,
            Self::One => interrupt::Interrupt::I2C_1,
        }
    }
}
/// Explicitly selected DMA pacing; actual hardware support belongs to handoff.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Burst {
    /// One beat per threshold request, receive watermark zero.
    One,
    /// Source four-beat profile; receive watermark3, lowered for the final1..3 bytes.
    Four,
}
impl Burst {
    pub(super) fn encoding(self) -> u8 {
        u8::from(self == Self::Four)
    }
    fn threshold(self) -> u32 {
        if self == Self::Four { 3 } else { 0 }
    }
}
macro_rules! regs {
    ($port:expr,$r:ident,$body:block) => {
        match $port {
            Port::Zero => {
                let $r = unsafe { &*I2c0::ptr() };
                $body
            }
            Port::One => {
                let $r = unsafe { &*I2c1::ptr() };
                $body
            }
        }
    };
}
/// Immutable owned halfword commands. Constructors emit exactly one final STOP,
/// and a repeated START only at the write/read direction boundary.
/// Raw command words cannot be submitted through this type.
pub struct Commands {
    buffer: Buffer<u16>,
    reads: u16,
}
impl Commands {
    /// Fill the whole buffer with read commands. Buffer length is receive length.
    pub fn read(buffer: Buffer<u16>) -> Self {
        let reads = buffer.range.count;
        Self::build(buffer, &[], reads).ok().unwrap()
    }
    /// Copy bytes into an equal-length command buffer, ending with STOP.
    /// Size mismatch returns the original allocation without mutation.
    pub fn write(buffer: Buffer<u16>, bytes: &[u8]) -> Result<Self, (Error, Buffer<u16>)> {
        Self::build(buffer, bytes, 0)
    }
    /// Copy writes followed by `reads` read commands with a repeated START.
    /// Requires nonempty writes and exactly `writes.len()+reads` command slots.
    pub fn write_read(buffer: Buffer<u16>, writes: &[u8], reads: NonZeroU16) -> Result<Self, (Error, Buffer<u16>)> {
        if writes.is_empty() {
            return Err((Error::LengthMismatch, buffer));
        }
        Self::build(buffer, writes, reads.get())
    }
    fn build(buffer: Buffer<u16>, writes: &[u8], reads: u16) -> Result<Self, (Error, Buffer<u16>)> {
        if writes.len().checked_add(usize::from(reads)) != Some(usize::from(buffer.range.count)) {
            return Err((Error::LengthMismatch, buffer));
        }
        // The Buffer owner is unused or returned only by an unsafe external-drain
        // handback. No active transfer can expose it through a safe method.
        let words =
            unsafe { core::slice::from_raw_parts_mut(buffer.pointer.as_ptr(), usize::from(buffer.range.count)) };
        encode(words, writes);
        Ok(Self { buffer, reads })
    }
    /// Recover an unused command allocation; transfers never expose Commands safely.
    pub fn into_buffer(self) -> Buffer<u16> {
        self.buffer
    }
}
fn encode(words: &mut [u16], writes: &[u8]) {
    let last = words.len() - 1;
    for (n, word) in words.iter_mut().enumerate() {
        *word = if n < writes.len() { u16::from(writes[n]) } else { 0x100 };
        if n == writes.len() && n != 0 {
            *word |= 0x400;
        }
        if n == last {
            *word |= 0x200;
        }
    }
}
/// Own an already configured, enabled idle I2C master, address and both routes.
pub struct I2cDma<'d, T: Instance> {
    token: T,
    pins: Pins<'d, T>,
    address: Address,
    burst: Burst,
}
/// Optional RX reservation. Pure writes require None; reads require exact byte count.
pub type Receive<'d> = Option<(Channel<'d>, Buffer<u8>)>;
impl<'d, T: Instance> I2cDma<'d, T> {
    /// Reserve the configured bus without MMIO or address changes.
    ///
    /// # Safety
    /// Exclusively own I2C, both requests, IRQ, reset/clock controls and routes.
    /// Drain prior CPU/ISR/DMA/control-core users. Keep source32MHz, idle master,
    /// target/address mode, restart and RX-full hold configured and enabled.
    /// Establish FIFO8, halfword TX/byte RX lanes, master0, TXprot1/RXprot2 and
    /// selected burst/threshold semantics on this hardware. In Four mode the
    /// RX watermark must be mutable while running and lowering to0 must safely
    /// permit a final short group. These are not proved by the disabled SDK DMA
    /// configuration or QEMU. Preserve pull-ups, peer and clocks through recovery.
    pub unsafe fn from_configured(token: T, pins: Pins<'d, T>, address: Address, burst: Burst) -> Self {
        Self { token, pins, address, burst }
    }
    /// Pair static disjoint allocations and unique channels without MMIO.
    /// All owners are returned on invalid length/overlap. The command buffer is
    /// the TX source even for a pure read; no stack-local read command is used.
    #[allow(clippy::type_complexity)]
    pub fn transfer(
        self,
        tx: Channel<'d>,
        commands: Commands,
        receive: Receive<'d>,
    ) -> Result<I2cTransfer<'d, T>, (Error, Self, Channel<'d>, Commands, Receive<'d>)> {
        let count = receive.as_ref().map_or(0, |(_, b)| b.range.count);
        let error = if count != commands.reads {
            Some(Error::LengthMismatch)
        } else if receive.as_ref().is_some_and(|(_, b)| b.range.overlaps(commands.buffer.range)) {
            Some(Error::Overlap)
        } else {
            None
        };
        if let Some(e) = error {
            return Err((e, self, tx, commands, receive));
        }
        let port = Port::of::<T>();
        let addr = regs!(port, r, { r.ic_data_write().as_ptr() as u32 });
        let engine = |id, rx, mem: Range| {
            let peripheral = Range { start: addr, end: addr, count: mem.count, width: mem.width };
            Engine {
                io: PacRegisters::new(id, Mode::I2c { port, receive: rx, burst: self.burst }),
                source: if rx { peripheral } else { mem },
                destination: if rx { mem } else { peripheral },
                state: State::Ready,
                stop_on_drop: false,
            }
        };
        let pair = Pair {
            tx: engine(tx.id, false, commands.buffer.range),
            rx: receive.as_ref().map(|(ch, buf)| engine(ch.id, true, buf.range)),
            io: PacI2c { port, address: self.address },
            state: State::Ready,
            requests_owned: false,
            rx_level: self.burst.threshold(),
        };
        Ok(I2cTransfer { pair, owner: Some(self), tx: Some(tx), commands: Some(commands), receive })
    }
    /// Return reservations after unused construction or independent external recovery.
    pub fn into_parts(self) -> (T, Pins<'d, T>) {
        (self.token, self.pins)
    }
}
/// One-shot command and optional receive transfer. All resources stay retained
/// on success, timeout, error, cancellation, Drop and forget.
///
/// ```compile_fail
/// use hisi_hal::{dma::i2c::I2cTransfer,peripherals::I2c0};
/// fn reclaim(t:I2cTransfer<'static,I2c0<'static>>) { t.release_after_external_quiescence(); }
/// ```
pub struct I2cTransfer<'d, T: Instance> {
    pair: Pair<PacRegisters, PacI2c>,
    owner: Option<I2cDma<'d, T>>,
    tx: Option<Channel<'d>>,
    commands: Option<Commands>,
    receive: Receive<'d>,
}
impl<'d, T: Instance> I2cTransfer<'d, T> {
    /// Guard idle channels and bus, isolate requests, arm RX then commands, open requests.
    pub fn start(&mut self) -> Result<(), Error> {
        self.pair.start()
    }
    /// One finite poll budget checks both DMA channels, I2C errors and final STOP.
    /// Four-beat RX tails lower the watermark once. Completion counts commands,
    /// not received bytes, and is not a physical drain/coherency certificate.
    pub fn wait(&mut self, polls: NonZeroU16, delay: &mut impl DelayNs) -> Result<Completion, Error> {
        self.pair.wait(polls, delay)
    }
    /// Close requests and halt both attempted channels once, retaining resources.
    /// Does not ABORT/disable the I2C controller, flush FIFO or stop its clocks.
    pub fn request_stop(&mut self) {
        self.pair.stop();
    }
    /// Return every owner after independently proven recovery; performs no MMIO.
    ///
    /// # Safety
    /// Prevent all old peripheral/CPU/ISR/DMA activity and prove all posted accesses
    /// completed. Recover bus/peer, FIFO, STOP/error history, configured address and
    /// routes; maintain allocation coherency. TC, ACTIVE, ENABLED, STOP or delays
    /// alone do not prove quiescence. I2cDma must again meet its handoff contract.
    #[allow(clippy::type_complexity)]
    pub unsafe fn release_after_external_quiescence(mut self) -> (I2cDma<'d, T>, Channel<'d>, Commands, Receive<'d>) {
        self.pair.requests_owned = false;
        self.pair.tx.stop_on_drop = false;
        if let Some(rx) = &mut self.pair.rx {
            rx.stop_on_drop = false;
        }
        compiler_fence(Ordering::Acquire);
        (self.owner.take().unwrap(), self.tx.take().unwrap(), self.commands.take().unwrap(), self.receive.take())
    }
}
#[derive(Clone, Copy)]
struct Snapshot {
    configured: bool,
    idle: bool,
    stop: bool,
    rx_count: u32,
    requests: u8,
    error: Option<Error>,
}
impl Snapshot {
    fn guard(self) -> Result<(), Error> {
        if !self.configured {
            return Err(Error::I2cConfiguration);
        }
        if let Some(e) = self.error {
            return Err(e);
        }
        if !self.idle || self.rx_count != 0 || self.requests != 0 {
            return Err(Error::I2cBusy);
        }
        Ok(())
    }
}
trait I2cRegisters {
    fn snapshot(&mut self) -> Snapshot;
    fn prepare(&mut self, rx_level: u32);
    fn requests(&mut self, tx: bool, rx: bool);
    fn rx_level(&mut self, level: u32);
    fn valid(&mut self, on: bool, rx: bool, level: u32) -> bool;
}
struct Pair<R: Registers, P: I2cRegisters> {
    tx: Engine<R>,
    rx: Option<Engine<R>>,
    io: P,
    state: State,
    requests_owned: bool,
    rx_level: u32,
}
impl<R: Registers, P: I2cRegisters> Pair<R, P> {
    fn start(&mut self) -> Result<(), Error> {
        if self.state != State::Ready {
            return Err(Error::NotReady);
        }
        self.state = State::Retained;
        let tx = self.tx.io.status();
        let rx = self.rx.as_mut().map(|e| e.io.status());
        if tx.enabled || tx.active || rx.is_some_and(|s| s.enabled || s.active) {
            return Err(Error::Occupied);
        }
        self.io.snapshot().guard()?;
        self.requests_owned = true;
        self.io.prepare(self.rx_level);
        let result = (|| {
            if !self.io.valid(false, self.rx.is_some(), self.rx_level) {
                return Err(Error::I2cConfiguration);
            }
            if let Some(rx) = &mut self.rx {
                rx.start()?;
            }
            self.tx.start()?;
            self.io.requests(true, self.rx.is_some());
            if !self.io.valid(true, self.rx.is_some(), self.rx_level) {
                return Err(Error::I2cConfiguration);
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
            self.io.requests(false, false);
            self.requests_owned = false;
        }
        self.tx.stop();
        if let Some(rx) = &mut self.rx {
            rx.stop();
        }
        self.state = State::Retained;
    }
    fn wait(&mut self, polls: NonZeroU16, delay: &mut impl DelayNs) -> Result<Completion, Error> {
        if self.state != State::Running {
            return Err(Error::NotReady);
        }
        let mut result = Err(Error::Timeout);
        for n in 0..polls.get() {
            let tx = self.tx.poll();
            let rx = self.rx.as_mut().map_or(Ok(Some(Completion { elements: 0 })), |e| e.poll());
            let error = match (tx, rx) {
                (Err(Error::Bus), _) | (_, Err(Error::Bus)) => Some(Error::Bus),
                (Err(e), _) | (_, Err(e)) => Some(e),
                _ => None,
            };
            if let Some(e) = error {
                result = Err(e);
                break;
            }
            let s = self.io.snapshot();
            if let Some(e) = s.error {
                result = Err(e);
                break;
            }
            if !s.configured || !self.io.valid(true, self.rx.is_some(), self.rx_level) {
                result = Err(Error::I2cConfiguration);
                break;
            }
            if let Some(rx) = &mut self.rx {
                let progress = rx.io.status();
                if progress.error {
                    result = Err(Error::Bus);
                    break;
                }
                let remaining = progress.remaining;
                if remaining > rx.source.count {
                    result = Err(Error::CompletionState);
                    break;
                }
                if self.rx_level != 0 && remaining > 0 && remaining < 4 {
                    self.io.rx_level(0);
                    self.rx_level = 0;
                    if !self.io.valid(true, true, 0) {
                        result = Err(Error::I2cConfiguration);
                        break;
                    }
                }
            }
            if let (Ok(Some(done)), Ok(Some(_))) = (tx, rx) {
                if s.rx_count != 0 {
                    result = Err(Error::I2cResidualReceive);
                    break;
                }
                if s.idle && s.stop {
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
impl<R: Registers, P: I2cRegisters> Drop for Pair<R, P> {
    fn drop(&mut self) {
        self.stop();
    }
}
struct PacI2c {
    port: Port,
    address: Address,
}
impl I2cRegisters for PacI2c {
    fn snapshot(&mut self) -> Snapshot {
        let clock = unsafe { &*pac::MCrg::PTR }.m_clken1().read();
        let clocked = match self.port {
            Port::Zero => clock.i2c0_enable().bit(),
            Port::One => clock.i2c1_enable().bit(),
        };
        let (address, ten) = self.address.parts();
        regs!(self.port, r, {
            let c = r.ic_con().read();
            let s = r.ic_status().read();
            let raw = r.ic_raw_intr_stat().read();
            let (hi, lo, min) = match c.speed().bits() {
                1 => (
                    r.ic_ss_scl_hcnt().read().ic_ss_scl_hcnt().bits(),
                    r.ic_ss_scl_lcnt().read().ic_ss_scl_lcnt().bits(),
                    12,
                ),
                2 => (
                    r.ic_fs_scl_hcnt().read().ic_fs_scl_hcnt().bits(),
                    r.ic_fs_scl_lcnt().read().ic_fs_scl_lcnt().bits(),
                    15,
                ),
                3 => (
                    r.ic_hs_scl_hcnt().read().ic_hs_scl_hcnt().bits(),
                    r.ic_hs_scl_lcnt().read().ic_hs_scl_lcnt().bits(),
                    21,
                ),
                _ => (0, 0, 1),
            };
            let dma = r.ic_dma_cr().read();
            Snapshot {
                configured: clocked
                    && r.ic_enable_status().read().ic_en().bit()
                    && r.ic_enable().read().bits() == 1
                    && c.bits() & !6 == (0x261 | u32::from(ten) << 4)
                    && hi >= 6
                    && lo >= min
                    && r.ic_tar().read().bits() == u32::from(address),
                idle: !s.activity().bit() && s.tfe().bit() && r.ic_txflr().read().txflr().bits() == 0,
                stop: raw.stop_det().bit(),
                rx_count: r.ic_rxflr().read().rxflr().bits(),
                requests: u8::from(dma.tdmae().bit()) | u8::from(dma.rdmae().bit()) << 1,
                error: if raw.bits() & 0x4b != 0 {
                    Some(Error::I2cHardware {
                        raw: raw.bits(),
                        source: r.ic_tx_abrt_source().read().bits(),
                        detail: r.ic_tx_abrt_slv_intx().read().bits(),
                    })
                } else {
                    None
                },
            }
        })
    }
    fn prepare(&mut self, rx_level: u32) {
        unsafe {
            interrupt::disable(self.port.irq());
        }
        self.requests(false, false);
        regs!(self.port, r, {
            unsafe {
                r.ic_intr_mask().write_with_zero(|w| w);
                r.ic_clr_int().write_with_zero(|w| w.clr_int().set_bit());
                r.ic_dma_tdlr().write_with_zero(|w| w.dmatdl().bits(4));
            }
        });
        self.rx_level(rx_level);
    }
    fn requests(&mut self, tx: bool, rx: bool) {
        regs!(self.port, r, {
            unsafe {
                r.ic_dma_cr().write_with_zero(|w| w.tdmae().bit(tx).rdmae().bit(rx));
            }
        });
    }
    fn rx_level(&mut self, level: u32) {
        regs!(self.port, r, {
            unsafe {
                r.ic_dma_rdlr().write_with_zero(|w| w.dmardl().bits(level));
            }
        });
    }
    fn valid(&mut self, on: bool, rx: bool, level: u32) -> bool {
        regs!(self.port, r, {
            let d = r.ic_dma_cr().read();
            d.tdmae().bit() == on
                && d.rdmae().bit() == (on && rx)
                && r.ic_dma_tdlr().read().dmatdl().bits() == 4
                && r.ic_dma_rdlr().read().dmardl().bits() == level
                && r.ic_intr_mask().read().bits() == 0
                && (on || !r.ic_raw_intr_stat().read().stop_det().bit())
        })
    }
}
#[cfg(all(test, not(target_arch = "riscv32")))]
#[path = "i2c_tests.rs"]
mod tests;
