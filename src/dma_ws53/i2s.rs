//! Experimental I2S merge DMA. Completion counts samples, never wire frames.
//! Cancellation only halts DMA; the peripheral remains reserved and clocked.
use super::{Buffer, Channel, Completion, Engine, Error, Mode, PacRegisters, Range, Registers, State};
use crate::{
    i2s::{ClockEdge, ClockMode, DataWidth, Pins},
    interrupt,
    peripherals::I2s,
    soc::pac,
};
use core::{
    num::NonZeroU16,
    sync::atomic::{Ordering, compiler_fence},
};
use embedded_hal::delay::DelayNs;

/// Equal source/destination burst choice; physical handshakes require qualification.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Burst {
    /// One sample per request.
    One,
    /// Four sample accesses per accepted request.
    Four,
    /// Eight sample accesses per accepted request.
    Eight,
    /// Sixteen sample accesses per accepted request.
    Sixteen,
}
impl Burst {
    pub(super) fn encoding(self) -> u8 {
        match self {
            Self::One => 0,
            Self::Four => 1,
            Self::Eight => 2,
            Self::Sixteen => 3,
        }
    }
    fn samples(self) -> u16 {
        match self {
            Self::One => 1,
            Self::Four => 4,
            Self::Eight => 8,
            Self::Sixteen => 16,
        }
    }
    fn level(self) -> u8 {
        match self {
            Self::One => 0,
            Self::Four => 1,
            Self::Eight => 3,
            Self::Sixteen => 7,
        }
    }
}
/// Owned left/right interleaved sample words, validated before any peripheral access.
pub struct Samples {
    buffer: Buffer<u32>,
    width: DataWidth,
}
impl Samples {
    /// Require complete stereo pairs with no high bits outside the sample width.
    /// Failure returns the unchanged allocation. This does not certify packing on silicon.
    pub fn new(buffer: Buffer<u32>, width: DataWidth) -> Result<Self, (Error, Buffer<u32>)> {
        let words = unsafe { core::slice::from_raw_parts(buffer.pointer.as_ptr(), buffer.range.count as usize) };
        if !valid_samples(words, width) {
            return Err((Error::I2sFraming, buffer));
        }
        Ok(Self { buffer, width })
    }
    /// Return unused or externally recovered memory.
    pub fn into_buffer(self) -> Buffer<u32> {
        self.buffer
    }
}
fn valid_samples(words: &[u32], width: DataWidth) -> bool {
    let mask = u32::MAX >> (32 - width.bits());
    !words.is_empty() && words.len().is_multiple_of(2) && words.iter().all(|v| v & !mask == 0)
}
/// Optional transmit channel and static stereo sample source.
pub type Transmit<'d> = Option<(Channel<'d>, Samples)>;
/// Optional receive channel and static sample-word destination.
pub type Receive<'d> = Option<(Channel<'d>, Buffer<u32>)>;
/// Reserve a configured, stopped stereo peripheral. No MMIO during construction.
pub struct I2sDma<'d> {
    token: I2s<'d>,
    pins: Pins<'d>,
    width: DataWidth,
    clock: ClockMode,
    edge: ClockEdge,
    burst: Burst,
}
impl<'d> I2sDma<'d> {
    /// Accept an exclusive configured peripheral with clocks available but serializer stopped.
    ///
    /// # Safety
    /// Own I2S, IRQ, both requests, clock controls and all routes. Drain previous
    /// CPU/ISR/DMA/control-core users; establish empty FIFOs, left-first merge
    /// phase and reset release. Both TX/RX must be disabled and I2S clock_enable
    /// clear. Keep the upstream source, format, divider, external slave clocks,
    /// peer and DMA coherent memory stable through external recovery. Qualify
    /// alternating 32-bit sample accesses, FIFO8, protection1 and selected burst/
    /// watermark behavior on the actual board. The reference product disables
    /// DMA; QEMU's merge semantics are assumptions. A late polling response may
    /// cause underrun/overflow: the peer must tolerate this until independently
    /// stopped. No automatic wire-drain or safe peripheral-release is provided.
    pub unsafe fn from_configured(
        token: I2s<'d>,
        pins: Pins<'d>,
        width: DataWidth,
        clock: ClockMode,
        edge: ClockEdge,
        burst: Burst,
    ) -> Self {
        Self { token, pins, width, clock, edge, burst }
    }
    /// Prepare transmit-only, receive-only or duplex DMA without MMIO.
    /// Counts must be even and allocations disjoint. Directions have independent
    /// sample counts: a finite RX window may use a longer, caller-provided TX
    /// stream to avoid underrun at its end. No hidden padding is transmitted.
    #[allow(clippy::type_complexity)]
    pub fn transfer(
        self,
        tx: Transmit<'d>,
        rx: Receive<'d>,
    ) -> Result<I2sTransfer<'d>, (Error, Self, Transmit<'d>, Receive<'d>)> {
        let source = tx.as_ref().map(|(_, s)| s.buffer.range);
        let dest = rx.as_ref().map(|(_, b)| b.range);
        let error = if tx.as_ref().is_some_and(|(_, s)| s.width != self.width) {
            Some(Error::I2sFraming)
        } else {
            validate_ranges(source, dest).err()
        };
        if let Some(e) = error {
            return Err((e, self, tx, rx));
        }
        let r = unsafe { &*I2s::ptr() };
        let engine = |ch: &Channel<'d>, receive: bool, m: Range| {
            let addr = if receive { r.i2s_dual_rx_chn().as_ptr() as u32 } else { r.i2s_dual_tx_chn().as_ptr() as u32 };
            let p = Range { start: addr, end: addr, count: m.count, width: 4 };
            Engine {
                io: PacRegisters::new(ch.id, Mode::I2s { receive, burst: self.burst }),
                source: if receive { p } else { m },
                destination: if receive { m } else { p },
                state: State::Ready,
                stop_on_drop: false,
            }
        };
        let pair = Pair {
            tx: tx.as_ref().map(|(c, s)| engine(c, false, s.buffer.range)),
            rx: rx.as_ref().map(|(c, b)| engine(c, true, b.range)),
            io: PacI2s { width: self.width, clock: self.clock, edge: self.edge },
            state: State::Ready,
            serial_started: false,
            level: self.burst.level(),
            burst: self.burst,
        };
        Ok(I2sTransfer { pair, owner: Some(self), tx, rx })
    }
    /// Return unused or externally recovered peripheral and routes.
    pub fn into_parts(self) -> (I2s<'d>, Pins<'d>) {
        (self.token, self.pins)
    }
}
fn validate_ranges(tx: Option<Range>, rx: Option<Range>) -> Result<(), Error> {
    if tx.is_none() && rx.is_none() || tx.into_iter().chain(rx).any(|r| !r.count.is_multiple_of(2)) {
        return Err(Error::I2sFraming);
    }
    if let (Some(t), Some(r)) = (tx, rx)
        && t.overlaps(r)
    {
        return Err(Error::Overlap);
    }
    Ok(())
}
/// DMA sample counts by direction, not wire completion or a drain certificate.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Progress {
    tx_samples: u16,
    rx_samples: u16,
}
impl Progress {
    /// Samples moved to the transmit FIFO; not necessarily serialized yet.
    pub fn tx_samples(self) -> u16 {
        self.tx_samples
    }
    /// Samples moved from the receive FIFO; not a physical integrity certificate.
    pub fn rx_samples(self) -> u16 {
        self.rx_samples
    }
}
/// Own the peripheral, optional DMA channels and static sample allocations.
/// All outcomes retain resources until unsafe external recovery.
///
/// ```compile_fail
/// use hisi_hal::dma::i2s::I2sTransfer;
/// fn reclaim(t:I2sTransfer<'static>) { t.release_after_external_quiescence(); }
/// ```
pub struct I2sTransfer<'d> {
    pair: Pair<PacRegisters, PacI2s>,
    owner: Option<I2sDma<'d>>,
    tx: Transmit<'d>,
    rx: Receive<'d>,
}
impl<'d> I2sTransfer<'d> {
    /// Guard inherited state, prepare merge and arm receive before transmit.
    /// Serializer start is deferred to wait_dma until a TX pair is prefetched.
    pub fn start(&mut self) -> Result<(), Error> {
        self.pair.start()
    }
    /// Finite progress budget with 1us delay. Start the serializer when ready,
    /// lower a short RX tail threshold, and check both DMA/error paths.
    /// Successful direction counts mean samples were moved, not played/drained.
    /// I2S remains running; independently stop/recover it before releasing owners.
    pub fn wait_dma(&mut self, polls: NonZeroU16, delay: &mut impl DelayNs) -> Result<Progress, Error> {
        self.pair.wait(polls, delay)
    }
    /// Halt attempted DMA channels once. Does not reset/discard I2S or gate its
    /// clock, and does not revoke requests or already-read transactions.
    pub fn request_stop(&mut self) {
        self.pair.stop();
    }
    /// Return reservations only after independent whole-stream recovery.
    ///
    /// # Safety
    /// Stop peer, serializer, requests and every competing user, drain all posted
    /// DMA writes, and reconcile FIFOs/phase/errors/cache. A DMA TC or FIFO-empty
    /// read is insufficient. Restore the constructor's stopped/empty/left-first
    /// configuration before reusing I2sDma. Forgotten owners must not be recreated.
    pub unsafe fn release_after_external_quiescence(mut self) -> (I2sDma<'d>, Transmit<'d>, Receive<'d>) {
        for e in self.pair.tx.iter_mut().chain(self.pair.rx.iter_mut()) {
            e.stop_on_drop = false;
        }
        compiler_fence(Ordering::Acquire);
        (self.owner.take().unwrap(), self.tx.take(), self.rx.take())
    }
}
#[derive(Clone, Copy)]
struct Snapshot {
    configured: bool,
    clock: bool,
    merge: bool,
    tx: [u8; 2],
    rx: [u8; 2],
    error: Option<Error>,
}
impl Snapshot {
    fn guard(self) -> Result<(), Error> {
        if !self.configured || self.tx.into_iter().chain(self.rx).any(|n| n > 8) {
            return Err(Error::I2sConfiguration);
        }
        if let Some(e) = self.error {
            return Err(e);
        }
        if self.clock || self.tx != [0, 0] || self.rx != [0, 0] {
            return Err(Error::I2sBusy);
        }
        Ok(())
    }
}
trait I2sRegisters {
    fn snapshot(&mut self) -> Snapshot;
    fn prepare(&mut self, level: u8);
    fn valid(&mut self, level: u8) -> bool;
    fn start(&mut self, tx: bool, rx: bool);
    fn rx_level(&mut self, level: u8);
}
struct Pair<R: Registers, P: I2sRegisters> {
    tx: Option<Engine<R>>,
    rx: Option<Engine<R>>,
    io: P,
    state: State,
    serial_started: bool,
    level: u8,
    burst: Burst,
}
impl<R: Registers, P: I2sRegisters> Pair<R, P> {
    fn start(&mut self) -> Result<(), Error> {
        if self.state != State::Ready {
            return Err(Error::NotReady);
        }
        self.state = State::Retained;
        for e in self.tx.iter_mut().chain(self.rx.iter_mut()) {
            let s = e.io.status();
            if s.enabled || s.active {
                return Err(Error::Occupied);
            }
        }
        self.io.snapshot().guard()?;
        self.io.prepare(self.level);
        let result = (|| {
            if !self.io.valid(self.level) {
                return Err(Error::I2sConfiguration);
            }
            if let Some(e) = &mut self.rx {
                e.start()?;
            }
            if let Some(e) = &mut self.tx {
                e.start()?;
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
        for e in self.tx.iter_mut().chain(self.rx.iter_mut()) {
            e.stop();
        }
        self.state = State::Retained;
    }
    fn wait(&mut self, polls: NonZeroU16, delay: &mut impl DelayNs) -> Result<Progress, Error> {
        if self.state != State::Running {
            return Err(Error::NotReady);
        }
        let mut result = Err(Error::Timeout);
        for n in 0..polls.get() {
            let tx = self.tx.as_mut().map_or(Ok(Some(Completion { elements: 0 })), |e| e.poll());
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
            if !s.configured
                || !s.clock
                || !s.merge
                || s.tx.into_iter().chain(s.rx).any(|v| v > 8)
                || !self.io.valid(self.level)
            {
                result = Err(Error::I2sConfiguration);
                break;
            }
            if !self.serial_started && (self.tx.is_none() || s.tx.iter().all(|&v| v > 0)) {
                self.io.start(self.tx.is_some(), self.rx.is_some());
                self.serial_started = true;
            }
            if let Some(e) = &mut self.rx {
                let p = e.io.status();
                if p.error {
                    result = Err(Error::Bus);
                    break;
                }
                if p.remaining > e.source.count {
                    result = Err(Error::CompletionState);
                    break;
                }
                if self.level != 0 && p.remaining > 0 && p.remaining < self.burst.samples() {
                    self.io.rx_level(0);
                    self.level = 0;
                    if !self.io.valid(0) {
                        result = Err(Error::I2sConfiguration);
                        break;
                    }
                }
            }
            if self.serial_started && matches!(tx, Ok(Some(_))) && matches!(rx, Ok(Some(_))) {
                result = Ok(Progress {
                    tx_samples: self.tx.as_ref().map_or(0, |e| e.source.count),
                    rx_samples: self.rx.as_ref().map_or(0, |e| e.source.count),
                });
                break;
            }
            if n + 1 < polls.get() {
                delay.delay_us(1);
            }
        }
        self.stop();
        result
    }
}
impl<R: Registers, P: I2sRegisters> Drop for Pair<R, P> {
    fn drop(&mut self) {
        self.stop();
    }
}
struct PacI2s {
    width: DataWidth,
    clock: ClockMode,
    edge: ClockEdge,
}
impl PacI2s {
    fn regs(&self) -> &'static pac::i2s::RegisterBlock {
        unsafe { &*I2s::ptr() }
    }
}
impl I2sRegisters for PacI2s {
    fn snapshot(&mut self) -> Snapshot {
        let r = self.regs();
        let mode = r.mode().read();
        let width = r.data_width_set().read();
        let c = r.i2s_crg().read();
        let tx = r.tx_sta().read();
        let rx = r.rx_sta().read();
        let flags = r.intstatus().read();
        let divider = match self.clock {
            ClockMode::Slave => !c.bclk_div_enable().bit(),
            ClockMode::Master(div) => {
                c.bclk_div_enable().bit()
                    && r.i2s_bclk_div_num().read().divider().bits() == div.get()
                    && r.i2s_fs_div_num().read().divider().bits() == u16::from(self.width.bits()) * 2
                    && r.i2s_fs_div_ratio_num().read().ratio().bits() == u16::from(self.width.bits())
            }
        };
        let configured = unsafe { &*pac::MCrg::PTR }.m_clken0_word().read().i2s_enable().bit()
            && unsafe { &*pac::AudioCmu::PTR }.new_cfg0().read().audio_divider_reset_n().bit()
            && !mode.mode().bit()
            && !mode.pcm_mode().bit()
            && !mode.rxws_select().bit()
            && !mode.extended_receive().bit()
            && mode.channels().bits() == 0
            && mode.master().bit() == matches!(self.clock, ClockMode::Master(_))
            && mode.clk_edge().bit() == (self.edge == ClockEdge::Rising)
            && width.tx_mode().bits() == self.width as u8
            && width.rx_mode().bits() == self.width as u8
            && divider
            && !c.bclk_select().bit()
            && !c.fs_select().bit()
            && !r.signed_ext().read().enable().bit()
            && !r.version().read().loopback().bit();
        Snapshot {
            configured,
            clock: c.clock_enable().bit(),
            merge: r.i2s_pos_merge_en().read().enable().bit(),
            tx: [tx.left().bits(), tx.right().bits()],
            rx: [rx.left().bits(), rx.right().bits()],
            error: if flags.bits() & 60 != 0 { Some(Error::I2sHardware { raw: flags.bits() }) } else { None },
        }
    }
    fn prepare(&mut self, level: u8) {
        unsafe {
            interrupt::disable(interrupt::Interrupt::I2S);
        }
        let r = self.regs();
        // Mask CPU threshold events but keep error status observable.
        unsafe {
            r.intmask().write_with_zero(|w| w.rx().set_bit().tx().set_bit());
        }
        r.fifo_threshold().modify(|_, w| unsafe { w.tx().bits(4).rx().bits(level) });
        r.i2s_pos_merge_en().modify(|_, w| w.enable().set_bit());
        r.i2s_crg().modify(|_, w| w.clock_enable().set_bit());
    }
    fn valid(&mut self, level: u8) -> bool {
        let s = self.snapshot();
        let r = self.regs();
        let t = r.fifo_threshold().read();
        s.configured
            && s.clock
            && s.merge
            && r.intmask().read().bits() == 3
            && t.tx().bits() == 4
            && t.rx().bits() == level
    }
    fn start(&mut self, tx: bool, rx: bool) {
        unsafe {
            self.regs().ct_set().write_with_zero(|w| w.tx_enable().bit(tx).rx_enable().bit(rx));
        }
    }
    fn rx_level(&mut self, level: u8) {
        self.regs().fifo_threshold().modify(|_, w| unsafe { w.rx().bits(level) });
    }
}
#[cfg(test)]
#[path = "i2s_tests.rs"]
mod tests;
