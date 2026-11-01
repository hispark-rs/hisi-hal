//! Experimental one-shot ADC/MCU_DIAG capture with retained static memory.
//!
//! No safe completion, timeout, cancellation or Drop returns a borrowed buffer.
//! Even forgetting this handle cannot make its static memory available again.
//! After independent hardware drain proof, an unsafe handoff returns resources.
//! This is not yet physical ADC qualification or a cache-maintenance driver.
use super::{Calibrator, Channel, PacRegisters, Registers};
use crate::{
    peripherals::McuDiag,
    pinctrl::PadControl,
    soc::{pac, ws53::pins::Pad},
};
use core::{
    marker::PhantomData,
    num::NonZeroU16,
    ptr::NonNull,
    sync::atomic::{Ordering, compiler_fence},
};
use embedded_hal::delay::DelayNs;

fn pad_channel(pad: Pad) -> Option<Channel> {
    Some(match pad {
        Pad::Agpio1 => Channel::Channel0,
        Pad::Agpio3 => Channel::Channel1,
        Pad::Agpio4 => Channel::Channel2,
        Pad::Agpio6 => Channel::Channel3,
        Pad::Mgpio22 => Channel::Channel4,
        Pad::Mgpio8 => Channel::Channel5,
        Pad::Mgpio9 => Channel::Channel6,
        Pad::Agpio0 => Channel::Channel7,
        _ => return None,
    })
}
/// One exclusively owned analog route; physical aliases normalize through Pad.
pub struct Input<'d> {
    pad: PadControl<'d>,
    channel: Channel,
}
impl<'d> Input<'d> {
    /// Adopt an already configured analog pad, rejecting non-ADC pads untouched.
    ///
    /// # Safety
    /// Analog mux, input/pull/drive and supply must be correct for this physical
    /// ADC channel; no digital driver, peripheral or other core may use it.
    /// Reserve it until the converter and diagnostic writer have independently
    /// been proved quiescent. This does not write or validate electrical setup.
    pub unsafe fn from_configured(pad: PadControl<'d>) -> Result<Self, PadControl<'d>> {
        let Some(channel) = pad_channel(pad.pad()) else {
            return Err(pad);
        };
        Ok(Self { pad, channel })
    }
    /// Normalized ADC identity, not an SDK pin number.
    pub fn channel(&self) -> Channel {
        self.channel
    }
    /// Return an unused route, or one returned after external quiescence.
    pub fn into_pad(self) -> PadControl<'d> {
        self.pad
    }
}
/// A nonempty set of owned analog inputs. No raw channel-mask constructor.
pub struct Inputs<'d> {
    routes: [Option<Input<'d>>; 8],
    mask: u8,
}
impl<'d> Inputs<'d> {
    /// Start with one owned route.
    pub fn new(input: Input<'d>) -> Self {
        let index = input.channel as usize;
        let mut routes = [const { None }; 8];
        routes[index] = Some(input);
        Self { routes, mask: 1 << index }
    }
    /// Add another channel; a duplicate is returned without replacing its owner.
    pub fn add(&mut self, input: Input<'d>) -> Result<(), Input<'d>> {
        let index = input.channel as usize;
        if self.routes[index].is_some() {
            return Err(input);
        }
        self.routes[index] = Some(input);
        self.mask |= 1 << index;
        Ok(())
    }
    /// Test a typed channel identity.
    pub fn contains(&self, channel: Channel) -> bool {
        self.mask & (1 << channel as u8) != 0
    }
    /// Return unused routes, or routes recovered by external quiescence.
    pub fn into_inputs(self) -> [Option<Input<'d>>; 8] {
        self.routes
    }
}
/// A buffer does not fit the audited MCU_MODE_1 DTCM capture aperture.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BufferError {
    /// At least one32-bit sample is required.
    Empty,
    /// Address alignment, length, arithmetic or DTCM bounds are invalid.
    Range,
}
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Range {
    start: u32,
    end: u32,
    words: u32,
}
impl Range {
    fn new(start: usize, words: usize) -> Result<Self, BufferError> {
        if words == 0 {
            return Err(BufferError::Empty);
        }
        let limit = words.checked_mul(4).and_then(|bytes| start.checked_add(bytes)).ok_or(BufferError::Range)?;
        if start & 3 != 0 || start < 0x2001_0000 || limit > 0x2006_0000 {
            return Err(BufferError::Range);
        }
        Ok(Self { start: start as u32, end: (limit - 1) as u32, words: words as u32 })
    }
}
/// Exclusively owned static memory. No reference to it is kept while hardware
/// can write. Dropping this owner intentionally leaves the allocation reserved.
///
/// ```compile_fail
/// use hisi_hal::adc::capture::Buffer;
/// fn stack_buffer() { let mut words = [0u32; 2]; unsafe { Buffer::from_coherent_static(&mut words) }; }
/// ```
pub struct Buffer {
    ptr: NonNull<u32>,
    range: Range,
    _owned: PhantomData<&'static mut [u32]>,
}
impl Buffer {
    /// Transfer a static coherent allocation without accessing its contents.
    /// Returns invalid allocations untouched, before any device access.
    ///
    /// # Safety
    /// Every byte must be application-owned DTCM accessible to MCU_DIAG in the
    /// actual MCU memory mode. It must be noncacheable or hardware coherent for
    /// both CPU and diagnostic writer, with no dirty cache aliases or other CPU,
    /// trace, DMA or control-core users (including shared cache lines). Complete
    /// required CPU/bus synchronization before handing it over. A compiler fence
    /// alone is NOT cache maintenance. Keep this contract for its entire use;
    /// this constructor checks bounds, not memory attributes or cross-core IPC.
    pub unsafe fn from_coherent_static(words: &'static mut [u32]) -> Result<Self, (BufferError, &'static mut [u32])> {
        let ptr = words.as_mut_ptr();
        let range = match Range::new(ptr as usize, words.len()) {
            Ok(r) => r,
            Err(e) => return Err((e, words)),
        };
        Ok(Self { ptr: NonNull::new(ptr).unwrap(), range, _owned: PhantomData })
    }
    /// Return an unused allocation, or one recovered after external drain proof.
    /// No Capture method can safely return this owner while a writer may exist.
    pub fn into_words(self) -> &'static mut [u32] {
        compiler_fence(Ordering::Acquire);
        // Buffer is only available before capture or after its unsafe handback.
        unsafe { core::slice::from_raw_parts_mut(self.ptr.as_ptr(), self.range.words as usize) }
    }
}
/// Failure retains the buffer, ADC, diagnostic writer and routes in the handle.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error {
    /// Capture has already been attempted or stopped; external recovery needed.
    NotReady,
    /// A successful analog calibration report is required before arming.
    NotCalibrated,
    /// Inherited CPU trace or diagnostic enable contradicts the handoff.
    Occupied,
    /// Programmed configuration changed or did not read back correctly.
    Configuration,
    /// Old completion state did not clear before arming.
    StaleCompletion,
    /// Completion did not become visible within the finite polling budget.
    Timeout,
    /// DONE was visible but its address did not match the last requested word.
    CompletionAddress,
}
/// Observed status only, never a guarantee that RAM is coherent or drained.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Completion {
    last_address: u32,
}
impl Completion {
    /// Last-address register snapshot; not a safe pointer or memory-validity proof.
    pub fn last_address(self) -> u32 {
        self.last_address
    }
}
#[derive(Clone, Copy, PartialEq, Eq)]
enum State {
    Ready,
    Running,
    Retained,
}
trait CaptureRegisters {
    fn idle(&mut self) -> bool;
    fn configure(&mut self, range: Range, channels: u8);
    fn valid(&mut self, range: Range, channels: u8, enabled: bool) -> bool;
    fn status(&mut self) -> (bool, u32);
    fn start(&mut self);
    fn stop(&mut self);
}
struct Engine<R: CaptureRegisters> {
    io: R,
    range: Range,
    channels: u8,
    state: State,
    stop_on_drop: bool,
}
impl<R: CaptureRegisters> Engine<R> {
    fn start(&mut self, calibrated: bool) -> Result<(), Error> {
        if self.state != State::Ready {
            return Err(Error::NotReady);
        }
        if !calibrated {
            return Err(Error::NotCalibrated);
        }
        if !self.io.idle() {
            self.state = State::Retained;
            return Err(Error::Occupied);
        }
        // From the first register mutation onward retain all resources, even if
        // configuration readback fails or a callback panics before arming.
        self.state = State::Retained;
        self.stop_on_drop = true;
        self.io.configure(self.range, self.channels);
        let result = if !self.io.valid(self.range, self.channels, false) {
            Err(Error::Configuration)
        } else if self.io.status().0 {
            Err(Error::StaleCompletion)
        } else {
            Ok(())
        };
        if let Err(e) = result {
            self.stop();
            return Err(e);
        }
        compiler_fence(Ordering::Release);
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
    fn wait(&mut self, polls: NonZeroU16, delay: &mut impl DelayNs) -> Result<Completion, Error> {
        if self.state != State::Running {
            return Err(Error::NotReady);
        }
        let mut result = Err(Error::Timeout);
        for attempt in 0..polls.get() {
            if !self.io.valid(self.range, self.channels, true) {
                result = Err(Error::Configuration);
                break;
            }
            let (done, address) = self.io.status();
            if done {
                result = if address == self.range.end - 3 {
                    Ok(Completion { last_address: address })
                } else {
                    Err(Error::CompletionAddress)
                };
                break;
            }
            if attempt + 1 < polls.get() {
                delay.delay_us(1);
            }
        }
        self.stop();
        result
    }
}
impl<R: CaptureRegisters> Drop for Engine<R> {
    fn drop(&mut self) {
        self.stop();
    }
}
/// One-shot capture retaining all resources even after observed completion.
/// No buffers with a nonstatic borrow are accepted; no safe `read`/`release` API.
///
/// ```compile_fail
/// use hisi_hal::adc::capture::Capture;
/// fn no_safe_release(c: Capture<'_>) { c.release_after_external_quiescence(); }
/// ```
/// ```compile_fail
/// use hisi_hal::adc::capture::Capture;
/// fn no_read_during_dma(c: &Capture<'_>) { c.samples(); }
/// ```
pub struct Capture<'d> {
    engine: Engine<PacCapture>,
    calibration: Option<Calibrator<'d>>,
    diag: Option<McuDiag<'d>>,
    inputs: Option<Inputs<'d>>,
    buffer: Option<Buffer>,
}
impl<'d> Capture<'d> {
    /// Reserve the resources without MMIO; start explicitly after calibration.
    ///
    /// # Safety
    /// Independently verify that this silicon's source6 capture uses32-bit word
    /// LENGTH, exact inclusive END byte and the source automatic scan profile
    /// (average8/sample6/start26/discard32). These semantics are not yet qualified
    /// on WS53 hardware; the explicit QEMU profile is only a test environment.
    /// Stop and drain all old ADC/MCU_DIAG/trace writers; hold exclusive ownership
    /// of clocks, reset, analog power, physical inputs and diagnostic configuration
    /// through Drop or handback. Grant monitor/save selection1 and clock mask0x1ff.
    /// Do not change MCU memory mode or cache attributes during capture. Keep bus
    /// MMIO accessible for stop requests. No unsafe handoff is implied by DONE.
    pub unsafe fn from_calibrated(
        calibration: Calibrator<'d>,
        diag: McuDiag<'d>,
        inputs: Inputs<'d>,
        buffer: Buffer,
    ) -> Self {
        Self {
            engine: Engine {
                io: PacCapture,
                range: buffer.range,
                channels: inputs.mask,
                state: State::Ready,
                stop_on_drop: false,
            },
            calibration: Some(calibration),
            diag: Some(diag),
            inputs: Some(inputs),
            buffer: Some(buffer),
        }
    }
    /// Program one-shot capture and start scanning. Any failure retains resources.
    pub fn start(&mut self) -> Result<(), Error> {
        self.engine.start(self.calibration.as_ref().unwrap().result().is_some())
    }
    /// Bound status polling, then request producer stop and diagnostic disable.
    /// Success is only a register observation, never permission to access memory.
    /// Timeout and every error leave the allocation retained; no automatic retry.
    pub fn wait(&mut self, polls: NonZeroU16, delay: &mut impl DelayNs) -> Result<Completion, Error> {
        self.engine.wait(polls, delay)
    }
    /// Request stop once and retain memory. Does not wait or claim bus quiescence.
    pub fn request_stop(&mut self) {
        self.engine.stop();
    }
    /// Recover resources after independently proving writer shutdown and drain.
    ///
    /// # Safety
    /// Prove no ADC/diagnostic/trace/other-core write can occur now or later until
    /// explicit rearming, including posted bus transactions. Synchronize CPU and
    /// bus accesses and preserve Buffer's coherent/no-cache-alias contract.
    /// STOP/DONE, address stability, CPU halt, elapsed delay or a compiler fence
    /// alone are insufficient. This function does not perform MMIO or cache work.
    /// Calibration is invalidated because external recovery may have reset analog
    /// state. Even if start failed, the caller must establish this same contract.
    pub unsafe fn release_after_external_quiescence(mut self) -> (Calibrator<'d>, McuDiag<'d>, Inputs<'d>, Buffer) {
        self.engine.stop_on_drop = false;
        self.engine.state = State::Retained;
        compiler_fence(Ordering::Acquire);
        let mut calibration = self.calibration.take().unwrap();
        calibration.engine.result = None;
        (calibration, self.diag.take().unwrap(), self.inputs.take().unwrap(), self.buffer.take().unwrap())
    }
}
struct PacCapture;
impl PacCapture {
    fn diag(&self) -> &'static pac::mcu_diag::RegisterBlock {
        unsafe { &*pac::McuDiag::PTR }
    }
}
impl CaptureRegisters for PacCapture {
    fn idle(&mut self) -> bool {
        let r = self.diag();
        let trace = r.cpu_trace().read();
        !trace.trace_enable().bit() && !trace.mcpu_enable().bit() && !r.sample_mode().read().enable().bit()
    }
    fn configure(&mut self, range: Range, channels: u8) {
        let r = self.diag();
        let a = unsafe { &*pac::Adc::PTR };
        // ADC calibration already enabled its gates. The diagnostic bank is
        // shared; ownership handoff grants the complete source clock mask.
        unsafe {
            r.monitor_select().write_with_zero(|w| w.select().set_bit());
            r.trace_save_select().write_with_zero(|w| w.select().set_bit());
            r.monitor_clock().write_with_zero(|w| w.mask().bits(0x1ff));
            r.sample_select().write_with_zero(|w| w.value().bits(6));
            r.length_low().write_with_zero(|w| w.value().bits(range.words as u16));
            r.length_high().write_with_zero(|w| w.value().bits((range.words >> 16) as u16));
            r.start_low().write_with_zero(|w| w.value().bits(range.start as u16));
            r.start_high().write_with_zero(|w| w.value().bits((range.start >> 16) as u16));
            r.end_low().write_with_zero(|w| w.value().bits(range.end as u16));
            r.end_high().write_with_zero(|w| w.value().bits((range.end >> 16) as u16));
            r.sample_mode().write_with_zero(|w| w.mode().once().enable().clear_bit().sync().clear_bit());
            r.sample_mode().write_with_zero(|w| w.mode().once().enable().clear_bit().sync().set_bit());
            a.manual_control().write_with_zero(|w| w.channel().bits(0).manual_select().clear_bit());
            a.scan_control().write_with_zero(|w| {
                w.channels()
                    .bits(channels)
                    .average()
                    .eight()
                    .sample_count()
                    .bits(6)
                    .start_count()
                    .bits(26)
                    .discard_count()
                    .bits(32)
            });
        }
    }
    fn valid(&mut self, range: Range, channels: u8, enabled: bool) -> bool {
        let r = self.diag();
        let a = unsafe { &*pac::Adc::PTR };
        let mode = r.sample_mode().read();
        let scan = a.scan_control().read();
        let manual = a.manual_control().read();
        let trace = r.cpu_trace().read();
        let length =
            u32::from(r.length_low().read().value().bits()) | u32::from(r.length_high().read().value().bits()) << 16;
        let start =
            u32::from(r.start_low().read().value().bits()) | u32::from(r.start_high().read().value().bits()) << 16;
        let end = u32::from(r.end_low().read().value().bits()) | u32::from(r.end_high().read().value().bits()) << 16;
        PacRegisters.configuration_valid()
            && !trace.trace_enable().bit()
            && !trace.mcpu_enable().bit()
            && r.monitor_select().read().select().bit()
            && r.trace_save_select().read().select().bit()
            && r.monitor_clock().read().mask().bits() == 0x1ff
            && r.sample_select().read().value().bits() == 6
            && mode.mode().is_once()
            && mode.enable().bit() == enabled
            && mode.sync().bit()
            && length == range.words
            && start == range.start
            && end == range.end
            && scan.channels().bits() == channels
            && scan.average().is_eight()
            && scan.sample_count().bits() == 6
            && scan.start_count().bits() == 26
            && scan.discard_count().bits() == 32
            && !manual.manual_select().bit()
            && manual.channel().bits() == 0
    }
    fn status(&mut self) -> (bool, u32) {
        let r = self.diag();
        let done = r.sample_done().read().done().bit();
        let address = u32::from(r.done_address_low().read().value().bits())
            | u32::from(r.done_address_high().read().value().bits()) << 16;
        (done, address)
    }
    fn start(&mut self) {
        unsafe {
            self.diag().sample_mode().write_with_zero(|w| w.mode().once().enable().set_bit().sync().set_bit());
            (&*pac::Adc::PTR).scan_command().write_with_zero(|w| w.start().set_bit().stop().clear_bit());
        }
    }
    fn stop(&mut self) {
        unsafe {
            (&*pac::Adc::PTR).scan_command().write_with_zero(|w| w.start().clear_bit().stop().set_bit());
            self.diag().sample_mode().write_with_zero(|w| w.mode().once().enable().clear_bit().sync().clear_bit());
        }
    }
}
#[cfg(all(test, not(target_arch = "riscv32")))]
#[path = "capture_tests.rs"]
mod tests;
