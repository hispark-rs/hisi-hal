//! One-command retained ADMA ownership. Source: f39e085 ROM transfer audit.
//! Length-delimited descriptor termination is an explicit caller precondition,
//! not something the SDK's misleading complete_send name proves on hardware.
use super::{
    AdmaLength, AdmaTransferImage, BlockSize, Cmd53Argument, Direction, EventSnapshot, Function1Events,
    Function1Request, RequestError,
};
use crate::{peripherals::Sdio, soc::pac};
use core::{
    marker::PhantomData,
    num::NonZeroU16,
    ptr::NonNull,
    sync::atomic::{Ordering, compiler_fence},
};
use embedded_hal::delay::DelayNs;

/// Invalid static allocation. No DMA request was submitted.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MemoryError {
    /// Empty or unrepresentable command-sized buffer.
    Length,
    /// Not word-aligned, overflowing, or outside the MCU_MODE_1 DTCM envelope.
    Range,
    /// Data and descriptor/trailer storage overlap.
    Overlap,
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct Range {
    start: u32,
    length: u32,
}
impl Range {
    fn new(start: usize, length: usize) -> Result<Self, MemoryError> {
        if length == 0 || length > 0x1fffff {
            return Err(MemoryError::Length);
        }
        let end = start.checked_add(length).ok_or(MemoryError::Range)?;
        if start & 3 != 0 || start < 0x20010000 || end > 0x20060000 {
            return Err(MemoryError::Range);
        }
        Ok(Self { start: start as u32, length: length as u32 })
    }
    fn overlaps(self, other: Self) -> bool {
        self.start < other.start + other.length && other.start < self.start + self.length
    }
}

/// Static, exclusively reserved data. Drop/forget retains the allocation.
/// No CPU slice is available while the buffer belongs to a transfer.
///
/// ```compile_fail
/// use hisi_hal::sdio::Buffer;
/// fn local() { let mut data=[0u8;4]; unsafe { Buffer::from_coherent_static(&mut data) }; }
/// ```
pub struct Buffer {
    pointer: NonNull<u8>,
    range: Range,
    _owned: PhantomData<&'static mut [u8]>,
}
impl Buffer {
    /// Reserve existing application-owned data without accessing its contents.
    /// The MCU envelope check does not establish actual RAM/core ownership.
    ///
    /// # Safety
    /// Entire allocation must be accessible to the SDIO bus master, exclusively
    /// application-owned and hardware coherent/noncacheable in the active mode.
    /// No CPU, control-core, DMA, trace or dirty cache alias may access it until
    /// handback. Own complete cache lines and perform required synchronization.
    /// Maintain these attributes after Drop/forget and during external recovery.
    pub unsafe fn from_coherent_static(bytes: &'static mut [u8]) -> Result<Self, (MemoryError, &'static mut [u8])> {
        let range = match Range::new(bytes.as_mut_ptr() as usize, bytes.len()) {
            Ok(range) => range,
            Err(error) => return Err((error, bytes)),
        };
        Ok(Self { pointer: NonNull::new(bytes.as_mut_ptr()).unwrap(), range, _owned: PhantomData })
    }
    /// Return unused data, or data handed back after independent external drain.
    pub fn into_bytes(self) -> &'static mut [u8] {
        compiler_fence(Ordering::Acquire);
        unsafe { core::slice::from_raw_parts_mut(self.pointer.as_ptr(), self.range.length as usize) }
    }
}

/// Statically allocated descriptor capacity plus a separately retained dummy
/// word matching the ROM's ordinary four-byte tail. No END bit is invented.
#[repr(C, align(4))]
pub struct AdmaStorage {
    entries: [[u8; 8]; 129],
    trailer: [u8; 4],
}
impl AdmaStorage {
    /// Empty owned storage; constructing it does not touch the controller.
    pub const fn new() -> Self {
        Self { entries: [[0; 8]; 129], trailer: [0; 4] }
    }
}
impl Default for AdmaStorage {
    fn default() -> Self {
        Self::new()
    }
}

/// Exclusive static descriptor/trailer reservation. Drop retains memory.
pub struct Table {
    pointer: NonNull<AdmaStorage>,
    range: Range,
    _owned: PhantomData<&'static mut AdmaStorage>,
}
impl Table {
    /// Reserve descriptor memory without publishing its address.
    ///
    /// # Safety
    /// The complete storage, including trailer and cache lines, must satisfy
    /// `Buffer::from_coherent_static`'s SDIO accessibility/coherence/exclusivity
    /// contract. No previous descriptor fetch or posted write may target it.
    pub unsafe fn from_coherent_static(
        storage: &'static mut AdmaStorage,
    ) -> Result<Self, (MemoryError, &'static mut AdmaStorage)> {
        let range = match Range::new(storage as *mut _ as usize, core::mem::size_of::<AdmaStorage>()) {
            Ok(range) => range,
            Err(error) => return Err((error, storage)),
        };
        Ok(Self { pointer: NonNull::from(storage), range, _owned: PhantomData })
    }
    /// Return unused storage or externally quiesced descriptor/trailer memory.
    pub fn into_storage(self) -> &'static mut AdmaStorage {
        compiler_fence(Ordering::Acquire);
        unsafe { &mut *self.pointer.as_ptr() }
    }
}

fn fill_storage(storage: &mut AdmaStorage, data: Range, table_address: u32) {
    *storage = AdmaStorage::new();
    let mut offset = 0;
    let mut index = 0;
    while offset < data.length {
        // Keep subsequent descriptor addresses word-aligned. No u16 truncation.
        let count = (data.length - offset).min(65532);
        let length = AdmaLength::try_new(count).unwrap();
        storage.entries[index] = AdmaTransferImage::try_new(data.start + offset, length).unwrap().to_le_bytes();
        offset += count;
        index += 1;
    }
    let trailer = table_address + core::mem::offset_of!(AdmaStorage, trailer) as u32;
    storage.entries[index] =
        AdmaTransferImage::try_new(trailer, AdmaLength::try_new(4).unwrap()).unwrap().to_le_bytes();
}

/// Progress reports observations only, never an SDIO bus-drain guarantee.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TransferProgress {
    /// Armed locally; no descriptor address has been published yet.
    WaitingForRequest,
    /// A matching host request was accepted and the table address published.
    Running,
    /// Expected host-completion status was observed; all memory remains owned.
    CompletionObserved,
}
/// Captured events and one finite transfer step.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct TransferPoll {
    /// Latest software progress.
    pub progress: TransferProgress,
    /// Full snapshot, including any message observed before acknowledgement.
    pub events: EventSnapshot,
}
/// Failure retains controller, descriptors, trailer and data without a restart.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TransferError {
    /// `start` has not yet been called; this error performs no MMIO.
    NotStarted,
    /// Already started; no further start attempt is admitted.
    AlreadyStarted,
    /// Terminal state; no further MMIO is performed.
    Retained,
    /// Clock readback failed; the SDIO window is not accessed.
    ClockGate,
    /// Host clock or READY disappeared; external recovery is required.
    Configuration,
    /// Pending events existed at arm time; retained without acknowledgement.
    DirtyStart(EventSnapshot),
    /// Request identity/size is unsupported; no table address was published.
    Request {
        /// Typed admission failure.
        error: RequestError,
        /// Events retained without acknowledgement.
        events: EventSnapshot,
    },
    /// Host requested a different length from the exclusively reserved buffer.
    LengthMismatch {
        /// Captured complete command byte count.
        requested: u32,
        /// Retained data allocation byte count.
        reserved: u32,
        /// Original events retained without acknowledgement.
        events: EventSnapshot,
    },
    /// Reset, unexpected direction, unknown or otherwise unhandled events.
    Unsupported(EventSnapshot),
    /// ADMA/read error status. It does not prove the bus has stopped.
    DeviceError(EventSnapshot),
    /// One total attempt budget expired across request and completion waiting.
    Timeout,
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum State {
    New,
    Waiting,
    Running,
    Retained,
}
trait Registers {
    fn clocks(&mut self) -> bool;
    fn configured(&mut self) -> bool;
    fn mask(&mut self);
    fn snapshot(&mut self) -> EventSnapshot;
    fn request(&mut self) -> (Cmd53Argument, Option<BlockSize>);
    fn prepare(&mut self, data: Range, table: Range);
    fn acknowledge(&mut self, events: EventSnapshot);
    fn send_length(&mut self, bytes: u32);
    fn announce_read(&mut self);
    #[cfg(feature = "async")]
    fn listen(&mut self);
    fn publish(&mut self, address: u32);
}
struct Engine<R> {
    io: R,
    state: State,
    direction: Direction,
    data: Range,
    table: Range,
}
fn pending(e: EventSnapshot) -> bool {
    e.global != 0 || e.function0 != 0 || e.function1.bits() != 0
}
fn unsupported(e: EventSnapshot, allowed: u32) -> bool {
    e.global & !5 != 0 || e.function0 & !0xffc != 0 || e.function1.bits() & !allowed != 0
}
impl<R: Registers> Engine<R> {
    fn fail(&mut self, error: TransferError) -> TransferError {
        self.io.mask();
        self.state = State::Retained;
        error
    }
    fn available(&mut self) -> Result<(), TransferError> {
        if !self.io.clocks() {
            self.state = State::Retained;
            return Err(TransferError::ClockGate);
        }
        if !self.io.configured() {
            return Err(self.fail(TransferError::Configuration));
        }
        Ok(())
    }
    fn start(&mut self) -> Result<(), TransferError> {
        if self.state != State::New {
            return Err(TransferError::AlreadyStarted);
        }
        self.available()?;
        self.io.mask();
        let events = self.io.snapshot();
        if pending(events) {
            return Err(self.fail(TransferError::DirtyStart(events)));
        }
        self.io.prepare(self.data, self.table);
        self.state = State::Waiting;
        if self.direction == Direction::HostRead {
            self.io.send_length(self.data.length);
            self.io.announce_read();
        }
        Ok(())
    }
    fn poll(&mut self) -> Result<TransferPoll, TransferError> {
        if self.state == State::New {
            return Err(TransferError::NotStarted);
        }
        if self.state == State::Retained {
            return Err(TransferError::Retained);
        }
        self.available()?;
        let events = self.io.snapshot();
        let bits = events.function1.bits();
        let write = self.direction == Direction::HostWrite;
        let start = if write { 0x10 } else { 8 };
        let complete = if write { 1 } else { 2 };
        // Message/ack notifications are supported; suspend/resume/re-enable
        // require separate lifecycle protocols and are not silently ignored.
        let notifications = 0x1200;
        if self.state == State::Waiting {
            if unsupported(events, notifications | start) {
                return Err(self.fail(TransferError::Unsupported(events)));
            }
            if bits & start != 0 {
                let (argument, block) = self.io.request();
                let request = match Function1Request::try_from_cmd53(argument, block, self.direction) {
                    Ok(request) => request,
                    Err(error) => return Err(self.fail(TransferError::Request { error, events })),
                };
                let requested = request.byte_length().get();
                if requested != self.data.length {
                    return Err(self.fail(TransferError::LengthMismatch {
                        requested,
                        reserved: self.data.length,
                        events,
                    }));
                }
                self.io.acknowledge(events);
                if !write {
                    self.io.send_length(requested);
                }
                self.io.publish(self.table.start);
                self.state = State::Running;
                return Ok(TransferPoll { progress: TransferProgress::Running, events });
            }
            self.io.acknowledge(events);
            Ok(TransferPoll { progress: TransferProgress::WaitingForRequest, events })
        } else {
            if bits & 0x84 != 0 {
                return Err(self.fail(TransferError::DeviceError(events)));
            }
            if unsupported(events, notifications | complete | 0x60) {
                return Err(self.fail(TransferError::Unsupported(events)));
            }
            if bits & complete != 0 {
                self.io.mask();
                self.state = State::Retained;
                // Keep completion/error observations intact for diagnosis.
                return Ok(TransferPoll { progress: TransferProgress::CompletionObserved, events });
            }
            self.io.acknowledge(events);
            Ok(TransferPoll { progress: TransferProgress::Running, events })
        }
    }
    fn cancel(&mut self) {
        if matches!(self.state, State::Waiting | State::Running) && self.io.clocks() {
            self.io.mask();
        }
        self.state = State::Retained;
    }
    fn wait(
        &mut self,
        attempts: NonZeroU16,
        delay: &mut impl DelayNs,
        mut notify: impl FnMut(EventSnapshot),
    ) -> Result<EventSnapshot, TransferError> {
        for i in 0..attempts.get() {
            let result = self.poll()?;
            if pending(result.events) {
                notify(result.events);
            }
            if result.progress == TransferProgress::CompletionObserved {
                return Ok(result.events);
            }
            if i + 1 < attempts.get() {
                delay.delay_us(1);
            }
        }
        Err(self.fail(TransferError::Timeout))
    }
}

/// One retained, host-initiated Function1 command. Safe APIs never return DMA
/// memory after start, failure, completion or cancellation. Drop has no MMIO.
///
/// ```compile_fail
/// use hisi_hal::sdio::Transfer;
/// fn premature(t:Transfer<'_>) { t.release_after_external_quiescence(); }
/// ```
/// ```compile_fail
/// use hisi_hal::sdio::Transfer;
/// fn alias(t:&Transfer<'_>) { t.buffer(); }
/// ```
pub struct Transfer<'d> {
    engine: Engine<PacRegisters>,
    token: Sdio<'d>,
    buffer: Buffer,
    table: Table,
}
impl<'d> Transfer<'d> {
    /// Combine disjoint static owners; performs no MMIO or data access.
    /// Length is exactly the reserved buffer length; the host must request it.
    ///
    /// # Safety
    /// Own the initialized/READY SDIO controller, IRQ46, pads, reset and clocks;
    /// serialize all shared clock words and establish prior ADMA physical drain.
    /// Keep local IRQ46 masked except through the exclusive `wait_async` adapter
    /// and its handler. Coordinate one fixed-address Function1 request
    /// only after `start` and no reset/reinit/other user until external recovery.
    /// Establish SDIO access to both coherent static reservations and the actual
    /// descriptor termination profile: VALID|TRAN records followed by an owned
    /// four-byte dummy, stopping according to the command length. This profile
    /// has model/source evidence only; it is not silicon-qualified by this API.
    /// Retain these ownership/coherence obligations after Drop/forget, error or
    /// timeout until independently quiesced. This API neither enumerates CIS nor
    /// negotiates voltage, starts a control-core handoff, or proves bus drain.
    #[allow(clippy::type_complexity)]
    pub unsafe fn new(
        token: Sdio<'d>,
        buffer: Buffer,
        table: Table,
        direction: Direction,
    ) -> Result<Self, (MemoryError, Sdio<'d>, Buffer, Table)> {
        if buffer.range.overlaps(table.range) {
            return Err((MemoryError::Overlap, token, buffer, table));
        }
        let engine = Engine {
            io: PacRegisters { table: table.pointer },
            state: State::New,
            direction,
            data: buffer.range,
            table: table.range,
        };
        Ok(Self { engine, token, buffer, table })
    }
    /// Prepare descriptors and wait for a fresh matching host request. HostRead
    /// advertises the reserved length with the audited readback-OR-one command;
    /// it does not enable CPU interrupts or publish the table before a request.
    pub fn start(&mut self) -> Result<(), TransferError> {
        self.engine.start()
    }
    /// Observe/service one bounded step. Completion retains all memory and status.
    pub fn poll(&mut self) -> Result<TransferPoll, TransferError> {
        self.engine.poll()
    }
    /// One total attempt budget covers host-request and completion waits. This is
    /// not a wall-clock deadline. Notifications reach the callback between polls.
    pub fn wait_bounded(
        &mut self,
        attempts: NonZeroU16,
        delay: &mut impl DelayNs,
        notify: impl FnMut(EventSnapshot),
    ) -> Result<EventSnapshot, TransferError> {
        self.engine.wait(attempts, delay, notify)
    }
    /// Stop software observation and mask known global sources; retain all memory.
    /// Does not abort DMA or prevent already accepted transactions from finishing.
    pub fn cancel(&mut self) {
        self.engine.cancel();
    }
    /// Return all reservations after independent physical quiescence and coherence.
    ///
    /// # Safety
    /// Prove no old host request/table/reset path can issue another read or write,
    /// and every posted transaction has drained. Keep the host stopped until a
    /// new owner is prepared. Completion flags, CPU halt, reset, elapsed time and
    /// clock gating alone do not prove this. Perform required cache invalidation
    /// and CPU/bus synchronization; this function performs no hardware recovery.
    pub unsafe fn release_after_external_quiescence(self) -> (Sdio<'d>, Buffer, Table) {
        compiler_fence(Ordering::Acquire);
        (self.token, self.buffer, self.table)
    }
}

struct PacRegisters {
    table: NonNull<AdmaStorage>,
}
fn regs() -> &'static pac::sdio::RegisterBlock {
    unsafe { &*pac::Sdio::PTR }
}
impl Registers for PacRegisters {
    fn clocks(&mut self) -> bool {
        let r = unsafe { &*pac::MCrg::PTR };
        r.m_clken0_word().read().sdio_device_enable().bit() && r.m_clken1_word().read().sdio_ahb_enable().bit()
    }
    fn configured(&mut self) -> bool {
        regs().host_clock().read().ready().bit() && regs().function1_ready().read().ready().bit()
    }
    fn mask(&mut self) {
        regs()
            .global_interrupt_enable()
            .modify(|_, w| w.function0().clear_bit().memory().clear_bit().function1().clear_bit());
    }
    fn snapshot(&mut self) -> EventSnapshot {
        let r = regs();
        let global = r.global_interrupt_status().read().bits();
        let function0 = r.function0_interrupt_status().read().value().bits();
        let function1 = Function1Events::from_bits(r.function1_interrupt_status().read().bits());
        let host_message = (function1.bits() & 0x200 != 0).then(|| r.host_message().read().value().bits());
        EventSnapshot { global, function0, function1, host_message }
    }
    fn request(&mut self) -> (Cmd53Argument, Option<BlockSize>) {
        (
            Cmd53Argument::from_bits(regs().cmd53_argument().read().bits()),
            BlockSize::try_new(regs().function1_block_size().read().bytes().bits().into()),
        )
    }
    fn prepare(&mut self, data: Range, table: Range) {
        fill_storage(unsafe { self.table.as_mut() }, data, table.start);
    }
    fn acknowledge(&mut self, e: EventSnapshot) {
        unsafe {
            if e.global != 0 {
                regs().global_interrupt_ack().write_with_zero(|w| w.value().bits(e.global));
            }
            if e.function0 != 0 {
                regs().function0_interrupt_ack().write_with_zero(|w| w.value().bits(e.function0));
            }
            if e.function1.bits() != 0 {
                regs().function1_interrupt_ack().write_with_zero(|w| w.value().bits(e.function1.bits()));
            }
        }
    }
    fn send_length(&mut self, bytes: u32) {
        regs().function1_send_length().modify(|_, w| unsafe { w.bytes().bits(bytes) });
    }
    fn announce_read(&mut self) {
        // Audited prepare_send_data ROM 0x1ea12: captured status OR 1 is a
        // full-width command. No W1C/self-clear behavior or read-modify-write
        // interface is invented for the write-only command alias.
        let status = regs().function1_send_status().read().value().bits();
        unsafe {
            regs().function1_send_command().write_with_zero(|w| w.value().bits(status | 1));
        }
    }
    #[cfg(feature = "async")]
    fn listen(&mut self) {
        regs()
            .global_interrupt_enable()
            .modify(|_, w| w.function0().set_bit().memory().set_bit().function1().set_bit());
    }
    fn publish(&mut self, address: u32) {
        compiler_fence(Ordering::Release);
        #[cfg(target_arch = "riscv32")]
        unsafe {
            core::arch::asm!("fence iorw, iorw", options(nostack));
        }
        unsafe {
            regs().adma_table().write_with_zero(|w| w.address().bits(address));
        }
    }
}

#[cfg(all(test, not(target_arch = "riscv32")))]
#[path = "transfer_tests.rs"]
mod tests;

#[cfg(feature = "async")]
#[path = "asynch.rs"]
mod asynch;
#[cfg(feature = "async")]
pub use asynch::{AsyncBindError, AsyncEvent, on_async_interrupt};
