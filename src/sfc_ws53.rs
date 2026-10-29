//! Experimental WS53 CPU-command reads, SDK 163d9150 / RDL 92a3d202.
//!
//! This is a configured, RAM-only attachment: no automatic clock/reset, mapping,
//! DMA, IRQ, program, erase or arbitrary-opcode API. No command-start idle bit
//! proves prior DMA drain or safe concurrent XIP. A failed transaction poisons
//! the instance permanently; recovery requires a new external handoff.
use crate::{peripherals::SfcCfg, soc::pac};
use core::num::{NonZeroU32, NonZeroU64};

/// A 24-bit NOR byte address; this is an address-width limit, not chip capacity.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct FlashAddress(u32);
impl FlashAddress {
    /// Reject rather than truncate an address outside the three-byte protocol.
    pub const fn try_new(value: u32) -> Option<Self> {
        if value < 0x0100_0000 { Some(Self(value)) } else { None }
    }
    /// Address sent to the NOR, not a CPU/XIP address.
    pub const fn get(self) -> u32 {
        self.0
    }
}

/// One command's payload length, bounded by the controller's 64-byte window.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ReadLength(u8);
impl ReadLength {
    /// Accept exactly 1..=64; no implicit splitting, clamping or empty command.
    pub const fn try_new(value: usize) -> Option<Self> {
        if value >= 1 && value <= 64 { Some(Self(value as u8)) } else { None }
    }
    /// Number of bytes, before the SDK's length-minus-one encoding.
    pub const fn get(self) -> usize {
        self.0 as usize
    }
}

/// Explicit allowed region inside the externally established device capacity.
/// Construction checks arithmetic only; it does not discover a NOR or authorize
/// access to its contents. The unsafe handoff must establish both facts.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ReadRegion {
    start: FlashAddress,
    end: u32,
}
impl ReadRegion {
    /// Reject zero size, overflow, and any range crossing the 24-bit boundary.
    pub const fn try_new(start: FlashAddress, bytes: NonZeroU32) -> Option<Self> {
        let end = start.0 as u64 + bytes.get() as u64;
        if end <= 0x0100_0000 { Some(Self { start, end: end as u32 }) } else { None }
    }
    /// Does the complete command fit inside the handed-over read region?
    pub const fn contains(self, address: FlashAddress, length: ReadLength) -> bool {
        address.0 >= self.start.0 && address.0 as u64 + length.0 as u64 <= self.end as u64
    }
}

/// Status command identity, not an arbitrary instruction or register index.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum StatusRegister {
    /// Opcode 05. This source profile uses bit0 as WIP for ordinary reads.
    Status1,
    /// Opcode 35. Raw byte only; no portable protection-bit interpretation.
    Status2,
    /// Opcode 15. Support must be established for the attached NOR.
    Status3,
}
impl StatusRegister {
    fn opcode(self) -> u8 {
        match self {
            Self::Status1 => 0x05,
            Self::Status2 => 0x35,
            Self::Status3 => 0x15,
        }
    }
}

/// Independent, bounded time-source read. Values are ticks, not microseconds.
/// Each call must return within a known bound, including fault paths. The source
/// must keep its rate and epoch, and keep progressing independently of SFC. A
/// decreasing value or `None` is an error; the separate poll limit also catches
/// a stopped source. This trait does not establish clock or RAM ownership.
pub trait TickSource {
    /// Sample monotonically increasing ticks, or report a failed sample.
    fn now(&mut self) -> Option<u64>;
}
impl TickSource for crate::tcxo::TcxoDriver<'_> {
    fn now(&mut self) -> Option<u64> {
        self.read_counter().ok().map(|n| n.ticks())
    }
}

/// Independent elapsed-tick deadline plus total START-poll limit per public call.
#[derive(Clone, Copy, Debug)]
pub struct Budget {
    ticks: NonZeroU64,
    polls: NonZeroU32,
}
impl Budget {
    /// Both bounds are explicit and nonzero. Frequency conversion is external;
    /// neither bound can interrupt a stalled physical MMIO bus transaction.
    pub const fn new(ticks: NonZeroU64, polls: NonZeroU32) -> Self {
        Self { ticks, polls }
    }
}

/// Readable admission failure; observing these values never proves ownership.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AdmissionError {
    /// Command START was already set; no new command was issued.
    CommandBusy,
    /// DMA_START is set. No attempt is made to stop or drain it.
    DmaBusy,
    /// The externally configured SFC clock-enable bit is clear.
    ClockDisabled,
    /// Only SDK source codes 0 and 6 are admitted, without a frequency claim.
    ClockSource,
    /// This initial CPU-read profile admits SPI mode0 only.
    SpiMode,
    /// This driver only sends three-byte addresses.
    AddressMode,
    /// Shared GLOBAL or clock state changed after attachment.
    ConfigurationChanged,
}
/// Transaction stage for deadline/poll errors.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Stage {
    /// Fresh 05 status reads before any non-05 operation (9f/35/15/03).
    NorReady,
    /// Controller command or data retrieval.
    Command,
}
/// No error result contains data, and a failed read leaves its buffer unchanged.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Error {
    /// A prior transaction failed. No further register or clock access occurs.
    Poisoned,
    /// Invalid slice size, including zero or a length not matching ReadLength.
    BufferLength,
    /// The entire command falls outside the allowed region.
    OutsideRegion,
    /// A readable handoff condition failed.
    Admission(AdmissionError),
    /// Independent time source failed or moved backwards.
    TimeSource,
    /// Elapsed ticks reached the explicit deadline.
    Deadline(Stage),
    /// Finite total START polls exhausted, even if the time source stopped.
    PollLimit(Stage),
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct Snapshot {
    global: u32,
    dma: u32,
    command: u32,
    clock: u32,
}
impl Snapshot {
    fn validate(self) -> Result<(), AdmissionError> {
        if self.command & 1 != 0 {
            return Err(AdmissionError::CommandBusy);
        }
        if self.dma & 1 != 0 {
            return Err(AdmissionError::DmaBusy);
        }
        if self.clock & 1 == 0 {
            return Err(AdmissionError::ClockDisabled);
        }
        if !matches!((self.clock >> 1) & 7, 0 | 6) {
            return Err(AdmissionError::ClockSource);
        }
        if self.global & 1 != 0 {
            return Err(AdmissionError::SpiMode);
        }
        if self.global & 4 != 0 {
            return Err(AdmissionError::AddressMode);
        }
        Ok(())
    }
}
struct Deadline {
    start: u64,
    last: u64,
    remaining: u32,
    ticks: u64,
}
impl Deadline {
    fn new(clock: &mut impl TickSource, budget: Budget) -> Result<Self, Error> {
        let start = clock.now().ok_or(Error::TimeSource)?;
        Ok(Self { start, last: start, remaining: budget.polls.get(), ticks: budget.ticks.get() })
    }
    fn check(&mut self, clock: &mut impl TickSource, stage: Stage) -> Result<(), Error> {
        let now = clock.now().ok_or(Error::TimeSource)?;
        if now < self.last {
            return Err(Error::TimeSource);
        }
        self.last = now;
        if now - self.start >= self.ticks {
            return Err(Error::Deadline(stage));
        }
        Ok(())
    }
    fn poll(&mut self, clock: &mut impl TickSource, stage: Stage) -> Result<(), Error> {
        self.check(clock, stage)?;
        if self.remaining == 0 {
            return Err(Error::PollLimit(stage));
        }
        self.remaining -= 1;
        Ok(())
    }
}
#[derive(Clone, Copy)]
enum Command {
    Jedec,
    Status(StatusRegister),
    Read(FlashAddress, ReadLength),
}
impl Command {
    fn shape(self) -> (u8, Option<u32>, usize) {
        match self {
            Self::Jedec => (0x9f, None, 3),
            Self::Status(status) => (status.opcode(), None, 1),
            Self::Read(address, length) => (0x03, Some(address.0), length.get()),
        }
    }
}
// Private to prevent applications substituting an unaudited MMIO backend.
trait Registers {
    fn snapshot(&mut self) -> Snapshot;
    fn start(&mut self, command: Command);
    fn busy(&mut self) -> bool;
    fn data_word(&mut self, index: usize) -> u32;
}
struct Engine<R> {
    io: R,
    initial: Snapshot,
    poisoned: bool,
}
impl<R: Registers> Engine<R> {
    fn attach(mut io: R) -> Result<Self, AdmissionError> {
        let initial = io.snapshot();
        initial.validate()?;
        Ok(Self { io, initial, poisoned: false })
    }
    fn guard(&mut self) -> Result<(), Error> {
        let now = self.io.snapshot();
        now.validate().map_err(Error::Admission)?;
        if now.global != self.initial.global || now.clock != self.initial.clock {
            return Err(Error::Admission(AdmissionError::ConfigurationChanged));
        }
        Ok(())
    }
    fn command(
        &mut self,
        command: Command,
        deadline: &mut Deadline,
        clock: &mut impl TickSource,
        stage: Stage,
    ) -> Result<[u8; 64], Error> {
        self.guard()?;
        deadline.check(clock, stage)?;
        if deadline.remaining == 0 {
            return Err(Error::PollLimit(stage));
        }
        self.io.start(command);
        loop {
            deadline.poll(clock, stage)?;
            if !self.io.busy() {
                break;
            }
        }
        self.guard()?;
        // SDK reads ceil(n/4) ordinary words after START clears, then copies n
        // little-endian bytes. Never inspect DATA to infer completion or WIP.
        let mut bytes = [0u8; 64];
        let length = command.shape().2;
        for index in 0..length.div_ceil(4) {
            bytes[index * 4..index * 4 + 4].copy_from_slice(&self.io.data_word(index).to_le_bytes());
        }
        deadline.check(clock, stage)?;
        self.guard()?;
        Ok(bytes)
    }
    fn run(&mut self, command: Command, budget: Budget, clock: &mut impl TickSource) -> Result<[u8; 64], Error> {
        if self.poisoned {
            return Err(Error::Poisoned);
        }
        // Remains poisoned on any error or unwinding; no Drop/reset writes.
        self.poisoned = true;
        let mut deadline = Deadline::new(clock, budget)?;
        if !matches!(command, Command::Status(StatusRegister::Status1)) {
            // SDK wait_ready(0): every WIP observation is a NEW completed 05.
            while self.command(Command::Status(StatusRegister::Status1), &mut deadline, clock, Stage::NorReady)?[0] & 1
                != 0
            {}
        }
        let bytes = self.command(command, &mut deadline, clock, Stage::Command)?;
        self.poisoned = false;
        Ok(bytes)
    }
    fn read(
        &mut self,
        region: ReadRegion,
        address: FlashAddress,
        length: ReadLength,
        output: &mut [u8],
        budget: Budget,
        clock: &mut impl TickSource,
    ) -> Result<(), Error> {
        if self.poisoned {
            return Err(Error::Poisoned);
        }
        if output.len() != length.get() {
            return Err(Error::BufferLength);
        }
        if !region.contains(address, length) {
            return Err(Error::OutsideRegion);
        }
        let data = self.run(Command::Read(address, length), budget, clock)?;
        output.copy_from_slice(&data[..length.get()]);
        Ok(())
    }
}

/// Read-only CPU command session. Requires `chip-ws53,unstable` and external
/// configured ownership. It deliberately offers no safe free/recover operation.
/// Dropping it sends no command and does not prove the controller is idle.
pub struct ReadOnly<'d, C> {
    _token: SfcCfg<'d>,
    engine: Engine<PacRegisters<'static>>,
    clock: C,
    region: ReadRegion,
    budget: Budget,
}
impl<'d, C: TickSource> ReadOnly<'d, C> {
    /// Attach by reading GLOBAL, DMA_START, CMD_START and SFC clock state only.
    /// Invalid conditions return an error without writing the controller.
    ///
    /// # Safety
    /// Establish exclusive ownership of the complete command/data bank, Flash
    /// bus, native CS1 and their configured single-line pads, with released reset
    /// and a continuously active, electrically qualified source clock. The NOR
    /// must support 9f/05/35/15/03, three-byte mode and 05 bit0 WIP semantics;
    /// `region` must fit its verified capacity and the caller's permitted range.
    /// Stop all application/control-core, IRQ/NMI, debugger and DMA users and
    /// prove old requests/linked lists/prefetches have drained. A CPU halt, idle
    /// START or DMA_START=0 is insufficient. Prevent IPC services, sleep, shared
    /// clock/reset/routing and mapping changes for the session's entire lifetime.
    /// Every reachable instruction, literal, vector, ISR, panic path, stack and
    /// buffer (including TickSource and compiler helpers) must be in independently
    /// accessible RAM, not Flash/XIP. Establish cache and prefetch/bus conditions
    /// that exclude all concurrent XIP accesses from every core/master. Merely
    /// disabling a cache does not prove completed maintenance or bus drain.
    /// Keep SFC MMIO and the independent bounded TickSource accessible, including
    /// failure paths. No IRQ is enabled here; its routing/old handlers must be
    /// externally isolated. This call performs no inter-core negotiation and no
    /// clock enable. After poison/Drop, do not reuse this resource without a new
    /// independently established handoff; an elapsed timeout is not cancellation.
    pub unsafe fn from_configured(
        token: SfcCfg<'d>,
        region: ReadRegion,
        budget: Budget,
        clock: C,
    ) -> Result<Self, AdmissionError> {
        let engine =
            Engine::attach(PacRegisters { sfc: unsafe { &*SfcCfg::ptr() }, clocks: unsafe { &*pac::MCrg::PTR } })?;
        Ok(Self { _token: token, engine, clock, region, budget })
    }
    /// Whether any transaction has failed; subsequent calls perform no MMIO.
    pub fn is_poisoned(&self) -> bool {
        self.engine.poisoned
    }
    /// Read three raw JEDEC bytes after fresh 05/WIP readiness. No capacity/vendor inference is performed.
    pub fn jedec_id(&mut self) -> Result<[u8; 3], Error> {
        let data = self.engine.run(Command::Jedec, self.budget, &mut self.clock)?;
        Ok([data[0], data[1], data[2]])
    }
    /// Read a fresh completed status command; no protection bits are changed.
    /// Status1 may report WIP; Status2/3 first require bounded fresh 05 readiness.
    pub fn status(&mut self, status: StatusRegister) -> Result<u8, Error> {
        Ok(self.engine.run(Command::Status(status), self.budget, &mut self.clock)?[0])
    }
    /// Read one 1..64-byte command after bounded, fresh 05/WIP readiness checks.
    /// The explicit length must equal the slice size, and the entire interval
    /// must fit the allowed region. On every returned error, `output` is intact.
    pub fn read(&mut self, address: FlashAddress, length: ReadLength, output: &mut [u8]) -> Result<(), Error> {
        self.engine.read(self.region, address, length, output, self.budget, &mut self.clock)
    }
}

struct PacRegisters<'r> {
    sfc: &'r pac::sfc_cfg::RegisterBlock,
    clocks: &'r pac::m_crg::RegisterBlock,
}
impl PacRegisters<'_> {
    fn regs(&self) -> &pac::sfc_cfg::RegisterBlock {
        self.sfc
    }
}
impl Registers for PacRegisters<'_> {
    fn snapshot(&mut self) -> Snapshot {
        let r = self.regs();
        Snapshot {
            global: r.global_config().read().bits(),
            dma: r.bus_dma_ctrl().read().bits(),
            command: r.cmd_config().read().bits(),
            clock: self.clocks.xip_sfc_clock_word().read().bits(),
        }
    }
    fn start(&mut self, command: Command) {
        let (opcode, address, length) = command.shape();
        let r = self.regs();
        r.cmd_ins().modify(|_, w| unsafe { w.reg_ins().bits(opcode) });
        if let Some(address) = address {
            unsafe { r.cmd_addr().write_with_zero(|w| w.cmd_addr().bits(address)) };
        }
        // SDK hal_sfc_regs_set_opt/set_opt_attr, complete non-DDR standard-SPI
        // configuration; length is validated before the lossless u8 conversion.
        unsafe {
            r.cmd_config().write_with_zero(|w| {
                w.sel_cs()
                    .set_bit()
                    .addr_en()
                    .bit(address.is_some())
                    .data_en()
                    .set_bit()
                    .rw()
                    .set_bit()
                    .data_cnt()
                    .bits((length - 1) as u8)
                    .start()
                    .set_bit()
            })
        };
    }
    fn busy(&mut self) -> bool {
        self.regs().cmd_config().read().start().bit_is_set()
    }
    fn data_word(&mut self, index: usize) -> u32 {
        let r = self.regs();
        // PAC has 16 named slots; no raw pointer arithmetic or MMIO literals.
        match index {
            0 => r.cmd_databuf_0().read().bits(),
            1 => r.cmd_databuf_1().read().bits(),
            2 => r.cmd_databuf_2().read().bits(),
            3 => r.cmd_databuf_3().read().bits(),
            4 => r.cmd_databuf_4().read().bits(),
            5 => r.cmd_databuf_5().read().bits(),
            6 => r.cmd_databuf_6().read().bits(),
            7 => r.cmd_databuf_7().read().bits(),
            8 => r.cmd_databuf_8().read().bits(),
            9 => r.cmd_databuf_9().read().bits(),
            10 => r.cmd_databuf_10().read().bits(),
            11 => r.cmd_databuf_11().read().bits(),
            12 => r.cmd_databuf_12().read().bits(),
            13 => r.cmd_databuf_13().read().bits(),
            14 => r.cmd_databuf_14().read().bits(),
            15 => r.cmd_databuf_15().read().bits(),
            _ => unreachable!("private validated command exceeded 16 words"),
        }
    }
}

#[cfg(test)]
#[path = "sfc_ws53/tests.rs"]
mod tests;
