//! WS53 v151 eFuse read service (SDK 163d9150), with explicit configured handoff.
//! Reads issue only the source's restricted volatile read-mode command. There is
//! no OTP data write, AVDD, programming switch, erase or provisioning API.
//! Chip-ID/die-ID/MAC schemas disagree in the SDK and are intentionally not
//! interpreted here. Raw contents are not a universal identity/provisioning schema.
use crate::{peripherals::Efuse, soc::pac};

/// Logical byte in the single 256-byte region, not a physical MMIO offset.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct EfuseByteAddress(u8);
impl EfuseByteAddress {
    /// Reject addresses outside the evidenced single region.
    pub const fn new(byte: u16) -> Option<Self> {
        if byte < 256 { Some(Self(byte as u8)) } else { None }
    }
    /// Logical byte index.
    pub const fn value(self) -> u8 {
        self.0
    }
}

/// Read-service failure. Unknown controller settings are never repaired silently.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum EfuseError {
    /// Region0 boot-done is clear. No wait or interpretation of manual status.
    NotReady,
    /// Controller timing differs from the source profile's low byte 0x1f.
    TimingMismatch(u8),
    /// Mode differs from read mode 0x5a5a; programming/unknown modes are rejected.
    ModeMismatch(u16),
    /// The complete destination range would exceed the 256-byte region.
    InvalidRange,
}
/// A read failed after producing a prefix; its remaining destination is unchanged.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ReadError {
    completed: usize,
    error: EfuseError,
}
impl ReadError {
    /// Number of bytes written into this call's destination before failure.
    pub const fn completed(self) -> usize {
        self.completed
    }
    /// The error that stopped the operation.
    pub const fn error(self) -> EfuseError {
        self.error
    }
}

/// Configured, exclusively handed-over eFuse read service.
/// No global IRQ lock is taken; the handoff must exclude competing users.
///
/// ```compile_fail
/// let p = hisi_hal::Peripherals::take().unwrap();
/// let _ = unsafe { hisi_hal::efuse::EfuseDriver::from_configured(p.EFUSE) };
/// ```
/// ```compile_fail
/// let p = hisi_hal::Peripherals::take().unwrap();
/// let token = unsafe { p.EFUSE.assume_application_ownership() };
/// let _ = hisi_hal::efuse::EfuseDriver::from_configured(token);
/// ```
pub struct EfuseDriver<'d> {
    token: Efuse<'d>,
    engine: Engine<PacRegisters>,
}
impl<'d> EfuseDriver<'d> {
    /// Inherit the settled v151 read profile without performing MMIO.
    ///
    /// # Safety
    /// Exclude application/control-core boot, security, IRQ and programming
    /// clients. No programming operation may be active, and programming supplies
    /// and switches must already be safely inactive. Establish working clocks,
    /// released reset, completed region0 boot, CLOCK_PERIOD low8=0x1f and mode
    /// low16=0x5a5a. The physical source frequency/settling for this profile must
    /// be qualified externally; this driver does not initialize power/timing.
    ///
    /// Own the entire mode read/command transaction and its unmodeled upper16
    /// bits. Confirm the SDK's repeated 0x5a5a command admits the following
    /// 32-bit data read without additional delay. Preserve exclusive ownership
    /// and MMIO access through errors/release/Drop. This is not IPC negotiation.
    pub unsafe fn from_configured(token: Efuse<'d>) -> Self {
        Self { token, engine: Engine { io: PacRegisters } }
    }

    /// Read one logical byte. Check the inherited profile before the command,
    /// after it, and after the data read. Failed reads return no candidate byte.
    /// The only write is low16=0x5a5a with the previous upper16 preserved.
    pub fn read_byte(&mut self, byte: EfuseByteAddress) -> Result<u8, EfuseError> {
        self.engine.read(byte)
    }

    /// Read at most 256 bytes, validating the entire range before any access.
    /// Empty slices succeed without MMIO. On failure, report the copied prefix
    /// and leave the remaining bytes unchanged. No interrupt-masked bulk loop.
    pub fn read_buffer(&mut self, start: EfuseByteAddress, bytes: &mut [u8]) -> Result<(), ReadError> {
        self.engine.read_buffer(start, bytes)
    }

    /// Return the permission without changing timing, mode, power or contents.
    pub fn release_without_stopping(self) -> Efuse<'d> {
        self.token
    }
}

#[derive(Clone, Copy)]
struct State {
    boot_done: bool,
    period: u8,
    mode: u32,
}
impl State {
    fn check(self) -> Result<(), EfuseError> {
        if !self.boot_done {
            return Err(EfuseError::NotReady);
        }
        if self.period != 0x1f {
            return Err(EfuseError::TimingMismatch(self.period));
        }
        let mode = self.mode as u16;
        if mode != 0x5a5a {
            return Err(EfuseError::ModeMismatch(mode));
        }
        Ok(())
    }
}
trait Registers {
    fn state(&mut self) -> State;
    fn read_command(&mut self, preserved_upper: u32);
    fn data(&mut self, slot: usize, odd: bool) -> u8;
}
struct Engine<R: Registers> {
    io: R,
}
impl<R: Registers> Engine<R> {
    fn read(&mut self, byte: EfuseByteAddress) -> Result<u8, EfuseError> {
        let state = self.io.state();
        state.check()?;
        self.io.read_command(state.mode & 0xffff0000);
        self.io.state().check()?;
        let value = self.io.data(usize::from(byte.0) / 2, byte.0 & 1 != 0);
        self.io.state().check()?;
        Ok(value)
    }
    fn read_buffer(&mut self, start: EfuseByteAddress, bytes: &mut [u8]) -> Result<(), ReadError> {
        if bytes.len() > 256 - usize::from(start.0) {
            return Err(ReadError { completed: 0, error: EfuseError::InvalidRange });
        }
        for (completed, byte) in bytes.iter_mut().enumerate() {
            // The whole range was checked above; no numeric address escapes.
            let address = EfuseByteAddress((usize::from(start.0) + completed) as u8);
            *byte = self.read(address).map_err(|error| ReadError { completed, error })?;
        }
        Ok(())
    }
}
struct PacRegisters;
impl Registers for PacRegisters {
    fn state(&mut self) -> State {
        let r = unsafe { &*pac::Efuse::PTR };
        State {
            boot_done: r.boot_status().read().boot0_done().bit_is_set(),
            period: r.clock_period().read().value().bits(),
            mode: r.mode_status().read().bits(),
        }
    }
    fn read_command(&mut self, preserved_upper: u32) {
        // RDL intentionally leaves upper16 opaque; the original SDK preserves
        // them. This is a complete WO command, not RMW on its WO accessor.
        // Satisfy the PAC's restricted low16 write constraint explicitly.
        unsafe {
            (&*pac::Efuse::PTR)
                .read_mode_command()
                .write_with_zero(|w| w.bits((preserved_upper & 0xffff0000) | 0x5a5a));
        }
    }
    fn data(&mut self, slot: usize, odd: bool) -> u8 {
        let value = unsafe { &*pac::Efuse::PTR }.data_read(slot).read();
        if odd { value.odd_byte().bits() } else { value.even_byte().bits() }
    }
}

#[cfg(all(test, not(target_arch = "riscv32")))]
#[path = "efuse_ws53_tests.rs"]
mod tests;
