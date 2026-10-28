//! WS53 SDIO v151 software formats and bounded polling initialization.
//!
//! Sources: SDK 163d9150 `hal_sdio.h` / `sdio_porting.h` and unmodified ROM
//! leaves `hal_sdio_v151_set_adma_table` and `sdio_cmd53_rw_bytes_cnt` executed
//! in QEMU (hisi-registers/evidence/ws53-sdio-rom-contract.json). These prove
//! encoding only, not hardware alignment, accessible memory, cache coherency,
//! transfer completion or bus drain. The initialization session does not submit
//! descriptors, negotiate CIS, deliver local IRQ46, or own DMA buffers. Complete
//! host enumeration and transfers remain separate, unqualified capabilities.
use core::num::NonZeroU16;

#[path = "sdio_ws53/request.rs"]
mod request;
pub use request::{Function1Request, RequestError};

#[cfg(feature = "chip-ws53")]
#[path = "sdio_ws53/controller.rs"]
mod controller;
#[cfg(feature = "chip-ws53")]
pub use controller::{
    EventSnapshot, Function1Event, Function1Events, InitError, InitProgress, Initialization, PollResult,
};

#[cfg(feature = "chip-ws53")]
#[path = "sdio_ws53/transfer.rs"]
mod transfer;
#[cfg(feature = "chip-ws53")]
pub use transfer::{AdmaStorage, Buffer, MemoryError, Table, Transfer, TransferError, TransferPoll, TransferProgress};
#[cfg(all(feature = "chip-ws53", feature = "async"))]
pub use transfer::{AsyncBindError, AsyncEvent, on_async_interrupt};

/// Nonempty descriptor payload representable without the ROM's u16 truncation.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct AdmaLength(NonZeroU16);
impl AdmaLength {
    /// Reject zero and values above 65535. In particular, 65536 is not a
    /// supported special encoding: the audited ROM silently truncates it to 0.
    pub const fn try_new(bytes: u32) -> Option<Self> {
        if bytes > u16::MAX as u32 {
            return None;
        }
        match NonZeroU16::new(bytes as u16) {
            Some(value) => Some(Self(value)),
            None => None,
        }
    }
    /// Exact payload byte count.
    pub const fn get(self) -> u16 {
        self.0.get()
    }
}

/// Slot in the SDK's 129-descriptor software table, not proof of allocation.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct AdmaIndex(u8);
impl AdmaIndex {
    /// Accept the ROM's inclusive range 0..=128.
    pub const fn try_new(index: u32) -> Option<Self> {
        if index <= 128 { Some(Self(index as u8)) } else { None }
    }
    /// Offset in bytes, using the ROM's eight-byte stride.
    pub const fn byte_offset(self) -> usize {
        self.0 as usize * 8
    }
}

/// A serialized ordinary ADMA transfer entry. This owned byte image is not an
/// aligned DMA table, a buffer borrow or permission to submit anything to SDIO.
/// It deliberately has no terminal/link descriptor or raw opcode constructor.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct AdmaTransferImage([u8; 8]);
impl AdmaTransferImage {
    /// Encode the ROM's VALID|TRAN flags (0x21), length, and bus address.
    /// Reject null addresses and ranges wrapping the 32-bit bus address space.
    /// Other addresses are merely representable: RAM ownership, mapping and
    /// hardware alignment are not inferred from this successful conversion.
    pub const fn try_new(bus_address: u32, length: AdmaLength) -> Option<Self> {
        if bus_address == 0 || bus_address.checked_add(length.get() as u32 - 1).is_none() {
            return None;
        }
        let address = bus_address.to_le_bytes();
        let bytes = length.get().to_le_bytes();
        Some(Self([0x21, 0, bytes[0], bytes[1], address[0], address[1], address[2], address[3]]))
    }
    /// Little-endian wire bytes; copying these does not publish a DMA table.
    pub const fn to_le_bytes(self) -> [u8; 8] {
        self.0
    }
}

/// Nonzero block length fitting the observed twelve-bit function-1 field.
/// This checks the source profile's width, not a negotiated host/card limit.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct BlockSize(NonZeroU16);
impl BlockSize {
    /// Accept 1..=4095 without masking. No default negotiated size is assumed.
    pub const fn try_new(bytes: u32) -> Option<Self> {
        if bytes > 4095 {
            return None;
        }
        match NonZeroU16::new(bytes as u16) {
            Some(value) => Some(Self(value)),
            None => None,
        }
    }
    /// Decoded block size in bytes.
    pub const fn get(self) -> u16 {
        self.0.get()
    }
}

/// Direction as seen by the external SDIO host.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Direction {
    /// The host reads data supplied by the WS53 device.
    HostRead,
    /// The host writes data to the WS53 device.
    HostWrite,
}

/// Address mode carried by CMD53; no CPU memory address is implied.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AddressMode {
    /// Repeated transfers to the same function address.
    Fixed,
    /// The function address increments during the transfer.
    Incrementing,
}

/// Length decoding failed; no sentinel is returned as a successful byte count.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LengthError {
    /// Block mode needs a separately validated nonzero block size.
    MissingBlockSize,
    /// The audited WS53 ROM rejects zero block count instead of treating it
    /// as 512 blocks. Byte mode retains its distinct zero-means-512 rule.
    ZeroBlockCount,
}

/// Observed CMD53 argument bits. Parsing does not accept a host request, claim
/// a function, touch a register, or authorize the address range for a transfer.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Cmd53Argument(u32);
impl Cmd53Argument {
    /// Preserve a complete observed argument; every bit has a protocol meaning.
    pub const fn from_bits(bits: u32) -> Self {
        Self(bits)
    }
    /// Original wire value.
    pub const fn bits(self) -> u32 {
        self.0
    }
    /// Function number 0..=7; this is observation, not a supported-function claim.
    pub const fn function(self) -> u8 {
        ((self.0 >> 28) & 7) as u8
    }
    /// Seventeen-bit function-relative address, not a CPU pointer.
    pub const fn address(self) -> u32 {
        (self.0 >> 9) & 0x1ffff
    }
    /// Whether the command counts blocks instead of bytes.
    pub const fn block_mode(self) -> bool {
        self.0 & (1 << 27) != 0
    }
    /// Nine-bit raw count. Zero has different meanings in the two modes.
    pub const fn count(self) -> u16 {
        (self.0 & 0x1ff) as u16
    }
    /// Host-relative transfer direction.
    pub const fn direction(self) -> Direction {
        if self.0 & (1 << 31) == 0 { Direction::HostRead } else { Direction::HostWrite }
    }
    /// Fixed or incrementing function address.
    pub const fn address_mode(self) -> AddressMode {
        if self.0 & (1 << 26) == 0 { AddressMode::Fixed } else { AddressMode::Incrementing }
    }
    /// Decode using WS53 ROM semantics. Byte mode does not use `block_size`.
    /// A block transfer can exceed one descriptor; callers must explicitly split
    /// it later. This function never narrows the resulting count to u16.
    pub const fn byte_length(self, block_size: Option<BlockSize>) -> Result<u32, LengthError> {
        let count = self.count() as u32;
        if !self.block_mode() {
            return Ok(if count == 0 { 512 } else { count });
        }
        if count == 0 {
            return Err(LengthError::ZeroBlockCount);
        }
        match block_size {
            Some(size) => Ok(count * size.get() as u32),
            None => Err(LengthError::MissingBlockSize),
        }
    }
}

#[cfg(test)]
#[path = "sdio_ws53/tests.rs"]
mod tests;
