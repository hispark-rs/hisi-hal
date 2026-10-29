use crate::km::{Algorithm, CipherSlot, Parity, Slot, Usage};

/// Source-profile SYMC channel; does not split ownership of shared SPACC.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Channel {
    /// SDK channel 1 (PAC array index 0).
    One,
    /// SDK channel 2 (PAC array index 1).
    Two,
}
impl Channel {
    pub(super) const fn index(self) -> usize {
        match self {
            Self::One => 0,
            Self::Two => 1,
        }
    }
    pub(super) const fn mask(self) -> u32 {
        1 << (self.index() + 1)
    }
}
/// Cipher direction, checked against the working key's loaded permissions.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Direction {
    /// Encryption.
    Encrypt,
    /// Decryption.
    Decrypt,
}
/// Full-block modes audited from the SDK configuration table. No padding,
/// authentication or counter/IV reuse policy is supplied by this low-level API.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Mode {
    /// Independent blocks; ECB does not hide repeated plaintext patterns.
    Ecb,
    /// Cipher block chaining with a complete 16-byte IV.
    Cbc([u8; 16]),
    /// Counter mode with a complete 16-byte starting counter.
    Ctr([u8; 16]),
    /// Output feedback with 128-bit blocks.
    Ofb([u8; 16]),
    /// Cipher feedback with a 128-bit feedback segment.
    Cfb128([u8; 16]),
}
impl Mode {
    pub(super) const fn code(self) -> u8 {
        match self {
            Self::Ecb => 1,
            Self::Cbc(_) => 3,
            Self::Ctr(_) => 6,
            Self::Ofb(_) => 7,
            Self::Cfb128(_) => 8,
        }
    }
    pub(super) const fn iv(self) -> [u8; 16] {
        match self {
            Self::Ecb => [0; 16],
            Self::Cbc(v) | Self::Ctr(v) | Self::Ofb(v) | Self::Cfb128(v) => v,
        }
    }
}
/// Nonzero whole AES/SM4 blocks. The upper limit is the mode-1 RAM envelope;
/// actual allocation capacity and disjointness are checked when preparing.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct BlockLength(u32);
impl BlockLength {
    /// Reject zero, partial blocks, oversized values and narrowing overflow.
    pub const fn from_bytes(bytes: usize) -> Option<Self> {
        if bytes == 0 || bytes > 0x50000 || !bytes.is_multiple_of(16) { None } else { Some(Self(bytes as u32)) }
    }
    /// Exact bytes submitted; no implicit padding or truncation.
    pub const fn bytes(self) -> u32 {
        self.0
    }
}
/// Rejected before any MMIO or descriptor modification. All resources return.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum PrepareError {
    /// A HMAC/unsupported key cannot be attached to this cipher transfer.
    KeyType,
    /// The loaded working key did not permit the requested direction.
    KeyUsage,
    /// Longer AES keys require the even start half.
    KeyParity,
    /// Input or output allocation is too small for the exact block length.
    Capacity,
    /// Input, output and descriptor bus ranges must be disjoint.
    Overlap,
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) struct Configuration {
    pub slot: CipherSlot,
    pub algorithm: u8,
    pub key_length: u8,
    pub parity: Parity,
    pub direction: Direction,
    pub mode: Mode,
}
impl Configuration {
    pub fn new(
        slot: Slot,
        algorithm: Algorithm,
        parity: Parity,
        usage: Option<Usage>,
        direction: Direction,
        mode: Mode,
    ) -> Result<Self, PrepareError> {
        let Slot::Cipher(slot) = slot else { return Err(PrepareError::KeyType) };
        let (algorithm, key_length) = match algorithm {
            Algorithm::Aes128 => (2, 1),
            Algorithm::Aes192 => (2, 2),
            Algorithm::Aes256 => (2, 3),
            Algorithm::Sm4 => (5, 1),
            Algorithm::Hmac(_) => return Err(PrepareError::KeyType),
        };
        if parity == Parity::Odd && key_length != 1 {
            return Err(PrepareError::KeyParity);
        }
        if !matches!(
            (usage, direction),
            (Some(Usage::Both), _)
                | (Some(Usage::Encrypt), Direction::Encrypt)
                | (Some(Usage::Decrypt), Direction::Decrypt)
        ) {
            return Err(PrepareError::KeyUsage);
        }
        Ok(Self { slot, algorithm, key_length, parity, direction, mode })
    }
}
