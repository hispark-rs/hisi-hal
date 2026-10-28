//! Typed key domains and validated byte lengths; no MMIO or allocation.

/// SDK application cipher keyslot capacity; distinct from a HMAC slot number.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum CipherSlot {
    /// Cipher slot0.
    Zero,
    /// Cipher slot1.
    One,
    /// Cipher slot2.
    Two,
    /// Cipher slot3.
    Three,
    /// Cipher slot4.
    Four,
    /// Cipher slot5.
    Five,
    /// Cipher slot6.
    Six,
    /// Cipher slot7.
    Seven,
}
impl CipherSlot {
    /// All source-defined application cipher slots.
    pub const ALL: [Self; 8] =
        [Self::Zero, Self::One, Self::Two, Self::Three, Self::Four, Self::Five, Self::Six, Self::Seven];
}
/// SDK application HMAC slot capacity.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum HmacSlot {
    /// HMAC slot0.
    Zero,
    /// HMAC slot1.
    One,
}
impl HmacSlot {
    /// Both source-defined application HMAC slots.
    pub const ALL: [Self; 2] = [Self::Zero, Self::One];
}
/// Distinguishes key-controller namespaces; equal numeric indices are separate slots.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Slot {
    /// Cipher key namespace.
    Cipher(CipherSlot),
    /// HMAC key namespace.
    Hmac(HmacSlot),
}
impl Slot {
    pub(super) fn index(self) -> usize {
        match self {
            Self::Cipher(s) => s as usize,
            Self::Hmac(s) => 8 + s as usize,
        }
    }
    pub(super) fn number(self) -> u32 {
        match self {
            Self::Cipher(s) => s as u32,
            Self::Hmac(s) => s as u32,
        }
    }
    pub(super) fn hmac(self) -> bool {
        matches!(self, Self::Hmac(_))
    }
}
/// Per-working-key direction permissions, not global security policy.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Usage {
    /// Permit encryption with the working key.
    Encrypt,
    /// Permit decryption with the working key.
    Decrypt,
    /// Permit both directions.
    Both,
}
/// Half-slot selection for 128-bit keys. Longer AES keys require both halves.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Parity {
    /// Even half, or the first half of a longer AES key.
    Even,
    /// Odd half; rejected for 192/256-bit AES keys.
    Odd,
}
/// Size-checked borrowed cipher keys; deliberately has no secret-bearing Debug.
pub enum CipherKey<'a> {
    /// A 128-bit AES key.
    Aes128(&'a [u8; 16]),
    /// A 192-bit AES key, submitted as 16 bytes then 8 bytes padded to 16.
    Aes192(&'a [u8; 24]),
    /// A 256-bit AES key, submitted in two 16-byte stages.
    Aes256(&'a [u8; 32]),
    /// A 128-bit SM4 key.
    Sm4(&'a [u8; 16]),
}
/// HMAC algorithm selection; enum gaps from the SDK cannot be represented.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum HmacAlgorithm {
    /// HMAC-SHA1; included for SDK compatibility.
    Sha1,
    /// HMAC-SHA224.
    Sha224,
    /// HMAC-SHA256.
    Sha256,
    /// HMAC-SHA384.
    Sha384,
    /// HMAC-SHA512.
    Sha512,
    /// HMAC-SM3.
    Sm3,
}
impl HmacAlgorithm {
    /// Maximum raw key length for this loader's already-normalized key form.
    pub const fn block_bytes(self) -> usize {
        match self {
            Self::Sha384 | Self::Sha512 => 128,
            _ => 64,
        }
    }
}
/// Metadata only, never the key bytes.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Algorithm {
    /// AES with 128-bit key.
    Aes128,
    /// AES with 192-bit key.
    Aes192,
    /// AES with 256-bit key.
    Aes256,
    /// SM4 with 128-bit key.
    Sm4,
    /// A specific HMAC algorithm.
    Hmac(HmacAlgorithm),
}
/// The caller must normalize a long HMAC key before loading it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct KeyLengthError {
    /// Actual caller length.
    pub actual: usize,
    /// Maximum length for the selected HMAC algorithm.
    pub maximum: usize,
}
/// Validated borrowed HMAC key; empty keys are permitted and zero-padded.
pub struct HmacKey<'a> {
    algorithm: HmacAlgorithm,
    bytes: &'a [u8],
}
impl<'a> HmacKey<'a> {
    /// Validate before any hardware access. No silent truncation or implicit hashing.
    pub fn new(algorithm: HmacAlgorithm, bytes: &'a [u8]) -> Result<Self, KeyLengthError> {
        let maximum = algorithm.block_bytes();
        if bytes.len() > maximum {
            return Err(KeyLengthError { actual: bytes.len(), maximum });
        }
        Ok(Self { algorithm, bytes })
    }
}
pub(super) enum Material<'a> {
    Cipher(CipherKey<'a>, Usage, Parity),
    Hmac(HmacKey<'a>),
}
impl Material<'_> {
    pub(super) fn usage(&self) -> Option<Usage> {
        match self {
            Self::Cipher(_, usage, _) => Some(*usage),
            Self::Hmac(_) => None,
        }
    }
    pub(super) fn algorithm(&self) -> Algorithm {
        match self {
            Self::Cipher(CipherKey::Aes128(_), _, _) => Algorithm::Aes128,
            Self::Cipher(CipherKey::Aes192(_), _, _) => Algorithm::Aes192,
            Self::Cipher(CipherKey::Aes256(_), _, _) => Algorithm::Aes256,
            Self::Cipher(CipherKey::Sm4(_), _, _) => Algorithm::Sm4,
            Self::Hmac(key) => Algorithm::Hmac(key.algorithm),
        }
    }
    pub(super) fn bytes(&self) -> &[u8] {
        match self {
            Self::Cipher(CipherKey::Aes128(k) | CipherKey::Sm4(k), _, _) => *k,
            Self::Cipher(CipherKey::Aes192(k), _, _) => *k,
            Self::Cipher(CipherKey::Aes256(k), _, _) => *k,
            Self::Hmac(k) => k.bytes,
        }
    }
    pub(super) fn blocks(&self) -> usize {
        match self {
            Self::Cipher(_, _, _) => self.bytes().len().div_ceil(16),
            Self::Hmac(k) => k.algorithm.block_bytes() / 16,
        }
    }
    pub(super) fn config(&self) -> (u8, bool, bool) {
        match self {
            Self::Cipher(k, usage, _) => (
                if matches!(k, CipherKey::Sm4(_)) { 0x50 } else { 0x20 },
                *usage != Usage::Decrypt,
                *usage != Usage::Encrypt,
            ),
            Self::Hmac(k) => (
                match k.algorithm {
                    HmacAlgorithm::Sha1 => 0xa0,
                    HmacAlgorithm::Sm3 => 0xa2,
                    _ => 0xa1,
                },
                true,
                false,
            ),
        }
    }
    pub(super) fn size_code(&self) -> u32 {
        match self {
            Self::Cipher(_, _, _) => ((self.bytes().len() - 8) / 8) as u32,
            Self::Hmac(_) => 0,
        }
    }
    pub(super) fn parity(&self) -> Parity {
        match self {
            Self::Cipher(_, _, parity) => *parity,
            Self::Hmac(_) => Parity::Even,
        }
    }
    pub(super) fn valid_parity(&self) -> bool {
        self.parity() == Parity::Even || self.bytes().len() == 16
    }
    pub(super) fn half(&self, block: usize) -> usize {
        if self.parity() == Parity::Odd { 1 } else { block }
    }
    pub(super) fn word(&self, block: usize, index: usize) -> u32 {
        let bytes = self.bytes();
        let mut word = [0u8; 4];
        for (i, byte) in word.iter_mut().enumerate() {
            *byte = bytes.get(block * 16 + index * 4 + i).copied().unwrap_or(0);
        }
        u32::from_le_bytes(word)
    }
}
