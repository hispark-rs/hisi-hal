//! Encoding follows SDK 163d9150; source C fixtures cover every padding tail.

/// Unkeyed SPACC HASH algorithm. SHA-1 is supplied for legacy interoperability;
/// it must not be used where collision resistance is required. No HMAC/keyslot
/// or key-management capability is implied by these variants.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Algorithm {
    /// SHA-1, 20 output bytes.
    Sha1,
    /// SHA-224, 28 output bytes.
    Sha224,
    /// SHA-256, 32 output bytes.
    Sha256,
    /// SHA-384, 48 output bytes.
    Sha384,
    /// SHA-512, 64 output bytes.
    Sha512,
    /// SM3, 32 output bytes.
    Sm3,
}

impl Algorithm {
    /// Compression block size in bytes.
    pub const fn block_size(self) -> usize {
        match self {
            Self::Sha384 | Self::Sha512 => 128,
            _ => 64,
        }
    }
    /// Digest size in bytes, including SHA-224/SHA-384 truncation.
    pub const fn digest_size(self) -> usize {
        match self {
            Self::Sha1 => 20,
            Self::Sha224 => 28,
            Self::Sha256 | Self::Sm3 => 32,
            Self::Sha384 => 48,
            Self::Sha512 => 64,
        }
    }
    /// Largest message admitted by this initial one-shot backend. Encoded data
    /// stays within the SDK's 256-byte bounce-buffer profile. This conservative
    /// software limit is not a claim about the hardware's maximum DMA length.
    pub const fn max_message_size(self) -> usize {
        256 - 1 - self.suffix_size()
    }
    pub(super) fn prepared_len(self, length: usize) -> Result<usize, FormatError> {
        let padded = self.padded_len(length)?;
        if padded > 256 {
            return Err(FormatError::ProfileLength);
        }
        Ok(padded)
    }
    pub(super) const fn suffix_size(self) -> usize {
        match self {
            Self::Sha384 | Self::Sha512 => 16,
            _ => 8,
        }
    }
    pub(super) const fn hardware(self) -> (u8, u8) {
        match self {
            Self::Sha1 => (10, 0),
            Self::Sha224 => (11, 0),
            Self::Sha256 => (11, 1),
            Self::Sha384 => (11, 2),
            Self::Sha512 => (11, 3),
            Self::Sm3 => (12, 0),
        }
    }
    pub(super) const fn state_words(self) -> usize {
        match self {
            Self::Sha1 => 5,
            Self::Sha384 | Self::Sha512 => 16,
            _ => 8,
        }
    }
    pub(super) fn initial_words(self) -> [u32; 16] {
        // Standard IV notation; SDK memcpy of IV bytes to the LE target's
        // state buffer makes each state-port word byte-swapped from notation.
        let mut state = match self {
            Self::Sha1 => [0x67452301, 0xefcdab89, 0x98badcfe, 0x10325476, 0xc3d2e1f0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            Self::Sha224 => [
                0xc1059ed8, 0x367cd507, 0x3070dd17, 0xf70e5939, 0xffc00b31, 0x68581511, 0x64f98fa7, 0xbefa4fa4, 0, 0,
                0, 0, 0, 0, 0, 0,
            ],
            Self::Sha256 => [
                0x6a09e667, 0xbb67ae85, 0x3c6ef372, 0xa54ff53a, 0x510e527f, 0x9b05688c, 0x1f83d9ab, 0x5be0cd19, 0, 0,
                0, 0, 0, 0, 0, 0,
            ],
            Self::Sha384 => [
                0xcbbb9d5d, 0xc1059ed8, 0x629a292a, 0x367cd507, 0x9159015a, 0x3070dd17, 0x152fecd8, 0xf70e5939,
                0x67332667, 0xffc00b31, 0x8eb44a87, 0x68581511, 0xdb0c2e0d, 0x64f98fa7, 0x47b5481d, 0xbefa4fa4,
            ],
            Self::Sha512 => [
                0x6a09e667, 0xf3bcc908, 0xbb67ae85, 0x84caa73b, 0x3c6ef372, 0xfe94f82b, 0xa54ff53a, 0x5f1d36f1,
                0x510e527f, 0xade682d1, 0x9b05688c, 0x2b3e6c1f, 0x1f83d9ab, 0xfb41bd6b, 0x5be0cd19, 0x137e2179,
            ],
            Self::Sm3 => [
                0x7380166f, 0x4914b2b9, 0x172442d7, 0xda8a0600, 0xa96f30bc, 0x163138aa, 0xe38dee4d, 0xb0fb0e4e, 0, 0,
                0, 0, 0, 0, 0, 0,
            ],
        };
        for word in &mut state {
            *word = u32::swap_bytes(*word);
        }
        state
    }

    /// Required one-shot DMA capacity, including software padding. Length is
    /// bounded by the 32-bit descriptor, independent of host pointer width.
    /// This format calculation does not override `max_message_size` when
    /// preparing a hardware transfer or establish a hardware length limit.
    pub fn padded_len(self, length: usize) -> Result<usize, FormatError> {
        let block = self.block_size();
        let size = length
            .checked_add(1 + self.suffix_size())
            .and_then(|n| n.checked_add(block - 1))
            .ok_or(FormatError::Length)?
            / block
            * block;
        if size > u32::MAX as usize {
            return Err(FormatError::Length);
        }
        Ok(size)
    }
    pub(super) fn encode(self, input: &[u8], output: &mut [u8]) -> Result<usize, FormatError> {
        let size = self.padded_len(input.len())?;
        if size > output.len() {
            return Err(FormatError::Capacity);
        }
        output[..size].fill(0);
        output[..input.len()].copy_from_slice(input);
        output[input.len()] = 0x80;
        // Input is below 2^32 bytes. The high half of a 128-bit length is zero,
        // but the low 64 bits still carry correctly across the 512MiB boundary.
        let bits = (input.len() as u64) * 8;
        output[size - 8..size].copy_from_slice(&bits.to_be_bytes());
        Ok(size)
    }
}

/// Invalid whole-message encoding; the output remains untouched on error.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FormatError {
    /// Padding/descriptor length would overflow the supported 32-bit encoding.
    Length,
    /// Owned DMA allocation cannot hold the message and padding.
    Capacity,
    /// Encoded message exceeds the initial 256-byte SDK transfer profile.
    /// Streaming and larger one-shot submissions are not exposed yet.
    ProfileLength,
}

/// Copied digest bytes; does not expose or release any DMA allocation.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Digest {
    bytes: [u8; 64],
    length: u8,
}
impl Digest {
    pub(super) fn from_state(algorithm: Algorithm, state: [u32; 16]) -> Self {
        let mut bytes = [0; 64];
        for (word, destination) in state.iter().zip(bytes.as_chunks_mut::<4>().0.iter_mut()) {
            destination.copy_from_slice(&word.to_le_bytes());
        }
        Self { bytes, length: algorithm.digest_size() as u8 }
    }
    /// Digest bytes in conventional display order.
    pub fn as_bytes(&self) -> &[u8] {
        &self.bytes[..self.length as usize]
    }
}
