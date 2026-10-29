use super::{Algorithm, FormatError};
use core::{
    marker::PhantomData,
    ptr::NonNull,
    sync::atomic::{Ordering, compiler_fence},
};

/// Invalid static DMA allocation; nothing has been submitted.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MemoryError {
    /// Allocation is not 32-byte aligned/sized or exceeds MCU_MODE_1 DTCM.
    Range,
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) struct Range {
    pub start: u32,
    pub length: u32,
}
impl Range {
    fn new(start: usize, length: usize) -> Result<Self, MemoryError> {
        let end = start.checked_add(length).ok_or(MemoryError::Range)?;
        if length == 0 || (start | length) & 31 != 0 || start < 0x2001_0000 || end > 0x2006_0000 {
            return Err(MemoryError::Range);
        }
        Ok(Self { start: start as u32, length: length as u32 })
    }
}

/// Unpublished static message storage. Drop/forget keeps the allocation reserved.
/// It cannot be constructed from temporary stack storage:
/// ```compile_fail
/// use hisi_hal::hash::Input;
/// fn local() { let mut bytes = [0u8; 128]; unsafe { Input::from_coherent_static(&mut bytes) }; }
/// ```
pub struct Input {
    pointer: NonNull<u8>,
    pub(super) range: Range,
    _owned: PhantomData<&'static mut [u8]>,
}
impl Input {
    /// Reserve application RAM; no contents are accessed and no DMA is started.
    /// The range check is only the mode-1 envelope, not actual RAM ownership.
    /// The SDK's similarly named allocator only requests aligned heap memory;
    /// it is not evidence of coherence. This constructor performs no cache
    /// maintenance or memory-attribute/clock configuration.
    ///
    /// # Safety
    /// Reserve the whole allocation and its cache lines exclusively for this
    /// owner. It must be bus-accessible to SPACC, identity-addressed, hardware
    /// coherent or noncacheable, and free of dirty/cache/CPU/control-core/DMA
    /// aliases. Drain all previous accesses before calling. Keep these conditions
    /// true after Drop/forget and through any external recovery until handback.
    pub unsafe fn from_coherent_static(bytes: &'static mut [u8]) -> Result<Self, (MemoryError, &'static mut [u8])> {
        let range = match Range::new(bytes.as_mut_ptr() as usize, bytes.len()) {
            Ok(range) => range,
            Err(error) => return Err((error, bytes)),
        };
        Ok(Self { pointer: NonNull::new(bytes.as_mut_ptr()).unwrap(), range, _owned: PhantomData })
    }

    /// Copy and pad a whole unkeyed message before any hardware publication.
    /// On error, the owned allocation and its contents are returned unchanged.
    pub fn prepare(self, algorithm: Algorithm, message: &[u8]) -> Result<PreparedInput, (FormatError, Self)> {
        if let Err(error) = algorithm.prepared_len(message.len()) {
            return Err((error, self));
        }
        let bytes = unsafe { core::slice::from_raw_parts_mut(self.pointer.as_ptr(), self.range.length as usize) };
        match algorithm.encode(message, bytes) {
            Ok(length) => Ok(PreparedInput { input: self, algorithm, length: length as u32 }),
            Err(error) => Err((error, self)),
        }
    }

    /// Return unused storage, or storage recovered after external quiescence.
    pub fn into_bytes(self) -> &'static mut [u8] {
        compiler_fence(Ordering::Acquire);
        unsafe { core::slice::from_raw_parts_mut(self.pointer.as_ptr(), self.range.length as usize) }
    }
}

/// Prepared one-shot message. No mutable CPU view exists during a transfer.
pub struct PreparedInput {
    pub(super) input: Input,
    pub(super) algorithm: Algorithm,
    pub(super) length: u32,
}
impl PreparedInput {
    /// Return the still-unpublished allocation without touching hardware.
    pub fn into_input(self) -> Input {
        self.input
    }
    /// Encoded message length, including software padding.
    pub const fn encoded_len(&self) -> u32 {
        self.length
    }
    /// Selected unkeyed algorithm.
    pub const fn algorithm(&self) -> Algorithm {
        self.algorithm
    }
}

/// The SDK's two 16-byte input descriptors, occupying complete 32-byte lines.
#[repr(C, align(32))]
pub struct DescriptorStorage {
    entries: [[u32; 4]; 2],
}
impl DescriptorStorage {
    /// Construct empty, unpublished descriptor storage.
    pub const fn new() -> Self {
        Self { entries: [[0; 4]; 2] }
    }
    pub(super) fn fill(&mut self, address: u32, length: u32) {
        // Packed descriptor FIRST/LAST are bits 0/1, although the SDK API
        // enum's FIRST/LAST values are bits 0/2. No API enum is written here.
        self.entries = [[3u32.to_le(), length.to_le(), address.to_le(), 0], [0; 4]];
    }
}
impl Default for DescriptorStorage {
    fn default() -> Self {
        Self::new()
    }
}

/// Static descriptor reservation; no implicit release or drop-time MMIO.
pub struct Table {
    pointer: NonNull<DescriptorStorage>,
    pub(super) range: Range,
    _owned: PhantomData<&'static mut DescriptorStorage>,
}
impl Table {
    /// Reserve unpublished descriptors.
    ///
    /// # Safety
    /// All `Input::from_coherent_static` obligations apply to this allocation,
    /// including complete cache-line ownership and absence of prior fetches.
    pub unsafe fn from_coherent_static(
        storage: &'static mut DescriptorStorage,
    ) -> Result<Self, (MemoryError, &'static mut DescriptorStorage)> {
        let range = match Range::new(storage as *mut _ as usize, core::mem::size_of::<DescriptorStorage>()) {
            Ok(range) => range,
            Err(error) => return Err((error, storage)),
        };
        Ok(Self { pointer: NonNull::from(storage), range, _owned: PhantomData })
    }
    pub(super) fn fill(&mut self, input: &PreparedInput) {
        unsafe { self.pointer.as_mut() }.fill(input.input.range.start, input.length);
    }
    /// Return unused, or externally quiesced descriptor storage.
    pub fn into_storage(self) -> &'static mut DescriptorStorage {
        compiler_fence(Ordering::Acquire);
        unsafe { &mut *self.pointer.as_ptr() }
    }
}

pub(super) fn publish_fence() {
    compiler_fence(Ordering::Release);
    #[cfg(target_arch = "riscv32")]
    unsafe {
        core::arch::asm!("fence iorw, iorw", options(nostack));
    }
}

#[cfg(all(test, not(target_arch = "riscv32")))]
mod tests {
    use super::*;
    #[test]
    fn mode1_ranges_require_whole_lines_and_do_not_wrap() {
        assert!(Range::new(0x20010000, 0x50000).is_ok());
        assert!(Range::new(0x2005ffe0, 32).is_ok());
        for (address, length) in [
            (0x20010000, 0),
            (0x2000ffe0, 32),
            (0x20010001, 32),
            (0x20010000, 31),
            (0x20060000, 32),
            (usize::MAX - 31, 64),
            (0x120010000, 32),
        ] {
            assert_eq!(Range::new(address, length), Err(MemoryError::Range));
        }
    }
    #[test]
    fn packed_descriptor_abi_and_unused_slot() {
        assert_eq!(core::mem::size_of::<DescriptorStorage>(), 32);
        assert_eq!(core::mem::align_of::<DescriptorStorage>(), 32);
        let mut table = DescriptorStorage::new();
        table.fill(0x20050100, 128);
        assert_eq!(table.entries, [[3, 128, 0x20050100, 0], [0; 4]]);
    }
}
