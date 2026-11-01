use super::{BlockLength, PrepareError, config::Configuration};
use crate::km::Parity;
use core::{
    marker::PhantomData,
    ptr::NonNull,
    sync::atomic::{Ordering, compiler_fence},
};

/// Allocation is not a nonempty whole set of 32-byte lines in mode-1 DTCM.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct MemoryError;
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) struct Range {
    pub start: u32,
    pub length: u32,
}
impl Range {
    pub fn new(start: usize, length: usize) -> Result<Self, MemoryError> {
        let end = start.checked_add(length).ok_or(MemoryError)?;
        if length == 0 || (start | length) & 31 != 0 || start < 0x2001_0000 || end > 0x2006_0000 {
            return Err(MemoryError);
        }
        Ok(Self { start: start as u32, length: length as u32 })
    }
    fn overlaps(self, other: Self) -> bool {
        self.start < other.start + other.length && other.start < self.start + self.length
    }
}
pub(super) fn validate(input: Range, output: Range, table: Range, length: BlockLength) -> Result<(), PrepareError> {
    if length.bytes() > input.length || length.bytes() > output.length {
        return Err(PrepareError::Capacity);
    }
    if input.overlaps(output) || input.overlaps(table) || output.overlaps(table) {
        return Err(PrepareError::Overlap);
    }
    Ok(())
}

/// Static unpublished or externally quiesced DMA buffer. During a transfer no
/// CPU view is available. Drop/forget does not return allocation ownership.
/// ```compile_fail
/// use hisi_hal::symc::Buffer;
/// fn stack() { let mut bytes = [0u8; 32]; unsafe { Buffer::from_coherent_static(&mut bytes) }; }
/// ```
pub struct Buffer {
    pointer: NonNull<u8>,
    pub(super) range: Range,
    _owned: PhantomData<&'static mut [u8]>,
}
impl Buffer {
    /// Reserve static application memory without touching contents or MMIO.
    /// This range check does not establish actual board ownership/coherence.
    ///
    /// # Safety
    /// Reserve all cache lines exclusively, drain all previous bus/CPU users,
    /// and ensure SPACC-accessible, identity-addressed coherent or noncacheable
    /// RAM with no dirty aliases. Preserve this after Drop/forget until physical
    /// handback. No cache maintenance or memory-attribute setup occurs here.
    pub unsafe fn from_coherent_static(bytes: &'static mut [u8]) -> Result<Self, (MemoryError, &'static mut [u8])> {
        let range = match Range::new(bytes.as_mut_ptr() as usize, bytes.len()) {
            Ok(r) => r,
            Err(e) => return Err((e, bytes)),
        };
        Ok(Self { pointer: NonNull::new(bytes.as_mut_ptr()).unwrap(), range, _owned: PhantomData })
    }
    /// CPU access only while this buffer is unpublished or externally quiesced.
    pub fn bytes_mut(&mut self) -> &mut [u8] {
        // SAFETY: sole static reservation, inaccessible while held by Transfer.
        unsafe { core::slice::from_raw_parts_mut(self.pointer.as_ptr(), self.range.length as usize) }
    }
    /// Consume an unpublished/quiesced buffer and return the original storage.
    pub fn into_bytes(self) -> &'static mut [u8] {
        compiler_fence(Ordering::Acquire);
        // SAFETY: only constructors or unsafe transfer handback yield Buffer.
        unsafe { core::slice::from_raw_parts_mut(self.pointer.as_ptr(), self.range.length as usize) }
    }
}
/// SDK ABI: two 32-byte inputs followed by two 16-byte outputs. All descriptors
/// are prepared before publishing either ring. The second entries stay zero.
#[repr(C, align(32))]
pub struct DescriptorStorage {
    input: [[u32; 8]; 2],
    output: [[u32; 4]; 2],
}
impl DescriptorStorage {
    /// Empty, unpublished storage.
    pub const fn new() -> Self {
        Self { input: [[0; 8]; 2], output: [[0; 4]; 2] }
    }
    pub(super) fn fill(&mut self, input: u32, output: u32, length: BlockLength, config: Configuration) {
        *self = Self::new();
        let flags = 3 | if config.parity == Parity::Odd { 1 << 9 } else { 0 };
        self.input[0][0] = u32::to_le(flags);
        self.input[0][1] = length.bytes().to_le();
        self.input[0][2] = input.to_le();
        for (i, bytes) in config.mode.iv().as_chunks::<4>().0.iter().enumerate() {
            self.input[0][4 + i] = u32::from_le_bytes(*bytes).to_le();
        }
        self.output[0] = [0, length.bytes().to_le(), output.to_le(), 0];
    }
}
impl Default for DescriptorStorage {
    fn default() -> Self {
        Self::new()
    }
}
/// Complete static descriptor reservation, retained on all transfer outcomes.
pub struct Table {
    pointer: NonNull<DescriptorStorage>,
    pub(super) range: Range,
    _owned: PhantomData<&'static mut DescriptorStorage>,
}
impl Table {
    /// Reserve descriptors without MMIO. All Buffer allocation/coherence and
    /// lifetime obligations also apply to this entire 96-byte reservation.
    ///
    /// # Safety
    /// See `Buffer::from_coherent_static`; no previous fetch may remain active.
    pub unsafe fn from_coherent_static(
        storage: &'static mut DescriptorStorage,
    ) -> Result<Self, (MemoryError, &'static mut DescriptorStorage)> {
        let range = match Range::new(storage as *mut _ as usize, core::mem::size_of::<DescriptorStorage>()) {
            Ok(r) => r,
            Err(e) => return Err((e, storage)),
        };
        Ok(Self { pointer: NonNull::from(storage), range, _owned: PhantomData })
    }
    pub(super) fn fill(&mut self, input: u32, output: u32, length: BlockLength, config: Configuration) {
        // SAFETY: called only before any publication, under exclusive reservation.
        unsafe { self.pointer.as_mut() }.fill(input, output, length, config);
    }
    /// Return unpublished, or externally quiesced, descriptor storage.
    pub fn into_storage(self) -> &'static mut DescriptorStorage {
        compiler_fence(Ordering::Acquire);
        // SAFETY: Transfer has no safe table handback; prior fetches were drained.
        unsafe { &mut *self.pointer.as_ptr() }
    }
}
pub(super) fn publish_fence() {
    compiler_fence(Ordering::Release);
    #[cfg(target_arch = "riscv32")]
    // SAFETY: ordering instruction only; not a cache-coherence or DMA-drain proof.
    unsafe {
        core::arch::asm!("fence iorw, iorw", options(nostack));
    }
}

#[cfg(all(test, not(target_arch = "riscv32")))]
#[path = "memory_tests.rs"]
mod tests;
