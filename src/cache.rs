//! WS63 D-cache maintenance via the HiSilicon custom RISC-V CSRs.
//!
//! The WS63 core has a **4 KB, 32-byte-line, non-coherent** data cache (enabled
//! at boot). A DMA / bus master sees physical RAM directly, so software must keep
//! the cache and RAM in sync around a transfer:
//!
//! * **Before** a master *reads* CPU-written data — [`clean_range`] (write the
//!   dirty cache lines back to RAM) so the master sees the new bytes.
//! * **After** a master *writes* memory, before the CPU reads it —
//!   [`invalidate_range`] (drop the stale cache lines) so the CPU re-fetches from
//!   RAM instead of returning the old cached value.
//!
//! Cache control uses two custom CSRs (mirroring the vendor LiteOS
//! `arch/riscv/include/arch/cache.h` + `osal_dcache_*`):
//! `DCINCVA` (`0x7C5`, the target address) and `DCMAINT` (`0x7C3`, the command,
//! whose bits are `VA = 0x1` "by virtual address", `DCIV = 0x4` "invalidate",
//! `DCC = 0x8` "clean"). The vendor `hal_dma_v151_enable()` flushes the whole
//! D-cache before kicking a transfer; these by-range ops are the finer-grained
//! equivalent.
//!
//! All ops are no-ops on a non-riscv (host) build so the crate's host unit tests
//! still compile.

/// D-cache line size, in bytes (WS63: 32).
pub const CACHE_LINE: usize = 32;

#[cfg(target_arch = "riscv32")]
#[inline]
unsafe fn maintain(addr: usize, len: usize, cmd: u32) {
    if len == 0 {
        return;
    }
    // Round the range out to whole cache lines (maintenance operates per line).
    let start = addr & !(CACHE_LINE - 1);
    let end = (addr + len + CACHE_LINE - 1) & !(CACHE_LINE - 1);
    let mut line = start;
    while line < end {
        // DCINCVA (0x7C5) = the line address; DCMAINT (0x7C3) = the command.
        // SAFETY: M-mode custom cache-maintenance CSR writes; no memory operands.
        unsafe {
            core::arch::asm!(
                "csrw 0x7c5, {a}",
                "csrw 0x7c3, {c}",
                a = in(reg) line,
                c = in(reg) cmd,
                options(nostack),
            );
        }
        line += CACHE_LINE;
    }
    // Order the maintenance before any following load/store of the range.
    // SAFETY: a plain memory fence, no operands.
    unsafe { core::arch::asm!("fence", options(nostack)) };
}

#[cfg(not(target_arch = "riscv32"))]
#[inline]
unsafe fn maintain(_addr: usize, _len: usize, _cmd: u32) {}

/// Clean (write back) `[addr, addr + len)` from the D-cache to memory.
///
/// Call **before** a DMA / bus master reads memory the CPU has written, so the
/// master sees the current data rather than stale RAM.
///
/// # Safety
/// `addr`/`len` must describe a real mapped range; the op affects whole 32-byte
/// cache lines covering the range.
#[inline]
pub unsafe fn clean_range(addr: usize, len: usize) {
    // DCC | VA
    unsafe { maintain(addr, len, 0x9) }
}

/// Invalidate `[addr, addr + len)` in the D-cache (drop the cached copy).
///
/// Call **after** a DMA / bus master writes memory, before the CPU reads it, so
/// the CPU re-fetches from RAM. Invalidation discards whole cache lines, so the
/// range should be 32-byte ([`CACHE_LINE`]) aligned — otherwise a partial line
/// shared with neighbouring *dirty* data would lose that neighbour's writes. Use
/// [`flush_range`] if the range may hold dirty data you need to keep.
///
/// # Safety
/// As [`clean_range`]; additionally the caller must accept that any cached writes
/// to the covered lines are discarded.
#[inline]
pub unsafe fn invalidate_range(addr: usize, len: usize) {
    // DCIV | VA
    unsafe { maintain(addr, len, 0x5) }
}

/// Clean **and** invalidate `[addr, addr + len)` (a full flush) by address.
///
/// # Safety
/// As [`clean_range`].
#[inline]
pub unsafe fn flush_range(addr: usize, len: usize) {
    // DCC | DCIV | VA
    unsafe { maintain(addr, len, 0xd) }
}

/// Clean and invalidate the entire D-cache.
///
/// This matches the vendor LiteOS `ArchDCacheFlush()` operation. Prefer the
/// range operations when ownership is limited to a known buffer; this function
/// exists for vendor ABI shims that explicitly request a whole-cache flush.
///
/// # Safety
///
/// The caller must ensure that globally flushing the D-cache cannot race with
/// another execution context that owns cacheable memory.
#[inline]
pub unsafe fn flush_all() {
    #[cfg(target_arch = "riscv32")]
    unsafe {
        // DCC | DCIV, without VA: operate on the complete D-cache.
        core::arch::asm!(
            "csrw 0x7c3, {command}",
            "fence",
            command = in(reg) 0xc,
            options(nostack),
        );
    }
}
