//! WS63 interrupt controller — riscv31 *custom local interrupts* (NOT a PLIC).
//!
//! WS63's application core is a HiSilicon "riscv31" hart with **no PLIC**. There
//! are two interrupt tiers, mirroring fbb_ws63
//! `drivers/chips/ws63/arch/riscv/riscv31/interrupt.c`:
//!
//! * **IRQ 26..=31** — standard machine *local* interrupts, enabled by the
//!   matching bit of the standard `mie` CSR (bit index == IRQ number).
//! * **IRQ >= 32** — HiSilicon custom local interrupts, gated by the custom
//!   CSRs `LOCIEN0..2` (`0xBE0..2`, enable, 32 IRQs/reg from base 32),
//!   `LOCIPD0..` (`0xBE8..`, pending) and cleared by writing the IRQ number to
//!   `LOCIPCLR` (`0xBF0`). Each local IRQ (from base **26**) also has a 4-bit
//!   priority in `LOCIPRI0..15` (`0xBC0..`, 8 IRQs/reg); a global threshold
//!   lives in `PRITHD` (`0xBFE`). An IRQ is delivered only when it is enabled
//!   **and** its priority is strictly greater than the threshold (ties broken
//!   by the lowest IRQ number). Reset default priority is 1, threshold 0.
//!
//! This module owns only the interrupt *controller* (mask / priority /
//! threshold / pending / clear). The trap vector table is the runtime's concern
//! (`hisi-riscv-rt`); the `timer_irq` (mie path) and `gpio_irq` (LOCIEN + LOCIPCLR
//! path) examples drive this API end-to-end and are exercised on ws63-qemu.

// On non-riscv (host) builds the CSR access compiles to no-op stubs, leaving the
// surrounding `unsafe` blocks empty — silence the resulting lints there.
#![cfg_attr(not(target_arch = "riscv32"), allow(unused_unsafe, unused_variables, unused_mut))]

/// Chip-specific interrupt-source enumeration (IRQ numbers) for this SoC.
pub use crate::soc::chip::Interrupt;

// --- model constants (fbb_ws63 vectors.h / riscv_interrupt.h) ---------------
/// First IRQ with a configurable `LOCIPRI` priority (IRQs 0..26 are system vectors).
const SYS_VECTOR_CNT: u16 = 26;
/// IRQs below this use `mie` bits; IRQs at/above it use the `LOCIEN*` custom CSRs.
const LOCAL_IRQ_VECTOR_CNT: u16 = 32;
/// IRQs per `LOCIEN`/`LOCIPD` register.
const LOCIEN_IRQ_NUM: u16 = 32;
/// IRQs per `LOCIPRI` register (each a [`LOCIPRI_IRQ_BITS`]-bit field).
const LOCIPRI_IRQ_NUM: u16 = 8;
const LOCIPRI_IRQ_BITS: u16 = 4;
const LOCIPRI_FIELD_MASK: u32 = 0xF;
/// Number of `LOCIPRI` registers (0..15) → highest priority-configurable IRQ.
const PRIORITY_IRQ_END: u16 = SYS_VECTOR_CNT + 16 * LOCIPRI_IRQ_NUM;
/// Default priority word written to every `LOCIPRI` register (priority 1 per IRQ).
const LOCIPRI_DEFAULT_VAL: u32 = 0x1111_1111;

// --- raw CSR helpers --------------------------------------------------------
// Custom CSR addresses must be assembler immediates, so each register needs its
// own instruction; the `match` arms below mirror the C SDK's per-register switch.
// Pseudo-ops (`csrs`/`csrc`/`csrw`/`csrr`) with a numeric CSR are accepted by the
// assembler (same forms the examples already use).

// Two macro sets: real CSR asm on riscv32, and no-op/zero stubs elsewhere so the
// crate (and its host unit tests) build on x86. The interrupt-controller functions
// are not exercised by host tests; the stubs only need to compile.
#[cfg(target_arch = "riscv32")]
macro_rules! csr_set {
    ($csr:literal, $v:expr) => {
        core::arch::asm!(concat!("csrs ", $csr, ", {0}"), in(reg) $v, options(nomem, nostack))
    };
}
#[cfg(target_arch = "riscv32")]
macro_rules! csr_clear {
    ($csr:literal, $v:expr) => {
        core::arch::asm!(concat!("csrc ", $csr, ", {0}"), in(reg) $v, options(nomem, nostack))
    };
}
#[cfg(target_arch = "riscv32")]
macro_rules! csr_write {
    ($csr:literal, $v:expr) => {
        core::arch::asm!(concat!("csrw ", $csr, ", {0}"), in(reg) $v, options(nomem, nostack))
    };
}
#[cfg(target_arch = "riscv32")]
macro_rules! csr_read {
    ($csr:literal) => {{
        let v: u32;
        core::arch::asm!(concat!("csrr {0}, ", $csr), out(reg) v, options(nomem, nostack));
        v
    }};
}

#[cfg(not(target_arch = "riscv32"))]
macro_rules! csr_set {
    ($csr:literal, $v:expr) => {{
        let _ = $v;
    }};
}
#[cfg(not(target_arch = "riscv32"))]
macro_rules! csr_clear {
    ($csr:literal, $v:expr) => {{
        let _ = $v;
    }};
}
#[cfg(not(target_arch = "riscv32"))]
macro_rules! csr_write {
    ($csr:literal, $v:expr) => {{
        let _ = $v;
    }};
}
#[cfg(not(target_arch = "riscv32"))]
macro_rules! csr_read {
    ($csr:literal) => {{ 0u32 }};
}

/// Set/clear a bit-mask in `LOCIEN{idx}` (idx 0..=2 cover IRQ 32..=127).
#[inline]
unsafe fn locien_write(idx: u16, mask: u32, set: bool) {
    unsafe {
        match (idx, set) {
            (0, true) => csr_set!("0xbe0", mask),
            (0, false) => csr_clear!("0xbe0", mask),
            (1, true) => csr_set!("0xbe1", mask),
            (1, false) => csr_clear!("0xbe1", mask),
            (2, true) => csr_set!("0xbe2", mask),
            (2, false) => csr_clear!("0xbe2", mask),
            _ => {}
        }
    }
}

#[inline]
fn locien_read(idx: u16) -> u32 {
    unsafe {
        match idx {
            0 => csr_read!("0xbe0"),
            1 => csr_read!("0xbe1"),
            2 => csr_read!("0xbe2"),
            _ => 0,
        }
    }
}

#[inline]
fn locipd_read(idx: u16) -> u32 {
    unsafe {
        match idx {
            0 => csr_read!("0xbe8"),
            1 => csr_read!("0xbe9"),
            2 => csr_read!("0xbea"),
            _ => 0,
        }
    }
}

/// Read-modify-write a 4-bit priority field in `LOCIPRI{idx}` (idx 0..=15).
/// A bare `csrs` (as the C SDK uses) cannot lower a field below its current
/// value, so we clear the nibble first — strictly more correct than the SDK.
#[inline]
fn locipri_set_field(idx: u16, shift: u16, value: u32) {
    let clear = LOCIPRI_FIELD_MASK << shift;
    let set = (value & LOCIPRI_FIELD_MASK) << shift;
    unsafe {
        macro_rules! rmw {
            ($csr:literal) => {{
                csr_clear!($csr, clear);
                csr_set!($csr, set);
            }};
        }
        match idx {
            0 => rmw!("0xbc0"),
            1 => rmw!("0xbc1"),
            2 => rmw!("0xbc2"),
            3 => rmw!("0xbc3"),
            4 => rmw!("0xbc4"),
            5 => rmw!("0xbc5"),
            6 => rmw!("0xbc6"),
            7 => rmw!("0xbc7"),
            8 => rmw!("0xbc8"),
            9 => rmw!("0xbc9"),
            10 => rmw!("0xbca"),
            11 => rmw!("0xbcb"),
            12 => rmw!("0xbcc"),
            13 => rmw!("0xbcd"),
            14 => rmw!("0xbce"),
            15 => rmw!("0xbcf"),
            _ => {}
        }
    }
}

#[inline]
fn locipri_read(idx: u16) -> u32 {
    unsafe {
        match idx {
            0 => csr_read!("0xbc0"),
            1 => csr_read!("0xbc1"),
            2 => csr_read!("0xbc2"),
            3 => csr_read!("0xbc3"),
            4 => csr_read!("0xbc4"),
            5 => csr_read!("0xbc5"),
            6 => csr_read!("0xbc6"),
            7 => csr_read!("0xbc7"),
            8 => csr_read!("0xbc8"),
            9 => csr_read!("0xbc9"),
            10 => csr_read!("0xbca"),
            11 => csr_read!("0xbcb"),
            12 => csr_read!("0xbcc"),
            13 => csr_read!("0xbcd"),
            14 => csr_read!("0xbce"),
            15 => csr_read!("0xbcf"),
            _ => 0,
        }
    }
}

// --- public types -----------------------------------------------------------

/// Local-interrupt priority. Valid range **1 (lowest) ..= 7 (highest)**; the
/// reset default is 1. An interrupt is delivered only when its priority is
/// strictly greater than the global threshold (configured by the unstable
/// `set_threshold` API).
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub struct Priority(u8);

impl Priority {
    /// Priority level 1 (lowest deliverable).
    pub const P1: Self = Priority(1);
    /// Priority level 2.
    pub const P2: Self = Priority(2);
    /// Priority level 3.
    pub const P3: Self = Priority(3);
    /// Priority level 4.
    pub const P4: Self = Priority(4);
    /// Priority level 5.
    pub const P5: Self = Priority(5);
    /// Priority level 6.
    pub const P6: Self = Priority(6);
    /// Priority level 7 (highest).
    pub const P7: Self = Priority(7);

    /// Lowest deliverable priority (1).
    pub const LOWEST: Self = Priority(1);
    /// Highest priority (7).
    pub const HIGHEST: Self = Priority(7);

    /// Build a priority level, rejecting values outside `1..=7`.
    pub const fn from_level(level: u8) -> Option<Self> {
        if level >= 1 && level <= 7 { Some(Priority(level)) } else { None }
    }

    const fn from_register(level: u8) -> Self {
        if level < 1 {
            Self::LOWEST
        } else if level > 7 {
            Self::HIGHEST
        } else {
            Priority(level)
        }
    }

    /// The numeric priority level (1..=7).
    pub const fn level(self) -> u8 {
        self.0
    }
}

/// Global local-interrupt priority threshold (`PRITHD`). Valid range is `0..=7`.
/// Only interrupts with priority strictly greater than this threshold are delivered.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub struct Threshold(u8);

impl Threshold {
    /// Threshold level 0: admit all deliverable priorities (`>= 1`).
    pub const T0: Self = Threshold(0);
    /// Threshold level 1.
    pub const T1: Self = Threshold(1);
    /// Threshold level 2.
    pub const T2: Self = Threshold(2);
    /// Threshold level 3.
    pub const T3: Self = Threshold(3);
    /// Threshold level 4.
    pub const T4: Self = Threshold(4);
    /// Threshold level 5.
    pub const T5: Self = Threshold(5);
    /// Threshold level 6.
    pub const T6: Self = Threshold(6);
    /// Threshold level 7: mask every priority representable by [`Priority`].
    pub const T7: Self = Threshold(7);

    /// Build a threshold, rejecting values outside `0..=7`.
    pub const fn from_level(level: u8) -> Option<Self> {
        if level <= 7 { Some(Threshold(level)) } else { None }
    }

    /// The numeric threshold level (0..=7).
    pub const fn level(self) -> u8 {
        self.0
    }
}

#[inline]
fn irq_num(irq: Interrupt) -> u16 {
    irq as u16
}

// --- enable / disable -------------------------------------------------------

/// Enable an interrupt source at the controller.
///
/// IRQ 26..=31 set the matching `mie` bit; IRQ >= 32 set the matching `LOCIEN`
/// bit. The global interrupt enable ([`enable_global`]) must also be set.
///
/// # Safety
/// Enabling an interrupt can cause its handler to run, which may break
/// assumptions held outside a critical section (reentrancy, data races with
/// data the handler touches). The caller must ensure the source's handler and
/// shared state are ready.
pub unsafe fn enable(irq: Interrupt) {
    let n = irq_num(irq);
    if (SYS_VECTOR_CNT..LOCAL_IRQ_VECTOR_CNT).contains(&n) {
        unsafe { csr_set!("mie", 1u32 << n) };
    } else if n >= LOCAL_IRQ_VECTOR_CNT {
        let bit = (n - LOCAL_IRQ_VECTOR_CNT) % LOCIEN_IRQ_NUM;
        let reg = (n - LOCAL_IRQ_VECTOR_CNT) / LOCIEN_IRQ_NUM;
        unsafe { locien_write(reg, 1u32 << bit, true) };
    }
    // Nuclei ECLIC delivers an IRQ only when its `LOCIPRI` priority is strictly
    // greater than the `PRITHD` threshold (both reset to 0, so a just-enabled IRQ is
    // masked). Give the IRQ a non-zero priority here so enabling it is sufficient —
    // callers no longer need a separate `interrupt::init()` for delivery. (`init`
    // remains for setting every priority at once / a custom threshold.)
    if (SYS_VECTOR_CNT..PRIORITY_IRQ_END).contains(&n) {
        let reg = (n - SYS_VECTOR_CNT) / LOCIPRI_IRQ_NUM;
        let shift = ((n - SYS_VECTOR_CNT) % LOCIPRI_IRQ_NUM) * LOCIPRI_IRQ_BITS;
        locipri_set_field(reg, shift, 1);
    }
    // Match the SDK's `osal_irq_enable()` completion barrier. Callers may
    // publish device state immediately before enabling the line, so the
    // controller update must not be observed ahead of those writes.
    #[cfg(target_arch = "riscv32")]
    unsafe {
        core::arch::asm!("fence", options(nostack));
    }
}

/// Disable an interrupt source at the controller and clear any latched pending
/// state (mirrors the C SDK's `int_disable_irq`).
///
/// # Safety
/// See [`enable`]; toggling interrupt masks outside a critical section can race
/// with a handler that is mid-flight.
pub unsafe fn disable(irq: Interrupt) {
    let n = irq_num(irq);
    if (SYS_VECTOR_CNT..LOCAL_IRQ_VECTOR_CNT).contains(&n) {
        unsafe { csr_clear!("mie", 1u32 << n) };
    } else if n >= LOCAL_IRQ_VECTOR_CNT {
        let bit = (n - LOCAL_IRQ_VECTOR_CNT) % LOCIEN_IRQ_NUM;
        let reg = (n - LOCAL_IRQ_VECTOR_CNT) / LOCIEN_IRQ_NUM;
        unsafe { locien_write(reg, 1u32 << bit, false) };
    }
    clear_pending(irq);
}

/// Whether the given interrupt source is currently enabled at the controller.
pub fn is_enabled(irq: Interrupt) -> bool {
    let n = irq_num(irq);
    if (SYS_VECTOR_CNT..LOCAL_IRQ_VECTOR_CNT).contains(&n) {
        let mie: u32 = unsafe { csr_read!("mie") };
        (mie & (1u32 << n)) != 0
    } else if n >= LOCAL_IRQ_VECTOR_CNT {
        let bit = (n - LOCAL_IRQ_VECTOR_CNT) % LOCIEN_IRQ_NUM;
        let reg = (n - LOCAL_IRQ_VECTOR_CNT) / LOCIEN_IRQ_NUM;
        (locien_read(reg) & (1u32 << bit)) != 0
    } else {
        false
    }
}

// --- priority / threshold ---------------------------------------------------

/// Set a local interrupt's 4-bit priority (`LOCIPRI`). Applies to IRQ >= 26;
/// a no-op for the system vectors below that.
#[instability::unstable]
pub fn set_priority(irq: Interrupt, priority: Priority) {
    let n = irq_num(irq);
    if (SYS_VECTOR_CNT..PRIORITY_IRQ_END).contains(&n) {
        let order = (n - SYS_VECTOR_CNT) % LOCIPRI_IRQ_NUM;
        let reg = (n - SYS_VECTOR_CNT) / LOCIPRI_IRQ_NUM;
        locipri_set_field(reg, order * LOCIPRI_IRQ_BITS, priority.0 as u32);
    }
}

/// Read back a local interrupt's configured priority (`LOCIPRI`).
#[instability::unstable]
pub fn priority(irq: Interrupt) -> Priority {
    let n = irq_num(irq);
    if (SYS_VECTOR_CNT..PRIORITY_IRQ_END).contains(&n) {
        let order = (n - SYS_VECTOR_CNT) % LOCIPRI_IRQ_NUM;
        let reg = (n - SYS_VECTOR_CNT) / LOCIPRI_IRQ_NUM;
        let shift = order * LOCIPRI_IRQ_BITS;
        let field = (locipri_read(reg) >> shift) & LOCIPRI_FIELD_MASK;
        Priority::from_register(field as u8)
    } else {
        Priority::LOWEST
    }
}

/// Set the global priority threshold (`PRITHD`, 0..=7). Only interrupts with a
/// priority **strictly greater** than this are delivered; 0 admits all
/// priorities >= 1.
#[instability::unstable]
pub fn set_threshold(threshold: Threshold) {
    unsafe { csr_write!("0xbfe", threshold.level() as u32) };
}

/// Read the current global priority threshold (`PRITHD`).
#[instability::unstable]
pub fn threshold() -> Threshold {
    Threshold((unsafe { csr_read!("0xbfe") } & 0x7) as u8)
}

// --- pending ----------------------------------------------------------------

/// Clear a pending local interrupt by writing its number to `LOCIPCLR`. Safe to
/// call from within the interrupt's own handler (the usual place).
pub fn clear_pending(irq: Interrupt) {
    let n = irq_num(irq) as u32;
    unsafe {
        #[cfg(target_arch = "riscv32")]
        core::arch::asm!("fence", options(nostack));
        csr_write!("0xbf0", n);
        #[cfg(target_arch = "riscv32")]
        core::arch::asm!("fence", options(nostack));
    }
}

/// Whether the given interrupt is currently pending (`LOCIPD` for IRQ >= 32,
/// `mip` for IRQ 26..=31).
pub fn is_pending(irq: Interrupt) -> bool {
    let n = irq_num(irq);
    if (SYS_VECTOR_CNT..LOCAL_IRQ_VECTOR_CNT).contains(&n) {
        let mip: u32 = unsafe { csr_read!("mip") };
        (mip & (1u32 << n)) != 0
    } else if n >= LOCAL_IRQ_VECTOR_CNT {
        let bit = (n - LOCAL_IRQ_VECTOR_CNT) % LOCIEN_IRQ_NUM;
        let reg = (n - LOCAL_IRQ_VECTOR_CNT) / LOCIEN_IRQ_NUM;
        (locipd_read(reg) & (1u32 << bit)) != 0
    } else {
        false
    }
}

// --- global enable / critical section / init --------------------------------

/// Set the default priority of every local interrupt to 1 (`LOCIPRI = 0x1111_1111`
/// for all registers), mirroring the C SDK's `int_setup`. Without this, an IRQ
/// whose priority resets to 0 would never clear the default threshold of 0.
pub fn init() {
    unsafe {
        csr_write!("0xbc0", LOCIPRI_DEFAULT_VAL);
        csr_write!("0xbc1", LOCIPRI_DEFAULT_VAL);
        csr_write!("0xbc2", LOCIPRI_DEFAULT_VAL);
        csr_write!("0xbc3", LOCIPRI_DEFAULT_VAL);
        csr_write!("0xbc4", LOCIPRI_DEFAULT_VAL);
        csr_write!("0xbc5", LOCIPRI_DEFAULT_VAL);
        csr_write!("0xbc6", LOCIPRI_DEFAULT_VAL);
        csr_write!("0xbc7", LOCIPRI_DEFAULT_VAL);
        csr_write!("0xbc8", LOCIPRI_DEFAULT_VAL);
        csr_write!("0xbc9", LOCIPRI_DEFAULT_VAL);
        csr_write!("0xbca", LOCIPRI_DEFAULT_VAL);
        csr_write!("0xbcb", LOCIPRI_DEFAULT_VAL);
        csr_write!("0xbcc", LOCIPRI_DEFAULT_VAL);
        csr_write!("0xbcd", LOCIPRI_DEFAULT_VAL);
        csr_write!("0xbce", LOCIPRI_DEFAULT_VAL);
        csr_write!("0xbcf", LOCIPRI_DEFAULT_VAL);
    }
}

/// Set the global machine-interrupt enable (`mstatus.MIE`).
///
/// # Safety
/// After this, any enabled-and-ready source can preempt. Configure sources and
/// their handlers first.
pub unsafe fn enable_global() {
    #[cfg(target_arch = "riscv32")]
    unsafe {
        core::arch::asm!("csrsi mstatus, 0x8", options(nomem, nostack))
    };
}

/// Clear the global machine-interrupt enable (`mstatus.MIE`).
///
/// # Safety
/// After this call interrupts stay globally masked until code explicitly
/// re-enables them. The caller must keep the masked region short and must not
/// wait on hardware, run transfers, call callbacks, or enter async executors
/// while interrupts are disabled.
pub unsafe fn disable_global() {
    #[cfg(target_arch = "riscv32")]
    unsafe {
        core::arch::asm!("csrci mstatus, 0x8", options(nomem, nostack))
    };
}

/// Run `f` with machine interrupts globally masked, restoring the previous
/// `mstatus.MIE` afterwards (nesting-safe — only re-enables if it was enabled).
///
/// # Safety
/// The closure runs with interrupts globally masked. It must be short,
/// non-blocking, and limited to small Rust metadata updates; it must not touch
/// long-running MMIO paths, transfers, delays, callbacks, or async executors.
#[instability::unstable]
pub unsafe fn free<F, R>(f: F) -> R
where
    F: FnOnce() -> R,
{
    #[cfg(target_arch = "riscv32")]
    let prev: usize;
    #[cfg(target_arch = "riscv32")]
    unsafe {
        core::arch::asm!("csrrci {0}, mstatus, 0x8", out(reg) prev, options(nomem, nostack))
    };
    let r = f();
    #[cfg(target_arch = "riscv32")]
    if prev & 0x8 != 0 {
        unsafe { core::arch::asm!("csrsi mstatus, 0x8", options(nomem, nostack)) };
    }
    r
}

// ── Tests ──────────────────────────────────────────────────────
// Host-only: pure logic only (priority/threshold typing + the IRQ→register/bit/field
// arithmetic). NO CSR access — the csr_* macros are no-op stubs off riscv32, but
// these tests never call the MMIO/CSR-touching driver functions regardless.

#[cfg(all(test, not(target_arch = "riscv32")))]
mod tests {
    use super::*;

    #[test]
    fn priority_from_level_rejects_low() {
        assert_eq!(Priority::from_level(0), None);
    }

    #[test]
    fn priority_from_level_rejects_high() {
        assert_eq!(Priority::from_level(8), None);
        assert_eq!(Priority::from_level(u8::MAX), None);
    }

    #[test]
    fn priority_from_level_passes_through_in_range() {
        // Every in-range level round-trips through from_level()/level() unchanged.
        for l in 1u8..=7 {
            assert_eq!(Priority::from_level(l).unwrap().level(), l);
        }
    }

    #[test]
    fn priority_constants_match_levels() {
        // The named P1..P7 constants carry the matching numeric level.
        assert_eq!(Priority::P1.level(), 1);
        assert_eq!(Priority::P2.level(), 2);
        assert_eq!(Priority::P3.level(), 3);
        assert_eq!(Priority::P4.level(), 4);
        assert_eq!(Priority::P5.level(), 5);
        assert_eq!(Priority::P6.level(), 6);
        assert_eq!(Priority::P7.level(), 7);
        // LOWEST/HIGHEST alias P1/P7.
        assert_eq!(Priority::LOWEST, Priority::P1);
        assert_eq!(Priority::HIGHEST, Priority::P7);
    }

    #[test]
    fn priority_ordering_is_monotonic() {
        // Ord follows numeric level: higher priority compares greater.
        assert!(Priority::P1 < Priority::P2);
        assert!(Priority::LOWEST < Priority::HIGHEST);
        assert!(Priority::P7 > Priority::P3);
    }

    #[test]
    fn irq_num_matches_enum_discriminant() {
        // irq_num is just the repr discriminant — TIMER_INT0 = 26, GPIO_INT0 = 33.
        assert_eq!(irq_num(Interrupt::TIMER_INT0), 26);
        assert_eq!(irq_num(Interrupt::GPIO_INT0), 33);
        assert_eq!(irq_num(Interrupt::GPIO_INT1), 34);
        assert_eq!(irq_num(Interrupt::GPIO_INT2), 35);
    }

    // Re-derive the tier classification used by enable/disable/is_enabled. This
    // mirrors the in-method `contains` checks against the model constants without
    // touching any CSR.
    fn uses_mie(n: u16) -> bool {
        (SYS_VECTOR_CNT..LOCAL_IRQ_VECTOR_CNT).contains(&n)
    }
    fn uses_locien(n: u16) -> bool {
        n >= LOCAL_IRQ_VECTOR_CNT
    }

    #[test]
    fn timer_int0_is_mie_tier() {
        // IRQ 26 sits in the standard mie window [26, 32), not the custom LOCIEN tier.
        let n = irq_num(Interrupt::TIMER_INT0);
        assert!(uses_mie(n));
        assert!(!uses_locien(n));
    }

    #[test]
    fn gpio_ints_are_locien_tier() {
        // GPIO IRQs 33..=35 are >= 32, so they use the custom LOCIEN CSRs.
        for irq in [Interrupt::GPIO_INT0, Interrupt::GPIO_INT1, Interrupt::GPIO_INT2] {
            let n = irq_num(irq);
            assert!(uses_locien(n));
            assert!(!uses_mie(n));
        }
    }

    #[test]
    fn vector_tier_boundary_is_exclusive_at_32() {
        // 31 is the last mie-tier IRQ; 32 is the first LOCIEN-tier IRQ.
        assert!(uses_mie(LOCAL_IRQ_VECTOR_CNT - 1));
        assert!(!uses_locien(LOCAL_IRQ_VECTOR_CNT - 1));
        assert!(!uses_mie(LOCAL_IRQ_VECTOR_CNT));
        assert!(uses_locien(LOCAL_IRQ_VECTOR_CNT));
        // System vectors below 26 are neither tier.
        assert!(!uses_mie(SYS_VECTOR_CNT - 1));
        assert!(!uses_locien(SYS_VECTOR_CNT - 1));
    }

    // Re-derive the LOCIEN register/bit split used by enable/disable/is_pending.
    fn locien_reg_bit(n: u16) -> (u16, u16) {
        let bit = (n - LOCAL_IRQ_VECTOR_CNT) % LOCIEN_IRQ_NUM;
        let reg = (n - LOCAL_IRQ_VECTOR_CNT) / LOCIEN_IRQ_NUM;
        (reg, bit)
    }

    #[test]
    fn locien_split_for_gpio_ints() {
        // GPIO_INT0=33 → LOCIEN0 bit 1; GPIO_INT2=35 → LOCIEN0 bit 3.
        assert_eq!(locien_reg_bit(irq_num(Interrupt::GPIO_INT0)), (0, 1));
        assert_eq!(locien_reg_bit(irq_num(Interrupt::GPIO_INT1)), (0, 2));
        assert_eq!(locien_reg_bit(irq_num(Interrupt::GPIO_INT2)), (0, 3));
    }

    #[test]
    fn locien_split_register_boundaries() {
        // Base of the custom tier maps to reg 0 / bit 0; 32 IRQs later rolls to reg 1.
        assert_eq!(locien_reg_bit(LOCAL_IRQ_VECTOR_CNT), (0, 0));
        assert_eq!(locien_reg_bit(LOCAL_IRQ_VECTOR_CNT + LOCIEN_IRQ_NUM - 1), (0, 31));
        assert_eq!(locien_reg_bit(LOCAL_IRQ_VECTOR_CNT + LOCIEN_IRQ_NUM), (1, 0));
    }

    // Re-derive the LOCIPRI register/field-shift split used by set_priority/priority.
    fn locipri_reg_shift(n: u16) -> (u16, u16) {
        let order = (n - SYS_VECTOR_CNT) % LOCIPRI_IRQ_NUM;
        let reg = (n - SYS_VECTOR_CNT) / LOCIPRI_IRQ_NUM;
        (reg, order * LOCIPRI_IRQ_BITS)
    }

    fn in_priority_range(n: u16) -> bool {
        (SYS_VECTOR_CNT..PRIORITY_IRQ_END).contains(&n)
    }

    #[test]
    fn locipri_split_for_known_irqs() {
        // SYS_VECTOR_CNT (26) is the first priority-configurable IRQ: LOCIPRI0 field 0.
        assert_eq!(locipri_reg_shift(SYS_VECTOR_CNT), (0, 0));
        // TIMER_INT0 == SYS_VECTOR_CNT, so it shares that mapping.
        assert_eq!(locipri_reg_shift(irq_num(Interrupt::TIMER_INT0)), (0, 0));
        // GPIO_INT0=33 → order 7 in LOCIPRI0 → shift 28 (last nibble of reg 0).
        assert_eq!(locipri_reg_shift(irq_num(Interrupt::GPIO_INT0)), (0, 28));
        // GPIO_INT1=34 → first field of LOCIPRI1.
        assert_eq!(locipri_reg_shift(irq_num(Interrupt::GPIO_INT1)), (1, 0));
    }

    #[test]
    fn locipri_shift_stays_within_a_word() {
        // The largest field shift is the last nibble (order 7 → shift 28); the
        // 4-bit field then occupies bits 28..=31 of a 32-bit register.
        let max_shift = (LOCIPRI_IRQ_NUM - 1) * LOCIPRI_IRQ_BITS;
        assert_eq!(max_shift, 28);
        assert!(u32::from(max_shift) + u32::from(LOCIPRI_IRQ_BITS) <= 32);
    }

    #[test]
    fn priority_range_end_matches_register_count() {
        // 16 LOCIPRI registers × 8 IRQs each cover IRQs SYS_VECTOR_CNT..PRIORITY_IRQ_END.
        assert_eq!(PRIORITY_IRQ_END, SYS_VECTOR_CNT + 16 * LOCIPRI_IRQ_NUM);
        // System vectors below 26 are not priority-configurable.
        assert!(!in_priority_range(SYS_VECTOR_CNT - 1));
        assert!(in_priority_range(SYS_VECTOR_CNT));
        // PRIORITY_IRQ_END is exclusive: the last configurable IRQ is one below it.
        assert!(in_priority_range(PRIORITY_IRQ_END - 1));
        assert!(!in_priority_range(PRIORITY_IRQ_END));
    }

    #[test]
    fn locipri_default_val_is_all_priority_one() {
        // LOCIPRI_DEFAULT_VAL must set every 4-bit field in the word to priority 1.
        for order in 0..LOCIPRI_IRQ_NUM {
            let shift = order * LOCIPRI_IRQ_BITS;
            let field = (LOCIPRI_DEFAULT_VAL >> shift) & LOCIPRI_FIELD_MASK;
            assert_eq!(field, 1, "field at order {order} should be priority 1");
        }
        // And no bits live above the eight 4-bit fields (32 bits exactly used).
        assert_eq!(LOCIPRI_DEFAULT_VAL, 0x1111_1111);
    }

    #[test]
    fn threshold_from_level_accepts_only_three_bit_values() {
        for level in 0u8..=7 {
            assert_eq!(Threshold::from_level(level).unwrap().level(), level);
        }
        assert_eq!(Threshold::from_level(8), None);
        assert_eq!(Threshold::from_level(u8::MAX), None);
    }
}

// ── Property-based fuzz tests ──────────────────────────────────

#[cfg(all(test, not(target_arch = "riscv32")))]
mod proptests {
    use super::*;
    use proptest::prelude::*;

    proptest! {
        /// Fuzz: Priority::from_level accepts exactly 1..=7.
        #[test]
        fn priority_from_level_acceptance_matches_range(level in any::<u8>()) {
            let p = Priority::from_level(level);
            prop_assert_eq!(p.is_some(), (1..=7).contains(&level));
        }

        /// Fuzz: in-range priority levels round-trip exactly.
        #[test]
        fn priority_from_level_roundtrips_when_valid(level in 1u8..=7) {
            prop_assert_eq!(Priority::from_level(level).unwrap().level(), level);
        }

        /// Fuzz: Threshold::from_level accepts exactly 0..=7.
        #[test]
        fn threshold_from_level_acceptance_matches_range(level in any::<u8>()) {
            let threshold = Threshold::from_level(level);
            prop_assert_eq!(threshold.is_some(), level <= 7);
            if let Some(threshold) = threshold {
                prop_assert_eq!(threshold.level(), level);
            }
        }

        /// Fuzz: for any custom-tier IRQ, the LOCIEN bit index is always < 32 and the
        /// (reg, bit) split reconstructs the original IRQ number.
        #[test]
        fn locien_split_reconstructs(n in LOCAL_IRQ_VECTOR_CNT..=127u16) {
            let bit = (n - LOCAL_IRQ_VECTOR_CNT) % LOCIEN_IRQ_NUM;
            let reg = (n - LOCAL_IRQ_VECTOR_CNT) / LOCIEN_IRQ_NUM;
            prop_assert!(bit < LOCIEN_IRQ_NUM);
            prop_assert_eq!(reg * LOCIEN_IRQ_NUM + bit + LOCAL_IRQ_VECTOR_CNT, n);
        }

        /// Fuzz: for any priority-configurable IRQ, the LOCIPRI field shift never lets a
        /// 4-bit field spill past the 32-bit register boundary.
        #[test]
        fn locipri_field_fits_in_word(n in SYS_VECTOR_CNT..PRIORITY_IRQ_END) {
            let order = (n - SYS_VECTOR_CNT) % LOCIPRI_IRQ_NUM;
            let shift = order * LOCIPRI_IRQ_BITS;
            prop_assert!(u32::from(shift) + u32::from(LOCIPRI_IRQ_BITS) <= 32);
        }

    }
}
