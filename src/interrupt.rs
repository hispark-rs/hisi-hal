//! Interrupt management for WS63.
pub use crate::soc::ws63::Interrupt;

#[inline]
pub fn enable(_interrupt: Interrupt) {
    unsafe { riscv::interrupt::enable() };
}

#[inline]
pub fn disable(_interrupt: Interrupt) {}

#[inline]
pub fn free<F, R>(f: F) -> R
where F: FnOnce() -> R
{
    riscv::interrupt::free(f)
}
