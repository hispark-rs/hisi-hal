//! WS63 CPU software interrupt 0.
//!
//! The source is `SYS_CTL1.SOFT_INT0`, delivered as custom local IRQ 36. It is
//! not the standard RISC-V machine-software interrupt (`mcause = 3`).

use crate::interrupt::{self, Interrupt};
use crate::peripherals::SysCtl1;

/// Exclusive owner of the WS63 software interrupt 0 source.
pub struct SoftwareInterrupt0<'d> {
    _sys_ctl1: SysCtl1<'d>,
}

impl<'d> SoftwareInterrupt0<'d> {
    /// Claims software interrupt 0, clearing stale state before enabling it.
    ///
    /// This enables both the `SYS_CTL1` source and custom local IRQ 36. Global
    /// machine interrupts remain under application/runtime control.
    pub fn new(sys_ctl1: SysCtl1<'d>) -> Self {
        Self::clear_interrupt();
        let regs = unsafe { &*SysCtl1::ptr() };
        regs.soft_int_en().write(|w| w.soft_int0_en().set_bit());
        unsafe { interrupt::enable(Interrupt::SOFT_INT0) };
        Self { _sys_ctl1: sys_ctl1 }
    }

    /// Pends software interrupt 0.
    pub fn pend(&self) {
        let regs = unsafe { &*SysCtl1::ptr() };
        regs.soft_int_set().write(|w| w.soft_int0_set().set_bit());
    }

    /// Returns whether software interrupt 0 is asserted at the source.
    pub fn is_pending(&self) -> bool {
        let regs = unsafe { &*SysCtl1::ptr() };
        regs.soft_int_sts().read().soft_int0_sts().bit_is_set()
    }

    /// Clears the source and the custom local interrupt-controller latch.
    ///
    /// This is an associated function so a named interrupt handler can call it
    /// while the owning token remains in thread context.
    pub fn clear_interrupt() {
        let regs = unsafe { &*SysCtl1::ptr() };
        regs.soft_int_clr().write(|w| w.soft_int0_clr().set_bit());
        interrupt::clear_pending(Interrupt::SOFT_INT0);
    }
}

impl Drop for SoftwareInterrupt0<'_> {
    fn drop(&mut self) {
        unsafe { interrupt::disable(Interrupt::SOFT_INT0) };
        let regs = unsafe { &*SysCtl1::ptr() };
        regs.soft_int_en().write(|w| w);
        Self::clear_interrupt();
    }
}
