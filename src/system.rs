//! System control — clock enables, resets, and power management.

use crate::peripherals::{CldoCrg, GlbCtlM, SysCtl0};

/// System control handle.
///
/// Holds the SYS_CTL0, GLB_CTL_M, and CLDO_CRG peripherals for clock and
/// reset configuration.
pub struct System<'d> {
    pub sys_ctl0: SysCtl0<'d>,
    pub glb_ctl_m: GlbCtlM<'d>,
    pub cldo_crg: CldoCrg<'d>,
}

impl<'d> System<'d> {
    pub fn new(sys_ctl0: SysCtl0<'d>, glb_ctl_m: GlbCtlM<'d>, cldo_crg: CldoCrg<'d>) -> Self {
        Self { sys_ctl0, glb_ctl_m, cldo_crg }
    }
}

/// Clocks after configuration.
#[derive(Debug, Clone, Copy)]
pub struct Clocks {
    /// System (CPU) clock frequency in Hz.
    pub sysclk: u32,
    /// Peripheral bus clock frequency in Hz.
    pub pclk: u32,
}

impl Default for Clocks {
    fn default() -> Self {
        Self {
            sysclk: crate::soc::ws63::SYSTEM_CLOCK_HZ,
            pclk: crate::soc::ws63::SYSTEM_CLOCK_HZ,
        }
    }
}
