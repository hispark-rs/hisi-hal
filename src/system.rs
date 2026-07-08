//! System control — clock enables, resets, and power management.

use crate::peripherals::{CldoCrg, GlbCtlM, SysCtl0};

/// System control handle.
///
/// Holds the SYS_CTL0, GLB_CTL_M, and CLDO_CRG peripherals for clock and
/// reset configuration.
pub struct System<'d> {
    /// SYS_CTL0 peripheral, controlling core system clock and reset settings.
    pub sys_ctl0: SysCtl0<'d>,
    /// GLB_CTL_M peripheral, controlling global clock/reset and chip reset.
    pub glb_ctl_m: GlbCtlM<'d>,
    /// CLDO_CRG peripheral (clock and reset generator) for peripheral gates.
    pub cldo_crg: CldoCrg<'d>,
}

impl<'d> System<'d> {
    /// Create a `System` handle from the SYS_CTL0, GLB_CTL_M, and CLDO_CRG peripherals.
    pub fn new(sys_ctl0: SysCtl0<'d>, glb_ctl_m: GlbCtlM<'d>, cldo_crg: CldoCrg<'d>) -> Self {
        Self { sys_ctl0, glb_ctl_m, cldo_crg }
    }
}

/// Reason for the last reset.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ResetReason {
    /// Power-on reset (cold boot).
    PowerOn,
    /// External reset pin.
    ExternalPin,
    /// Watchdog timer reset.
    Watchdog,
    /// Software reset.
    Software,
    /// Brown-out reset.
    BrownOut,
    /// Unknown reset reason.
    Unknown,
}

// Reset registers, from fbb_ws63 drivers/chips/ws63/porting/reboot/reboot_porting.c:
//   chip-reset trigger:  GLB_CTL_M (0x4000_2000) + 0x110, set bit 2 (HAL_CHIP_RESET_REG)
//   reset-reason record: GLB_CTL   (0x4000_0000) + 0xA0    (SYS_RST_RECORD_0)
//   reason-clear:        GLB_CTL   (0x4000_0000) + 0xA4    (SYS_DIAG_CLR_1)
// History bits in SYS_RST_RECORD_0.
const SYS_WDT_RST_HIS: u32 = 0x1;
const SYS_SOFT_RST_HIS: u32 = 0x2;
const POR_RST_FILTER_HIS: u32 = 0x8;

impl System<'_> {
    /// Read (and clear) the last reset reason from `SYS_RST_RECORD_0`.
    ///
    /// Decodes the WS63 reset-history record (`reboot_port_get_rst_reason`):
    /// watchdog takes precedence over software over power-on. The matched bit is
    /// cleared via `SYS_DIAG_CLR_1` so the next boot reports its own cause.
    /// Reasons this SoC's record does not distinguish (`ExternalPin`, `BrownOut`)
    /// are never returned here. An empty record reads back as [`ResetReason::Unknown`].
    pub fn reset_reason(&self) -> ResetReason {
        let r = unsafe { &*SysCtl0::ptr() };
        let record = r.sys_rst_record_0().read();
        let val = record.bits();
        let (reason, clr) = if val & SYS_WDT_RST_HIS != 0 {
            (ResetReason::Watchdog, SYS_WDT_RST_HIS)
        } else if val & SYS_SOFT_RST_HIS != 0 {
            (ResetReason::Software, SYS_SOFT_RST_HIS)
        } else if val & POR_RST_FILTER_HIS != 0 {
            (ResetReason::PowerOn, POR_RST_FILTER_HIS)
        } else {
            (ResetReason::Unknown, 0)
        };
        if clr != 0 {
            r.sys_diag_clr_1().write(|w| unsafe { w.sys_diag_clr().bits(clr) });
        }
        reason
    }

    /// Trigger a full software reset of the chip and never return.
    ///
    /// Sets the chip-reset enable bit (bit 2) of `GLB_CTL_M + 0x110`, the same
    /// register `reboot_port_reboot_chip` uses. The CPU is reset before the
    /// following spin loop completes.
    #[instability::unstable]
    pub fn software_reset(&self) -> ! {
        let r = unsafe { &*GlbCtlM::ptr() };
        r.chip_reset().modify(|_, w| w.chip_reset_en().set_bit());
        loop {
            core::hint::spin_loop();
        }
    }

    /// Trigger a software reset and never return.
    ///
    /// WS63's porting layer exposes only a whole-chip reset, so this is an alias
    /// of [`software_reset`](Self::software_reset).
    #[instability::unstable]
    pub fn software_reset_cpu(&self) -> ! {
        self.software_reset()
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
        Self { sysclk: crate::soc::chip::SYSTEM_CLOCK_HZ, pclk: crate::soc::chip::SYSTEM_CLOCK_HZ }
    }
}
