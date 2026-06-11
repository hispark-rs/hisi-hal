//! BS2X USB 2.0 OTG — Synopsys DesignWare DWC OTG controller.
//!
//! BS2X-only (`chip-bs21`); no WS63 analogue. A full USB device/host stack
//! (enumeration, endpoints, descriptors, transfers) is a large separate effort —
//! this driver covers controller presence: reading the Synopsys core-ID register
//! (`gsnpsid`), whose top half is the ASCII "OT" signature (0x4F54). bs2x-pac has a
//! register-level `usb` block (DWC OTG global + device CSRs); `Usb` @ 0x5800_0000.

use crate::peripherals::Usb as UsbPeriph;
use core::marker::PhantomData;

/// The Synopsys signature in the top half of GSNPSID ("OT").
pub const SNPS_SIGNATURE: u16 = 0x4F54;

pub struct Usb<'d> {
    _u: PhantomData<UsbPeriph<'d>>,
}

impl<'d> Usb<'d> {
    fn regs(&self) -> &'static crate::soc::pac::usb::RegisterBlock {
        // SAFETY: static physical MMIO base (0x5800_0000) from bs2x-pac.
        unsafe { &*UsbPeriph::ptr() }
    }

    pub fn new(_u: UsbPeriph<'d>) -> Self {
        Self { _u: PhantomData }
    }

    /// Read the DWC OTG core-ID register (`gsnpsid`). The top 16 bits are the
    /// Synopsys "OT" signature ([`SNPS_SIGNATURE`]); the low bits encode the
    /// release. A correct read proves the USB controller is present + accessible.
    pub fn core_id(&self) -> u32 {
        self.regs().gsnpsid().read().bits()
    }

    /// True if the core-ID carries the Synopsys DWC OTG signature.
    pub fn is_present(&self) -> bool {
        (self.core_id() >> 16) as u16 == SNPS_SIGNATURE
    }
}
