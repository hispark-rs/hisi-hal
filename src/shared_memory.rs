//! WS63 configurable shared-RAM ownership.
//!
//! The default Wi-Fi layout mirrors `fbb_ws63`'s `dyn_mem_cfg`: packet RAM for
//! RAM5-RAM9/RAM12, DTCM for RAM10, and ITCM for RAM11.

use crate::peripherals::{BtEmCtl, ShareMemCtl};

/// Exclusive controller for WS63 shared-RAM bank ownership.
pub struct SharedMemory<'d> {
    _share_mem: ShareMemCtl<'d>,
    _bt_em: BtEmCtl<'d>,
}

impl<'d> SharedMemory<'d> {
    /// Create a shared-memory controller from its singleton tokens.
    pub fn new(share_mem: ShareMemCtl<'d>, bt_em: BtEmCtl<'d>) -> Self {
        Self {
            _share_mem: share_mem,
            _bt_em: bt_em,
        }
    }

    /// Apply the official default Wi-Fi shared-RAM layout.
    ///
    /// This routine runs from flash and briefly gates all configurable RAM-bank
    /// clocks while changing ownership. Call it before starting RF workers.
    pub fn configure_default_wifi(&mut self) {
        let share = unsafe { &*ShareMemCtl::ptr() };
        let bt_em = unsafe { &*BtEmCtl::ptr() };

        share.cfg_ram_cken().modify(|_, w| {
            // SAFETY: 0 fits the 14-bit shared-RAM clock field.
            unsafe { w.share_ram_cken().bits(0) }
        });
        bt_em.em_gt_mode().modify(|_, w| w.enable().set_bit());
        fence_io();

        share.cfg_ram_sel().modify(|_, w| {
            // SAFETY: every value is one of the 2-bit hardware selections:
            // packet=0, ITCM=2, DTCM=3.
            unsafe {
                w.ram12_sel()
                    .clear_bit()
                    .ram11_sel()
                    .bits(2)
                    .ram10_sel()
                    .bits(3)
                    .ram9_sel()
                    .bits(0)
                    .ram8_sel()
                    .bits(0)
                    .ram7_sel()
                    .bits(0)
                    .ram6_sel()
                    .bits(0)
                    .ram5_sel()
                    .bits(0)
            }
        });

        share.cfg_ram_cken().modify(|_, w| {
            // SAFETY: 0x3fff enables every bit in the 14-bit field.
            unsafe { w.share_ram_cken().bits(0x3fff) }
        });
        bt_em.em_gt_mode().modify(|_, w| w.enable().clear_bit());
        fence_io();
    }
}

#[inline(always)]
fn fence_io() {
    #[cfg(target_arch = "riscv32")]
    // SAFETY: orders the preceding/following device register accesses.
    unsafe {
        core::arch::asm!("fence iorw, iorw", options(nostack));
    }
}
