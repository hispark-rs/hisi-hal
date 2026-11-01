//! BS2X USB 2.0 OTG — Synopsys DesignWare DWC2 OTG controller, device mode.
//!
//! BS2X-only (`chip-bs21`); no WS63 analogue. This brings the DWC2 OTG **device
//! controller** up to the enumerated state: core soft-reset → force device mode →
//! global config → device config → soft-connect → wait for the host USB-reset and
//! enumeration-done, then read the negotiated speed. Higher layers (descriptors,
//! endpoint transfers, the USB class stack) sit on top and are out of scope here.
//!
//! Sequence + bits from the fbb_bs2x DWC OTG PCD (`dwc_otgreg.h` / `dwc_otg_pcd.h`,
//! the canonical Synopsys DWC2 device init). All registers are in bs2x-pac's `usb`
//! block; `Usb` @ 0x5800_0000. The PHY/PMU power-on (a CRG/PHY concern) is assumed
//! done by the boot/clock layer.

use crate::peripherals::Usb as UsbPeriph;
use core::marker::PhantomData;

/// The Synopsys signature in the top half of GSNPSID ("OT").
pub const SNPS_SIGNATURE: u16 = 0x4F54;

const POLL_LIMIT: u32 = 1_000_000;

/// True if a raw `gsnpsid` core-ID value carries the Synopsys DWC OTG signature
/// in its top 16 bits.
const fn id_has_signature(core_id: u32) -> bool {
    (core_id >> 16) as u16 == SNPS_SIGNATURE
}

/// Decode the 2-bit `DSTS.ENUMSPD` field into the negotiated [`Speed`].
const fn speed_from_enumspd(enumspd: u8) -> Speed {
    match enumspd {
        0 => Speed::High,
        1 => Speed::FullFs,
        2 => Speed::Low,
        _ => Speed::Full,
    }
}

/// Errors from bringing up the DWC2 OTG device controller.
#[derive(Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum UsbError {
    /// A status bit (reset-done / USB reset / enumeration done) never asserted.
    Timeout,
    /// The DWC OTG core-ID signature was wrong (controller not present).
    NotPresent,
}

/// The USB speed the host enumerated the device at (`DSTS.ENUMSPD`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Speed {
    /// High speed (`ENUMSPD` = 0).
    High,
    /// Full speed via the high-speed PHY (`ENUMSPD` = 3).
    Full,
    /// Low speed (`ENUMSPD` = 2).
    Low,
    /// Full speed via a full-speed PHY (`ENUMSPD` = 1).
    FullFs,
}

/// Driver for the BS2X DWC2 OTG controller in USB 2.0 device mode.
pub struct Usb<'d> {
    _u: PhantomData<UsbPeriph<'d>>,
}

impl<'d> Usb<'d> {
    fn regs(&self) -> &'static crate::soc::pac::usb::RegisterBlock {
        // SAFETY: static physical MMIO base (0x5800_0000) from bs2x-pac.
        unsafe { &*UsbPeriph::ptr() }
    }

    /// Create the driver from the `Usb` peripheral token.
    pub fn new(_u: UsbPeriph<'d>) -> Self {
        Self { _u: PhantomData }
    }

    /// Read the DWC OTG core-ID register (`gsnpsid`); the top 16 bits are the
    /// Synopsys "OT" signature.
    pub fn core_id(&self) -> u32 {
        self.regs().gsnpsid().read().bits()
    }

    /// True if the core-ID carries the Synopsys DWC OTG signature.
    pub fn is_present(&self) -> bool {
        id_has_signature(self.core_id())
    }

    /// Bring the DWC2 OTG device controller up and wait until the host has reset
    /// and enumerated it; returns the negotiated [`Speed`].
    ///
    /// Steps (DWC2 device init): core soft reset → force device mode → GAHBCFG +
    /// interrupt unmask → DCFG → enable global interrupts → soft-connect (clear
    /// `DCTL.SFTDISCON`) → wait `GINTSTS.USBRST` → wait `GINTSTS.ENUMDONE` → read
    /// `DSTS.ENUMSPD`.
    pub fn device_enumerate(&mut self) -> Result<Speed, UsbError> {
        let r = self.regs();
        if !self.is_present() {
            return Err(UsbError::NotPresent);
        }

        // A. Core soft reset: wait AHB idle, assert CSFTRST, wait it self-clear.
        self.poll(|| r.grstctl().read().ahbidle().bit_is_set())?;
        r.grstctl().write(|w| w.csftrst().set_bit());
        self.poll(|| r.grstctl().read().csftrst().bit_is_clear())?;
        self.poll(|| r.grstctl().read().ahbidle().bit_is_set())?;

        // B. Global config: force device mode, clear pending, unmask reset + enum.
        r.gusbcfg().modify(|_, w| w.forcedevmode().set_bit());
        unsafe {
            r.gintsts().write(|w| w.bits(0xFFFF_FFFF)); // clear all (W1C)
        }
        r.gintmsk().modify(|_, w| {
            w.usbrstmsk().set_bit();
            w.enumdonemsk().set_bit()
        });

        // C. Device config: high speed, address 0.
        unsafe {
            r.dcfg().modify(|_, w| {
                w.devspd().bits(0); // 0 = high speed
                w.devaddr().bits(0)
            });
        }

        // D. Enable global interrupts.
        r.gahbcfg().modify(|_, w| w.glblintrmsk().set_bit());

        // E. Soft-connect (attach D+ pull-up): clear SFTDISCON.
        r.dctl().modify(|_, w| w.sftdiscon().clear_bit());

        // F. Wait for the host reset + enumeration, clearing each (W1C).
        self.poll(|| r.gintsts().read().usbrst().bit_is_set())?;
        r.gintsts().write(|w| w.usbrst().set_bit());
        self.poll(|| r.gintsts().read().enumdone().bit_is_set())?;
        r.gintsts().write(|w| w.enumdone().set_bit());

        // G. Read the negotiated speed.
        Ok(speed_from_enumspd(r.dsts().read().enumspd().bits()))
    }

    fn poll(&self, mut ready: impl FnMut() -> bool) -> Result<(), UsbError> {
        for _ in 0..POLL_LIMIT {
            if ready() {
                return Ok(());
            }
            core::hint::spin_loop();
        }
        Err(UsbError::Timeout)
    }
}

// ── Tests ──────────────────────────────────────────────────────

#[cfg(all(test, not(target_arch = "riscv32")))]
mod tests {
    use super::*;

    #[test]
    fn signature_is_ascii_ot() {
        // The GSNPSID top-half signature is the ASCII bytes "OT" (0x4F, 0x54).
        assert_eq!(SNPS_SIGNATURE, u16::from_be_bytes(*b"OT"));
        assert_eq!(SNPS_SIGNATURE, 0x4F54);
    }

    #[test]
    fn signature_check_uses_high_half_only() {
        // A core-ID whose top 16 bits are the signature is recognized regardless
        // of the (revision) low half; the low half must not affect the decision.
        assert!(id_has_signature(0x4F54_0000));
        assert!(id_has_signature(0x4F54_FFFF));
        assert!(id_has_signature((SNPS_SIGNATURE as u32) << 16 | 0x1234));
    }

    #[test]
    fn signature_check_rejects_wrong_high_half() {
        // The signature living in the low half (or anywhere but the top) is not
        // a match — the controller is considered absent.
        assert!(!id_has_signature(0x0000_4F54));
        assert!(!id_has_signature(0));
        assert!(!id_has_signature(0xFFFF_FFFF));
        // Off-by-one in the high half is rejected.
        assert!(!id_has_signature(0x4F55_0000));
        assert!(!id_has_signature(0x4F53_0000));
    }

    #[test]
    fn enumspd_known_values() {
        // DSTS.ENUMSPD encoding from the DWC2 device init (dwc_otgreg.h):
        // 0=High, 1=Full(FS PHY), 2=Low, 3=Full.
        assert_eq!(speed_from_enumspd(0), Speed::High);
        assert_eq!(speed_from_enumspd(1), Speed::FullFs);
        assert_eq!(speed_from_enumspd(2), Speed::Low);
        assert_eq!(speed_from_enumspd(3), Speed::Full);
    }

    #[test]
    fn enumspd_out_of_range_is_full() {
        // The field is 2 bits in hardware (masked & 3), but the decode catches
        // every value ≥ 3 via the wildcard arm → Full (never panics).
        for v in 3u8..=u8::MAX {
            assert_eq!(speed_from_enumspd(v), Speed::Full);
        }
    }

    #[test]
    fn speed_variants_are_distinct() {
        // The four documented field values map to four distinct speeds, so the
        // round-trip through the decoder loses no information for 0..=3.
        let speeds = [speed_from_enumspd(0), speed_from_enumspd(1), speed_from_enumspd(2), speed_from_enumspd(3)];
        for i in 0..speeds.len() {
            for j in (i + 1)..speeds.len() {
                assert_ne!(speeds[i], speeds[j]);
            }
        }
    }
}

// ── Property-based fuzz tests ──────────────────────────────────

#[cfg(all(test, not(target_arch = "riscv32")))]
mod proptests {
    use super::*;
    use proptest::prelude::*;

    proptest! {
        /// Fuzz: signature decode never panics and matches the explicit
        /// high-half comparison for any 32-bit core-ID.
        #[test]
        fn signature_matches_high_half(id in any::<u32>()) {
            prop_assert_eq!(id_has_signature(id), (id >> 16) as u16 == SNPS_SIGNATURE);
        }

        /// Fuzz: only IDs with exactly the signature in the top half are present;
        /// changing any high-half bit away from the signature breaks the match.
        #[test]
        fn signature_iff_high_half_equals(low in any::<u16>(), high in any::<u16>()) {
            let id = ((high as u32) << 16) | low as u32;
            prop_assert_eq!(id_has_signature(id), high == SNPS_SIGNATURE);
        }

        /// Fuzz: speed decode is total over all u8 inputs and only 0/1/2 differ
        /// from Full; everything else collapses to Full.
        #[test]
        fn speed_decode_total(v in any::<u8>()) {
            let s = speed_from_enumspd(v);
            match v {
                0 => prop_assert_eq!(s, Speed::High),
                1 => prop_assert_eq!(s, Speed::FullFs),
                2 => prop_assert_eq!(s, Speed::Low),
                _ => prop_assert_eq!(s, Speed::Full),
            }
        }
    }
}
