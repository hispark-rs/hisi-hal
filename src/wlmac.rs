//! Read-only WS63 Wi-Fi MAC diagnostics.
//!
//! The RF stack owns the WLMAC lifecycle. This module therefore does not expose
//! a safe constructor or add WLMAC to [`crate::Peripherals`]. It only provides a
//! bounded snapshot for diagnostics after the caller has established that the
//! radio block is powered and initialized.

use core::marker::PhantomData;

use crate::soc::pac;

/// A snapshot of the receive counters used by the WS63 mask-ROM helper.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct WlmacRxCounters {
    /// Received aggregate MPDU count.
    pub rx_ampdu: u16,
    /// MPDUs with a valid FCS received inside an aggregate.
    pub rx_success_mpdu_in_ampdu: u32,
    /// MPDUs with an invalid FCS received inside an aggregate.
    pub rx_failed_mpdu_in_ampdu: u32,
    /// Received non-aggregate MPDUs with a valid FCS.
    pub rx_success_mpdu: u32,
    /// Received non-aggregate MPDUs with an invalid FCS.
    pub rx_failed_mpdu: u32,
    /// MPDUs rejected by the MAC receive filter.
    pub rx_filtered_mpdu: u16,
}

/// Read-only access to initialized WS63 WLMAC diagnostic counters.
///
/// This capability is neither `Send` nor `Sync`. Constructing another instance
/// remains possible through the unsafe constructor because the RF integration,
/// rather than the HAL peripheral singleton, owns this register block.
pub struct WlmacDiagnostics {
    _not_send_sync: PhantomData<*mut ()>,
}

impl WlmacDiagnostics {
    /// Assumes that the RF integration has initialized and retained WLMAC.
    ///
    /// # Safety
    ///
    /// The caller must ensure the WLMAC register block is powered, clocked, and
    /// not concurrently reset or powered down for the lifetime of this value.
    /// The caller must also ensure that reading these counters is permitted by
    /// the active RF firmware profile.
    #[inline]
    pub const unsafe fn assume_radio_ready() -> Self {
        Self {
            _not_send_sync: PhantomData,
        }
    }

    /// Reads the six WLMAC receive counters used by the mask-ROM statistics API.
    ///
    /// Interrupts are masked only for the bounded register snapshot, matching
    /// the mask-ROM helper's software-coherence behavior. Hardware can continue
    /// incrementing counters while the snapshot is read, so the fields are not
    /// an atomic hardware-time sample.
    #[inline]
    pub fn snapshot_rx(&self) -> WlmacRxCounters {
        critical_section::with(|_| {
            // SAFETY: construction requires the caller to keep WLMAC accessible.
            let regs = unsafe { &*pac::WlmacStats::ptr() };

            WlmacRxCounters {
                rx_ampdu: regs.rx_ampdu_count().read().count().bits(),
                rx_success_mpdu_in_ampdu: regs
                    .rx_success_mpdu_in_ampdu_count()
                    .read()
                    .count()
                    .bits(),
                rx_failed_mpdu_in_ampdu: regs
                    .rx_failed_mpdu_in_ampdu_count()
                    .read()
                    .count()
                    .bits(),
                rx_success_mpdu: regs.rx_success_mpdu_count().read().count().bits(),
                rx_failed_mpdu: regs.rx_failed_mpdu_count().read().count().bits(),
                rx_filtered_mpdu: regs.rx_filtered_mpdu_count().read().count().bits(),
            }
        })
    }
}
