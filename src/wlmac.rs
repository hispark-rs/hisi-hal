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

/// A snapshot of receive-security failures used by the WS63 mask-ROM helper.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct WlmacRxSecurityCounters {
    /// CCMP replay failures.
    pub ccmp_replay_failures: u16,
    /// TKIP replay failures.
    pub tkip_replay_failures: u16,
    /// CCMP MIC failures.
    pub ccmp_mic_failures: u16,
    /// TKIP MIC failures.
    pub tkip_mic_failures: u16,
    /// Receive key-search failures.
    pub key_search_failures: u16,
}

/// A read-only snapshot of the active VAP0 receive-filter identity.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct WlmacFilterState {
    /// Packed receive-filter control state currently programmed into WLMAC.
    pub rx_filter_control: u32,
    /// Station address programmed for hardware VAP0.
    pub station_address: [u8; 6],
    /// BSSID programmed for hardware VAP0.
    pub bssid: [u8; 6],
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

    /// Reads the WLMAC receive replay, MIC, and key-search failure counters.
    ///
    /// These registers are the three security-counter inputs consumed by the
    /// mask-ROM `hh503_get_mac_statistics_data` helper. As with
    /// [`Self::snapshot_rx`], hardware can increment them during the bounded
    /// snapshot.
    #[inline]
    pub fn snapshot_rx_security(&self) -> WlmacRxSecurityCounters {
        critical_section::with(|_| {
            // SAFETY: construction requires the caller to keep WLMAC accessible.
            let regs = unsafe { &*pac::WlmacStats::ptr() };
            let replay = regs.rx_replay_failure_count().read();
            let mic = regs.rx_mic_failure_count().read();

            WlmacRxSecurityCounters {
                ccmp_replay_failures: replay.ccmp().bits(),
                tkip_replay_failures: replay.tkip().bits(),
                ccmp_mic_failures: mic.ccmp().bits(),
                tkip_mic_failures: mic.tkip().bits(),
                key_search_failures: regs
                    .rx_key_search_failure_count()
                    .read()
                    .count()
                    .bits(),
            }
        })
    }

    /// Reads the active receive-filter command and VAP0 address identity.
    ///
    /// Interrupts are masked only while taking the four-register snapshot. The
    /// vendor WLMAC state machine owns these registers, so callers must treat
    /// the result as diagnostic evidence rather than a configuration surface.
    #[inline]
    pub fn snapshot_filter_state(&self) -> WlmacFilterState {
        critical_section::with(|_| {
            // SAFETY: construction requires the caller to keep WLMAC accessible.
            let regs = unsafe { &*pac::WlmacStats::ptr() };
            let heads = regs.vap0_address_heads().read();

            WlmacFilterState {
                rx_filter_control: regs.rx_filter_control().read().control().bits(),
                station_address: decode_address(
                    heads.station().bits(),
                    regs.vap0_station_address_tail().read().bytes().bits(),
                ),
                bssid: decode_address(
                    heads.bssid().bits(),
                    regs.vap0_bssid_tail().read().bytes().bits(),
                ),
            }
        })
    }
}

#[inline]
fn decode_address(head: u16, tail: u32) -> [u8; 6] {
    let head = head.to_be_bytes();
    let tail = tail.to_be_bytes();
    [head[0], head[1], tail[0], tail[1], tail[2], tail[3]]
}

#[cfg(test)]
mod tests {
    use super::decode_address;

    #[test]
    fn decodes_vendor_address_register_order() {
        assert_eq!(
            decode_address(0x02_11, 0x42_63_24_a5),
            [0x02, 0x11, 0x42, 0x63, 0x24, 0xa5]
        );
    }
}
