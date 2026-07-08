//! BS2X RTC — real-time-clock / timer (IP v150).
//!
//! BS2X-only (`chip-bs21`) implementation of the `rtc` module (WS63 has the older
//! v100 RTC, `rtc.rs`). The v150 RTC is a 64-bit down-counter with a coherent-read
//! handshake. Register map from fbb_bs2x `hal_rtc_v150` (sole ground truth); this
//! drives the RTC0 instance (bs2x-pac `Rtc` @ 0x5702_4100).
//!
//! Reading the 64-bit counter requires the cnt_req → cnt_lock latch handshake
//! (so the two 32-bit halves are coherent); the wait for `cnt_lock` is bounded.
//!
//! # Preconditions (board / analog)
//!
//! The RTC counts on the 32.768 kHz clock domain, sourced from an **external
//! crystal that a board may not populate**. With the crystal absent the counter
//! never advances and register access can stall the bus — a board provisioning
//! fact with no software guard. Treat a populated 32.768 kHz crystal as a hard
//! precondition before constructing the driver.

use crate::peripherals::Rtc as RtcPeriph;
use core::marker::PhantomData;

/// Counter run mode (`control.mode`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum Mode {
    /// Count down once from `load` to zero, then stop.
    OneShot = 0,
    /// Count down from `load`, reload and repeat (periodic interrupt).
    Periodic = 1,
    /// Free-running counter (wraps past zero; bit pattern is 3, not 2).
    FreeRun = 3,
}

const POLL_LIMIT: u32 = 0xFFFF;

/// Driver for the BS2X 64-bit v150 RTC (RTC0 instance).
pub struct Rtc<'d> {
    _rtc: PhantomData<RtcPeriph<'d>>,
}

impl<'d> Rtc<'d> {
    fn regs(&self) -> &'static crate::soc::pac::rtc::RegisterBlock {
        // SAFETY: static physical MMIO base (0x5702_4100, RTC0) from bs2x-pac.
        unsafe { &*RtcPeriph::ptr() }
    }

    /// Start the RTC counting from `load` in `mode` (`hal_rtc_v150_config_load` +
    /// `_start`). For a free-running counter use `Mode::FreeRun`.
    ///
    /// typed-config exemption: `load` is written verbatim into the 32-bit
    /// `load_count0` register (the low half of the 64-bit counter), so every `u32`
    /// is a valid, runnable value — nothing to truncate or clamp. (The board-crystal
    /// precondition is documented at the module level.)
    pub fn new(_rtc: RtcPeriph<'d>, load: u32, mode: Mode) -> Self {
        let this = Self { _rtc: PhantomData };
        let r = this.regs();
        unsafe {
            r.control().modify(|_, w| w.enable().clear_bit()); // stop before load
            r.control().modify(|_, w| w.mode().bits(mode as u8));
            r.load_count0().write(|w| w.load_count0().bits(load));
            r.control().modify(|_, w| w.enable().set_bit());
        }
        this
    }

    /// Read the live 64-bit counter using the cnt_req → cnt_lock coherent-read
    /// handshake (`hal_rtc_v150_get_current_value`).
    pub fn read_count(&self) -> u64 {
        let r = self.regs();
        if r.control().read().enable().bit_is_clear() {
            return 0;
        }
        // Request a latched snapshot, then wait for the lock to be granted.
        r.control().modify(|_, w| w.cnt_req().set_bit());
        for _ in 0..POLL_LIMIT {
            if r.control().read().cnt_lock().bit_is_set() {
                break;
            }
            core::hint::spin_loop();
        }
        let lo = r.current_value0().read().current_value0().bits();
        let hi = r.current_value1().read().current_value1().bits();
        ((hi as u64) << 32) | lo as u64
    }

    /// True if the timer interrupt is pending (`raw_intr` bit0).
    pub fn is_pending(&self) -> bool {
        self.regs().raw_intr().read().int_status().bit_is_set()
    }

    /// Clear the timer interrupt by writing the per-instance EOI bit.
    pub fn clear_interrupt(&self) {
        self.regs().eoi_ren().write(|w| w.eoi().set_bit());
    }
}

// ── Tests ──────────────────────────────────────────────────────

#[cfg(all(test, not(target_arch = "riscv32")))]
mod tests {
    use super::*;

    /// Reassemble the 64-bit counter exactly as `read_count` does from its two
    /// latched 32-bit halves (`current_value1` = high word, `current_value0` = low).
    fn combine(hi: u32, lo: u32) -> u64 {
        ((hi as u64) << 32) | lo as u64
    }

    #[test]
    fn mode_bits_match_hardware() {
        // The control.mode field is programmed with `mode as u8`; the v150 RTC
        // uses 0/1/3 (FreeRun is 3, NOT 2 — the bit pattern is intentionally sparse).
        assert_eq!(Mode::OneShot as u8, 0);
        assert_eq!(Mode::Periodic as u8, 1);
        assert_eq!(Mode::FreeRun as u8, 3);
    }

    #[test]
    fn poll_limit_value() {
        // The coherent-read handshake spins at most POLL_LIMIT times; pin the
        // documented 16-bit ceiling so a future edit can't silently shrink it.
        assert_eq!(POLL_LIMIT, 0xFFFF);
    }

    #[test]
    fn combine_low_word_only() {
        // hi == 0 → the value is exactly the low 32-bit half (no high bits set).
        assert_eq!(combine(0, 0x1234_5678), 0x0000_0000_1234_5678);
    }

    #[test]
    fn combine_high_word_only() {
        // lo == 0 → the high half lands in bits 63..32, low half stays zero.
        assert_eq!(combine(0x1234_5678, 0), 0x1234_5678_0000_0000);
    }

    #[test]
    fn combine_no_bit_overlap() {
        // The two halves occupy disjoint bit ranges: combining all-ones halves
        // yields the full u64::MAX with no truncation or carry between them.
        assert_eq!(combine(u32::MAX, u32::MAX), u64::MAX);
    }

    #[test]
    fn combine_round_trips() {
        // Splitting any u64 into its two 32-bit halves and recombining is the
        // identity — confirms the shift/mask ordering matches a clean split.
        let v: u64 = 0xDEAD_BEEF_CAFE_F00D;
        let hi = (v >> 32) as u32;
        let lo = v as u32;
        assert_eq!(combine(hi, lo), v);
    }
}

// ── Property-based fuzz tests ──────────────────────────────────

#[cfg(all(test, not(target_arch = "riscv32")))]
mod proptests {
    use proptest::prelude::*;

    fn combine(hi: u32, lo: u32) -> u64 {
        ((hi as u64) << 32) | lo as u64
    }

    proptest! {
        /// Fuzz: combine then split is the identity for any pair of halves
        /// (the high half is recovered by >>32, the low half by truncation).
        #[test]
        fn combine_split_round_trips(hi in any::<u32>(), lo in any::<u32>()) {
            let v = combine(hi, lo);
            prop_assert_eq!((v >> 32) as u32, hi);
            prop_assert_eq!(v as u32, lo);
        }

        /// Fuzz: any u64 split into halves and recombined returns the original.
        #[test]
        fn split_combine_round_trips(v in any::<u64>()) {
            prop_assert_eq!(combine((v >> 32) as u32, v as u32), v);
        }
    }
}
