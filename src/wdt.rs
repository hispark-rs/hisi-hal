//! Watchdog Timer (WDT) driver for WS63.
//!
//! The WS63 WDT is a 24-bit down-counter that can generate an interrupt
//! or system reset when it reaches zero. The watchdog must be periodically
//! "fed" (restarted) to prevent a timeout.
//!
//! # Lock mechanism
//!
//! The WDT registers are protected by a lock. Write `0x5A5A5A5A` to
//! `WDT_LOCK` to unlock, any other value to lock. The driver handles
//! this automatically.
//!
//! # Operating modes
//!
//! - **Single-interrupt mode** (`wdt_mode = 0`): One interrupt before reset
//! - **Double-interrupt mode** (`wdt_mode = 1`): Two interrupts before reset
//!
//! # Clock source
//!
//! The WDT counts at the **TCXO crystal clock** ([`WDT_CLOCK_HZ`] = 24 MHz on
//! 24 MHz-crystal boards) — NOT a 32.768 kHz RTC clock — programmed by the vendor
//! `watchdog_port_set_clock(REQ_24M)` in `clock_init`. The 24-bit `WDT_LOAD` field
//! lives in bits [31:8], so the effective counter has 256-cycle resolution
//! (`WDT_LOAD_RESEV = 8`) and the maximum timeout is `(0xFFFFFF << 8) / 24 MHz ≈ 178 s`.

use crate::peripherals::Wdt;

/// WDT reset pulse length in clock cycles.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ResetPulseLength {
    /// 2 clock cycles (~83 ns at 24 MHz)
    Cycles2 = 0,
    /// 4 clock cycles
    Cycles4 = 1,
    /// 8 clock cycles
    Cycles8 = 2,
    /// 16 clock cycles
    Cycles16 = 3,
    /// 32 clock cycles
    Cycles32 = 4,
    /// 64 clock cycles
    Cycles64 = 5,
    /// 128 clock cycles
    Cycles128 = 6,
    /// 256 clock cycles (~10.7 µs at 24 MHz)
    Cycles256 = 7,
}

/// WDT operating mode.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum WdtMode {
    /// One interrupt, then reset on next timeout.
    SingleInterrupt,
    /// Two interrupts, then reset on third timeout.
    DoubleInterrupt,
}

/// Watchdog configuration / operation error.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum WdtError {
    /// The counter-value latch did not become valid within the bounded poll
    /// (the WDT block is unclocked or wedged).
    Busy,
}

/// A validated watchdog timeout in milliseconds.
///
/// The constructor [`WdtTimeout::from_ms`] returns `None` for any value that would
/// fall outside the representable range — instead of the old silent saturation
/// (`timeout > ~178 s → WDT_MAX_LOAD`). A `WdtTimeout` in hand always programs a
/// `WDT_LOAD` field in `[1, WDT_MAX_LOAD]`, so [`Watchdog::configure`] can never
/// clamp or truncate it.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct WdtTimeout(u32);

impl WdtTimeout {
    /// The largest timeout (ms) whose 24-bit `WDT_LOAD` field does not overflow:
    /// `(WDT_MAX_LOAD << RESEV) / WDT_CLOCK_HZ · 1000`, ≈ 178 000 ms at 24 MHz.
    pub const MAX_MS: u32 = {
        let max_cycles = (WDT_MAX_LOAD as u64) << WDT_LOAD_RESEV;
        (max_cycles * 1000 / (WDT_CLOCK_HZ as u64)) as u32
    };

    /// Construct from milliseconds. `None` if `ms == 0` (no useful watchdog) or if
    /// the resulting `WDT_LOAD` field would exceed [`WDT_MAX_LOAD`] (i.e.
    /// `ms > MAX_MS`, the old silent-saturation boundary).
    pub const fn from_ms(ms: u32) -> Option<Self> {
        if ms == 0 {
            return None;
        }
        let cycles = (ms as u64) * (WDT_CLOCK_HZ as u64) / 1000;
        if (cycles >> WDT_LOAD_RESEV) > WDT_MAX_LOAD as u64 {
            return None;
        }
        Some(WdtTimeout(ms))
    }

    /// The timeout in milliseconds.
    pub const fn as_ms(self) -> u32 {
        self.0
    }

    /// The 24-bit `WDT_LOAD` field value (`cycles >> RESEV`), guaranteed
    /// `<= WDT_MAX_LOAD` by construction.
    const fn load_field(self) -> u32 {
        let cycles = (self.0 as u64) * (WDT_CLOCK_HZ as u64) / 1000;
        (cycles >> WDT_LOAD_RESEV) as u32
    }
}

/// Bounded spin limit for the `WDT_CCVR` counter-value latch (`request_counter`).
/// The latch normally validates in a handful of cycles; this is a generous cap so a
/// wedged/unclocked block returns [`WdtError::Busy`] instead of hanging forever.
const WDT_CCVR_POLL_LIMIT: u32 = 100_000;

/// Watchdog Timer driver.
pub struct Watchdog<'d> {
    _wdt: Wdt<'d>,
}

/// WDT counting clock = the TCXO crystal ([`crate::soc::chip::TCXO_HZ`], 24 MHz).
pub const WDT_CLOCK_HZ: u32 = crate::soc::chip::TCXO_HZ;

/// Maximum `WDT_LOAD` field value (24-bit; the field occupies `WDT_LOAD` bits [31:8]).
pub const WDT_MAX_LOAD: u32 = 0x00FF_FFFF;

/// Reserved low bits of `WDT_LOAD`: the 24-bit load field sits in bits [31:8], so
/// the total timeout cycle count is shifted right by this to form the field value
/// (matches the vendor `LOAD_RESEV = 8` in `hal_watchdog_v151`).
const WDT_LOAD_RESEV: u32 = 8;

impl<'d> Watchdog<'d> {
    /// Create a new watchdog driver from the WDT peripheral.
    pub fn new(wdt: Wdt<'d>) -> Self {
        let wd = Self { _wdt: wdt };
        wd.unlock();
        wd
    }

    fn regs(&self) -> &'static crate::soc::pac::wdt::RegisterBlock {
        // SAFETY: PAC peripheral pointer is a static physical MMIO address, always valid
        unsafe { &*Wdt::ptr() }
    }

    /// Unlock the WDT registers for configuration.
    fn unlock(&self) {
        unsafe {
            self.regs().wdt_lock().write(|w| w.bits(0x5A5A5A5A));
        }
    }

    /// Lock the WDT registers to prevent accidental modification.
    fn lock(&self) {
        unsafe {
            self.regs().wdt_lock().write(|w| w.bits(0x0000_0000));
        }
    }

    /// Configure and enable the watchdog.
    ///
    /// The timeout is a validated [`WdtTimeout`] (built via [`WdtTimeout::from_ms`]),
    /// so an out-of-range period is rejected at construction rather than silently
    /// saturated here.
    ///
    /// # Arguments
    ///
    /// * `timeout` - Validated timeout period (see [`WdtTimeout`]).
    /// * `mode` - Interrupt mode (single or double interrupt before reset).
    /// * `reset_enable` - Whether to enable system reset on timeout.
    /// * `reset_pulse` - Reset pulse length if reset is enabled.
    ///
    /// # Errors
    ///
    /// [`WdtError::Busy`] if the counter-value latch does not validate within the
    /// bounded poll (an unclocked/wedged WDT block).
    pub fn configure(
        &mut self,
        timeout: WdtTimeout,
        mode: WdtMode,
        reset_enable: bool,
        reset_pulse: ResetPulseLength,
    ) -> Result<(), WdtError> {
        // The 24-bit WDT_LOAD field, guaranteed <= WDT_MAX_LOAD by `WdtTimeout`.
        let load = timeout.load_field();

        self.unlock();

        // Write the 24-bit load field into WDT_LOAD bits [31:8].
        unsafe {
            self.regs().wdt_load().write(|w| w.bits(load << WDT_LOAD_RESEV));
        }

        // Configure control register
        let mut cr: u32 = 0;
        cr |= 0x01; // wdt_en = 1
        if reset_enable {
            cr |= 1 << 2; // rst_en
        }
        cr |= (reset_pulse as u32) << 3; // rst_pl
        cr |= 1 << 6; // wdt_imsk = 1 (mask interrupt initially)
        if matches!(mode, WdtMode::DoubleInterrupt) {
            cr |= 1 << 7; // wdt_mode = 1 (double interrupt)
        }

        unsafe {
            self.regs().wdt_cr().write(|w| w.bits(cr));
        }

        // Request counter value update (bounded; propagates Busy on a wedged block).
        let res = self.request_counter();

        self.lock();
        res
    }

    /// Enable the watchdog.
    pub fn enable(&mut self) {
        self.unlock();
        let cr = self.regs().wdt_cr().read().bits();
        unsafe {
            self.regs().wdt_cr().write(|w| w.bits(cr | 0x01));
        }
        self.lock();
    }

    /// Disable the watchdog.
    ///
    /// # Safety
    ///
    /// Disabling the watchdog may leave the system vulnerable to lock-ups.
    /// Only use this when you have an alternative safety mechanism.
    pub fn disable(&mut self) {
        self.unlock();
        let cr = self.regs().wdt_cr().read().bits();
        unsafe {
            self.regs().wdt_cr().write(|w| w.bits(cr & !0x01));
        }
        self.lock();
    }

    /// Feed (restart) the watchdog to prevent timeout.
    ///
    /// Writes any value other than `0x5A5A5A5A` to the restart register.
    pub fn feed(&mut self) {
        self.unlock();
        unsafe {
            self.regs().wdt_restart().write(|w| w.bits(0x0000_0001));
        }
        self.lock();
    }

    /// Read the current counter value.
    ///
    /// # Errors
    ///
    /// [`WdtError::Busy`] if the counter-value latch does not validate within the
    /// bounded poll.
    pub fn counter_value(&self) -> Result<u32, WdtError> {
        self.request_counter()?;
        Ok(self.regs().wdt_cnt().read().bits())
    }

    /// Request a counter-value latch and wait (bounded) for it to validate.
    ///
    /// Returns [`WdtError::Busy`] after [`WDT_CCVR_POLL_LIMIT`] spins instead of
    /// hanging forever on an unclocked/wedged block.
    fn request_counter(&self) -> Result<(), WdtError> {
        unsafe {
            self.regs().wdt_ccvr_en().write(|w| w.bits(0x01));
        }
        for _ in 0..WDT_CCVR_POLL_LIMIT {
            if self.regs().wdt_ccvr_en().read().bits() & 0x02 != 0 {
                return Ok(());
            }
            core::hint::spin_loop();
        }
        Err(WdtError::Busy)
    }

    /// Check if a watchdog interrupt is pending (raw, unmasked).
    pub fn interrupt_pending(&self) -> bool {
        self.regs().wdt_raw_intr().read().bits() & 0x01 != 0
    }

    /// Check if a watchdog interrupt is pending (after mask).
    pub fn interrupt_masked(&self) -> bool {
        self.regs().wdt_intr().read().bits() & 0x01 != 0
    }

    /// Clear the watchdog interrupt (read from `WDT_EOI`).
    pub fn clear_interrupt(&self) {
        let _ = self.regs().wdt_eoi().read().bits();
    }

    /// Enable the watchdog interrupt (unmask).
    pub fn enable_interrupt(&mut self) {
        self.unlock();
        let cr = self.regs().wdt_cr().read().bits();
        unsafe {
            self.regs().wdt_cr().write(|w| w.bits(cr & !(1 << 6)));
        }
        self.lock();
    }

    /// Disable the watchdog interrupt (mask).
    pub fn disable_interrupt(&mut self) {
        self.unlock();
        let cr = self.regs().wdt_cr().read().bits();
        unsafe {
            self.regs().wdt_cr().write(|w| w.bits(cr | (1 << 6)));
        }
        self.lock();
    }

    /// Check if the watchdog is currently busy.
    pub fn is_busy(&self) -> bool {
        self.regs().wdt_status().read().bits() & 0x01 == 0
    }
}

// ── Tests ──────────────────────────────────────────────────────

#[cfg(all(test, not(target_arch = "riscv32")))]
mod tests {
    use super::{ResetPulseLength, WDT_CLOCK_HZ, WDT_LOAD_RESEV, WDT_MAX_LOAD, WdtMode, WdtTimeout};

    /// The `WDT_LOAD` field a *valid* `WdtTimeout` programs — `from_ms(ms)` then the
    /// private `load_field()`. Panics on an out-of-range `ms` (the test's contract:
    /// only valid timeouts reach the field math now; over-range is rejected up front).
    fn load_field(timeout_ms: u32) -> u32 {
        WdtTimeout::from_ms(timeout_ms).expect("ms in range").load_field()
    }

    /// Re-derivation of the control-register encoding from `configure()`.
    fn control_bits(mode: WdtMode, reset_enable: bool, reset_pulse: ResetPulseLength) -> u32 {
        let mut cr: u32 = 0;
        cr |= 0x01; // wdt_en
        if reset_enable {
            cr |= 1 << 2; // rst_en
        }
        cr |= (reset_pulse as u32) << 3; // rst_pl
        cr |= 1 << 6; // wdt_imsk
        if matches!(mode, WdtMode::DoubleInterrupt) {
            cr |= 1 << 7; // wdt_mode
        }
        cr
    }

    #[test]
    fn reset_pulse_discriminants_are_the_3bit_field() {
        // The eight pulse lengths map to the contiguous 3-bit rst_pl field 0..=7.
        assert_eq!(ResetPulseLength::Cycles2 as u32, 0);
        assert_eq!(ResetPulseLength::Cycles4 as u32, 1);
        assert_eq!(ResetPulseLength::Cycles8 as u32, 2);
        assert_eq!(ResetPulseLength::Cycles16 as u32, 3);
        assert_eq!(ResetPulseLength::Cycles32 as u32, 4);
        assert_eq!(ResetPulseLength::Cycles64 as u32, 5);
        assert_eq!(ResetPulseLength::Cycles128 as u32, 6);
        assert_eq!(ResetPulseLength::Cycles256 as u32, 7);
    }

    #[test]
    fn zero_timeout_rejected() {
        // A 0 ms timeout is not a useful watchdog → rejected at construction.
        assert!(WdtTimeout::from_ms(0).is_none());
    }

    #[test]
    fn known_timeout_matches_hand_computed_field() {
        // 1000 ms = WDT_CLOCK_HZ cycles; the field is that shifted right by RESEV.
        let cycles = WDT_CLOCK_HZ as u64; // 1000 ms * clock / 1000
        let expected = (cycles >> WDT_LOAD_RESEV) as u32;
        assert_eq!(load_field(1000), expected);
        assert!(expected <= WDT_MAX_LOAD);
    }

    #[test]
    fn over_range_timeout_rejected() {
        // u32::MAX ms vastly exceeds the 24-bit field → rejected (was silently
        // saturated to WDT_MAX_LOAD before the tightening).
        assert!(WdtTimeout::from_ms(u32::MAX).is_none());
        // The boundary is exact: MAX_MS constructs, MAX_MS+1 does not.
        assert!(WdtTimeout::from_ms(WdtTimeout::MAX_MS).is_some());
        assert!(WdtTimeout::from_ms(WdtTimeout::MAX_MS + 1).is_none());
        // The largest accepted timeout still fits the 24-bit field.
        assert!(WdtTimeout::from_ms(WdtTimeout::MAX_MS).unwrap().load_field() <= WDT_MAX_LOAD);
    }

    #[test]
    fn load_field_is_monotonic_in_range() {
        // More milliseconds never produce a smaller field (within the valid range).
        let a = load_field(1000);
        let b = load_field(2000);
        assert!(b >= a);
        assert!(b <= WDT_MAX_LOAD);
    }

    #[test]
    fn first_timeout_past_boundary_is_rejected() {
        // A timeout past the (WDT_MAX_LOAD+1)<<RESEV cycle boundary no longer
        // clamps — it is rejected. Compute that boundary in ms with CEILING
        // division (+1 ms margin) so the input definitely clears the threshold.
        let threshold_cycles = (WDT_MAX_LOAD as u64 + 1) << WDT_LOAD_RESEV;
        let big_ms = (threshold_cycles * 1000).div_ceil(WDT_CLOCK_HZ as u64) + 1;
        let ms = big_ms.min(u32::MAX as u64) as u32;
        assert!(WdtTimeout::from_ms(ms).is_none());
    }

    #[test]
    fn control_bits_minimal_config() {
        // Single-interrupt, no reset, pulse 0: only wdt_en (bit0) + wdt_imsk (bit6).
        let cr = control_bits(WdtMode::SingleInterrupt, false, ResetPulseLength::Cycles2);
        assert_eq!(cr, 0x01 | (1 << 6));
    }

    #[test]
    fn control_bits_full_config() {
        // Double-interrupt + reset + pulse 256 (field 7): all encoded bits set.
        let cr = control_bits(WdtMode::DoubleInterrupt, true, ResetPulseLength::Cycles256);
        let expected = 0x01            // wdt_en
            | (1 << 2)                 // rst_en
            | (7 << 3)                 // rst_pl = Cycles256
            | (1 << 6)                 // wdt_imsk
            | (1 << 7); // wdt_mode (double)
        assert_eq!(cr, expected);
    }

    #[test]
    fn control_bits_reset_pulse_occupies_field() {
        // rst_pl lives in bits [5:3]; verify each variant lands there and nowhere else.
        for (variant, val) in
            [(ResetPulseLength::Cycles2, 0u32), (ResetPulseLength::Cycles16, 3), (ResetPulseLength::Cycles256, 7)]
        {
            let cr = control_bits(WdtMode::SingleInterrupt, false, variant);
            assert_eq!((cr >> 3) & 0x7, val);
        }
    }

    #[test]
    fn mode_only_affects_bit7() {
        // The only difference between single and double interrupt is wdt_mode (bit7).
        let single = control_bits(WdtMode::SingleInterrupt, true, ResetPulseLength::Cycles32);
        let double = control_bits(WdtMode::DoubleInterrupt, true, ResetPulseLength::Cycles32);
        assert_eq!(single ^ double, 1 << 7);
    }
}

// ── Property-based fuzz tests ──────────────────────────────────

#[cfg(all(test, not(target_arch = "riscv32")))]
mod proptests {
    use super::{WDT_MAX_LOAD, WdtTimeout};
    use proptest::prelude::*;

    proptest! {
        /// Fuzz: `WdtTimeout::from_ms` never panics for any u32 (incl. 0 and
        /// u32::MAX) — it returns `None` out of range instead of overflowing/clamping.
        #[test]
        fn from_ms_never_panics(ms in any::<u32>()) {
            let _ = WdtTimeout::from_ms(ms);
        }

        /// Fuzz: every timeout that `WdtTimeout` ACCEPTS programs a load field within
        /// the 24-bit max (invalid timeouts are rejected up front, so this is the
        /// only path that reaches `configure`).
        #[test]
        fn accepted_timeout_fits_field(ms in any::<u32>()) {
            if let Some(t) = WdtTimeout::from_ms(ms) {
                prop_assert!(t.load_field() <= WDT_MAX_LOAD);
            }
        }

        /// Fuzz: acceptance is monotone — if a timeout is accepted, every smaller
        /// non-zero timeout is accepted too (the valid range is a contiguous interval).
        #[test]
        fn acceptance_is_monotone(ms in 1u32..=WdtTimeout::MAX_MS) {
            prop_assert!(WdtTimeout::from_ms(ms).is_some());
        }
    }
}
