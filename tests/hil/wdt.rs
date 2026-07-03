use crate::{hal, pac};

/// Watchdog timeout validation + load programming (wdt.rs).
#[cfg(feature = "chip-ws63")]
pub(crate) fn wdt_configure_saturates_load() {
    use hal::wdt::{ResetPulseLength, WDT_MAX_LOAD, Watchdog, WdtMode, WdtTimeout};

    assert!(WdtTimeout::from_ms(300_000).is_none(), "over-range timeout must be rejected");
    assert!(WdtTimeout::from_ms(WdtTimeout::MAX_MS).is_some());
    assert!(WdtTimeout::from_ms(WdtTimeout::MAX_MS + 1).is_none());

    let timeout = WdtTimeout::from_ms(1_000).unwrap();
    // SAFETY: sequential single-hart run; WDT singleton not otherwise held.
    let mut wdt = Watchdog::new(unsafe { hal::peripherals::Wdt::steal() });
    wdt.configure(timeout, WdtMode::SingleInterrupt, false, ResetPulseLength::Cycles2)
        .expect("configure should succeed on live silicon");

    let expected = ((timeout.as_ms() as u64 * hal::wdt::WDT_CLOCK_HZ as u64 / 1000) >> 8) as u32;
    assert!(expected <= WDT_MAX_LOAD);
    // SAFETY: read-only MMIO load of the WDT load register; field is in [31:8].
    let r = unsafe { &*pac::Wdt::PTR };
    let load = r.wdt_load().read().bits() >> 8;
    assert_eq!(load, expected, "WDT load mismatch: got 0x{:06x} want 0x{:06x}", load, expected);
    wdt.disable();
}

/// Watchdog counter + feed liveness (wdt.rs).
#[cfg(feature = "chip-ws63")]
pub(crate) fn wdt_counter_value_and_feed() {
    use hal::wdt::{ResetPulseLength, Watchdog, WdtMode, WdtTimeout};

    // SAFETY: sequential single-hart run; WDT singleton not otherwise held.
    let mut wdt = Watchdog::new(unsafe { hal::peripherals::Wdt::steal() });
    wdt.configure(WdtTimeout::from_ms(1_000).unwrap(), WdtMode::SingleInterrupt, false, ResetPulseLength::Cycles2)
        .expect("configure should succeed on live silicon");

    let before = wdt.counter_value().expect("first WDT counter latch timed out");
    wdt.feed();
    let after = wdt.counter_value().expect("second WDT counter latch timed out");
    assert_ne!(before, 0xFFFF_FFFF, "WDT counter read returned the bus-floating all-ones pattern");
    assert_ne!(after, 0xFFFF_FFFF, "WDT counter read after feed returned all-ones");
    wdt.disable();
}

/// WDT drop-to-disable and `into_armed()` escape hatch.
#[cfg(feature = "chip-ws63")]
pub(crate) fn wdt_drop_disables_unless_armed() {
    use hal::wdt::{ResetPulseLength, Watchdog, WdtMode, WdtTimeout};

    let r = unsafe { &*pac::Wdt::PTR };
    let cfg = |wdt: &mut Watchdog| {
        wdt.configure(WdtTimeout::from_ms(1_000).unwrap(), WdtMode::SingleInterrupt, false, ResetPulseLength::Cycles2)
            .expect("configure on live silicon");
    };

    {
        // SAFETY: sequential single-hart run; WDT singleton not otherwise held.
        let mut wdt = Watchdog::new(unsafe { hal::peripherals::Wdt::steal() });
        cfg(&mut wdt);
        assert_eq!(r.wdt_cr().read().bits() & 0x1, 0x1, "WDT not enabled after configure");
    }
    assert_eq!(r.wdt_cr().read().bits() & 0x1, 0x0, "drop did not clear WDT_CR.wdt_en");

    {
        let mut wdt = Watchdog::new(unsafe { hal::peripherals::Wdt::steal() });
        cfg(&mut wdt);
        let _armed = wdt.into_armed();
    }
    assert_eq!(r.wdt_cr().read().bits() & 0x1, 0x1, "into_armed must keep WDT enabled");

    Watchdog::new(unsafe { hal::peripherals::Wdt::steal() }).disable();
}

/// WDT `leak()` escape hatch is the documented alias for `into_armed()`.
#[cfg(feature = "chip-ws63")]
pub(crate) fn wdt_leak_keeps_watchdog_armed() {
    use hal::wdt::{ResetPulseLength, Watchdog, WdtMode, WdtTimeout};

    let r = unsafe { &*pac::Wdt::PTR };
    // SAFETY: sequential single-hart run; WDT singleton not otherwise held.
    let mut wdt = Watchdog::new(unsafe { hal::peripherals::Wdt::steal() });
    wdt.configure(WdtTimeout::from_ms(1_000).unwrap(), WdtMode::SingleInterrupt, false, ResetPulseLength::Cycles2)
        .expect("configure on live silicon");
    let _armed = wdt.leak();

    assert_eq!(r.wdt_cr().read().bits() & 0x1, 0x1, "leak must keep WDT enabled");

    Watchdog::new(unsafe { hal::peripherals::Wdt::steal() }).disable();
}
