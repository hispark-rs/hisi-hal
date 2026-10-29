//! embassy-time `Driver` for the HiSilicon WS63 / BS2X family (`embassy` feature).
//!
//! Makes the chip an [embassy-time](https://docs.rs/embassy-time) provider so
//! `Timer::after`, `Instant`, `Ticker`, etc. work under embassy-executor:
//!
//! * `now()` reads the **TCXO 64-bit free-running counter** (the crystal rate
//!   [`crate::soc::chip::TCXO_HZ`] — 24 MHz on WS63, 32 MHz on BS2X) and scales
//!   it to embassy-time's 1 MHz tick (microseconds). It is monotonic and tracks
//!   real (virtual, on QEMU) elapsed time.
//! * Alarms use **one TIMER channel** (`ALARM_CH`) as a one-shot; on expiry the
//!   app routes the trap to [`on_alarm_interrupt`], which drains the expired
//!   wakers and re-arms for the next deadline ([`embassy_time_queue_utils`]).
//!
//! TIMER access is shared; TCXO counter layout is selected by chip. WS63 uses
//! two 32-bit data registers and BS21 uses four 16-bit data registers. Crystal
//! rate and alarm IRQ come from `soc::chip`.
//!
//! Requirements (see `examples/ws63/embassy_multitask`, `examples/bs21/embassy_multitask`):
//! * The application enables `embassy-time/tick-hz-1_000_000` (matches `TICK_HZ`).
//! * The application routes the TIMER alarm-channel trap (`mcause & 0xFFF` ==
//!   [`ALARM_IRQ`]) to [`on_alarm_interrupt`] and has enabled global interrupts.

use core::cell::RefCell;
use core::num::NonZeroU32;
use core::task::Waker;

use critical_section::{CriticalSection, Mutex};
use embassy_time_driver::Driver;
use embassy_time_queue_utils::Queue;

use crate::interrupt;
use crate::peripherals::{Tcxo, Timer};
use crate::soc::chip::TIMER_CLOCK_HZ;

const TCXO_HZ: u64 = crate::soc::chip::TCXO_HZ as u64;
/// embassy-time tick rate — MUST match the app's `embassy-time/tick-hz-*` feature.
const TICK_HZ: u64 = 1_000_000;
const _: () = assert!(TCXO_HZ.is_multiple_of(TICK_HZ) && TCXO_HZ >= TICK_HZ);
const _: () = assert!((TIMER_CLOCK_HZ as u64).is_multiple_of(TICK_HZ) && TIMER_CLOCK_HZ as u64 >= TICK_HZ);
const TCXO_COUNTS_PER_TICK: NonZeroU32 = NonZeroU32::new((TCXO_HZ / TICK_HZ) as u32).unwrap();
const TIMER_COUNTS_PER_TICK: NonZeroU32 = NonZeroU32::new((TIMER_CLOCK_HZ as u64 / TICK_HZ) as u32).unwrap();
/// TIMER channel reserved for embassy-time alarms.
const ALARM_CH: usize = 0;

/// IRQ number of the embassy-time alarm interrupt — **26 on WS63** (`TIMER_INT0`,
/// a standard `mie`-bit local interrupt), **53 on BS2X** (`TIMER_0`, a LOCI custom
/// local interrupt). Both are delivered with `mcause = <irq>`, so the application's
/// machine-trap handler routes the alarm by testing `mcause & 0xFFF == ALARM_IRQ`.
pub const ALARM_IRQ: u32 = crate::soc::chip::ALARM_INTERRUPT as u32;

/// Read the TCXO 64-bit counter and scale to embassy-time ticks (µs).
fn now_ticks() -> u64 {
    // SAFETY: read-only/RMW of the TCXO MMIO block via the singleton pointer.
    let r = unsafe { &*Tcxo::ptr() };
    let st = r.tcxo_status().read().bits();
    unsafe { r.tcxo_status().write(|w| w.bits(st | 0x01)) }; // refresh-latch
    for _ in 0..100 {
        if r.tcxo_status().read().bits() & 0x10 != 0 {
            break; // count-valid
        }
    }
    let count = crate::tcxo_words::read_counter(r);
    // TCXO crystal counter -> 1 MHz ticks (TCXO_HZ is per-chip from soc::chip).
    crate::embassy_time_math::counter_ticks(count, TCXO_COUNTS_PER_TICK)
}

struct HisiTimeDriver {
    queue: Mutex<RefCell<Queue>>,
}

impl HisiTimeDriver {
    /// Program the alarm TIMER one-shot to fire at tick `at` (or disable it for
    /// `u64::MAX`). Called inside a critical section.
    fn set_alarm(&self, _cs: CriticalSection<'_>, at: u64) {
        // SAFETY: RMW of the TIMER MMIO block via the singleton pointer.
        let r = unsafe { &*Timer::ptr() };
        // Stop + clear the alarm channel first.
        let prev = r.timer0_control(ALARM_CH).read().bits();
        unsafe { r.timer0_control(ALARM_CH).write(|w| w.bits(prev & !1)) };
        // SAFETY: TIMER_EOI is a write-only command register; `1` is the
        // complete command value and does not preserve prior bits.
        r.timer0_eoi(ALARM_CH).write(|w| unsafe { w.bits(1) });
        interrupt::clear_pending(crate::soc::chip::ALARM_INTERRUPT);

        if at == u64::MAX {
            return; // no pending timers
        }

        let now = now_ticks();
        // Keep far deadlines as bounded hardware segments without overflowing.
        let delta = at.saturating_sub(now).max(1);
        let counts = crate::embassy_time_math::alarm_counts(delta, 0, TIMER_COUNTS_PER_TICK);
        unsafe {
            r.timer0_load_count(ALARM_CH).write(|w| w.bits(counts));
            // Raw control = 0x1: enable (bit0), mode bits = 0b00 (one-shot in HW —
            // do NOT use the PAC's typed `.mode()`, whose enum is inverted vs the
            // dw_apb timer), int_mask (bit3) = 0 = unmasked. Verified vs fbb SDKs.
            r.timer0_control(ALARM_CH).write(|w| w.bits(1));
            interrupt::enable(crate::soc::chip::ALARM_INTERRUPT);
        }
    }
}

impl Driver for HisiTimeDriver {
    fn now(&self) -> u64 {
        now_ticks()
    }

    fn schedule_wake(&self, at: u64, waker: &Waker) {
        critical_section::with(|cs| {
            let mut q = self.queue.borrow(cs).borrow_mut();
            if q.schedule_wake(at, waker) {
                // Earliest deadline changed — re-arm the hardware alarm.
                let next = q.next_expiration(now_ticks());
                self.set_alarm(cs, next);
            }
        });
    }
}

embassy_time_driver::time_driver_impl!(
    static DRIVER: HisiTimeDriver = HisiTimeDriver {
        queue: Mutex::new(RefCell::new(Queue::new()))
    }
);

/// Alarm-channel handler: wakes expired timers and re-arms the next deadline.
///
/// The application or runtime that owns trap routing must call this from the
/// alarm IRQ. The HAL deliberately does not export a strong `TIMER_INT0` symbol:
/// Cargo feature unification can otherwise install that handler in unrelated
/// workspace binaries and conflict with their runtime-owned vector.
pub fn on_alarm_interrupt() {
    critical_section::with(|cs| {
        let mut q = DRIVER.queue.borrow(cs).borrow_mut();
        let next = q.next_expiration(now_ticks());
        DRIVER.set_alarm(cs, next);
    });
}
