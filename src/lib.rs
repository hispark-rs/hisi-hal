//! # hisi-riscv-hal — Hardware Abstraction Layer for HiSilicon WS63 (RISC-V).
//!
//! A comprehensive HAL providing safe, idiomatic Rust APIs for all WS63
//! peripherals. Modeled on esp-hal patterns with typed GPIO drivers, audited
//! clock metadata, typed identities, and embedded-hal trait implementations.
//!
//! ## Clock gating
//!
//! Most peripherals need their CLDO_CRG clock gate enabled before register
//! access. The gates default to enabled out of reset; `clock_init::init_clocks()`
//! sets up the system clocks for firmware that does not boot through flashboot.
//! Constructors like `I2c::new_i2c0()`, `Uart::new_uart0()`, and `Watchdog::new()`
//! write configuration registers immediately. WDT/RTC/TCXO are always-on.
//!
//! ```ignore
//! // Illustrative shape only (`system`/`peripherals` stand in for your tokens):
//! let clocks = clock_init::init_clocks(&system.sys_ctl0, &system.cldo_crg);
//! // Now safe to construct peripheral drivers
//! let uart = Uart::new_uart0(peripherals.UART0, Config::default());
//! ```
//!
//! ## MSRV
//!
//! The minimum supported Rust version is **1.88** (declared as `rust-version` in
//! `Cargo.toml` — bumped from 1.85 for the `instability` crate). An MSRV bump is a
//! minor-version change, not a patch.
// `no_std` for firmware builds; `std` is linked under `cfg(test)` ONLY on the
// host so the lib unit tests can use the standard test harness (run via
// `cargo test --target x86_64`). On the RISC-V target the lib stays `no_std`
// even under the `test` cfg: `cargo test --target riscv32imfc-...` builds the
// lib-test target too (alongside tests/hil.rs), and `std`/`test` don't exist on
// the bare-metal target — the host-only unit-test modules are themselves gated
// `#[cfg(all(test, not(target_arch = "riscv32")))]`, so they vanish there.
#![cfg_attr(not(all(test, not(target_arch = "riscv32"))), no_std)]
#![allow(non_camel_case_types)]
#![allow(rustdoc::broken_intra_doc_links)]
// 0.5.0: every public item is documented; `deny` so a future undocumented pub item
// fails the build (and the doc CI job) rather than silently regressing.
#![deny(missing_docs)]
// docs.rs-only nightly features for the `#[instability::unstable]` "requires unstable"
// markers (the `doc(cfg(feature="unstable"))` annotations). Only active under
// `--cfg docsrs` (set by docs.rs); stable builds are unaffected. Mirrors esp-hal.
#![cfg_attr(docsrs, feature(doc_cfg, custom_inner_attributes, proc_macro_hygiene))]
#![cfg_attr(docsrs, allow(invalid_doc_attributes))]
#![cfg_attr(docsrs, doc(auto_cfg = false))]

// Exactly one chip must be selected (each pulls its PAC + soc/<chip>.rs). There is
// NO default chip (esp-hal style) — every consumer names one explicitly.
#[cfg(all(feature = "chip-ws63", feature = "chip-bs21"))]
compile_error!("hisi-riscv-hal: select exactly ONE chip feature — `chip-ws63` OR `chip-bs21`, not both.");
#[cfg(not(any(feature = "chip-ws63", feature = "chip-bs21")))]
compile_error!(
    "hisi-riscv-hal: no chip selected — enable exactly one chip feature, e.g. \
     `features = [\"chip-ws63\"]` (WS63) or `features = [\"chip-bs21\", \"unstable\"]` (BS2X). \
     There is no default chip."
);
#[cfg(all(feature = "chip-bs21", not(feature = "unstable")))]
compile_error!(
    "hisi-riscv-hal: BS2X support is experimental; enable `unstable` with \
     `features = [\"chip-bs21\", \"unstable\"]`."
);

// ── Internal helper macros (must come before any `unstable_module!` use) ────
// Crate-local `unstable_module!`/`unstable_driver!` + the `#[macro_export]`'d
// `any_peripheral!`/`infallible!`. Private module — the gating macros are NOT
// `#[macro_export]` (esp-hal pattern: crate-internal helpers), and `#[macro_use]`
// makes them visible crate-wide from this point on. The two `#[macro_export]`
// macros still land at the crate root for downstream use regardless.
#[macro_use]
mod macros;

// ── Chip-neutral core (compiles for every chip) ────────────────────────────
unstable_module! {
    /// Busy-wait delay providers (`embedded-hal` `DelayNs`).
    pub mod delay;
}
/// GPIO drivers: type-erased `AnyPin`, typed `Input`/`Output`/`Flex`.
pub mod gpio;
/// Interrupt controller access and IRQ enable/handler registration.
pub mod interrupt;
/// Peripheral singleton tokens and the `Peripherals` struct (`take()`/`steal()`).
pub mod peripherals;
/// Common re-exports for `use hisi_riscv_hal::prelude::*;`.
pub mod prelude;
// Sealed marker traits restricting external trait implementations.
mod private;
/// SoC-specific constants and PAC aliases for the selected chip.
pub mod soc;
/// TCXO always-on clock/counter driver.
pub mod tcxo;
/// Time/frequency newtypes (`Hertz`, durations) used across the HAL.
pub mod time;
/// General-purpose hardware timer driver.
pub mod timer;
/// UART serial driver (blocking and `embedded-io`).
pub mod uart;

// ── WS63-specific / not-yet-ported-to-BS21 drivers ──────────────────────────
// Gated to chip-ws63 for now: they touch WS63-only peripherals (Wi-Fi/RF, the
// full crypto block, SFC) or WS63-specific CRG/clock registers. BS21 ports land
// in later milestones; gating keeps the BS21 build to the milestone-1 subset
// while leaving the WS63 build (the default) byte-identical.
unstable_driver! {
    /// Async runtime helpers: `block_on`, `IrqSignal`, and per-driver `on_interrupt`.
    #[cfg(all(feature = "chip-ws63", feature = "async"))]
    pub mod asynch;
}
// D-cache maintenance (custom HiSilicon CSRs). WS63-only: the cache CSR layout is
// core-specific and tied to the still-unstable DMA/cache-line ownership contract.
unstable_module! {
    /// D-cache maintenance via custom HiSilicon CSRs (clean/invalidate for DMA).
    #[cfg(feature = "chip-ws63")]
    pub mod cache;
}
/// Clock and reset generator metadata: audited peripheral clock-gate bits.
#[cfg(feature = "chip-ws63")]
pub mod clock;
unstable_module! {
    /// System clock setup for firmware that does not boot through flashboot.
    #[cfg(feature = "chip-ws63")]
    pub mod clock_init;
}
// DMA: the register block + the mem-to-mem path are chip-neutral (Dma0 uses the
// chip-neutral PAC base). Peripheral-paced flow control (DmaPeripheral request IDs)
// is chip-ws63-only within the module; BS2X gets mem-to-mem.
unstable_module! {
    /// DMA controllers (Dma0/Sdma0): mem-to-mem and peripheral-paced transfers.
    pub mod dma;
}
/// eFuse one-time-programmable memory access.
#[cfg(feature = "chip-ws63")]
pub mod efuse;
// Chip-neutral: the embassy-time driver reads TCXO_HZ/TIMER_CLOCK_HZ and the
// alarm interrupt from `soc::chip`, and the TCXO/TIMER register blocks are
// register-identical across WS63 and BS2X (verified vs fbb_ws63 / fbb_bs2x).
unstable_driver! {
    /// embassy-time `Driver` implementation backed by TCXO/TIMER for `embassy-executor`.
    #[cfg(feature = "embassy")]
    pub mod embassy;
}
// I2C is a DIFFERENT IP per chip: WS63 has a custom v150 core (i2c.rs), BS2X has a
// Synopsys DesignWare v151 core (i2c_v151.rs). Both are exposed as `hal::i2c`; the
// register blocks come from each chip's PAC (the BS2X v151 layout was rewritten
// into BS2X.svd / bs2x-pac for this).
/// I2C driver (WS63 custom v150 core).
#[cfg(feature = "chip-ws63")]
pub mod i2c;
unstable_module! {
    /// I2C driver (BS2X Synopsys DesignWare v151 core).
    #[cfg(feature = "chip-bs21")]
    #[path = "i2c_v151.rs"]
    pub mod i2c;
}
/// I2S audio interface driver.
#[cfg(feature = "chip-ws63")]
pub mod i2s;
/// Pin mux / pad I/O configuration helpers.
#[cfg(feature = "chip-ws63")]
pub mod io_config;
unstable_module! {
    /// Key management (KM) crypto block driver.
    #[cfg(feature = "chip-ws63")]
    pub mod km;
}
/// Low-speed ADC (v154) driver.
#[cfg(feature = "chip-ws63")]
pub mod lsadc;
// BS2X 13-bit ADC (v153) — chip-bs21-only (WS63's ADC is the different `lsadc`
// v154). bs2x-pac has the correct `gadc` register block; the driver reaches the
// ANA/PMU power sub-blocks (not in the PAC) via raw addresses. See gadc.rs.
unstable_module! {
    /// BS2X 13-bit general-purpose ADC (v153) driver.
    #[cfg(feature = "chip-bs21")]
    pub mod gadc;
}
// BS2X-only HID peripherals (no WS63 analogue): key-matrix scanner + quadrature
// decoder. bs2x-pac has faithful register blocks; see keyscan.rs / qdec.rs.
unstable_module! {
    /// BS2X key-matrix scanner (HID) driver.
    #[cfg(feature = "chip-bs21")]
    pub mod keyscan;
}
// BS2X PDM-mic audio front-end (v150) — config-level (the PCM data path is DMA-fed).
unstable_module! {
    /// BS2X PDM microphone audio front-end (v150) driver.
    #[cfg(feature = "chip-bs21")]
    pub mod pdm;
}
// BS2X USB 2.0 OTG (Synopsys DWC OTG) — config-level (core-ID; full stack deferred).
unstable_module! {
    /// Public-key engine (PKE) crypto accelerator driver.
    #[cfg(feature = "chip-ws63")]
    pub mod pke;
}
unstable_module! {
    /// BS2X quadrature decoder (HID) driver.
    #[cfg(feature = "chip-bs21")]
    pub mod qdec;
}
unstable_module! {
    /// BS2X USB 2.0 OTG (Synopsys DWC OTG) config-level driver.
    #[cfg(feature = "chip-bs21")]
    pub mod usb;
}
// BS2X-enabled shared drivers (`pwm`, `spi`, `wdt`, `ulp_gpio`). These are not
// per-module soft-gated, but the whole `chip-bs21` target is hard-gated by
// `unstable` above until BS2X silicon HIL exists. They build for BS2X because
// (a) the driver code is chip-neutral (it goes through `crate::soc::pac` aliases),
// (b) their peripheral instances are in both `Peripherals` structs, and (c) BS2X
// uses the SAME IP version as WS63 — PWM v151, SPI v151, WDT v151 (the fbb_bs2x
// vs fbb_ws63 `*_regs_def.h` headers are byte-identical bar copyright comments),
// and `ulp_gpio` reuses the GPIO v150 block that regular `gpio` already drives on
// BS2X (blinky). So the register layout is verified, not guessed. What's guaranteed
// today is a register-correct, compiling blocking API.
//
// Still chip-ws63-only (different IP version or no BS2X peripheral): i2c (v150 vs
// BS2X v151), rtc (v100 vs v150), lsadc (v154 vs v153), i2s/sfc (no BS2X instance),
// and the clock/crypto/system stack (deeper port).
/// PWM driver (v151) with typed `PwmPeriod`/`Duty` config.
pub mod pwm;
// RTC is a different IP per chip: WS63 v100 (rtc.rs), BS2X v150 (rtc_v150.rs, a
// 64-bit counter with a coherent-read handshake). Both exposed as `hal::rtc`.
unstable_module! {
    /// Real-time clock driver (WS63 v100).
    #[cfg(feature = "chip-ws63")]
    pub mod rtc;
}
unstable_module! {
    /// Real-time clock driver (BS2X v150, 64-bit counter with coherent-read handshake).
    #[cfg(feature = "chip-bs21")]
    #[path = "rtc_v150.rs"]
    pub mod rtc;
}
unstable_module! {
    /// Functional-safety / lockstep support driver.
    #[cfg(feature = "chip-ws63")]
    pub mod safety;
}
unstable_driver! {
    /// WS63 application-owned RF analog-power and clock preparation.
    #[cfg(feature = "chip-ws63")]
    pub mod rf_power;
}
unstable_module! {
    /// SFC (serial flash controller) driver.
    #[cfg(feature = "chip-ws63")]
    pub mod sfc;
}
unstable_module! {
    /// SPACC symmetric-crypto accelerator driver.
    #[cfg(feature = "chip-ws63")]
    pub mod spacc;
}
/// SPI driver (v151), blocking and `embedded-hal`.
pub mod spi;
/// System control block access and the `System` token.
#[cfg(feature = "chip-ws63")]
pub mod system;
unstable_module! {
    /// WS63 configurable shared-RAM ownership and clock control.
    #[cfg(feature = "chip-ws63")]
    pub mod shared_memory;
}
// TRNG differs per chip: WS63 (trng.rs) vs BS2X v1 (trng_v1.rs). Both `hal::trng`.
/// True random number generator driver (WS63).
#[cfg(feature = "chip-ws63")]
pub mod trng;
unstable_module! {
    /// True random number generator driver (BS2X v1).
    #[cfg(feature = "chip-bs21")]
    #[path = "trng_v1.rs"]
    pub mod trng;
}
/// On-chip temperature sensor driver.
#[cfg(feature = "chip-ws63")]
pub mod tsensor;
unstable_module! {
    /// Ultra-low-power GPIO driver (GPIO v150 block).
    pub mod ulp_gpio;
}
/// Watchdog timer driver (v151).
pub mod wdt;

/// Re-export of the peripheral singleton token struct.
pub use peripherals::Peripherals;
/// Re-export of the system control token.
#[cfg(feature = "chip-ws63")]
pub use system::System;
