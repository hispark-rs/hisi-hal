//! # hisi-riscv-hal — Hardware Abstraction Layer for HiSilicon WS63 (RISC-V).
//!
//! A comprehensive HAL providing safe, idiomatic Rust APIs for all WS63
//! peripherals. Modeled on esp-hal patterns with type-state GPIO, RAII
//! clock guards, DMA typing, and embedded-hal trait implementations.
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
//! let clocks = clock_init::init_clocks(&system.sys_ctl0, &system.cldo_crg);
//! // Now safe to construct peripheral drivers
//! let uart = Uart::new_uart0(peripherals.UART0, Config::default());
//! ```
// `no_std` for firmware builds; `std` is linked under `cfg(test)` so the host
// unit tests can use the standard test harness (run via `cargo test --target x86_64`).
#![cfg_attr(not(test), no_std)]
#![allow(non_camel_case_types)]
#![allow(rustdoc::broken_intra_doc_links)]

// Exactly one chip must be selected (each pulls its PAC + soc/<chip>.rs).
#[cfg(all(feature = "chip-ws63", feature = "chip-bs21"))]
compile_error!("hisi-riscv-hal: select exactly one chip — chip-ws63 (default) OR chip-bs21");
#[cfg(not(any(feature = "chip-ws63", feature = "chip-bs21")))]
compile_error!(
    "hisi-riscv-hal: no chip selected — enable chip-ws63 (default) or chip-bs21 \
     (e.g. --no-default-features --features chip-bs21)"
);

// ── Chip-neutral core (compiles for every chip) ────────────────────────────
pub mod delay;
pub mod gpio;
pub mod interrupt;
pub mod macros;
pub mod peripherals;
pub mod prelude;
pub mod private;
pub mod soc;
pub mod tcxo;
pub mod time;
pub mod timer;
pub mod uart;

// ── WS63-specific / not-yet-ported-to-BS21 drivers ──────────────────────────
// Gated to chip-ws63 for now: they touch WS63-only peripherals (Wi-Fi/RF, the
// full crypto block, SFC) or WS63-specific CRG/clock registers. BS21 ports land
// in later milestones; gating keeps the BS21 build to the milestone-1 subset
// while leaving the WS63 build (the default) byte-identical.
#[cfg(all(feature = "chip-ws63", feature = "async"))]
pub mod asynch;
#[cfg(feature = "chip-ws63")]
pub mod clock;
#[cfg(feature = "chip-ws63")]
pub mod clock_init;
#[cfg(feature = "chip-ws63")]
pub mod dma;
#[cfg(feature = "chip-ws63")]
pub mod efuse;
#[cfg(all(feature = "chip-ws63", feature = "embassy"))]
pub mod embassy;
// I2C is a DIFFERENT IP per chip: WS63 has a custom v150 core (i2c.rs), BS2X has a
// Synopsys DesignWare v151 core (i2c_v151.rs). Both are exposed as `hal::i2c`; the
// register blocks come from each chip's PAC (the BS2X v151 layout was rewritten
// into BS2X.svd / bs2x-pac for this).
#[cfg(feature = "chip-ws63")]
pub mod i2c;
#[cfg(feature = "chip-bs21")]
#[path = "i2c_v151.rs"]
pub mod i2c;
#[cfg(feature = "chip-ws63")]
pub mod i2s;
#[cfg(feature = "chip-ws63")]
pub mod io_config;
#[cfg(feature = "chip-ws63")]
pub mod km;
#[cfg(feature = "chip-ws63")]
pub mod lsadc;
// BS2X 13-bit ADC (v153) — chip-bs21-only (WS63's ADC is the different `lsadc`
// v154). bs2x-pac has the correct `gadc` register block; the driver reaches the
// ANA/PMU power sub-blocks (not in the PAC) via raw addresses. See gadc.rs.
#[cfg(feature = "chip-bs21")]
pub mod gadc;
#[cfg(feature = "chip-ws63")]
pub mod pke;
// BS2X-enabled drivers (ungated below: `pwm`, `spi`, `wdt`, `ulp_gpio`). These were
// chip-ws63-only but build for BS2X too because (a) the driver code is already
// chip-neutral (it goes through `crate::soc::pac` aliases), (b) their peripheral
// instances are in both `Peripherals` structs, and (c) BS2X uses the SAME IP version
// as WS63 — PWM v151, SPI v151, WDT v151 (the fbb_bs2x vs fbb_ws63 `*_regs_def.h`
// headers are byte-identical bar copyright comments), and `ulp_gpio` reuses the GPIO
// v150 block that regular `gpio` already drives on BS2X (blinky). So the register
// layout is verified, not guessed. NB: functional bring-up on BS2X silicon/QEMU is
// still future work (the bs2x QEMU machine doesn't model these yet) — what's
// guaranteed today is a register-correct, compiling blocking API.
//
// Still chip-ws63-only (different IP version or no BS2X peripheral): i2c (v150 vs
// BS2X v151), rtc (v100 vs v150), lsadc (v154 vs v153), i2s/sfc (no BS2X instance),
// and the clock/crypto/system stack (deeper port).
pub mod pwm;
#[cfg(feature = "chip-ws63")]
pub mod rtc;
#[cfg(feature = "chip-ws63")]
pub mod safety;
#[cfg(feature = "chip-ws63")]
pub mod sfc;
#[cfg(feature = "chip-ws63")]
pub mod spacc;
pub mod spi;
#[cfg(feature = "chip-ws63")]
pub mod system;
#[cfg(feature = "chip-ws63")]
pub mod trng;
#[cfg(feature = "chip-ws63")]
pub mod tsensor;
pub mod ulp_gpio;
pub mod wdt;

pub use peripherals::Peripherals;
#[cfg(feature = "chip-ws63")]
pub use system::System;
