//! Per-chip SoC description + the chip-neutral PAC alias.
//!
//! This HAL supports a family of HiSilicon "HimiDeer" RISC-V SoCs. Exactly one
//! chip is selected at compile time via a cargo feature:
//! - `chip-ws63` (default) — WS63 (Wi-Fi6 + BLE + SLE), `ws63-pac`
//! - `chip-bs21` — BS21/BS2X (BLE + SLE/NearLink), `bs2x-pac`
//!
//! Drivers never name a vendor PAC or a chip module directly; they go through the
//! chip-neutral aliases [`pac`] (the active Peripheral Access Crate) and [`chip`]
//! (the active `soc/<chip>.rs` description: memory map, clock constants, instance
//! counts, the `Interrupt` enum). Adding a chip = a new `soc/<chip>.rs` + a PAC.

#[cfg(feature = "chip-bs21")]
pub mod bs21;
#[cfg(feature = "chip-ws63")]
pub mod ws63;

#[cfg(feature = "chip-bs21")]
pub use bs21 as chip;
/// The active chip's SoC description (`soc/ws63.rs` or `soc/bs21.rs`).
#[cfg(feature = "chip-ws63")]
pub use ws63 as chip;

#[cfg(feature = "chip-bs21")]
pub(crate) use bs2x_pac as pac;
/// The active chip's Peripheral Access Crate (`ws63_pac` or `bs2x_pac`).
#[cfg(feature = "chip-ws63")]
pub(crate) use ws63_pac as pac;

/// WS53 application-core integration facts.
#[cfg(feature = "chip-ws53")]
pub mod ws53;
#[cfg(feature = "chip-ws53")]
pub use ws53 as chip;
#[cfg(feature = "chip-ws53")]
pub(crate) use ws53_pac as pac;
