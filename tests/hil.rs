//! On-target (semihosting) HIL **driver** tests for the HAL, run on real WS63
//! silicon (and safe under QEMU). These live here — inside the HAL crate's
//! `tests/` — so the HAL's own driver tests ship and run with the HAL and inherit
//! its chip gating (`chip-ws63` default, experimental `chip-bs21` via
//! `--features chip-bs21,unstable`).
//! The cross-cutting CPU/PAC smoke tests live separately in the `tests-hil`
//! crate.
//!
//! Built with
//! `cargo test -p hisi-riscv-hal --no-default-features --features chip-ws63 \
//!      --target riscv32imfc-unknown-none-elf --test hil`
//! and run on real silicon by the patched probe-rs fork via the
//! `hil/embedded-test-runner.sh` cargo runner (see ../../../hil/README.md). Each
//! test is executed in turn over the semihosting channel; the result is reported
//! back to `probe-rs run` (libtest-compatible).
//!
//! ## Why `--test hil` (not a bare `cargo test --target riscv…`)
//!
//! The HAL keeps its host unit tests in `src/*.rs` (`#[cfg(all(test,
//! not(target_arch = "riscv32")))]`), so the crate's *lib* test target uses the
//! default libtest harness — which links the `test`/`std` crates that do NOT
//! exist for the bare-metal `riscv32imfc-unknown-none-elf` target (only
//! core/alloc are shipped). A bare `cargo test --target riscv…` would try to
//! build that lib-test target and fail to link `test`. Scoping to `--test hil`
//! builds ONLY this embedded-test integration target (its own `harness = false`
//! provides `main`), so the on-target build/run never touches the host-only
//! lib-test harness. The host unit tests are unaffected and still run via
//! `cargo test -p hisi-riscv-hal --target x86_64-unknown-linux-gnu`.
//!
//! ## Entry-point interaction with hisi-riscv-rt
//!
//! We do NOT use `hisi_riscv_rt::entry` here. embedded-test exports the C symbol
//! `main` (its test dispatcher), and hisi-riscv-rt's `runtime_init` (the tail of
//! the assembly startup) calls `extern "Rust" fn main()` after BSS-zero/data-copy
//! — so embedded-test's `main` IS the entry. hisi-riscv-rt still supplies the
//! reset vector, the `critical-section-single-hart` impl (backing
//! portable-atomic's RMW polyfill on this no-atomic core), and — via the
//! `boot-header` feature (turned on by the HAL's chip-ws63 feature) — the 0x300
//! image header that makes the ELF bootable. embedded-test also provides the
//! `#[panic_handler]` (it aborts via semihosting), so we must not define one.
//!
//! As an external integration test this still depends on the chip PAC via a
//! cfg-selected alias (`ws63_pac as pac` / `bs2x_pac as pac`) — the HAL keeps its
//! own PAC `pub(crate)`, so the test names the PAC directly for the raw register
//! and singleton facts.
//!
//! The default suite is self-contained: no jumpers / external wiring, safe on a
//! bare board and under QEMU. The opt-in `hil-loopback` cargo feature adds tests
//! that DO require external jumpers — GPIO3↔GPIO5, SPI0 GPIO9↔GPIO11, and UART1
//! GPIO15↔GPIO16 — for validating real on-silicon data paths; run them with
//! `--features chip-ws63,hil-loopback` on a board wired accordingly.
//!
//! The harness stays centralized in this file because `embedded-test` owns the
//! no_std test entrypoint. Per-driver test implementations live in `tests/hil/*.rs`
//! and are called by the small registration wrappers below.

// This is an on-target (RISC-V, semihosting) integration test: it links
// hisi-riscv-rt + the embedded-test harness, both of which are riscv-only
// dev-deps (see Cargo.toml's `[target.'cfg(target_arch = "riscv32")']` block).
// `cargo test` builds every integration-test target for whatever target is being
// tested, so on the host (`cargo test --target x86_64`, the HAL's lib unit-test
// run) this file would otherwise fail to find those crates. Gate the whole file
// to riscv so on the host it compiles to a trivial harness-less binary (an empty
// `main`, required because this `[[test]]` sets `harness = false`) and the host
// unit-test build is unaffected.
#![cfg_attr(target_arch = "riscv32", no_std)]
#![cfg_attr(target_arch = "riscv32", no_main)]

// On the host (`cargo test --target x86_64`, the HAL's lib unit-test run) this
// integration test target is built too. Its real body links hisi-riscv-rt + the
// embedded-test harness, both riscv-only dev-deps — so on the host we compile to
// a no-op `main` instead (this `[[test]]` is `harness = false`, so cargo expects
// a `main` rather than libtest's). It runs nothing; the on-target cases only ever
// run on a real WS63 board via `probe-rs run`.
#[cfg(not(target_arch = "riscv32"))]
fn main() {}

// Pull in hisi-riscv-rt so its startup, reset vector, linker scripts and
// critical-section impl are linked even though we never name a symbol from it.
#[cfg(target_arch = "riscv32")]
use hisi_riscv_rt as _;

#[cfg(all(test, target_arch = "riscv32", feature = "chip-bs21"))]
use bs2x_pac as pac;
#[cfg(all(test, target_arch = "riscv32"))]
use hisi_riscv_hal as hal;
#[cfg(all(test, target_arch = "riscv32", feature = "chip-ws63"))]
use ws63_pac as pac;

#[cfg(all(test, target_arch = "riscv32"))]
#[path = "hil/clock.rs"]
mod clock;
#[cfg(all(test, target_arch = "riscv32"))]
#[path = "hil/dma.rs"]
mod dma;
#[cfg(all(test, target_arch = "riscv32"))]
#[path = "hil/efuse.rs"]
mod efuse;
#[cfg(all(test, target_arch = "riscv32"))]
#[path = "hil/gadc.rs"]
mod gadc;
#[cfg(all(test, target_arch = "riscv32"))]
#[path = "hil/gpio.rs"]
mod gpio;
#[cfg(all(test, target_arch = "riscv32"))]
#[path = "hil/i2c.rs"]
mod i2c;
#[cfg(all(test, target_arch = "riscv32"))]
#[path = "hil/i2s.rs"]
mod i2s;
#[cfg(all(test, target_arch = "riscv32"))]
#[path = "hil/irq.rs"]
mod irq;
#[cfg(all(test, target_arch = "riscv32"))]
#[path = "hil/lsadc.rs"]
mod lsadc;
#[cfg(all(test, target_arch = "riscv32"))]
#[path = "hil/peripherals.rs"]
mod peripherals;
#[cfg(all(test, target_arch = "riscv32"))]
#[path = "hil/pwm.rs"]
mod pwm;
#[cfg(all(test, target_arch = "riscv32"))]
#[path = "hil/rtc.rs"]
mod rtc;
#[cfg(all(test, target_arch = "riscv32"))]
#[path = "hil/spi.rs"]
mod spi;
#[cfg(all(test, target_arch = "riscv32"))]
#[path = "hil/system.rs"]
mod system;
#[cfg(all(test, target_arch = "riscv32"))]
#[path = "hil/tcxo.rs"]
mod tcxo;
#[cfg(all(test, target_arch = "riscv32"))]
#[path = "hil/timer.rs"]
mod timer;
#[cfg(all(test, target_arch = "riscv32"))]
#[path = "hil/trng.rs"]
mod trng;
#[cfg(all(test, target_arch = "riscv32"))]
#[path = "hil/tsensor.rs"]
mod tsensor;
#[cfg(all(test, target_arch = "riscv32"))]
#[path = "hil/uart.rs"]
mod uart;
#[cfg(all(test, target_arch = "riscv32"))]
#[path = "hil/wdt.rs"]
mod wdt;

// riscv-only: the embedded-test harness + the riscv-only dev-deps (hisi-riscv-rt,
// the chip PAC alias) it names exist only in the riscv graph. On the host this
// whole module is dropped and `main()` above is the (no-op) entry.
#[cfg(all(test, target_arch = "riscv32"))]
#[embedded_test::tests]
mod tests {
    /// `#[init]` runs before every test. It takes the singleton `Peripherals`
    /// once and hands them to each test as shared state — proving the PAC's
    /// critical-section-guarded `take()` (backed by hisi-riscv-rt's
    /// single-hart critical-section impl) works on-target.
    #[init]
    fn init() -> crate::pac::Peripherals {
        crate::pac::Peripherals::take().expect("PAC Peripherals::take() returned None on first call")
    }

    #[test]
    fn read_tcxo_status_register(p: crate::pac::Peripherals) {
        crate::tcxo::read_tcxo_status_register(p);
    }

    #[cfg(feature = "chip-ws63")]
    #[test]
    fn hal_peripherals_construct() {
        crate::peripherals::hal_peripherals_construct();
    }

    #[test]
    fn gpio_output_readback() {
        crate::gpio::gpio_output_readback();
    }

    #[test]
    fn tcxo_counter_monotonic() {
        crate::tcxo::tcxo_counter_monotonic();
    }

    #[test]
    fn tcxo_counter64_monotonic() {
        crate::tcxo::tcxo_counter64_monotonic();
    }

    #[test]
    fn timer_counter_advances() {
        crate::timer::timer_counter_advances();
    }

    #[cfg(all(feature = "chip-ws63", feature = "unstable"))]
    #[test]
    fn dma_mem_to_mem() {
        crate::dma::dma_mem_to_mem();
    }

    #[cfg(all(feature = "chip-ws63", feature = "unstable"))]
    #[test]
    fn dma_transfer_guard() {
        crate::dma::dma_transfer_guard();
    }

    #[cfg(feature = "chip-ws63")]
    #[test]
    fn clock_gate_uart0_enabled() {
        crate::clock::clock_gate_uart0_enabled();
    }

    #[cfg(feature = "chip-ws63")]
    #[test]
    fn system_reset_reason_valid() {
        crate::system::system_reset_reason_valid();
    }

    #[test]
    fn uart0_divider_config() {
        crate::uart::uart0_divider_config();
    }

    #[cfg(feature = "chip-ws63")]
    #[test]
    fn uart0_boot_clock_divider_config() {
        crate::uart::uart0_boot_clock_divider_config();
    }

    #[cfg(feature = "chip-ws63")]
    #[test]
    fn uart0_write_and_flush() {
        crate::uart::uart0_write_and_flush();
    }

    #[test]
    fn trng_produces_entropy() {
        crate::trng::trng_produces_entropy();
    }

    #[test]
    fn trng_fill_bytes_produces_data() {
        crate::trng::trng_fill_bytes_produces_data();
    }

    #[cfg(feature = "chip-ws63")]
    #[test]
    fn efuse_read_byte0_ok() {
        crate::efuse::efuse_read_byte0_ok();
    }

    #[cfg(feature = "chip-ws63")]
    #[test]
    fn tsensor_reads_in_range() {
        crate::tsensor::tsensor_reads_in_range();
    }

    #[cfg(feature = "chip-ws63")]
    #[test]
    fn i2c0_scl_config() {
        crate::i2c::i2c0_scl_config();
    }

    #[cfg(feature = "chip-ws63")]
    #[test]
    fn i2c0_rejects_invalid_7bit_address() {
        crate::i2c::i2c0_rejects_invalid_7bit_address();
    }

    #[cfg(feature = "chip-ws63")]
    #[test]
    fn pwm_configure_and_enable() {
        crate::pwm::pwm_configure_and_enable();
    }

    #[cfg(feature = "chip-ws63")]
    #[test]
    fn pwm_set_duty_cycle_rejects_out_of_range() {
        crate::pwm::pwm_set_duty_cycle_rejects_out_of_range();
    }

    #[cfg(feature = "chip-ws63")]
    #[test]
    fn wdt_configure_saturates_load() {
        crate::wdt::wdt_configure_saturates_load();
    }

    #[cfg(feature = "chip-ws63")]
    #[test]
    fn wdt_counter_value_and_feed() {
        crate::wdt::wdt_counter_value_and_feed();
    }

    #[cfg(feature = "chip-ws63")]
    #[test]
    fn wdt_drop_disables_unless_armed() {
        crate::wdt::wdt_drop_disables_unless_armed();
    }

    #[cfg(feature = "chip-ws63")]
    #[test]
    fn wdt_leak_keeps_watchdog_armed() {
        crate::wdt::wdt_leak_keeps_watchdog_armed();
    }

    #[cfg(feature = "chip-ws63")]
    #[test]
    fn i2s_version_live() {
        crate::i2s::i2s_version_live();
    }

    #[cfg(feature = "chip-ws63")]
    #[test]
    fn lsadc_scan_config() {
        crate::lsadc::lsadc_scan_config();
    }

    #[cfg(all(feature = "chip-ws63", feature = "hil-rtc", feature = "unstable"))]
    #[test]
    fn rtc_counter_advances() {
        crate::rtc::rtc_counter_advances();
    }

    #[cfg(feature = "chip-bs21")]
    #[test]
    fn gadc_register_liveness() {
        crate::gadc::gadc_register_liveness();
    }

    #[cfg(all(feature = "chip-ws63", not(feature = "async"), not(feature = "embassy")))]
    #[test]
    fn timer_int0_named_routing() {
        crate::irq::timer_int0_named_routing();
    }

    #[cfg(all(feature = "chip-ws63", feature = "async", feature = "hil-loopback", feature = "unstable"))]
    #[test]
    fn gpio_int0_named_routing() {
        crate::irq::gpio_int0_named_routing();
    }

    #[cfg(all(feature = "chip-ws63", feature = "hil-loopback"))]
    #[test]
    fn gpio_loopback_0_to_3() {
        crate::gpio::gpio_loopback_0_to_3();
    }

    #[cfg(all(feature = "chip-ws63", feature = "hil-loopback"))]
    #[test]
    fn spi0_loopback_mosi_to_miso() {
        crate::spi::spi0_loopback_mosi_to_miso();
    }

    #[cfg(all(feature = "chip-ws63", feature = "hil-loopback", feature = "unstable"))]
    #[test]
    fn spi_dma_tx_loopback() {
        crate::spi::spi_dma_tx_loopback();
    }

    #[cfg(all(feature = "chip-ws63", feature = "hil-loopback", feature = "unstable"))]
    #[test]
    fn spi_dma_fullduplex_loopback() {
        crate::spi::spi_dma_fullduplex_loopback();
    }

    #[cfg(all(feature = "chip-ws63", feature = "hil-loopback"))]
    #[test]
    fn uart1_loopback_tx_to_rx() {
        crate::uart::uart1_loopback_tx_to_rx();
    }

    #[cfg(all(feature = "chip-ws63", feature = "hil-loopback", feature = "async", feature = "unstable"))]
    #[test]
    fn spi_dma_irq59_fires_on_completion() {
        crate::spi::spi_dma_irq59_fires_on_completion();
    }

    #[cfg(all(feature = "chip-ws63", feature = "hil-loopback", feature = "async", feature = "unstable"))]
    #[test]
    fn spi_dma_write_async() {
        crate::spi::spi_dma_write_async();
    }
}
