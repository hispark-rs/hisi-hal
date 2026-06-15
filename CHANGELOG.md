# Changelog

All notable changes to this project are documented here.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

- **HIL suite**: five more on-target tests (`tests/hil.rs`), all passing on real
  WS63 silicon — `i2c0_scl_config` (I2C SCL divider off the 24 MHz TCXO),
  `pwm_configure_and_enable` (CKEN gate + 16-bit period latch),
  `wdt_configure_saturates_load` (validates the WDT load u64-saturation fix:
  300 s clamps to `WDT_MAX_LOAD`), `i2s_version_live` (I2S block clocked +
  register-map liveness, silicon version 0x13), and `lsadc_scan_config`
  (validates the LSADC register-map fix). The default silicon suite is now
  **17/17**. Plus an opt-in `gadc_register_liveness` (gated `chip-bs21` — GADC is
  BS2X-only; WS63 uses the LSADC) and an opt-in `rtc_counter_advances` behind a
  new **`hil-rtc`** feature (the common WS63 EVB does not populate the RTC's
  32.768 kHz crystal, so touching the RTC stalls the bus / drops the debug link;
  enable only on a board that has the crystal).

## [0.4.0] - 2026-06-15

This is the first silicon-validated HAL release: every driver below was brought
up and verified on real WS63 hardware (12 HIL driver tests + a jumper loopback
group, all passing). It requires `ws63-pac` 0.2 for the SPI/TIMER register fixes,
so it is a minor (0.x-breaking) bump.

### Fixed

- **timer**: `current_value()` now performs the TIMER_V150 `cnt_req`/`cnt_lock`
  snapshot handshake before reading `current_value0` (a latched register, not a
  live counter). Without it both reads returned the same stale latch — the timer
  appeared frozen on silicon (QEMU exposes a live counter, hiding the bug).
  **Silicon-validated**: `timer_counter_advances` now passes (hisi-riscv-rs#10).
- **wdt**: `configure()` saturates the load field in `u64` before narrowing to
  `u32`. The old `((cycles >> RESEV) as u32).min(MAX)` truncated (wrapped) a
  multi-hour timeout to a bogus small load instead of clamping. Caught by a new
  property test.
- **sfc**: `configure_timing()` masks `tshsl` to the 4-bit field *before* applying
  the `MIN_TSHSL` floor. The old order (floor then mask) let a value whose low
  nibble was below the floor (e.g. 16/32/48) be masked back to 0, defeating the
  minimum. Caught by a new property test.
- **dma**: `configure_channel()` now actually **starts** the transfer by setting
  the channel's bit in the global `dmac_en_chns` register (the DesignWare ChEnReg)
  — on silicon the per-channel CFG `ch_enable` bit alone does not kick the engine
  (it does in QEMU). Completion is the hardware auto-clearing that bit, matching
  the vendor `hal_dma_v151_is_enabled`. **Silicon-validated end-to-end**: the HIL
  `dma_mem_to_mem` mem→mem test now passes (was `#[ignore]`'d). QEMU's divergent
  start/complete path is tracked as hisi-riscv-qemu#5 (QEMU is not the reference).
- **spi**: SPI transfers no longer time out on silicon. Root cause was a `ws63-pac`
  `SPI_WSR` bit-layout bug (the v151 silicon places `txfnf` at bit 11, not bit 1),
  so the driver's `txfnf` poll watched a reserved always-0 bit. Fixed in the PAC;
  the HAL polls the same `txfnf`/`rxfne`/`txfe`/`busy` field names unchanged.
  **Silicon-validated**: the HIL `spi0_loopback_mosi_to_miso` MOSI→MISO test now
  round-trips a 4-byte buffer (was `Err(Timeout)`).
- **gpio**: `init_input()` now explicitly asserts the pad input-enable (IE, bit 11)
  via `apply_pull` instead of relying on the boot reset default. On WS63 the ROM
  leaves IE = 1 (measured: `pad_gpio_03_ctrl` = 0x800 at entry) and the vendor
  pinctrl never writes it (`CONFIG_PINCTRL_SUPPORT_IE` undefined), so reads already
  worked — but a pad whose IE was cleared by an earlier mux would have read a dead
  buffer. Setting IE is the same hardware state the vendor relies on, just
  self-contained. **Silicon-validated**: the HIL `gpio_loopback_0_to_3` GPIO0→GPIO3
  test reads the driven input high and low through the plain HAL path.

### Added

- **uart**: `Config::clock_hz: Option<u32>` (a per-instance baud-base override)
  plus `soc::chip::uart_boot_clock_hz()` and `UART_BOOT_CLOCK_{24M,40M}_HZ`.
  Examples that skip the (XIP-unsafe) `clock_init` run on flashboot's raw-TCXO
  console clock, not the 160 MHz PLL — **confirmed 40 MHz on silicon** (HW_CTL
  TCXO strap; `div_fra = 44` ⟺ 40 MHz), resolving the boot-baud root cause of
  #15/#10. Default `None` keeps the 160 MHz post-`clock_init` behaviour.
- **`cache`** module (WS63): `clean_range` / `invalidate_range` / `flush_range`
  D-cache maintenance via the HiSilicon custom CSRs (`DCINCVA`/`DCMAINT`). The
  RV32 core's D-cache is non-coherent with the DMA master, so a mem→mem transfer
  must clean the source and invalidate the destination — now done by the HIL DMA
  test (no-ops on the host build).

### Changed

- **deps**: bump the `ws63-pac` dependency requirement to `0.2` (was `0.1`). The
  SPI_WSR and TIMER register-layout fixes the silicon bring-up relies on ship in
  ws63-pac 0.2.0; pairing this HAL with the older 0.1.x PAC would reintroduce the
  SPI-timeout / frozen-timer bugs.
- **dma**: `enable_controller()` bypasses the controller's auto clock-gate when it
  has one (the WS63 M_DMA `DMA_CLK_AUTO_CTRL_REG` bit, via the new
  `DmaInstance::CLK_AUTO_CTRL`), so the clock stays on across a transfer. Mirrors
  the vendor `dma_porting`.
- **timer**: `current_value()` uses the now-named `cnt_req`/`cnt_lock` fields
  (added to `ws63-pac`'s `TIMER%s_CONTROL`) instead of raw bit pokes.

### Internal

- Greatly expanded host coverage: new unit + property (`proptest`) tests across
  previously-untested driver modules (gpio, interrupt, io_config, pwm, i2s, uart,
  sfc, wdt, tsensor, rtc, gadc, dma, i2c, clock, tcxo, …) — **305 host tests**
  total, all green. The wdt + sfc fixes above were both found by these new property
  tests; the gpio IE hardening added 3 (`apply_pull` always enables the input
  buffer / keeps unrelated bits). No API change from the test work.
- HIL suite: added `efuse_read_byte0_ok`, `trng_produces_entropy`, and
  `tsensor_reads_in_range` (on-die temperature) — all silicon-validated;
  **12 driver tests, all passing on real WS63 silicon**. Plus an opt-in
  (`hil-loopback` feature) jumper-wired loopback group — `gpio_loopback_0_to_3`
  (GPIO0→GPIO3), `spi0_loopback_mosi_to_miso` (GPIO9→GPIO11),
  `uart1_loopback_tx_to_rx` (GPIO15→GPIO16) — all three passing on silicon.
- Code-review follow-up: the `tcxo` status-register bit values are now named
  consts (`TCXO_EN_BIT`/`TCXO_CLEAR_BIT`/`TCXO_REFRESH_BIT`/`TCXO_VALID_BIT`) that
  both the driver and its property tests use; 4 tautological status-bit unit tests
  (which asserted literals against themselves) were removed.

## [0.3.2] - 2026-06-14

### Fixed

- **uart**: compute the fractional baud divider `div_fra`
  (`(UART_CLK * 4 / baud) & 0x3F`) instead of hardcoding 0 — a real on-wire baud
  error at low clocks. On real WS63 silicon flashboot itself programs
  `div_fra = 44`. Fixes #15.

### Internal

- Add an on-target `embedded-test` integration suite (`tests/hil.rs`, run via
  `cargo test --test hil` + the patched probe-rs fork over semihosting) covering
  the HAL drivers on real WS63 silicon (7 passing, 2 ignored bring-up TODOs).
  Dev-deps + a build.rs linker-script step only; no change to the published API.

## [0.3.1] - 2026-06-12

### Added

- **`embassy` time-driver now supports the BS2X family** (BS21/BS22/BS20), not just
  WS63. The TCXO `now()` and TIMER one-shot alarm are register-identical across the
  family (verified against the fbb_ws63 / fbb_bs2x C SDKs); only the crystal rate and
  the alarm IRQ were chip-specific.
  - New `soc::chip::ALARM_INTERRUPT` (the TIMER-channel-0 interrupt the alarm uses):
    `TIMER_INT0` (IRQ 26, a standard `mie`-bit local interrupt) on WS63, `TIMER_0`
    (IRQ 53, a HiSilicon LOCI custom local interrupt) on BS2X.
  - New `pub const embassy::ALARM_IRQ: u32` so an application's machine-trap handler
    routes the alarm chip-neutrally (`mcause & 0xFFF == ALARM_IRQ`) instead of a literal.

### Changed

- The `embassy` feature **no longer implies `async`**. The embassy-time driver needs
  only `critical-section` + the embassy-time crates; `async` (the `embedded-hal-async`
  driver layer) remains `chip-ws63`-only. This lets `embassy` build on BS2X without
  dragging in the WS63-only async drivers. WS63 is unaffected.

## [0.3.0] - 2026-06-05

### Added

- **Async HAL** (`async` feature): interrupt + waker driven `embedded-hal-async` /
  `embedded-io-async` drivers, runnable on the no-atomics WS63 via the existing
  portable-atomic + critical-section polyfill.
  - `asynch`: a minimal `wfi`-based `block_on` + `IrqSignal` (the const-constructible
    ISR→future bridge) shared by all async drivers.
  - `timer::AsyncDelay`: `embedded_hal_async::delay::DelayNs` on a TIMER one-shot.
  - `gpio`: `embedded_hal_async::digital::Wait` for `Input` (edge/level via the GPIO IRQ).
  - `uart`: `embedded_io_async::{Read, Write}` for Uart0/1/2 (RX via the UART IRQ).
  - `spi`: `embedded_hal_async::spi::SpiBus` for Spi0/1 (FIFO-paced; synchronous loopback on the model).
  - `i2c`: `embedded_hal_async::i2c::I2c` for I2c0/1.
  - `lsadc`: `LsAdc::read_async()` (bespoke; LSADC_INTR = IRQ 72).
  - `dma`: `DmaDriver::<Dma0>::wait_transfer_done()` (bespoke; DMA_INT = IRQ 59).
  - This covers every embedded-hal-async / embedded-io-async trait applicable to WS63
    (DelayNs, Wait, SpiBus, I2c, Read/Write) plus the completion-IRQ peripherals.
  - Drivers expose `on_interrupt` hooks instead of installing ISRs, so enabling the
    feature never changes non-async firmware (safe under workspace feature unification).
- **embassy support** (`embassy` feature): an `embassy-time` `Driver` for WS63 —
  `now()` from the TCXO 64-bit counter, alarms from a TIMER channel +
  `embassy-time-queue-utils`. With `embassy-executor` (platform-riscv32) this runs
  `Timer::after`, multi-task scheduling, and the async drivers above. Validated on
  ws63-qemu (`embassy_multitask`, `embassy_async_io`).
- **Two-stage SPI clock**: `spi.rs::configure_spi_source_clock` programs the CLDO_CRG
  divider (480 MHz PLL → 160 MHz SSI_CLK) and switches the SPI source TCXO→PLL on init
  (mirrors the vendor `spi_porting_clock_init`), so `SCK = SSI_CLK / SCKDV` holds on
  silicon rather than assuming an unconfigured SSI_CLK.
- New `soc::ws63` clock constants: `TCXO_HZ`, `TIMER_CLOCK_HZ`, `UART_CLOCK_HZ`,
  `SPI_CLOCK_HZ`, `I2C_CLOCK_HZ`.

### Fixed

- **Peripheral clocks corrected to the real silicon clocks** (all were wrongly the
  240 MHz CPU/PLL clock; verified against fbb_ws63 `clock_init.c`):
  - Timer & WDT count at the **24 MHz TCXO crystal** (was 240 MHz → ~10× mistiming);
    WDT also gained the missing `>>8` load-field conversion (was doubly wrong/saturated).
  - UART baud base = **160 MHz** PLL; SPI SSI_CLK = **160 MHz** PLL; I2C SCL clock =
    **24 MHz** TCXO crystal (ch2's nominal 80 MHz is the bus-capability figure, not the
    divisor base the SDK uses).
  - The embassy-time alarm conversion uses the 24 MHz timer clock.
- **I2S clock-gate bit** in `Peripheral::cken_info`: corrected to `CKEN_CTL0` bit 12
  (clk) + bit 11 (bus); was wrongly bit 24 (SDK audit vs `sio_porting.c`).

### Changed

- **`Peripheral::cken_info` now returns `Option<(u8, u8)>`** (was `(u8, u8)`) — `None`
  for peripherals the SDK does not individually gate (was a fabricated bit), `Some` for
  the SDK/SVD-confirmed gates (PWM, I2S, UART0/1/2, SPI0). *Breaking* for external
  callers of `cken_info`.

## [0.2.1] - 2026-06-02

### Changed

- CI: first release cut by hisi-riscv-hal's own repo pipeline (no functional change since 0.2.0).

## [0.2.0] - 2026-06-02

### Added

- **SDMA logical channel mapping** (`dma.rs`): Map logical channels 8-11 to physical channels 0-3 with proper request ID dispatch
- **DMA flow control**: Wire flow control configuration for peripheral request IDs per dma_porting.h
- **GPIO pull/interrupt triggers** (`gpio.rs`): Apply pull configuration and interrupt trigger modes (Phase 2)
- **I2C bounded timeouts** (`i2c.rs`): Add timeout bounding to I2C operations preventing indefinite waits
- **System reset** (`system.rs`): Implement real system reset (replacing PLIC-era stubs)
- **Interrupt handling rewrite** (`interrupt.rs`): Rewrite for WS63 custom-CSR local INTC (no PLIC dependency)
- **Test infrastructure**: Unit tests compile and run on host (cargo test --target x86_64)
- **ARCHITECTURE.md**: Document codebase architecture and design patterns

### Changed

- **Interrupt module**: Complete rewrite for WS63 custom CSR controller (no PLIC compatibility)
- **DMA peripheral request IDs**: Corrected to match WS63 C SDK dma_porting.h register definitions
- **SPI driver**: Corrected trsm field handling (TX&RX=0, was incorrectly 0b11 for EEPROM reads), SCKDV divisor calculation, added wait timeouts
- **EFUSE driver**: Corrected register access patterns to match WS63 C SDK behavior
- **LSADC driver**: Corrected register access patterns to match WS63 C SDK behavior
- **Code quality**: Resolved all clippy warnings with -D warnings flag for Rust 1.96

### Fixed

- Removed duplicate `#[inline]` in peripheral! macro
- Suppressed clippy warnings in const_assert! and ptr_aligned! macros
- Registry ws63-pac dependency (unified single PAC import)
- Portable-atomic critical-section polyfill for no-atomic RISC-V ISA
- I2C ACK check, repeated START condition handling
- SPI division-by-zero edge case
- Clock initialization: Accurate boot ROM PLL facts from C SDK
- Serial flush: Made non-blocking in nb::serial trait
- Timer overflow handling in periodic timer mode
- PWM bit field calculations
- Reference-count TOCTOU vulnerability in atomic fetch_add/fetch_sub
- Baud rate clamping edge cases
- Clock drop guard lifecycle issues
- Flex pin state management
- MMIO address assertions in safety.rs
- Code review fixes: 28 bugs across 20 files (includes prior fixes)
- 80 additional unit tests + 14 const_assert checks + 17 proptest fuzz tests
- PERIPHERAL_COUNT off-by-one error
- SPI asymmetry in CS handling

### Removed

- Unused `paste` crate dependency
- Private apply_pull documentation linkage (CI Docs fix)
- Pinned Cargo.lock from repository (CI consistency)

## [0.1.0] - 2026-05-28

### Added

- **WDT driver** (`wdt.rs`): Watchdog timer with lock/unlock, interrupt/reset modes, configurable timeout
- **RTC driver** (`rtc.rs`): Real-Time Clock with 48-bit counter, free-running and periodic modes
- **TCXO driver** (`tcxo.rs`): 64-bit free-running counter with latched reads
- **LSADC driver** (`lsadc.rs`): Low-Speed ADC (12-bit SAR, 6 channels, CIC filter, offset/gain correction)
- **TSENSOR driver** (`tsensor.rs`): Temperature sensor (10-bit code, high/low thresholds, auto-refresh)
- **DMA driver** (`dma.rs`): DMA + SDMA controllers (4 channels each, scatter-gather, burst config)
- **TRNG driver** (`trng.rs`): True Random Number Generator (FRO entropy, FIFO read)
- **I2S driver** (`i2s.rs`): I2S/PCM audio interface (master/slave, 2-8 channels, clock dividers)
- **SFC driver** (`sfc.rs`): SPI Flash Controller (Standard/Dual/Quad SPI, Bus DMA, command operations)
- **IO_CONFIG driver** (`io_config.rs`): Pin mux and pad control (drive strength, pull, Schmitt trigger)
- **EFUSE driver** (`efuse.rs`): eFuse OTP controller (read/write control, status, AVDD control)
- **ULP_GPIO driver** (`ulp_gpio.rs`): Ultra-Low-Power GPIO (8 pins, full embedded-hal digital traits)
- **time module** (`time.rs`): `Instant`, `Duration`, `Rate` types (TCXO-backed)
- **delay module** (`delay.rs`): `Delay` driver implementing `embedded_hal::delay::DelayNs`
- **macros module** (`macros.rs`): `unstable_module!`, `any_peripheral!`, `infallible!`
- **private module** (`private.rs`): Sealed traits (`Sealed`, `DmaWord`, `DriverMode`, `Blocking`, `Async`)
- **SPACC driver** (`spacc.rs`): Security accelerator stub (AES/SM4/LEA/TDES/HASH/HMAC)
- **PKE driver** (`pke.rs`): Public Key Engine stub (RSA/ECC)
- **KM driver** (`km.rs`): Key Management stub (KLAD, keyslot locking)
- `embedded_hal::i2c::I2c` trait for I2C0 and I2C1
- `embedded_hal::spi::SpiBus` trait for SPI1
- `embedded_hal::pwm::SetDutyCycle` trait for PwmChannel
- `embedded_hal_nb::serial::Read/Write` traits for UART0/1/2
- `embedded_io::Read/Write` traits for UART1 and UART2
- `Peripheral` enum and `PeripheralGuard` RAII clock management with reference counting
- `DmaEligible` trait, `DmaChannelFor` trait, `DmaPeripheral` enum for DMA-peripheral binding
- `InputConfig`/`OutputConfig` structs for GPIO pin configuration
- `Flex` pin driver (combined input+output with embedded-hal traits)
- `AnyPin` type-erased GPIO pin
- `InputSignal`/`OutputSignal` types for peripheral interconnect
- `PeripheralOutput`/`PeripheralInput` sealed traits
- `OneShotTimer` and `PeriodicTimer` wrappers
- `embedded_hal::delay::DelayNs` for `OneShotTimer`
- `Delay` driver with `embedded_hal::delay::DelayNs`
- `ResetReason` enum and `software_reset()` in system module
- `Priority` levels, `InterruptConfigurable` trait, `InterruptHandler` struct
- `disable_peripheral()` and `reset_peripheral()` clock control methods
- `read_raw()` and `EfuseStatus::boot_complete()` eFuse methods
- Inline unit tests in `time.rs`

### Changed

- **GPIO**: Added `Input`/`Output`/`Flex` driver types alongside legacy `GpioPin<MODE>`
- **Timer**: Enhanced with `OneShotTimer` and `PeriodicTimer` wrapper types
- **Clock**: Refactored with private `write_cken_bit()` helper, eliminating duplicated register dispatch
- **prelude**: Expanded from 8 to 27 re-exports
- **soc/ws63**: Added `ULP_GPIO_COUNT`, `LSADC_CHANNEL_COUNT`, `TCXO_COUNTER_WIDTH`, `RTC_COUNTER_WIDTH`
- Made `soc` module and `Peripheral::cken_info()` public for testability
- Formatted entire codebase with `cargo fmt`

### Fixed

- `DmaChannelFor` blanket impl removed (provided zero type safety)
- Duplicate `read_chip_id()` removed from efuse (alias for existing `read_control_data()`)
- Clock register dispatch deduplicated (15 methods → shared helper)
