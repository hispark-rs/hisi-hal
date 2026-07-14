# Changelog

All notable changes to this project are documented here.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Changed

- Async GPIO, UART, timer, DMA, LSADC, and Embassy time support no longer
  exports strong device interrupt symbols from the HAL library. Firmware owns
  vector routing and calls the corresponding `on_interrupt` hook, preventing
  Cargo feature unification from installing handlers in unrelated binaries.
- Require `ws63-pac 0.3.0` for the software-interrupt vectors and corrected
  write-only timer EOI model used by the scheduler port. Timer EOI is expressed
  as a complete command write so the BS2X PAC stays compatible.

## [0.7.0-alpha.1] - 2026-07-12

### Changed

- Rename the package from `hisi-riscv-hal` to `hisi-hal` and the Rust crate
  path from `hisi_riscv_hal` to `hisi_hal`. The stable API and feature surface
  are otherwise identical to the 0.6 line; CI compares normalized public API
  snapshots to enforce rename-only parity.

## [0.6.1] - 2026-07-12

### Documentation

- Add the package/repository migration guide for the upcoming `hisi-hal`
  package. The 0.6 stable API, feature semantics, and implementation are
  unchanged from 0.6.0.

## [0.6.0] - 2026-07-12

The first stable release of the 0.6 API line. Its default WS63 public surface,
feature semantics, and silicon evidence are unchanged from `0.6.0-rc.1`:
30/30 stable HIL tests and 32/32 tests with `unstable` passed on real WS63.
DMA, Embassy, RF helpers, BS2X, and other ungraduated surfaces remain opt-in
experimental APIs.

## [0.6.0-rc.1] - 2026-07-12

First release candidate for the 0.6.0 stable-API cycle. This release freezes
the default WS63 public surface; the RF bring-up helpers below remain behind
`unstable` and do not widen the 0.6 stable API.

### Added

- Add the unstable WS63 `SharedMemory` driver for the official default Wi-Fi
  packet-RAM/ITCM/DTCM bank configuration.
- Add unstable typed RF power sequencing, factory XO-trim recovery, externally
  owned eFuse reads, and whole-cache flush support for the connectivity runtime.
- Commit a generated 0.6 stable API snapshot and check it in CI with
  `cargo-public-api`; stable-only rustdoc also rejects links to gated items.

### Changed

- Pin standalone builds to the official `nightly-2026-07-09` toolchain and make
  CI compile, lint, and document the real `riscv32imfc-unknown-none-elf` target
  with `-Zbuild-std=core,alloc` instead of accepting a host-only build.
- Require `ws63-pac 0.2.2` for the RF, shared-RAM, XO-trim, and ROM-patch
  register definitions used by the HAL.
- Expand the self-contained WS63 embedded-test suite to 30 stable tests and 32
  tests with `unstable`; both matrices passed on real silicon on 2026-07-12.

### Fixed

- Align WS63 I2C v150 polling with the vendor SDK: initialize non-FIFO polling
  mode, expose all command status bits, wait for `int_done`, then inspect
  `int_ack_err`, and clear the transfer status as one W1C group. A silicon HIL
  test covers the reserved-address NACK completion path.
- Make `chip-ws63` select the `critical-section` dependency needed by the eFuse
  implementation even when the HAL's `rt` feature is disabled, restoring the
  documented embedded-test build contract.
- Read UART divisor aliases under `UART_CTL.div_en` in HIL and restore the line
  control register afterwards, so the test observes the divisor latch rather
  than the shared TX/RX data register.

## [0.6.0-alpha.2] - 2026-07-09

Second pre-release for the 0.6.0 stabilization cycle. This is a small cleanup
release on top of `0.6.0-alpha.1`; it does not widen the stable API surface.

### Fixed

- Avoid a private intra-doc link in the GADC documentation so rustdoc/docs.rs can
  render without a private-item link warning.
- Silence dead-code warnings for the unstable timer `DelayNs` saturating helper
  functions when building the default stable surface with `unstable` disabled.
- Update package metadata comments to reflect the official nightly +
  `-Zbuild-std=core,alloc` toolchain path instead of the retired custom
  `hisi-riscv` tarball.

## [0.6.0-alpha.1] - 2026-07-09

Pre-release for the 0.6.0 stabilization cycle. This publishes the current
stable/unstable gating, typed-config tightening, register-access cleanup, and
expanded HIL evidence so downstream users can test the new shape before the
final 0.6.0 release.

### Added

- **HIL default driver suite expanded to 25 self-contained WS63 tests, all passing
  on silicon** via `probe-rs run` + `embedded-test`. New coverage: UART boot-clock
  divider and TX flush, I2C invalid 7-bit address rejection, PWM `SetDutyCycle`
  out-of-range error, TCXO 64-bit counter, TRNG byte fill path, and WDT counter/feed
  liveness.

### Changed (BREAKING)

- **Stable/unstable API gating**: a new `unstable` cargo feature (OFF by default)
  gates every surface without an on-silicon HIL test. Previously-`pub` items are now
  `pub(crate)` (soft-gate) or absent when `unstable` is off. Uses `instability` 0.3.12
  proc-macro (MSRV 1.85 → 1.88). `unstable_module!`/`unstable_driver!` macros
  rewritten in esp-hal crate-local form. See `docs/src/reference/10-stable-api.md`
  for the full inventory.
- **BS2X target requires `unstable`**: the entire `chip-bs21` target is now
  experimental — no BS2X silicon HIL has run yet.
- **`embassy` now requires `unstable`**: the `embassy` feature gate alone no longer
  exposes the embassy module; you need both `embassy` and `unstable` until the
  integration passes end-to-end HIL.
- **Typed UART clock**: `Config::clock_hz: Option<u32>` replaced with
  `Config::clock: UartClock` (`Pll` or `Boot`).
- **GPIO**: removed no-op `OutputConfig::open_drain` / `with_open_drain`.
- **I2C**: `addr > 0x7f` now returns `I2cError::InvalidAddress`.
- **PWM**: `SetDutyCycle` returns `PwmError::DutyOutOfRange` above `max_duty_cycle()`.

### Changed

- **hisi-riscv-rt** dependency bumped to `0.5` (runtime adapter architecture release).
- **docs.rs** now requires `unstable` + nightly for "requires unstable" markers.

### UNSTABLE (gated behind `unstable` feature — add to your `Cargo.toml` to restore)

- **`dma` module**: `Dma0`/`Sdma0`, `DmaDriver`, typed channel tokens, mem-to-mem
  `Transfer`, `SpiDma`/`UartDma`. Individual HIL tests exist, but safe-DMA invariants
  (cache-line, timeout, cancellation) are not yet closed.
- **Async interrupt/waker helpers**: `asynch::block_on`, `IrqSignal`, GPIO `Wait`,
  timer `AsyncDelay`, UART async I/O, LSADC async.
- **Scoped knobs**: `EfuseDriver::set_clock_period`/`read_buffer`/`write_byte`,
  `System::software_reset*`, `Instant::now`/`elapsed`, interrupt priority/threshold,
  SFC pad config, broad I2S/LSADC/TSENSOR/TRNG methods.
- **Drivers without HIL**: `clock_init`, `km`, `pke`, `safety`, `sfc`, `spacc`,
  `ulp_gpio`, `rtc`-WS63, `delay`.
- **`prelude`**: `Delay`, `Dma0`/`DmaDriver`/`Sdma0`, `RtcDriver`, `SfcDriver`,
  `UlpGpioPin`.

### Fixed

- `dma_mem_to_mem` HIL test now `#[cfg(feature="chip-ws63")]` (was broken on BS2X).
- `UartDma` doc comment corrected (write_dma is not silicon-verified).


## [0.5.1] - 2026-06-30

### Added

- **Peripheral-paced DMA (SPI)** — the deferred Area C (hisi-riscv-hal#6) lands for
  SPI, silicon-validated on real WS63:
  - `DmaDriver::start_mem_to_peripheral` / `start_peripheral_to_mem` + the
    `PeripheralTransfer<'d, BUF>` guard (owns driver + channel + the single memory
    buffer; `wait()` returns `Result` — a wedged channel is `Err(Timeout)`, not
    silently "done"). `Drop` runs cancel-then-quiesce (clear the peripheral
    DMA-enable first, then halt → drain `active` → disable).
  - `DmaChannel` / `DmaChannels` typed tokens + `DmaDriver::split_channels()` — a
    runtime-claimed channel-ownership bitmask (no-atomics-safe). At most one
    in-flight transfer per channel.
  - `DmaFrame` (Byte/HalfWord) + `PeriKind`/`PeriDmaCtl` (a POD teardown handle that
    clears the peripheral DMA-enable, so the guard needn't be generic over the driver).
  - `Spi::with_dma(self, DmaDriver) -> SpiDma` (consumes the blocking `Spi` — blocking
    + DMA APIs are mutually exclusive, esp-hal style). `SpiDma::write_dma` (TX-only,
    drains looped-back RX) and `SpiDma::transfer_dma` (full-duplex, two channels);
    both blocking, bounded-wait, with the vendor handshake order
    (watermark → clean → start → set `spi_dcr.tdmae`). `SpiDma::release`.
  - `SpiError::BufferTooLong` for the >4095-beat `trans_size` cap (no silent truncation).
  - HIL `spi_dma_tx_loopback` + `spi_dma_fullduplex_loopback` — both PASS on silicon.
- **`UartDma`** (`Uart::with_dma`/`write_dma`/`read_dma`/`release`) — the UART DMA
  ergonomic API (P3). The register sequence compiles and is correct, but loopback
  data-correctness is **blocked by hisi-riscv-hal#5** (UART1 TX shift register
  doesn't advance on this board); silicon round-trip deferred to a #5-fixed board.

### Changed

- **`DmaDriver`: `Transfer::wait` (mem-to-mem) now quiesces on timeout** (halt → drain
  `active` → disable) before returning the buffers — was handing back a live channel
  (a latent UAF on a wedged transfer). `Transfer::drop` gains the same `active`-bit
  settle (clearing `ch_enable` mid-burst could let an outstanding bus write land after
  the buffer is freed). Both backported from the peripheral-DMA design review.

### Notes

- The SPI1 handshake (QSPI0_2CS 9/10 vs the legacy SPI_MS1 13/14) is **silicon-
  unverified** and intentionally NOT changed in 0.5.1 — `DmaPeripheral::Spi1Tx/Rx`
  remain 13/14. The correct `Qspi02csTx/Rx` variant is deferred to when SPI1 DMA is
  wired and silicon-verified (adding it now would be a breaking enum change for an
  unverified claim).
- Pure SPI RX-only `read_dma` is descoped to 0.5.2 (can't loopback-test on a
  single-SPI0 board with no external master).
- Async `.await` DMA variants (P4) are not in 0.5.1 — deferred to 0.5.2.

## [0.5.0] - 2026-06-16

### Changed

- **pwm** (BREAKING): `PwmChannel::configure` now takes a validated
  `PwmPeriod` + `Duty` instead of `(freq: u32, duty_percent: u8)`, so a config the
  hardware cannot run is **unrepresentable** rather than silently wrong:
  - `Duty::from_percent` rejects `> 100 %`; `PwmPeriod::from_count`/`try_from_hz`
    reject a 0 period and (because WS63 silicon does **not** latch the `pwm_freq_h`
    high half — measured, even with the full clock tree up) a period that exceeds
    16 bits. `PwmPeriod` is a `u16` accordingly. New `PWM_CLOCK_HZ`
    (`SYSTEM_CLOCK_HZ / 6`, the vendor high-freq ÷6 divider).
  - `configure` now **brings up the PWM clock tree itself** (CLK_SEL + CKEN_CTL0
    `[10:2]` + DIV_CTL3 divider, per vendor `pwm_port_clock_enable`) — the
    precondition the old API silently assumed, without which the registers don't
    latch. WS63-gated; a no-op on BS2X (different clock tree — follow-up).
  - The `embedded_hal::pwm::SetDutyCycle` impl is unchanged in shape (the
    operational layer stays standard `u16` + `Result`); `max_duty_cycle()` now
    reflects the configured period instead of `u16::MAX`.
  - **Silicon-validated**: the HIL `pwm_configure_and_enable` test programs a
    24 000-tick / 50 % waveform and confirms `pwm_freq_l0` = 24 000, `pwm_freq_h0`
    = 0, duty = 12 000.
- **Typed config across all drivers** (BREAKING) — the "if it compiles, it runs on
  silicon" pass: every writable config value is now one the hardware can actually
  run (no silent clamp / truncate / unclocked path). Per driver:
  - **spi**: `Config.frequency: SpiHz` (rejects an SCK outside
    `[SPI_CLOCK_HZ/0xFFFE, /2]`) and `data_bits: DataBits` (validated `4..=16`),
    replacing the old `u32`/`u8` that silently clamped the SCKDV divider.
  - **i2c**: `new_i2c0/1(speed: Speed)` (`Standard`/`Fast` = 100/400 kHz, aligned
    with the BS2X v151 core) instead of a raw frequency that could overflow the SCL
    counter.
  - **uart**: `Config.baudrate: BaudRate` — `BaudRate::try_new` rejects a baud whose
    16-bit divider would under/overflow against the 160 MHz base, removing the
    silent low-clamp inside `configure_uart`.
  - **i2s** (rewrite): role is now type-state — `I2sDriver::new_master(&MasterConfig)`
    / `new_slave(&SlaveConfig)` (fusing the old `new()+configure()`). A master derives
    its BCLK/FS dividers from `(data_width, channels)` exactly as the vendor
    `sio_porting` does, so a **zero-divider master is unrepresentable** and every
    divider provably fits its field. Register bits (`mode`/`i2s_crg`/`data_width_set`)
    and the `DataWidth` (16/18/20/24/32) + `ChannelCount` (2/4/8/16) enums are
    re-tabled to the authoritative vendor `hal_sio_v151_regs_def.h` (the ws63-pac SVD
    layout was fabricated). `new_master` self-enables the I2S clock tree (CMU + CKEN
    bus/clk).
  - **wdt**: `configure` takes a validated `WdtTimeout` (`from_ms` rejects 0 /
    `> MAX_MS`) and returns `Result<(), WdtError>`; `counter_value` → `Result`. The
    old silent saturation to `WDT_MAX_LOAD` is gone, and the counter-latch poll is
    now bounded (`WdtError::Busy`).
  - **timer**: `start_micros`/`start_millis` return `Result<(), TimerError>` and
    reject `TicksOverflow` instead of clamping a too-long duration to `u32::MAX`
    (≈178 s). The blocking `DelayNs` impl keeps saturating semantics (the trait has
    no error channel).
  - **gadc**: `read` → `Result<i32, GadcError>` with a **bounded** conversion-done
    poll (`ConversionTimeout`) instead of a `while {}` that hangs on an unpowered
    AFE; `new` documents the analog preconditions.
  - **lsadc**: the silently-`& mask`-truncated `sample_cnt` / `cast_cnt` /
    `rxintsize` fields are now validated `SampleCount` / `CastCount` /
    `FifoWaterline` newtypes.
  - **rtc**: the 32.768 kHz-crystal board precondition is documented (both the WS63
    v100 and BS2X v150 modules); reads use a bounded latch poll.
  - **sfc**: `command_with_data` rejects a `> 64`-byte write with
    `SfcError::BufferTooLong` instead of silently truncating it.
- **Drop-to-disable** (BREAKING semantics) — hazardous peripherals return to a safe
  state when their handle drops, touching only the peripheral's own enable bit:
  `Watchdog` stops, `PwmChannel` clears its `pwm_enN`, `Output` reverts to input /
  high-Z. Escape hatches keep them live: `Watchdog::into_armed()`/`leak()`,
  `PwmChannel::into_running()`, `Output::into_latched()`/`into_flex()` (each → a
  zero-sized marker). `Timer` is deliberately exempt.
- **chip-feature de-default** (BREAKING): the HAL no longer defaults to `chip-ws63`
  (esp-hal style). `default = ["rt", "dep:critical-section"]`; building the HAL
  standalone now requires an explicit `chip-ws63` *or* `chip-bs21` (a `compile_error!`
  guides you otherwise). `cargo build`/`cargo check --workspace` still work via
  feature unification from the default-member ws63 examples. `[package.metadata.docs.rs]`
  pins the riscv target + chip-ws63.
- **API cleanup** (BREAKING): removed the legacy type-state `GpioPin<MODE>` (and
  `create_input_pin`/`create_output_pin`, `InputMode`/`OutputMode`) — use
  `AnyPin::init_input`/`init_output`/`init_flex`. All `*Error` enums are
  `#[non_exhaustive]` + `defmt::Format`-derivable; `SpiError::Overflow` maps to
  `ErrorKind::Overrun`; the BS2X `i2c_v151::I2cError` implements
  `embedded_hal::i2c::Error`. `io_config::get_gpio_mux` → `gpio_mux` (C-GETTER).
  Added a safe `Peripheral::reborrow(&mut self)`. New optional `defmt` feature.
- **docs**: `#![warn(missing_docs)]` enabled and every public item documented (flips
  to `deny` once green).
- **interrupt routing → device.x named handlers** (BREAKING; requires
  `hisi-riscv-rt 0.4`): with the `async` feature, each driver now exports the
  rt-named handler symbol its IRQ vectors to (`TIMER_INT0..2`, `UART0..2_INT`,
  `GPIO_INT0..2`, `DMA_INT`, `LSADC_INTR`), and that handler calls the driver's
  static `on_interrupt`. rt 0.4 runs `mtvec` in **direct mode** and dispatches a
  custom IRQ through its `__INTERRUPTS` table to these named symbols — so an enabled
  IRQ is delivered to the right driver with no app-side `mcause` trap shim. Bumps the
  `hisi-riscv-rt` dev-dep `0.3 → 0.4`. (Area D / #7; silicon-validated by the HIL
  `timer_irq_direct_mode_dispatch` + `gpio_int0_named_routing` tests.)
- **interrupt::enable** now also raises the IRQ's `LOCIPRI` priority above the
  `PRITHD` threshold (both reset to 0 on the WS63 Nuclei ECLIC), so a custom IRQ
  (≥ 26) is actually **deliverable** after `enable()` alone — previously delivery
  silently required a separate `interrupt::init()`.
- **embassy**: the `embassy` feature exports a named `TIMER_INT0` handler that drives
  the alarm callback through rt's direct-mode dispatch (no app `#[interrupt]` shim);
  it is `cfg`-exclusive with the `async` timer's own `TIMER_INT0` to avoid a
  duplicate symbol.

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

### Fixed

- **uart**: `read_byte` now gates on the `rx_fifo_cnt` register (offset 0x4c, the
  field the vendor `hal_uart_v151` polls) instead of the `rx_fifo_empty` status bit,
  which on real silicon does **not** track a single-byte pop — the old check could
  return a stale byte or miss a fresh one. The async `on_interrupt` RX path uses the
  same count.

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

- **uart**: `Config::clock: UartClock` (a typed baud-base selection)
  plus `soc::chip::uart_boot_clock_hz()` and `UART_BOOT_CLOCK_{24M,40M}_HZ`.
  Examples that skip the (XIP-unsafe) `clock_init` run on flashboot's raw-TCXO
  console clock, not the 160 MHz PLL — **confirmed 40 MHz on silicon** (HW_CTL
  TCXO strap; `div_fra = 44` ⟺ 40 MHz), resolving the boot-baud root cause of
  #15/#10. `UartClock::Pll` keeps the 160 MHz post-`clock_init` behaviour.
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
