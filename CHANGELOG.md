# Changelog

All notable changes to this project are documented here.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

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

## [0.2.1] - 2026-06-02

### Changed

- CI: first release cut by ws63-hal's own repo pipeline (no functional change since 0.2.0).

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
