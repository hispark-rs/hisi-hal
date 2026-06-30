//! On-target (semihosting) HIL **driver** tests for the HAL, run on real WS63
//! silicon (and safe under QEMU). These live here — inside the HAL crate's
//! `tests/` — so the HAL's own driver tests ship and run with the HAL and inherit
//! its chip gating (`chip-ws63` default, `chip-bs21` via `--features chip-bs21`).
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

// riscv-only: the embedded-test harness + the riscv-only dev-deps (hisi-riscv-rt,
// the chip PAC alias) it names exist only in the riscv graph. On the host this
// whole module is dropped and `main()` above is the (no-op) entry.
#[cfg(all(test, target_arch = "riscv32"))]
#[embedded_test::tests]
mod tests {
    use core::hint::black_box;
    use hisi_riscv_hal as hal;
    // Chip-selected PAC alias: the suite names `pac::{Peripherals, Gpio0, ...}`
    // chip-agnostically and the active chip feature picks the concrete PAC.
    #[cfg(feature = "chip-bs21")]
    use bs2x_pac as pac;
    #[cfg(feature = "chip-ws63")]
    use ws63_pac as pac;

    /// `#[init]` runs before every test. It takes the singleton `Peripherals`
    /// once and hands them to each test as shared state — proving the PAC's
    /// critical-section-guarded `take()` (backed by hisi-riscv-rt's
    /// single-hart critical-section impl) works on-target.
    #[init]
    fn init() -> pac::Peripherals {
        pac::Peripherals::take().expect("PAC Peripherals::take() returned None on first call")
    }

    /// Read a real SoC register through the PAC singleton handed over by
    /// `#[init]` and assert a structural fact about it. We read the TCXO status
    /// register and assert the read completed (the value is whatever the bus
    /// returns); the point is that an MMIO load to the TCXO window succeeds
    /// on-target without trapping. Reads only — no writes, no state change.
    #[test]
    fn read_tcxo_status_register(p: pac::Peripherals) {
        // `bits()` performs a volatile 32-bit load from 0x4400_04c0 + offset.
        let status = p.tcxo.tcxo_status().read().bits();
        // The reserved upper bits are not all-ones on a sane bus read; this is a
        // weak-but-real liveness assertion that the load returned bus data rather
        // than the all-ones "no device" pattern.
        assert_ne!(status, 0xFFFF_FFFF, "TCXO status read returned the bus-floating all-ones pattern");
    }

    /// HAL `Peripherals` construction smoke test (PAC/HAL structural #8). The
    /// HAL's `Peripherals::take()` was already consumed by the PAC `take()` in
    /// `#[init]` (both back onto the same singleton), so we `steal()` the HAL
    /// peripherals — safe here because tests run sequentially on a single hart —
    /// and assert that several driver `ptr()`s resolve to the documented SoC
    /// windows. This proves the HAL peripheral wrappers construct without panic
    /// and point at the same addresses as the raw PAC. Mirrors the
    /// `peripherals!`/`peripheral!` macros in hisi-riscv-hal/src/peripherals.rs.
    ///
    /// Asserts WS63-specific peripheral-window addresses → gated `chip-ws63`. A
    /// `#[cfg(feature = "chip-bs21")]` sibling with the BS21 addresses can be
    /// added when a BS21 board exists.
    #[cfg(feature = "chip-ws63")]
    #[test]
    fn hal_peripherals_construct() {
        // SAFETY: sequential single-hart test run; no other live handles.
        let hp = unsafe { hal::Peripherals::steal() };
        // The HAL ZSTs construct; their register pointers match the PAC bases.
        assert_eq!(hal::peripherals::Gpio0::ptr() as usize, 0x4402_8000, "HAL GPIO0 ptr mismatch");
        assert_eq!(hal::peripherals::Tcxo::ptr() as usize, 0x4400_04c0, "HAL TCXO ptr mismatch");
        assert_eq!(hal::peripherals::Timer::ptr() as usize, 0x4400_2000, "HAL TIMER ptr mismatch");
        assert_eq!(hal::peripherals::Dma::ptr() as usize, 0x4a00_0000, "HAL DMA ptr mismatch");
        assert_eq!(hal::peripherals::Uart0::ptr() as usize, 0x4401_0000, "HAL UART0 ptr mismatch");
        // The struct itself constructed (fields are ZSTs); touch one to prove it.
        let _ = hp.GPIO0;
    }

    /// GPIO output read-back (gpio.rs / examples/ws63/blinky). Steal GPIO0's
    /// validated pin 0, drive it as a push-pull output, and assert the GPIO0
    /// block's data-out register (`gpio_sw_out`, the HAL's `is_set_high()` source)
    /// reflects each written level. `set_high()` writes `gpio_data_set`,
    /// `set_low()` writes `gpio_data_clr`; both are observed back through
    /// `gpio_sw_out` bit 0. Real pin I/O, no external wiring — pin 0 is the
    /// validated-safe pin used by blinky.
    #[test]
    fn gpio_output_readback() {
        use hal::gpio::{AnyPin, OutputConfig};
        // SAFETY: pin 0 is a valid WS63 GPIO (0..=18); sequential single-hart run
        // owns it exclusively. Mirrors blinky's `AnyPin::steal(0)`.
        let mut led = unsafe { AnyPin::steal(0) }.init_output(OutputConfig::new().with_initial(false));

        // Drive high → GPIO0 swout/data register bit 0 reads 1.
        led.set_high();
        // SAFETY: read-only MMIO load of the GPIO0 data register.
        let r = unsafe { &*pac::Gpio0::PTR };
        assert_eq!(r.gpio_sw_out().read().bits() & 1, 1, "GPIO0 bit0 did not read high after set_high()");
        assert!(led.is_set_high(), "Output::is_set_high() disagreed after set_high()");

        // Drive low → bit 0 reads 0.
        led.set_low();
        assert_eq!(r.gpio_sw_out().read().bits() & 1, 0, "GPIO0 bit0 did not read low after set_low()");
        assert!(!led.is_set_high(), "Output::is_set_high() disagreed after set_low()");
    }

    /// TCXO free-running counter is monotonic (tcxo.rs). Read the 32-bit counter
    /// twice with a busy-wait between; assert it strictly increased. The driver's
    /// `read_counter32()` latches via a refresh and returns `None` on refresh
    /// timeout — we require both reads to succeed AND the second to exceed the
    /// first (within a non-wrapping window). TCXO is validated-working silicon.
    #[test]
    fn tcxo_counter_monotonic() {
        use hal::tcxo::TcxoDriver;
        // SAFETY: sequential single-hart run; TCXO singleton not otherwise held.
        let tcxo = TcxoDriver::new(unsafe { hal::peripherals::Tcxo::steal() });

        let a = tcxo.read_counter32().expect("first TCXO refresh timed out");
        // Short busy-wait so the 24 MHz counter advances by a comfortable margin.
        for _ in 0..50_000 {
            black_box(0u32);
        }
        let b = tcxo.read_counter32().expect("second TCXO refresh timed out");
        assert!(b > a, "TCXO counter did not advance: first=0x{:08x} second=0x{:08x}", a, b);
    }

    /// Timer counter advances (timer.rs / examples/ws63/timer_irq). Configure
    /// TIMER channel 0 in periodic mode with a large load, enable it, and read the
    /// down-counter (`timer0_current_value`) twice with a busy-wait between;
    /// assert the count changed (advanced). Register/poll level only — we do NOT
    /// rely on the interrupt firing (embedded-test owns the trap handler). The
    /// timer ticks at the 24 MHz TCXO clock, so it moves quickly.
    // Previously #[ignore]'d: the counter appeared frozen on silicon because the
    // old current_value() read the latched `current_value0` register raw. WS63's
    // TIMER_V150 only refreshes that latch on a cnt_req/cnt_lock handshake (the
    // vendor HAL always does it; QEMU exposes a live counter so the bug hid). The
    // driver's current_value() now performs the handshake, so this runs on silicon.
    #[test]
    fn timer_counter_advances() {
        use hal::timer::{TimerDriver, TimerMode};
        // SAFETY: sequential single-hart run; TIMER singleton not otherwise held.
        let timer = TimerDriver::new(unsafe { hal::peripherals::Timer::steal() });
        // Large periodic load so the counter is plainly mid-flight across reads.
        timer.configure(0, TimerMode::Periodic, 0x00FF_FFFF);
        timer.enable(0);

        let a = timer.current_value(0);
        for _ in 0..50_000 {
            black_box(0u32);
        }
        let b = timer.current_value(0);
        timer.disable(0);
        assert_ne!(a, b, "TIMER ch0 current_value did not advance: a=0x{:08x} b=0x{:08x}", a, b);
    }

    /// DMA memory-to-memory end-to-end (dma.rs / examples/ws63/dma_loopback
    /// part 1's controller). Run a real mem→mem transfer on the PRIMARY,
    /// non-secure DMA (`Dma0` @ 0x4A00_0000) logical channel 0, wait for the
    /// channel-enable bit to auto-clear (completion), then assert the destination
    /// buffer equals the source. The highest-value end-to-end test: actual data
    /// movement by the DMA engine, self-contained (no external wiring).
    /// **Silicon-validated 2026-06-14.**
    //
    // Truth is the vendor C SDK + silicon, NOT QEMU. Three silicon facts the QEMU
    // model glosses over (each is a tracked QEMU issue, not something we work
    // around here):
    //   1. START: the transfer is kicked by setting the channel's bit in the
    //      global `dmac_en_chns` (DesignWare ChEnReg), which `configure_channel`
    //      now does — the per-channel CFG `ch_enable` bit alone does not start the
    //      engine on silicon (it does in QEMU). The DMAC auto-clears `en_chns[ch]`
    //      on completion, which is how the vendor (`hal_dma_v151_is_enabled`) and
    //      this test detect "done" — NOT `dmac_ori_int_st` (the QEMU path).
    //   2. CLOCK: `enable_controller()` bypasses the M_DMA auto-clock-gate
    //      (DMA_CLK_AUTO_CTRL_REG 0x4400_0244 |= 0x80000) so the clock stays on.
    //   3. CACHE: the RV32 core has a non-coherent D-cache, so we
    //      `cache::clean_range(src)` before (flush the CPU's bytes to RAM) and
    //      `cache::invalidate_range(dst)` after (drop the stale cached zeros).
    //      Buffers are 32-byte (cache-line) aligned so those ops touch only them.
    //
    // Uses Dma0 (M_DMA), not the secure SDMA (0x520A_0000): the WS63 vendor SDK
    // does ALL mem-to-mem through the primary controller (CONFIG_DMA_SUPPORT_SMDMA
    // is unset in every build, g_sdma_base_addr is never assigned), so the secure
    // block is never provisioned on silicon — a transfer there stalls AXI and
    // drops the debug link.
    #[cfg(feature = "chip-ws63")]
    #[test]
    fn dma_mem_to_mem() {
        use hal::dma::{Dma0, DmaChannelConfig, DmaDriver};
        const N: usize = 8;
        // 32-byte (cache-line) aligned so the by-range clean/invalidate below
        // only ever touches these buffers' own lines.
        #[repr(C, align(32))]
        struct Aligned([u32; N]);
        let src = Aligned([
            0xaaaa_0001,
            0xaaaa_0002,
            0xaaaa_0003,
            0xaaaa_0004,
            0xaaaa_0005,
            0xaaaa_0006,
            0xaaaa_0007,
            0xaaaa_0008,
        ]);
        let mut dst = Aligned([0u32; N]);
        let bytes = N * core::mem::size_of::<u32>();
        let src_ptr = src.0.as_ptr() as usize;
        let dst_ptr = dst.0.as_mut_ptr() as usize;

        // Clean the source out of the D-cache so the DMA master reads the bytes
        // the CPU just wrote, not stale RAM. SAFETY: real, owned stack range.
        unsafe { hal::cache::clean_range(src_ptr, bytes) };

        // SAFETY: sequential single-hart run; DMA singleton not otherwise held.
        let mut dma = DmaDriver::<Dma0>::new_dma(unsafe { hal::peripherals::Dma::steal() });
        dma.enable_controller();
        // Logical channel 0 == physical channel 0 on the primary controller.
        dma.configure_channel(0, src_ptr as u32, dst_ptr as u32, N as u16, &DmaChannelConfig::default());

        // Wait (bounded) for completion the way the vendor driver does
        // (`hal_dma_v151_is_enabled`): the DMAC auto-clears this channel's bit in
        // `en_chns` when the single-block transfer finishes, so the channel going
        // *not enabled* is the done signal. The bound stops a stuck transfer from
        // hanging the run. (We deliberately do NOT poll `dmac_ori_int_st` — that
        // is the path QEMU happens to drive; the silicon truth is `en_chns`.)
        let mut done = false;
        let mut budget = 1_000_000u32;
        while budget > 0 {
            if !dma.channel_enabled(0) {
                done = true;
                break;
            }
            budget -= 1;
        }
        dma.clear_transfer_interrupt(0);
        assert!(done, "DMA channel 0 transfer never completed (en_chns[0] stayed set)");

        // Invalidate the destination so the CPU reads what the DMA wrote to RAM,
        // not the stale (zero) copy cached when `dst` was initialised.
        unsafe { hal::cache::invalidate_range(dst_ptr, bytes) };

        for (i, &want) in src.0.iter().enumerate() {
            // Volatile: the DMA engine wrote `dst` behind the compiler's back.
            let got = unsafe { core::ptr::read_volatile(dst.0.as_ptr().add(i)) };
            assert_eq!(got, want, "DMA mem→mem mismatch @{}: got=0x{:08x} want=0x{:08x}", i, got, want);
        }
    }

    /// The owned-buffer `Transfer` guard (Area C) over the SAME mem-to-mem path on
    /// silicon: `start_mem_to_mem` cleans the source, launches, and returns a guard;
    /// `wait()` polls completion, invalidates the destination cache, and hands back
    /// the driver + both buffers. The guard owns the buffers (so a use-after-free is
    /// unrepresentable) and folds the cache maintenance into the type. `'static`
    /// 32-byte-aligned buffers satisfy embedded-dma's stable-deref contract.
    #[cfg(feature = "chip-ws63")]
    #[test]
    fn dma_transfer_guard() {
        use hal::dma::{Dma0, DmaDriver};
        #[repr(C, align(32))]
        struct Aligned([u32; 8]);
        static mut SRC: Aligned = Aligned([
            0xbbbb_0001,
            0xbbbb_0002,
            0xbbbb_0003,
            0xbbbb_0004,
            0xbbbb_0005,
            0xbbbb_0006,
            0xbbbb_0007,
            0xbbbb_0008,
        ]);
        static mut DST: Aligned = Aligned([0u32; 8]);
        // SAFETY: sequential single-hart run; these statics are touched only here.
        let src: &'static mut [u32] = unsafe { &mut (*core::ptr::addr_of_mut!(SRC)).0 };
        let dst: &'static mut [u32] = unsafe { &mut (*core::ptr::addr_of_mut!(DST)).0 };
        let want =
            [0xbbbb_0001u32, 0xbbbb_0002, 0xbbbb_0003, 0xbbbb_0004, 0xbbbb_0005, 0xbbbb_0006, 0xbbbb_0007, 0xbbbb_0008];

        // SAFETY: sequential single-hart run; DMA singleton not otherwise held.
        let mut dma = DmaDriver::<Dma0>::new_dma(unsafe { hal::peripherals::Dma::steal() });
        dma.enable_controller();

        let transfer = dma.start_mem_to_mem(0, &*src, dst);
        let (_dma, _src, dst) = transfer.wait();

        for (i, &w) in want.iter().enumerate() {
            let got = unsafe { core::ptr::read_volatile(dst.as_ptr().add(i)) };
            assert_eq!(got, w, "DMA guard mem→mem mismatch @{}: got=0x{:08x} want=0x{:08x}", i, got, w);
        }
    }

    /// Clock-gate enable (clock.rs). The HAL's CKEN bit map lives in
    /// `clock::Peripheral::cken_info()` (the old `ClockControl` RAII layer was
    /// removed as dead code — see clock.rs module docs). UART0's gate is
    /// `CKEN_CTL1` bit 18. WS63 clocks default to ENABLED out of reset, so we
    /// assert the gate is already set; then set it again through the PAC
    /// `CldoCrg` register and re-read to confirm the bit is high. Read-modify-set
    /// of a clock-enable bit is non-destructive (it keeps the clock running).
    ///
    /// Asserts the WS63-specific UART0 CKEN gate (CKEN_CTL1 bit 18) → gated
    /// `chip-ws63`. A `#[cfg(feature = "chip-bs21")]` sibling with the BS21 gate
    /// location can be added when a BS21 board exists.
    #[cfg(feature = "chip-ws63")]
    #[test]
    fn clock_gate_uart0_enabled() {
        use hal::clock::Peripheral;
        // The map must agree with the documented UART0 gate (CKEN_CTL1 bit 18).
        let (reg_idx, bit) = Peripheral::Uart0.cken_info().expect("UART0 should be a gated peripheral");
        assert_eq!((reg_idx, bit), (1, 18), "UART0 CKEN gate moved");

        // SAFETY: read-only / RMW-set of the clock-enable register; setting an
        // already-set enable bit keeps the peripheral clock running.
        let crg = unsafe { &*pac::CldoCrg::PTR };
        let before = crg.cken_ctl1().read().bits();
        assert_ne!(before & (1 << bit), 0, "UART0 clock gate (CKEN_CTL1 bit 18) not set out of reset");

        // Re-assert the gate and confirm it reads back high.
        crg.cken_ctl1().modify(|r, w| unsafe { w.bits(r.bits() | (1 << bit)) });
        let after = crg.cken_ctl1().read().bits();
        assert_ne!(after & (1 << bit), 0, "UART0 clock gate not high after re-enable");
    }

    /// System reset-reason read-only decode (system.rs / examples/ws63/reset_demo).
    /// Construct `System` from stolen SYS_CTL0/GLB_CTL_M/CLDO_CRG handles and call
    /// `reset_reason()`, asserting it returns one of the valid variants. We do NOT
    /// call `software_reset()` (it reboots the chip and would break the test run).
    /// Note: `reset_reason()` reads AND CLEARS the matched history bit, so it is
    /// run once. Mirrors reset_demo's `System::new(...).reset_reason()`.
    ///
    /// WS63-only: the HAL's `system` module and the `SysCtl0`/`CldoCrg`
    /// peripherals are `#[cfg(feature = "chip-ws63")]` in the HAL (the reset/CRG
    /// stack is a deeper port), so this test is gated `chip-ws63`. A
    /// `#[cfg(feature = "chip-bs21")]` sibling can be added once the HAL exposes a
    /// BS21 reset-reason API.
    #[cfg(feature = "chip-ws63")]
    #[test]
    fn system_reset_reason_valid() {
        use hal::system::{ResetReason, System};
        // SAFETY: sequential single-hart run; these singletons not otherwise held.
        let system = unsafe {
            System::new(
                hal::peripherals::SysCtl0::steal(),
                hal::peripherals::GlbCtlM::steal(),
                hal::peripherals::CldoCrg::steal(),
            )
        };
        let reason = system.reset_reason();
        assert!(
            matches!(
                reason,
                ResetReason::PowerOn
                    | ResetReason::ExternalPin
                    | ResetReason::Watchdog
                    | ResetReason::Software
                    | ResetReason::BrownOut
                    | ResetReason::Unknown
            ),
            "reset_reason() returned an out-of-range variant",
        );
    }

    /// UART divider register configuration (uart.rs). Construct UART0 via
    /// `Uart::new_uart0(.., Config::default())` (115200 8N1) and assert the
    /// programmed `div_l`/`div_h`/`div_fra` registers match the HAL's
    /// fixed-point baud formula: div*64 = UART_CLOCK_HZ*4 / baud, with the low 6
    /// bits the fractional part. This tests the register CONFIG only — NOT actual
    /// serial output (on-silicon UART baud is a known-open issue #15; we do not
    /// assert bytes on the wire).
    #[test]
    fn uart0_divider_config() {
        use hal::uart::{Config, Uart};
        let cfg = Config::default(); // 115200 8N1
        // SAFETY: sequential single-hart run; UART0 singleton not otherwise held.
        let _uart = Uart::new_uart0(unsafe { hal::peripherals::Uart0::steal() }, cfg);

        // Recompute the expected divider exactly as configure_uart() does.
        let pclk = hal::soc::chip::UART_CLOCK_HZ; // 160 MHz
        let div64 = ((pclk as u64) * 4 / (cfg.baudrate.baud() as u64)) as u32; // = div * 64
        let div = div64 >> 6;
        let exp_div_fra = (div64 & 0x3F) as u16;
        let exp_div_l = (div & 0xFF) as u16;
        let exp_div_h = ((div >> 8) & 0xFF) as u16;

        // SAFETY: read-only MMIO loads of the UART0 divider registers.
        let r = unsafe { &*pac::Uart0::PTR };
        assert_eq!(r.div_l().read().bits(), exp_div_l, "UART0 div_l mismatch");
        assert_eq!(r.div_h().read().bits(), exp_div_h, "UART0 div_h mismatch");
        assert_eq!(r.div_fra().read().bits(), exp_div_fra, "UART0 div_fra mismatch");
    }

    // (A pwm0_period_duty_config register-config test was tried but needs full
    // PWM clock-tree bring-up — the 9-bit CKEN_CTL0 field AND the CLDO_CRG_DIV_CTL3
    // dividers with LOAD_DIV_EN, per vendor `pwm_port_clock_enable` — before the
    // PWM registers latch writes. Deferred as PWM bring-up, not added as a flaky
    // or #[ignore]'d test.)

    /// TRNG produces entropy (trng.rs). Read several 32-bit words from the TRNG
    /// hardware and assert at least two reads succeed AND not all are identical —
    /// a real on-silicon entropy liveness check (no jumpers). The FRO source can
    /// need a few attempts to stabilise on cold start (per `read_blocking`), so
    /// we retry.
    #[test]
    fn trng_produces_entropy() {
        use hal::trng::TrngDriver;
        // SAFETY: sequential single-hart run; TRNG singleton not otherwise held.
        let trng = TrngDriver::new(unsafe { hal::peripherals::Trng::steal() });
        let mut samples = [0u32; 4];
        let mut got = 0usize;
        for _ in 0..16 {
            if got >= samples.len() {
                break;
            }
            if let Ok(w) = trng.read_blocking() {
                samples[got] = w;
                got += 1;
            }
        }
        assert!(got >= 2, "TRNG produced fewer than 2 words (got {got})");
        let all_same = samples[..got].iter().all(|&w| w == samples[0]);
        assert!(!all_same, "TRNG returned {got} identical words 0x{:08x} — no entropy", samples[0]);
    }

    /// eFuse read path (efuse.rs / reset_demo). Set the read clock period for the
    /// detected TCXO, then read byte 0 and assert the read COMPLETES (`Ok`) — a
    /// read-only liveness check of the eFuse controller on silicon. Contents are
    /// board-specific, so we assert the path works, not a particular value.
    ///
    /// WS63-only: `set_clock_period`/`read_byte` and the `Efuse` peripheral are
    /// chip-ws63 in the HAL.
    #[cfg(feature = "chip-ws63")]
    #[test]
    fn efuse_read_byte0_ok() {
        use hal::efuse::EfuseDriver;
        // SAFETY: sequential single-hart run; EFUSE singleton not otherwise held.
        let mut efuse = EfuseDriver::new(unsafe { hal::peripherals::Efuse::steal() });
        // 0x29 @ 24 MHz TCXO, 0x19 @ 40 MHz (per the driver docs / vendor SDK).
        let period: u8 =
            if hal::soc::chip::uart_boot_clock_hz() == hal::soc::chip::UART_BOOT_CLOCK_24M_HZ { 0x29 } else { 0x19 };
        efuse.set_clock_period(period);
        let r = efuse.read_byte(0);
        assert!(r.is_ok(), "eFuse read_byte(0) failed: {:?}", r.err());
    }

    /// On-die temperature sensor (tsensor.rs). Enable the sensor, trigger a
    /// conversion, then bounded-poll `read_raw()` and assert the 10-bit code is
    /// within the driver's documented valid range (114..=896). Self-contained:
    /// the sensor is on-die, no external wiring. Uses the bounded `read_raw()`
    /// (not the unbounded `read_blocking()`) so a non-responsive sensor cannot
    /// hang the run.
    #[cfg(feature = "chip-ws63")]
    #[test]
    fn tsensor_reads_in_range() {
        use hal::tsensor::TempSensor;
        // SAFETY: sequential single-hart run; TSENSOR singleton not otherwise held.
        let mut ts = TempSensor::new(unsafe { hal::peripherals::Tsensor::steal() });
        ts.enable();
        ts.start_conversion();
        let mut code = None;
        for _ in 0..1_000_000u32 {
            if let Some(c) = ts.read_raw() {
                code = Some(c);
                break;
            }
        }
        let code = code.expect("tsensor never asserted data-ready");
        assert!((114..=896).contains(&code), "tsensor code {code} outside the valid 114..=896 range");
    }

    /// I2C0 SCL divider configuration (i2c.rs / examples/ws63/i2c_scan).
    /// Construct I2C0 at 100 kHz and assert the programmed `i2c_scl_h`/`i2c_scl_l`
    /// half-period registers match the HAL's divider (each = (I2C_CLOCK_HZ /
    /// (2·freq)) / 2, off the **24 MHz TCXO** — not the CPU clock; the vendor
    /// `clock_init` leaves I2C on the crystal) and that `i2c_en` is set. Register
    /// CONFIG only — no bus transaction, so no wired peer / pull-ups are needed.
    /// I2C0 is not individually clock-gated (default-on), so its window is live.
    #[cfg(feature = "chip-ws63")]
    #[test]
    fn i2c0_scl_config() {
        use hal::i2c::{I2c, Speed};
        // SAFETY: sequential single-hart run; I2C0 singleton not otherwise held.
        let _i2c = I2c::new_i2c0(unsafe { hal::peripherals::I2c0::steal() }, Speed::Standard);

        let pclk = hal::soc::chip::I2C_CLOCK_HZ; // 24 MHz TCXO
        let expected_half = (pclk / (2 * Speed::Standard.hz())) / 2;
        // SAFETY: read-only MMIO loads of the I2C0 config registers.
        let r = unsafe { &*pac::I2c0::PTR };
        assert_eq!(r.i2c_scl_h().read().bits(), expected_half, "I2C0 scl_h mismatch");
        assert_eq!(r.i2c_scl_l().read().bits(), expected_half, "I2C0 scl_l mismatch");
        assert!(r.i2c_ctrl().read().i2c_en().bit_is_set(), "I2C0 i2c_en not set after new_i2c0");
    }

    /// PWM typed config + clock-tree bring-up (pwm.rs). `configure` brings up the
    /// PWM clock tree itself (CLK_SEL high-freq + CKEN_CTL0 [10:2] + DIV_CTL3
    /// divider, per vendor `pwm_port_clock_enable`) and takes a validated
    /// `PwmPeriod` + `Duty`, so an invalid frequency or duty is unrepresentable.
    /// The usable period is **16-bit**: silicon was measured NOT to latch the
    /// `pwm_freq_h0` half (it reads back 0 even with the full clock tree up), which
    /// is exactly why `PwmPeriod` is a `u16`. Configures a 24 000-tick / 50 % duty
    /// waveform and asserts `pwm_freq_l0` latched it, `pwm_freq_h0` stayed 0, and the
    /// duty register holds 12 000; then enable/disable. No pin output asserted.
    #[cfg(feature = "chip-ws63")]
    #[test]
    fn pwm_configure_and_enable() {
        use hal::pwm::{Duty, PwmChannel, PwmPeriod};
        // SAFETY: sequential single-hart run; PWM singleton not otherwise held.
        let pwm = unsafe { hal::peripherals::Pwm::steal() };
        let mut ch = PwmChannel::new(&pwm, 0);
        let period = PwmPeriod::from_count(24_000).expect("non-zero period");
        ch.configure(period, Duty::HALF);
        ch.enable();

        // SAFETY: read-only MMIO loads of the PWM ch0 registers.
        let r = unsafe { &*pac::Pwm::PTR };
        assert_eq!(r.pwm_freq_l0().read().bits() as u32, 24_000, "PWM freq_l0 not latched");
        assert_eq!(r.pwm_freq_h0().read().bits() as u32, 0, "PWM freq_h0 unexpectedly non-zero");
        assert_eq!(r.pwm_duty_l0().read().bits() as u32, 12_000, "PWM 50% duty not latched");
        assert_ne!(r.pwm_en0().read().bits() & 1, 0, "PWM ch0 not enabled");

        ch.disable();
        assert_eq!(r.pwm_en0().read().bits() & 1, 0, "PWM ch0 still enabled after disable");
    }

    /// Watchdog timeout **validation + load programming** (wdt.rs).
    /// The old silent u64-saturation is gone: an out-of-range timeout (300 s, far
    /// beyond the 24-bit `wdt_load[31:8]` field's ~178 s ceiling at 24 MHz) is now
    /// REJECTED at `WdtTimeout::from_ms` (returns `None`), while a valid in-range
    /// timeout programs the exact computed load field on silicon. Configured with
    /// **reset DISABLED** so the WDT can never reboot the board.
    #[cfg(feature = "chip-ws63")]
    #[test]
    fn wdt_configure_saturates_load() {
        use hal::wdt::{ResetPulseLength, WDT_MAX_LOAD, Watchdog, WdtMode, WdtTimeout};
        // 300 s ≫ the field's ~178 s max → rejected at construction, no clamp.
        assert!(WdtTimeout::from_ms(300_000).is_none(), "over-range timeout must be rejected");
        assert!(WdtTimeout::from_ms(WdtTimeout::MAX_MS).is_some());
        assert!(WdtTimeout::from_ms(WdtTimeout::MAX_MS + 1).is_none());

        // A valid 1 s timeout programs the exact load field on silicon.
        let timeout = WdtTimeout::from_ms(1_000).unwrap();
        // SAFETY: sequential single-hart run; WDT singleton not otherwise held.
        let mut wdt = Watchdog::new(unsafe { hal::peripherals::Wdt::steal() });
        wdt.configure(timeout, WdtMode::SingleInterrupt, false, ResetPulseLength::Cycles2)
            .expect("configure should succeed on live silicon");

        // Expected field = (1000 ms · 24 MHz / 1000) >> 8 = 24_000_000 >> 8 = 93_750.
        let expected = (timeout.as_ms() as u64 * hal::wdt::WDT_CLOCK_HZ as u64 / 1000 >> 8) as u32;
        assert!(expected <= WDT_MAX_LOAD);
        // SAFETY: read-only MMIO load of the WDT load register; field is in [31:8].
        let r = unsafe { &*pac::Wdt::PTR };
        let load = r.wdt_load().read().bits() >> 8;
        assert_eq!(load, expected, "WDT load mismatch: got 0x{:06x} want 0x{:06x}", load, expected);
        wdt.disable();
    }

    /// drop-to-disable (Area E) on silicon: a dropped `Watchdog` clears `WDT_CR.wdt_en`
    /// (scoped safety — it cannot reset the board after its scope), while
    /// `into_armed()` is the escape hatch that keeps it enabled. Configured with
    /// **reset DISABLED**, so even the briefly-armed window cannot reboot the board.
    #[cfg(feature = "chip-ws63")]
    #[test]
    fn wdt_drop_disables_unless_armed() {
        use hal::wdt::{ResetPulseLength, Watchdog, WdtMode, WdtTimeout};
        let r = unsafe { &*pac::Wdt::PTR };
        let cfg = |wdt: &mut Watchdog| {
            wdt.configure(
                WdtTimeout::from_ms(1_000).unwrap(),
                WdtMode::SingleInterrupt,
                false, // reset DISABLED — armed window cannot reboot the board
                ResetPulseLength::Cycles2,
            )
            .expect("configure on live silicon");
        };

        // Dropping the handle clears the enable bit.
        {
            // SAFETY: sequential single-hart run; WDT singleton not otherwise held.
            let mut wdt = Watchdog::new(unsafe { hal::peripherals::Wdt::steal() });
            cfg(&mut wdt);
            assert_eq!(r.wdt_cr().read().bits() & 0x1, 0x1, "WDT not enabled after configure");
        } // <- Drop runs here
        assert_eq!(r.wdt_cr().read().bits() & 0x1, 0x0, "drop did not clear WDT_CR.wdt_en");

        // The escape hatch keeps it armed past the scope.
        {
            let mut wdt = Watchdog::new(unsafe { hal::peripherals::Wdt::steal() });
            cfg(&mut wdt);
            let _armed = wdt.into_armed(); // no disabling Drop
        }
        assert_eq!(r.wdt_cr().read().bits() & 0x1, 0x1, "into_armed must keep WDT enabled");

        // Cleanup: stop the armed watchdog so the harness continues cleanly.
        Watchdog::new(unsafe { hal::peripherals::Wdt::steal() }).disable();
    }

    /// I2S register liveness (i2s.rs). I2S is clock-gated (CKEN_CTL0 bit 12 = clk,
    /// bit 11 = bus — `sio_porting.c`); enable both, `configure` the block (master,
    /// I2S, 16-bit) without faulting, and assert the IP `version` register reads a
    /// sane, non-floating ID (silicon returns 0x13) — proving the I2S window is
    /// clocked and its register map resolves. Register CONFIG only — a full TX/RX
    /// data path needs the BCLK/FS dividers + an external codec (or internal
    /// loopback), out of scope for a self-contained liveness check.
    #[cfg(feature = "chip-ws63")]
    #[test]
    fn i2s_version_live() {
        use hal::i2s::{I2sDriver, MasterConfig};
        // `new_master` self-enables the I2S clk (bit 12) + bus (bit 11) gates and the
        // CMU divider reset-sync, then configures the block (master, I2S, 16-bit).
        // SAFETY: sequential single-hart run; I2S singleton not otherwise held.
        let i2s = I2sDriver::new_master(unsafe { hal::peripherals::I2s::steal() }, &MasterConfig::default());

        let ver = i2s.version();
        assert!(
            ver != 0 && ver != 0xFF,
            "I2S version register read an unsane value 0x{:02x} (block not clocked?)",
            ver
        );
    }

    /// LSADC scan-config register mapping (lsadc.rs). The LSADC register map had a
    /// block-wide offset bug fixed in phase 2; this asserts the corrected map on
    /// silicon. `configure_scan` sets the selected channel bit + the averaging /
    /// sample-count fields in `lsadc_ctrl_0`; read them back. LSADC is not
    /// individually clock-gated (default-on). Register CONFIG only — NO analog
    /// conversion: a full conversion needs the AFE/LDO power-up sequence and an
    /// unbounded done-poll, which could stall the bus if the analog supply isn't
    /// provisioned (same risk class as the RTC crystal below).
    #[cfg(feature = "chip-ws63")]
    #[test]
    fn lsadc_scan_config() {
        use hal::lsadc::{AdcChannel, AdcConfig, LsAdc};
        // SAFETY: sequential single-hart run; LSADC singleton not otherwise held.
        let mut adc = LsAdc::new(unsafe { hal::peripherals::Lsadc::steal() });
        let cfg = AdcConfig::default();
        adc.configure_scan(AdcChannel::Channel0, &cfg);

        // SAFETY: read-only MMIO load of the LSADC control register.
        let r = unsafe { &*pac::Lsadc::PTR };
        let ctrl0 = r.lsadc_ctrl_0().read();
        assert_ne!(ctrl0.channel().bits() & (1 << 0), 0, "LSADC channel-0 select bit not set");
        assert_eq!(ctrl0.equ_model_sel().bits(), cfg.averaging as u8, "LSADC averaging field mismatch");
        assert_eq!(ctrl0.sample_cnt().bits(), cfg.sample_count.bits(), "LSADC sample_cnt field mismatch");
    }

    /// RTC free-running counter advances (rtc.rs) — **opt-in** (`hil-rtc` feature).
    /// This board does NOT populate the RTC's 32.768 kHz crystal, so its clock
    /// domain never comes up and touching the RTC registers stalls the bus / drops
    /// the debug link (same failure class as the unprovisioned secure SDMA) — hence
    /// it is gated behind `hil-rtc` and is OFF in the default suite. On a board that
    /// DOES populate the crystal, run `--features chip-ws63,hil-rtc`: configure
    /// free-running, enable, and assert `current_value` advanced across a long
    /// busy-wait (the 32 kHz counter is slow, so the CPU wait is large).
    #[cfg(all(feature = "chip-ws63", feature = "hil-rtc"))]
    #[test]
    fn rtc_counter_advances() {
        use hal::rtc::{RtcDriver, RtcMode};
        // SAFETY: sequential single-hart run; RTC singleton not otherwise held.
        let mut rtc = RtcDriver::new(unsafe { hal::peripherals::Rtc::steal() });
        rtc.configure(RtcMode::FreeRunning, 0);
        rtc.enable();
        let a = rtc.current_value();
        // 32.768 kHz counter: a large CPU busy-loop spans many ms / hundreds of
        // ticks at 240 MHz, well over the ~30 µs tick period.
        for _ in 0..3_000_000u32 {
            black_box(0u32);
        }
        let b = rtc.current_value();
        rtc.disable();
        assert_ne!(a, b, "RTC counter did not advance: a=0x{:08x} b=0x{:08x}", a, b);
    }

    /// GADC register liveness (gadc.rs) — **BS2X-only** (`chip-bs21`). The WS63 has
    /// no GADC (it uses the LSADC, tested above), so the user-requested GADC check
    /// lives here for a BS21 board: read the done/status register (`rpt_gadc_data_3`)
    /// and assert it is not the all-ones bus-floating pattern. Reads only — no
    /// AFE/LDO power-up or conversion (those need the analog supply + an unbounded
    /// done-poll; out of scope for a bare register-liveness check). Never built for
    /// the WS63 board.
    #[cfg(feature = "chip-bs21")]
    #[test]
    fn gadc_register_liveness() {
        // SAFETY: read-only MMIO load of a GADC status register via the PAC.
        let r = unsafe { &*pac::Gadc::PTR };
        let status = r.rpt_gadc_data_3().read().bits();
        assert_ne!(status, 0xFFFF_FFFF, "GADC status read returned the all-ones bus-floating pattern");
    }

    /// **Area D — full device.x-named interrupt routing.** hisi-riscv-rt runs in
    /// DIRECT mtvec mode: every trap reaches `trap_entry`, which routes interrupts
    /// (mcause bit 31) by IRQ number through `__rt_irq_dispatch` → the
    /// `__INTERRUPTS` table → the **named `device.x` handler** for that IRQ. We
    /// define `TIMER_INT0` (IRQ 26 = the embassy alarm channel) and fire TIMER
    /// channel 0 — asserting our named handler runs proves the whole chain
    /// end-to-end on real silicon, with no `mcause` test in the app. The WS63
    /// (Nuclei ECLIC) only delivers a custom IRQ once its `LOCIPRI` priority >
    /// threshold, which `interrupt::init()` sets. No external wiring.
    ///
    /// Gated to non-async/non-embassy builds: with those features the HAL itself
    /// defines `TIMER_INT0` (async timer / embassy alarm), so this test cannot also
    /// define it. The async path is covered by `gpio_int0_named_routing` instead.
    #[cfg(all(feature = "chip-ws63", not(feature = "async"), not(feature = "embassy")))]
    #[test]
    fn timer_int0_named_routing() {
        use core::sync::atomic::{AtomicBool, Ordering};
        use hal::interrupt;
        use hal::timer::{TimerDriver, TimerMode};

        static FIRED: AtomicBool = AtomicBool::new(false);

        // The named device.x handler the rt routes IRQ 26 to (overrides the weak
        // PROVIDE = DefaultHandler). Clear + stop the timer so it can't re-fire,
        // then record the hit.
        #[unsafe(no_mangle)]
        extern "C" fn TIMER_INT0() {
            let t = TimerDriver::new(unsafe { hal::peripherals::Timer::steal() });
            t.clear_interrupt(0);
            t.disable(0);
            FIRED.store(true, Ordering::SeqCst);
        }

        let t = TimerDriver::new(unsafe { hal::peripherals::Timer::steal() });
        // Periodic/user-defined mode counts from the load value (24 MHz TCXO → ~1
        // ms); the handler disables it on the first fire.
        t.configure(0, TimerMode::Periodic, 24_000);
        // SAFETY: `enable` now also raises this IRQ's LOCIPRI priority above the
        // reset-0 threshold (no separate `interrupt::init()` needed for delivery);
        // then the global machine-interrupt enable. The handler clears the source.
        unsafe {
            interrupt::enable(interrupt::Interrupt::TIMER_INT0);
            interrupt::enable_global();
        }
        t.enable(0);

        let mut spun = 0u32;
        while !FIRED.load(Ordering::SeqCst) && spun < 20_000_000 {
            spun += 1;
            core::hint::spin_loop();
        }
        unsafe { interrupt::disable(interrupt::Interrupt::TIMER_INT0) };
        t.disable(0);
        t.clear_interrupt(0);

        assert!(
            FIRED.load(Ordering::SeqCst),
            "named TIMER_INT0 (IRQ 26) handler never ran — rt device.x named routing broken"
        );
    }

    /// **Area D / #7 — async-driver named interrupt routing on silicon.** A real
    /// GPIO edge interrupt must reach the HAL's named `GPIO_INT0` handler (which the
    /// rt routes by IRQ number) and run `Gpio::on_interrupt(0)` — with NO app
    /// `mcause` trap. Uses the GPIO0→GPIO3 jumper: arm GPIO3 for a rising edge, drive
    /// GPIO0 high, and assert `on_interrupt` ran by checking it masked GPIO3's
    /// `gpio_int_en` bit (its documented side effect). Needs `async` (the named
    /// handler is async-gated) + the jumper.
    #[cfg(all(feature = "chip-ws63", feature = "async", feature = "hil-loopback"))]
    #[test]
    fn gpio_int0_named_routing() {
        use hal::gpio::{AnyPin, InputConfig, InterruptTrigger, OutputConfig};
        use hal::interrupt;
        use hal::io_config::IoConfigDriver;

        let mut io = IoConfigDriver::new(unsafe { hal::peripherals::IoConfig::steal() });
        io.set_gpio_mux(0, 0); // plain-GPIO on both pads
        io.set_gpio_mux(3, 0);

        // SAFETY: GPIO0/3 owned by this test; jumpered 0->3.
        let mut out = unsafe { AnyPin::steal(0) }.init_output(OutputConfig::new().with_initial(false));
        let inp = unsafe { AnyPin::steal(3) }.init_input(InputConfig::default());
        inp.set_interrupt_trigger(InterruptTrigger::RisingEdge);
        inp.enable_interrupt(); // GPIO0 bank `gpio_int_en` bit 3 = 1

        // SAFETY: enabling GPIO_INT0 (IRQ 33; enable also raises its LOCIPRI) + global.
        unsafe {
            interrupt::enable(interrupt::Interrupt::GPIO_INT0);
            interrupt::enable_global();
        }

        let g0 = unsafe { &*pac::Gpio0::PTR };
        assert_ne!(g0.gpio_int_en().read().bits() & (1 << 3), 0, "GPIO3 int-enable not set");

        out.set_high(); // 0->3 rising edge -> GPIO_INT0 -> on_interrupt(0) masks bit 3

        let mut spun = 0u32;
        while (g0.gpio_int_en().read().bits() & (1 << 3)) != 0 && spun < 20_000_000 {
            spun += 1;
            core::hint::spin_loop();
        }
        unsafe { interrupt::disable(interrupt::Interrupt::GPIO_INT0) };

        assert_eq!(
            g0.gpio_int_en().read().bits() & (1 << 3),
            0,
            "named GPIO_INT0 handler never ran — on_interrupt(0) did not mask GPIO3 (rt async named routing broken)"
        );
    }

    // ── Loopback tests (opt-in `hil-loopback` feature; need board jumpers) ──

    /// GPIO output→input loopback (gpio.rs + io_config.rs). Drive GPIO0 as a
    /// push-pull output and assert GPIO3 (input) follows it high and low — exercises
    /// the GPIO **input** read path (`gpio_sw_out` reflects the pad in input mode),
    /// which the output-only `gpio_output_readback` test never touches. The whole
    /// path goes through the public HAL (`init_output`/`init_input`/`is_high`): with
    /// `init_input` asserting the pad's input-enable (IE) bit, no manual pad writes
    /// are needed (silicon-confirmed 2026-06-14: GPIO3 tracks GPIO0 both ways).
    /// **Requires a jumper GPIO0 → GPIO3.** (GPIO5 is unusable on this board — it
    /// drives an SK6805 addressable LED.) GPIO0 is the validated-driving pin
    /// (blinky); the gpio drivers set direction, we just select plain-GPIO mux.
    #[cfg(all(feature = "chip-ws63", feature = "hil-loopback"))]
    #[test]
    fn gpio_loopback_0_to_3() {
        use hal::gpio::{AnyPin, InputConfig, OutputConfig, Pull};
        use hal::io_config::IoConfigDriver;
        // SAFETY: sequential single-hart run; IO_CONFIG singleton not otherwise held.
        let mut io = IoConfigDriver::new(unsafe { hal::peripherals::IoConfig::steal() });
        io.set_gpio_mux(0, 0); // plain-GPIO function on both pads
        io.set_gpio_mux(3, 0);
        let settle = || {
            for _ in 0..50_000u32 {
                black_box(0u32);
            }
        };
        // SAFETY: GPIO0/3 owned by this test; jumpered 0->3 on the board.
        let mut out = unsafe { AnyPin::steal(0) }.init_output(OutputConfig::new().with_initial(false));
        // Pull-down so a missing/loose jumper reads low (fails) rather than floating.
        let inp = unsafe { AnyPin::steal(3) }.init_input(InputConfig::new().with_pull(Pull::Down));

        out.set_high();
        settle();
        let hi = inp.is_high();
        out.set_low();
        settle();
        let lo = inp.is_high();

        semihosting::println!("[gpio-lb] GPIO0->GPIO3: drive-high read={hi} drive-low read={lo}");
        assert!(hi, "GPIO3 did not read high when GPIO0 driven high — check GPIO0->GPIO3 jumper");
        assert!(!lo, "GPIO3 did not read low when GPIO0 driven low — check GPIO0->GPIO3 jumper");
    }

    /// SPI0 MOSI→MISO loopback (spi.rs + io_config.rs). Transfer 4 bytes and assert
    /// the received buffer equals the sent one. **Requires a jumper GPIO9 (SPI0
    /// DO/MOSI) → GPIO11 (SPI0 DI/MISO);** SCK (GPIO7) and CS (GPIO10) are
    /// master-driven (no jumper). The HAL has no SPI pin-mux helper, so we mux the
    /// four SPI0 pads to function 3 first.
    #[cfg(all(feature = "chip-ws63", feature = "hil-loopback"))]
    #[test]
    fn spi0_loopback_mosi_to_miso() {
        use hal::io_config::IoConfigDriver;
        use hal::spi::{Config, Spi};
        let mut io = IoConfigDriver::new(unsafe { hal::peripherals::IoConfig::steal() });
        for pin in [7u8, 9, 10, 11] {
            io.set_gpio_mux(pin, 3);
        }
        // SAFETY: SPI0 singleton not otherwise held; GPIO9->GPIO11 jumpered.
        let mut spi = Spi::new_spi0(unsafe { hal::peripherals::Spi0::steal() }, Config::default());
        let tx = [0xA5u8, 0x3C, 0x00, 0xFF];
        let mut rx = [0u8; 4];
        let res = spi.transfer(&tx, &mut rx);
        semihosting::println!("[spi-lb] transfer={res:?} tx={tx:02x?} rx={rx:02x?}");
        res.expect("SPI0 transfer returned an error");
        assert_eq!(rx, tx, "SPI0 loopback mismatch — check GPIO9->GPIO11 jumper: tx={tx:02x?} rx={rx:02x?}");
    }

    /// SPI0 TX DMA (mem→peripheral) via the ergonomic `SpiDma::write_dma` API.
    /// Drives MOSI via DMA; `write_dma` drains the looped-back RX FIFO internally
    /// and returns `Ok` on completion. The first silicon proof of the `SpiDma`
    /// wrapper (vendor handshake order, cache clean, bounded wait, teardown).
    /// **Requires the GPIO9→GPIO11 (MOSI→MISO) jumper.**
    #[cfg(all(feature = "chip-ws63", feature = "hil-loopback", feature = "unstable"))]
    #[test]
    fn spi_dma_tx_loopback() {
        use hal::dma::{Dma0, DmaDriver};
        use hal::io_config::IoConfigDriver;
        use hal::spi::{Config, Spi};

        const N: usize = 8;
        #[repr(C, align(32))]
        struct Aligned([u8; N]);
        static TX: Aligned = Aligned([0xA5, 0x3C, 0x00, 0xFF, 0x5A, 0xC3, 0x0F, 0xF0]);

        let mut io = IoConfigDriver::new(unsafe { hal::peripherals::IoConfig::steal() });
        for pin in [7u8, 9, 10, 11] {
            io.set_gpio_mux(pin, 3);
        }
        // SAFETY: SPI0/DMA singletons not otherwise held.
        let spi = Spi::new_spi0(unsafe { hal::peripherals::Spi0::steal() }, Config::default());
        let mut dma = DmaDriver::<Dma0>::new_dma(unsafe { hal::peripherals::Dma::steal() });
        dma.enable_controller();
        let chs = dma.split_channels().expect("DMA channels already claimed");
        let mut sd = spi.with_dma(dma);

        // write_dma: vendor order (watermark → clean → start → tdmae), bounded wait,
        // drain RX, clear tdmae — all inside. Ok ⇒ the ergonomic TX path works.
        sd.write_dma(chs.ch0, &TX.0[..]).expect("SpiDma::write_dma failed");
        semihosting::println!("[spi-dma-tx] SpiDma::write_dma ok for {N} bytes");
    }

    /// SPI0 full-duplex DMA via the ergonomic `SpiDma::transfer_dma` API: TX DMA
    /// drives MOSI while RX DMA drains MISO concurrently (two channels). With the
    /// GPIO9→GPIO11 (MOSI→MISO) jumper, rx_buf must equal tx_buf. Proves the
    /// ergonomic wrapper + dual-channel concurrency + RX-side invalidate on silicon.
    /// **Requires the GPIO9→GPIO11 jumper.**
    #[cfg(all(feature = "chip-ws63", feature = "hil-loopback", feature = "unstable"))]
    #[test]
    fn spi_dma_fullduplex_loopback() {
        use hal::dma::{Dma0, DmaDriver};
        use hal::io_config::IoConfigDriver;
        use hal::spi::{Config, Spi};

        const N: usize = 8;
        #[repr(C, align(32))]
        struct Aligned([u8; N]);
        static TX: Aligned = Aligned([0x1A, 0x2B, 0x3C, 0x4D, 0x5E, 0x6F, 0x70, 0x81]);
        static mut RX: Aligned = Aligned([0u8; N]);
        // SAFETY: sequential single-hart run; RX touched only here.
        let rx: &'static mut [u8] = unsafe { &mut (*core::ptr::addr_of_mut!(RX)).0 };

        let mut io = IoConfigDriver::new(unsafe { hal::peripherals::IoConfig::steal() });
        for pin in [7u8, 9, 10, 11] {
            io.set_gpio_mux(pin, 3);
        }
        // SAFETY: SPI0/DMA singletons not otherwise held.
        let spi = Spi::new_spi0(unsafe { hal::peripherals::Spi0::steal() }, Config::default());
        let mut dma = DmaDriver::<Dma0>::new_dma(unsafe { hal::peripherals::Dma::steal() });
        dma.enable_controller();
        let chs = dma.split_channels().expect("DMA channels already claimed");
        let mut sd = spi.with_dma(dma);

        // transfer_dma: arms both channels, sets tdmae+rdmae, bounded-waits, invalidates RX.
        let (rx_buf, _tx_buf) = sd.transfer_dma(chs.ch0, chs.ch1, rx, &TX.0[..]).expect("SpiDma::transfer_dma failed");
        semihosting::println!("[spi-dma-fdx-lb] tx={:02x?} rx={:02x?}", TX.0, &rx_buf[..]);
        assert_eq!(&rx_buf[..], &TX.0, "SPI0 full-duplex DMA loopback mismatch — check GPIO9->GPIO11 jumper");
    }

    /// UART1 TX→RX loopback (uart.rs + io_config.rs). Send a byte on UART1 and read
    /// it back. **Requires a jumper GPIO15 (UART1 TXD) → GPIO16 (UART1 RXD).** UART1
    /// is used, not the UART0 console. We mux the UART1 pads (function 1), enable
    /// the RX input pad (IE), and ungate UART1's clock (CKEN_CTL1 bit 19 — unlike
    /// UART0 it is not on out of reset). TX and RX share the instance/divider, so
    /// the byte round-trips even if the absolute baud differs from nominal (#15).
    #[cfg(all(feature = "chip-ws63", feature = "hil-loopback"))]
    #[test]
    fn uart1_loopback_tx_to_rx() {
        use hal::io_config::{DriveStrength, IoConfigDriver, PinMux, PullResistor};
        use hal::uart::{Config, Uart};
        // Ungate the UART1 clock (CKEN_CTL1 bit 19). SAFETY: RMW-set of one
        // clock-enable bit, leaves the other clocks running.
        let crg = unsafe { &*pac::CldoCrg::PTR };
        crg.cken_ctl1().modify(|r, w| unsafe { w.bits(r.bits() | (1 << 19)) });

        let mut io = IoConfigDriver::new(unsafe { hal::peripherals::IoConfig::steal() });
        io.set_uart_mux(PinMux::Uart1Txd, 1);
        io.set_uart_mux(PinMux::Uart1Rxd, 1);
        io.configure_uart_pad(PinMux::Uart1Txd, DriveStrength::Strong, PullResistor::None, false, false);
        io.configure_uart_pad(PinMux::Uart1Rxd, DriveStrength::Strong, PullResistor::Up, true, true);

        // SAFETY: UART1 singleton not otherwise held; GPIO15->GPIO16 jumpered. TX and
        // RX share the instance/divider, so the byte round-trips regardless of the
        // absolute baud (the HIL crate runs no clock_init; whatever the boot clock
        // is, TX and RX agree). read_byte now gates on rx_fifo_cnt, so the drain and
        // the read pop correctly.
        // The HIL crate runs no clock_init; pass the boot-clock base so the divider
        // is sane (ch2 clock tree: UART=160 MHz only in normal operation; at boot
        // it's the ~40 MHz TCXO). TX and RX share the divider, so the byte round-trips
        // regardless of the absolute baud.
        let cfg = Config { clock_hz: Some(40_000_000), ..Config::default() };
        let uart = Uart::new_uart1(unsafe { hal::peripherals::Uart1::steal() }, cfg);
        while uart.read_byte(1).is_some() {} // drain stale RX (read_byte now pops via rx_fifo_cnt)
        let sent = 0x5Au8;
        uart.write_byte(1, sent);
        let mut got = None;
        for _ in 0..2_000_000u32 {
            if let Some(b) = uart.read_byte(1) {
                got = Some(b);
                break;
            }
        }
        // NOTE: the loop requires the jumper to be on the ACTUAL UART1 TXD/RXD pads.
        // The HAL pinmux (uart1_txd_sel/rxd_sel = 1) matches the vendor SDK, and
        // read_byte/rx_fifo_cnt is fixed; if `got` is None the physical TXD→RXD link
        // is open (UART1 uses dedicated `pad_uart1_txd/rxd_ctrl` pads — verify which
        // physical pins those are vs the jumper). On a correctly-wired board the byte
        // round-trips.
        let got = got.expect("UART1 RX got nothing — verify the jumper is on the real UART1 TXD/RXD pads");
        assert_eq!(got, sent, "UART1 loopback mismatch: sent 0x{sent:02x} got 0x{got:02x}");
    }

    /// **P4 gating silicon proof**: does the DMA completion IRQ (DMA_INT = IRQ 59)
    /// fire for a *peripheral-paced* (mem→peri) single-block transfer the same way
    /// it does for mem-to-mem? Configures a SPI0 TX DMA channel with
    /// `transfer_int = true`, arms it, then `block_on(wait_transfer_done(0))` — which
    /// parks on `wfi` until IRQ59 wakes it. If this returns (rather than hanging),
    /// async `.await` DMA is viable on this silicon. **Requires the GPIO9→GPIO11
    /// jumper.**
    #[cfg(all(feature = "chip-ws63", feature = "hil-loopback", feature = "async", feature = "unstable"))]
    #[test]
    fn spi_dma_irq59_fires_on_completion() {
        use hal::asynch::block_on;
        use hal::dma::{BurstSize, Dma0, DmaChannelConfig, DmaDriver, DmaPeripheral, FlowControl, TransferWidth};
        use hal::interrupt;
        use hal::io_config::IoConfigDriver;
        use hal::spi::{Config, Spi};

        const N: usize = 8;
        #[repr(C, align(32))]
        struct Aligned([u8; N]);
        static TX: Aligned = Aligned([0xA5, 0x3C, 0x00, 0xFF, 0x5A, 0xC3, 0x0F, 0xF0]);

        let mut io = IoConfigDriver::new(unsafe { hal::peripherals::IoConfig::steal() });
        for pin in [7u8, 9, 10, 11] {
            io.set_gpio_mux(pin, 3);
        }
        let _spi = Spi::new_spi0(unsafe { hal::peripherals::Spi0::steal() }, Config::default());
        let r = unsafe { &*hal::peripherals::Spi0::ptr() };
        unsafe {
            r.spi_dtdl().write(|w| w.bits(4));
            r.spi_drdl().write(|w| w.bits(0));
        }
        unsafe { hal::cache::clean_range(TX.0.as_ptr() as usize, N) };

        let mut dma = DmaDriver::<Dma0>::new_dma(unsafe { hal::peripherals::Dma::steal() });
        dma.enable_controller();
        // transfer_int = TRUE so the channel's done bit sets int_tc → dmac_int_st → IRQ 59.
        let cfg = DmaChannelConfig {
            src_peripheral: 0,
            dst_peripheral: DmaPeripheral::Spi0Tx.request_id(),
            flow_control: FlowControl::MemToPeripheral,
            src_width: TransferWidth::Width8,
            dst_width: TransferWidth::Width8,
            src_burst: BurstSize::Beats1,
            dst_burst: BurstSize::Beats1,
            src_inc: true,
            dst_inc: false,
            transfer_int: true,
            error_int: false,
            bus_lock: false,
        };
        dma.configure_channel(0, TX.0.as_ptr() as u32, 0x4402_0060, N as u16, &cfg);
        r.spi_dcr().modify(|_, w| w.tdmae().set_bit());

        // Enable global MIE so wfi wakes on IRQ 59. wait_transfer_done enables the
        // DMA_INT IRQ itself (and raises LOCIPRI). If IRQ59 never fires, block_on
        // hangs — the test runner's timeout catches that as a failure.
        unsafe { interrupt::enable_global() };
        block_on(async { dma.wait_transfer_done(0).await });
        unsafe { interrupt::disable_global() };

        r.spi_dcr().modify(|_, w| w.tdmae().clear_bit());
        semihosting::println!("[spi-dma-irq59] IRQ 59 fired for peripheral DMA completion — async .await is viable");
    }

    /// **P4 async capstone**: `SpiDma::write_dma_async` via `block_on` — the
    /// ergonomic async API on silicon. Drives MOSI via DMA parking on IRQ 59 (wfi)
    /// until completion. **Requires the GPIO9→GPIO11 jumper + the `async` feature.**
    #[cfg(all(feature = "chip-ws63", feature = "hil-loopback", feature = "async", feature = "unstable"))]
    #[test]
    fn spi_dma_write_async() {
        use hal::asynch::block_on;
        use hal::dma::{Dma0, DmaDriver};
        use hal::interrupt;
        use hal::io_config::IoConfigDriver;
        use hal::spi::{Config, Spi};

        const N: usize = 8;
        #[repr(C, align(32))]
        struct Aligned([u8; N]);
        static TX: Aligned = Aligned([0x99, 0x88, 0x77, 0x66, 0x55, 0x44, 0x33, 0x22]);

        let mut io = IoConfigDriver::new(unsafe { hal::peripherals::IoConfig::steal() });
        for pin in [7u8, 9, 10, 11] {
            io.set_gpio_mux(pin, 3);
        }
        let spi = Spi::new_spi0(unsafe { hal::peripherals::Spi0::steal() }, Config::default());
        let mut dma = DmaDriver::<Dma0>::new_dma(unsafe { hal::peripherals::Dma::steal() });
        dma.enable_controller();
        let chs = dma.split_channels().expect("DMA channels already claimed");
        let mut sd = spi.with_dma(dma);

        unsafe { interrupt::enable_global() };
        block_on(async { sd.write_dma_async(chs.ch0, &TX.0[..]).await }).expect("SpiDma::write_dma_async failed");
        unsafe { interrupt::disable_global() };
        semihosting::println!("[spi-dma-async] SpiDma::write_dma_async ok for {N} bytes");
    }
}
