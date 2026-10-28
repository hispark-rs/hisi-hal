# WS53 integration notes (historical)

These notes preserve earlier implementation and validation context. API safety contracts live in rustdoc; current qualification status belongs to the integration project evidence ledger. Historical observations do not imply that a feature is stable or fully validated.

WS53 currently requires `chip-ws53,unstable`. UART/TIMER and basic GPIO drivers attach to an
audited boot configuration after explicit application-core ownership handoff.
Other MCU peripherals remain under implementation; this is not general WS53
silicon graduation or qualification of the SFC HAL's Flash-programming path.

WS53 `uart::Uart::initialize_h0` accepts the unclaimed UART1 reservation directly.
Under an explicit exclusive UART/source/pad/shared-word handoff, it enables the
measured H0 bit4, fences, waits an independent10us observation margin and checks
the complete halfword before touching UART registers. It then uses the measured
reset/typed-line path and rechecks the clock. Local IRQ41 enabled/pending and an
asserted H0 reset are refused first; peripheral-IRQ/TX-busy refusals occur after
gate admission. `H0InitFailure` retains the unclaimed token. No failure or Drop
rolls back clocks, resets a peer, or returns DMA memory; platform recovery remains
explicit. It neither configures upstream clocks/pads nor proves bus drain/cold
startup. UART0/2 gate identities remain unsupported, and `from_configured` is
unchanged. The whole WS53 surface remains behind `chip-ws53,unstable`.

The experimental WS53 `wdt::WatchdogDriver` accepts an explicitly handed-over
`Peripherals::WDT` token and a `wdt::Config` constructed from the actual nonzero
watchdog clock, SDK timeout seconds and expiry mode. It does not change shared
clock/reset or NMI routing. `start`/`stop` require an `embedded_hal::delay::DelayNs`
provider for bounded unlock polling; snapshot polling is bounded separately.
`feed` uses the WS53 magic key and rejects a locked bank. Snapshot errors are
distinct from a valid zero count, and `CounterSnapshot::micros` uses u64.
Dropping the driver leaves the watchdog state unchanged. NMI handlers must use
the documented unsafe static methods without borrowing the interrupted driver.
The SDK LOAD conversion discards eight bits whose hardware initial value is not
known, so requested seconds are not an exact expiry guarantee. Host operation
traces are checked against fixtures generated from pinned vendor C control flow.
QEMU control/fault tests use an explicit assumed-timing model; physical timing,
CPU NMI delivery and SoC reset verification remain open.

The experimental WS53 `tcxo::TcxoDriver` takes a handed-over `Peripherals::TCXO`
token and a nonzero status-poll budget. Construction performs no MMIO; enable
preserves the inherited epoch and clearing it requires an explicit call. Failed
snapshots are errors, never a fabricated zero count. `CounterTicks` exposes raw
64-bit ticks; duration conversion requires an externally established frequency.
All commands require mutable access, but this cannot exclude an independently
running control core, NMI, DMA or vendor user: the unsafe handoff must establish
that exclusion for the entire ownership period. Drop leaves hardware unchanged.
The four QEMU HAL scenarios cover normal/persistent command readback, stopped
clock and stalled snapshot. The SDK reference 32 MHz is not a measured board
rate, and its application selects ROM routines rather than the C helper oracle.
Clock/reset provisioning and low-power compensation remain unverified.
The explicit inherited-clock Embassy driver and its bounded HIL evidence are
described below; neither it nor TCXO claims a calibrated physical clock.

WS53 `pwm::PwmDriver` owns the complete PWM block, four commit groups and both
shared IRQs. It configures the SDK divider16 clock profile after explicit unsafe
ownership handoff, including upstream clock validity and serialization of the
shared clock halfwords. DMA is disabled. High count halves must already be zero;
only the source-profile low halves are written. A-board MIO21/SDK47/mode1/PWM0P
has three-run continuous/duty and local-divider waveform evidence; other channels,
finite/IRQ timing and the upstream source identity remain unqualified.

`ClockDivider` accepts only measured Div1/2/4/8/16. `set_clock_divider` rejects
changes before MMIO if any group is running, even after a latched completion
event. Stop every group first. Constructor default remains Div16. In the bounded
A-board takeover, the effective input is32MHz: divisor16 produces2MHz counter
rate, whereas the SDK reports a fixed32MHz. `SDK_REPORTED_CLOCK_HZ` records that
source claim, not a clock guarantee; the old `PWM_CLOCK_HZ` alias is deprecated.
No upstream/PLL clock is changed and no glitch-free live switching is promised.

`Waveform` rejects zero/overflow or totals above255 and offset greater than low;
`PulseCount` accepts1..32767. These are source API limits, not measured hardware
widths. Group configuration rejects duplicate/overlapping channels, missing
routes and reconfiguration while running before any peripheral write. Each
channel retains its own Continuous/Finite/Repeating policy; mixed groups stage
EN=0 only for finite channels between two START commands with an independent1us
delay. Restart requires an explicit stop, even after a finite completion event.

Output routes consume `PadControl` through unsafe `OutputRoute::from_configured`:
actual pinmux/channel association must be established externally, since the
complete per-pad PWM signal map has not been qualified. PWM0P on SDK47/mode1
has board evidence; SDK source also identifies MGPIO10/mode4 and AGPIO2/mode2. The driver retains each route
until its group is stopped/cleared and the route detached. It never guesses board
MIO/AIO mappings or remuxes SWD pins. This is not automatic PWM pin provisioning.

Single-channel continuous/repeating groups expose `embedded_hal::pwm::SetDutyCycle`;
duty updates validate the phase constraint and use a group refresh, restarting
phase. Finite or multi-channel groups use explicit group configuration instead.
Group operations suspend only the two PWM sources, leaving unrelated IRQs usable
through delays; priority, enable and pending state are restored. ISR status/ack
uses no mutable driver borrow or callbacks. Events are bitmaps and may coalesce.
Stopping a group does not disable another group's IRQs. Controller Drop masks
both sources, stops groups, acknowledges events and gates the PWM clock while
preserving unmodeled clock-control bits. Register/pad clocks and independent delay
must survive this cleanup. Thirteen host state/ordering tests pass; actual PAC
execution and QEMU waveform/IRQ/fault tests pass. A-board single-pad/divider
results do not qualify automatic pin provisioning or the complete peripheral.

WS53 `rtc::RtcDriver` accepts independently handed-over `RTC0` or `RTC1`, an
owned `DelayNs` provider and a nonzero snapshot-poll budget. It implements the
reference two-channel **32-bit, 32768 Hz** profile, not the older BS2X 64-bit
reader. Construction performs no MMIO. Starting cancels the old alarm, waits
32 us after stop, configures the typed mode/load, enables counting and waits
65 us. Drop uses the same explicit stop/mask/acknowledge sequence; the independent
delay source must stay functional until that cleanup finishes.

`RtcTicks::from_duration` rounds up and rejects zero/overflow; it does not use
the SDK's approximate multiplier or truncate a u64 load. `read_count` returns
Disabled, SnapshotTimeout, Expired or a valid u32 (including zero). It never consumes
counter data after timeout. Snapshot request and pending check suspend only the
owned channel's local IRQ: stale ENABLE written after one-shot expiry must not
rearm the counter. Polling runs outside global critical sections, so unrelated
IRQs retain their prior delivery state. The source enable state, priority and
pending event are preserved; no clear-pending or priority-reset helper is used.
An expired one-shot is stopped without acknowledging its raw event; the 32 us
wait occurs after restoring interrupt state, so the original IRQ is preserved.
`unmask_interrupt` preserves pending events; install
the matching handler before explicitly enabling its local cause 49/50. The
handler calls unsafe `rtc::acknowledge_interrupt(channel)` without borrowing an
interrupted mutable driver. `mask_interrupt` disables the local IRQ but retains
the raw peripheral event; `stop` additionally clears it after synchronization.

The mask field is provisionally treated as active-high from the SDK name and
timer-family convention, not an RTC source-helper or silicon proof. Shared
clock/reset/common-bank ownership, board clock availability, physical mask and
expiry behavior still need acceptance. Host tests establish operation ordering,
conversion boundaries and error handling. An explicit-assumption QEMU model
and six Rust RAM scenarios cover both channels, IRQ49/50, pending stop/Drop,
snapshot faults and the expiry RMW interleaving. Physical propagation timing,
clock-domain synchronization and silicon recovery still need qualification.

GPIO bank tokens and pad ownership require separate unsafe handoffs.
`pinctrl::PadControl::claim` accepts exclusive ownership after existing users and
output drivers are quiesced and clocks are guaranteed. Its typed `Config`
validates pad-specific mux/drive settings, pull, input-enable and Schmitt control;
PAC field writes preserve reserved bits and update both extended-drive fields
together before changing mux. `into_gpio` consumes the owner after reading back
GPIO mode and input-enable. SWD/reset claims are rejected before MMIO. Clocks,
GPIO_SW_CTL source selection and control-core ownership negotiation remain
external preconditions; two register writes are not electrically atomic.
`gpio::GpioDriver` borrows one `Input`/`Output` per bank to serialize configuration;
different banks can run concurrently. The drivers implement `embedded-hal`
digital traits and preserve unrelated control bits. SET/CLR/EOI commands are
never read. Input notification supports rising/falling/both edges and high/low
levels with debounce disabled. The ISR snapshots and acknowledges status, masks
the bank's local IRQ, and requires explicit rearming after service. Cancellation
clears the peripheral source and local pending latch. Output Drop disables its
digital driver. SWD and reset pads cannot be claimed as `AnyPin`.

These paths pass RAM QEMU tests, including real ISR entry/return and cancellation
of an already pending edge, plus all 28 allowed pad configurations and ownership
transfers. Physical clock/pad provisioning and GPIO HIL remain
open; the constructor does not make an unconfigured or shared pin safe to use.

`soc::ws53::pins` distinguishes chip pad names, canonical pads, SDK IDs and
GPIO controller bits. These are descriptions, not ownership tokens. For example:

```rust,ignore
use hisi_hal::soc::ws53::pins::{Mgpio, Pad};

let pad = Pad::from_mgpio(Mgpio::from_index(1).unwrap());
assert_eq!(pad, Pad::Agpio6);
assert_eq!(pad.sdk_id().number(), 38);
assert_eq!(pad.gpio_line().unwrap().bit(), 6); // GPIO1, not GPIO0 bit1
```

Board `MIO/AIO` labels need a separate board mapping. Reset has no GPIO line;
SWD pads remain marked as debug pads. Pad-specific mux and drive validation
rejects unsupported encodings before configuration writes. These metadata values
do not grant shared resources; actual configuration requires `PadControl`.

The host tests compare all canonical pads and aliases with a checked-in fixture.
To recheck that fixture against the pinned SDK and generated WS53 SVD:

```sh
uv run --script scripts/check-ws53-pins.py --sdk /path/to/fbb_ws53 --svd /path/to/ws53-pac/svd/WS53.svd
```

This optional check compiles SDK data declarations on the host, performs no
MMIO, and requires neither an attached board nor SDK files during normal builds.

WS53 `i2c::I2c::new_i2c0/1` implements an experimental blocking master over the
source-audited PAC, with typed instance/pad ownership, seven/ten-bit addresses,
validated timing and a nonzero poll budget. Own the 32MHz source, inherited SDA
hold/setup configuration and complete M_CLKEN1 halfword coordination before
handoff. `Pins<T>` requires externally verified SCL/SDA open-drain routes and
pull-ups; it does not guess a board pin map or perform a scan.

Adjacent equal-direction embedded-hal operations concatenate; changes use
repeated START and only the final byte requests STOP. Empty buffers produce no
commands. Reads keep at most one read command outstanding, including operations
longer than the eight-byte FIFO. Source count overflow and unachievable minimum
counts are rejected: 1MHz is not accepted under the current 32MHz formula,
although the SDK silently programs minimum mode3 counts for that request.
This is a count-profile limit, not silicon frequency qualification.

All waits are bounded and outside global critical sections. Every transaction
finishes with disable/inactivity confirmation; errors retain address/data NACK,
arbitration or FIFO detail. Failures sampled around STOP/disable are retained
before write-one clear-all. Failed shutdown poisons the instance; `recover()`
must succeed before reuse. `try_free()` returns pads/token only after confirmed
shutdown, returning the still-owned driver on error. Drop is bounded best effort:
if shutdown cannot be verified it leaves the clock enabled for external recovery
and does not promise released bus lines. Keep resources owned and use `try_free`
when that confirmation matters. No shared reset or pin-clock recovery is attempted.

These semantics have host fault-injection, compile-time and real PAC execution
evidence in an explicitly assumed QEMU profile (two buses, long/combined reads,
address NACK recovery, stopped enable clock and ownership return). The profile
assumes CON addressing selection, deferred read ACK and 1us disable crossing;
these remain unverified on silicon. IRQ-driven/async, slave, DMA, sleep coordination,
electrical bus recovery and physical 1MHz capability remain pending. The legacy
WS63/BS2X implementations and public interfaces remain separate and unchanged.

WS53 `spi::Spi::new_spi0/new_qspi1` provides experimental blocking single-line
Motorola master buses. The two controllers use separate v151_100 PAC layouts,
including their different FIFO addresses, status bits and DMA-enable positions.
Typed instance/pad ownership and an explicit 32MHz source/XIP handoff are required.
`Pins<T>` takes preconfigured exclusive SCLK/MOSI/MISO routes; hardware CS must
be unrouted, with a separately owned software CS framing device transactions.
The driver does not discover board wiring or promise native-CS continuity.

Only source-qualified `u8` and `u32` frame configurations are exposed. `SpiHz`
chooses the smallest legal even divider whose exact profile rate does not exceed
the request; out-of-range requests are rejected. For example, a 10MHz request
uses divider4 (8MHz), instead of the SDK's integer divider3. `actual_hz()` is
the rounded-down source-profile rate, not a measured electrical frequency.
Each word drains its reply before queuing the next; the 36 register aliases are
not treated as a FIFO-depth specification. Unequal transfers clock the longer
buffer, send zero for missing TX words and discard excess RX words.

All progress waits are bounded and outside global critical sections. Successful
operations confirm idle, drain replies and disable the controller. Error status
is sampled before and after shutdown before read-clear acknowledgement. Failed
cleanup preserves the original cause and poisons the instance until `recover()`
succeeds. `try_free()` retains the driver on failed shutdown; Drop is best effort
and cannot promise inactive pins if shutdown fails. Clock gates stay enabled
even after successful release: the required sleep acknowledgement and safe
shared-XIP shutdown are not yet qualified.

The SPI engine has host fault-injection and compile-fail tests plus RISC-V
compilation. Real HAL/PAC execution passes in the explicitly assumed QEMU
profile: both controllers, word widths/modes, long/unequal/in-place transfers,
resource return/reacquisition and stopped-clock poisoning. Physical loopback
and timing remain pending; the model's FIFO8 and 1us clock crossing are assumptions.
Dual/quad protocols, native-CS transactions, DMA, interrupt-driven/async access
and low-power clock shutdown remain unimplemented. This is partial experimental
SPI/QSPI support, not a completed or stable peripheral capability.

WS53 `tsensor::TempSensor` implements the source port's polling profile. The
`TSENSOR` reservation requires explicit thermal-service, analog power/reset and
inherited 32kHz clock handoff; the driver does not guess missing clock registers.
CONTROL=0 performs the five SDK initialization writes; CONTROL=3 attaches without
restarting; other values fail unchanged and retain the driver. Initialization
readback checks configuration, not whether the sampling clock is running.

`try_read()` takes one coherent READY/code snapshot. `read_bounded()` accepts a
nonzero attempt budget and an independent delay provider (10us between attempts).
Timeout leaves the sampler running and permits retry. Repeated READY samples may
be stale; there is no evidenced clear/stop/fresh-conversion/IRQ API. Drop leaves
hardware untouched. `release_without_stopping()` transfers the still-running or
unknown state and its clock/power obligations back with the peripheral token.

`SampleCode` preserves all ten raw bits. `nominal_temperature()` only maps the
SDK reference endpoints code118..933 to -40..125C, with centidegree representation
and explicit signed truncation. Outside the reference range it returns None
instead of the SDK's upper-only saturation; the raw sample remains available.
This is not per-device calibration or evidence of physical accuracy/rated range.
The original pinned C port supplies an exhaustive 1024-code test oracle. The
HAL's source-contract, 12 host behavior tests, two compile-fail boundaries and
RISC-V compilation pass. Real HAL/PAC execution passes in an explicitly assumed
QEMU profile for reference/out-of-reference raw codes, clock timeout/retry and
unknown CONTROL rejection. QOM counters verify reattachment does not restart.
The model's averaging cadence/sticky READY are assumptions; physical measurements,
calibration, clock ownership and thermal-service handoff remain pending.

WS53 `i2s::I2sStream` is an experimental polling stereo interface. The I2S
reservation requires exclusive block/IRQ/DMA ownership, the source FIFO8
profile and serialized access to complete AUDIO_CMU/M_CLKEN0 words (including
other halfword users). `Pins::from_configured` retains four distinct physical
pad tokens and records master/slave clock direction; a mismatch with Config
fails before any MMIO, including Drop. The driver releases the shared audio
divider reset without pulsing it, enables the word gate and resets only I2S.

`DataWidth` uses the original C enum1..5; zero is reserved. `BclkDivider` accepts
only1..127 and master mode requires it; slave mode cannot inherit an accidental
zero-divider master configuration. FS period/ratio derive from two channels and
the selected sample width. These are register counts, not a measured rate;
upstream frequency and electrical timing still require a board handoff.

TX/duplex prefill1..8 pairs before clock/start, then `try_queue`/`queue_frames`
report FIFO progress. RX checks both FIFO depths directly, so short bursts
below the IRQ threshold can be read. Buffers are never retained and operations
return partial pair counts on failure. Invalid TX sample bits are rejected
before queuing, not silently truncated. Errors/timeouts/configuration loss
stop the stream best-effort and poison it until explicit `discard_and_recover`;
failed recovery retains all resources. CPU IRQ67 stays masked; error sources
remain observable, with threshold events masked and no callback/ISR API.

`wait_tx_fifo_empty` does **not** mean the shift register or wire is idle. There
is no safe `flush`/`free` promise and no fixed295us completion delay copied from
the SDK. Drop/reset recovery can truncate samples. Shared clocks remain on;
pad/token reclamation requires `unsafe release_after_external_quiescence`.
Twenty host tests, three compile-fail boundaries and host/RISC-V Clippy pass.
Actual Rust/PAC RAM streams pass20 width/role/edge combinations in the explicit
QEMU profile, including data-dependent replies, short RX, FIFO-empty and stopped
clock timeouts. Configuration recovery alone does not prove the upstream clock
runs. Physical audio tests are pending. Multichannel/PCM,
merged FIFO/DMA, interrupt-driven/async and low-power paths are not implemented;
this is not complete or stable I2S support.

WS53 `adc::Calibrator` is the first, explicitly partial v155 ADC layer. It consumes
an unsafe application-core handoff, enables the source-profile APB/analog gates,
pulses ADC reset and applies the SDK analog configuration. Calibration has three
bounded stages (automatic offset, corrected manual offset, capacitor), checks
CAP_ERROR and configuration readback, and requires completion status to clear
before each stage. Failed attempts invalidate the previous report and request
both calibration enables off. No global critical section encloses waiting.

The whole ADC/IRQ80/reset and shared clock RMW ownership must be negotiated;
old scans, CPU trace and MCU_DIAG writers must already be quiesced externally.
Construction performs no MMIO; `calibrate` is explicit. It does not configure pads,
start a scan, touch diagnostic addresses or borrow buffers. Shared clocks and
analog power remain enabled after calibration/handback. No physical shutdown,
calibration accuracy or safe RAM-write drain guarantee is inferred from readback.

`Sample::from_capture_word` decodes an already copied word into a typed channel
and12-bit code while preserving the raw word's reserved bits. It neither starts
nor proves a conversion and does not turn the SDK1800mV nominal value into a board
voltage limit. The source oracle executes the original C over host RAM and exposes
its timeout-success/free-with-diagnostic-enabled behavior; that behavior is not
copied into Rust. Ten new host tests and compile-fail boundaries pass. Calibration
also passes the explicit QEMU timing/fault model and actual Rust/PAC RAM fixture
(eight boundary codes repeated20 times, plus five fault scenarios). These do not
prove physical calibration. Physical ADC acceptance remains open.

WS53 `adc::capture` adds experimental one-shot capture. It consumes the calibrated
ADC, a separately handed-over `MCU_DIAG` token, a nonempty set of owned analog pad
routes and a static coherent DTCM allocation. Range checks reject overflow and
non-DTCM destinations; the exact inclusive-end/source-profile semantics require
an explicit unsafe handoff and have not been qualified on hardware. All register
access uses generated PAC fields. No ordinary cargo build generates registers.

`wait` reports status observations, not readable sample memory. Success, timeout,
configuration errors and cancellation retain the allocation. Drop requests stop
but cannot reclaim memory; forgetting the handle cannot invalidate the static
allocation. Only `unsafe release_after_external_quiescence` returns resources,
after independent writer-drain/coherency proof; it also invalidates calibration.
There is no borrowed stack buffer, safe automatic release or invented cache flush.
The coherent-memory constructor excludes unmaintained cache aliases explicitly.

Ten additional host tests and three compile-fail boundaries cover the capture
state machine. Seven actual Rust/PAC QEMU scenarios independently inspect RAM,
guard words and pending writes, including early DONE, bus/sample faults, Drop
and forget. The host supplies a model-only drain handshake before guest handback;
QOM counters are not silicon APIs. Physical profile/cache/drain qualification,
shared trace/control-core handoff, repeat capture, interrupt/async and low power
remain open; this is still partial, unstable ADC support.

WS53 `dma` provides experimental eight-channel, same-width memory transfers
and `dma::uart::{UartDma, UartTransfer}` for UART0/1 byte requests. UART2 has no
valid SDK DMA route and is excluded by a sealed trait. Channels and coherent
static buffers are owned tokens; no safe raw port, address or channel input is
accepted. UART DMA reserves the entire configured UART for one direction, after
an unsafe handoff of its requests, IRQ, clocks and pads. It does not invent a
writable DMA_EXTRA bit or resolve the SDK's conflicting HPROT constants.

Start rejects unsupported capability/configuration, queued data and a busy
serializer. Polling and transmitter-idle waits have finite budgets. DMA completion
is distinct from wire completion, and neither proves bus drain. Success, error,
cancel, Drop and forget retain the static allocation and peripheral owners.
Only `unsafe release_after_external_quiescence` permits resource return after
independent drain and FIFO/error-history recovery. Polling can miss errors on
characters consumed by DMA: an empty line-error report is not integrity proof.

Twenty DMA host tests and nine compile-fail boundaries pass. Actual Rust/PAC
QEMU fixtures cover 24 memory cases (three widths) and 76 UART cases, including
FIFO pressure, late writes, abandoned transfers and serializer stalls. Physical
request timing, byte lanes, clocks, cache and cross-core ownership remain
unverified. Non-UART/SPI requests, LLI, SDMA, full-duplex UART ownership splitting,
complete RX error capture and peripheral async remain open. Memory-copy async IRQ58 is described below. All WS53 APIs require
`chip-ws53,unstable`; these results do not graduate any API to stable.

WS53 `dma::spi::{SpiDma, SpiTransfer}` adds single-line full-duplex SPI0/QSPI1
u8/u32 transfers. An unsafe configured-master handoff reserves the complete
peripheral, IRQ, clocks, external CS protocol and typed SCLK/MOSI/MISO routes.
Two unique channel tokens and disjoint, equal-length coherent static buffers
remain owned together. Initial busy/configuration/error guards precede mutation;
RX then TX arm with requests disabled, followed by request enable/readback.
The source profile uses request6/7 or4/5, master0/protection2 and levels4/0.

One finite wait budget checks both channels, SPI errors, residual RX and serializer
idle. Failures, cancel and Drop disable both peripheral requests and request both
channels halted; no FIFO flush, shared reset, clock shutdown or CS release occurs.
Completion, cancellation and forget do not reclaim memory or prove bus drain.
Only external drain/coherency/FIFO/CS recovery permits unsafe handback. Ten host
tests and two compile-fail boundaries cover ownership and ordering. Two actual
Rust/PAC QEMU fixtures pass100 scenarios across both widths, eight channel pairs,
both sides' errors/stalls and abandoned transfers. No physical SPI DMA acceptance,
in-place transfer,16-bit/packing,burst/LLI,receive-only clock generation or async
support is implied. Existing blocking SPI does not silently switch to this API.

WS53 `dma::i2c::{I2cDma, I2cTransfer, Commands, Burst}` adds experimental
command-driven write/read/write-then-read operations. `Commands` owns a static
halfword buffer; builders validate lengths before mutation, emit exactly one
final STOP and a repeated START only at the write/read boundary. Pure writes
own one DMA channel; reads additionally own a byte receive buffer and channel.
No caller-provided raw command stream or stack-local command is accepted.

`unsafe from_configured` hands over an enabled idle master, typed address, pins,
requests, IRQ and clocks; this adapter does not configure a board or switch a
live blocking driver. TX uses halfword commands/protection1, RX bytes/protection2.
Both one/four-beat profiles are explicit assumptions. Four-beat RX starts at
threshold3 and lowers to0 for the final1..3 bytes, with register readback. The
handoff must establish real FIFO/lane/threshold/burst behavior and coherency;
source enums and simulation are not physical qualification.

One bounded wait checks both channels independently, I2C errors, residual RX,
final STOP and bus idle. All exits retain the bus, pads, channels and buffers;
request-stop/Drop disable requests and halt attempted channels without flushing
FIFO or resetting shared resources. A posted write may still arrive. The only
handback is unsafe after external drain and bus recovery. `forget` leaks owners
safely and does not reconstruct them. Ten host tests and 166 actual Rust/PAC QEMU
scenarios cover both ports/eight channel pairs, pure writes with peer readback,
reads/repeated START, short tails, failures and cancellation. Hardware, automatic
clock/address configuration, IRQ/async and stable status remain unverified.

The shared WS53 DMA engine uses `fence iorw, iorw` immediately before enabling
its channel, in addition to a compiler release fence. This orders command-buffer
stores and channel MMIO, but does not establish cache coherency or peripheral
bus drain; the unsafe allocation/handoff contracts still apply.

WS53 `dma::i2s::{I2sDma, I2sTransfer, Samples, Burst, Progress}` is an experimental
merge-DMA implementation with ten host tests and 669 actual Rust/PAC RAM QEMU
scenarios covering widths, bursts, channels, master/slave, independent directions,
duplex receive windows, short tails, single-direction faults/cancellation and
99 duplex fault cases (either side, including channel7/0 wrapping).
This validates the explicit model assumptions, not physical hardware.

An unsafe handoff reserves a configured, stopped, reset-released stereo block,
routes and clocks. `Samples` accepts initialized static word pairs and rejects
bits outside the chosen sample width. Optional TX/RX allocations have independent
even sample counts, so the caller can explicitly supply additional TX frames
around a shorter receive window. No hidden padding is generated. Requests12/13,
word access, protection1 and burst1/4/8/16 follow the audited/assumed profile.
Actual packing, lanes, FIFO watermarks and ownership require board qualification.

Start guards channels/peripheral before mutation, configures merge and thresholds,
and arms RX before TX. `wait_dma` starts the serializer only after a complete TX
pair is prefetched (or immediately for receive-only). It bounds polling, checks
both DMA errors and I2S status, and lowers a short receive tail's threshold.
`Progress` reports TX/RX **samples moved**, never wire completion. Independent
stream clocking can still produce FIFO under/overflow, which remains an error.

Success, failure, request_stop and Drop halt attempted DMA channels without
resetting the peripheral, flushing FIFOs, disabling merge or gating clocks.
Requests and the serializer can remain active, and posted writes can arrive.
All tokens and allocations remain retained; only independent unsafe recovery can
return them. Ten new host tests cover framing, ownership, startup ordering,
bounded prefill, configuration/error paths, short tails and Drop/forget/unwind.
There is no automatic live-PIO conversion, async/IRQ or stable API claim.

WS53 `trng::TrngDriver` provides experimental configured reads and SDK ring
startup. Pair reads, byte fills and startup have controlled RAM observations;
production handoff and entropy qualification remain pending. QEMU uses an
explicit deterministic test source, never an entropy claim.
`Peripherals::TRNG` is `Unclaimed`, and its unsafe
handoff must exclude SDK application/control-core random-buffer services and
security consumers. `from_configured` does not access registers or initialize
the source; it inherits clock/reset/source configuration and settling. Release
and Drop leave the source unchanged.

Unsafe `start_sdk_ring(token, delay)` enables only the existing16-bit ring bit,
preserves other handed-off bits, fences, waits at least200us even if already
enabled and checks the complete control word. `RingStartFailure` retains the
reservation on a readback error, without rollback. This implements the pinned
application's RING_ENABLE path, not optional FRO/CLK_OPEN or upstream SoC clock,
power and entropy-source setup. Those conditions must be independently handed
over and kept stable. No FIFO data is consumed during startup.

Two startup host tests cover every16-bit initial word and readback faults.
The same ELF exercises zero-to-enabled and already-enabled starts in QEMU
without host priming and in three controlled RAM runs; each also checks13
exclusive random-buffer image cases and restores the factory application.
The controlled SDMA drain/ownership limitation remains; these results do not
qualify a production peer lease, full source initialization or Flash/cold boot.

Reads check the ring and both ready bits, then repeat the ready check before
consuming a candidate/comparison pair. Zero/all-ones/equal candidates are
discarded; rejected values are never returned as successful output. This is
the pinned SDK filter, not an entropy health proof or a cryptographic RNG trait.
The hardware guarantee admitting two FIFO reads remains a handoff qualification.

`read_word` uses a finite attempt budget and 1us delays between failed attempts.
`fill_bytes` shares one total budget, uses explicit little-endian words, discards
unused tail bytes and reports `FillError::written` on partial failure. The
remaining destination is unchanged. Empty slices succeed without MMIO. No IRQ,
IPC negotiation, entropy guarantee or WS63/BS2X stable API is inherited.

WS53 `efuse::{EfuseDriver, EfuseByteAddress, EfuseError, ReadError}` is an
experimental read-service draft with eight host tests and 111 actual Rust/PAC
QEMU cases against immutable synthetic contents; hardware execution is pending. `EFUSE` starts as an
`Unclaimed` reservation. An unsafe configured handoff must exclude competing
boot/security/programming users and establish safe inactive programming supplies,
completed region0 boot, settled source timing 0x1f and read mode 0x5a5a.

Logical addresses are typed and limited to 256 bytes; the complete buffer range
is checked before MMIO. Each byte read validates the inherited profile, reissues
only the SDK's volatile read-mode command while preserving upper16 control bits,
and checks readback/profile before publishing the byte. It never writes OTP data,
AVDD, programming switches or security provisioning. Errors retain the successful
prefix and return its length; the remaining bytes are unchanged. Empty reads do
no MMIO, and no global IRQ lock encloses the loop. Construction/release/Drop do
not initialize or stop the controller. Die-ID/MAC/security schemas remain unresolved.

WS53 H1/H0/L0 reinitialization is available as unsafe `reset_and_configure` on
`Uart<Uart0>`/`Uart<Uart1>`/`Uart<Uart2>`. It rejects held reset, enabled peripheral IRQs
and active TX, pulses the PAC halfword reset field, restores a validated
`LineConfig`, enables FIFOs and selects typed `SignalPath`. It does not provision
clocks/pads or negotiate cross-core ownership. Local IRQ/DMA must be quiesced
and shared reset-word RMW externally serialized. UART2 selects its separate GLB_CTL halfword; UART0/1 use M_CRG. A-board qualification covers internal-loopback recovery
and neighbor scratch/frame/modem isolation, plus UART2 return to inherited
console Pins. No new pad configuration or physical frame-timing claim is made.


WS53 `chip-ws53,unstable,async` now exposes an interrupt-driven RX-only adapter:
`unsafe { uart.async_receiver() }` exclusively borrows a configured UART and
returns `AsyncReceiver` implementing `embedded_io_async::Read`. Bind this UART's
ISR to `Uart::<T>::on_async_receive_interrupt` on the owning application hart;
keep clock/handoff and IRQ routing valid, and enable global delivery externally.
The constructor rejects any existing peripheral/controller interrupt enable.
The adapter is neither Send nor Sync; it does not establish multicore ownership.

A nonempty read returns one byte or the existing `ReceiveError`; an empty read
returns zero without arming. A pending read consumes no data and leaves its
buffer untouched. Waker registration and check/arm/recheck run under the local
critical section. Old waker destruction and ISR wake callbacks happen outside
the driver's critical section. Dropping a pending future masks RX and the local
IRQ, clears its pending latch and removes the waker, preserving queued data and
read-clear errors. Dropping the adapter performs the same cleanup. Cancelled
reads can be restarted; forgetting a future is not a cancellation mechanism.

The `uart2-async-smoke` RAM example completed three same-ELF UART2 hardware/QEMU
runs with external7-byte input, wake-driven progress, cancellation, empty reads,
and two takeover refusal cases. Each run restored vendor firmware via reset.
Generic UART0/1 async instantiations and error behavior need further target
coverage; UART2 bad-frame reception is not yet physically qualified. Async TX,
DMA futures remain pending; TIMER/GPIO waits and the experimental Embassy
time driver are described below. There is no hidden blocking-backed async TX
or WFI executor, and no WS53 API graduates to stable.


WS53 also provides `unsafe { timer.async_delay(channel) }` with `async,unstable`.
The `timer::AsyncDelay` exclusively borrows the existing whole `TimerDriver`
for one typed channel and implements `embedded_hal_async::delay::DelayNs` plus
`delay_ticks(TimerTicks)`. Bind that channel's ISR to
`TimerDriver::on_async_interrupt(channel)` on its owning hart. Running counters,
enabled controller sources and pending notifications are rejected without
changing state. This does not provision clocks or split the controller token.

The established32MHz profile is required. Nanoseconds round upward; long
microsecond/millisecond requests retain the remainder across32-bit one-shots.
Zero-duration calls do not arm. Registering the waker precedes starting the
counter, re-polling does not restart it, and a spurious IRQ does not complete it.
Cancelling a Pending future stops/masks/acknowledges that channel and clears the
waker; other-channel registers are preserved. Wake/drop callbacks run outside
the driver's critical section. The adapter is not Send/Sync and provides no WFI
executor, clock-loss timeout, low-power compensation or Embassy driver.

The same-ELF RAM fixture checks both TIMER0/1,16 wake-driven expiries,4 cancels
(before expiry and after pending latched),6 refusal guards, zero requests and
TCXO-observed delay lower bounds. A failed first HIL round exposed an incorrect
LOAD readback expectation: after explicit writes the board reads `written | 0xF`.
The source RDL/PAC description and QEMU now document/model this read behavior;
requests remain full32-bit writes. Actual sub16-tick quantization and POR readback
are unqualified. `hisi-registers/evidence/ws53-timer-load-hil.json` preserves the
measurement. This is experimental async support, not stable timing qualification.


WS53 `async,unstable` now exposes `unsafe { input.async_wait() }` and
`gpio::AsyncInput`, implementing all five `embedded_hal_async::digital::Wait`
methods. It borrows the existing `Input` and therefore the entire bank; one
waiter per bank, not one independently allocated waiter per pin. The input must
already have an explicit pad/bank/clock handoff. Bind its ISR exclusively to
`GpioDriver::on_async_interrupt(bank)` on the application hart. Enabled bank
notifications/local IRQ or a pending local IRQ refuse takeover. SWD/reset pads
remain excluded from the production GPIO API.

Level waits return immediately when already satisfied. Edge waits start on
first poll after stale edges are cleared; the final level alone does not count
as an edge. ISR completion is sticky even if the pin reverses before re-poll.
Spurious interrupts keep an active waiter armed. Cancelling before a transition
or after it latches disables/masks/acknowledges this line and removes its waker;
reuse starts with a fresh event. Waker callbacks and MMIO occur outside driver
critical sections; bank IRQ exclusion protects configuration changes. The
adapter is not Send/Sync; no debounce, global MIE, WFI executor, clock-loss
recovery, low-power compensation or cross-core ownership protocol is supplied.

The `gpio-async-smoke` RAM fixture exercises MGPIO17 input with only weak pull
stimulus: eight real IRQ34/wake completions (rising/falling/both/high/low), two
immediate levels, cancellation before/after latching, reuse, two enabled-source
refusals, peer-register preservation and pad restoration. Three final same-ELF
hardware/QEMU runs passed and reset back to vendor firmware. This is physical
GPIO0 evidence only; GPIO1/2 async, external pulse widths, debounce and arbitrary
wiring remain unqualified. All WS53 interfaces remain experimental.


WS53 TCXO also supports nonblocking refresh transactions:
`begin_snapshot()` returns a `CounterSnapshot` borrowing the driver;
`try_read()` checks VALID once and returns `nb::Error::WouldBlock` without
reading stale data. A completed result is cached. Dropping an incomplete
transaction retains the hardware request in the driver; it does not pretend
that refresh can be cancelled. `try_discard_snapshot()` checks once and drops
that old result after completion. A new `begin_snapshot()` can also discard a
completed old request before issuing a fresh one, but returns
`TcxoError::SnapshotBusy` while the old request remains unfinished.

The blocking `read_counter()` shares this state machine: timeout retains its
request, so a retry cannot overwrite it or report a late old snapshot as new.
WS53 `enable()` now returns `Result<(), TcxoError>`; callers must handle Busy.
Both `enable()` and `clear_counter()` refuse retained requests without writes.
The existing WS63/BS2X interfaces are unchanged. A stopped clock can keep a
request busy indefinitely; recovery requires the owner to restore the hardware
clock/window, not a fictitious software cancel or implicit counter clear.

The `tcxo-nonblocking-smoke` RAM fixture passed three hardware/QEMU runs:
40 nonblocking samples,32 cached re-reads,8 dropped requests with drain/reuse,
16 enable/clear refusals, and the existing TIMER1-relative monotonicity checks.
No physical clock faults or counter clears were performed. Two QEMU-only staged
fault cases separately exercise clock loss and stalled completion, cancellation,
blocking timeout, refusal and recovery. This supplies the nonblocking time source
for the separate WS53 Embassy integration below.


### WS53 Embassy time (experimental)

With `chip-ws53,rt,unstable,embassy`, `embassy::init(timer, tcxo)` consumes two
explicitly handed-over drivers for the lifetime of the application. This unsafe
handoff requires exclusive single-hart access, an uninterrupted inherited32MHz
TCXO/TIMER epoch, and TIMER1/IRQ54 routed to `embassy::on_alarm_interrupt()`.
Both TIMER channels must be stopped with local interrupts disabled and no pending
source; TCXO must be enabled with no outstanding snapshot. Failed initialization
returns both resources. Success reserves the whole blocks but preserves TIMER0.
There is no deinit, clock provisioning, cross-core negotiation or sleep support.
The Embassy tick frequency is1MHz; incompatible tick features fail compilation.

`now()` is callable before initialization (returns0, no MMIO), then returns the
last monotonic TCXO snapshot. Each service checks VALID at most once, outside the
metadata critical section. A pending/faulted snapshot holds time, while `status()`
reports its condition. TIMER1 retries within a1ms hardware segment when a deadline
is queued; permanent clock loss requires external recovery. This does not prove
wall-clock continuity during sleep or clock loss. It does not silently clear the
counter or fabricate zero on failure.

The fixed queue retains16 distinct wakers. Same-task registrations keep the
earliest deadline; overflow wakes the evicted task early to re-poll. Deadlines
remain64-bit across32-bit hardware alarm segments. IRQs and callbacks cannot run
with the queue borrowed; a concurrent schedule during alarm programming causes
bounded early wake/re-poll rather than a lost deadline. Waker callbacks and their
clone/drop operations run outside the driver's critical sections. The generic
`embassy_time::Timer` API has no cancellation deregistration: dropping a future
leaves its bounded waker until expiry or eviction, not immediate alarm shutdown.

The `embassy-smoke` RAM fixture tests actual Timer/Ticker/with_timeout futures,
earlier-deadline reordering, initialization guards, cancellation/reuse, queue
capacity, far-future arithmetic, spurious ISR and reentrant callbacks. Three
same-ELF A-board/QEMU runs passed and recovered vendor firmware. Two QEMU-only
fault phases stall the snapshot or gate TCXO, verify frozen time/no early wake,
and restore deadline delivery. Physical clock faults, adversarial ISR preemption,
real multi-minute expiry, low-power and production resource handoff remain open.
See parent evidence `ws53-embassy-hil-2026-10-24/summary.json`.

### WS53 asynchronous UART TX (experimental)

`async,unstable` exposes `unsafe { uart.async_transmitter() }` and
`uart::AsyncTransmitter` implementing `embedded_io_async::Write`. It exclusively
borrows the configured UART on its owning application hart. Route its ISR to
`Uart::on_async_transmit_interrupt`; IER (including PTIM) and local IRQ must
be disabled before binding. Existing clock, pads and ownership remain required.
A nonempty write returns1 only after accepting that byte. Pending write accepts
none; cancellation sends none from that call. Default write_all may already
have completed a prefix. Flush waits for FIFO and serializer completion;
cancelling flush keeps queued/in-flight data. Drop masks notifications and
clears the waker, without resetting FIFOs or consuming RX.

Each poll checks at most twice. Empty-FIFO TX notifications may repeatedly wake
flush while the final character is shifting. Clock loss/flow-control stalls need
external recovery or cancellation. There is no implicit timeout, WFI executor
or concurrent RX/TX split. Waker metadata alone uses critical sections; MMIO
and waker callbacks are outside those sections for both RX and TX. UART2's
257-byte transfer, pending-write cancellation, two flush cancellations, three
bind guards and preserved3-byte RX each pass three same-ELF physical/QEMU runs.
The refactored RX path separately passes three runs. UART0/1 asynchronous TX,
arbitrary flow control, production resource handoff and clock faults remain
unqualified. Evidence: integration docs/plan/evidence/ws53-uart-async-tx-hil-2026-10-24.

### WS53 memory-copy DMA async (experimental)

`chip-ws53,unstable,async` exposes `unsafe { transfer.run_async() }`, returning
`dma::AsyncTransfer`. It exclusively borrows a ready memory `Transfer`; already
running or retained transfers are refused. The first poll registers its waker,
starts the configured copy and enables IRQ58, then returns Pending. Completion
requires actual IRQ delivery followed by a finite status/configuration check.
The caller must route the shared IRQ58 exclusively to `dma::on_async_interrupt`
on this application hart; all interrupt users of this controller must use this
protocol. The ISR masks the shared local source and wakes all registered
channels without clearing their raw flags. Each subsequent poll checks its own
channel and rearms shared delivery if still pending. Polling/waker callbacks
never run in the metadata critical section. No WFI executor is supplied.

An unpolled future changes no hardware. Drop after start requests halt/disable
only for its channel, clears its waker and retains both buffers and the channel.
Completion/error also retain all resources; raw events remain available for
explicit acknowledgement. A cancelled/finished transfer cannot restart. Neither
IRQ nor completion proves that posted transactions drained: even after awaiting
success, reclaim still needs independent unsafe external quiescence proof.
Timeout combinators may cancel; no hidden time limit or clock provisioning is
provided. Multiple pending channels are supported under the same unsafe routing
contract. Physical tests cover the four observed channels, not SDK slots4..7.

The RAM fixture covers actual wakeup, two concurrent waiters, cancellation
before delivery and after wake, unpolled cancellation, twelve state guards and
retained-buffer debugger verification. QEMU-only faults exercise read/write bus
errors, no-progress cancellation and an accepted write that finishes after Drop.
Physical in-flight cancellation/drain, peripheral async, LLI, SDMA, cache
maintenance and production control-core resource negotiation remain unqualified.

### WS53 SDIO polling initialization (experimental)

`chip-ws53,unstable` provides typed CMD53/descriptor software formats and
`sdio::Initialization`. SDIO has its own `Unclaimed` token, independent of MDMA.
The unsafe constructor requires external ownership, drained previous ADMA,
serialized complete clock-word transactions, configured pads and a coordinated
host. Local IRQ46 must remain masked throughout the session.

Initialization enables the two audited word-sized clock gates, requires a clean
starting state, waits for host clock, writes the source profile and accepts a
fresh Function1 enable event. A single poll budget covers both waiting stages;
captured notifications are returned before another poll. Unsupported events
remain unacknowledged. Failure/cancellation retains the owner, with later polls
performing no MMIO. Drop and `release_without_stopping` do not reset or drain.

The source profile is checked against 36 complete original-ROM RAM observations.
Ten actual Rust QEMU scenarios cover messages, enable, timeouts, clock loss,
unsupported events, READY readback failure and cancellation. This explicit
assumed model does not qualify real SDIO electrical behavior, CIS enumeration,
ADMA transfers, CPU interrupt service, or safe buffer reclamation. The independent
parent example `examples/ws53-sdio-init` is a host-paced model fixture and lacks
board handoff; it is not a firmware image to deploy on a board.

`Function1Request` validates observed transfer metadata before any future table
submission: Function1, agreement with the captured event direction, fixed zero
address and a nonzero command length. It preserves multi-descriptor lengths and
rejects the ROM's zero-block error sentinel. A recorded-ROM comparison covers288
inputs (74 accepted,214 rejected); this does not reserve memory or initiate DMA.
Original-ROM execution also confirms that `complete_send` appends a regular
four-byte VALID|TRAN entry without END. Its name is not a bus-drain guarantee;
descriptor-chain termination and safe ownership return still require evidence.

`Transfer` now reserves coherent, exclusive static data and an aligned static
descriptor/trailer table, validates a matching host request, and publishes the
table through the PAC. Its unsafe entry requires independently established
CMD53-length termination, initialized hardware and coordinated host ownership.
Completion, timeout, errors and cancellation retain every allocation. Cancellation
masks notifications only; it cannot withdraw an accepted posted bus write.
Returning memory requires unsafe `release_after_external_quiescence` with an
external drain/coherence proof. Drop neither performs MMIO nor returns memory.

HostRead starts by notifying the peer of the retained length, matching the
original prepare-send ROM sequence (168 full-window input comparisons). HostWrite
does not issue that notification. Cancelling after notification retains the
buffer and refuses a later host request without publishing descriptors; the
notification itself is not withdrawn or considered cleared by cancellation.

Fourteen actual Rust CPU scenarios exercise read/write payloads and failure
retention against an explicitly assumed QEMU ADMA profile. Guard bytes and the
complete descriptor image are checked from the debugger, never through an active
CPU buffer alias. No hardware transfer, cache behavior, CIS, CPU IRQ service,
async data path or physical stop/drain is qualified by these model results.

With `async`, the experimental `Transfer::wait_async` returns one captured
notification/start/completion event. Its sole IRQ46 handler masks the local source
and wakes outside the metadata critical section; it neither borrows the transfer
nor acknowledges SDIO events. Pending cancellation masks notification and retains
all allocations; unpolled cancellation does nothing. Seven additional host tests
cover arrival races, spurious wakes, message preservation and cancellation.
Fifteen independent CPU/QEMU scenarios exercise IRQ46 delivery, both directions,
messages, errors, cancellation and posted writes after cancellation. These rely
on the opt-in ADMA profile; silicon operation and physical drain remain unqualified.

### WS53 unkeyed HASH polling (experimental)

`chip-ws53,unstable` exposes `hash::{Algorithm, Channel, Input, Table, Transfer}`
using the source-audited SPACC PAC. Both SDK channels 1/2 are selectable, but one
transfer reserves the whole shared `SPACC` token. `Unclaimed` and the unsafe
configured constructor require external application/control-core/IRQ59 handoff;
neither call negotiates IPC, initializes clocks or proves the block is idle.

The initial subset is one-shot SHA-1/224/256/384/512 and SM3, without HMAC,
keyslot operations, streaming or interrupts. Encoded messages are limited to
the SDK's 256-byte bounce-buffer profile (at most 247 input bytes, or 239 for
SHA-384/512). This is a conservative software boundary, not a hardware limit.
SHA-1 is a legacy compatibility option, unsuitable for collision resistance.

Input and the two-entry descriptor table need exclusive, coherent/noncacheable,
32-byte-aligned static application RAM. The mode-1 DTCM range check does not
prove actual core ownership or bus accessibility. `prepare` copies and pads
before publication; source C fixtures check six configurations, both state
windows and all 512 padding tails. SDK API node flags and packed descriptor bits
are deliberately kept separate. State words use the source-observed byte order.

`start` verifies the REE claim, masks the selected HASH notification, performs a
bounded clear handshake, rejects stale/uncleared status and nonempty rings, then
publishes one descriptor through PAC commands. `poll`/`wait` return a copied
digest only after completion and ring checks. Errors, lost ownership and timeout
retain resources without reading a partial digest. The global IRQ gate, other
channel fields and security error banks are preserved; no error clear is guessed.

Success, cancellation, Drop and forget retain input, descriptors and controller.
Cancellation has no hardware abort or drain semantics. There is no safe memory
return or implicit unlock; `release_after_external_quiescence` is unsafe and
requires an independently performed drain/coherence/ownership handback.

Host tests cover these failure paths and PAC field emission, and RISC-V builds
pass. The separate opt-in QEMU model now computes real SHA/SM3 compression and
checks IRQ59, clock/reset and retained transactions. The integration repository's
`ws53-hash-model` fixture runs this production driver on a RISC-V CPU in198
scenarios (156 digest comparisons,12 length rejections,30 fault/cancellation
cases). Its mailbox host does not initialize MMIO or descriptors. The original
host control-flow fake remains a separate test and does not calculate digests.

The SDK's `malloc_coherent` port actually requests ordinary aligned LiteOS heap;
OS cache hooks perform separate architecture operations and Boot weak defaults
are empty. Neither32-byte alignment nor identity address translation establishes
coherence or bus reachability. The source handoff audit in hisi-registers also
identifies shared auto-gating setup and application sleep veto; these do not
establish a DMA stop/drain protocol. This backend does not maintain cached input
or implement that shared clock/PM handoff. RAM/Flash hardware operation, cold
startup and Rust HASH IRQ/async remain unqualified; the API stays experimental.

### WS53 keyslot and clear-key loader (experimental)

`km::KeyManager` owns the complete `Peripherals::KM` reservation: one shared
loader/selector, eight cipher slots, two HMAC slots and IRQ61. The unsafe
`from_configured` constructor performs no MMIO. It requires external handoff
from all cores, previous key consumers (including Flash decryption), handlers,
and root/common/clear routes; clocks, REE identity and completion/flush semantics
must be independently qualified. The SDK common-route lock does not establish
clear-loader exclusion. This layer does not configure clocks or permanent policy.

Typed slot namespaces, AES-128/192/256 and SM4 byte arrays, HMAC algorithms and
fallible HMAC key lengths prevent truncated identifiers or silent key truncation.
Long HMAC keys require caller normalization; empty keys are zero-padded. Loading
uses explicit little-endian words, without assuming byte-slice alignment. Odd
half selection is rejected for keys requiring both halves. All polling has a
nonzero caller budget; reentry returns an error without nested register access.

Noncopyable leases borrow the manager. Drop/forget retain reservations without
MMIO. Terminal faults preserve the first error and prohibit further controller
access; uncertain or failed loads never clear staging words or unlock slots.
Explicit slot release is unsafe: prior consumers must already be drained, then
the driver checks flush failure/timeout bits and rereads owner zero before reuse.
Successful release is not a claim of physical key erasure. Caller bytes and
temporary CPU/stack copies have no guaranteed zeroization; secret-bearing inputs
have neither Debug nor defmt formatting.

Fifteen host tests cover all slots, every HMAC key-length boundary, segmented
loading, faults at each AES/HMAC stage, reservation retention, reentry, bounded
polling, ownership changes and PAC writes. Compile checks reject copying leases,
handing back a manager with a live lease and safe release without quiescence.
The control-flow fake does not implement encryption. A separate opt-in QEMU
model now copies synthetic staging bytes into actual slot storage and covers
ownership, clock/reset and faults. The integration repository's `ws53-km-model`
fixture runs this production HAL on a RISC-V CPU in 357 cases, including all ten
slots, per-stage timeouts/ownership loss, Drop/forget and explicit release.
KM IRQ/async, real shared-resource handoff and RAM/Flash/cold-start
hardware qualification remain open. Model completion/flush/ACK semantics are
explicit assumptions. Do not treat this loader as a working AES/SM4 or HMAC
computation API.

### WS53 SYMC block transfers (experimental)

`symc::Transfer` consumes the whole SPACC reservation, a loaded KM lease,
distinct static input/output buffers, and a 96-byte table containing two
32-byte input and two 16-byte output descriptors. AES-128/192/256 and SM4 use
the source-profile ECB, CBC, CTR, OFB and CFB128 modes, with complete 16-byte
blocks only. This is a low-level unauthenticated primitive; it adds no padding,
IV/counter reuse policy or authentication. Partial stream blocks, CFB8, AEAD/MAC,
streaming, IRQ/async and production handoff remain unfinished.

The unsafe configured constructor does no MMIO. It checks cipher namespace,
loaded direction permissions, key parity, exact capacities and disjoint bus
ranges before modifying descriptors; rejection returns every reservation.
`BlockLength` rejects zero, partial blocks and narrowing overflow. Memory must
be exclusively reserved, identity-addressed, bus-accessible and coherent or
noncacheable; the SDK aligned allocator does not establish those properties.
Clock/reset, REE identity, shared HASH/SYMC/IRQ59 ownership and selected-bit
ACK semantics remain external, unqualified obligations. No security policy,
global interrupt gate or clock register is silently changed.

Start checks KM health, claims a free REE channel, masks its notifications and
uses a bounded clear handshake. Both rings must start empty. Both tables are
finished before publishing output then input; a key/ownership/error failure
between publications retains everything and does not submit input. Completion
requires the output event plus both ring read pointers; fixed-size IV metadata
is copied only after health and ACK checks. Cached success/error, cancellation,
Drop and forget perform no extra MMIO. The driver never unlocks automatically.

Success does not expose DMA output memory. Only the unsafe external-quiescence
handback returns buffers, SPACC and the still-owned key lease after independent
physical drain and cache/ownership reconciliation. Slot release is a separate
checked KM operation. This avoids treating interrupt flags or equal pointers as
proof that bus masters stopped.

Host tests cover 960 valid key/permission/mode combinations, rejection domains,
all mode-1 length boundaries, descriptor ABI, both channels, clear/wait budgets,
stale ACKs, partial completions, fault injection around publication/IV reads,
reserved-bit preservation and resource retention. Compile checks reject safe
release/output access and use of a key after transfer preparation. These are
control/format tests. The opt-in QEMU model now computes real AES/SM4 output,
checks KM permissions, and combines HASH/SYMC IRQ59 sources. The integration
repository's `ws53-symc-model` executes production HAL instructions in 154 CPU
scenarios, including independent ciphertext/IV comparison, timeout, cancellation,
Drop/forget and retained key/buffer ownership. All SYMC hardware/RAM/Flash/
cold-start verification and production shared-resource handoff remain open.

WS53 `ipc` adds an experimental configured doorbell view and defensive inline
message format. `Channel` selects causes26/27; patched sends are16-bit, ACKs are
32-bit and followed by fence/discard readback. Existing notifications are not
overwritten; unknown IDs/priorities, payloads over60 bytes and inconsistent
frames are rejected. This is not a shared-ring transport, peer resource lease,
TRNG callback service, cache-maintenance implementation or low-power handshake.
No clock/reset initialization is inferred; attachment and publication retain
explicit unsafe external obligations. All WS53 IPC APIs require `unstable` and
remain unqualified on hardware until named model and board tests prove them.

`ReceiveEnableState` explicitly hands over the SDK disabled/all-enabled word.
The driver maintains the owned receive-mask shadow and performs whole-word
writes, never reading an unevidenced enable register. Constructor safety now
requires that exact handoff state and exclusion of every other mask writer.
Other handed-off low16 SDK bits are retained; the upper16 bits stay zero.
Per-channel enable/IRQ semantics still need hardware validation.

`ipc::mailbox::{MailboxImage, MailboxSize, Route}` adds bounded transactions on
an **exclusively owned ordinary-memory image**, using the SDK twelve-byte ring
header and four-byte message header. Typed extents/routes reject mismatched
capacity, core/channel identity, cursors and flags. Wrapped messages are copied
and validated before consuming; errors preserve cursors, bytes and caller output.
Full and empty images are distinguished exactly as in serialized ROM execution.
This safe image API is not permission to create `&mut [u8]` into concurrently
accessed shared RAM. Local IRQ masking, a doorbell ACK and an address observed
in a factory image do not supply cross-core exclusion or cache ownership.

The ROM consumer and producer both write `empty`. A source-bound, bounded SC
model admits a full-as-empty counterexample for244/1268-byte capacities. This
is a protocol possibility, not a measured hardware failure or a proof about
the installed Ccore patch path. Reserving a byte eliminates that counterexample
in the tested bounded schedules but is not known to be enforced by the retained
vendor peer. Concurrent RX/transport publication remains unqualified. The
integration fixture tests real Rust image transactions with an independently
implemented virtual peer that runs only while the Rust CPU is stopped; it does
not turn serialized model success into live control-core IPC support.

`ipc::spsc::{Transmit,Receive,CoherentMemory}` is a separate experimental
concurrent storage profile. TX reserves a byte and leaves empty1 untouched;
the specifically qualified SDK consumer checks differing cursors first. RX
stores its read cursor one byte behind the logical head and leaves empty0;
the specifically qualified SDK producer counts an occupied sentinel. This
removes shared empty-byte writes without changing those complementary
algorithms. It does not prove the installed Ccore actually uses them, nor may
an arbitrary SDK empty-byte fast path be substituted. Initial preparation and
handoff must happen while the actual peer and old local handlers are quiescent.

`SharedMailbox` is an unsafe, static-lifetime coherent-memory backend contract.
The native adapter uses aligned volatile cells for ordinary RAM, acquire/release
RISC-V fences and no A-extension operations; it does not validate PMP, enable
clocks, flush noncoherent aliases or make a memory lease. Both mappings must
be coherent/uncached, and peer ordering must be qualified as well. Local fences
cannot repair an unordered peer; coherent/uncached visibility is insufficient.
Attachment validates exact routes/capacity/cursors/flags and an empty prepared
profile; methods are bounded, corruption leaves output
and cursors unchanged, and TX publication remains unsafe because wire-format
validation does not authorize services or embedded pointers. Drop/forget retain
the backend through ManuallyDrop; only unsafe quiescent reclaim releases it.
These rules also apply to external mappings after the Rust handle is gone.

Four capacities per direction with independently transcribed SDK complementary
algorithms pass8000 threaded host transfers. The actual Rust CPU fixture also
runs while the virtual peer accesses complementary RAM, with no per-message
CPU stop; max occupancy, wrapping, corrupt-ID quarantine, reset and explicit
quiescent reclaim are exercised. It remains a coherent digital model, not
weak-memory/cache qualification or a real vendor-control-core bootstrap.
Installed Ccore code, both cache attributes, service/ownership handoff,
notification/rearm, native RAM/Flash/cold-start and waveform tests remain open.

The HAL development Cargo.lock now includes the staged WS53 dependencies.
`cargo-standalone-lock.py --patch-crate NAME=DIR` makes those local overrides
explicit without committing machine paths, performs full dependency resolution,
and refuses to call this package/registry validation. Ordinary registry-only
availability and release/package checks still need the published dependencies;
the local-patch development check is not a substitute for them.
