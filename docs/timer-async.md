# One-shot async timer boundary

WS53 and WS63 share `src/timer/async_alarm.rs`: waiter registration, arming,
completion consumption, cancellation and Waker retirement. The small `AlarmIo`
backend supplies only stop, arm and raw-expiry observation. It has no scheduler,
deadline queue, Embassy registration or additional hardware owner. BS21 async
support is unchanged; sharing an IP name alone does not qualify a new backend.

WS53 retains its unsafe stopped-channel borrow of the whole TimerDriver and
explicit application-core handoff. WS63 retains the experimental constructor
consuming the singleton TIMER. Both keep their existing channel/IRQ types and
public signatures. Neither splits the resource with RTOS/Embassy timekeeping.

| Event | Shared behavior | Backend obligation |
| --- | --- | --- |
| First poll | Publish active state and waiter before arm | Stop/mask/acknowledge old state first; arm one nonzero period |
| Immediate IRQ during arm | Latch completion and wake; current poll can return Ready | ISR may run after delivery is enabled |
| Empty notification | Leave the current wait active | Check raw expiry without clearing an event arriving after a false sample |
| Actual expiry | Stop first, then latch completion and wake outside metadata lock | Mask local delivery and acknowledge peripheral/controller |
| Drop after poll | Stop/acknowledge, retire waiter and completion | Bounded per-channel stop, never wait for external progress |
| Drop before poll | No MMIO | None |

Metadata critical sections contain only Rust state updates. Register operations
and all Waker callbacks run outside them. Per-channel IRQ masking and the
single-hart, exclusive, non-reentrant platform contract make the ordering valid;
disabling local interrupts does not synchronize another core. Backend methods
must not block. The parent adapter keeps the real peripheral borrow/ownership
for the public Future's lifetime; a software signal alone is not an ownership token.

WS53 retains zero/no-MMIO, upward rounding, nonzero chunks and cancellation
semantics. WS63 now follows these rules instead of flooring nanoseconds, arming
zero, completing on an empty interrupt and leaving a dropped wait running.
WS63 `delay_us`/`delay_ms` now use the same checked-width chunk calculation as
WS53; frequency is a platform configuration premise, not a measurement.

The child-owned host model exercises cancellation before/after expiry, reuse,
unpolled drop, coalesced/empty notification, late expiry after a false sample,
IRQ during arm, another channel remaining active and callback/IO lock discipline.
The previous WS53 lifecycle tests moved here; its conversion tests remain at the
chip adapter. Parent QEMU scenarios inspect real driver registers and traps.
No new physical timing, stop/acknowledgement latency or cross-core handoff is
qualified by those models. APIs remain unstable. The minimal `asynch::block_on`
poll-to-WFI race and DMA ownership/cancellation are separate outstanding work.
