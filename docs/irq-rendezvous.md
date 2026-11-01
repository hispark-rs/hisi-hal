# Executor-independent IRQ rendezvous

`asynch::IrqSignal` is an experimental single-consumer, coalescing software
signal. DMA and LSADC use the same `poll_wait` implementation;
UART uses its [shared receive lifecycle](uart-async.md); GPIO uses its [shared line lifecycle](gpio-async.md), and TIMER uses the shared
[one-shot lifecycle](timer-async.md), which additionally
owns arming and cancellation. Neither depends on Embassy or RTOS. Multiple hardware events do not become
multiple queued completions.

After `take_fired()` returns false, an ISR may fire before the future registers
its waker. Registration checks the event flag while installing the waiter and
issues a wake if completion is already latched. The future may return Pending
after that wake; the executor must honor wakes during polling. Registration
does not consume the completion flag.

Only the flag and waiter slot are changed under the metadata critical section.
Waker clone, replacement drop, reset drop and wake run after leaving that
section. Callers must not surround these operations with a longer critical
section if they need the same property. Wake callbacks still need to be valid
in the caller's ISR context; this is not deferred execution.

The driver owns hardware masking, acknowledgement, stopping and cancellation.
It must stop/acknowledge the old operation before resetting software state.
This change does not establish those invariants for every existing driver.
DMA buffer/stop safety and the minimal `block_on` poll-to-WFI race remain separate
audits. The TIMER lifecycle has host/model evidence, not new physical qualification.
Single-hart critical sections do not arbitrate control-core/shared hardware.

Host tests in `src/asynch/tests.rs` cover arrival before/after registration,
arrival during Waker cloning, event coalescing, replacement, reset, reentry,
and callback execution outside the metadata lock. The callback test acquires
the host critical-section lock on another thread; same-thread reentry alone
would not detect nested critical sections. These tests are not a proof of
peripheral timing, interrupt-controller semantics, or hardware cancellation.

Run the child repository's `scripts/check-ws53-integration.sh` for independent
development checks (remote runtime revision, registry PACs); registry-only
release qualification remains blocked until the required runtime is published.
QEMU and physical-device acceptance belong to the parent integration repository.
