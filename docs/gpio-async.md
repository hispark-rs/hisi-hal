# GPIO async wait lifecycle

`gpio/async_wait.rs` owns first-poll configuration, waiter replacement, latched
completion, level sampling and cancellation. It contains no PAC, chip ID,
executor or time queue. Backends mask local delivery before configuration and
publish the waiter before resuming IRQs. Register operations and Waker callbacks
run outside the metadata critical section.

The resource boundaries remain different:

- WS53 borrows one input and its whole bank through the existing unsafe handoff.
  One slot per bank carries the actual typed line. The ISR observes that line's
  masked status; an empty IRQ resumes delivery without consuming the waiter.
- WS63 owns each input pin independently. Each pin now has its own slot; bank
  IRQ bindings and pin count come from `soc/ws63`. The ISR stops/acknowledges
  enabled captured sources and wakes only their waiters. Cancelling one pin
  preserves other enabled pins and does not clear their bank-wide pending latch.

Temporary GPIO masking uses `interrupt::mask`, which preserves the controller
pending latch. The historical `interrupt::disable` still masks and acknowledges;
using it during re-poll or cancelling a shared pin loses a latched peer event.
Both WS63 peer cancellation and WS53 re-poll-before-ISR are separate model cases.

The legacy WS63 bank-wide completion flag let an unrelated or empty IRQ finish
a wait, and dropping a pending future left its notification enabled. These
experimental behaviors are corrected without changing published stable APIs.
`wait_for_any_edge` still chooses the legacy polarity from the initial sample;
this does not add hardware both-edge capture or close the setup bounce window.
The WS53 both-edge backend and explicit ownership prerequisites are retained.

Seven shared host tests cover cancellation/reuse, unpolled drop, unrelated and
duplicate notifications, IRQ during enable, immediate level completion,
independent slots, Waker replacement and callback/IO lock discipline. These
tests validate software ordering, not peripheral timing. Parent integration
fixtures validate WS63 same-bank input8/input9 via QEMU output0/output1 nets and
the existing WS53 MGPIO17 scenario. They do not qualify all physical pins.

Use these drivers on the owning hart with exclusive pin configuration and the
documented non-reentrant GPIO ISR. A metadata lock does not arbitrate shared
register RMWs with other cores or unrelated ISRs, and no cross-core handoff is
established here. Physical IRQ/ack/cancellation timing needs fresh acceptance.
BS21 async support is unchanged. The minimal `asynch::block_on` now records
software wakes during polling, but its poll-to-WFI race and DMA safety remain
separate open work; see [the rendezvous boundary](irq-rendezvous.md).
No new HAL scheduler, Embassy dependency or stable graduation is added.
