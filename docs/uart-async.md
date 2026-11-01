# Async UART receive lifecycle

`uart/async_receive.rs` owns the bounded receive poll and cancellation guard,
shared by WS53 and WS63. It contains no PAC, scheduler, time queue or executor.
The public `embedded_io_async::Read` implementations still return one byte per
nonempty successful read; an empty read does not touch hardware.

Polling installs the current waker before any register access, masks receive
notification, attempts one read, and, on WouldBlock, rearms and tries once more.
An arrival during arming is either read on that second check or retains a
hardware notification. A notification during poll may wake before Pending is
returned: the executor must honor that wake. Empty/duplicate interrupts merely
schedule another check; they cannot fabricate received bytes.

Pending means no byte was consumed. Dropping a polled wait masks its receive
source and removes the waker without touching DATA. A subsequent read consumes
any retained FIFO bytes. Dropping a never-polled guard has no effect. Waker
clone, drop and wake run outside the driver's metadata critical section.
Neither this algorithm nor a software waker prevents physical FIFO overrun.

## Hardware backends and compatibility

- WS53 retains the whole-UART async adapter, unsafe ownership/IRQ handoff,
  line-status error preservation, peripheral and local-controller masking and
  acknowledgement. Its ISR still does not consume DATA or read-clear errors.
- WS63 keeps its existing trait and trap-hook signatures, externally enabled
  local IRQ and infallible receive API. The ISR now masks RX and wakes without
  removing a byte; the waiter reads using the FIFO count, as the blocking
  driver does. The former per-port one-byte software latch could be overwritten
  by another interrupt before consumption, and a cancelled waiter left RX
  enabled. Both paths are removed. This is a behavior fix, not inherited HIL
  qualification. Only RX-data interrupts are owned; route the UART on its
  owning hart and do not use this hook as a TX/modem/DMA dispatcher.
- WS63 TX remains blocking-backed even through the async Write trait. WS53 TX
  has its existing distinct nonblocking backend. This batch does not claim to
  unify their transmit semantics or to add BS2X interrupt-driven reception.

Do not mechanically replace WS53 interrupt acknowledgement with mask-only:
FIFO/status retention and the two reads are part of the receive contract,
whereas a GPIO edge may exist only in a pending latch. Hardware ownership and
register side effects remain backend responsibilities.

## Validation ownership

The child owns eight common tests: arrival before/during arm, error preservation,
waker replacement, wake during poll, unpolled drop, cancellation with retained
arrival, coalesced/duplicate notifications and empty notification. These check
software ordering and lifecycle with a retained FIFO model, not electrical or
interrupt-controller behavior. The parent's QEMU and HIL evidence must identify
the actual backend and model used. In particular the existing WS63 QEMU UART
lacks RX FIFO count and complete IRQ mask/reassert behavior and cannot qualify
this receive path yet. Physical timing, all UART instances, overflow/errors,
and the separate `block_on` idle race remain open qualifications.
