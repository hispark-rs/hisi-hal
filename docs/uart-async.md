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
  removing a byte; the waiter delegates to the blocking driver's existing
  `read_byte` status/data path (`LINE_STATUS.data_available`). The receive-count
  alias must not be substituted across chips. The initial extraction incorrectly
  used that count and is corrected here; its host-only FIFO tests could not
  establish which hardware register to read. A pending read also selects the
  existing blocking FIFO levels (RX one byte, TX empty), keeps FIFO enabled and
  issues no reset strobes. It no longer relies on the constructor's quarter-full
  DMA threshold or character timeout to wake for a short input. These levels
  remain after cancellation; DMA setup selects its own levels on its next use.
  The former per-port one-byte software latch could be overwritten
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
the actual backend and model used. The parent now exercises actual WS63 RX/IRQ/cancellation against the shared
WS63/BS2X FIFO model, including mask/reassert and one-byte notification. This
does not qualify the receive-count alias (which remains deliberately diagnosed),
physical timing, all real UART instances or overflow/error behavior. The separate
`block_on` idle race also remains open.


## Shared nonblocking transmit contract

All chips and instances forward `embedded_hal_nb::serial::Write::write` to the
same bounded `try_write_byte` path. A full FIFO returns `WouldBlock` without
writing DATA. Earlier WS63/BS2X trait implementations incorrectly called the
blocking `write_byte`; this is a behavior correction, not just a file move.
The blocking methods intentionally retain their existing semantics.

The trait's error type and `tx_flushed` remain backend-specific: legacy chips
check FIFO-empty, while WS53 also waits for the shift register and preserves
read-clear RX errors. This change does not assert that these flush contracts are
equivalent, nor turn legacy async TX into an interrupt-driven implementation.

Parent integration verifies the public trait with a virtual peer under explicit
FIFO backpressure, repeated rejected writes, ordered drain and recovery. The
same fixture with the old HAL stalls before reporting full. The synthetic FIFO
capacity is not a physical-depth or baud claim; new hardware acceptance remains
pending. Child host tests and cross-chip feature/Clippy checks run independently.
