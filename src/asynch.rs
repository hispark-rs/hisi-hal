//! Async support for the WS63 HAL (`async` feature).
//!
//! Two pieces shared by the async drivers:
//!
//! * [`block_on`] — a minimal single-future executor. It polls the future and,
//!   while it is `Pending` without a recorded wake, sleeps the core with `wfi`. The hardware interrupt
//!   that will eventually complete the future wakes the core, and `block_on`
//!   re-polls. No allocator, no heap, no global executor — enough to drive one
//!   `embedded-hal-async` future to completion. (For multitasking, run these
//!   futures under a real executor such as Embassy instead.) This experimental
//!   helper still has an unresolved IRQ-to-WFI race; see its limitations below.
//!
//! * [`IrqSignal`] — the bridge from an interrupt handler to a waiting future:
//!   the driver's ISR calls [`IrqSignal::signal`]; the future polls
//!   [`IrqSignal::take_fired`] and parks its waker via [`IrqSignal::register`].
//!   It is `const`-constructible so drivers can hold one in a `static`.
//!
//! The async drivers (see [`crate::timer::AsyncDelay`], the `Wait` impl in
//! [`crate::gpio`]) do NOT install interrupt handlers themselves — the
//! application routes its trap to the driver's `on_interrupt` (mirroring the
//! `timer_irq` / `gpio_irq` examples). So enabling the `async` feature never
//! changes the behaviour of firmware that doesn't use it.

use core::cell::Cell;
use core::future::Future;
use core::task::{Context, Poll, RawWaker, RawWakerVTable, Waker};

use critical_section::Mutex;
use portable_atomic::{AtomicBool, Ordering};

// Static storage is necessary because a Future may retain its Waker after
// returning Ready or unwinding. No Waker may refer to the block_on stack.
// One active call owns this notification bit; late wakes are harmless spurious
// notifications, including when a subsequent invocation owns it.
static BLOCK_ON_ACTIVE: AtomicBool = AtomicBool::new(false);
static BLOCK_ON_NOTIFIED: AtomicBool = AtomicBool::new(false);
static VTABLE: RawWakerVTable = RawWakerVTable::new(
    |_| RAW_WAKER,
    |_| BLOCK_ON_NOTIFIED.store(true, Ordering::Release),
    |_| BLOCK_ON_NOTIFIED.store(true, Ordering::Release),
    |_| {},
);
const RAW_WAKER: RawWaker = RawWaker::new(core::ptr::null(), &VTABLE);

struct BlockOnClaim;
impl BlockOnClaim {
    fn acquire() -> Self {
        assert!(
            BLOCK_ON_ACTIVE.compare_exchange(false, true, Ordering::Acquire, Ordering::Relaxed).is_ok(),
            "block_on does not support nested or concurrent calls"
        );
        BLOCK_ON_NOTIFIED.store(false, Ordering::Relaxed);
        Self
    }
}
impl Drop for BlockOnClaim {
    fn drop(&mut self) {
        BLOCK_ON_ACTIVE.store(false, Ordering::Release);
    }
}

#[inline]
fn wait_for_interrupt() {
    #[cfg(target_arch = "riscv32")]
    unsafe {
        core::arch::asm!("wfi", options(nomem, nostack));
    }
    #[cfg(not(target_arch = "riscv32"))]
    core::hint::spin_loop();
}

/// Drive one future, re-polling recorded wakes before considering `wfi`.
///
/// For futures requiring hardware IRQ progress, global interrupts must be enabled
/// (`interrupt::enable_global`) and the completing peripheral's IRQ routed to its
/// `on_interrupt`, or `wfi` will never wake. Pure software-waking futures do not
/// require interrupt delivery; this helper preserves the caller's enable state.
///
/// # Limitations
/// Software wakes issued during polling are coalesced and cause another poll.
/// **This is not yet an atomic sleep handoff:** an IRQ/wake after the notification
/// check but before WFI can still be lost until another interrupt occurs. Use a
/// platform-qualified executor where this window matters. This helper does not
/// change the caller's interrupt enable state or reserve a timer/software IRQ.
///
/// # Panics
/// Nested and concurrent calls are rejected before polling the new future.
/// There is one static notification slot, not a task scheduler. A retained Waker
/// stays memory-safe after completion and may cause a spurious poll in a later
/// call; it carries no pointer to the completed future. The claim is released on
/// return or unwinding (a panic-abort runtime does not return).
pub fn block_on<F: Future>(fut: F) -> F::Output {
    block_on_with_idle(fut, wait_for_interrupt)
}

fn block_on_with_idle<F: Future>(fut: F, mut idle: impl FnMut()) -> F::Output {
    // Declared before the pinned future so its destructor also runs while this
    // invocation owns the slot. No user code is polled/dropped under a CS.
    let _claim = BlockOnClaim::acquire();
    let mut fut = core::pin::pin!(fut);
    // SAFETY: the vtable's clone returns the same null-data RawWaker; wake/drop
    // access only static atomic metadata. Nothing dereferences the data pointer.
    let waker = unsafe { Waker::from_raw(RAW_WAKER) };
    let mut cx = Context::from_waker(&waker);
    loop {
        if let Poll::Ready(v) = fut.as_mut().poll(&mut cx) {
            return v;
        }
        if !BLOCK_ON_NOTIFIED.swap(false, Ordering::AcqRel) {
            idle();
        }
    }
}

/// One-shot interrupt→future signal: an ISR calls [`signal`](Self::signal), the
/// future polls [`take_fired`](Self::take_fired) and parks via
/// [`register`](Self::register). `const`-constructible for use in a `static`.
///
/// Each signal has one logical consumer: registering another waiter replaces
/// the previous waiter. Drivers must enforce that ownership and quiesce old
/// hardware activity before resetting/reusing the signal. This helper does not
/// implement peripheral cancellation or an atomic poll-to-`wfi` transition.
pub struct IrqSignal {
    fired: AtomicBool,
    waker: Mutex<Cell<Option<Waker>>>,
}

impl IrqSignal {
    /// Create a cleared signal.
    pub const fn new() -> Self {
        Self { fired: AtomicBool::new(false), waker: Mutex::new(Cell::new(None)) }
    }

    /// ISR side: latch one event and wake the parked waiter outside the lock.
    /// Multiple events coalesce until `take_fired` consumes the flag; this is not
    /// an event counter. Waker callbacks must be safe in the calling ISR context.
    pub fn signal(&self) {
        let wake = critical_section::with(|cs| {
            self.fired.store(true, Ordering::Release);
            self.waker.borrow(cs).take()
        });
        if let Some(waker) = wake {
            waker.wake();
        }
    }

    /// Future side: register the current waiter. If an event arrived before
    /// registration, wake immediately so a caller returning Pending is polled
    /// again. Registration never consumes the event flag. Replacement, clone,
    /// drop and wake callbacks all run outside the metadata critical section.
    pub fn register(&self, waker: &Waker) {
        let cloned = waker.clone();
        let (old, wake) = critical_section::with(|cs| {
            let slot = self.waker.borrow(cs);
            let old = slot.replace(Some(cloned));
            let wake = if self.fired.load(Ordering::Acquire) { slot.take() } else { None };
            (old, wake)
        });
        drop(old);
        if let Some(waker) = wake {
            waker.wake();
        }
    }

    /// Consume the coalesced event flag once. After a false result, register the
    /// waiter before returning Pending; `register` closes that arrival window.
    pub fn take_fired(&self) -> bool {
        self.fired.swap(false, Ordering::Acquire)
    }

    /// Clear the current event and waiter before arming a new operation.
    /// The driver must first quiesce/acknowledge its old hardware operation;
    /// this only resets software state and cannot cancel a device transaction.
    pub fn reset(&self) {
        let old = critical_section::with(|cs| {
            self.fired.store(false, Ordering::Relaxed);
            self.waker.borrow(cs).take()
        });
        drop(old);
    }

    /// Common polling contract for peripheral wrappers. Ready consumes one
    /// coalesced event; Pending always has either a parked waiter or a wake
    /// already issued after an event arrived in the registration window.
    pub(crate) fn poll_wait(&self, cx: &mut Context<'_>) -> Poll<()> {
        if self.take_fired() {
            Poll::Ready(())
        } else {
            self.register(cx.waker());
            Poll::Pending
        }
    }
}

impl Default for IrqSignal {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(all(test, not(target_arch = "riscv32")))]
#[path = "asynch/tests.rs"]
mod tests;

#[cfg(all(test, not(target_arch = "riscv32")))]
#[path = "asynch/block_on_tests.rs"]
mod block_on_tests;
