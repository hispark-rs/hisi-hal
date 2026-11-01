//! Shared one-shot lifecycle; no PAC, chip IDs, executor or time queue.
//!
//! The enclosing driver owns the channel on one hart and keeps its clock and
//! ISR routing valid. `stop` must mask this channel's IRQ before returning;
//! `arm` may deliver an IRQ immediately. No other core/NMI may use the channel.
//! These are backend/platform obligations, not provided by the metadata lock.
use core::{
    cell::Cell,
    future::Future,
    num::NonZeroU32,
    pin::Pin,
    task::{Context, Poll, Waker},
};
use critical_section::Mutex;

pub(crate) trait AlarmIo {
    /// Mask the local IRQ, stop/mask the counter and acknowledge stale state.
    /// Must be bounded; no waiting for external progress or user callbacks.
    fn stop(&mut self);
    /// Arm one nonzero period and enable delivery. Called only after stop.
    fn arm(&mut self, ticks: NonZeroU32);
    /// Check actual raw expiry. A false result must not acknowledge an expiry
    /// that arrives after the check (controller-only stale clearing is allowed).
    fn pending(&mut self) -> bool;
}

struct State {
    active: Cell<bool>,
    fired: Cell<bool>,
    waker: Cell<Option<Waker>>,
}
pub(crate) struct Signal(Mutex<State>);
impl Signal {
    pub(crate) const fn new() -> Self {
        Self(Mutex::new(State { active: Cell::new(false), fired: Cell::new(false), waker: Cell::new(None) }))
    }
}

pub(crate) fn cancel(io: &mut impl AlarmIo, signal: &Signal) {
    io.stop();
    let old = critical_section::with(|cs| {
        let s = signal.0.borrow(cs);
        s.active.set(false);
        s.fired.set(false);
        s.waker.take()
    });
    drop(old);
}

/// Call only for the owned channel, from its non-reentrant ISR (or with that
/// channel's IRQ disabled). Callbacks run after metadata access, still in the
/// caller's context. A spurious IRQ must not stop a running counter.
pub(crate) fn notify(io: &mut impl AlarmIo, signal: &Signal) -> bool {
    if !io.pending() {
        return false;
    }
    io.stop();
    let (fired, waker) = critical_section::with(|cs| {
        let s = signal.0.borrow(cs);
        if s.active.replace(false) {
            s.fired.set(true);
            (true, s.waker.take())
        } else {
            (false, None)
        }
    });
    if let Some(waker) = waker {
        waker.wake();
    }
    fired
}

/// Outer public futures must retain the actual peripheral ownership/borrow.
/// Dropping an unpolled Wait performs no hardware operation.
pub(crate) struct Wait<'a, B: AlarmIo> {
    io: B,
    signal: &'a Signal,
    ticks: NonZeroU32,
    started: bool,
}
impl<'a, B: AlarmIo> Wait<'a, B> {
    pub(crate) fn new(io: B, signal: &'a Signal, ticks: NonZeroU32) -> Self {
        Self { io, signal, ticks, started: false }
    }
}
impl<B: AlarmIo + Unpin> Future for Wait<'_, B> {
    type Output = ();
    fn poll(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<()> {
        let this = self.get_mut();
        let waker = cx.waker().clone();
        if !this.started {
            // Disable only this channel before resetting metadata. Publishing
            // the waiter before arm covers immediate expiry during programming.
            this.io.stop();
            let old = critical_section::with(|cs| {
                let s = this.signal.0.borrow(cs);
                s.fired.set(false);
                s.active.set(true);
                s.waker.replace(Some(waker))
            });
            this.started = true;
            drop(old);
            this.io.arm(this.ticks);
        } else {
            let old = critical_section::with(|cs| this.signal.0.borrow(cs).waker.replace(Some(waker)));
            drop(old);
        }
        let (ready, retired) = critical_section::with(|cs| {
            let s = this.signal.0.borrow(cs);
            if s.fired.replace(false) { (true, s.waker.take()) } else { (false, None) }
        });
        drop(retired);
        if ready { Poll::Ready(()) } else { Poll::Pending }
    }
}
impl<B: AlarmIo> Drop for Wait<'_, B> {
    fn drop(&mut self) {
        if self.started {
            cancel(&mut self.io, self.signal);
        }
    }
}

#[cfg(all(test, not(target_arch = "riscv32")))]
#[path = "async_alarm_tests.rs"]
mod tests;
