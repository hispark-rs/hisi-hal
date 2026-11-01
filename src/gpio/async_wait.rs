//! GPIO wait lifecycle, independent of PAC, pad layout and executor.
//!
//! A backend owns its line on one hart. `pause` masks local delivery before
//! configuration/metadata changes; `configure` must leave delivery masked.
//! Other cores/NMIs must not use the line. Bank sharing is a backend obligation.
use core::{
    cell::Cell,
    future::Future,
    pin::Pin,
    task::{Context, Poll, Waker},
};
use critical_section::Mutex;

struct State<L: Copy> {
    line: Cell<Option<L>>,
    fired: Cell<bool>,
    waker: Cell<Option<Waker>>,
}
pub(crate) struct Signal<L: Copy>(Mutex<State<L>>);
impl<L: Copy + Eq> Signal<L> {
    pub(crate) const fn new() -> Self {
        Self(Mutex::new(State { line: Cell::new(None), fired: Cell::new(false), waker: Cell::new(None) }))
    }
    pub(crate) fn active_line(&self) -> Option<L> {
        critical_section::with(|cs| self.0.borrow(cs).line.get())
    }
    fn arm(&self, line: L) {
        let old = critical_section::with(|cs| {
            let s = self.0.borrow(cs);
            s.line.set(Some(line));
            s.fired.set(false);
            s.waker.take()
        });
        drop(old);
    }
    pub(crate) fn clear(&self) {
        let old = critical_section::with(|cs| {
            let s = self.0.borrow(cs);
            s.line.set(None);
            s.fired.set(false);
            s.waker.take()
        });
        drop(old);
    }
    /// Backend has observed and stopped this actual line before calling here.
    /// Unrelated, duplicate and retired notifications cannot complete a wait.
    pub(crate) fn notify(&self, line: L) -> bool {
        let (matched, wake) = critical_section::with(|cs| {
            let s = self.0.borrow(cs);
            if s.line.get() != Some(line) {
                return (false, None);
            }
            s.line.set(None);
            s.fired.set(true);
            (true, s.waker.take())
        });
        if let Some(waker) = wake {
            waker.wake();
        }
        matched
    }
}

pub(crate) trait WaitIo {
    type Line: Copy + Eq;
    fn line(&self) -> Self::Line;
    fn pause(&mut self);
    fn configure(&mut self);
    fn resume(&mut self);
    /// Stop/ack only the owned line; preserve other lines according to the
    /// backend's bank ownership. Must be bounded, not wait for external progress.
    fn stop(&mut self);
    /// Edge modes always return false. Level modes sample the actual input.
    fn level_matches(&mut self) -> bool;
}

pub(crate) struct Wait<'a, B: WaitIo> {
    io: B,
    signal: &'a Signal<B::Line>,
    started: bool,
}
impl<'a, B: WaitIo> Wait<'a, B> {
    pub(crate) fn new(io: B, signal: &'a Signal<B::Line>) -> Self {
        Self { io, signal, started: false }
    }
}
impl<B: WaitIo + Unpin> Future for Wait<'_, B> {
    type Output = ();
    fn poll(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<()> {
        let this = self.get_mut();
        let waker = cx.waker().clone();
        this.io.pause();
        if !this.started {
            this.io.configure();
            this.signal.arm(this.io.line());
            this.started = true;
        }
        // Publish the waiter before sampling levels or permitting IRQ delivery.
        let (old, fired) = critical_section::with(|cs| {
            let s = this.signal.0.borrow(cs);
            (s.waker.replace(Some(waker)), s.fired.get())
        });
        let ready = fired || this.io.level_matches();
        if ready {
            this.io.stop();
            this.signal.clear();
        } else {
            this.io.resume();
        }
        drop(old);
        if ready { Poll::Ready(()) } else { Poll::Pending }
    }
}
impl<B: WaitIo> Drop for Wait<'_, B> {
    fn drop(&mut self) {
        if self.started {
            self.io.stop();
            self.signal.clear();
        }
    }
}

#[cfg(all(test, not(target_arch = "riscv32")))]
#[path = "async_wait_tests.rs"]
mod tests;
