//! UART request completion and serializer drain use different interrupt sources.
use super::super::asynch::{ChannelSlot, Slot, cancel, poll_engine};
use super::*;
use core::{
    cell::Cell,
    future::Future,
    pin::Pin,
    task::{Context, Poll, Waker},
};
use critical_section::Mutex;

static LINE_WAKERS: Mutex<[Cell<Option<Waker>>; 2]> = Mutex::new([const { Cell::new(None) }; 2]);
struct LineSlot(Port);
impl Slot for LineSlot {
    fn replace(&self, value: Option<Waker>) -> Option<Waker> {
        critical_section::with(|cs| LINE_WAKERS.borrow(cs)[self.0 as usize].replace(value))
    }
}

/// Starts one UART TX/RX DMA operation and waits for MDMA notification.
/// Completion is an element-count observation, not serializer completion,
/// received-byte integrity or physical bus drain. All owners remain in the
/// borrowed `UartTransfer` after completion, error and cancellation.
/// Dropping an unpolled future changes no hardware; after start, cancellation
/// requests channel halt/disable and removes its waker, without resetting UART,
/// discarding FIFO contents or cancelling accepted bus transactions.
/// No implicit timeout. Neither Send nor Sync; route IRQ58 on the owning hart.
///
/// ```compile_fail
/// use hisi_hal::{dma::uart::AsyncUartTransfer, peripherals::Uart0};
/// fn send<T: Send>() {}
/// send::<AsyncUartTransfer<'static,'static,Uart0<'static>>>();
/// ```
/// ```compile_fail
/// use hisi_hal::{dma::uart::UartTransfer, peripherals::Uart0};
/// fn exclusive(t: &mut UartTransfer<'_,Uart0<'_>>) {
///     let f = unsafe { t.run_async() }.unwrap();
///     t.request_stop(); drop(f);
/// }
/// ```
pub struct AsyncUartTransfer<'a, 'd, T: Instance> {
    transfer: &'a mut UartTransfer<'d, T>,
    started: bool,
    _local: PhantomData<*mut ()>,
}

/// Separately wait for a successfully completed TX DMA's FIFO and serializer.
/// Cancellation only masks UART notifications and removes the waker; accepted
/// bytes continue transmitting. Empty-FIFO notifications may wake repeatedly
/// before the final shift finishes. RX errors observed during polling remain
/// accumulated in the DMA transfer. No allocation is returned even on success.
/// Requires uninterrupted clocks or external cancellation/recovery; !Send/!Sync.
///
/// ```compile_fail
/// use hisi_hal::{dma::uart::AsyncTransmitterIdle, peripherals::Uart1};
/// fn send<T: Send>() {}
/// send::<AsyncTransmitterIdle<'static,'static,Uart1<'static>>>();
/// ```
pub struct AsyncTransmitterIdle<'a, 'd, T: Instance> {
    transfer: &'a mut UartTransfer<'d, T>,
    polled: bool,
    _local: PhantomData<*mut ()>,
}

impl<'d, T: Instance> UartTransfer<'d, T> {
    /// Start and await a ready DMA transfer; running/retained state is refused.
    /// Uses shared MDMA IRQ58; UART line interrupts remain disabled while DMA
    /// runs. A line error without a DMA notification may only be observed on a
    /// later poll; this does not promise lossless error capture. TX success still
    /// needs a separate transmitter-idle observation if wire completion matters.
    ///
    /// # Safety
    /// Meet `dma::on_async_interrupt`'s shared IRQ58 protocol for every concurrent
    /// controller user, on the same application hart. Preserve `from_configured`'s
    /// clocks, request pacing, coherent buffers, peer and exclusive UART/controller
    /// handoff. Global interrupts/threshold must allow delivery. Neither this call
    /// nor the returned result establishes physical request/drain qualification.
    pub unsafe fn run_async(&mut self) -> Result<AsyncUartTransfer<'_, 'd, T>, Error> {
        if self.engine.state != State::Ready {
            return Err(Error::NotReady);
        }
        Ok(AsyncUartTransfer { transfer: self, started: false, _local: PhantomData })
    }

    /// Await serializer empty after successful TX DMA (polling or async).
    /// RX, incomplete DMA and enabled UART IRQ/PTIM are refused without changes.
    ///
    /// # Safety
    /// Route this UART exclusively to `on_async_transmit_interrupt`, on the same
    /// hart, non-reentrantly for the borrow; global MIE/priority must permit its
    /// IRQ. Maintain the UART's clocks/pads and exclusive ownership. Wakers must
    /// be ISR-safe and schedule normal-context polling, not synchronously poll.
    /// The separate shared MDMA handler contract still applies to other channels.
    pub unsafe fn transmitter_idle_async(&mut self) -> Result<AsyncTransmitterIdle<'_, 'd, T>, Error> {
        if !self.completed || self.engine.io.mode.flow() != 1 {
            return Err(Error::NotReady);
        }
        let port = Port::of::<T>();
        if port.regs().intr_en().read().bits() != 0 || interrupt::is_enabled(port.interrupt()) {
            return Err(Error::UartBusy);
        }
        Ok(AsyncTransmitterIdle { transfer: self, polled: false, _local: PhantomData })
    }

    /// UART ISR for the idle future, independent of DMA IRQ58. Acknowledges IIR2,
    /// masks TX/local notification and wakes outside the metadata critical section.
    /// Does not read LSR/RX or access the borrowed Rust driver value.
    ///
    /// # Safety
    /// The live `transmitter_idle_async` UART routing contract must hold. This is
    /// not the handler for `uart::AsyncTransmitter`: their waker ownership differs.
    pub unsafe fn on_async_transmit_interrupt() -> crate::uart::TransmitInterrupt {
        let port = Port::of::<T>();
        let id = port.regs().intr_id().read().intr_id().bits();
        mask_line(port);
        let wake = LineSlot(port).replace(None);
        if let Some(w) = wake {
            w.wake();
        }
        match id {
            2 => crate::uart::TransmitInterrupt::FifoAvailable,
            1 => crate::uart::TransmitInterrupt::NoPending,
            x => crate::uart::TransmitInterrupt::Other(x),
        }
    }
}
impl<T: Instance> Future for AsyncUartTransfer<'_, '_, T> {
    type Output = Result<Completion, Error>;
    fn poll(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
        let this = self.get_mut();
        let slot = ChannelSlot(this.transfer.engine.io.id);
        let result = poll_engine(&mut this.transfer.engine, &mut this.started, &slot, cx.waker().clone(), || {
            unsafe { interrupt::enable(interrupt::Interrupt::M_DMA) };
        });
        this.transfer.completed |= matches!(result, Poll::Ready(Ok(_)));
        result
    }
}
impl<T: Instance> Drop for AsyncUartTransfer<'_, '_, T> {
    fn drop(&mut self) {
        let slot = ChannelSlot(self.transfer.engine.io.id);
        cancel(&mut self.transfer.engine, self.started, &slot);
    }
}
fn mask_line(port: Port) {
    port.regs().intr_en().modify(|_, w| w.tran_em_intr_en().clear_bit());
    unsafe { interrupt::disable(port.interrupt()) };
}
trait IdleIo {
    fn observe(&mut self) -> Result<bool, Error>;
    fn arm(&mut self);
    fn disarm(&mut self);
}
struct PacIdle<'a>(&'a mut PacRegisters, Port);
impl IdleIo for PacIdle<'_> {
    fn observe(&mut self) -> Result<bool, Error> {
        let (idle, flags) = self.0.observe();
        match LineErrors::from_flags(flags) {
            Some(e) => Err(Error::UartLine(e)),
            None => Ok(idle),
        }
    }
    fn arm(&mut self) {
        self.1.regs().intr_en().modify(|_, w| w.tran_em_intr_en().set_bit());
        unsafe { interrupt::enable(self.1.interrupt()) };
    }
    fn disarm(&mut self) {
        mask_line(self.1);
    }
}
fn poll_idle(io: &mut impl IdleIo, slot: &impl Slot, waker: Waker) -> Poll<Result<(), Error>> {
    drop(slot.replace(Some(waker)));
    io.disarm();
    let mut result = io.observe();
    if result == Ok(false) {
        io.arm();
        result = io.observe();
    }
    match result {
        Ok(false) => Poll::Pending,
        other => {
            io.disarm();
            drop(slot.replace(None));
            Poll::Ready(other.map(|_| ()))
        }
    }
}
impl<T: Instance> Future for AsyncTransmitterIdle<'_, '_, T> {
    type Output = Result<(), Error>;
    fn poll(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
        let this = self.get_mut();
        this.polled = true;
        let port = Port::of::<T>();
        poll_idle(&mut PacIdle(&mut this.transfer.engine.io, port), &LineSlot(port), cx.waker().clone())
    }
}
impl<T: Instance> Drop for AsyncTransmitterIdle<'_, '_, T> {
    fn drop(&mut self) {
        if self.polled {
            let port = Port::of::<T>();
            mask_line(port);
            drop(LineSlot(port).replace(None));
        }
    }
}

#[cfg(all(test, not(target_arch = "riscv32")))]
mod tests {
    use super::*;
    use std::{
        collections::VecDeque,
        sync::{
            Arc,
            atomic::{AtomicUsize, Ordering},
        },
        task::Wake,
    };
    struct Io {
        samples: VecDeque<Result<bool, Error>>,
        armed: bool,
        arms: u32,
    }
    impl IdleIo for Io {
        fn observe(&mut self) -> Result<bool, Error> {
            self.samples.pop_front().unwrap()
        }
        fn arm(&mut self) {
            self.armed = true;
            self.arms += 1;
        }
        fn disarm(&mut self) {
            self.armed = false;
        }
    }
    #[test]
    fn idle_before_or_during_arm_completes_without_retained_waker() {
        for samples in [std::vec![Ok(true)], std::vec![Ok(false), Ok(true)]] {
            let mut io = Io { samples: samples.into(), armed: false, arms: 0 };
            let slot = Cell::new(None);
            assert_eq!(poll_idle(&mut io, &slot, Waker::noop().clone()), Poll::Ready(Ok(())));
            assert!(!io.armed && slot.take().is_none());
        }
    }
    #[test]
    fn fifo_notifications_do_not_finish_until_serializer_empty() {
        let mut io = Io { samples: std::vec![Ok(false);4].into(), armed: false, arms: 0 };
        let slot = Cell::new(None);
        for _ in 0..2 {
            assert!(poll_idle(&mut io, &slot, Waker::noop().clone()).is_pending());
        }
        assert!(io.armed && io.arms == 2);
        io.samples.push_back(Ok(true));
        assert_eq!(poll_idle(&mut io, &slot, Waker::noop().clone()), Poll::Ready(Ok(())));
        assert!(!io.armed && slot.take().is_none());
    }
    #[test]
    fn interrupt_during_disarm_requests_a_repoll_before_pending() {
        struct Count(AtomicUsize);
        impl Wake for Count {
            fn wake(self: Arc<Self>) {
                self.0.fetch_add(1, Ordering::SeqCst);
            }
        }
        struct Race<'a> {
            slot: &'a Cell<Option<Waker>>,
            interrupt_once: bool,
        }
        impl IdleIo for Race<'_> {
            fn observe(&mut self) -> Result<bool, Error> {
                Ok(false)
            }
            fn arm(&mut self) {}
            fn disarm(&mut self) {
                if core::mem::take(&mut self.interrupt_once) {
                    self.slot.take().unwrap().wake();
                }
            }
        }
        let slot = Cell::new(None);
        let count = Arc::new(Count(AtomicUsize::new(0)));
        let mut io = Race { slot: &slot, interrupt_once: true };
        assert!(poll_idle(&mut io, &slot, Waker::from(count.clone())).is_pending());
        assert_eq!(count.0.load(Ordering::SeqCst), 1);
        assert!(slot.take().is_none());
        assert!(poll_idle(&mut io, &slot, Waker::from(count.clone())).is_pending());
        slot.take().unwrap().wake();
        assert_eq!(count.0.load(Ordering::SeqCst), 2);
    }
    #[test]
    fn line_error_before_or_during_arm_is_returned_and_notification_removed() {
        let error = Error::UartLine(LineErrors::from_flags(4).unwrap());
        for samples in [std::vec![Err(error)], std::vec![Ok(false), Err(error)]] {
            let mut io = Io { samples: samples.into(), armed: false, arms: 0 };
            let slot = Cell::new(None);
            assert_eq!(poll_idle(&mut io, &slot, Waker::noop().clone()), Poll::Ready(Err(error)));
            assert!(!io.armed && slot.take().is_none());
        }
    }
}
