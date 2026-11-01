//! Single-hart RX/TX adapters, each exclusively borrowing the whole UART.
use super::async_wait::{Wait, WaitIo, WakerSlot};
use super::{ReceiveError, ReceiveInterrupt, Uart, UartInstance, uart_regs};
use crate::interrupt;
use core::{cell::Cell, future::poll_fn, marker::PhantomData, task::Waker};
use critical_section::Mutex;

static WAKERS: Mutex<[Cell<Option<Waker>>; 3]> = Mutex::new([const { Cell::new(None) }; 3]);

struct PortSlot(super::UartPort);
impl WakerSlot for PortSlot {
    fn replace(&self, waker: Option<Waker>) -> Option<Waker> {
        critical_section::with(|cs| WAKERS.borrow(cs)[self.0.index()].replace(waker))
    }
}
/// An async adapter cannot take over an already enabled UART interrupt.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum AsyncBindError {
    /// Disable both the UART interrupt enables and its local-controller source.
    InterruptsEnabled,
}

/// Exclusive, single-hart async RX borrow of an already configured WS53 UART.
///
/// Implements `embedded_io_async::Read`. Each nonempty read returns one byte;
/// errors have the same preservation/discard semantics as `Uart::read_byte`.
/// A pending read has not consumed a byte or changed its caller's buffer.
/// Dropping that future masks RX and the local IRQ, clears the pending latch
/// and removes its waker, preserving unread bytes/errors. The next read rearms.
/// Dropping the adapter also performs this cleanup. No DMA, allocation, hidden
/// executor, clock/pad changes or global-interrupt changes are performed.
///
/// Release this borrow before using the blocking UART or async transmitter.
/// Input can overrun while a task is not consuming it.
pub struct AsyncReceiver<'a, 'd, T: UartInstance> {
    uart: &'a mut Uart<'d, T>,
    // Its interrupt state and critical section belong to this application hart.
    _local: PhantomData<*mut ()>,
}

impl<'d, T: UartInstance> Uart<'d, T> {
    /// Borrow this configured UART for interrupt-driven reception.
    ///
    /// # Safety
    /// Install this instance's non-reentrant handler calling only
    /// `Uart::<T>::on_async_receive_interrupt` for UART service, on this same
    /// application hart. Keep that routing, the clock and handoff valid for
    /// the adapter's lifetime. Global interrupts and the controller priority
    /// must permit delivery while awaiting input. No other core, DMA or handler
    /// may read/reconfigure this UART. This extends `from_configured`'s contract;
    /// it does not establish clock/pad or cross-core ownership.
    pub unsafe fn async_receiver(&mut self) -> Result<AsyncReceiver<'_, 'd, T>, AsyncBindError> {
        if uart_regs(T::PORT).intr_en().read().bits() != 0 || interrupt::is_enabled(T::PORT.interrupt()) {
            return Err(AsyncBindError::InterruptsEnabled);
        }
        // Clear a stale latch/waker without draining RX or read-clear status.
        cancel::<T>(self);
        Ok(AsyncReceiver { uart: self, _local: PhantomData })
    }

    /// One-shot async RX ISR: mask/ack the notification, then wake the task.
    /// Does not read bytes/LSR or access the borrowed driver value.
    ///
    /// # Safety
    /// The requirements of `on_receive_interrupt` apply. Route only the UART
    /// bound by `async_receiver`, on its owning hart, and do not reenter this
    /// handler. Waking schedules the consumer; the waker must be ISR-safe.
    pub unsafe fn on_async_receive_interrupt() -> ReceiveInterrupt {
        let cause = unsafe { Self::on_receive_interrupt() };
        let waker = critical_section::with(|cs| WAKERS.borrow(cs)[T::PORT.index()].take());
        // Never run executor callbacks with the waker cell borrowed or inside
        // our critical section. ISR entry itself still has interrupts masked.
        if let Some(waker) = waker {
            waker.wake();
        }
        cause
    }
}

struct PacRx<'a, 'd, T: UartInstance>(&'a mut Uart<'d, T>);
impl<T: UartInstance> WaitIo for PacRx<'_, '_, T> {
    type Output = u8;
    type Error = ReceiveError;
    fn try_complete(&mut self) -> nb::Result<u8, Self::Error> {
        self.0.read_byte()
    }
    fn arm(&mut self) {
        self.0.set_receive_interrupts(true);
        unsafe { interrupt::enable(T::PORT.interrupt()) };
    }
    fn disarm(&mut self) {
        self.0.set_receive_interrupts(false);
        unsafe { interrupt::disable(T::PORT.interrupt()) };
    }
}

fn cancel<T: UartInstance>(uart: &mut Uart<'_, T>) {
    PacRx(uart).disarm();
    let old = PortSlot(T::PORT).take();
    drop(old);
}
impl<T: UartInstance> Drop for AsyncReceiver<'_, '_, T> {
    fn drop(&mut self) {
        cancel(self.uart);
    }
}
impl<T: UartInstance> embedded_io::ErrorType for AsyncReceiver<'_, '_, T> {
    type Error = ReceiveError;
}
impl<T: UartInstance> embedded_io_async::Read for AsyncReceiver<'_, '_, T> {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        if buf.is_empty() {
            return Ok(0);
        }
        let mut wait = Wait::new(PacRx(&mut *self.uart), PortSlot(T::PORT));
        let byte = poll_fn(|cx| wait.poll(cx)).await?;
        buf[0] = byte;
        Ok(1)
    }
}

/// One-shot TX notification. FIFO availability is not line drain completion.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum TransmitInterrupt {
    /// TX FIFO/THR request acknowledged by reading IIR=2.
    FifoAvailable,
    /// Stale/spurious notification (IIR=1).
    NoPending,
    /// Other source; this adapter does not service modem/RX interrupts.
    Other(u8),
}

/// Exclusive single-hart asynchronous sender for a configured WS53 UART.
///
/// `Write::write` accepts one byte per nonempty successful call. A Pending
/// write has accepted no byte, so dropping it preserves the caller's buffer
/// and sends nothing from that call. Default `write_all` may have completed
/// earlier writes before cancellation; it is not an atomic whole-buffer send.
/// `flush` waits for both FIFO and shift register to be empty, preserving RX
/// error observations just like `Uart::tx_flushed`. Cancelling flush disables
/// notification but leaves accepted/in-flight bytes transmitting. No TX reset,
/// halt, FIFO discard, DMA, global-MIE or clock/pad changes are performed.
///
/// Polls check status at most twice; they never busy-wait. When FIFO is empty
/// but a character is still shifting, rearming THRE may wake early repeatedly
/// until a later poll sees true line completion. A permanently stalled UART
/// requires external recovery or cancellation; no timeout is fabricated.
/// No concurrent RX/TX split or WFI executor is provided. The adapter is !Send
/// and !Sync because its interrupt state belongs to the owning hart.
///
/// ```compile_fail
/// use hisi_hal::{peripherals::Uart2, uart::AsyncTransmitter};
/// fn require_send<T: Send>() {}
/// require_send::<AsyncTransmitter<'static, 'static, Uart2<'static>>>();
/// ```
///
/// ```compile_fail
/// use hisi_hal::{peripherals::Uart2, uart::Uart};
/// fn exclusive(uart: &mut Uart<'_, Uart2<'_>>) {
///     let tx = unsafe { uart.async_transmitter() }.unwrap();
///     let _ = uart.read_byte();
///     drop(tx);
/// }
/// ```
pub struct AsyncTransmitter<'a, 'd, T: UartInstance> {
    uart: &'a mut Uart<'d, T>,
    _local: PhantomData<*mut ()>,
}
impl<'d, T: UartInstance> Uart<'d, T> {
    /// Borrow the entire UART for asynchronous writing and flush.
    /// Requires every IER bit (including programmable THRE) and the local UART
    /// interrupt to be disabled. Leaves FIFO thresholds and RX data unchanged.
    ///
    /// # Safety
    /// Route this instance exclusively to `on_async_transmit_interrupt` on the
    /// same application hart, non-reentrantly, for the full borrow. Preserve
    /// its configured clock/pads and exclusive application ownership; no other
    /// CPU, DMA or handler may consume or reconfigure it. Global MIE/priority
    /// must allow notification. Wakers must be ISR-safe. This extends the
    /// `from_configured` handoff; it does not establish cross-core ownership.
    pub unsafe fn async_transmitter(&mut self) -> Result<AsyncTransmitter<'_, 'd, T>, AsyncBindError> {
        if uart_regs(T::PORT).intr_en().read().bits() != 0 || interrupt::is_enabled(T::PORT.interrupt()) {
            return Err(AsyncBindError::InterruptsEnabled);
        }
        cancel_tx::<T>(self);
        Ok(AsyncTransmitter { uart: self, _local: PhantomData })
    }

    /// Acknowledge/mask one TX notification and wake outside the metadata CS.
    /// Does not read RX, LSR or the main-context borrowed driver value.
    ///
    /// # Safety
    /// The live `async_transmitter` handoff and ISR routing contract must hold.
    /// Call only in that UART's non-reentrant ISR or with its local IRQ disabled.
    pub unsafe fn on_async_transmit_interrupt() -> TransmitInterrupt {
        let r = uart_regs(T::PORT);
        let id = r.intr_id().read().intr_id().bits();
        r.intr_en().modify(|_, w| w.tran_em_intr_en().clear_bit());
        unsafe { interrupt::disable(T::PORT.interrupt()) };
        let waker = PortSlot(T::PORT).take();
        if let Some(waker) = waker {
            waker.wake();
        }
        match id {
            2 => TransmitInterrupt::FifoAvailable,
            1 => TransmitInterrupt::NoPending,
            v => TransmitInterrupt::Other(v),
        }
    }
}
#[derive(Clone, Copy)]
enum TxOperation {
    Write(u8),
    Flush,
}
struct PacTx<'a, 'd, T: UartInstance> {
    uart: &'a mut Uart<'d, T>,
    operation: TxOperation,
}
impl<T: UartInstance> WaitIo for PacTx<'_, '_, T> {
    type Output = ();
    type Error = core::convert::Infallible;
    fn try_complete(&mut self) -> nb::Result<(), Self::Error> {
        let ready = match self.operation {
            TxOperation::Write(byte) => self.uart.try_write_byte(byte).is_ok(),
            TxOperation::Flush => self.uart.tx_flushed(),
        };
        if ready { Ok(()) } else { Err(nb::Error::WouldBlock) }
    }
    fn arm(&mut self) {
        uart_regs(T::PORT).intr_en().modify(|_, w| w.tran_em_intr_en().set_bit());
        unsafe { interrupt::enable(T::PORT.interrupt()) };
    }
    fn disarm(&mut self) {
        uart_regs(T::PORT).intr_en().modify(|_, w| w.tran_em_intr_en().clear_bit());
        unsafe { interrupt::disable(T::PORT.interrupt()) };
    }
}
fn cancel_tx<T: UartInstance>(uart: &mut Uart<'_, T>) {
    // Disarm has no operation-specific side effects; never reset TX here.
    PacTx { uart, operation: TxOperation::Flush }.disarm();
    drop(PortSlot(T::PORT).take());
}
impl<T: UartInstance> Drop for AsyncTransmitter<'_, '_, T> {
    fn drop(&mut self) {
        cancel_tx(self.uart);
    }
}
impl<T: UartInstance> embedded_io::ErrorType for AsyncTransmitter<'_, '_, T> {
    type Error = core::convert::Infallible;
}
impl<T: UartInstance> embedded_io_async::Write for AsyncTransmitter<'_, '_, T> {
    async fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        let Some(&byte) = buf.first() else {
            return Ok(0);
        };
        let mut wait =
            Wait::new(PacTx { uart: &mut *self.uart, operation: TxOperation::Write(byte) }, PortSlot(T::PORT));
        poll_fn(|cx| wait.poll(cx)).await?;
        Ok(1)
    }
    async fn flush(&mut self) -> Result<(), Self::Error> {
        let mut wait = Wait::new(PacTx { uart: &mut *self.uart, operation: TxOperation::Flush }, PortSlot(T::PORT));
        poll_fn(|cx| wait.poll(cx)).await?;
        Ok(())
    }
}
