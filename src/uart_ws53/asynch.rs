//! Single-hart RX/TX adapters, each exclusively borrowing the whole UART.
use super::{ReceiveError, ReceiveInterrupt, Uart, UartInstance, uart_regs};
use crate::interrupt;
use core::{
    cell::Cell,
    future::poll_fn,
    marker::PhantomData,
    task::{Context, Poll, Waker},
};
use critical_section::Mutex;

static WAKERS: Mutex<[Cell<Option<Waker>>; 3]> = Mutex::new([const { Cell::new(None) }; 3]);

// Metadata only: no MMIO, callbacks, Waker cloning or destruction under CS.
trait WakerSlot {
    fn replace(&self, waker: Option<Waker>) -> Option<Waker>;
    fn take(&self) -> Option<Waker> {
        self.replace(None)
    }
}
struct PortSlot(super::UartPort);
impl WakerSlot for PortSlot {
    fn replace(&self, waker: Option<Waker>) -> Option<Waker> {
        critical_section::with(|cs| WAKERS.borrow(cs)[self.0.index()].replace(waker))
    }
}
#[cfg(test)]
impl WakerSlot for Cell<Option<Waker>> {
    fn replace(&self, waker: Option<Waker>) -> Option<Waker> {
        Cell::replace(self, waker)
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

trait RxIo {
    type Error;
    fn read(&mut self) -> nb::Result<u8, Self::Error>;
    fn arm(&mut self);
    fn disarm(&mut self);
}
struct PacRx<'a, 'd, T: UartInstance>(&'a mut Uart<'d, T>);
impl<T: UartInstance> RxIo for PacRx<'_, '_, T> {
    type Error = ReceiveError;
    fn read(&mut self) -> nb::Result<u8, Self::Error> {
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

// Only the slot methods hold a short metadata CS. Register before checking,
// then check again after arm: bytes/errors arriving at either boundary cannot
// leave a pending future without a registered wake or retained hardware source.
// Old/completed wakers are returned for destruction OUTSIDE the critical section.
type PollOutcome<E> = (Poll<Result<u8, E>>, Option<Waker>, Option<Waker>);
fn poll_rx<I: RxIo>(io: &mut I, slot: &impl WakerSlot, waker: Waker) -> PollOutcome<I::Error> {
    let old = slot.replace(Some(waker));
    io.disarm();
    let mut result = io.read();
    if matches!(result, Err(nb::Error::WouldBlock)) {
        io.arm();
        result = io.read();
    }
    match result {
        Err(nb::Error::WouldBlock) => (Poll::Pending, old, None),
        result => {
            io.disarm();
            let result = match result {
                Ok(byte) => Ok(byte),
                Err(nb::Error::Other(error)) => Err(error),
                Err(nb::Error::WouldBlock) => unreachable!(),
            };
            (Poll::Ready(result), old, slot.take())
        }
    }
}

fn cancel<T: UartInstance>(uart: &mut Uart<'_, T>) {
    PacRx(uart).disarm();
    let old = PortSlot(T::PORT).take();
    drop(old);
}
struct ReadWait<'a, 'd, T: UartInstance>(&'a mut Uart<'d, T>);
impl<T: UartInstance> ReadWait<'_, '_, T> {
    fn poll(&mut self, cx: &mut Context<'_>) -> Poll<Result<u8, ReceiveError>> {
        let waker = cx.waker().clone();
        let (result, old, completed) = poll_rx(&mut PacRx(self.0), &PortSlot(T::PORT), waker);
        drop((old, completed));
        result
    }
}
impl<T: UartInstance> Drop for ReadWait<'_, '_, T> {
    fn drop(&mut self) {
        cancel(self.0);
    }
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
        let mut wait = ReadWait(&mut *self.uart);
        let byte = poll_fn(|cx| wait.poll(cx)).await?;
        buf[0] = byte;
        Ok(1)
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
    struct Counter(AtomicUsize);
    impl Wake for Counter {
        fn wake(self: Arc<Self>) {
            self.0.fetch_add(1, Ordering::SeqCst);
        }
    }
    struct Io {
        reads: VecDeque<nb::Result<u8, u8>>,
        armed: bool,
    }
    impl RxIo for Io {
        type Error = u8;
        fn read(&mut self) -> nb::Result<u8, u8> {
            self.reads.pop_front().unwrap_or(Err(nb::Error::WouldBlock))
        }
        fn arm(&mut self) {
            self.armed = true;
        }
        fn disarm(&mut self) {
            self.armed = false;
        }
    }
    #[test]
    fn arrival_before_check_or_during_arm_completes_without_parked_waker() {
        for reads in [vec![Ok(0x53)], vec![Err(nb::Error::WouldBlock), Ok(0x53)]] {
            let mut io = Io { reads: reads.into(), armed: false };
            let slot = Cell::new(None);
            let count = Arc::new(Counter(AtomicUsize::new(0)));
            let (result, old, done) = poll_rx(&mut io, &slot, Waker::from(count.clone()));
            assert_eq!(result, Poll::Ready(Ok(0x53)));
            assert!(!io.armed);
            assert!(slot.take().is_none());
            drop((old, done));
            assert_eq!(Arc::strong_count(&count), 1);
        }
    }
    #[test]
    fn error_before_or_during_arm_is_reported_and_following_byte_preserved() {
        for mut reads in [vec![], vec![Err(nb::Error::WouldBlock)]] {
            reads.extend([Err(nb::Error::Other(7)), Ok(0x55)]);
            let mut io = Io { reads: reads.into(), armed: false };
            let slot = Cell::new(None);
            let waker = Waker::noop();
            assert_eq!(poll_rx(&mut io, &slot, waker.clone()).0, Poll::Ready(Err(7)));
            assert!(!io.armed);
            assert!(slot.take().is_none());
            assert_eq!(poll_rx(&mut io, &slot, waker.clone()).0, Poll::Ready(Ok(0x55)));
        }
    }
    #[test]
    fn pending_repoll_replaces_waker_and_notification_wakes_latest_task() {
        let mut io = Io { reads: VecDeque::new(), armed: false };
        let slot = Cell::new(None);
        let first = Arc::new(Counter(AtomicUsize::new(0)));
        let last = Arc::new(Counter(AtomicUsize::new(0)));
        assert_eq!(poll_rx(&mut io, &slot, Waker::from(first.clone())).0, Poll::Pending);
        assert!(io.armed);
        let (result, old, done) = poll_rx(&mut io, &slot, Waker::from(last.clone()));
        assert_eq!(result, Poll::Pending);
        drop((old, done));
        assert_eq!(Arc::strong_count(&first), 1);
        io.disarm();
        slot.take().unwrap().wake();
        assert_eq!(first.0.load(Ordering::SeqCst), 0);
        assert_eq!(last.0.load(Ordering::SeqCst), 1);
        io.reads.push_back(Ok(0xa5));
        assert_eq!(poll_rx(&mut io, &slot, Waker::from(last.clone())).0, Poll::Ready(Ok(0xa5)));
        assert!(!io.armed);
        assert_eq!(Arc::strong_count(&last), 1);
    }

    #[test]
    fn interrupt_during_disarm_schedules_repoll_before_pending_return() {
        struct Race<'a> {
            slot: &'a Cell<Option<Waker>>,
            interrupt_once: bool,
        }
        impl RxIo for Race<'_> {
            type Error = u8;
            fn read(&mut self) -> nb::Result<u8, u8> {
                Err(nb::Error::WouldBlock)
            }
            fn arm(&mut self) {}
            fn disarm(&mut self) {
                if core::mem::take(&mut self.interrupt_once) {
                    // Model the ISR taking and waking the newly registered
                    // waker before the pending poll returns.
                    self.slot.take().unwrap().wake();
                }
            }
        }
        let slot = Cell::new(None);
        let count = Arc::new(Counter(AtomicUsize::new(0)));
        let mut io = Race { slot: &slot, interrupt_once: true };
        assert!(poll_rx(&mut io, &slot, Waker::from(count.clone())).0.is_pending());
        assert_eq!(count.0.load(Ordering::SeqCst), 1);
        assert!(slot.take().is_none());
        // The wake schedules a later poll; that poll installs a fresh waker.
        assert!(poll_rx(&mut io, &slot, Waker::from(count.clone())).0.is_pending());
        slot.take().unwrap().wake();
        assert_eq!(count.0.load(Ordering::SeqCst), 2);
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
trait TxIo {
    fn ready(&mut self, operation: TxOperation) -> bool;
    fn arm(&mut self);
    fn disarm(&mut self);
}
struct PacTx<'a, 'd, T: UartInstance>(&'a mut Uart<'d, T>);
impl<T: UartInstance> TxIo for PacTx<'_, '_, T> {
    fn ready(&mut self, operation: TxOperation) -> bool {
        match operation {
            TxOperation::Write(byte) => self.0.try_write_byte(byte).is_ok(),
            TxOperation::Flush => self.0.tx_flushed(),
        }
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
fn poll_tx(
    io: &mut impl TxIo,
    slot: &impl WakerSlot,
    operation: TxOperation,
    waker: Waker,
) -> (Poll<()>, Option<Waker>, Option<Waker>) {
    let old = slot.replace(Some(waker));
    io.disarm();
    let mut ready = io.ready(operation);
    if !ready {
        io.arm();
        ready = io.ready(operation);
    }
    if ready {
        io.disarm();
        (Poll::Ready(()), old, slot.take())
    } else {
        (Poll::Pending, old, None)
    }
}
fn cancel_tx<T: UartInstance>(uart: &mut Uart<'_, T>) {
    PacTx(uart).disarm();
    drop(PortSlot(T::PORT).take());
}
struct TxWait<'a, 'd, T: UartInstance> {
    uart: &'a mut Uart<'d, T>,
    operation: TxOperation,
}
impl<T: UartInstance> TxWait<'_, '_, T> {
    fn poll(&mut self, cx: &mut Context<'_>) -> Poll<()> {
        let waker = cx.waker().clone();
        let (result, old, done) = poll_tx(&mut PacTx(self.uart), &PortSlot(T::PORT), self.operation, waker);
        drop((old, done));
        result
    }
}
impl<T: UartInstance> Drop for TxWait<'_, '_, T> {
    fn drop(&mut self) {
        cancel_tx(self.uart);
    }
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
        let mut wait = TxWait { uart: &mut *self.uart, operation: TxOperation::Write(byte) };
        poll_fn(|cx| wait.poll(cx)).await;
        Ok(1)
    }
    async fn flush(&mut self) -> Result<(), Self::Error> {
        let mut wait = TxWait { uart: &mut *self.uart, operation: TxOperation::Flush };
        poll_fn(|cx| wait.poll(cx)).await;
        Ok(())
    }
}

#[cfg(all(test, not(target_arch = "riscv32")))]
mod tx_tests {
    use super::*;
    use std::{
        collections::VecDeque,
        sync::{
            Arc,
            atomic::{AtomicUsize, Ordering},
        },
        task::Wake,
    };
    struct Count(AtomicUsize);
    impl Wake for Count {
        fn wake(self: Arc<Self>) {
            self.0.fetch_add(1, Ordering::SeqCst);
        }
    }
    struct Io {
        ready: VecDeque<bool>,
        writes: std::vec::Vec<u8>,
        armed: bool,
    }
    impl TxIo for Io {
        fn ready(&mut self, op: TxOperation) -> bool {
            let ready = self.ready.pop_front().unwrap_or(false);
            if ready && let TxOperation::Write(byte) = op {
                self.writes.push(byte);
            }
            ready
        }
        fn arm(&mut self) {
            self.armed = true;
        }
        fn disarm(&mut self) {
            self.armed = false;
        }
    }
    #[test]
    fn room_before_check_or_during_arm_writes_exactly_one_byte() {
        for ready in [std::vec![true], std::vec![false, true]] {
            let mut io = Io { ready: ready.into(), writes: std::vec![], armed: false };
            let slot = Cell::new(None);
            let (result, old, done) = poll_tx(&mut io, &slot, TxOperation::Write(0xa5), Waker::noop().clone());
            drop((old, done));
            assert_eq!(result, Poll::Ready(()));
            assert_eq!(io.writes, [0xa5]);
            assert!(!io.armed);
            assert!(slot.take().is_none());
        }
    }
    #[test]
    fn pending_repoll_has_no_unreported_write_and_replaces_waker() {
        let mut io = Io { ready: VecDeque::new(), writes: std::vec![], armed: false };
        let slot = Cell::new(None);
        let first = Arc::new(Count(AtomicUsize::new(0)));
        let last = Arc::new(Count(AtomicUsize::new(0)));
        assert_eq!(poll_tx(&mut io, &slot, TxOperation::Write(1), Waker::from(first.clone())).0, Poll::Pending);
        let (result, old, done) = poll_tx(&mut io, &slot, TxOperation::Write(1), Waker::from(last.clone()));
        drop((old, done));
        assert_eq!(result, Poll::Pending);
        assert!(io.writes.is_empty());
        assert_eq!(Arc::strong_count(&first), 1);
        io.disarm();
        slot.take().unwrap().wake();
        assert_eq!(last.0.load(Ordering::SeqCst), 1);
        io.ready.push_back(true);
        assert_eq!(poll_tx(&mut io, &slot, TxOperation::Write(1), Waker::from(last.clone())).0, Poll::Ready(()));
        assert_eq!(io.writes, [1]);
        assert_eq!(Arc::strong_count(&last), 1);
    }

    #[test]
    fn interrupt_during_disarm_schedules_repoll_without_writing() {
        struct Race<'a> {
            slot: &'a Cell<Option<Waker>>,
            interrupt_once: bool,
        }
        impl TxIo for Race<'_> {
            fn ready(&mut self, _: TxOperation) -> bool {
                false
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
        assert!(poll_tx(&mut io, &slot, TxOperation::Write(0xa5), Waker::from(count.clone())).0.is_pending());
        assert_eq!(count.0.load(Ordering::SeqCst), 1);
        assert!(slot.take().is_none());
        assert!(poll_tx(&mut io, &slot, TxOperation::Write(0xa5), Waker::from(count.clone())).0.is_pending());
        slot.take().unwrap().wake();
        assert_eq!(count.0.load(Ordering::SeqCst), 2);
    }
    #[test]
    fn fifo_notification_does_not_complete_flush_until_serializer_is_empty() {
        let mut io = Io { ready: VecDeque::new(), writes: std::vec![], armed: false };
        let slot = Cell::new(None);
        for _ in 0..3 {
            assert_eq!(poll_tx(&mut io, &slot, TxOperation::Flush, Waker::noop().clone()).0, Poll::Pending);
            io.disarm();
            slot.take().unwrap().wake();
        }
        io.ready.push_back(true);
        assert_eq!(poll_tx(&mut io, &slot, TxOperation::Flush, Waker::noop().clone()).0, Poll::Ready(()));
        assert!(io.writes.is_empty());
        assert!(!io.armed);
        assert!(slot.take().is_none());
    }
}
