//! Legacy WS63 UART register adapter; RX lifecycle is shared.
use super::async_wait::{Wait, WaitIo, WakerSlot};
use super::{Uart, UartInstance, UartPort, uart_regs, write_fifo_blocking_levels};
use crate::peripherals::{Uart0, Uart1, Uart2};
use core::{cell::Cell, convert::Infallible, future::poll_fn, task::Waker};
use critical_section::Mutex;
use embedded_io_async::{Read, Write};

static WAKERS: Mutex<[Cell<Option<Waker>>; 3]> = Mutex::new([const { Cell::new(None) }; 3]);
struct PortSlot(UartPort);
impl WakerSlot for PortSlot {
    fn replace(&self, waker: Option<Waker>) -> Option<Waker> {
        critical_section::with(|cs| WAKERS.borrow(cs)[self.0.index()].replace(waker))
    }
}

/// One-shot receive notification: mask RX and wake the waiting reader.
/// Leaves bytes in the hardware FIFO, including when no task is waiting.
///
/// Route UART0..2 IRQs (53..55) here on the owning hart; keep the UART clock
/// and local IRQ enabled while awaiting reception. This hook is for async
/// RX only, not a TX/modem/DMA interrupt dispatcher. Waking must schedule
/// a later poll, not synchronously reenter this handler or the driver.
pub fn on_interrupt(port: UartPort) {
    uart_regs(port).intr_en().modify(|_, w| w.rece_data_intr_en().clear_bit());
    if let Some(waker) = PortSlot(port).take() {
        waker.wake();
    }
}

struct PacRx<'a, 'd, T> {
    uart: &'a Uart<'d, T>,
    port: UartPort,
}
impl<T: UartInstance> WaitIo for PacRx<'_, '_, T> {
    type Output = u8;
    type Error = Infallible;
    fn try_complete(&mut self) -> nb::Result<u8, Self::Error> {
        // Use the same qualified status/data path as blocking reception.
        // The receive-count alias is not interchangeable across these chips.
        self.uart.read_byte().ok_or(nb::Error::WouldBlock)
    }
    fn arm(&mut self) {
        let r = uart_regs(self.port);
        // One-byte Read must not depend on the constructor's DMA-oriented
        // quarter-full threshold or an unverified character-timeout path.
        // This existing configuration keeps FIFO enabled with no reset strobes.
        write_fifo_blocking_levels(r);
        r.intr_en().modify(|_, w| w.rece_data_intr_en().set_bit());
    }
    fn disarm(&mut self) {
        uart_regs(self.port).intr_en().modify(|_, w| w.rece_data_intr_en().clear_bit());
    }
}

async fn read_one<T: UartInstance>(uart: &Uart<'_, T>, port: UartPort) -> u8 {
    let mut wait = Wait::new(PacRx { uart, port }, PortSlot(port));
    match poll_fn(|cx| wait.poll(cx)).await {
        Ok(byte) => byte,
        Err(never) => match never {},
    }
}

macro_rules! async_uart {
    ($uart:ty, $port:expr) => {
        // embedded_io::ErrorType is already implemented for the blocking impls;
        // the async Read/Write traits reuse it.
        impl Write for Uart<'_, $uart> {
            async fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
                // WS63 UART TX drains immediately on this model; write_byte polls
                // tx_fifo_full, so this completes without parking.
                for &b in buf {
                    self.write_byte(b);
                }
                Ok(buf.len())
            }
            async fn flush(&mut self) -> Result<(), Self::Error> {
                self.flush_tx();
                Ok(())
            }
        }
        impl Read for Uart<'_, $uart> {
            async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
                if buf.is_empty() {
                    return Ok(0);
                }
                buf[0] = read_one(self, $port).await;
                Ok(1)
            }
        }
    };
}
async_uart!(Uart0<'_>, UartPort::Uart0);
async_uart!(Uart1<'_>, UartPort::Uart1);
async_uart!(Uart2<'_>, UartPort::Uart2);
