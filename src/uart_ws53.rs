//! WS53 receive errors and trait integration. Register facts are in the PAC.
use super::{Uart, UartInstance, UartPort, uart_regs};
use crate::interrupt::{self, Interrupt};
use core::cell::Cell;

/// Cause captured before masking a WS53 UART receive notification.
///
/// This is a wake-up reason, not a count of bytes or an error acknowledgement.
/// Drain with `read_byte` in normal context to retain read-clear line errors.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ReceiveInterrupt {
    /// The configured receive FIFO threshold has been reached.
    DataAvailable,
    /// Receive line status needs service; reading it will acknowledge errors.
    LineStatus,
    /// A partial receive FIFO timed out. The hardware duration is not specified here.
    CharacterTimeout,
    /// No source was active when the handler read the interrupt ID.
    NoPending,
    /// Another source or an unknown ID. Receive recovery is not defined for it.
    Other(u8),
}

impl ReceiveInterrupt {
    fn from_id(id: u8) -> Self {
        // INTR_ID field encodings are described by the generated PAC and the
        // SDK hal_uart_v151_regs_op.h hal_uart_interrupt_t enum.
        match id {
            1 => Self::NoPending,
            4 => Self::DataAvailable,
            6 => Self::LineStatus,
            12 => Self::CharacterTimeout,
            other => Self::Other(other),
        }
    }
}

impl UartPort {
    /// WS53 application local interrupt, using the PAC's absolute cause number.
    pub const fn interrupt(self) -> Interrupt {
        match self {
            Self::Uart0 => Interrupt::UART_0,
            Self::Uart1 => Interrupt::UART_1,
            Self::Uart2 => Interrupt::UART_2,
        }
    }
}

/// Captured WS53 line-error flags. TX polls may coalesce multiple observations.
///
/// Framing/parity/break errors discard the affected head byte when present.
/// Overrun alone reports lost input but preserves queued bytes. The FIFO-wide
/// summary alone does not reject a good byte ahead of an errored character.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct ReceiveError {
    flags: u8,
}

impl ReceiveError {
    fn from_status(status: crate::soc::pac::uart0::line_status::R) -> Option<Self> {
        let flags = u8::from(status.rx_fifo_err().bit())
            | (u8::from(status.frame_err().bit()) << 1)
            | (u8::from(status.parity_err().bit()) << 2)
            | (u8::from(status.overrun_err().bit()) << 3)
            | (u8::from(status.break_intr().bit()) << 4);
        Self::from_flags(flags)
    }

    fn from_flags(flags: u8) -> Option<Self> {
        if flags & 0x1e == 0 { None } else { Some(Self { flags }) }
    }

    /// At capture time, at least one character in the receive FIFO had an error.
    pub const fn fifo_error(self) -> bool {
        self.flags & 1 != 0
    }
    /// A captured FIFO head had a framing error.
    pub const fn framing(self) -> bool {
        self.flags & 2 != 0
    }
    /// A captured FIFO head had a parity error.
    pub const fn parity(self) -> bool {
        self.flags & 4 != 0
    }
    /// Input was lost because the receive storage was full.
    pub const fn overrun(self) -> bool {
        self.flags & 8 != 0
    }
    /// A break sequence was detected.
    pub const fn break_detected(self) -> bool {
        self.flags & 16 != 0
    }
    fn discard_head(self) -> bool {
        self.framing() || self.parity() || self.break_detected()
    }
}

impl embedded_hal_nb::serial::Error for ReceiveError {
    fn kind(&self) -> embedded_hal_nb::serial::ErrorKind {
        use embedded_hal_nb::serial::ErrorKind;
        if self.overrun() {
            ErrorKind::Overrun
        } else if self.framing() || self.break_detected() {
            ErrorKind::FrameFormat
        } else if self.parity() {
            ErrorKind::Parity
        } else {
            ErrorKind::Other
        }
    }
}

impl embedded_io::Error for ReceiveError {
    fn kind(&self) -> embedded_io::ErrorKind {
        embedded_io::ErrorKind::InvalidData
    }
}

impl<T: UartInstance> Uart<'_, T> {
    /// True only when both the transmit FIFO and shift register are empty.
    ///
    /// LSR reads also acknowledge receive errors. Preserve these flags for the
    /// next nonempty read, immediately discarding only a corrupt FIFO head as
    /// `read_byte` would. Good queued bytes remain untouched. Repeated polls may
    /// coalesce several receive errors into one flag report, not an event count.
    /// Capturing an error may clear its hardware IRQ source; this does not raise
    /// a synthetic receive IRQ. IRQ-driven consumers must check `read_byte`
    /// before waiting/rearming rather than assuming every saved error still has
    /// an asserted hardware source.
    pub fn tx_flushed(&self) -> bool {
        capture_transmit_status(&PacStatus(uart_regs(T::PORT)), &self.pending_receive_error)
    }

    /// Block until the transmit FIFO and shift register are empty, preserving
    /// receive errors as described by `tx_flushed`. This blocking trait-style
    /// operation requires a progressing clock/transmitter; use `tx_flushed`
    /// with an external deadline when stalls must be bounded.
    pub fn flush_tx(&self) {
        while !self.tx_flushed() {
            core::hint::spin_loop();
        }
    }

    /// Enable or mask RX-data/character-timeout and line-error notifications.
    ///
    /// Preserves other enable bits and all queued data/errors. Does not enable
    /// the local interrupt controller or global interrupts: install the handler,
    /// configure the receive FIFO, then explicitly enable `T::PORT.interrupt()`.
    /// Masking does not clear an already latched controller IRQ; to cancel a
    /// notification, also call `interrupt::disable(T::PORT.interrupt())`.
    /// After `on_receive_interrupt`, drain data/errors and re-enable both this
    /// peripheral mask and the controller. Pending FIFO data can notify again;
    /// data below the configured threshold needs the hardware character timeout.
    pub fn set_receive_interrupts(&mut self, enabled: bool) {
        uart_regs(T::PORT)
            .intr_en()
            .modify(|_, w| w.rece_data_intr_en().bit(enabled).rece_line_stat_intr_en().bit(enabled));
    }

    /// Capture a one-shot notification without reading RX or read-clear LSR.
    ///
    /// Masks RX-data/error notifications and disables this UART's local IRQ,
    /// then clears the controller's pending latch. It never drains bytes, polls,
    /// or invokes user code. The caller records the result in Rust-owned state
    /// (or wakes a task); main context consumes bytes/errors and rearms. Bytes
    /// may continue arriving while masked, so slow consumers can still overrun.
    /// `Other` must be handled separately before rearming; this helper provides
    /// no TX/modem/busy service. `NoPending` can occur for a stale IRQ.
    ///
    /// # Safety
    /// Call only from this UART's non-reentrant handler, or with that IRQ
    /// disabled. A live `Uart<T>` must own the application handoff and clock;
    /// no other CPU, DMA or handler may consume/reconfigure the UART. The handler
    /// must not access the main-context driver value. Its only MMIO operations
    /// on this UART must be this helper (no competing RX/status reads).
    pub unsafe fn on_receive_interrupt() -> ReceiveInterrupt {
        let r = uart_regs(T::PORT);
        let cause = ReceiveInterrupt::from_id(r.intr_id().read().intr_id().bits());
        r.intr_en().modify(|_, w| w.rece_data_intr_en().clear_bit().rece_line_stat_intr_en().clear_bit());
        // disable() includes the SDK completion fence and pending-latch clear.
        unsafe { interrupt::disable(T::PORT.interrupt()) };
        cause
    }

    /// Read one byte without waiting, capturing line errors before consuming data.
    ///
    /// Returns `WouldBlock` for no data. A corrupt head byte is discarded once;
    /// overrun alone preserves queued bytes. A deferred error from a partial
    /// `embedded_io::Read` is delivered before further hardware reads. No other
    /// CPU, DMA or ISR may consume this UART (the constructor's handoff contract).
    pub fn read_byte(&mut self) -> nb::Result<u8, ReceiveError> {
        if let Some(error) = self.pending_receive_error.take() {
            return Err(nb::Error::Other(error));
        }
        let r = uart_regs(T::PORT);
        // A single read captures and acknowledges the read-clear flags. Do not
        // read status once per flag, which would lose simultaneous errors.
        let status = r.line_status().read();
        let available = status.data_available().bit_is_set();
        if let Some(error) = ReceiveError::from_status(status) {
            if available && error.discard_head() {
                let _ = r.rx().read().data().bits();
            }
            return Err(nb::Error::Other(error));
        }
        if available { Ok(r.rx().read().data().bits()) } else { Err(nb::Error::WouldBlock) }
    }
}

impl<T: UartInstance> embedded_io::ErrorType for Uart<'_, T> {
    type Error = ReceiveError;
}

impl<T: UartInstance> embedded_io::Read for Uart<'_, T> {
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        let mut count = 0;
        while count < buf.len() {
            match self.read_byte() {
                Ok(byte) => {
                    buf[count] = byte;
                    count += 1;
                }
                Err(nb::Error::WouldBlock) if count != 0 => break,
                Err(nb::Error::WouldBlock) => core::hint::spin_loop(),
                Err(nb::Error::Other(error)) if count == 0 => return Err(error),
                Err(nb::Error::Other(error)) => {
                    // Return already received bytes now; deliver the consumed
                    // line-error report exactly once on the next nonempty read.
                    self.pending_receive_error.set(Some(error));
                    break;
                }
            }
        }
        Ok(count)
    }
}

struct LineStatus {
    transmit_empty: bool,
    data_available: bool,
    error: Option<ReceiveError>,
}
trait StatusIo {
    fn status(&self) -> LineStatus;
    fn discard_corrupt_head(&self);
}
struct PacStatus(&'static crate::soc::pac::uart0::RegisterBlock);
impl StatusIo for PacStatus {
    fn status(&self) -> LineStatus {
        let status = self.0.line_status().read();
        LineStatus {
            transmit_empty: status.tx_empty_s().bit(),
            data_available: status.data_available().bit(),
            error: ReceiveError::from_status(status),
        }
    }
    fn discard_corrupt_head(&self) {
        let _ = self.0.rx().read().data().bits();
    }
}
fn capture_transmit_status(io: &impl StatusIo, pending: &Cell<Option<ReceiveError>>) -> bool {
    let status = io.status();
    if let Some(mut error) = status.error {
        // Discard based only on this fresh status, never on older deferred
        // flags: a previous errored byte was already consumed at capture time.
        if status.data_available && error.discard_head() {
            io.discard_corrupt_head();
        }
        if let Some(previous) = pending.get() {
            error.flags |= previous.flags;
        }
        pending.set(Some(error));
    }
    status.transmit_empty
}

impl<T: UartInstance> embedded_hal_nb::serial::ErrorType for Uart<'_, T> {
    type Error = ReceiveError;
}
impl<T: UartInstance> embedded_hal_nb::serial::Read for Uart<'_, T> {
    fn read(&mut self) -> nb::Result<u8, Self::Error> {
        self.read_byte()
    }
}
impl<T: UartInstance> embedded_hal_nb::serial::Write for Uart<'_, T> {
    fn write(&mut self, word: u8) -> nb::Result<(), Self::Error> {
        self.try_write_byte(word).map_err(|error| match error {
            nb::Error::WouldBlock => nb::Error::WouldBlock,
            nb::Error::Other(infallible) => match infallible {},
        })
    }
    fn flush(&mut self) -> nb::Result<(), Self::Error> {
        if self.tx_flushed() { Ok(()) } else { Err(nb::Error::WouldBlock) }
    }
}

#[cfg(all(test, not(target_arch = "riscv32")))]
mod tests {
    use super::*;

    #[test]
    fn transmit_status_preserves_errors_without_discarding_a_later_good_head() {
        use std::{cell::RefCell, collections::VecDeque};
        struct Mock {
            rows: RefCell<VecDeque<LineStatus>>,
            discarded: Cell<usize>,
        }
        impl StatusIo for Mock {
            fn status(&self) -> LineStatus {
                self.rows.borrow_mut().pop_front().unwrap()
            }
            fn discard_corrupt_head(&self) {
                self.discarded.set(self.discarded.get() + 1);
            }
        }
        let io = Mock {
            rows: RefCell::new(
                [
                    LineStatus { transmit_empty: false, data_available: true, error: ReceiveError::from_flags(3) },
                    LineStatus { transmit_empty: false, data_available: true, error: ReceiveError::from_flags(5) },
                    LineStatus { transmit_empty: true, data_available: true, error: ReceiveError::from_flags(8) },
                    LineStatus { transmit_empty: true, data_available: true, error: ReceiveError::from_flags(1) },
                ]
                .into(),
            ),
            discarded: Cell::new(0),
        };
        let pending = Cell::new(None);
        assert!(!capture_transmit_status(&io, &pending));
        assert!(!capture_transmit_status(&io, &pending));
        assert!(capture_transmit_status(&io, &pending));
        assert!(capture_transmit_status(&io, &pending));
        assert_eq!(io.discarded.get(), 2); // Old frame/parity flags do not drop the good head.
        let error = pending.take().unwrap();
        assert!(error.framing() && error.parity() && error.overrun() && error.fifo_error());
        assert_eq!(pending.take(), None);
    }

    #[test]
    fn empty_receive_fifo_never_causes_a_discard_during_transmit_poll() {
        struct Mock;
        impl StatusIo for Mock {
            fn status(&self) -> LineStatus {
                LineStatus { transmit_empty: false, data_available: false, error: ReceiveError::from_flags(16) }
            }
            fn discard_corrupt_head(&self) {
                panic!("empty RX must not be read");
            }
        }
        let pending = Cell::new(None);
        assert!(!capture_transmit_status(&Mock, &pending));
        assert!(pending.take().unwrap().break_detected());
    }

    #[test]
    fn receive_interrupt_ids_preserve_non_receive_causes() {
        for id in 0..16 {
            let expected = match id {
                1 => ReceiveInterrupt::NoPending,
                4 => ReceiveInterrupt::DataAvailable,
                6 => ReceiveInterrupt::LineStatus,
                12 => ReceiveInterrupt::CharacterTimeout,
                other => ReceiveInterrupt::Other(other),
            };
            assert_eq!(ReceiveInterrupt::from_id(id), expected);
        }
        assert_eq!(UartPort::Uart0.interrupt() as u16, 39);
        assert_eq!(UartPort::Uart1.interrupt() as u16, 41);
        assert_eq!(UartPort::Uart2.interrupt() as u16, 42);
    }

    #[test]
    fn all_line_flag_combinations_preserve_reports_and_only_discard_corrupt_heads() {
        for flags in 0u8..32 {
            let error = ReceiveError::from_flags(flags);
            if flags & 0x1e == 0 {
                assert_eq!(error, None, "FIFO summary alone must not reject a good head");
                continue;
            }
            let error = error.unwrap();
            assert_eq!(error.flags, flags);
            assert_eq!(error.discard_head(), flags & 0x16 != 0);
            assert_eq!(embedded_io::Error::kind(&error), embedded_io::ErrorKind::InvalidData);
            let expected = if flags & 8 != 0 {
                embedded_hal_nb::serial::ErrorKind::Overrun
            } else if flags & 0x12 != 0 {
                embedded_hal_nb::serial::ErrorKind::FrameFormat
            } else {
                embedded_hal_nb::serial::ErrorKind::Parity
            };
            assert_eq!(embedded_hal_nb::serial::Error::kind(&error), expected);
        }
    }
}
