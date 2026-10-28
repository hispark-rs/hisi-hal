//! H1/H0/L0 reset under an externally serialized clock/reset/pad handoff.
use super::{LineConfig, Uart, Uart0, Uart1, Uart2, UartInstance, UartPort, uart_regs};
use embedded_hal::delay::DelayNs;

/// Instances with measured H1/H0/L0 soft reset.
/// Sealed through `UartInstance` and the crate-owned peripheral token types.
///
/// ```no_run
/// use hisi_hal::{uart::{Uart, LineConfig, SignalPath}, peripherals::Uart2};
/// fn l0_reset_with_handoff(uart: &mut Uart<'_, Uart2<'_>>, config: LineConfig,
///                delay: &mut impl embedded_hal::delay::DelayNs) {
///     unsafe { uart.reset_and_configure(config, SignalPath::InternalLoopback, delay); }
/// }
/// ```
pub trait ResettableUart: UartInstance {}
impl ResettableUart for Uart0<'_> {}
impl ResettableUart for Uart1<'_> {}
impl ResettableUart for Uart2<'_> {}

/// Signal connection after resetting the modem configuration.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum SignalPath {
    /// Use pads whose routing, voltage and ownership were already established.
    Pins,
    /// Internal TX-to-RX loopback; no physical pad configuration is performed.
    InternalLoopback,
}

/// A refused reset or an observed reset/configuration failure.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ResetError {
    /// The selected UART is already held in reset; no UART registers were read.
    AlreadyAsserted,
    /// Peripheral interrupt enables are nonzero; no reset was issued.
    InterruptsEnabled,
    /// TX FIFO or serializer is still active; no reset was issued.
    TransmitBusy,
    /// The reset assertion or release did not read back. Release was attempted.
    ResetReadback,
    /// Reconfiguration did not read back; keep traffic stopped until recovery.
    ConfigurationReadback,
}

impl<T: ResettableUart> Uart<'_, T> {
    /// Discard queued RX, reset H1/H0/L0, and configure a FIFO-enabled UART.
    ///
    /// Rejects an asserted reset, enabled peripheral IRQs and unfinished TX.
    /// Performs the SDK halfword clear/set sequence, then a10us observation
    /// margin (not a specified minimum pulse width). Restores `config`, enables
    /// and clears both FIFOs, selects `path`, and leaves peripheral IRQs masked.
    /// Successfully resetting discards deferred software receive errors too.
    /// Does not change clock gates, pads, shared IRQ routing or the peer UART.
    /// Local-controller pending state must be cleared before IRQ rearming.
    ///
    /// # Safety
    /// Own this UART, its reset and the complete selected shared reset-word
    /// transaction (M_CRG for H1/H0; GLB_CTL for L0). Exclude other cores, DMA, handlers and competing reset-word
    /// RMW for this call. Local UART IRQ delivery must be disabled, all previous
    /// DMA requests drained, and the external peer silent. Permission to discard
    /// queued RX is required. Keep clocks/MMIO active and `config.clock_hz()`
    /// accurate, with an independent progressing delay. For `Pins`, the pads
    /// must already be configured and exclusively handed over. This does not
    /// establish any of those resources or negotiate with the control core.
    /// After a readback error, keep users/traffic stopped until a successful
    /// reinitialization or board recovery; do not resume a previous context.
    pub unsafe fn reset_and_configure(
        &mut self,
        config: LineConfig,
        path: SignalPath,
        delay: &mut impl DelayNs,
    ) -> Result<(), ResetError> {
        let result = pulse(&mut PacReset { uart: self }, delay);
        result?;
        self.pending_receive_error.set(None);
        let r = uart_regs(T::PORT);
        unsafe {
            r.intr_en().write_with_zero(|w| w);
            r.fifo_ctl().write_with_zero(|w| w.fifo_en().set_bit().rx_fifo_rst().set_bit().tx_fifo_rst().set_bit());
            r.modem_ctl().write_with_zero(|w| w.lb_mode().bit(path == SignalPath::InternalLoopback));
            self.configure_line(config, delay);
        }
        if !config.readback_matches(r)
            || r.rx_fifo_cnt().read().rx_fifo_level().bits() != 0
            || !r.intr_id().read().fifo_en_s().bit()
            || r.modem_ctl().read().lb_mode().bit() != (path == SignalPath::InternalLoopback)
        {
            return Err(ResetError::ConfigurationReadback);
        }
        Ok(())
    }
}

trait ResetRegisters {
    fn released(&mut self) -> bool;
    fn set_released(&mut self, released: bool);
    fn interrupts_enabled(&mut self) -> bool;
    fn transmit_idle(&mut self) -> bool;
}

fn pulse(io: &mut impl ResetRegisters, delay: &mut impl DelayNs) -> Result<(), ResetError> {
    if !io.released() {
        return Err(ResetError::AlreadyAsserted);
    }
    if io.interrupts_enabled() {
        return Err(ResetError::InterruptsEnabled);
    }
    if !io.transmit_idle() {
        return Err(ResetError::TransmitBusy);
    }
    io.set_released(false);
    let asserted = !io.released();
    // Attempt release even when assertion did not read back; never abandon a
    // newly asserted reset just because its diagnostic check failed.
    io.set_released(true);
    let released = io.released();
    delay.delay_us(10);
    if asserted && released { Ok(()) } else { Err(ResetError::ResetReadback) }
}

struct PacReset<'a, 'd, T: ResettableUart> {
    uart: &'a Uart<'d, T>,
}
impl<T: ResettableUart> ResetRegisters for PacReset<'_, '_, T> {
    fn released(&mut self) -> bool {
        match T::PORT {
            UartPort::Uart0 => unsafe { &*crate::soc::pac::MCrg::PTR }.uart_soft_reset().read().uart_h1_reset_n().bit(),
            UartPort::Uart1 => unsafe { &*crate::soc::pac::MCrg::PTR }.uart_soft_reset().read().uart_h0_reset_n().bit(),
            UartPort::Uart2 => {
                unsafe { &*crate::soc::pac::GlbCtl::PTR }.uart_soft_reset().read().uart_l0_reset_n().bit()
            }
        }
    }
    fn set_released(&mut self, released: bool) {
        let crg = unsafe { &*crate::soc::pac::MCrg::PTR };
        match T::PORT {
            UartPort::Uart0 => crg.uart_soft_reset().modify(|_, w| w.uart_h1_reset_n().bit(released)),
            UartPort::Uart1 => crg.uart_soft_reset().modify(|_, w| w.uart_h0_reset_n().bit(released)),
            UartPort::Uart2 => unsafe { &*crate::soc::pac::GlbCtl::PTR }
                .uart_soft_reset()
                .modify(|_, w| w.uart_l0_reset_n().bit(released)),
        };
    }
    fn interrupts_enabled(&mut self) -> bool {
        uart_regs(T::PORT).intr_en().read().bits() != 0
    }
    fn transmit_idle(&mut self) -> bool {
        self.uart.tx_flushed()
    }
}

#[cfg(all(test, not(target_arch = "riscv32")))]
#[path = "reset/tests.rs"]
mod tests;
