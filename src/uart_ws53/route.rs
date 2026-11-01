//! SDK's single evidenced L0-to-application interrupt command.
//! No route readback, silicon reset value, alternate destination or clock-bit
//! meaning is inferred. UART0/1 do not share this L0-only API.
use super::{Uart, Uart2, UartPort, uart_regs};
use crate::interrupt;

/// A refused UART2 interrupt-route change. No routing command was issued.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum InterruptRouteError {
    /// Local IRQ42 or any peripheral IER bit is enabled.
    InterruptsEnabled,
    /// Local IRQ42 has an unacknowledged pending notification.
    PendingInterrupt,
}

impl Uart<'_, Uart2<'_>> {
    /// Issue the SDK full-word command1 routing L0 interrupts only to acore.
    ///
    /// Requires local IRQ42 disabled/not pending and all UART2 IER bits zero;
    /// otherwise returns without writing the route. Does not enable an IRQ,
    /// consume FIFO/error status, clear pending state, reset UART or alter
    /// clocks/pads. The route command has no established readback contract:
    /// success means command1 and an I/O fence were issued, not proof that a
    /// later hardware notification reached the CPU.
    ///
    /// # Safety
    /// Exclusively own UART2 and the complete shared GLB_CTL L0 routing
    /// transaction. Quiesce other cores, DMA, NMI/handlers and firmware users
    /// that could change routing or notification state. UART2/GLB_CTL MMIO
    /// clocks and reset state must remain accessible throughout. This extends
    /// the UART's existing clock/pad handoff; it establishes none of them and
    /// performs no cross-core ownership negotiation. Install the application's
    /// IRQ42 handler before subsequently enabling notifications. There is no
    /// evidenced rollback command; do not write0 on error or drop.
    ///
    /// ```compile_fail
    /// use hisi_hal::{uart::Uart, peripherals::Uart0};
    /// fn wrong_instance(uart: &mut Uart<'_, Uart0<'_>>) {
    ///     unsafe { uart.route_interrupt_to_application().unwrap(); }
    /// }
    /// ```
    pub unsafe fn route_interrupt_to_application(&mut self) -> Result<(), InterruptRouteError> {
        route(&mut PacRoute)
    }
}

trait Registers {
    fn local_enabled(&mut self) -> bool;
    fn local_pending(&mut self) -> bool;
    fn peripheral_enabled(&mut self) -> bool;
    fn command_application_only(&mut self);
    fn fence(&mut self);
}

struct PacRoute;
impl Registers for PacRoute {
    fn local_enabled(&mut self) -> bool {
        interrupt::is_enabled(UartPort::Uart2.interrupt())
    }
    fn local_pending(&mut self) -> bool {
        interrupt::is_pending(UartPort::Uart2.interrupt())
    }
    fn peripheral_enabled(&mut self) -> bool {
        uart_regs(UartPort::Uart2).intr_en().read().bits() != 0
    }
    fn command_application_only(&mut self) {
        // SDK163d9150 uart_port_config_pinmux, CONFIG_UART_MULTI_CORE_RESUME.
        unsafe {
            (&*crate::soc::pac::GlbCtl::PTR).uart_l0_irq_route_command().write_with_zero(|w| w.acore_only().bits(1));
        }
    }
    fn fence(&mut self) {
        #[cfg(target_arch = "riscv32")]
        unsafe {
            // No nomem/readonly: order compiler and device I/O accesses.
            core::arch::asm!("fence iorw, iorw", options(nostack));
        }
        #[cfg(not(target_arch = "riscv32"))]
        core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);
    }
}

fn route(io: &mut impl Registers) -> Result<(), InterruptRouteError> {
    if io.local_enabled() {
        return Err(InterruptRouteError::InterruptsEnabled);
    }
    if io.local_pending() {
        return Err(InterruptRouteError::PendingInterrupt);
    }
    if io.peripheral_enabled() {
        return Err(InterruptRouteError::InterruptsEnabled);
    }
    io.command_application_only();
    io.fence();
    Ok(())
}

#[cfg(all(test, not(target_arch = "riscv32")))]
mod tests {
    use super::*;
    use std::vec::Vec;

    struct Fake {
        enabled: bool,
        pending: bool,
        ier: u32,
        events: Vec<&'static str>,
    }
    impl Registers for Fake {
        fn local_enabled(&mut self) -> bool {
            self.events.push("local-enable");
            self.enabled
        }
        fn local_pending(&mut self) -> bool {
            self.events.push("local-pending");
            self.pending
        }
        fn peripheral_enabled(&mut self) -> bool {
            self.events.push("IER-read");
            self.ier != 0
        }
        fn command_application_only(&mut self) {
            self.events.push("command1");
        }
        fn fence(&mut self) {
            self.events.push("fence");
        }
    }
    fn idle() -> Fake {
        Fake { enabled: false, pending: false, ier: 0, events: Vec::new() }
    }
    #[test]
    fn command_is_fenced_after_quiescent_notifications_without_reading_route_or_fifo() {
        let mut io = idle();
        assert_eq!(route(&mut io), Ok(()));
        assert_eq!(io.events, ["local-enable", "local-pending", "IER-read", "command1", "fence"]);
    }
    #[test]
    fn enabled_local_source_refuses_without_accessing_uart() {
        let mut io = idle();
        io.enabled = true;
        assert_eq!(route(&mut io), Err(InterruptRouteError::InterruptsEnabled));
        assert_eq!(io.events, ["local-enable"]);
    }
    #[test]
    fn pending_source_is_preserved_and_never_read_cleared() {
        let mut io = idle();
        io.pending = true;
        assert_eq!(route(&mut io), Err(InterruptRouteError::PendingInterrupt));
        assert_eq!(io.events, ["local-enable", "local-pending"]);
        assert!(io.pending);
    }
    #[test]
    fn all_peripheral_enable_bits_including_unknown_ones_refuse_without_writes() {
        for ier in [1, 4, 5, 8, 16, 0x80000000, u32::MAX] {
            let mut io = idle();
            io.ier = ier;
            assert_eq!(route(&mut io), Err(InterruptRouteError::InterruptsEnabled));
            assert_eq!(io.events, ["local-enable", "local-pending", "IER-read"]);
            assert_eq!(io.ier, ier);
        }
    }
}
