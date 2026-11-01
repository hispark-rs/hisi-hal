//! UART1/H0 initialization through the measured M_CLKEN1.bit4 gate.
//! Only the caller's exclusive handoff makes the shared RMW and reset legal.
//! This does not select a source, negotiate a peer, configure pads or prove drain.
use super::{LineConfig, ResetError, SignalPath, Uart, Uart1, UartPort};
use crate::{interrupt, peripherals::Unclaimed, soc::pac};
use core::{cell::Cell, marker::PhantomData};
use embedded_hal::delay::DelayNs;

/// An unsuccessful UART1 clock/reset/line initialization.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum H0InitError {
    /// Local IRQ41 is enabled. No clock, reset or UART access was performed.
    InterruptsEnabled,
    /// Local IRQ41 is pending. No clock, reset or UART access was performed.
    PendingInterrupt,
    /// H0 reset is asserted; no clock write or UART access was performed.
    ResetAsserted,
    /// The entire shared halfword differed; hardware state is not established.
    ClockReadback {
        /// Original word with only bit4 set.
        expected: u16,
        /// Complete observed word, including unmodeled bits.
        observed: u16,
    },
    /// Clock admission succeeded but the measured reset/line sequence failed.
    Configuration(ResetError),
}

/// Failure retaining the original unclaimed reservation, not an operational UART.
/// Drop performs no clock/reset/FIFO access and does not establish handback.
///
/// ```compile_fail
/// use hisi_hal::uart::H0InitFailure;
/// fn still_unclaimed(error: H0InitFailure<'static>) {
///     let (token, _) = error.into_parts();
///     token.try_write_byte(0x55).unwrap();
/// }
/// ```
#[must_use = "retain the reservation and recover the platform before reuse"]
pub struct H0InitFailure<'d> {
    token: Unclaimed<Uart1<'d>>,
    error: H0InitError,
}
impl<'d> H0InitFailure<'d> {
    /// Inspect the failure without accessing hardware or relinquishing ownership.
    pub const fn error(&self) -> H0InitError {
        self.error
    }
    /// Recover the reservation and error for external recovery. The gate may be
    /// enabled and UART/reset/line settings may already have changed. This is not
    /// permission to attach, resume an old context or return memory to a DMA user.
    pub fn into_parts(self) -> (Unclaimed<Uart1<'d>>, H0InitError) {
        (self.token, self.error)
    }
}
impl core::fmt::Debug for H0InitFailure<'_> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("H0InitFailure").field("error", &self.error).finish_non_exhaustive()
    }
}
#[cfg(feature = "defmt")]
impl defmt::Format for H0InitFailure<'_> {
    fn format(&self, f: defmt::Formatter<'_>) {
        defmt::write!(f, "H0InitFailure({})", self.error);
    }
}

impl<'d> Uart<'d, Uart1<'d>> {
    /// Enable the measured H0 gate, reset UART1 and apply typed line settings.
    ///
    /// Accepts an unclaimed token directly: unlike `from_configured`, this call
    /// does not require bit4 already enabled. Rejects enabled/pending local IRQ41
    /// and asserted H0 reset before any clock write or UART MMIO. It sets bit4
    /// only when off, fences, waits10us through `delay`, and checks the complete
    /// halfword before the first UART access. The10us is an observation margin,
    /// not a newly established silicon gate-settling specification.
    ///
    /// Then runs `reset_and_configure`, including its IER/TX-idle refusals,
    /// FIFO discard and divider/readback checks, and checks the clock word again.
    /// Success leaves the gate enabled and local/peripheral interrupts disabled.
    /// Neither success, failure, Drop nor token handback disables the gate.
    /// No retry/rollback, source selection, pad change or pending-IRQ clear occurs.
    /// A stopped MMIO bus cannot be recovered by a software timeout here.
    ///
    /// # Hardware requirements
    /// Stable externally prepared source/bus power and clocks, readable M_CRG,
    /// and an independent progressing delay. This enables only H0's bit4; it
    /// does not establish a source frequency, pad supply or shared ownership.
    ///
    /// # Safety
    ///
    /// Establish exclusive application ownership of UART1, all previous CPU,
    /// IRQ/NMI, DMA and vendor/control-core clients, and their physically drained
    /// transactions. The peer must be silent and discard of queued RX permitted.
    /// Clock gating is not a quiescence proof. If the admitted UART has queued
    /// TX, it may continue after the gate opens; grant permission for that
    /// progress. The reset/configuration step refuses a still-busy transmitter.
    /// Serialize the entire M_CLKEN1 and UART_SOFT_RESET halfword transactions
    /// against every other core/handler/peripheral user throughout this call.
    /// No interrupt-disabled region here supplies that cross-master exclusion.
    ///
    /// Power and both independent bus access and the stable upstream input
    /// matching `config.clock_hz()` must already exist. An initially asserted
    /// H0 reset is refused through the independently accessible M_CRG status.
    /// M_CRG must be accessible even when the UART gate is off. Establish that
    /// gate admission plus the10us margin permits UART MMIO in this platform
    /// profile. `delay` must progress independently of UART1 and shared changes.
    /// For `Pins`, establish exclusive, correct physical pads before calling;
    /// internal loopback grants no pad ownership. Retain all clock/power and
    /// exclusivity obligations while the returned UART is used. These checks
    /// neither initialize a complete clock tree nor negotiate a live vendor peer.
    ///
    /// On failure or an interrupted sequence, retain resources and keep traffic
    /// stopped until external recovery; never resume the old UART/application
    /// context merely because a token was returned. Successful configuration is
    /// not a physical DMA-drain, source-frequency or cold-start proof.
    ///
    /// ```compile_fail
    /// use hisi_hal::{uart::{Uart,LineConfig,SignalPath}, peripherals::{Unclaimed,Uart1}};
    /// fn missing_handoff(p:Unclaimed<Uart1<'static>>, c:LineConfig,
    ///                    d:&mut impl embedded_hal::delay::DelayNs) {
    ///     let _ = Uart::initialize_h0(p,c,SignalPath::InternalLoopback,d);
    /// }
    /// ```
    /// ```compile_fail
    /// use hisi_hal::{uart::{Uart,LineConfig,SignalPath}, peripherals::{Unclaimed,Uart0}};
    /// fn wrong_port(p:Unclaimed<Uart0<'static>>, c:LineConfig,
    ///               d:&mut impl embedded_hal::delay::DelayNs) {
    ///     let _ = unsafe { Uart::initialize_h0(p,c,SignalPath::Pins,d) };
    /// }
    /// ```
    pub unsafe fn initialize_h0(
        token: Unclaimed<Uart1<'d>>,
        config: LineConfig,
        path: SignalPath,
        delay: &mut impl DelayNs,
    ) -> Result<Self, H0InitFailure<'d>> {
        // A local, nonescaping handle only. No UART MMIO before gate admission;
        // on failure the original unclaimed token is returned and this has no Drop.
        let mut uart = Self { _peripheral: PhantomData, pending_receive_error: Cell::new(None) };
        if let Err(error) = initialize(&mut Pac { uart: &mut uart }, config, path, delay) {
            return Err(H0InitFailure { token, error });
        }
        // All attachment requirements were now established by this sequence and
        // the caller's external ownership/source/pad promises.
        let _token = unsafe { token.assume_application_ownership() };
        Ok(uart)
    }
}

trait Registers {
    fn local_enabled(&mut self) -> bool;
    fn local_pending(&mut self) -> bool;
    fn reset_released(&mut self) -> bool;
    fn clock(&mut self) -> u16;
    fn enable_h0(&mut self);
    fn fence(&mut self);
    fn configure(&mut self, config: LineConfig, path: SignalPath, delay: &mut impl DelayNs) -> Result<(), ResetError>;
}
fn initialize(
    io: &mut impl Registers,
    config: LineConfig,
    path: SignalPath,
    delay: &mut impl DelayNs,
) -> Result<(), H0InitError> {
    if io.local_enabled() {
        return Err(H0InitError::InterruptsEnabled);
    }
    if io.local_pending() {
        return Err(H0InitError::PendingInterrupt);
    }
    if !io.reset_released() {
        return Err(H0InitError::ResetAsserted);
    }
    let before = io.clock();
    let expected = before | 0x10;
    if before != expected {
        io.enable_h0();
    }
    io.fence();
    delay.delay_us(10);
    let observed = io.clock();
    if observed != expected {
        return Err(H0InitError::ClockReadback { expected, observed });
    }
    io.configure(config, path, delay).map_err(H0InitError::Configuration)?;
    let observed = io.clock();
    if observed != expected {
        return Err(H0InitError::ClockReadback { expected, observed });
    }
    Ok(())
}
struct Pac<'a, 'd> {
    uart: &'a mut Uart<'d, Uart1<'d>>,
}
impl Registers for Pac<'_, '_> {
    fn local_enabled(&mut self) -> bool {
        interrupt::is_enabled(UartPort::Uart1.interrupt())
    }
    fn local_pending(&mut self) -> bool {
        interrupt::is_pending(UartPort::Uart1.interrupt())
    }
    fn reset_released(&mut self) -> bool {
        unsafe { &*pac::MCrg::PTR }.uart_soft_reset().read().uart_h0_reset_n().bit()
    }
    fn clock(&mut self) -> u16 {
        unsafe { &*pac::MCrg::PTR }.m_clken1().read().bits()
    }
    fn enable_h0(&mut self) {
        unsafe { &*pac::MCrg::PTR }.m_clken1().modify(|_, w| w.uart_h0_enable().set_bit());
    }
    fn fence(&mut self) {
        #[cfg(target_arch = "riscv32")]
        unsafe {
            core::arch::asm!("fence iorw, iorw", options(nostack));
        }
        #[cfg(not(target_arch = "riscv32"))]
        core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);
    }
    fn configure(&mut self, c: LineConfig, p: SignalPath, d: &mut impl DelayNs) -> Result<(), ResetError> {
        unsafe { self.uart.reset_and_configure(c, p, d) }
    }
}

#[cfg(all(test, not(target_arch = "riscv32")))]
#[path = "h0/tests.rs"]
mod tests;
