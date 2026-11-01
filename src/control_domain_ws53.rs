//! WS53 control-domain logic/clock release after an exclusive platform handoff.
//!
//! This narrow startup operation observes the public reset-status word, releases
//! only logic/clocks, waits on an independent timebase and rechecks that word.
//! It never starts the control CPU or accesses a peripheral inside the domain.
//! A successful readback is not proof of power, clock frequency, DMA coherency,
//! a working 32kHz source, or temperature-sensor ownership/readiness.

use crate::soc::pac;
use core::marker::PhantomData;
use embedded_hal::delay::DelayNs;

/// Failure to establish this exact release sequence; no rollback is attempted.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Error {
    /// The complete initial word was not zero. No command or delay was issued.
    /// Even a word of3 cannot be attached to another owner's active domain.
    /// This observation does not qualify resuming a previously saved context.
    UnsupportedInitialState(u32),
    /// The complete word was not3 after writing3, fencing and waiting5us.
    /// Hardware state is now unknown. Keep dependent peripherals untouched and
    /// use the platform's proven recovery path; never resume an old context.
    ReadbackMismatch(u32),
}

/// Evidence of one successful exclusive logic/clock release on this context.
///
/// This is neither a general peripheral token nor proof of an individual
/// driver's handoff. Keep it alive while using resources enabled by this
/// sequence, and meet each peripheral's separate ownership requirements.
/// Dropping it performs no register access and does not turn the domain off or
/// relinquish the caller's power/clock and exclusivity obligations.
///
/// ```compile_fail
/// use hisi_hal::control_domain::LogicClockLease;
/// fn copy_only<T: Copy>() {}
/// copy_only::<LogicClockLease>();
/// ```
/// ```compile_fail
/// use hisi_hal::control_domain::LogicClockLease;
/// fn clone_only<T: Clone>() {}
/// clone_only::<LogicClockLease>();
/// ```
/// ```compile_fail
/// use hisi_hal::control_domain::LogicClockLease;
/// fn send_only<T: Send>() {}
/// send_only::<LogicClockLease>();
/// ```
/// ```compile_fail
/// use hisi_hal::control_domain::LogicClockLease;
/// fn sync_only<T: Sync>() {}
/// sync_only::<LogicClockLease>();
/// ```
#[must_use = "retain this lease while using the released domain; dropping does not restore reset"]
pub struct LogicClockLease {
    _exclusive: PhantomData<*mut ()>,
}

/// Release control-domain logic/clocks, leaving the control CPU held in reset.
///
/// Only the exact initial word0 is accepted. This writes the single complete
/// command3, executes an I/O fence, waits at least5us through `delay`, then
/// requires the complete status word3. There is no retry, reset assertion,
/// read-modify-write, or access to TSENSOR/other domain peripherals.
/// No timeout can make a stalled MMIO access recoverable in software.
///
/// # Safety
///
/// The caller must have an explicit exclusive platform handoff covering the
/// entire control logic/clock/reset domain, including every CPU, interrupt
/// handler, DMA master, debugger and firmware service that could access it.
/// All such competing access must be quiescent; merely disabling application
/// interrupts is insufficient to exclude other bus masters. The control CPU
/// must remain held in reset, and nobody may change this word concurrently.
/// The reset-status/command register itself must be accessible without entering
/// the held domain. An observed zero alone establishes none of these facts.
///
/// The caller must independently establish and maintain the required power and
/// input clocks for the lifetime of all dependent use. `delay` must honor the
/// `DelayNs` minimum duration and progress without reading this domain or
/// relying on its interrupts, DMA, clocks or reset state. For Flash integration,
/// an exclusively handed-over TIMER in the separate5200 domain can supply
/// that delay; its clocks/IRQ state remain the caller's responsibility.
///
/// Subsequent peripheral access requires its own handoff. This lease does not
/// authorize starting the control CPU, supply a32kHz source or qualify DMA.
/// On any incomplete sequence (including a panic, exception or bus stall after
/// the write), assume unknown state and use the proven platform recovery path.
/// Do not write zero as rollback, probe held-domain peripherals or resume a
/// previously saved application context. This API performs no such recovery.
///
/// ```compile_fail
/// use embedded_hal::delay::DelayNs;
/// fn needs_explicit_handoff(delay: &mut impl DelayNs) {
///     let _ = hisi_hal::control_domain::release_logic_and_clock(delay);
/// }
/// ```
pub unsafe fn release_logic_and_clock(delay: &mut impl DelayNs) -> Result<LogicClockLease, Error> {
    release_with(&mut PacRegisters, delay)
}

trait Registers {
    fn reset_status(&mut self) -> u32;
    fn release_logic_clock_only(&mut self);
    fn fence(&mut self);
}

fn release_with(registers: &mut impl Registers, delay: &mut impl DelayNs) -> Result<LogicClockLease, Error> {
    let initial = registers.reset_status();
    if initial != 0 {
        return Err(Error::UnsupportedInitialState(initial));
    }
    registers.release_logic_clock_only();
    registers.fence();
    delay.delay_us(5);
    let observed = registers.reset_status();
    if observed != 3 {
        return Err(Error::ReadbackMismatch(observed));
    }
    Ok(LogicClockLease { _exclusive: PhantomData })
}

struct PacRegisters;
impl Registers for PacRegisters {
    fn reset_status(&mut self) -> u32 {
        // The whole word is significant: masking to bits0..2 would silently
        // accept unknown upper bits and another owner's/reset implementation.
        unsafe { &*pac::SystemMemory::PTR }.ccore_reset_status().read().bits()
    }

    fn release_logic_clock_only(&mut self) {
        // Generated WO alias, complete32-bit command. No read/modify/reset API
        // and no control-CPU release bit are used by this operation.
        unsafe {
            (&*pac::SystemMemory::PTR)
                .ccore_logic_clock_release_command()
                .write_with_zero(|w| w.logic_clock_release().bits(3));
        }
    }

    fn fence(&mut self) {
        #[cfg(target_arch = "riscv32")]
        unsafe {
            // Deliberately no nomem/readonly option: this orders compiler and
            // processor I/O before the independent timer and status readback.
            core::arch::asm!("fence iorw, iorw", options(nostack));
        }
        #[cfg(not(target_arch = "riscv32"))]
        core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);
    }
}

#[cfg(all(test, not(target_arch = "riscv32")))]
#[path = "control_domain_ws53/tests.rs"]
mod tests;
