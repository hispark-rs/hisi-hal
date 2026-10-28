//! WS53 source-profile temperature sampler (SDK 163d9150).
//!
//! This port has no evidenced stop, status-clear or IRQ contract. Reads return
//! the latest available sample, not proof of a new conversion. Dropping/releasing
//! the driver does not stop the sensor. Clock, thermal-service and power ownership
//! must be established before handing over the peripheral.
use crate::{peripherals::Tsensor, soc::pac};
use core::num::NonZeroU16;
use embedded_hal::delay::DelayNs;

/// A ten-bit ADC code; this does not assert a calibrated temperature range.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SampleCode(u16);
impl SampleCode {
    /// Reject values which cannot be represented by the ten-bit status field.
    pub const fn new(raw: u16) -> Option<Self> {
        if raw <= 1023 { Some(Self(raw)) } else { None }
    }
    /// Unmodified ten-bit code.
    pub const fn raw(self) -> u16 {
        self.0
    }
    /// Nominal SDK line between code118 (-40 C) and code933 (+125 C).
    /// Outside those reference endpoints return None instead of silently
    /// saturating. The raw code remains available; None is not a sensor fault.
    /// This is a source formula, not per-chip calibration or accuracy evidence.
    pub const fn nominal_temperature(self) -> Option<NominalTemperature> {
        if self.0 < 118 || self.0 > 933 {
            return None;
        }
        let centi = (self.0 as i32 - 118) * 16500 / 815 - 4000;
        Some(NominalTemperature(centi as i16))
    }
}
/// Nominal source-derived temperature in hundredths of a degree Celsius.
/// Resolution of this representation is not physical measurement accuracy.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct NominalTemperature(i16);
impl NominalTemperature {
    /// Source formula with signed integer truncation, in centidegrees Celsius.
    pub const fn centi_celsius(self) -> i16 {
        self.0
    }
    /// Whole degrees, truncated toward zero as in the SDK C implementation.
    pub const fn whole_celsius(self) -> i16 {
        self.0 / 100
    }
}
/// Read/initialization failure. None of these errors promises a stopped sensor.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Error {
    /// Initialization/recovery is required before reading.
    NotConfigured,
    /// CONTROL differs from the only evidenced reference profile (3).
    /// Preserve the unexpected value; do not overwrite another thermal service.
    UnsupportedControl(u32),
    /// No READY sample in the finite polling budget; a later read may succeed.
    Timeout,
}
/// Exclusive application-side handle for the source-profile sampler.
/// A nonzero unknown CONTROL value is rejected instead of trusted like the SDK.
///
/// ```compile_fail
/// use hisi_hal::{tsensor::TempSensor, peripherals::{Tsensor,Unclaimed}};
/// fn no_handoff(token:Unclaimed<Tsensor<'static>>) { let _ = TempSensor::new(token); }
/// ```
/// ```compile_fail
/// use hisi_hal::tsensor::TempSensor;
/// fn no_invented_stop(mut sensor:TempSensor<'_>) { sensor.stop(); }
/// ```
pub struct TempSensor<'d> {
    token: Tsensor<'d>,
    engine: Engine<PacRegisters>,
}
impl<'d> TempSensor<'d> {
    /// If CONTROL is zero, perform the five source-profile writes in order.
    /// If CONTROL is 3, attach without restarting; other values fail unchanged.
    /// Failure returns the retained driver for inspection, recovery or handback.
    ///
    /// The unsafe peripheral handoff must provide the inherited 32kHz source,
    /// power/reset and thermal-service ownership. No clock/reset register is
    /// guessed here. CONTROL readback verifies configuration, not running clocks.
    pub fn new(token: Tsensor<'d>) -> Result<Self, (Error, Self)> {
        let mut sensor = Self { token, engine: Engine { io: PacRegisters, configured: false } };
        match sensor.recover() {
            Ok(()) => Ok(sensor),
            Err(e) => Err((e, sensor)),
        }
    }
    /// Revalidate profile; reinitialize only if CONTROL is exactly zero.
    /// No stop/reset, READY clear or conversion freshness guarantee is implied.
    pub fn recover(&mut self) -> Result<(), Error> {
        self.engine.recover()
    }
    /// Read one coherent READY/code snapshot, or None if not ready.
    /// Repeated calls may return the same sample; reading does not acknowledge it.
    pub fn try_read(&mut self) -> Result<Option<SampleCode>, Error> {
        self.engine.try_read()
    }
    /// Bound status polling with 10us delays between attempts. The bound counts
    /// attempts, not a wall-clock deadline; the delay provider must progress.
    /// Timeout leaves the sampler running and permits retry. Never blocks IRQs.
    pub fn read_bounded(&mut self, polls: NonZeroU16, delay: &mut impl DelayNs) -> Result<SampleCode, Error> {
        self.engine.read_bounded(polls, delay)
    }
    /// Return the peripheral permission without writing registers or stopping
    /// the sensor. The receiver inherits the running/unknown hardware state and
    /// must preserve clock/power/thermal-service obligations until valid handoff.
    pub fn release_without_stopping(self) -> Tsensor<'d> {
        self.token
    }
}
trait Registers {
    fn control(&mut self) -> u32;
    fn initialize_profile(&mut self);
    fn sample(&mut self) -> Option<SampleCode>;
}
struct Engine<R: Registers> {
    io: R,
    configured: bool,
}
impl<R: Registers> Engine<R> {
    fn recover(&mut self) -> Result<(), Error> {
        self.configured = false;
        match self.io.control() {
            0 => self.io.initialize_profile(),
            3 => {}
            value => return Err(Error::UnsupportedControl(value)),
        }
        let control = self.io.control();
        if control != 3 {
            return Err(Error::UnsupportedControl(control));
        }
        self.configured = true;
        Ok(())
    }
    fn try_read(&mut self) -> Result<Option<SampleCode>, Error> {
        if !self.configured {
            return Err(Error::NotConfigured);
        }
        let control = self.io.control();
        if control != 3 {
            self.configured = false;
            return Err(Error::UnsupportedControl(control));
        }
        Ok(self.io.sample())
    }
    fn read_bounded(&mut self, polls: NonZeroU16, delay: &mut impl DelayNs) -> Result<SampleCode, Error> {
        for attempt in 0..polls.get() {
            if let Some(code) = self.try_read()? {
                return Ok(code);
            }
            if attempt + 1 < polls.get() {
                delay.delay_us(10);
            }
        }
        Err(Error::Timeout)
    }
}
struct PacRegisters;
impl Registers for PacRegisters {
    fn control(&mut self) -> u32 {
        unsafe { &*pac::Tsensor::PTR }.control().read().value().bits()
    }
    fn initialize_profile(&mut self) {
        let r = unsafe { &*pac::Tsensor::PTR };
        // Opaque source-profile words, named PAC fields. No invented bit layout.
        unsafe {
            r.auto_refresh_period().write_with_zero(|w| w.value().bits(160));
            r.control().write_with_zero(|w| w.value().bits(3));
            r.sample_time().write_with_zero(|w| w.value().bits(2));
            r.auto_refresh_config().write_with_zero(|w| w.value().bits(0));
            r.start().write_with_zero(|w| w.value().bits(1));
        }
    }
    fn sample(&mut self) -> Option<SampleCode> {
        let status = unsafe { &*pac::Tsensor::PTR }.auto_status().read();
        if status.ready().bit() { Some(SampleCode(status.code().bits())) } else { None }
    }
}
#[cfg(all(test, not(target_arch = "riscv32")))]
#[path = "tsensor_ws53/tests.rs"]
mod tests;
