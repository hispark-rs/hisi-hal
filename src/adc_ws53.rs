//! WS53 v155 analog preparation and capture-word decoding.
//!
//! Calibration never starts a scan or writes diagnostic buffer addresses.
//! Experimental one-shot capture lives in [`capture`]. It retains a static
//! coherent buffer until an independent unsafe quiescence handoff; DONE or STOP
//! alone never expose data or reclaim memory. Silicon capture/cache/drain
//! qualification remains open.
use crate::{interrupt, peripherals::Adc, soc::pac};
use core::num::NonZeroU16;
use embedded_hal::delay::DelayNs;

#[path = "adc_ws53/capture.rs"]
pub mod capture;

/// ADC channel identity, distinct from SDK pin_t and board MIO/AIO numbers.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum Channel {
    /// Source pad AGPIO1.
    Channel0,
    /// Source pad AGPIO3.
    Channel1,
    /// Source pad AGPIO4.
    Channel2,
    /// Source pad AGPIO6/MGPIO1 (aliases one physical pad).
    Channel3,
    /// Source pad MGPIO22.
    Channel4,
    /// Source pad MGPIO8.
    Channel5,
    /// Source pad MGPIO9.
    Channel6,
    /// Source pad AGPIO0/MGPIO6 (aliases one physical pad).
    Channel7,
}
/// An immutable copy of one diagnostic capture word. Constructing this value
/// does not initiate or prove a conversion. Preserve reserved bits for diagnosis.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Sample(u32);
impl Sample {
    /// Decode a copied word after the caller has established buffer validity.
    pub const fn from_capture_word(word: u32) -> Self {
        Self(word)
    }
    /// Original word, including undocumented/reserved bits.
    pub const fn raw(self) -> u32 {
        self.0
    }
    /// Twelve-bit raw code; no voltage range or calibration accuracy claim.
    pub const fn code(self) -> u16 {
        ((self.0 >> 2) & 0xfff) as u16
    }
    /// Three-bit channel tag from the original C data format.
    pub const fn channel(self) -> Channel {
        match (self.0 >> 14) & 7 {
            0 => Channel::Channel0,
            1 => Channel::Channel1,
            2 => Channel::Channel2,
            3 => Channel::Channel3,
            4 => Channel::Channel4,
            5 => Channel::Channel5,
            6 => Channel::Channel6,
            _ => Channel::Channel7,
        }
    }
}
/// Calibration phase used in bounded failure reporting.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Stage {
    /// Automatic six-bit offset calibration.
    OffsetAuto,
    /// SDK piecewise correction loaded as a manual offset.
    OffsetManual,
    /// Capacitor calibration, including its separate error status.
    Capacitor,
}
/// Calibration failure; the token remains owned and no capture is started.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Error {
    /// A stage did not finish within the polling budget.
    Timeout(Stage),
    /// Explicit clear did not remove a previous completion/error status.
    StaleStatus(Stage),
    /// CAP_STATUS reported error, even if FINISHED was also asserted.
    Capacitor,
    /// Clock/reset/analog configuration did not read back as programmed.
    Configuration,
}
/// Result of the source-profile sequence, not physical accuracy qualification.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Calibration {
    /// Original six-bit automatic offset result.
    pub automatic_offset: u8,
    /// SDK piecewise correction written into the six-bit manual offset field.
    pub manual_offset: u8,
}
fn corrected_offset(value: u8) -> u8 {
    match value {
        0..=3 => 0,
        4..=31 => value - 3,
        32..=59 => value + 3,
        _ => 63,
    }
}
/// Exclusive analog calibration handle. Does not own or configure physical pads.
/// The unsafe handoff must establish stopped scans/diagnostic writers and valid
/// analog power/source clock before construction. Shared clocks remain enabled.
///
/// ```compile_fail
/// use hisi_hal::{adc::Calibrator, peripherals::{Adc,Unclaimed}};
/// fn needs_handoff(p:Unclaimed<Adc<'static>>) { let _ = Calibrator::new(p); }
/// ```
/// ```compile_fail
/// use hisi_hal::adc::Calibrator;
/// fn no_unverified_conversion(c:&mut Calibrator<'_>) { c.read(); }
/// ```
pub struct Calibrator<'d> {
    token: Adc<'d>,
    engine: Engine<PacRegisters>,
}
impl<'d> Calibrator<'d> {
    /// Reserve a handed-over ADC without accessing registers. `calibrate` runs
    /// the bounded sequence explicitly; construction does not turn on analog power.
    pub fn new(token: Adc<'d>) -> Self {
        Self { token, engine: Engine { io: PacRegisters, result: None } }
    }
    /// Enable gates, pulse ADC reset, prepare analog settings and run all three
    /// stages. Polls bounds each phase, with1us between attempts after the source
    /// settling delay. The delay provider must be independent and operational.
    /// No global critical section is held; failure requests both calibration
    /// enables off and invalidates the report. This is not proof of physical
    /// quiescence. Does not power off or restore trace.
    pub fn calibrate(&mut self, polls: NonZeroU16, delay: &mut impl DelayNs) -> Result<Calibration, Error> {
        self.engine.calibrate(polls, delay)
    }
    /// Last fully completed sequence; any new attempt invalidates the old report.
    pub fn result(&self) -> Option<Calibration> {
        self.engine.result
    }
    /// Return the exclusive ADC token without powering down or restoring any
    /// previous configuration. No scan was started by this driver. The next owner
    /// must retain the handoff's clock/power and no-concurrent-user obligations.
    pub fn release_without_powering_off(self) -> Adc<'d> {
        self.token
    }
}
#[derive(Clone, Copy, Default)]
struct Status {
    finished: bool,
    error: bool,
}
trait Registers {
    fn clocks(&mut self);
    fn reset(&mut self, released: bool);
    fn analog_first(&mut self);
    fn analog_release(&mut self);
    fn configuration_valid(&mut self) -> bool;
    fn prepare(&mut self, stage: Stage, manual: u8);
    fn enable(&mut self, stage: Stage);
    fn status(&mut self, stage: Stage) -> Status;
    fn offset(&mut self) -> u8;
    fn disable(&mut self);
    fn finish(&mut self);
}
struct Engine<R> {
    io: R,
    result: Option<Calibration>,
}
impl<R: Registers> Engine<R> {
    fn run_stage(
        &mut self,
        stage: Stage,
        manual: u8,
        polls: NonZeroU16,
        delay: &mut impl DelayNs,
    ) -> Result<(), Error> {
        self.io.prepare(stage, manual);
        // Clear before EVERY phase, including manual offset; the SDK can reuse
        // the automatic phase's stale FINISHED bit here. Never silently accept it.
        let status = self.io.status(stage);
        if status.finished || status.error {
            return Err(Error::StaleStatus(stage));
        }
        self.io.enable(stage);
        delay.delay_us(match stage {
            Stage::OffsetAuto => 100,
            Stage::OffsetManual => 128,
            Stage::Capacitor => 160,
        });
        for attempt in 0..polls.get() {
            if !self.io.configuration_valid() {
                return Err(Error::Configuration);
            }
            let status = self.io.status(stage);
            if status.error {
                return Err(Error::Capacitor);
            }
            if status.finished {
                self.io.disable();
                return Ok(());
            }
            if attempt + 1 < polls.get() {
                delay.delay_us(1);
            }
        }
        Err(Error::Timeout(stage))
    }
    fn calibrate(&mut self, polls: NonZeroU16, delay: &mut impl DelayNs) -> Result<Calibration, Error> {
        self.result = None;
        self.io.clocks();
        self.io.reset(false);
        delay.delay_us(1);
        self.io.reset(true);
        delay.delay_us(1);
        self.io.analog_first();
        delay.delay_us(5);
        self.io.analog_release();
        let attempt = (|| {
            if !self.io.configuration_valid() {
                return Err(Error::Configuration);
            }
            self.run_stage(Stage::OffsetAuto, 0, polls, delay)?;
            let automatic_offset = self.io.offset();
            let manual_offset = corrected_offset(automatic_offset);
            self.run_stage(Stage::OffsetManual, manual_offset, polls, delay)?;
            self.run_stage(Stage::Capacitor, manual_offset, polls, delay)?;
            self.io.finish();
            Ok(Calibration { automatic_offset, manual_offset })
        })();
        self.io.disable();
        if let Ok(report) = attempt {
            self.result = Some(report);
        }
        attempt
    }
}
struct PacRegisters;
impl PacRegisters {
    fn regs(&self) -> &'static pac::adc::RegisterBlock {
        unsafe { &*pac::Adc::PTR }
    }
}
impl Registers for PacRegisters {
    fn clocks(&mut self) {
        unsafe {
            interrupt::disable(interrupt::Interrupt::LSADC);
        }
        let clock = unsafe { &*pac::MCrg::PTR };
        clock.m_clken1().modify(|_, w| w.lsadc_apb_enable().set_bit());
        clock.global_clken().modify(|_, w| w.lsadc_enable().set_bit());
    }
    fn reset(&mut self, released: bool) {
        unsafe { &*pac::MCrg::PTR }.soft_reset1().modify(|_, w| w.lsadc_reset_n().bit(released));
    }
    fn analog_first(&mut self) {
        let r = self.regs();
        unsafe {
            r.analog_config(0).write_with_zero(|w| w.value().bits(0x2aa6010));
            r.analog_config(1).write_with_zero(|w| w.value().bits(0x2085));
            r.analog_config(2).write_with_zero(|w| w.value().bits(12));
            r.analog_control().write_with_zero(|w| w.analog_enable().bits(0x260).reset_n().clear_bit());
        }
    }
    fn analog_release(&mut self) {
        let r = self.regs();
        unsafe {
            r.analog_control().write_with_zero(|w| w.analog_enable().bits(0x3f8).reset_n().set_bit());
            r.analog_control().write_with_zero(|w| w.analog_enable().bits(0xffff).reset_n().set_bit());
        }
    }
    fn configuration_valid(&mut self) -> bool {
        let clock = unsafe { &*pac::MCrg::PTR };
        let analog = self.regs().analog_control().read();
        clock.m_clken1().read().lsadc_apb_enable().bit()
            && clock.global_clken().read().lsadc_enable().bit()
            && clock.soft_reset1().read().lsadc_reset_n().bit()
            && analog.reset_n().bit()
            && analog.analog_enable().bits() == 0xffff
    }
    fn prepare(&mut self, stage: Stage, manual: u8) {
        let r = self.regs();
        unsafe {
            match stage {
                Stage::OffsetAuto | Stage::OffsetManual => {
                    r.offset_state_clear().write_with_zero(|w| w.pulse().set_bit());
                    r.offset_state_clear().write_with_zero(|w| w.pulse().clear_bit());
                    // Explicit full profile for all evidenced fields, including
                    // inherited manual/inversion bits that SDK RMW can retain.
                    r.offset_calibration().write_with_zero(|w| {
                        w.mode()
                            .bit(stage == Stage::OffsetAuto)
                            .loop_select()
                            .clear_bit()
                            .inverse()
                            .clear_bit()
                            .accumulation_cycle()
                            .bits(0)
                            .range()
                            .bits(0)
                            .manual_data()
                            .bits(manual)
                            .data_inverse()
                            .clear_bit()
                    });
                    if stage == Stage::OffsetAuto {
                        r.offset_data_enable().write_with_zero(|w| w.enable().clear_bit());
                        r.offset_refresh().write_with_zero(|w| w.pulse().set_bit());
                        r.offset_refresh().write_with_zero(|w| w.pulse().clear_bit());
                    }
                }
                Stage::Capacitor => {
                    r.cap_finish_clear().write_with_zero(|w| w.pulse().set_bit());
                    r.cap_finish_clear().write_with_zero(|w| w.pulse().clear_bit());
                    r.gain_state_clear().write_with_zero(|w| w.pulse().set_bit());
                    r.gain_state_clear().write_with_zero(|w| w.pulse().clear_bit());
                    r.cap_refresh().write_with_zero(|w| w.pulse().set_bit());
                    r.cap_refresh().write_with_zero(|w| w.pulse().clear_bit());
                    r.cap_calibration().write_with_zero(|w| {
                        w.mode()
                            .set_bit()
                            .algorithm()
                            .clear_bit()
                            .accumulation_cycle()
                            .bits(2)
                            .start_index()
                            .bits(4)
                            .weight_select()
                            .set_bit()
                            .dac_delay()
                            .set_bit()
                            .manual_cap()
                            .bits(0)
                            .manual_sh_enable()
                            .clear_bit()
                            .manual_pn_select()
                            .clear_bit()
                    });
                }
            }
        }
    }
    fn enable(&mut self, stage: Stage) {
        let r = self.regs();
        unsafe {
            if stage == Stage::Capacitor {
                r.cap_enable().write_with_zero(|w| w.enable().set_bit());
            } else {
                r.offset_enable().write_with_zero(|w| w.enable().set_bit());
            }
            if stage == Stage::OffsetManual {
                r.offset_data_enable().write_with_zero(|w| w.enable().set_bit());
                r.offset_refresh().write_with_zero(|w| w.pulse().set_bit());
                r.offset_refresh().write_with_zero(|w| w.pulse().clear_bit());
            }
        }
    }
    fn status(&mut self, stage: Stage) -> Status {
        if stage == Stage::Capacitor {
            let s = self.regs().cap_status().read();
            Status { finished: s.finished().bit(), error: s.error().bit() }
        } else {
            Status { finished: self.regs().offset_status().read().finished().bit(), error: false }
        }
    }
    fn offset(&mut self) -> u8 {
        self.regs().offset_result().read().data().bits()
    }
    fn disable(&mut self) {
        unsafe {
            self.regs().offset_enable().write_with_zero(|w| w.enable().clear_bit());
            self.regs().cap_enable().write_with_zero(|w| w.enable().clear_bit());
        }
    }
    fn finish(&mut self) {
        unsafe {
            self.regs().analog_config(1).write_with_zero(|w| w.value().bits(0x2085));
        }
    }
}
#[cfg(all(test, not(target_arch = "riscv32")))]
#[path = "adc_ws53/tests.rs"]
mod tests;
