//! PWM driver for WS63 (8 channels, 32-bit).
use crate::peripherals::Pwm;
use core::marker::PhantomData;

pub struct PwmChannel<'d> {
    channel: u8,
    _marker: PhantomData<&'d ()>,
}

impl<'d> PwmChannel<'d> {
    pub fn new(_pwm: &Pwm<'d>, channel: u8) -> Self {
        assert!(channel < 8);
        Self { channel, _marker: PhantomData }
    }

    fn regs(&self) -> &'static ws63_pac::pwm::RegisterBlock {
        unsafe { &*Pwm::ptr() }
    }

    pub fn configure(&mut self, freq: u32, duty_percent: u8) {
        let r = self.regs();
        let pclk = crate::soc::ws63::SYSTEM_CLOCK_HZ;
        let period = pclk / freq;
        let duty = (period as u64 * duty_percent as u64 / 100) as u32;
        r.pwm_freq_l0().write(|w| unsafe { w.bits(period & 0xFFFF) });
        r.pwm_freq_h0().write(|w| unsafe { w.bits((period >> 16) & 0xFFFF) });
        r.pwm_duty_l0().write(|w| unsafe { w.bits(duty & 0xFFFF) });
        r.pwm_duty_h0().write(|w| unsafe { w.bits((duty >> 16) & 0xFFFF) });
    }

    pub fn enable(&mut self) {
        self.regs().pwm_en0().write(|w| unsafe { w.bits(1u32) });
    }
    pub fn disable(&mut self) {
        self.regs().pwm_en0().write(|w| unsafe { w.bits(0u32) });
    }
    pub fn set_polarity(&mut self, active_high: bool) {
        self.regs().pwm_portity0().write(|w| unsafe { w.bits(if active_high { 1u32 } else { 0u32 }) });
    }
    pub fn start(&mut self) {
        self.regs().pwm_start0().write(|w| unsafe { w.bits(1u32 << self.channel) });
    }
    pub fn set_pulse_count(&mut self, count: u32) {
        self.regs().pwm_period_val0().write(|w| unsafe { w.bits(count) });
    }
}
