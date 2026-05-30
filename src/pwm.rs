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
        // SAFETY: PAC peripheral pointer is a static physical MMIO address, always valid
        unsafe { &*Pwm::ptr() }
    }

    pub fn configure(&mut self, freq: u32, duty_percent: u8) {
        assert!(freq > 0, "PWM frequency must be non-zero");
        let r = self.regs();
        let pclk = crate::soc::ws63::SYSTEM_CLOCK_HZ;
        let period = pclk / freq;
        let duty = (period as u64 * duty_percent as u64 / 100) as u32;
        match self.channel {
            0 => {
                r.pwm_freq_l0().write(|w| unsafe { w.bits(period & 0xFFFF) });
                r.pwm_freq_h0().write(|w| unsafe { w.bits((period >> 16) & 0xFFFF) });
                r.pwm_duty_l0().write(|w| unsafe { w.bits(duty & 0xFFFF) });
                r.pwm_duty_h0().write(|w| unsafe { w.bits((duty >> 16) & 0xFFFF) });
            }
            1 => {
                r.pwm_freq_l1().write(|w| unsafe { w.bits(period & 0xFFFF) });
                r.pwm_freq_h1().write(|w| unsafe { w.bits((period >> 16) & 0xFFFF) });
                r.pwm_duty_l1().write(|w| unsafe { w.bits(duty & 0xFFFF) });
                r.pwm_duty_h1().write(|w| unsafe { w.bits((duty >> 16) & 0xFFFF) });
            }
            2 => {
                r.pwm_freq_l2().write(|w| unsafe { w.bits(period & 0xFFFF) });
                r.pwm_freq_h2().write(|w| unsafe { w.bits((period >> 16) & 0xFFFF) });
                r.pwm_duty_l2().write(|w| unsafe { w.bits(duty & 0xFFFF) });
                r.pwm_duty_h2().write(|w| unsafe { w.bits((duty >> 16) & 0xFFFF) });
            }
            3 => {
                r.pwm_freq_l3().write(|w| unsafe { w.bits(period & 0xFFFF) });
                r.pwm_freq_h3().write(|w| unsafe { w.bits((period >> 16) & 0xFFFF) });
                r.pwm_duty_l3().write(|w| unsafe { w.bits(duty & 0xFFFF) });
                r.pwm_duty_h3().write(|w| unsafe { w.bits((duty >> 16) & 0xFFFF) });
            }
            4 => {
                r.pwm_freq_l4().write(|w| unsafe { w.bits(period & 0xFFFF) });
                r.pwm_freq_h4().write(|w| unsafe { w.bits((period >> 16) & 0xFFFF) });
                r.pwm_duty_l4().write(|w| unsafe { w.bits(duty & 0xFFFF) });
                r.pwm_duty_h4().write(|w| unsafe { w.bits((duty >> 16) & 0xFFFF) });
            }
            5 => {
                r.pwm_freq_l5().write(|w| unsafe { w.bits(period & 0xFFFF) });
                r.pwm_freq_h5().write(|w| unsafe { w.bits((period >> 16) & 0xFFFF) });
                r.pwm_duty_l5().write(|w| unsafe { w.bits(duty & 0xFFFF) });
                r.pwm_duty_h5().write(|w| unsafe { w.bits((duty >> 16) & 0xFFFF) });
            }
            6 => {
                r.pwm_freq_l6().write(|w| unsafe { w.bits(period & 0xFFFF) });
                r.pwm_freq_h6().write(|w| unsafe { w.bits((period >> 16) & 0xFFFF) });
                r.pwm_duty_l6().write(|w| unsafe { w.bits(duty & 0xFFFF) });
                r.pwm_duty_h6().write(|w| unsafe { w.bits((duty >> 16) & 0xFFFF) });
            }
            7 => {
                r.pwm_freq_l7().write(|w| unsafe { w.bits(period & 0xFFFF) });
                r.pwm_freq_h7().write(|w| unsafe { w.bits((period >> 16) & 0xFFFF) });
                r.pwm_duty_l7().write(|w| unsafe { w.bits(duty & 0xFFFF) });
                r.pwm_duty_h7().write(|w| unsafe { w.bits((duty >> 16) & 0xFFFF) });
            }
            _ => unreachable!(),
        }
    }

    pub fn enable(&mut self) {
        match self.channel {
            0 => self.regs().pwm_en0().write(|w| unsafe { w.bits(1u32) }),
            1 => self.regs().pwm_en1().write(|w| unsafe { w.bits(1u32) }),
            2 => self.regs().pwm_en2().write(|w| unsafe { w.bits(1u32) }),
            3 => self.regs().pwm_en3().write(|w| unsafe { w.bits(1u32) }),
            4 => self.regs().pwm_en4().write(|w| unsafe { w.bits(1u32) }),
            5 => self.regs().pwm_en5().write(|w| unsafe { w.bits(1u32) }),
            6 => self.regs().pwm_en6().write(|w| unsafe { w.bits(1u32) }),
            7 => self.regs().pwm_en7().write(|w| unsafe { w.bits(1u32) }),
            _ => unreachable!(),
        };
    }
    pub fn disable(&mut self) {
        match self.channel {
            0 => self.regs().pwm_en0().write(|w| unsafe { w.bits(0u32) }),
            1 => self.regs().pwm_en1().write(|w| unsafe { w.bits(0u32) }),
            2 => self.regs().pwm_en2().write(|w| unsafe { w.bits(0u32) }),
            3 => self.regs().pwm_en3().write(|w| unsafe { w.bits(0u32) }),
            4 => self.regs().pwm_en4().write(|w| unsafe { w.bits(0u32) }),
            5 => self.regs().pwm_en5().write(|w| unsafe { w.bits(0u32) }),
            6 => self.regs().pwm_en6().write(|w| unsafe { w.bits(0u32) }),
            7 => self.regs().pwm_en7().write(|w| unsafe { w.bits(0u32) }),
            _ => unreachable!(),
        };
    }
    pub fn set_polarity(&mut self, active_high: bool) {
        let val = if active_high { 1u32 } else { 0u32 };
        match self.channel {
            0 => self.regs().pwm_portity0().write(|w| unsafe { w.bits(val) }),
            1 => self.regs().pwm_portity1().write(|w| unsafe { w.bits(val) }),
            2 => self.regs().pwm_portity2().write(|w| unsafe { w.bits(val) }),
            3 => self.regs().pwm_portity3().write(|w| unsafe { w.bits(val) }),
            4 => self.regs().pwm_portity4().write(|w| unsafe { w.bits(val) }),
            5 => self.regs().pwm_portity5().write(|w| unsafe { w.bits(val) }),
            6 => self.regs().pwm_portity6().write(|w| unsafe { w.bits(val) }),
            7 => self.regs().pwm_portity7().write(|w| unsafe { w.bits(val) }),
            _ => unreachable!(),
        };
    }
    pub fn start(&mut self) {
        self.regs().pwm_start0().write(|w| unsafe { w.bits(1u32 << self.channel) });
    }
    pub fn set_pulse_count(&mut self, count: u32) {
        match self.channel {
            0 => self.regs().pwm_period_val0().write(|w| unsafe { w.bits(count) }),
            1 => self.regs().pwm_period_val1().write(|w| unsafe { w.bits(count) }),
            2 => self.regs().pwm_period_val2().write(|w| unsafe { w.bits(count) }),
            3 => self.regs().pwm_period_val3().write(|w| unsafe { w.bits(count) }),
            4 => self.regs().pwm_period_val4().write(|w| unsafe { w.bits(count) }),
            5 => self.regs().pwm_period_val5().write(|w| unsafe { w.bits(count) }),
            6 => self.regs().pwm_period_val6().write(|w| unsafe { w.bits(count) }),
            7 => self.regs().pwm_period_val7().write(|w| unsafe { w.bits(count) }),
            _ => unreachable!(),
        };
    }
}

impl embedded_hal::pwm::ErrorType for PwmChannel<'_> {
    type Error = core::convert::Infallible;
}

impl embedded_hal::pwm::SetDutyCycle for PwmChannel<'_> {
    fn max_duty_cycle(&self) -> u16 {
        u16::MAX
    }

    fn set_duty_cycle(&mut self, duty: u16) -> Result<(), Self::Error> {
        let r = self.regs();
        let duty_val = duty as u32;
        match self.channel {
            0 => {
                unsafe { r.pwm_duty_l0().write(|w| w.bits(duty_val & 0xFFFF)) };
                unsafe { r.pwm_duty_h0().write(|w| w.bits((duty_val >> 16) & 0xFFFF)) };
            }
            1 => {
                unsafe { r.pwm_duty_l1().write(|w| w.bits(duty_val & 0xFFFF)) };
                unsafe { r.pwm_duty_h1().write(|w| w.bits((duty_val >> 16) & 0xFFFF)) };
            }
            2 => {
                unsafe { r.pwm_duty_l2().write(|w| w.bits(duty_val & 0xFFFF)) };
                unsafe { r.pwm_duty_h2().write(|w| w.bits((duty_val >> 16) & 0xFFFF)) };
            }
            3 => {
                unsafe { r.pwm_duty_l3().write(|w| w.bits(duty_val & 0xFFFF)) };
                unsafe { r.pwm_duty_h3().write(|w| w.bits((duty_val >> 16) & 0xFFFF)) };
            }
            4 => {
                unsafe { r.pwm_duty_l4().write(|w| w.bits(duty_val & 0xFFFF)) };
                unsafe { r.pwm_duty_h4().write(|w| w.bits((duty_val >> 16) & 0xFFFF)) };
            }
            5 => {
                unsafe { r.pwm_duty_l5().write(|w| w.bits(duty_val & 0xFFFF)) };
                unsafe { r.pwm_duty_h5().write(|w| w.bits((duty_val >> 16) & 0xFFFF)) };
            }
            6 => {
                unsafe { r.pwm_duty_l6().write(|w| w.bits(duty_val & 0xFFFF)) };
                unsafe { r.pwm_duty_h6().write(|w| w.bits((duty_val >> 16) & 0xFFFF)) };
            }
            7 => {
                unsafe { r.pwm_duty_l7().write(|w| w.bits(duty_val & 0xFFFF)) };
                unsafe { r.pwm_duty_h7().write(|w| w.bits((duty_val >> 16) & 0xFFFF)) };
            }
            _ => unreachable!(),
        }
        Ok(())
    }
}
