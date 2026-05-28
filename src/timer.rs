use crate::peripherals::Timer;
pub struct TimerDriver<'d> { _timer: Timer<'d> }
impl<'d> TimerDriver<'d> {
    pub fn new(timer: Timer<'d>) -> Self { Self { _timer: timer } }
    fn regs(&self) -> &'static ws63_pac::timer::RegisterBlock { unsafe { &*Timer::ptr() } }
    pub fn configure(&self, n: usize, mode: u8, load_value: u32) {
        let r = self.regs();
        match n {
            0 => r.timer0_load_count(0).write(|w| unsafe { w.bits(load_value) }),
            1 => r.timer0_load_count(1).write(|w| unsafe { w.bits(load_value) }),
            2 => r.timer0_load_count(2).write(|w| unsafe { w.bits(load_value) }),
            _ => unreachable!(),
        };
        let ctl = (mode as u32 & 0x3) << 1;
        match n {
            0 => r.timer0_control(0).write(|w| unsafe { w.bits(ctl) }),
            1 => r.timer0_control(1).write(|w| unsafe { w.bits(ctl) }),
            2 => r.timer0_control(2).write(|w| unsafe { w.bits(ctl) }),
            _ => unreachable!(),
        };
    }
    pub fn enable(&self, n: usize) {
        let r = self.regs();
        let prev = match n { 0 => r.timer0_control(0).read().bits(), 1 => r.timer0_control(1).read().bits(), 2 => r.timer0_control(2).read().bits(), _ => unreachable!() };
        match n { 0 => r.timer0_control(0).write(|w| unsafe { w.bits(prev | 1) }), 1 => r.timer0_control(1).write(|w| unsafe { w.bits(prev | 1) }), 2 => r.timer0_control(2).write(|w| unsafe { w.bits(prev | 1) }), _ => unreachable!() };
    }
    pub fn current_value(&self, n: usize) -> u32 {
        let r = self.regs();
        match n { 0 => r.timer0_current_value(0).read().bits(), 1 => r.timer0_current_value(1).read().bits(), 2 => r.timer0_current_value(2).read().bits(), _ => unreachable!() }
    }
    pub fn interrupt_pending(&self, n: usize) -> bool { (self.regs().raw_intr_stat().read().bits() >> n) & 1 != 0 }
    pub fn clear_interrupt(&self, _n: usize) { let _ = self.regs().eoi_ren().read().bits(); }
}
