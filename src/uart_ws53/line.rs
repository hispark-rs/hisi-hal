//! Non-PRO v151 line configuration; clock/pad/reset handoff remains external.
use super::{DataBits, Parity, Uart, UartInstance, uart_regs};
use core::num::NonZeroU32;
use embedded_hal::delay::DelayNs;

/// Physical stop duration. One-and-a-half is available only with five data bits;
/// two stop bits require six, seven or eight data bits.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum StopDuration {
    /// One stop bit for any data length.
    One,
    /// One-and-a-half stop bits with five data bits.
    OneAndHalf,
    /// Two stop bits with six, seven or eight data bits.
    Two,
}

/// Validated WS53 non-PRO UART line settings for an explicitly supplied clock.
///
/// Matches pinned SDK 163d9150's 16x/8x sampling selection and truncated 6-bit
/// fractional divider. Unlike that C helper it rejects zero/overflowing integer
/// dividers and uses widened arithmetic. Construction does not configure or
/// prove the supplied clock, pads, reset or idle state. No requested baud is
/// silently clamped; inspect `actual_baud` for representational rounding.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct LineConfig {
    clock: NonZeroU32,
    requested_baud: NonZeroU32,
    integer: u16,
    fraction: u8,
    sampling: u8,
    data: DataBits,
    parity: Parity,
    stop: StopDuration,
}

impl LineConfig {
    /// Rejects unrepresentable baud and stop/data combinations. Clock must be
    /// the actual stable UART input frequency when applying this configuration.
    pub const fn new(clock: NonZeroU32, baud: u32, data: DataBits, parity: Parity, stop: StopDuration) -> Option<Self> {
        let Some(requested_baud) = NonZeroU32::new(baud) else { return None };
        if matches!((data, stop), (DataBits::Five, StopDuration::Two))
            || (!matches!(data, DataBits::Five) && matches!(stop, StopDuration::OneAndHalf))
        {
            return None;
        }
        let hz = clock.get() as u64;
        let sampling = if (baud as u64) * 16 <= hz { 16 } else { 8 };
        let denominator = (baud as u64) * sampling;
        let integer = hz / denominator;
        if integer == 0 || integer > u16::MAX as u64 {
            return None;
        }
        let fraction = ((hz % denominator) * 64 / denominator) as u8;
        Some(Self {
            clock,
            requested_baud,
            integer: integer as u16,
            fraction,
            sampling: sampling as u8,
            data,
            parity,
            stop,
        })
    }
    /// Declared stable input clock; no clock-tree measurement is implied.
    pub const fn clock_hz(self) -> NonZeroU32 {
        self.clock
    }
    /// Requested line baud, before divisor quantization.
    pub const fn requested_baud(self) -> NonZeroU32 {
        self.requested_baud
    }
    /// Representable baud, rounded down to whole baud, based on declared clock.
    pub const fn actual_baud(self) -> u32 {
        ((self.clock.get() as u64) * 64 / (self.sampling as u64 * (self.integer as u64 * 64 + self.fraction as u64)))
            as u32
    }
    /// Hardware samples per bit: either 8 or 16.
    pub const fn samples_per_bit(self) -> u8 {
        self.sampling
    }
    fn data_code(self) -> u8 {
        match self.data {
            DataBits::Five => 0,
            DataBits::Six => 1,
            DataBits::Seven => 2,
            DataBits::Eight => 3,
        }
    }

    pub(super) fn readback_matches(self, r: &crate::soc::pac::uart0::RegisterBlock) -> bool {
        let ctl = r.uart_ctl().read();
        let frame = !ctl.div_en().bit()
            && !ctl.xbreak().bit()
            && !ctl.sps().bit()
            && ctl.dlen().bits() == self.data_code()
            && ctl.pen().bit() == (self.parity != Parity::None)
            && ctl.eps().bit() == (self.parity == Parity::Even)
            && ctl.stp().bit() == (self.stop != StopDuration::One);
        r.uart_ctl().modify(|_, w| w.div_en().set_bit());
        let enabled = r.uart_ctl().read().div_en().bit();
        let integer = u16::from(r.div_l().read().div_l().bits()) | (u16::from(r.div_h().read().div_h().bits()) << 8);
        r.uart_ctl().modify(|_, w| w.div_en().clear_bit());
        let sampling = r.baud_ctl().read();
        frame
            && enabled
            && integer == self.integer
            && r.div_fra().read().div_fra().bits() == self.fraction
            && sampling.baud_div().bits() + 1 == self.sampling
            && sampling.sample_phase().bits() + 1 == self.sampling / 2
    }
}

impl<T: UartInstance> Uart<'_, T> {
    /// Configure only frame and divider, then wait the SDK's 10us settle period.
    /// Does not consume RX/error status, reset FIFOs, enable clock gates, change
    /// routing/pads or enable interrupts. Pending software receive errors remain.
    ///
    /// # Safety
    /// Both directions must be idle, with the peer silent and no queued RX/TX.
    /// No IRQ, DMA, other CPU or boot client may access this UART. The UART must
    /// already have its stable `config.clock_hz()` input clock, reset released,
    /// and correct pad routing under this application's ownership. The delay
    /// must progress independently of this UART. This is not a cold-start API:
    /// H1/L0 clock-gate definitions remain conflicting in the pinned SDK.
    pub unsafe fn configure_line(&mut self, config: LineConfig, delay: &mut impl DelayNs) {
        apply(&mut Pac(uart_regs(T::PORT)), config, delay);
    }
}

trait Registers {
    fn frame(&mut self, config: LineConfig);
    fn sampling(&mut self, samples: u8);
    fn divisor_access(&mut self, enabled: bool);
    fn divisor(&mut self, integer: u16, fraction: u8);
}
struct Pac(&'static crate::soc::pac::uart0::RegisterBlock);
impl Registers for Pac {
    fn frame(&mut self, c: LineConfig) {
        self.0.uart_ctl().modify(|_, w| unsafe {
            w.div_en()
                .clear_bit()
                .xbreak()
                .clear_bit()
                .dlen()
                .bits(c.data_code())
                .eps()
                .bit(c.parity == Parity::Even)
                .pen()
                .bit(c.parity != Parity::None)
                .sps()
                .clear_bit()
                .stp()
                .bit(c.stop != StopDuration::One)
        });
    }
    fn sampling(&mut self, samples: u8) {
        self.0.baud_ctl().modify(|_, w| unsafe { w.baud_div().bits(samples - 1).sample_phase().bits(samples / 2 - 1) });
    }
    fn divisor_access(&mut self, enabled: bool) {
        self.0.uart_ctl().modify(|_, w| w.div_en().bit(enabled));
    }
    fn divisor(&mut self, integer: u16, fraction: u8) {
        unsafe {
            self.0.div_l().write_with_zero(|w| w.div_l().bits(integer as u8));
            self.0.div_h().write_with_zero(|w| w.div_h().bits((integer >> 8) as u8));
            self.0.div_fra().write_with_zero(|w| w.div_fra().bits(fraction));
        }
    }
}
fn apply(io: &mut impl Registers, config: LineConfig, delay: &mut impl DelayNs) {
    io.frame(config);
    io.sampling(config.sampling);
    io.divisor_access(true);
    io.divisor(config.integer, config.fraction);
    io.divisor_access(false);
    delay.delay_us(10);
}

#[cfg(all(test, not(target_arch = "riscv32")))]
#[path = "line/tests.rs"]
mod tests;
