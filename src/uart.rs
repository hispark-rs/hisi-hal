//! UART driver for WS63 (UART0/1/2, 16C550-compatible with FIFO).
//!
//! Baud rate: div = (div_h << 8 | div_l) + div_fra / 64.
//! Clock source: by default the 160 MHz PLL-derived UART clock
//! ([`crate::soc::chip::UART_CLOCK_HZ`]), NOT the 240 MHz CPU clock (vendor
//! `clock_init` sets the baud base to 160 MHz). Examples that skip `clock_init`
//! run on flashboot's raw-TCXO console clock (24/40 MHz, confirmed 40 MHz on
//! silicon) — they pass `Config::clock_hz = Some(soc::chip::uart_boot_clock_hz())`
//! so the divider matches the real base (issue #15/#10).

use crate::peripherals::{Uart0, Uart1, Uart2};
use core::marker::PhantomData;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DataBits {
    Five,
    Six,
    Seven,
    Eight,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Parity {
    None,
    Even,
    Odd,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StopBits {
    One,
    Two,
}

/// A validated UART baud rate. The 16-bit integer divider is
/// `div = UART_CLOCK_HZ / (16 · baud)`, so against the default 160 MHz PLL base the
/// realisable baud is `[~153, 10 000 000]`. `try_new` rejects anything outside that
/// instead of the old silent low-clamp (`baud < min_baud → min_baud`). Note: a
/// `Config.clock_hz` override (the 24/40 MHz flashboot console clock) shifts the
/// realisable range down — `BaudRate` validates against the default base, so pairing
/// a very high baud with a low override clock is the caller's responsibility (the
/// documented `clock_hz` footgun).
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct BaudRate(u32);

impl BaudRate {
    /// 115 200 baud (the default).
    pub const BAUD_115200: BaudRate = BaudRate(115_200);

    /// Construct from a baud rate. `None` if the 16-bit divider against the default
    /// 160 MHz base would fall outside `[1, 0xFFFF]` (baud too high or too low).
    pub const fn try_new(baud: u32) -> Option<Self> {
        if baud == 0 {
            return None;
        }
        // div = (UART_CLOCK_HZ * 4 / baud) >> 6  (the fixed-point form configure uses).
        let div = ((crate::soc::chip::UART_CLOCK_HZ as u64) * 4 / (baud as u64)) >> 6;
        if div < 1 || div > 0xFFFF {
            return None;
        }
        Some(BaudRate(baud))
    }

    /// The baud rate in Hz.
    pub const fn baud(self) -> u32 {
        self.0
    }
}

#[derive(Debug, Clone, Copy)]
pub struct Config {
    pub baudrate: BaudRate,
    pub data_bits: DataBits,
    pub parity: Parity,
    pub stop_bits: StopBits,
    /// UART baud-base clock in Hz. `None` = the post-`clock_init` PLL base
    /// ([`crate::soc::chip::UART_CLOCK_HZ`], 160 MHz). Examples that skip
    /// `clock_init` run on the flashboot console clock — the raw 24/40 MHz TCXO,
    /// NOT the PLL — so they must set this to
    /// `Some(crate::soc::chip::uart_boot_clock_hz())` for a correct divider on
    /// real hardware. Otherwise the baud is off by the PLL/TCXO ratio (≈4× on a
    /// 40 MHz board, confirmed on silicon), the root cause of garbled-console
    /// issue #15/#10.
    pub clock_hz: Option<u32>,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            baudrate: BaudRate::BAUD_115200,
            data_bits: DataBits::Eight,
            parity: Parity::None,
            stop_bits: StopBits::One,
            clock_hz: None,
        }
    }
}

pub struct Uart<'d, T> {
    _peripheral: PhantomData<&'d T>,
}

#[allow(dead_code)]
fn regs() -> &'static crate::soc::pac::uart0::RegisterBlock {
    // SAFETY: PAC peripheral pointer is a static physical MMIO address, always valid
    unsafe { &*Uart0::ptr() }
}

fn uart_ptr(idx: u8) -> *const crate::soc::pac::uart0::RegisterBlock {
    match idx {
        0 => Uart0::ptr(),
        1 => Uart1::ptr(),
        2 => Uart2::ptr(),
        _ => unreachable!(),
    }
}

fn uart_regs(idx: u8) -> &'static crate::soc::pac::uart0::RegisterBlock {
    // SAFETY: uart_ptr(idx) returns valid PAC MMIO addresses (UART0/1/2 at 0x4401_0000/1000/2000)
    unsafe { &*uart_ptr(idx) }
}

impl<'d> Uart<'d, Uart0<'d>> {
    pub fn new_uart0(_uart: Uart0<'d>, config: Config) -> Self {
        configure_uart(0, &config);
        Self { _peripheral: PhantomData }
    }
}

impl<'d> Uart<'d, Uart1<'d>> {
    pub fn new_uart1(_uart: Uart1<'d>, config: Config) -> Self {
        configure_uart(1, &config);
        Self { _peripheral: PhantomData }
    }
}

impl<'d> Uart<'d, Uart2<'d>> {
    pub fn new_uart2(_uart: Uart2<'d>, config: Config) -> Self {
        configure_uart(2, &config);
        Self { _peripheral: PhantomData }
    }
}

fn configure_uart(idx: u8, config: &Config) {
    let r = uart_regs(idx);

    // Enable divider access
    r.uart_ctl().modify(|_, w| unsafe { w.bits(0) });
    r.uart_ctl().write(|w| w.div_en().set_bit());

    // Set baud rate: div = UART_CLK / (16 * baudrate)
    // Valid range: div ∈ [1, 65535] (16-bit divider). `BaudRate::try_new` already
    // rejected any baud whose divider would fall outside that against the 160 MHz
    // PLL base, so no runtime clamp is needed (and `baud()` is never 0 → no div0).
    // `config.clock_hz` overrides the base for pre-`clock_init` examples (the
    // 24/40 MHz flashboot console clock); `None` keeps the 160 MHz PLL default.
    let pclk = config.clock_hz.unwrap_or(crate::soc::chip::UART_CLOCK_HZ);
    let baudrate = config.baudrate.baud();
    // div = pclk / (16 * baud), as fixed-point with 6 fractional bits (div_fra ∈
    // [0,63] sixty-fourths). Dropping the fraction (old div_fra=0) is fine at high
    // clocks but a significant baud error at the TCXO base — flashboot itself
    // programs a non-zero div_fra. (issue #15)
    let div64 = ((pclk as u64) * 4 / (baudrate as u64)) as u32; // = div * 64
    let div = div64 >> 6;
    let div_fra = (div64 & 0x3F) as u16;
    let div_l = (div & 0xFF) as u16;
    let div_h = ((div >> 8) & 0xFF) as u16;
    r.div_l().write(|w| unsafe { w.bits(div_l) });
    r.div_h().write(|w| unsafe { w.bits(div_h) });
    r.div_fra().write(|w| unsafe { w.bits(div_fra) });

    // Configure data bits, parity, stop bits
    let mut ctl = 0u16;
    ctl |= match config.data_bits {
        DataBits::Five => 0,
        DataBits::Six => 1 << 2,
        DataBits::Seven => 2 << 2,
        DataBits::Eight => 3 << 2,
    };
    match config.parity {
        Parity::Even => {
            ctl |= 1 << 5;
            ctl |= 1 << 4;
        }
        Parity::Odd => {
            ctl |= 1 << 5;
        }
        Parity::None => {}
    }
    if matches!(config.stop_bits, StopBits::Two) {
        ctl |= 1 << 7;
    }
    r.uart_ctl().write(|w| unsafe { w.bits(ctl | (1 << 0)) }); // div_en=1

    // Enable FIFO
    r.fifo_ctl().write(|w| unsafe { w.bits(0x01) });

    // Clear FIFO
    r.fifo_ctl().write(|w| unsafe { w.bits(0x07) });
}

impl<T> Uart<'_, T> {
    pub fn write_byte(&self, idx: u8, byte: u8) {
        let r = uart_regs(idx);
        while r.fifo_status().read().tx_fifo_full().bit_is_set() {}
        r.data().write(|w| unsafe { w.bits(byte as u16) });
    }

    pub fn read_byte(&self, idx: u8) -> Option<u8> {
        let r = uart_regs(idx);
        if r.fifo_status().read().rx_fifo_empty().bit_is_set() { None } else { Some(r.data().read().bits() as u8) }
    }

    pub fn flush_tx(&self, idx: u8) {
        let r = uart_regs(idx);
        while !r.fifo_status().read().tx_fifo_empty().bit_is_set() {}
    }

    /// Non-blocking check: returns true if TX FIFO is fully drained.
    pub fn tx_flushed(&self, idx: u8) -> bool {
        uart_regs(idx).fifo_status().read().tx_fifo_empty().bit_is_set()
    }

    pub fn write(&self, idx: u8, data: &[u8]) {
        for &b in data {
            self.write_byte(idx, b);
        }
    }

    pub fn uart_regs(&self, idx: u8) -> &'static crate::soc::pac::uart0::RegisterBlock {
        uart_regs(idx)
    }
}

impl embedded_io::ErrorType for Uart<'_, Uart0<'_>> {
    type Error = core::convert::Infallible;
}
impl embedded_io::Write for Uart<'_, Uart0<'_>> {
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        for &b in buf {
            self.write_byte(0, b);
        }
        Ok(buf.len())
    }
    fn flush(&mut self) -> Result<(), Self::Error> {
        self.flush_tx(0);
        Ok(())
    }
}
impl embedded_io::Read for Uart<'_, Uart0<'_>> {
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        let mut n = 0;
        for b in buf.iter_mut() {
            if let Some(byte) = self.read_byte(0) {
                *b = byte;
                n += 1;
            } else {
                break;
            }
        }
        Ok(n)
    }
}

// UART1 embedded-io traits
impl embedded_io::ErrorType for Uart<'_, Uart1<'_>> {
    type Error = core::convert::Infallible;
}
impl embedded_io::Write for Uart<'_, Uart1<'_>> {
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        for &b in buf {
            self.write_byte(1, b);
        }
        Ok(buf.len())
    }
    fn flush(&mut self) -> Result<(), Self::Error> {
        self.flush_tx(1);
        Ok(())
    }
}
impl embedded_io::Read for Uart<'_, Uart1<'_>> {
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        let mut n = 0;
        for b in buf.iter_mut() {
            if let Some(byte) = self.read_byte(1) {
                *b = byte;
                n += 1;
            } else {
                break;
            }
        }
        Ok(n)
    }
}

// UART2 embedded-io traits
impl embedded_io::ErrorType for Uart<'_, Uart2<'_>> {
    type Error = core::convert::Infallible;
}
impl embedded_io::Write for Uart<'_, Uart2<'_>> {
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        for &b in buf {
            self.write_byte(2, b);
        }
        Ok(buf.len())
    }
    fn flush(&mut self) -> Result<(), Self::Error> {
        self.flush_tx(2);
        Ok(())
    }
}
impl embedded_io::Read for Uart<'_, Uart2<'_>> {
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        let mut n = 0;
        for b in buf.iter_mut() {
            if let Some(byte) = self.read_byte(2) {
                *b = byte;
                n += 1;
            } else {
                break;
            }
        }
        Ok(n)
    }
}

// ── embedded-hal-nb serial traits ──────────────────────────────

macro_rules! impl_nb_serial {
    ($uart:ty, $idx:expr) => {
        impl embedded_hal_nb::serial::ErrorType for Uart<'_, $uart> {
            type Error = core::convert::Infallible;
        }
        impl embedded_hal_nb::serial::Read for Uart<'_, $uart> {
            fn read(&mut self) -> nb::Result<u8, Self::Error> {
                match self.read_byte($idx) {
                    Some(b) => Ok(b),
                    None => Err(nb::Error::WouldBlock),
                }
            }
        }
        impl embedded_hal_nb::serial::Write for Uart<'_, $uart> {
            fn write(&mut self, byte: u8) -> nb::Result<(), Self::Error> {
                self.write_byte($idx, byte);
                Ok(())
            }
            fn flush(&mut self) -> nb::Result<(), Self::Error> {
                if self.tx_flushed($idx) { Ok(()) } else { Err(nb::Error::WouldBlock) }
            }
        }
    };
}

impl_nb_serial!(Uart0<'_>, 0);
impl_nb_serial!(Uart1<'_>, 1);
impl_nb_serial!(Uart2<'_>, 2);

// ── Async UART (embedded-io-async) ──────────────────────────────────────────
#[cfg(feature = "async")]
mod asynch_impl {
    use super::{Uart, uart_regs};
    use crate::asynch::IrqSignal;
    use crate::peripherals::{Uart0, Uart1, Uart2};
    use core::cell::Cell;
    use core::future::Future;
    use core::pin::Pin;
    use core::task::{Context, Poll};
    use critical_section::Mutex;
    use embedded_io_async::{Read, Write};

    static UART_RX: [IrqSignal; 3] = [IrqSignal::new(), IrqSignal::new(), IrqSignal::new()];
    static UART_BYTE: [Mutex<Cell<u8>>; 3] = [const { Mutex::new(Cell::new(0)) }; 3];

    fn uart_base(idx: u8) -> usize {
        match idx {
            0 => 0x4401_0000,
            1 => 0x4401_1000,
            _ => 0x4401_2000,
        }
    }

    /// UART trap hook: read the received byte (which de-asserts the RX IRQ) and
    /// wake the awaiting reader. Call from the trap when `mcause` is UART0..2_INT
    /// (IRQ 53..55). Reading in the ISR avoids a level-triggered RX storm.
    pub fn on_interrupt(idx: u8) {
        let r = uart_regs(idx);
        if !r.fifo_status().read().rx_fifo_empty().bit_is_set() {
            let b = r.data().read().bits() as u8;
            critical_section::with(|cs| UART_BYTE[idx as usize].borrow(cs).set(b));
            UART_RX[idx as usize].signal();
        }
    }

    struct RxFuture {
        idx: u8,
    }
    impl Future for RxFuture {
        type Output = ();
        fn poll(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<()> {
            if UART_RX[self.idx as usize].take_fired() {
                Poll::Ready(())
            } else {
                UART_RX[self.idx as usize].register(cx.waker());
                Poll::Pending
            }
        }
    }

    async fn read_one(uart: &Uart<'_, impl Sized>, idx: u8) -> u8 {
        let r = uart_regs(idx);
        if !r.fifo_status().read().rx_fifo_empty().bit_is_set() {
            return r.data().read().bits() as u8; // byte already waiting
        }
        UART_RX[idx as usize].reset();
        // Enable the UART RX interrupt (INTR_EN @ +0x18) so a byte raises the IRQ.
        unsafe { core::ptr::write_volatile((uart_base(idx) + 0x18) as *mut u32, 1) };
        let _ = uart; // keep the &Uart borrow for the await's lifetime
        RxFuture { idx }.await;
        critical_section::with(|cs| UART_BYTE[idx as usize].borrow(cs).get())
    }

    macro_rules! async_uart {
        ($uart:ty, $idx:expr) => {
            // embedded_io::ErrorType is already implemented for the blocking impls;
            // the async Read/Write traits reuse it.
            impl Write for Uart<'_, $uart> {
                async fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
                    // WS63 UART TX drains immediately on this model; write_byte polls
                    // tx_fifo_full, so this completes without parking.
                    for &b in buf {
                        self.write_byte($idx, b);
                    }
                    Ok(buf.len())
                }
                async fn flush(&mut self) -> Result<(), Self::Error> {
                    self.flush_tx($idx);
                    Ok(())
                }
            }
            impl Read for Uart<'_, $uart> {
                async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
                    if buf.is_empty() {
                        return Ok(0);
                    }
                    buf[0] = read_one(self, $idx).await;
                    Ok(1)
                }
            }
        };
    }
    async_uart!(Uart0<'_>, 0);
    async_uart!(Uart1<'_>, 1);
    async_uart!(Uart2<'_>, 2);
}

#[cfg(feature = "async")]
pub use asynch_impl::on_interrupt;

// ── Tests ──────────────────────────────────────────────────────

#[cfg(all(test, not(target_arch = "riscv32")))]
mod tests {
    use super::*;
    use crate::soc::chip::UART_CLOCK_HZ;

    // Re-derivation of the pure baud-rate math that `configure_uart` inlines.
    // div = pclk / (16 * baud) carried as 6-bit fixed point:
    //   div64 = pclk * 4 / baud  (= div * 64), div = div64 >> 6, frac = div64 & 0x3F.
    // The 16-bit divider gives a minimum representable baud of pclk/(16*65535)+1 —
    // `BaudRate::try_new` rejects anything below it instead of the old silent clamp.
    fn min_baud() -> u32 {
        (UART_CLOCK_HZ / (16 * 65535)) + 1
    }
    /// Returns (div_l, div_h, div_fra) exactly as `configure_uart` would program them
    /// for a valid (already-validated) baud — no clamp, mirroring the tightened path.
    fn baud_regs(baud: u32) -> (u16, u16, u16) {
        let div64 = ((UART_CLOCK_HZ as u64) * 4 / (baud as u64)) as u32;
        let div = div64 >> 6;
        let div_fra = (div64 & 0x3F) as u16;
        let div_l = (div & 0xFF) as u16;
        let div_h = ((div >> 8) & 0xFF) as u16;
        (div_l, div_h, div_fra)
    }

    /// Default config matches the conventional 115200-8N1 settings.
    #[test]
    fn default_config_is_115200_8n1() {
        let c = Config::default();
        assert_eq!(c.baudrate, BaudRate::BAUD_115200);
        assert_eq!(c.baudrate.baud(), 115200);
        assert_eq!(c.data_bits, DataBits::Eight);
        assert_eq!(c.parity, Parity::None);
        assert_eq!(c.stop_bits, StopBits::One);
    }

    /// `BaudRate::try_new` rejects bauds whose 16-bit divider would over/underflow
    /// against the 160 MHz PLL base — the boundary that the old silent clamp hid.
    #[test]
    fn baud_rate_rejects_out_of_range() {
        // Too low: a sub-`min_baud` rate demands div > 0xFFFF → None.
        assert!(BaudRate::try_new(0).is_none());
        assert!(BaudRate::try_new(min_baud() - 1).is_none());
        // Zero baud is always rejected (no div-by-zero downstream).
        assert!(BaudRate::try_new(min_baud()).is_some());
        // Too high: div underflows below 1 (baud > pclk/16) → None.
        assert!(BaudRate::try_new(UART_CLOCK_HZ).is_none());
        assert!(BaudRate::try_new(UART_CLOCK_HZ / 16 + 1).is_none());
        // Common rates all construct.
        for &b in &[300u32, 9600, 115200, 921600, 1_000_000] {
            assert!(BaudRate::try_new(b).is_some(), "baud {b} should be valid");
            assert_eq!(BaudRate::try_new(b).unwrap().baud(), b);
        }
    }

    /// 115200 baud at 160 MHz: div = 160e6/(16*115200) ≈ 86.8 → div=86, frac≈51.
    /// div * 64 = 5546 → div=86 (0x56), frac=42 (5546 & 0x3F). Verify exact bytes.
    #[test]
    fn baud_115200_known_divisor() {
        // div64 = 160_000_000 * 4 / 115200 = 5555 (integer truncation).
        let div64 = (UART_CLOCK_HZ as u64) * 4 / 115200;
        assert_eq!(div64, 5555);
        let (div_l, div_h, div_fra) = baud_regs(115200);
        // div = 5555 >> 6 = 86 = 0x56, frac = 5555 & 0x3F = 51.
        assert_eq!(div_l, 0x56);
        assert_eq!(div_h, 0x00);
        assert_eq!(div_fra, 51);
    }

    /// The fractional field is always a 6-bit value (sixty-fourths), never out of range.
    #[test]
    fn frac_field_is_six_bits() {
        for &baud in &[300u32, 9600, 115200, 921600, 1_000_000, 10_000_000] {
            let (_, _, frac) = baud_regs(baud);
            assert!(frac <= 0x3F, "baud {baud} -> frac {frac} exceeds 6 bits");
        }
    }

    /// Each divider byte stays within 0..=0xFF (the 8-bit DIV_L/DIV_H registers).
    #[test]
    fn divider_bytes_fit_in_eight_bits() {
        for &baud in &[min_baud(), 1200, 115200, 3_000_000] {
            let (div_l, div_h, _) = baud_regs(baud);
            assert!(div_l <= 0xFF);
            assert!(div_h <= 0xFF);
        }
    }

    /// At the minimum valid baud the full divider (div64 >> 6) fits the 16-bit
    /// DIV_H:DIV_L pair — the boundary `BaudRate::try_new` enforces at construction.
    #[test]
    fn min_baud_divider_fits_sixteen_bits() {
        assert!(BaudRate::try_new(min_baud()).is_some());
        let div64 = ((UART_CLOCK_HZ as u64) * 4 / (min_baud() as u64)) as u32;
        let div = div64 >> 6;
        assert!(div <= 0xFFFF, "div {div} overflows 16-bit divider at min_baud");
    }

    /// Higher baud rates yield a smaller (or equal) integer divisor — the divider
    /// is monotonically non-increasing in baud (div = pclk*4/baud >> 6).
    #[test]
    fn divisor_monotonic_in_baud() {
        let div_of = |baud: u32| -> u32 {
            let (l, h, _) = baud_regs(baud);
            ((h as u32) << 8) | (l as u32)
        };
        let mut prev = u32::MAX;
        for &baud in &[1200u32, 9600, 19200, 57600, 115200, 230400, 460800] {
            let d = div_of(baud);
            assert!(d <= prev, "baud {baud} div {d} not <= prev {prev}");
            prev = d;
        }
    }

    // ── Control-register (UART_CTL) bit encoding ──────────────────
    // Mirrors the `ctl` assembly in `configure_uart` (lines 117-137):
    //   data bits at [3:2], parity-enable at bit5 / even-select at bit4, 2 stop at bit7.

    fn data_bits_field(d: DataBits) -> u16 {
        match d {
            DataBits::Five => 0,
            DataBits::Six => 1 << 2,
            DataBits::Seven => 2 << 2,
            DataBits::Eight => 3 << 2,
        }
    }
    fn parity_field(p: Parity) -> u16 {
        match p {
            Parity::Even => (1 << 5) | (1 << 4),
            Parity::Odd => 1 << 5,
            Parity::None => 0,
        }
    }
    fn build_ctl(c: &Config) -> u16 {
        let mut ctl = data_bits_field(c.data_bits) | parity_field(c.parity);
        if matches!(c.stop_bits, StopBits::Two) {
            ctl |= 1 << 7;
        }
        ctl
    }

    /// Data-bit widths map to consecutive codes 0..=3 in the [3:2] field.
    #[test]
    fn data_bits_encoding() {
        assert_eq!(data_bits_field(DataBits::Five), 0 << 2);
        assert_eq!(data_bits_field(DataBits::Six), 1 << 2);
        assert_eq!(data_bits_field(DataBits::Seven), 2 << 2);
        assert_eq!(data_bits_field(DataBits::Eight), 3 << 2);
    }

    /// Parity: None clears both bits; Odd sets only the enable bit (5); Even sets
    /// enable (5) AND the even-select bit (4).
    #[test]
    fn parity_encoding() {
        assert_eq!(parity_field(Parity::None), 0);
        assert_eq!(parity_field(Parity::Odd), 1 << 5);
        assert_eq!(parity_field(Parity::Even), (1 << 5) | (1 << 4));
        // Odd and Even both enable parity (bit 5 set); None does not.
        assert_ne!(parity_field(Parity::Odd) & (1 << 5), 0);
        assert_ne!(parity_field(Parity::Even) & (1 << 5), 0);
        assert_eq!(parity_field(Parity::None) & (1 << 5), 0);
    }

    /// Two stop bits sets bit 7; one stop bit leaves it clear.
    #[test]
    fn stop_bits_encoding() {
        let mut c = Config::default();
        c.stop_bits = StopBits::One;
        assert_eq!(build_ctl(&c) & (1 << 7), 0);
        c.stop_bits = StopBits::Two;
        assert_eq!(build_ctl(&c) & (1 << 7), 1 << 7);
    }

    /// 8N1 (the default frame) encodes to exactly the 8-data-bits field with no
    /// parity and one stop bit: 0b1100 = 0x0C.
    #[test]
    fn ctl_8n1_known_value() {
        assert_eq!(build_ctl(&Config::default()), 0x0C);
    }

    /// 7E2 (7 data, even parity, 2 stop) sets distinct, non-overlapping fields.
    #[test]
    fn ctl_7e2_known_value() {
        let c = Config {
            baudrate: BaudRate::try_new(9600).unwrap(),
            data_bits: DataBits::Seven,
            parity: Parity::Even,
            stop_bits: StopBits::Two,
            clock_hz: None,
        };
        // data=2<<2 (0x08) | parity even (0x30) | 2-stop (0x80) = 0xB8.
        assert_eq!(build_ctl(&c), (2 << 2) | (1 << 5) | (1 << 4) | (1 << 7));
        assert_eq!(build_ctl(&c), 0xB8);
    }
}

// ── Property-based fuzz tests ──────────────────────────────────

#[cfg(all(test, not(target_arch = "riscv32")))]
mod proptests {
    use super::BaudRate;
    use crate::soc::chip::UART_CLOCK_HZ;
    use proptest::prelude::*;

    /// The register triple `configure_uart` programs for a (valid) baud — no clamp,
    /// mirroring the tightened path where `BaudRate` already guaranteed the range.
    fn baud_regs(baud: u32) -> (u16, u16, u16) {
        let div64 = ((UART_CLOCK_HZ as u64) * 4 / (baud as u64)) as u32;
        let div = div64 >> 6;
        let div_fra = (div64 & 0x3F) as u16;
        let div_l = (div & 0xFF) as u16;
        let div_h = ((div >> 8) & 0xFF) as u16;
        (div_l, div_h, div_fra)
    }

    proptest! {
        /// Fuzz: `BaudRate::try_new` never panics for any u32 baud (incl. 0) — it
        /// returns `None` out of range instead of clamping or dividing by zero.
        #[test]
        fn try_new_never_panics(baud in any::<u32>()) {
            let _ = BaudRate::try_new(baud);
        }

        /// Fuzz: every baud that `BaudRate` accepts programs an in-range register
        /// triple — 6-bit fraction, 8-bit DIV_L/DIV_H, 16-bit assembled divider.
        /// (Invalid bauds are rejected at construction, so this is the only path
        /// that ever reaches `configure_uart`.)
        #[test]
        fn accepted_baud_fits_registers(baud in any::<u32>()) {
            if let Some(br) = BaudRate::try_new(baud) {
                let (div_l, div_h, frac) = baud_regs(br.baud());
                prop_assert!(frac <= 0x3F);
                prop_assert!(div_l <= 0xFF);
                prop_assert!(div_h <= 0xFF);
                let div = ((div_h as u32) << 8) | (div_l as u32);
                prop_assert!(div <= 0xFFFF, "baud={} div={}", baud, div);
            }
        }
    }
}
