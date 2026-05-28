//! UART driver for WS63 (UART0/1/2, 16C550-compatible with FIFO).
//!
//! Baud rate: div = (div_h << 8 | div_l) + div_fra / 64.
//! Clock source: 240MHz system clock.

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

#[derive(Debug, Clone, Copy)]
pub struct Config {
    pub baudrate: u32,
    pub data_bits: DataBits,
    pub parity: Parity,
    pub stop_bits: StopBits,
}

impl Default for Config {
    fn default() -> Self {
        Self { baudrate: 115200, data_bits: DataBits::Eight, parity: Parity::None, stop_bits: StopBits::One }
    }
}

pub struct Uart<'d, T> {
    _peripheral: PhantomData<&'d T>,
}

#[allow(dead_code)]
fn regs() -> &'static ws63_pac::uart0::RegisterBlock {
    unsafe { &*Uart0::ptr() }
}

fn uart_ptr(idx: u8) -> *const ws63_pac::uart0::RegisterBlock {
    match idx {
        0 => Uart0::ptr(),
        1 => Uart1::ptr(),
        2 => Uart2::ptr(),
        _ => unreachable!(),
    }
}

fn uart_regs(idx: u8) -> &'static ws63_pac::uart0::RegisterBlock {
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

    // Set baud rate: div = PCLK / (16 * baudrate)
    let pclk = crate::soc::ws63::SYSTEM_CLOCK_HZ;
    let div = pclk / (16 * config.baudrate);
    let div_l = (div & 0xFF) as u16;
    let div_h = ((div >> 8) & 0xFF) as u16;
    r.div_l().write(|w| unsafe { w.bits(div_l) });
    r.div_h().write(|w| unsafe { w.bits(div_h) });
    r.div_fra().write(|w| unsafe { w.bits(0) });

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

    pub fn write(&self, idx: u8, data: &[u8]) {
        for &b in data {
            self.write_byte(idx, b);
        }
    }

    pub fn uart_regs(&self, idx: u8) -> &'static ws63_pac::uart0::RegisterBlock {
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
