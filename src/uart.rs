//! UART driver for WS63 (UART0/1/2, 16C550-compatible with FIFO).
//!
//! Baud rate: div = (div_h << 8 | div_l) + div_fra / 64.
//! Clock source: by default the 160 MHz PLL-derived UART clock
//! ([`crate::soc::chip::UART_CLOCK_HZ`]), NOT the 240 MHz CPU clock (vendor
//! `clock_init` sets the baud base to 160 MHz). Examples that skip `clock_init`
//! run on flashboot's raw-TCXO console clock (24/40 MHz, confirmed 40 MHz on
//! silicon) — they use [`UartClock::Boot`] so the divider matches the real base
//! (issue #15/#10).

use crate::peripherals::{Uart0, Uart1, Uart2};
use core::marker::PhantomData;

mod sealed {
    pub trait UartInstanceSealed {}
}

/// UART port identity for the three UART instances.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum UartPort {
    /// UART0.
    Uart0,
    /// UART1.
    Uart1,
    /// UART2.
    Uart2,
}

impl UartPort {
    /// Build a UART port from a raw index, rejecting values outside 0..=2.
    pub const fn from_index(index: u8) -> Option<Self> {
        match index {
            0 => Some(Self::Uart0),
            1 => Some(Self::Uart1),
            2 => Some(Self::Uart2),
            _ => None,
        }
    }

    /// The UART port index (0-2).
    pub const fn index(self) -> usize {
        match self {
            Self::Uart0 => 0,
            Self::Uart1 => 1,
            Self::Uart2 => 2,
        }
    }
}

/// Sealed marker implemented by the HAL's UART peripheral token types.
pub trait UartInstance: sealed::UartInstanceSealed {
    /// The concrete UART port for this instance.
    const PORT: UartPort;
}

impl<'d> sealed::UartInstanceSealed for Uart0<'d> {}
impl<'d> sealed::UartInstanceSealed for Uart1<'d> {}
impl<'d> sealed::UartInstanceSealed for Uart2<'d> {}

impl<'d> UartInstance for Uart0<'d> {
    const PORT: UartPort = UartPort::Uart0;
}

impl<'d> UartInstance for Uart1<'d> {
    const PORT: UartPort = UartPort::Uart1;
}

impl<'d> UartInstance for Uart2<'d> {
    const PORT: UartPort = UartPort::Uart2;
}

/// Number of data bits per UART frame ([3:2] field of UART_CTL).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DataBits {
    /// 5 data bits (field code 0).
    Five,
    /// 6 data bits (field code 1).
    Six,
    /// 7 data bits (field code 2).
    Seven,
    /// 8 data bits (field code 3).
    Eight,
}

/// UART parity mode (parity-enable bit 5 / even-select bit 4 of UART_CTL).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Parity {
    /// No parity bit (parity disabled).
    None,
    /// Even parity (enable + even-select set).
    Even,
    /// Odd parity (enable set, even-select clear).
    Odd,
}

/// Number of stop bits per UART frame (2-stop = bit 7 of UART_CTL).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StopBits {
    /// One stop bit (bit 7 clear).
    One,
    /// Two stop bits (bit 7 set).
    Two,
}

/// A validated UART baud rate. The 16-bit integer divider is
/// `div = UART_CLOCK_HZ / (16 · baud)`, so against the default 160 MHz PLL base the
/// realisable baud is `[~153, 10 000 000]`. `try_new` rejects anything outside that
/// instead of the old silent low-clamp (`baud < min_baud → min_baud`). Note: a
/// [`UartClock::Boot`] (the 24/40 MHz flashboot console clock) shifts the realisable
/// range down — keep boot-console firmware at ordinary baud rates such as 115200.
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

/// UART baud-base clock selection.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum UartClock {
    /// Normal post-clock-init PLL-derived UART clock.
    Pll,
    /// Boot-console clock inherited from flashboot before normal clock init.
    Boot,
}

impl UartClock {
    fn hz(self) -> u32 {
        match self {
            Self::Pll => crate::soc::chip::UART_CLOCK_HZ,
            Self::Boot => boot_uart_clock_hz(),
        }
    }
}

#[cfg(feature = "chip-ws63")]
fn boot_uart_clock_hz() -> u32 {
    crate::soc::chip::uart_boot_clock_hz()
}

#[cfg(not(feature = "chip-ws63"))]
fn boot_uart_clock_hz() -> u32 {
    crate::soc::chip::UART_CLOCK_HZ
}

/// UART frame and clock configuration passed to the `new_uartN` constructors.
#[derive(Debug, Clone, Copy)]
pub struct Config {
    /// Validated baud rate.
    pub baudrate: BaudRate,
    /// Number of data bits per frame.
    pub data_bits: DataBits,
    /// Parity mode.
    pub parity: Parity,
    /// Number of stop bits per frame.
    pub stop_bits: StopBits,
    /// UART baud-base clock source.
    pub clock: UartClock,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            baudrate: BaudRate::BAUD_115200,
            data_bits: DataBits::Eight,
            parity: Parity::None,
            stop_bits: StopBits::One,
            clock: UartClock::Pll,
        }
    }
}

/// UART driver instance, generic over the UART peripheral type `T` (Uart0/1/2).
pub struct Uart<'d, T> {
    _peripheral: PhantomData<&'d T>,
}

#[allow(dead_code)]
fn regs() -> &'static crate::soc::pac::uart0::RegisterBlock {
    // SAFETY: PAC peripheral pointer is a static physical MMIO address, always valid
    unsafe { &*Uart0::ptr() }
}

fn uart_ptr(port: UartPort) -> *const crate::soc::pac::uart0::RegisterBlock {
    match port {
        UartPort::Uart0 => Uart0::ptr(),
        UartPort::Uart1 => Uart1::ptr(),
        UartPort::Uart2 => Uart2::ptr(),
    }
}

fn uart_regs(port: UartPort) -> &'static crate::soc::pac::uart0::RegisterBlock {
    // SAFETY: uart_ptr(port) returns valid PAC MMIO addresses (UART0/1/2 at 0x4401_0000/1000/2000)
    unsafe { &*uart_ptr(port) }
}

#[allow(dead_code)]
fn write_fifo_blocking_levels(r: &crate::soc::pac::uart0::RegisterBlock) {
    r.fifo_ctl().write(|w| w.fifo_en().set_bit().tx_empty_trig().empty().rx_empty_trig().char1());
}

fn write_fifo_dma_levels(r: &crate::soc::pac::uart0::RegisterBlock) {
    r.fifo_ctl().write(|w| w.fifo_en().set_bit().tx_empty_trig().chars2().rx_empty_trig().quarter());
}

impl<'d> Uart<'d, Uart0<'d>> {
    /// Create and configure a UART0 driver from the peripheral token and config.
    pub fn new_uart0(_uart: Uart0<'d>, config: Config) -> Self {
        configure_uart(UartPort::Uart0, &config);
        Self { _peripheral: PhantomData }
    }
}

impl<'d> Uart<'d, Uart1<'d>> {
    /// Create and configure a UART1 driver from the peripheral token and config.
    pub fn new_uart1(_uart: Uart1<'d>, config: Config) -> Self {
        configure_uart(UartPort::Uart1, &config);
        Self { _peripheral: PhantomData }
    }
}

impl<'d> Uart<'d, Uart2<'d>> {
    /// Create and configure a UART2 driver from the peripheral token and config.
    #[instability::unstable]
    pub fn new_uart2(_uart: Uart2<'d>, config: Config) -> Self {
        configure_uart(UartPort::Uart2, &config);
        Self { _peripheral: PhantomData }
    }
}

fn configure_uart(port: UartPort, config: &Config) {
    let r = uart_regs(port);

    // Enable divisor-latch access only while programming the baud divider.
    r.uart_ctl().write(|w| w.div_en().set_bit());

    // Set baud rate: div = UART_CLK / (16 * baudrate)
    // Valid range: div ∈ [1, 65535] (16-bit divider). `BaudRate::try_new` already
    // rejected any baud whose divider would fall outside that against the 160 MHz
    // PLL base, so no runtime clamp is needed (and `baud()` is never 0 → no div0).
    // `UartClock::Boot` selects the pre-`clock_init` flashboot console clock; the
    // default `Pll` keeps the normal 160 MHz PLL base.
    let pclk = config.clock.hz();
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
    r.uart_ctl().write(|w| unsafe { w.bits(ctl) }); // div_en=0: DATA maps back to TX/RX.

    // Reset and enable FIFO with the vendor default trigger levels:
    // TX interrupt threshold = 2 chars, RX threshold = 1/4 full.
    r.fifo_ctl().write(|w| w.fifo_en().set_bit().tx_fifo_rst().set_bit().rx_fifo_rst().set_bit());
    write_fifo_dma_levels(r);
}

impl<T: UartInstance> Uart<'_, T> {
    /// Write one byte, blocking while this UART's TX FIFO is full.
    pub fn write_byte(&self, byte: u8) {
        let r = uart_regs(T::PORT);
        while r.fifo_status().read().tx_fifo_full().bit_is_set() {}
        r.data().write(|w| unsafe { w.bits(byte as u16) });
    }

    /// Read one byte, or `None` if this UART's RX FIFO is empty.
    pub fn read_byte(&self) -> Option<u8> {
        let r = uart_regs(T::PORT);
        // Gate on the RX FIFO *count* (0x4c), not the `fifo_status.rx_fifo_empty`
        // bit. The vendor `hal_uart_v151` polls `rx_fifo_cnt` to decide whether to
        // read `data` (0x04), and on silicon the `rx_fifo_empty` status bit does not
        // track a single-byte pop (gating on it loops forever / re-reads a stale
        // byte — the cause of the long-broken `uart1_loopback`). `rx_fifo_cnt`
        // decrements correctly as each `data` read drains the FIFO.
        if r.rx_fifo_cnt().read().bits() == 0 { None } else { Some(r.data().read().bits() as u8) }
    }

    /// Block until this UART's TX FIFO is fully drained.
    pub fn flush_tx(&self) {
        let r = uart_regs(T::PORT);
        while !r.fifo_status().read().tx_fifo_empty().bit_is_set() {}
    }

    /// Non-blocking check: returns true if TX FIFO is fully drained.
    pub fn tx_flushed(&self) -> bool {
        uart_regs(T::PORT).fifo_status().read().tx_fifo_empty().bit_is_set()
    }

    /// Write a byte slice, blocking per byte.
    pub fn write(&self, data: &[u8]) {
        for &b in data {
            self.write_byte(b);
        }
    }

    /// Return this UART's PAC register block.
    ///
    /// # Safety
    /// This bypasses the typed UART driver. The caller must uphold all PAC
    /// aliasing, ordering, and peripheral-state invariants.
    #[instability::unstable]
    pub unsafe fn register_block(&self) -> &'static crate::soc::pac::uart0::RegisterBlock {
        uart_regs(T::PORT)
    }
}

#[cfg(feature = "chip-ws63")]
impl<'d> Uart<'d, Uart0<'d>> {
    /// Consume the blocking UART0 + a DMA driver → DMA-capable [`UartDma`] (idx 0).
    #[instability::unstable]
    pub fn with_dma(self, dma: crate::dma::DmaDriver<'d, crate::dma::Dma0>) -> UartDma<'d, Uart0<'d>> {
        UartDma { port: UartPort::Uart0, dma, _p: PhantomData }
    }
}
#[cfg(feature = "chip-ws63")]
impl<'d> Uart<'d, Uart1<'d>> {
    /// Consume the blocking UART1 + a DMA driver → DMA-capable [`UartDma`] (idx 1).
    #[instability::unstable]
    pub fn with_dma(self, dma: crate::dma::DmaDriver<'d, crate::dma::Dma0>) -> UartDma<'d, Uart1<'d>> {
        UartDma { port: UartPort::Uart1, dma, _p: PhantomData }
    }
}
#[cfg(feature = "chip-ws63")]
impl<'d> Uart<'d, Uart2<'d>> {
    /// Consume the blocking UART2 + a DMA driver → DMA-capable [`UartDma`] (idx 2).
    #[instability::unstable]
    pub fn with_dma(self, dma: crate::dma::DmaDriver<'d, crate::dma::Dma0>) -> UartDma<'d, Uart2<'d>> {
        UartDma { port: UartPort::Uart2, dma, _p: PhantomData }
    }
}

/// A DMA-capable UART. Built from [`Uart::with_dma`]; owns the [`DmaDriver`] and
/// remembers the instance index so it can pick the correct handshaking ID and
/// register block (binding the `DmaPeripheral` to the type, compile-time, rather
/// than re-deriving from a runtime idx).
///
/// **UNSTABLE** (behind the `unstable` feature): no UartDma HIL test exists — the
/// `write_dma` (TX) silicon attempt timed out (the UART1 TX shift register never
/// advances on this board, `hisi-riscv-hal#5`), and `read_dma` (RX, fixed-length)
/// loopback is blocked by the same #5 board issue. The register sequence compiles
/// and is correct, but round-trip verification is deferred to a #5-fixed board.
/// The `uart_parameter.dma_mode` field is PAC-read-only, so DMA pacing is driven
/// by the FIFO_CTL trigger levels (the design's P3 silicon question — answered:
/// it works with triggers alone).
#[cfg(feature = "chip-ws63")]
#[instability::unstable]
pub struct UartDma<'d, T> {
    port: UartPort,
    dma: crate::dma::DmaDriver<'d, crate::dma::Dma0>,
    _p: PhantomData<&'d T>,
}

#[cfg(feature = "chip-ws63")]
impl<'d, T> UartDma<'d, T> {
    fn regs(&self) -> &'static crate::soc::pac::uart0::RegisterBlock {
        uart_regs(self.port)
    }

    fn data_addr(&self) -> u32 {
        self.regs().data() as *const _ as u32
    }

    fn tx_peri(&self) -> crate::dma::DmaPeripheral {
        match self.port {
            UartPort::Uart0 => crate::dma::DmaPeripheral::Uart0Tx,
            UartPort::Uart1 => crate::dma::DmaPeripheral::Uart1Tx,
            UartPort::Uart2 => crate::dma::DmaPeripheral::Uart2Tx,
        }
    }

    fn rx_peri(&self) -> crate::dma::DmaPeripheral {
        match self.port {
            UartPort::Uart0 => crate::dma::DmaPeripheral::Uart0Rx,
            UartPort::Uart1 => crate::dma::DmaPeripheral::Uart1Rx,
            UartPort::Uart2 => crate::dma::DmaPeripheral::Uart2Rx,
        }
    }

    /// Write `buf` to the UART TX FIFO via DMA (mem→peripheral). `buf` is owned for
    /// the whole call and returned on success. `ch` is a claimed
    /// [`DmaChannel`](crate::dma::DmaChannel) token.
    #[instability::unstable]
    pub fn write_dma<B: embedded_dma::ReadBuffer<Word = u8>>(
        &mut self,
        ch: crate::dma::DmaChannel,
        buf: B,
    ) -> Result<B, UartDmaError> {
        use crate::dma::{DmaChannelConfig, DmaFrame, DmaTransferSize};
        let r = self.regs();
        let (ptr, beats) = unsafe { buf.read_buffer() };
        if beats > 0xFFF {
            return Err(UartDmaError::BufferTooLong);
        }
        let size = DmaTransferSize::from_beats(beats).ok_or(UartDmaError::BufferTooLong)?;
        let bytes = beats;
        let data_addr = self.data_addr();

        // FIFO_CTL trigger levels (vendor defaults hal_uart_v151.c:127/133):
        // TX = 2-chars-empty, RX = 1/4. Written as the LAST FIFO_CTL write (the
        // register is WO; a later FIFO reset would clobber pacing). fifo_en stays set.
        write_fifo_dma_levels(r);

        // Clean the TX source (non-coherent core). The DATA register is uncached MMIO.
        unsafe { crate::cache::clean_range(ptr as usize, bytes) };

        let cfg = DmaChannelConfig::default().mem_to_peripheral(self.tx_peri()).with_width(DmaFrame::Byte);
        let chn = ch.logical();
        self.dma.configure_channel_raw(chn, ptr as u32, data_addr, size, &cfg);
        // (uart_parameter.dma_mode is PAC-read-only — DMA paces on the FIFO triggers above.)

        // Bounded wait for the channel to auto-clear its enable bit.
        let mut n = 1_000_000u32;
        while self.dma.channel_enabled_raw(chn) {
            n -= 1;
            if n == 0 {
                write_fifo_blocking_levels(r);
                self.dma.halt_channel_raw(chn);
                self.dma.disable_channel_raw(chn);
                return Err(UartDmaError::Timeout);
            }
            core::hint::spin_loop();
        }
        // Restore the blocking trigger levels (DMA no longer pacing).
        write_fifo_blocking_levels(r);
        Ok(buf)
    }

    /// Fixed-length RX DMA (peripheral→memory). Reads exactly `buf.len()` bytes — a
    /// short/idle line will **time out** (no char-timeout path here). `buf` is owned
    /// for the whole call and returned on success. **Loopback data-correctness is
    /// blocked by hisi-riscv-hal#5** on this board; the register sequence compiles
    /// and is correct but cannot be round-trip-verified here.
    #[instability::unstable]
    pub fn read_dma<B: embedded_dma::WriteBuffer<Word = u8>>(
        &mut self,
        ch: crate::dma::DmaChannel,
        mut buf: B,
    ) -> Result<B, UartDmaError> {
        use crate::dma::{DmaChannelConfig, DmaFrame, DmaTransferSize};
        let r = self.regs();
        let (ptr, beats) = unsafe { buf.write_buffer() };
        if beats > 0xFFF {
            return Err(UartDmaError::BufferTooLong);
        }
        let size = DmaTransferSize::from_beats(beats).ok_or(UartDmaError::BufferTooLong)?;
        let bytes = beats;
        let data_addr = self.data_addr();

        write_fifo_dma_levels(r);

        let cfg = DmaChannelConfig::default().peripheral_to_mem(self.rx_peri()).with_width(DmaFrame::Byte);
        let chn = ch.logical();
        self.dma.configure_channel_raw(chn, data_addr, ptr as u32, size, &cfg);

        let mut n = 1_000_000u32;
        while self.dma.channel_enabled_raw(chn) {
            n -= 1;
            if n == 0 {
                write_fifo_blocking_levels(r);
                self.dma.halt_channel_raw(chn);
                self.dma.disable_channel_raw(chn);
                return Err(UartDmaError::Timeout);
            }
            core::hint::spin_loop();
        }
        // Invalidate the RX destination so the CPU re-reads what DMA wrote.
        unsafe { crate::cache::invalidate_range(ptr as usize, bytes) };
        write_fifo_blocking_levels(r);
        Ok(buf)
    }

    /// Reclaim the blocking `Uart` and the `DmaDriver`. Restores the blocking
    /// FIFO_CTL trigger levels.
    #[instability::unstable]
    pub fn release(self) -> (Uart<'d, T>, crate::dma::DmaDriver<'d, crate::dma::Dma0>) {
        let r = self.regs();
        write_fifo_blocking_levels(r);
        (Uart { _peripheral: PhantomData }, self.dma)
    }
}

/// Errors from a UART DMA transfer.
#[cfg(feature = "chip-ws63")]
#[instability::unstable]
#[derive(Debug)]
#[non_exhaustive]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum UartDmaError {
    /// The buffer exceeded 4095 beats (the 12-bit `trans_size` cap).
    BufferTooLong,
    /// The channel never completed within the bounded poll (e.g. a short/idle RX line).
    Timeout,
}

impl embedded_io::ErrorType for Uart<'_, Uart0<'_>> {
    type Error = core::convert::Infallible;
}
impl embedded_io::Write for Uart<'_, Uart0<'_>> {
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        for &b in buf {
            self.write_byte(b);
        }
        Ok(buf.len())
    }
    fn flush(&mut self) -> Result<(), Self::Error> {
        self.flush_tx();
        Ok(())
    }
}
impl embedded_io::Read for Uart<'_, Uart0<'_>> {
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        let mut n = 0;
        for b in buf.iter_mut() {
            if let Some(byte) = self.read_byte() {
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
            self.write_byte(b);
        }
        Ok(buf.len())
    }
    fn flush(&mut self) -> Result<(), Self::Error> {
        self.flush_tx();
        Ok(())
    }
}
impl embedded_io::Read for Uart<'_, Uart1<'_>> {
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        let mut n = 0;
        for b in buf.iter_mut() {
            if let Some(byte) = self.read_byte() {
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
            self.write_byte(b);
        }
        Ok(buf.len())
    }
    fn flush(&mut self) -> Result<(), Self::Error> {
        self.flush_tx();
        Ok(())
    }
}
impl embedded_io::Read for Uart<'_, Uart2<'_>> {
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        let mut n = 0;
        for b in buf.iter_mut() {
            if let Some(byte) = self.read_byte() {
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
    ($uart:ty) => {
        impl embedded_hal_nb::serial::ErrorType for Uart<'_, $uart> {
            type Error = core::convert::Infallible;
        }
        impl embedded_hal_nb::serial::Read for Uart<'_, $uart> {
            fn read(&mut self) -> nb::Result<u8, Self::Error> {
                match self.read_byte() {
                    Some(b) => Ok(b),
                    None => Err(nb::Error::WouldBlock),
                }
            }
        }
        impl embedded_hal_nb::serial::Write for Uart<'_, $uart> {
            fn write(&mut self, byte: u8) -> nb::Result<(), Self::Error> {
                self.write_byte(byte);
                Ok(())
            }
            fn flush(&mut self) -> nb::Result<(), Self::Error> {
                if self.tx_flushed() { Ok(()) } else { Err(nb::Error::WouldBlock) }
            }
        }
    };
}

impl_nb_serial!(Uart0<'_>);
impl_nb_serial!(Uart1<'_>);
impl_nb_serial!(Uart2<'_>);

// ── Async UART (embedded-io-async) ──────────────────────────────────────────
#[cfg(all(feature = "chip-ws63", feature = "async", feature = "unstable"))]
mod asynch_impl {
    use super::{Uart, UartPort, uart_regs};
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

    /// UART trap hook: read the received byte (which de-asserts the RX IRQ) and
    /// wake the awaiting reader. Call from the trap when `mcause` is UART0..2_INT
    /// (IRQ 53..55). Reading in the ISR avoids a level-triggered RX storm.
    pub fn on_interrupt(port: UartPort) {
        let idx = port.index();
        let r = uart_regs(port);
        // Gate on rx_fifo_cnt, not rx_fifo_empty (the status bit does not track a
        // single-byte pop on silicon — same fix as the blocking `read_byte`).
        if r.rx_fifo_cnt().read().bits() != 0 {
            let b = r.data().read().bits() as u8;
            critical_section::with(|cs| UART_BYTE[idx].borrow(cs).set(b));
            UART_RX[idx].signal();
        }
    }

    // Named device.x handlers (UART0/1/2_INT = IRQ 53/54/55): the rt routes the
    // UART IRQ here by number, so an async UART app needs no `mcause` trap.
    #[unsafe(no_mangle)]
    extern "C" fn UART0_INT() {
        on_interrupt(UartPort::Uart0);
    }
    #[unsafe(no_mangle)]
    extern "C" fn UART1_INT() {
        on_interrupt(UartPort::Uart1);
    }
    #[unsafe(no_mangle)]
    extern "C" fn UART2_INT() {
        on_interrupt(UartPort::Uart2);
    }

    struct RxFuture {
        port: UartPort,
    }
    impl Future for RxFuture {
        type Output = ();
        fn poll(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<()> {
            let idx = self.port.index();
            if UART_RX[idx].take_fired() {
                Poll::Ready(())
            } else {
                UART_RX[idx].register(cx.waker());
                Poll::Pending
            }
        }
    }

    async fn read_one(uart: &Uart<'_, impl Sized>, port: UartPort) -> u8 {
        let idx = port.index();
        let r = uart_regs(port);
        if !r.fifo_status().read().rx_fifo_empty().bit_is_set() {
            return r.data().read().bits() as u8; // byte already waiting
        }
        UART_RX[idx].reset();
        // Enable the UART RX-data interrupt so a byte raises the IRQ.
        r.intr_en().modify(|_, w| w.rece_data_intr_en().set_bit());
        let _ = uart; // keep the &Uart borrow for the await's lifetime
        RxFuture { port }.await;
        critical_section::with(|cs| UART_BYTE[idx].borrow(cs).get())
    }

    macro_rules! async_uart {
        ($uart:ty, $port:expr) => {
            // embedded_io::ErrorType is already implemented for the blocking impls;
            // the async Read/Write traits reuse it.
            impl Write for Uart<'_, $uart> {
                async fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
                    // WS63 UART TX drains immediately on this model; write_byte polls
                    // tx_fifo_full, so this completes without parking.
                    for &b in buf {
                        self.write_byte(b);
                    }
                    Ok(buf.len())
                }
                async fn flush(&mut self) -> Result<(), Self::Error> {
                    self.flush_tx();
                    Ok(())
                }
            }
            impl Read for Uart<'_, $uart> {
                async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
                    if buf.is_empty() {
                        return Ok(0);
                    }
                    buf[0] = read_one(self, $port).await;
                    Ok(1)
                }
            }
        };
    }
    async_uart!(Uart0<'_>, UartPort::Uart0);
    async_uart!(Uart1<'_>, UartPort::Uart1);
    async_uart!(Uart2<'_>, UartPort::Uart2);
}

#[cfg(all(feature = "chip-ws63", feature = "async", feature = "unstable"))]
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
            clock: UartClock::Pll,
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
