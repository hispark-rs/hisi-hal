//! SPI master driver for WS63 (SPI0/1, SSI v151).
//! DesignWare SSI: SCK = SSI_CLK / SCKDV. Two-stage clock: a CLDO_CRG divider sets
//! SSI_CLK = 160MHz off the 480MHz PLL (`configure_spi_source_clock`, mirrors the
//! vendor `spi_porting_clock_init`), then SCKDV divides to SCK. SSI_CLK =
//! [`crate::soc::chip::SPI_CLOCK_HZ`] (NOT the 240MHz CPU clock; SCKDV is even, >= 2).

use crate::peripherals::{Spi0, Spi1};
use core::marker::PhantomData;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SpiMode {
    Mode0,
    Mode1,
    Mode2,
    Mode3,
}

/// A validated SPI bus clock. The DesignWare SSI divides `SPI_CLOCK_HZ` (160 MHz)
/// by an even SCKDV in `[2, 0xFFFE]`, so the achievable SCK is
/// `[SPI_CLOCK_HZ/0xFFFE ≈ 2.4 kHz, SPI_CLOCK_HZ/2 = 80 MHz]`. `try_from_hz` rejects
/// anything outside that band instead of silently clamping the divider (the old
/// `frequency: u32` path clamped to 2× / ½× the requested SCK without telling you).
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct SpiHz(u32);

impl SpiHz {
    /// 1 MHz — the default SPI bus clock.
    pub const ONE_MHZ: SpiHz = SpiHz(1_000_000);

    /// Construct from a target SCK frequency. `None` if the resulting SCKDV
    /// divider would fall outside `[2, 0xFFFE]` (frequency too high or too low).
    pub const fn try_from_hz(hz: u32) -> Option<Self> {
        if hz == 0 {
            return None;
        }
        let div = crate::soc::chip::SPI_CLOCK_HZ / hz;
        if div < 2 || div > 0xFFFE {
            return None;
        }
        Some(SpiHz(hz))
    }

    /// The requested SCK frequency in Hz.
    pub const fn hz(self) -> u32 {
        self.0
    }

    /// The even SCKDV divider for this frequency (always in `[2, 0xFFFE]`).
    const fn to_sckdv(self) -> u32 {
        sckdv(crate::soc::chip::SPI_CLOCK_HZ, self.0)
    }
}

/// SPI data frame size in bits, validated to the SSI DFS range `4..=16`.
///
/// (The 4-bit DFS field caps the documented range at 16; whether this silicon also
/// supports the 32-bit DFS extension is an on-board open question — `4..=16` is the
/// conservative, always-runnable choice. A value outside it is unrepresentable
/// rather than silently masked into the register.)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct DataBits(u8);

impl DataBits {
    /// 8-bit frames (the common default).
    pub const EIGHT: DataBits = DataBits(8);
    /// 16-bit frames.
    pub const SIXTEEN: DataBits = DataBits(16);

    /// Construct from a frame size. `None` outside `4..=16`.
    pub const fn new(bits: u8) -> Option<Self> {
        if bits >= 4 && bits <= 16 { Some(DataBits(bits)) } else { None }
    }

    /// The frame size in bits (always `4..=16`).
    pub const fn bits(self) -> u8 {
        self.0
    }
}

#[derive(Debug, Clone, Copy)]
pub struct Config {
    pub frequency: SpiHz,
    pub mode: SpiMode,
    pub data_bits: DataBits,
}

impl Default for Config {
    fn default() -> Self {
        Self { frequency: SpiHz::ONE_MHZ, mode: SpiMode::Mode0, data_bits: DataBits::EIGHT }
    }
}

pub struct Spi<'d, T> {
    idx: u8,
    _peripheral: PhantomData<&'d T>,
}

fn spi_regs(idx: u8) -> &'static crate::soc::pac::spi0::RegisterBlock {
    unsafe {
        match idx {
            0 => &*Spi0::ptr(),
            1 => &*Spi1::ptr(),
            _ => unreachable!(),
        }
    }
}

/// SCKDV clock divider for the DesignWare SSI: `SCK = SSI_CLK / SCKDV`.
///
/// Matches the WS63 C SDK (`hal_spi_v151.c`: `clk_div = bus_clk / freq`,
/// clamped to `SPI_MINUMUM_CLK_DIV` = 2). SCKDV bit 0 is read-only 0, so the
/// value must be even. There is NO `/2` and NO `-1` (an earlier version of this
/// driver had both, producing ~2x the requested SCK).
const fn sckdv(pclk: u32, freq: u32) -> u32 {
    let freq = if freq == 0 { 1 } else { freq };
    let div = match pclk / freq {
        d if d < 2 => 2,
        d if d > 0xFFFF => 0xFFFF,
        d => d,
    };
    div & !1 // SCKDV LSB is read-only 0 (must be even)
}

/// Bounded busy-wait. Returns [`SpiError::Timeout`] instead of hanging the CPU
/// forever if a status bit never asserts (no slave, stuck CS, wrong mode) — the
/// C SDK guards every equivalent wait with `hal_spi_check_timeout_by_count`.
const SPI_WAIT_LOOPS: u32 = 1_000_000;

#[inline]
fn wait_until(mut ready: impl FnMut() -> bool) -> Result<(), SpiError> {
    let mut n = SPI_WAIT_LOOPS;
    while !ready() {
        n -= 1;
        if n == 0 {
            return Err(SpiError::Timeout);
        }
    }
    Ok(())
}

impl<'d> Spi<'d, Spi0<'d>> {
    pub fn new_spi0(_spi: Spi0<'d>, config: Config) -> Self {
        configure_spi(0, &config);
        Self { idx: 0, _peripheral: PhantomData }
    }
}
impl<'d> Spi<'d, Spi1<'d>> {
    pub fn new_spi1(_spi: Spi1<'d>, config: Config) -> Self {
        configure_spi(1, &config);
        Self { idx: 1, _peripheral: PhantomData }
    }
}

// ── SPI clock source (CLDO_CRG two-stage divider) ───────────────────────────
// The SPI controller input clock (SSI_CLK) is derived from the 480 MHz FNPLL tap
// by a CLDO_CRG divider, then divided again by the in-controller SCKDV to form
// SCK (SCK = SSI_CLK / SCKDV). The HAL targets SSI_CLK = SPI_CLOCK_HZ (160 MHz);
// `configure_spi_source_clock` programs the CRG divider to establish it and
// switches the SPI clock source TCXO→PLL. Mirrors fbb_ws63 `spi_porting_clock_init`
// (480/bus_clk_MHz into DIV_CTL3[9:5]); see ws63-guide ch8 "时钟树".
#[cfg(feature = "chip-ws63")]
const CLDO_CRG_DIV_CTL3: usize = 0x4400_1114; // SPI source divider
#[cfg(feature = "chip-ws63")]
const CLDO_CRG_CLK_SEL: usize = 0x4400_1134; // bit 6 = SPI source: 1=PLL, 0=TCXO
#[cfg(feature = "chip-ws63")]
const CLDO_SUB_CRG_CKEN_CTL1: usize = 0x4400_1104; // bit 25 = SPI clock gate
#[cfg(feature = "chip-ws63")]
const SPI_PLL_ROOT_MHZ: u32 = 480; // FNPLL SPI/QSPI tap (2880 / 6)

/// Establish the two-stage SPI clock: program the CLDO_CRG divider so the SPI
/// controller input clock (SSI_CLK) = [`crate::soc::chip::SPI_CLOCK_HZ`] off the
/// 480 MHz PLL, then switch the SPI clock source from TCXO to PLL
/// (gate-close → switch → gate-open). Bus-agnostic (one divider/select/gate for
/// the whole SPI domain) and idempotent. Requires the PLL to already be locked
/// (the app's `clock_init` does this before any driver init).
#[cfg(feature = "chip-ws63")]
fn configure_spi_source_clock() {
    // CRG divider output = 480 MHz / div → div = 480 / SSI_CLK_MHz (e.g. 3 for 160 MHz).
    let ssi_mhz = (crate::soc::chip::SPI_CLOCK_HZ / 1_000_000).max(1);
    let div = (SPI_PLL_ROOT_MHZ / ssi_mhz).clamp(1, 0x1F);
    // SAFETY: fixed CLDO_CRG MMIO registers (0x4400_11xx, within the SYS_CTL1
    // range). Word-aligned 32-bit RMW; the load-bit handshake matches the SDK.
    unsafe {
        let div_ctl3 = CLDO_CRG_DIV_CTL3 as *mut u32;
        // load-disable (bit10) → set [9:5]=div, [4:0]=1 → load-enable, per the SDK.
        let v = core::ptr::read_volatile(div_ctl3) & !(1 << 10);
        core::ptr::write_volatile(div_ctl3, v);
        let v = (core::ptr::read_volatile(div_ctl3) & !(0x1F << 5) & !0x1F) | ((div & 0x1F) << 5) | 0x1;
        core::ptr::write_volatile(div_ctl3, v);
        core::ptr::write_volatile(div_ctl3, core::ptr::read_volatile(div_ctl3) | (1 << 10));

        // Gate-close → switch SPI source to PLL (CLK_SEL bit 6) → gate-open (CKEN1 bit 25).
        let cken1 = CLDO_SUB_CRG_CKEN_CTL1 as *mut u32;
        let clk_sel = CLDO_CRG_CLK_SEL as *mut u32;
        core::ptr::write_volatile(cken1, core::ptr::read_volatile(cken1) & !(1 << 25));
        core::ptr::write_volatile(clk_sel, core::ptr::read_volatile(clk_sel) | (1 << 6));
        core::ptr::write_volatile(cken1, core::ptr::read_volatile(cken1) | (1 << 25));
    }
}

fn configure_spi(idx: u8, config: &Config) {
    // WS63 two-stage SPI clock (CLDO_CRG @0x4400_11xx → PLL). BS2X has a different
    // clock tree (64 MHz app core, different CRG layout), so this WS63-specific CRG
    // sequence must NOT run on BS2X — there the SPI runs off its default input clock
    // and only the in-controller SCKDV divider (spi_brs, below) is programmed against
    // soc::chip::SPI_CLOCK_HZ. TODO(bs2x): port the BS2X SPI source-clock setup from
    // fbb_bs2x spi_porting.c when bringing SPI up on silicon (QEMU ignores the divisor).
    #[cfg(feature = "chip-ws63")]
    configure_spi_source_clock();
    let r = spi_regs(idx);
    r.spi_er().write(|w| unsafe { w.bits(0) });
    r.spi_brs().write(|w| unsafe { w.bits(config.frequency.to_sckdv()) });

    let mut ctra = 0u32;
    match config.mode {
        SpiMode::Mode0 => {}
        SpiMode::Mode1 => ctra |= 1 << 3,
        SpiMode::Mode2 => ctra |= 1 << 4,
        SpiMode::Mode3 => ctra |= (1 << 3) | (1 << 4),
    }
    ctra |= ((config.data_bits.bits() - 1) as u32) << 13;
    // CTRA.trsm (bits 18:19): 0b00 = transmit-and-receive (full duplex).
    // (0b11 is EEPROM-read, NOT TX+RX — leaving trsm = 0 is correct.)
    r.spi_ctra().write(|w| unsafe { w.bits(ctra) });
    r.spi_slenr().write(|w| unsafe { w.bits(0x1) });
    r.spi_er().write(|w| unsafe { w.bits(0x1) });
}

impl<T> Spi<'_, T> {
    pub fn transfer(&mut self, write: &[u8], read: &mut [u8]) -> Result<(), SpiError> {
        let r = spi_regs(self.idx);
        let len = write.len().max(read.len());
        for i in 0..len {
            let tx = if i < write.len() { write[i] as u32 } else { 0 };
            wait_until(|| r.spi_wsr().read().txfnf().bit_is_set())?;
            unsafe { r.spi_dr().write(|w| w.bits(tx)) };
            wait_until(|| r.spi_wsr().read().rxfne().bit_is_set())?;
            let rx = r.spi_dr().read().bits();
            if i < read.len() {
                read[i] = rx as u8;
            }
        }
        Ok(())
    }

    pub fn write(&mut self, data: &[u8]) -> Result<(), SpiError> {
        let r = spi_regs(self.idx);
        for &byte in data {
            wait_until(|| r.spi_wsr().read().txfnf().bit_is_set())?;
            unsafe { r.spi_dr().write(|w| w.bits(byte as u32)) };
        }
        Ok(())
    }

    pub fn register_block(&self) -> &'static crate::soc::pac::spi0::RegisterBlock {
        spi_regs(self.idx)
    }

    /// Wait for all pending TX data to be transmitted and the SPI bus to become idle.
    pub fn wait_idle(&self) -> Result<(), SpiError> {
        let r = spi_regs(self.idx);
        // Wait for TX FIFO to drain, then for the bus to leave the busy state.
        wait_until(|| r.spi_wsr().read().txfe().bit_is_set())?;
        wait_until(|| !r.spi_wsr().read().busy().bit_is_set())?;
        Ok(())
    }
}

fn transfer_in_place_on(idx: u8, buf: &mut [u8]) -> Result<(), SpiError> {
    let r = spi_regs(idx);
    for byte in buf.iter_mut() {
        let tx = *byte as u32;
        wait_until(|| r.spi_wsr().read().txfnf().bit_is_set())?;
        unsafe { r.spi_dr().write(|w| w.bits(tx)) };
        wait_until(|| r.spi_wsr().read().rxfne().bit_is_set())?;
        *byte = r.spi_dr().read().bits() as u8;
    }
    Ok(())
}

#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum SpiError {
    Overflow,
    Timeout,
}

impl embedded_hal::spi::Error for SpiError {
    fn kind(&self) -> embedded_hal::spi::ErrorKind {
        match self {
            SpiError::Overflow => return embedded_hal::spi::ErrorKind::Overrun,
            _ => {}
        }
        #[allow(unreachable_code)]
        embedded_hal::spi::ErrorKind::Other
    }
}
impl embedded_hal::spi::ErrorType for Spi<'_, Spi0<'_>> {
    type Error = SpiError;
}
impl embedded_hal::spi::SpiBus for Spi<'_, Spi0<'_>> {
    fn transfer(&mut self, read: &mut [u8], write: &[u8]) -> Result<(), Self::Error> {
        self.transfer(write, read)
    }
    fn read(&mut self, buf: &mut [u8]) -> Result<(), Self::Error> {
        self.transfer(&[], buf)
    }
    fn write(&mut self, buf: &[u8]) -> Result<(), Self::Error> {
        Spi::write(self, buf)
    }
    fn transfer_in_place(&mut self, buf: &mut [u8]) -> Result<(), Self::Error> {
        transfer_in_place_on(self.idx, buf)
    }
    fn flush(&mut self) -> Result<(), Self::Error> {
        self.wait_idle()
    }
}

// SPI1 also implements the same traits
impl embedded_hal::spi::ErrorType for Spi<'_, Spi1<'_>> {
    type Error = SpiError;
}
impl embedded_hal::spi::SpiBus for Spi<'_, Spi1<'_>> {
    fn transfer(&mut self, read: &mut [u8], write: &[u8]) -> Result<(), Self::Error> {
        self.transfer(write, read)
    }
    fn read(&mut self, buf: &mut [u8]) -> Result<(), Self::Error> {
        self.transfer(&[], buf)
    }
    fn write(&mut self, buf: &[u8]) -> Result<(), Self::Error> {
        Spi::write(self, buf)
    }
    fn transfer_in_place(&mut self, buf: &mut [u8]) -> Result<(), Self::Error> {
        transfer_in_place_on(self.idx, buf)
    }
    fn flush(&mut self) -> Result<(), Self::Error> {
        self.wait_idle()
    }
}

// ── Tests ──────────────────────────────────────────────────────

#[cfg(all(test, not(target_arch = "riscv32")))]
mod tests {
    use super::sckdv;
    use crate::soc::chip::SPI_CLOCK_HZ;

    #[test]
    fn test_sckdv_basic() {
        // 160 MHz / 1 MHz = 160 (SDK writes the divisor directly, no /2, no -1).
        assert_eq!(sckdv(SPI_CLOCK_HZ, 1_000_000), 160);
    }

    #[test]
    fn test_sckdv_is_even_and_min_two() {
        // SCKDV bit0 is read-only 0 → result always even.
        assert_eq!(sckdv(SPI_CLOCK_HZ, 1_000_000) & 1, 0);
        // freq >= pclk clamps to the minimum divisor of 2.
        assert_eq!(sckdv(SPI_CLOCK_HZ, SPI_CLOCK_HZ), 2);
        assert_eq!(sckdv(SPI_CLOCK_HZ, u32::MAX), 2);
    }

    #[test]
    fn test_sckdv_zero_freq_guard() {
        // freq == 0 is treated as 1 Hz → very large divisor, clamped to even max.
        assert_eq!(sckdv(SPI_CLOCK_HZ, 0), 0xFFFE);
    }

    #[test]
    fn test_sckdv_clamps_at_max() {
        assert_eq!(sckdv(SPI_CLOCK_HZ, 1000), 0xFFFE);
    }

    #[test]
    fn spi_hz_rejects_out_of_range() {
        use super::SpiHz;
        // 0 and frequencies whose divider leaves [2, 0xFFFE] are rejected.
        assert!(SpiHz::try_from_hz(0).is_none());
        // Too high: > SPI_CLOCK_HZ/2 (=80 MHz) → div < 2.
        assert!(SpiHz::try_from_hz(SPI_CLOCK_HZ / 2 + 1).is_none());
        assert!(SpiHz::try_from_hz(SPI_CLOCK_HZ).is_none());
        // Too low: div > 0xFFFE (1 Hz → 160 000 000 div).
        assert!(SpiHz::try_from_hz(1).is_none());
        // In range: 1 MHz → div 160; the default const agrees.
        assert_eq!(SpiHz::try_from_hz(1_000_000).unwrap().hz(), 1_000_000);
        assert_eq!(SpiHz::ONE_MHZ.to_sckdv(), 160);
        // Exactly the edges resolve to the min/max even divider.
        assert_eq!(SpiHz::try_from_hz(SPI_CLOCK_HZ / 2).unwrap().to_sckdv(), 2);
    }

    #[test]
    fn data_bits_validates_4_to_16() {
        use super::DataBits;
        assert!(DataBits::new(3).is_none());
        assert!(DataBits::new(17).is_none());
        assert_eq!(DataBits::new(4).unwrap().bits(), 4);
        assert_eq!(DataBits::new(16).unwrap().bits(), 16);
        assert_eq!(DataBits::EIGHT.bits(), 8);
    }
}

// ── Property-based fuzz tests ──────────────────────────────────

#[cfg(all(test, not(target_arch = "riscv32")))]
mod proptests {
    use super::sckdv;
    use proptest::prelude::*;

    proptest! {
        /// Fuzz: sckdv never panics and is always a valid even divisor in [2, 0xFFFE].
        #[test]
        fn sckdv_in_valid_range(freq in any::<u32>()) {
            let d = sckdv(crate::soc::chip::SPI_CLOCK_HZ, freq);
            prop_assert!((2..=0xFFFE).contains(&d), "divisor {} out of range for freq={}", d, freq);
            prop_assert_eq!(d & 1, 0, "divisor {} not even for freq={}", d, freq);
        }

        /// Fuzz: higher frequency → lower-or-equal divisor (monotonic non-increasing).
        #[test]
        fn sckdv_monotonic(freq1 in 1u32.., freq2 in 1u32..) {
            let pclk = crate::soc::chip::SPI_CLOCK_HZ;
            let d1 = sckdv(pclk, freq1);
            let d2 = sckdv(pclk, freq2);
            if freq1 > freq2 {
                prop_assert!(d1 <= d2, "freq1={}(d={}) freq2={}(d={})", freq1, d1, freq2, d2);
            }
        }
    }
}

// ── Async SPI (embedded-hal-async) ──────────────────────────────────────────
// WS63 SPI transfers are FIFO-paced (and the ws63-qemu model loops back
// synchronously), so the async SpiBus methods complete promptly by reusing the
// blocking transfer logic — valid `async fn`s usable from embassy/async tasks. A
// genuinely IRQ-parking variant would add an on_interrupt + IrqSignal once the
// SPI completion IRQ (43/52) is modelled.
#[cfg(feature = "async")]
mod asynch_impl {
    use super::{Spi, Spi0, Spi1};

    macro_rules! async_spi {
        ($inst:ty) => {
            impl embedded_hal_async::spi::SpiBus<u8> for Spi<'_, $inst> {
                async fn read(&mut self, buf: &mut [u8]) -> Result<(), Self::Error> {
                    embedded_hal::spi::SpiBus::read(self, buf)
                }
                async fn write(&mut self, buf: &[u8]) -> Result<(), Self::Error> {
                    embedded_hal::spi::SpiBus::write(self, buf)
                }
                async fn transfer(&mut self, read: &mut [u8], write: &[u8]) -> Result<(), Self::Error> {
                    embedded_hal::spi::SpiBus::transfer(self, read, write)
                }
                async fn transfer_in_place(&mut self, buf: &mut [u8]) -> Result<(), Self::Error> {
                    embedded_hal::spi::SpiBus::transfer_in_place(self, buf)
                }
                async fn flush(&mut self) -> Result<(), Self::Error> {
                    embedded_hal::spi::SpiBus::flush(self)
                }
            }
        };
    }
    async_spi!(Spi0<'_>);
    async_spi!(Spi1<'_>);
}
