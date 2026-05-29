//! DMA (Direct Memory Access) driver for WS63.
//!
//! The WS63 has two DMA controllers:
//! - **DMA** (MDMA) at 0x4A00_0000 — 4 channels, used for general-purpose transfers
//! - **SDMA** (Secure DMA) at 0x520A_0000 — 4 additional channels, same register layout
//!
//! Each channel supports:
//! - Memory-to-memory, memory-to-peripheral, peripheral-to-memory transfers
//! - Linked list (scatter-gather) mode
//! - Configurable source/destination burst sizes and widths
//! - Address increment control
//! - Transfer complete and error interrupts
//!
//! # Channel addressing
//!
//! - DMA channels: 0-3
//! - SDMA channels: 0-3 (logically mapped as 8-11 by the hardware)

use crate::peripherals::{Dma, Sdma};
use core::marker::PhantomData;

// ── Type-level DMA instance markers ───────────────────────────────

/// DMA instance trait.
pub trait DmaInstance {
    /// Returns the PAC pointer for this DMA controller.
    fn ptr() -> *const ws63_pac::dma::RegisterBlock;
}

/// Marker type for the primary DMA controller.
pub struct Dma0;
impl DmaInstance for Dma0 {
    fn ptr() -> *const ws63_pac::dma::RegisterBlock {
        Dma::ptr()
    }
}

/// Marker type for the secure DMA controller.
pub struct Sdma0;
impl DmaInstance for Sdma0 {
    fn ptr() -> *const ws63_pac::dma::RegisterBlock {
        Sdma::ptr()
    }
}

// ── Configuration types ───────────────────────────────────────────

/// Transfer width (data size per beat).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TransferWidth {
    /// 8 bits (1 byte).
    Width8 = 0,
    /// 16 bits (2 bytes).
    Width16 = 1,
    /// 32 bits (4 bytes).
    Width32 = 2,
}

/// Burst size (number of beats per burst).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BurstSize {
    /// 1 beat.
    Beats1 = 0,
    /// 4 beats.
    Beats4 = 1,
    /// 8 beats.
    Beats8 = 2,
    /// 16 beats.
    Beats16 = 3,
    /// 32 beats.
    Beats32 = 4,
    /// 64 beats.
    Beats64 = 5,
    /// 128 beats.
    Beats128 = 6,
    /// 256 beats.
    Beats256 = 7,
}

/// DMA transfer direction / flow control.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FlowControl {
    /// Memory-to-memory transfer.
    MemToMem = 0,
    /// Memory-to-peripheral transfer.
    MemToPeripheral = 1,
    /// Peripheral-to-memory transfer.
    PeripheralToMem = 2,
    /// Peripheral-to-peripheral transfer.
    PeripheralToPeripheral = 3,
}

/// DMA channel configuration.
#[derive(Debug, Clone, Copy)]
pub struct DmaChannelConfig {
    /// Source peripheral select (0-15).
    pub src_peripheral: u8,
    /// Destination peripheral select (0-15).
    pub dst_peripheral: u8,
    /// Flow control mode.
    pub flow_control: FlowControl,
    /// Source transfer width.
    pub src_width: TransferWidth,
    /// Destination transfer width.
    pub dst_width: TransferWidth,
    /// Source burst size.
    pub src_burst: BurstSize,
    /// Destination burst size.
    pub dst_burst: BurstSize,
    /// Increment source address after each beat.
    pub src_inc: bool,
    /// Increment destination address after each beat.
    pub dst_inc: bool,
    /// Enable transfer complete interrupt.
    pub transfer_int: bool,
    /// Enable error interrupt.
    pub error_int: bool,
    /// Bus lock during transfer.
    pub bus_lock: bool,
}

impl Default for DmaChannelConfig {
    fn default() -> Self {
        Self {
            src_peripheral: 0,
            dst_peripheral: 0,
            flow_control: FlowControl::MemToMem,
            src_width: TransferWidth::Width32,
            dst_width: TransferWidth::Width32,
            src_burst: BurstSize::Beats1,
            dst_burst: BurstSize::Beats1,
            src_inc: true,
            dst_inc: true,
            transfer_int: false,
            error_int: false,
            bus_lock: false,
        }
    }
}

// ── DMA driver ────────────────────────────────────────────────────

/// DMA controller driver.
pub struct DmaDriver<'d, T: DmaInstance> {
    _instance: PhantomData<&'d T>,
}

impl<'d, T: DmaInstance> DmaDriver<'d, T> {
    /// Create a new DMA driver from a DMA peripheral.
    pub fn new(_dma: impl Into<PhantomData<&'d T>>) -> Self {
        Self {
            _instance: PhantomData,
        }
    }

    fn regs() -> &'static ws63_pac::dma::RegisterBlock {
        unsafe { &*T::ptr() }
    }

    /// Enable the DMA controller.
    pub fn enable_controller(&mut self) {
        let r = Self::regs();
        unsafe {
            r.dmac_config().write(|w| w.bits(0x01));
        }
    }

    /// Disable the DMA controller.
    pub fn disable_controller(&mut self) {
        unsafe {
            Self::regs().dmac_config().write(|w| w.bits(0));
        }
    }

    /// Configure a DMA channel.
    ///
    /// * `channel` — Channel index (0-3).
    /// * `src_addr` — Source address.
    /// * `dst_addr` — Destination address.
    /// * `transfer_size` — Number of source-width beats to transfer.
    /// * `config` — Channel configuration.
    pub fn configure_channel(
        &mut self,
        channel: u8,
        src_addr: u32,
        dst_addr: u32,
        transfer_size: u16,
        config: &DmaChannelConfig,
    ) {
        assert!(channel < 4);
        let ch = channel as usize;
        let r = Self::regs();

        // Disable channel before configuration
        unsafe {
            r.dmac_chn_config_0(ch).write(|w| w.bits(0));
        }

        // Set source address
        unsafe {
            r.dmac_s_addr_0(ch).write(|w| w.bits(src_addr));
        }

        // Set destination address
        unsafe {
            r.dmac_d_addr_0(ch).write(|w| w.bits(dst_addr));
        }

        // Clear linked list pointer
        unsafe {
            r.dmac_lli_0(ch).write(|w| w.bits(0));
        }

        // Build control register
        let mut control: u32 = 0;
        control |= (transfer_size as u32) & 0xFFF; // trans_size [0:11]
        control |= ((config.src_burst as u32) & 0x07) << 12; // s_bsize [12:14]
        control |= ((config.dst_burst as u32) & 0x07) << 15; // d_bsize [15:17]
        control |= ((config.src_width as u32) & 0x07) << 18; // s_width [18:20]
        control |= ((config.dst_width as u32) & 0x07) << 21; // d_width [21:23]
        control |= 0 << 24; // s_master = M1
        control |= 0 << 25; // d_master = M1
        if config.src_inc {
            control |= 1 << 26;
        }
        if config.dst_inc {
            control |= 1 << 27;
        }
        control |= 0 << 28; // prot = 0
        if config.transfer_int {
            control |= 1 << 31;
        }

        unsafe {
            r.dmac_chn_control_0(ch).write(|w| w.bits(control));
        }

        // Build channel config register
        let mut ch_cfg: u32 = 0;
        ch_cfg |= 0x01; // chn_en
        ch_cfg |= ((config.src_peripheral as u32) & 0x0F) << 1; // s_peripheral [1:4]
        ch_cfg |= ((config.dst_peripheral as u32) & 0x0F) << 5; // d_peripheral [5:8]
        ch_cfg |= ((config.flow_control as u32) & 0x07) << 9; // flow_ctl [9:11]
        if config.error_int {
            ch_cfg |= 1 << 12; // int_en
        }
        if config.transfer_int {
            ch_cfg |= 1 << 13; // int_tc
        }
        if config.bus_lock {
            ch_cfg |= 1 << 14; // lock
        }

        unsafe {
            r.dmac_chn_config_0(ch).write(|w| w.bits(ch_cfg));
        }
    }

    /// Enable a specific DMA channel.
    pub fn enable_channel(&mut self, channel: u8) {
        assert!(channel < 4);
        let ch = channel as usize;
        let r = Self::regs();
        let cfg = r.dmac_chn_config_0(ch).read().bits();
        unsafe {
            r.dmac_chn_config_0(ch).write(|w| w.bits(cfg | 0x01));
        }
    }

    /// Disable a specific DMA channel.
    pub fn disable_channel(&mut self, channel: u8) {
        assert!(channel < 4);
        let ch = channel as usize;
        let r = Self::regs();
        let cfg = r.dmac_chn_config_0(ch).read().bits();
        unsafe {
            r.dmac_chn_config_0(ch).write(|w| w.bits(cfg & !0x01));
        }
    }

    /// Check if a DMA channel is enabled.
    pub fn channel_enabled(&self, channel: u8) -> bool {
        let mask = 1 << channel;
        Self::regs().dmac_en_chns().read().bits() & mask != 0
    }

    /// Check if a channel has data in its FIFO (active transfer).
    pub fn channel_active(&self, channel: u8) -> bool {
        assert!(channel < 4);
        let ch = channel as usize;
        Self::regs().dmac_chn_config_0(ch).read().bits() & (1 << 15) != 0
    }

    /// Halt a DMA channel (ignore further DMA requests).
    pub fn halt_channel(&mut self, channel: u8) {
        assert!(channel < 4);
        let ch = channel as usize;
        let r = Self::regs();
        let cfg = r.dmac_chn_config_0(ch).read().bits();
        unsafe {
            r.dmac_chn_config_0(ch).write(|w| w.bits(cfg | (1 << 16)));
        }
    }

    /// Resume a halted DMA channel.
    pub fn resume_channel(&mut self, channel: u8) {
        assert!(channel < 4);
        let ch = channel as usize;
        let r = Self::regs();
        let cfg = r.dmac_chn_config_0(ch).read().bits();
        unsafe {
            r.dmac_chn_config_0(ch).write(|w| w.bits(cfg & !(1 << 16)));
        }
    }

    /// Issue a software burst request for a channel.
    pub fn burst_request(&mut self, channel: u8) {
        unsafe {
            Self::regs()
                .dmac_burst_req()
                .write(|w| w.bits(1 << channel));
        }
    }

    /// Issue a software single request for a channel.
    pub fn single_request(&mut self, channel: u8) {
        unsafe {
            Self::regs()
                .dmac_single_req()
                .write(|w| w.bits(1 << channel));
        }
    }

    /// Get the raw interrupt status.
    ///
    /// Returns `(transfer_done_mask, error_mask)`.
    pub fn raw_interrupt_status(&self) -> (u8, u8) {
        let sts = Self::regs().dmac_ori_int_st().read().bits();
        ((sts & 0xFF) as u8, ((sts >> 8) & 0xFF) as u8)
    }

    /// Get the masked interrupt status.
    ///
    /// Returns `(transfer_done_mask, error_mask)`.
    pub fn interrupt_status(&self) -> (u8, u8) {
        let sts = Self::regs().dmac_int_st().read().bits();
        (
            (sts & 0xFF) as u8,
            ((sts >> 16) & 0xFF) as u8,
        )
    }

    /// Clear transfer complete interrupt for a channel.
    pub fn clear_transfer_interrupt(&mut self, channel: u8) {
        unsafe {
            Self::regs().dmac_int_clr().write(|w| w.bits(1 << channel));
        }
    }

    /// Clear error interrupt for a channel.
    pub fn clear_error_interrupt(&mut self, channel: u8) {
        unsafe {
            Self::regs()
                .dmac_int_clr()
                .write(|w| w.bits(1 << (channel + 8)));
        }
    }

    /// Set DMA sync configuration.
    ///
    /// Each bit controls sync for the corresponding channel
    /// (0 = enable sync logic, 1 = disable sync logic).
    pub fn set_sync(&mut self, sync_mask: u16) {
        unsafe {
            Self::regs().dmac_sync().write(|w| w.bits(sync_mask as u32));
        }
    }
}

// ── Convenience constructors ──────────────────────────────────────

impl<'d> DmaDriver<'d, Dma0> {
    /// Create a new primary DMA driver.
    pub fn new_dma(_dma: Dma<'d>) -> Self {
        Self {
            _instance: PhantomData,
        }
    }
}

// ── DMA peripheral binding traits ─────────────────────────────────

/// Enumeration of DMA peripheral request sources.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DmaPeripheral {
    /// SPI0 TX
    Spi0Tx = 0,
    /// SPI0 RX
    Spi0Rx = 1,
    /// SPI1 TX
    Spi1Tx = 2,
    /// SPI1 RX
    Spi1Rx = 3,
    /// UART0 TX
    Uart0Tx = 4,
    /// UART0 RX
    Uart0Rx = 5,
    /// UART1 TX
    Uart1Tx = 6,
    /// UART1 RX
    Uart1Rx = 7,
    /// UART2 TX
    Uart2Tx = 8,
    /// UART2 RX
    Uart2Rx = 9,
    /// I2S TX
    I2sTx = 10,
    /// I2S RX
    I2sRx = 11,
}

/// Trait indicating a peripheral is DMA-eligible.
///
/// Each peripheral that supports DMA transfers specifies which
/// DMA channel it can use and its peripheral request ID.
pub trait DmaEligible {
    /// The DMA peripheral identifier for this peripheral.
    fn dma_peripheral() -> DmaPeripheral;
}

/// Trait bound ensuring a DMA channel can be used with a given peripheral.
pub trait DmaChannelFor<P: DmaEligible> {}

impl<'d, T, P> DmaChannelFor<P> for DmaDriver<'d, T>
where
    T: DmaInstance,
    P: DmaEligible,
{
}

/// Helper type alias for a DMA channel compatible with a peripheral.
pub type PeripheralDmaChannel<P> = core::marker::PhantomData<P>;

// ── DmaEligible implementations ──────────────────────────────────

use crate::peripherals::{Spi0, Spi1};

impl DmaEligible for Spi0<'static> {
    fn dma_peripheral() -> DmaPeripheral {
        DmaPeripheral::Spi0Tx
    }
}

impl DmaEligible for Spi1<'static> {
    fn dma_peripheral() -> DmaPeripheral {
        DmaPeripheral::Spi1Tx
    }
}

impl<'d> DmaDriver<'d, Sdma0> {
    /// Create a new secure DMA driver.
    pub fn new_sdma(_sdma: Sdma<'d>) -> Self {
        Self {
            _instance: PhantomData,
        }
    }
}
