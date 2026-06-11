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
//! The WS63 numbers DMA channels in a single **logical** space:
//! - **MDMA** (`Dma0`) owns logical channels **0-3**
//! - **SDMA** (`Sdma0`) owns logical channels **8-11**
//!
//! Both are backed by physical channels 0-3 on their own register block, so
//! `DmaDriver<Sdma0>` accepts channel numbers 8-11 and translates them to the
//! controller-local 0-3 internally (matching the C SDK `hal_dma_ch_get` /
//! `hal_dma_type_get` convention in fbb_ws63 `hal_dmac_v151.c`). Passing a
//! channel outside a controller's logical range panics.

use crate::peripherals::{Dma, Sdma};
use core::marker::PhantomData;

// ── Type-level DMA instance markers ───────────────────────────────

/// DMA instance trait.
pub trait DmaInstance {
    /// Returns the PAC pointer for this DMA controller.
    fn ptr() -> *const crate::soc::pac::dma::RegisterBlock;

    /// Logical channel number of this controller's first physical channel.
    ///
    /// MDMA exposes logical channels `CHANNEL_BASE..CHANNEL_BASE+4` (0-3); SDMA
    /// exposes 8-11. The driver subtracts this base to index the controller's
    /// physical channels 0-3 — see the module-level "Channel addressing" docs.
    const CHANNEL_BASE: u8;
}

/// Marker type for the primary DMA controller (logical channels 0-3).
pub struct Dma0;
impl DmaInstance for Dma0 {
    fn ptr() -> *const crate::soc::pac::dma::RegisterBlock {
        Dma::ptr()
    }
    const CHANNEL_BASE: u8 = 0;
}

/// Marker type for the secure DMA controller (logical channels 8-11).
pub struct Sdma0;
impl DmaInstance for Sdma0 {
    fn ptr() -> *const crate::soc::pac::dma::RegisterBlock {
        Sdma::ptr()
    }
    const CHANNEL_BASE: u8 = 8;
}

/// Translate a logical channel number to this controller's physical channel
/// index (0-3), validating it falls in the controller's logical range.
#[inline]
fn physical_channel_index(base: u8, channel: u8) -> usize {
    assert!(channel >= base && channel < base + 4, "DMA channel out of range for this controller");
    (channel - base) as usize
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
        Self { _instance: PhantomData }
    }

    fn regs() -> &'static crate::soc::pac::dma::RegisterBlock {
        // SAFETY: PAC peripheral pointer is a static physical MMIO address, always valid
        unsafe { &*T::ptr() }
    }

    /// Translate a logical channel number (0-3 for MDMA, 8-11 for SDMA) to this
    /// controller's physical channel index (0-3). Panics if out of range.
    #[inline]
    fn physical_channel(channel: u8) -> usize {
        physical_channel_index(T::CHANNEL_BASE, channel)
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
    /// * `channel` — Logical channel number (0-3 for MDMA, 8-11 for SDMA).
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
        let ch = Self::physical_channel(channel);
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
        let ch = Self::physical_channel(channel);
        let r = Self::regs();
        let cfg = r.dmac_chn_config_0(ch).read().bits();
        unsafe {
            r.dmac_chn_config_0(ch).write(|w| w.bits(cfg | 0x01));
        }
    }

    /// Disable a specific DMA channel.
    pub fn disable_channel(&mut self, channel: u8) {
        let ch = Self::physical_channel(channel);
        let r = Self::regs();
        let cfg = r.dmac_chn_config_0(ch).read().bits();
        unsafe {
            r.dmac_chn_config_0(ch).write(|w| w.bits(cfg & !0x01));
        }
    }

    /// Check if a DMA channel is enabled.
    pub fn channel_enabled(&self, channel: u8) -> bool {
        let ch = Self::physical_channel(channel);
        let mask = 1u32 << ch;
        Self::regs().dmac_en_chns().read().bits() & mask != 0
    }

    /// Check if a channel has data in its FIFO (active transfer).
    pub fn channel_active(&self, channel: u8) -> bool {
        let ch = Self::physical_channel(channel);
        Self::regs().dmac_chn_config_0(ch).read().bits() & (1 << 15) != 0
    }

    /// Halt a DMA channel (ignore further DMA requests).
    pub fn halt_channel(&mut self, channel: u8) {
        let ch = Self::physical_channel(channel);
        let r = Self::regs();
        let cfg = r.dmac_chn_config_0(ch).read().bits();
        unsafe {
            r.dmac_chn_config_0(ch).write(|w| w.bits(cfg | (1 << 16)));
        }
    }

    /// Resume a halted DMA channel.
    pub fn resume_channel(&mut self, channel: u8) {
        let ch = Self::physical_channel(channel);
        let r = Self::regs();
        let cfg = r.dmac_chn_config_0(ch).read().bits();
        unsafe {
            r.dmac_chn_config_0(ch).write(|w| w.bits(cfg & !(1 << 16)));
        }
    }

    /// Issue a software burst request for a channel.
    pub fn burst_request(&mut self, channel: u8) {
        let ch = Self::physical_channel(channel);
        unsafe {
            Self::regs().dmac_burst_req().write(|w| w.bits(1 << ch));
        }
    }

    /// Issue a software single request for a channel.
    pub fn single_request(&mut self, channel: u8) {
        let ch = Self::physical_channel(channel);
        unsafe {
            Self::regs().dmac_single_req().write(|w| w.bits(1 << ch));
        }
    }

    /// Get the raw interrupt status.
    ///
    /// Returns `(transfer_done_mask, error_mask)`. Bit `n` of each mask is the
    /// **physical** channel `n` of this controller — i.e. logical channel
    /// `CHANNEL_BASE + n`. For SDMA, logical channel 8 is bit 0, channel 9 is
    /// bit 1, etc. (the controller's per-channel status registers are local).
    pub fn raw_interrupt_status(&self) -> (u8, u8) {
        let sts = Self::regs().dmac_ori_int_st().read().bits();
        ((sts & 0xFF) as u8, ((sts >> 8) & 0xFF) as u8)
    }

    /// Get the masked interrupt status.
    ///
    /// Returns `(transfer_done_mask, error_mask)`, with the same **physical**
    /// channel bit indexing as [`raw_interrupt_status`](Self::raw_interrupt_status)
    /// (bit `n` = physical channel `n` = logical `CHANNEL_BASE + n`).
    pub fn interrupt_status(&self) -> (u8, u8) {
        let sts = Self::regs().dmac_int_st().read().bits();
        ((sts & 0xFF) as u8, ((sts >> 16) & 0xFF) as u8)
    }

    /// Clear transfer complete interrupt for a channel.
    pub fn clear_transfer_interrupt(&mut self, channel: u8) {
        let ch = Self::physical_channel(channel);
        unsafe {
            Self::regs().dmac_int_clr().write(|w| w.bits(1 << ch));
        }
    }

    /// Clear error interrupt for a channel.
    pub fn clear_error_interrupt(&mut self, channel: u8) {
        let ch = Self::physical_channel(channel);
        unsafe {
            Self::regs().dmac_int_clr().write(|w| w.bits(1 << (ch + 8)));
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
        Self { _instance: PhantomData }
    }
}

// ── DMA peripheral handshaking request IDs ────────────────────────

/// DMA peripheral hardware-handshaking request ID.
///
/// Values are the `HAL_DMA_HANDSHAKING_*` indices from fbb_ws63
/// `drivers/chips/ws63/porting/dma/dma_porting.h` — the hardware request line a
/// channel uses for peripheral-paced flow control. They go into the channel
/// config's `src_peripheral` / `dst_peripheral` field (a 4-bit field; all the
/// IDs below fit). UART bus mapping per `platform_core.h`: UART0 = UART_L,
/// UART1 = UART_H0, UART2 = UART_H1.
///
/// (These superseded the earlier fabricated sequential 0..11 values; only the
/// main-DMA (MDMA) sources hisi-riscv-hal models are listed — the SDMA-group I2C IDs
/// (≥29) don't fit the 4-bit field and aren't modelled here.)
// Peripheral-paced DMA request IDs are WS63-specific (the fbb_ws63 vs fbb_bs2x
// dma_porting.h handshake enums are entirely different). chip-ws63 only; BS2X DMA
// is enabled for memory-to-memory transfers, whose flow control needs no request
// ID. A BS2X request-ID table can be added later from fbb_bs2x dma_porting.h.
#[cfg(feature = "chip-ws63")]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum DmaPeripheral {
    /// No handshaking (tie-off) — used for memory-to-memory transfers.
    Tie0 = 0,
    /// UART0 (UART_L) transmit.
    Uart0Tx = 1,
    /// UART0 (UART_L) receive.
    Uart0Rx = 2,
    /// UART1 (UART_H0) transmit.
    Uart1Tx = 3,
    /// UART1 (UART_H0) receive.
    Uart1Rx = 4,
    /// UART2 (UART_H1) transmit.
    Uart2Tx = 5,
    /// UART2 (UART_H1) receive.
    Uart2Rx = 6,
    /// SPI0 (SPI_MS0) transmit.
    Spi0Tx = 7,
    /// SPI0 (SPI_MS0) receive.
    Spi0Rx = 8,
    /// I2S transmit.
    I2sTx = 11,
    /// I2S receive.
    I2sRx = 12,
    /// SPI1 (SPI_MS1) transmit.
    Spi1Tx = 13,
    /// SPI1 (SPI_MS1) receive.
    Spi1Rx = 14,
}

#[cfg(feature = "chip-ws63")]
impl DmaPeripheral {
    /// The hardware handshaking request ID (the `dma_porting.h` index), as
    /// programmed into the channel config's peripheral-select field.
    pub const fn request_id(self) -> u8 {
        self as u8
    }
}

/// DMA transfer direction.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DmaDirection {
    /// Transmit (memory to peripheral).
    Tx,
    /// Receive (peripheral to memory).
    Rx,
}

impl DmaChannelConfig {
    /// Configure this channel for a **memory → peripheral** transfer to `peri`:
    /// sets `MemToPeripheral` flow control, the destination handshaking ID, and
    /// holds the destination address fixed (a peripheral data register).
    #[cfg(feature = "chip-ws63")]
    pub fn mem_to_peripheral(mut self, peri: DmaPeripheral) -> Self {
        self.flow_control = FlowControl::MemToPeripheral;
        self.dst_peripheral = peri.request_id();
        self.dst_inc = false;
        self
    }

    /// Configure this channel for a **peripheral → memory** transfer from `peri`:
    /// sets `PeripheralToMem` flow control, the source handshaking ID, and holds
    /// the source address fixed (a peripheral data register).
    #[cfg(feature = "chip-ws63")]
    pub fn peripheral_to_mem(mut self, peri: DmaPeripheral) -> Self {
        self.flow_control = FlowControl::PeripheralToMem;
        self.src_peripheral = peri.request_id();
        self.src_inc = false;
        self
    }
}

// The `DmaEligible` / `DmaChannelFor` binding traits were removed (dead — impl'd
// for Spi0/Spi1 but never called; no DmaChannelFor impls). Peripheral-paced DMA
// is now wired through [`DmaPeripheral`] + [`DmaChannelConfig::mem_to_peripheral`]
// / [`peripheral_to_mem`](DmaChannelConfig::peripheral_to_mem), which feed the
// correct handshaking ID + flow control into `configure_channel`.

impl<'d> DmaDriver<'d, Sdma0> {
    /// Create a new secure DMA driver.
    pub fn new_sdma(_sdma: Sdma<'d>) -> Self {
        Self { _instance: PhantomData }
    }
}

// ── Tests ──────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_dma_direction_tx_rx_distinct() {
        // TX and RX directions must map to different peripheral IDs
        assert_ne!(DmaDirection::Tx as u8, DmaDirection::Rx as u8);
    }

    // The request IDs below are the HAL_DMA_HANDSHAKING_* indices from fbb_ws63
    // dma_porting.h (the single source of truth) — NOT a fabricated 0..11 run.

    #[test]
    fn test_dma_peripheral_spi_handshaking_ids() {
        // SPI_MS0 = 7/8, SPI_MS1 = 13/14.
        assert_eq!(DmaPeripheral::Spi0Tx.request_id(), 7);
        assert_eq!(DmaPeripheral::Spi0Rx.request_id(), 8);
        assert_eq!(DmaPeripheral::Spi1Tx.request_id(), 13);
        assert_eq!(DmaPeripheral::Spi1Rx.request_id(), 14);
    }

    #[test]
    fn test_dma_peripheral_uart_handshaking_ids() {
        // UART0=UART_L (1/2), UART1=UART_H0 (3/4), UART2=UART_H1 (5/6).
        assert_eq!(DmaPeripheral::Uart0Tx.request_id(), 1);
        assert_eq!(DmaPeripheral::Uart0Rx.request_id(), 2);
        assert_eq!(DmaPeripheral::Uart1Tx.request_id(), 3);
        assert_eq!(DmaPeripheral::Uart1Rx.request_id(), 4);
        assert_eq!(DmaPeripheral::Uart2Tx.request_id(), 5);
        assert_eq!(DmaPeripheral::Uart2Rx.request_id(), 6);
    }

    #[test]
    fn test_dma_peripheral_i2s_handshaking_ids() {
        assert_eq!(DmaPeripheral::I2sTx.request_id(), 11);
        assert_eq!(DmaPeripheral::I2sRx.request_id(), 12);
    }

    #[test]
    fn test_dma_peripheral_ids_fit_4bit_field() {
        // src/dst_peripheral is a 4-bit channel-config field.
        for p in [
            DmaPeripheral::Uart0Tx,
            DmaPeripheral::Uart2Rx,
            DmaPeripheral::Spi0Tx,
            DmaPeripheral::Spi1Rx,
            DmaPeripheral::I2sTx,
            DmaPeripheral::I2sRx,
        ] {
            assert!(p.request_id() <= 0x0F, "{:?} id {} > 4 bits", p, p.request_id());
        }
    }

    #[test]
    fn test_dma_channel_config_peripheral_wiring() {
        // mem_to_peripheral / peripheral_to_mem set flow control + the handshaking
        // ID on the correct side, and hold the peripheral-register address fixed.
        let tx = DmaChannelConfig::default().mem_to_peripheral(DmaPeripheral::Spi0Tx);
        assert_eq!(tx.flow_control, FlowControl::MemToPeripheral);
        assert_eq!(tx.dst_peripheral, 7);
        assert!(!tx.dst_inc);

        let rx = DmaChannelConfig::default().peripheral_to_mem(DmaPeripheral::Uart1Rx);
        assert_eq!(rx.flow_control, FlowControl::PeripheralToMem);
        assert_eq!(rx.src_peripheral, 4);
        assert!(!rx.src_inc);
    }

    #[test]
    fn test_channel_base_consts() {
        // MDMA owns logical channels 0-3; SDMA owns 8-11.
        assert_eq!(Dma0::CHANNEL_BASE, 0);
        assert_eq!(Sdma0::CHANNEL_BASE, 8);
    }

    #[test]
    fn test_mdma_logical_to_physical() {
        // MDMA: logical channel n == physical channel n.
        for ch in 0u8..4 {
            assert_eq!(physical_channel_index(Dma0::CHANNEL_BASE, ch), ch as usize);
        }
    }

    #[test]
    fn test_sdma_logical_to_physical() {
        // SDMA: logical channels 8-11 map to physical 0-3 on the secure block
        // (matches fbb_ws63 hal_dma_ch_get: ch % 4 with the SDMA base).
        assert_eq!(physical_channel_index(Sdma0::CHANNEL_BASE, 8), 0);
        assert_eq!(physical_channel_index(Sdma0::CHANNEL_BASE, 9), 1);
        assert_eq!(physical_channel_index(Sdma0::CHANNEL_BASE, 10), 2);
        assert_eq!(physical_channel_index(Sdma0::CHANNEL_BASE, 11), 3);
    }

    #[test]
    #[should_panic(expected = "out of range")]
    fn test_mdma_channel_4_panics() {
        // 4 is out of MDMA's logical range (0-3).
        physical_channel_index(Dma0::CHANNEL_BASE, 4);
    }

    #[test]
    #[should_panic(expected = "out of range")]
    fn test_sdma_channel_7_panics() {
        // 7 is below SDMA's logical range (8-11) — passing an MDMA-style index
        // to the secure controller must not silently alias physical channel 0.
        physical_channel_index(Sdma0::CHANNEL_BASE, 7);
    }

    #[test]
    #[should_panic(expected = "out of range")]
    fn test_sdma_channel_12_panics() {
        // 12 is above SDMA's logical range (8-11).
        physical_channel_index(Sdma0::CHANNEL_BASE, 12);
    }
}

// ── Async DMA completion (bespoke; DMA_INT = IRQ 59) ────────────────────────
#[cfg(feature = "async")]
mod asynch_impl {
    use super::{Dma0, DmaDriver};
    use crate::asynch::IrqSignal;
    use crate::interrupt::{self, Interrupt};
    use core::future::Future;
    use core::pin::Pin;
    use core::task::{Context, Poll};

    static DMA_SIGNAL: IrqSignal = IrqSignal::new();

    /// DMA trap hook (IRQ 59): wake the awaiting transfer.
    pub fn on_interrupt() {
        DMA_SIGNAL.signal();
        interrupt::clear_pending(Interrupt::DMA_INT);
    }

    struct DmaDoneFuture;
    impl Future for DmaDoneFuture {
        type Output = ();
        fn poll(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<()> {
            if DMA_SIGNAL.take_fired() {
                Poll::Ready(())
            } else {
                DMA_SIGNAL.register(cx.waker());
                Poll::Pending
            }
        }
    }

    impl DmaDriver<'_, Dma0> {
        /// Await transfer-complete for `channel` (after configuring + enabling it),
        /// then clear its interrupt. Parks on the DMA IRQ on hardware; the WS63
        /// model completes the copy synchronously, so the fast path returns at once.
        pub async fn wait_transfer_done(&mut self, channel: u8) {
            let bit = 1u8 << channel; // Dma0: physical channel == logical channel
            if self.raw_interrupt_status().0 & bit == 0 {
                DMA_SIGNAL.reset();
                // SAFETY: enabling a known, fixed WS63 IRQ line.
                unsafe { interrupt::enable(Interrupt::DMA_INT) };
                if self.raw_interrupt_status().0 & bit == 0 {
                    DmaDoneFuture.await;
                }
            }
            self.clear_transfer_interrupt(channel);
        }
    }
}

#[cfg(feature = "async")]
pub use asynch_impl::on_interrupt;
