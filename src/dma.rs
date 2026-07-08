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

mod sealed {
    pub trait DmaInstanceSealed {}
}

// ── Type-level DMA instance markers ───────────────────────────────

/// DMA instance trait.
pub trait DmaInstance: sealed::DmaInstanceSealed {
    /// Returns the PAC pointer for this DMA controller.
    fn ptr() -> *const crate::soc::pac::dma::RegisterBlock;

    /// Logical channel number of this controller's first physical channel.
    ///
    /// MDMA exposes logical channels `CHANNEL_BASE..CHANNEL_BASE+4` (0-3); SDMA
    /// exposes 8-11. The driver subtracts this base to index the controller's
    /// physical channels 0-3 — see the module-level "Channel addressing" docs.
    const CHANNEL_BASE: u8;

    /// Bypass auto clock-gating for this controller, if needed.
    ///
    /// The WS63 M_DMA clock is auto-gated when the controller is idle; the
    /// primary controller must set this bit so its clock keeps running across a
    /// transfer, otherwise a started transfer never advances (its done bit never
    /// sets). Mirrors the vendor `dma_port_register_irq()` "BYPASS dma自动门控"
    /// (`DMA_CLK_AUTO_CTRL_REG |= DMA_CLK_ON_MASK`).
    fn bypass_auto_clock_gate() {}
}

/// Marker type for the primary DMA controller (logical channels 0-3).
pub struct Dma0;
impl sealed::DmaInstanceSealed for Dma0 {}
impl DmaInstance for Dma0 {
    fn ptr() -> *const crate::soc::pac::dma::RegisterBlock {
        Dma::ptr()
    }
    const CHANNEL_BASE: u8 = 0;
    fn bypass_auto_clock_gate() {
        #[cfg(feature = "chip-ws63")]
        {
            let r = unsafe { &*crate::peripherals::SysCtl1::ptr() };
            r.ip_auto_cg_bypass().modify(|_, w| w.dma_clk_on().set_bit());
        }
    }
}

/// Marker type for the secure DMA controller (logical channels 8-11).
#[instability::unstable]
pub struct Sdma0;
impl sealed::DmaInstanceSealed for Sdma0 {}
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

/// DMA transfer length in source-width beats.
///
/// The DesignWare v151 single-block `trans_size` field is 12 bits, so a single
/// descriptor can transfer at most 4095 beats. Values are constructed fallibly so
/// callers cannot accidentally rely on the old silent low-12-bit truncation.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
#[instability::unstable]
pub struct DmaTransferSize(u16);

impl DmaTransferSize {
    /// Maximum number of beats in one single-block DMA transfer.
    pub const MAX_BEATS: usize = 0x0fff;

    /// Build a transfer size from a beat count, rejecting values that do not fit
    /// the hardware `trans_size` field.
    pub const fn from_beats(beats: usize) -> Option<Self> {
        if beats <= Self::MAX_BEATS { Some(Self(beats as u16)) } else { None }
    }

    #[inline]
    const fn get(self) -> u16 {
        self.0
    }
}

/// DMA sync register mask for the four physical channels of one controller.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[instability::unstable]
pub struct DmaSyncMask(u8);

impl DmaSyncMask {
    /// Build a sync mask. Only bits 0..=3 are valid because each controller has
    /// four physical channels.
    pub const fn from_bits(bits: u16) -> Option<Self> {
        if bits & !0x000f == 0 { Some(Self(bits as u8)) } else { None }
    }

    /// No channel sync bypass bits set.
    pub const NONE: Self = Self(0);

    /// All channel sync bypass bits set.
    pub const ALL: Self = Self(0x0f);

    #[inline]
    const fn bits(self) -> u16 {
        self.0 as u16
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
    src_peripheral: u8,
    /// Destination peripheral select (0-15).
    dst_peripheral: u8,
    /// Flow control mode.
    flow_control: FlowControl,
    /// Source transfer width.
    src_width: TransferWidth,
    /// Destination transfer width.
    dst_width: TransferWidth,
    /// Source burst size.
    src_burst: BurstSize,
    /// Destination burst size.
    dst_burst: BurstSize,
    /// Increment source address after each beat.
    src_inc: bool,
    /// Increment destination address after each beat.
    dst_inc: bool,
    /// Enable transfer complete interrupt.
    transfer_int: bool,
    /// Enable error interrupt.
    error_int: bool,
    /// Bus lock during transfer.
    bus_lock: bool,
}

impl Default for DmaChannelConfig {
    fn default() -> Self {
        Self::mem_to_mem()
    }
}

impl DmaChannelConfig {
    /// Memory-to-memory, 32-bit beat, incrementing on both sides.
    pub const fn mem_to_mem() -> Self {
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

    #[inline]
    fn control_word(self, transfer_size: DmaTransferSize) -> u32 {
        let mut control: u32 = 0;
        control |= (transfer_size.get() as u32) & 0x0fff; // trans_size [0:11]
        control |= ((self.src_burst as u32) & 0x07) << 12; // s_bsize [12:14]
        control |= ((self.dst_burst as u32) & 0x07) << 15; // d_bsize [15:17]
        control |= ((self.src_width as u32) & 0x07) << 18; // s_width [18:20]
        control |= ((self.dst_width as u32) & 0x07) << 21; // d_width [21:23]
        if self.src_inc {
            control |= 1 << 26;
        }
        if self.dst_inc {
            control |= 1 << 27;
        }
        if self.transfer_int {
            control |= 1 << 31;
        }
        control
    }

    #[inline]
    fn channel_config_word(self) -> u32 {
        let mut ch_cfg: u32 = 0;
        ch_cfg |= 0x01; // chn_en
        ch_cfg |= ((self.src_peripheral as u32) & 0x0f) << 1; // s_peripheral [1:4]
        ch_cfg |= ((self.dst_peripheral as u32) & 0x0f) << 5; // d_peripheral [5:8]
        ch_cfg |= ((self.flow_control as u32) & 0x07) << 9; // flow_ctl [9:11]
        if self.error_int {
            ch_cfg |= 1 << 12;
        }
        if self.transfer_int {
            ch_cfg |= 1 << 13;
        }
        if self.bus_lock {
            ch_cfg |= 1 << 14;
        }
        ch_cfg
    }
}

// ── DMA driver ────────────────────────────────────────────────────

/// DMA controller driver.
pub struct DmaDriver<'d, T: DmaInstance> {
    _instance: PhantomData<&'d T>,
}

impl<'d, T: DmaInstance> DmaDriver<'d, T> {
    #[cfg(all(test, not(target_arch = "riscv32")))]
    fn new_for_test() -> Self {
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
    ///
    /// First bypasses the controller's auto clock-gate (if it has one — the WS63
    /// M_DMA does), so the DMA clock stays running across a transfer. Without
    /// this the idle-gated clock leaves a started transfer stuck (done bit never
    /// sets); this was why `dma_mem_to_mem` failed on silicon while passing under
    /// QEMU, which models no clock gating. Mirrors the vendor `dma_porting`.
    pub fn enable_controller(&mut self) {
        T::bypass_auto_clock_gate();
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
    pub(crate) fn configure_channel_raw(
        &mut self,
        channel: u8,
        src_addr: u32,
        dst_addr: u32,
        transfer_size: DmaTransferSize,
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

        let control = config.control_word(transfer_size);
        unsafe {
            r.dmac_chn_control_0(ch).write(|w| w.bits(control));
        }

        let ch_cfg = config.channel_config_word();
        unsafe {
            r.dmac_chn_config_0(ch).write(|w| w.bits(ch_cfg));
        }

        // Actually START the transfer: set this channel's bit in the global
        // channel-enable register (`dmac_en_chns`, the DesignWare ChEnReg). On
        // real silicon the per-channel CFG `ch_enable` bit alone does NOT kick the
        // engine — the vendor `hal_dma_v151_enable` writes `en_chns`, and the
        // hardware auto-clears the bit when the (single-block) transfer completes.
        // (QEMU runs its synchronous memcpy on the CFG write and may not model
        // `en_chns`, so this is a harmless extra write there.) Completion is then
        // observed via `channel_enabled()` going false — see the HIL test.
        let en = r.dmac_en_chns().read().bits();
        unsafe {
            r.dmac_en_chns().write(|w| w.bits(en | (1 << ch)));
        }
    }

    pub(crate) fn enable_channel_raw(&mut self, channel: u8) {
        let ch = Self::physical_channel(channel);
        let r = Self::regs();
        let cfg = r.dmac_chn_config_0(ch).read().bits();
        unsafe {
            r.dmac_chn_config_0(ch).write(|w| w.bits(cfg | 0x01));
        }
    }

    pub(crate) fn disable_channel_raw(&mut self, channel: u8) {
        let ch = Self::physical_channel(channel);
        let r = Self::regs();
        let cfg = r.dmac_chn_config_0(ch).read().bits();
        unsafe {
            r.dmac_chn_config_0(ch).write(|w| w.bits(cfg & !0x01));
        }
    }

    pub(crate) fn channel_enabled_raw(&self, channel: u8) -> bool {
        let ch = Self::physical_channel(channel);
        let mask = 1u32 << ch;
        Self::regs().dmac_en_chns().read().bits() & mask != 0
    }

    pub(crate) fn channel_active_raw(&self, channel: u8) -> bool {
        let ch = Self::physical_channel(channel);
        Self::regs().dmac_chn_config_0(ch).read().bits() & (1 << 15) != 0
    }

    pub(crate) fn halt_channel_raw(&mut self, channel: u8) {
        let ch = Self::physical_channel(channel);
        let r = Self::regs();
        let cfg = r.dmac_chn_config_0(ch).read().bits();
        unsafe {
            r.dmac_chn_config_0(ch).write(|w| w.bits(cfg | (1 << 16)));
        }
    }

    pub(crate) fn resume_channel_raw(&mut self, channel: u8) {
        let ch = Self::physical_channel(channel);
        let r = Self::regs();
        let cfg = r.dmac_chn_config_0(ch).read().bits();
        unsafe {
            r.dmac_chn_config_0(ch).write(|w| w.bits(cfg & !(1 << 16)));
        }
    }

    pub(crate) fn burst_request_raw(&mut self, channel: u8) {
        let ch = Self::physical_channel(channel);
        unsafe {
            Self::regs().dmac_burst_req().write(|w| w.bits(1 << ch));
        }
    }

    pub(crate) fn single_request_raw(&mut self, channel: u8) {
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

    pub(crate) fn clear_transfer_interrupt_raw(&mut self, channel: u8) {
        let ch = Self::physical_channel(channel);
        unsafe {
            Self::regs().dmac_int_clr().write(|w| w.bits(1 << ch));
        }
    }

    pub(crate) fn clear_error_interrupt_raw(&mut self, channel: u8) {
        let ch = Self::physical_channel(channel);
        unsafe {
            Self::regs().dmac_int_clr().write(|w| w.bits(1 << (ch + 8)));
        }
    }

    /// Set DMA sync configuration.
    ///
    /// Each bit controls sync for the corresponding channel
    /// (0 = enable sync logic, 1 = disable sync logic).
    #[instability::unstable]
    pub fn set_sync(&mut self, sync_mask: DmaSyncMask) {
        unsafe {
            Self::regs().dmac_sync().write(|w| w.bits(sync_mask.bits() as u32));
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
    ///
    /// NOTE: the PAC's `Spi1` instance (0x4402_1000) may actually be QSPI0_2CS
    /// (vendor `HAL_DMA_HANDSHAKING_QSPI0_2CS_TX` = 9) per `spi_porting.h:38` —
    /// silicon-unverified. The correct `Qspi02csTx = 9` variant is deferred to P2
    /// (when the SPI1 DMA wiring is silicon-verified) so an unverified breaking
    /// enum addition isn't shipped in P0.
    Spi1Tx = 13,
    /// SPI1 (SPI_MS1) receive. See [`Spi1Tx`](Self::Spi1Tx) — QSPI0_2CS (10) unverified.
    Spi1Rx = 14,
}

#[cfg(feature = "chip-ws63")]
impl DmaPeripheral {
    /// The hardware handshaking request ID (the `dma_porting.h` index), as
    /// programmed into the channel config's peripheral-select field.
    pub(crate) const fn request_id(self) -> u8 {
        self as u8
    }
}

/// DMA transfer direction.
#[cfg(feature = "chip-ws63")]
#[instability::unstable]
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
    #[instability::unstable]
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
    #[instability::unstable]
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
    #[instability::unstable]
    pub fn new_sdma(_sdma: Sdma<'d>) -> Self {
        Self { _instance: PhantomData }
    }
}

// ── Owned-buffer transfer guard (embedded-dma) ──────────────────────────────

use core::mem::ManuallyDrop;
use embedded_dma::{ReadBuffer, WriteBuffer};

/// The DMA cache-maintenance granularity (the 32-byte D-cache line). Source and
/// destination buffers should be [`DMA_ALIGN`]-aligned so a partial-line invalidate
/// on `wait()` cannot clobber a neighbouring allocation's dirty data (see
/// [`crate::cache::invalidate_range`]).
pub const DMA_ALIGN: usize = 32;

/// Bounded completion poll for [`Transfer::wait`] — a wedged channel returns control
/// (the transfer is then treated as done) instead of spinning the CPU forever.
const DMA_WAIT_LOOPS: u32 = 5_000_000;

impl<'d> DmaDriver<'d, Dma0> {
    /// Launch a **memory-to-memory** transfer on `channel` that OWNS both buffers
    /// for its whole lifetime, returning a [`Transfer`] guard you [`wait`] to reclaim
    /// the driver + buffers.
    ///
    /// Because the guard owns the buffers (and aborts the channel on `Drop`), a
    /// use-after-free of an in-flight DMA region is **unrepresentable in safe code**:
    /// the buffers cannot be touched, moved, or freed while the DMA reads/writes them.
    /// The source cache lines are cleaned before launch and the destination
    /// invalidated on `wait()` (the WS63 core is non-coherent), so the cache
    /// maintenance lives inside the type rather than being a caller obligation.
    ///
    /// The DMA beat is a 32-bit word (`Word = u32`); buffers should be
    /// [`DMA_ALIGN`]-aligned. The transfer length is `min(src.len(), dst.len())` words.
    ///
    /// [`wait`]: Transfer::wait
    #[instability::unstable]
    pub fn start_mem_to_mem<SRC, DST>(
        mut self,
        channel: DmaChannel,
        src: SRC,
        mut dst: DST,
    ) -> Result<Transfer<'d, SRC, DST>, DmaStartError<'d, SRC, DST>>
    where
        SRC: ReadBuffer<Word = u32>,
        DST: WriteBuffer<Word = u32>,
    {
        // SAFETY: `src`/`dst` are moved into the returned Transfer and held until
        // wait() reclaims them, so these pointers stay valid for the whole transfer.
        let (src_ptr, src_len) = unsafe { src.read_buffer() };
        let (dst_ptr, dst_len) = unsafe { dst.write_buffer() };
        let words = src_len.min(dst_len);
        let transfer_size = match DmaTransferSize::from_beats(words) {
            Some(size) => size,
            None => return Err(DmaStartError { error: DmaError::TransferTooLarge, driver: self, channel, src, dst }),
        };
        let bytes = words * core::mem::size_of::<u32>();

        self.enable_controller();

        // Clean the source so the DMA master reads CPU-written data (non-coherent core).
        #[cfg(feature = "chip-ws63")]
        unsafe {
            crate::cache::clean_range(src_ptr as usize, bytes);
        }

        // mem-to-mem, 32-bit, both addresses incrementing (the Default config).
        let cfg = DmaChannelConfig::default();
        let ch = channel.logical;
        self.configure_channel_raw(ch, src_ptr as u32, dst_ptr as u32, transfer_size, &cfg);

        Ok(Transfer { driver: self, channel, src, dst, dst_addr: dst_ptr as usize, bytes })
    }
}

/// An in-flight DMA transfer that **owns the driver and both buffers**. Reclaim them
/// with [`wait`](Self::wait); dropping the guard early aborts the channel (so the DMA
/// stops touching the buffers before they are freed). Owning the buffers is what makes
/// a use-after-free of an active DMA region unrepresentable in safe code.
#[instability::unstable]
pub struct Transfer<'d, SRC, DST> {
    driver: DmaDriver<'d, Dma0>,
    channel: DmaChannel,
    src: SRC,
    dst: DST,
    // Only read by the chip-ws63 cache maintenance in `wait()`; BS2X DMA is coherent
    // here, so these are unused there.
    #[cfg_attr(not(feature = "chip-ws63"), allow(dead_code))]
    dst_addr: usize,
    #[cfg_attr(not(feature = "chip-ws63"), allow(dead_code))]
    bytes: usize,
}

/// Driver, channel token, and buffers recovered from a completed DMA transfer.
#[instability::unstable]
pub type DmaTransferParts<'d, SRC, DST> = (DmaDriver<'d, Dma0>, DmaChannel, SRC, DST);

/// Result returned by [`Transfer::wait`].
#[instability::unstable]
pub type DmaWaitResult<'d, SRC, DST> = Result<DmaTransferParts<'d, SRC, DST>, DmaWaitError<'d, SRC, DST>>;

impl<'d, SRC, DST> Transfer<'d, SRC, DST> {
    /// True once the channel has auto-cleared its enable bit (single-block done).
    pub fn is_done(&self) -> bool {
        !self.driver.channel_enabled_raw(self.channel.logical)
    }

    /// Block (bounded) until the transfer completes, invalidate the destination cache
    /// lines, and return the driver + both buffers. Consumes the guard, so no abort
    /// `Drop` runs.
    ///
    /// On `DMA_WAIT_LOOPS` exhaustion (a wedged channel — rare for mem-to-mem, which
    /// always self-completes) the channel is **quiesced** (halt → drain `active` →
    /// disable) *before* the buffers are handed back, so the DMA engine is provably
    /// stopped before the buffers can be freed. A wedged transfer is reported as
    /// [`DmaError::Timeout`] with the driver, channel token, and buffers returned.
    pub fn wait(self) -> DmaWaitResult<'d, SRC, DST> {
        // Skip the abort-on-drop: the transfer is completing normally.
        let mut this = ManuallyDrop::new(self);
        let mut n = DMA_WAIT_LOOPS;
        while this.driver.channel_enabled_raw(this.channel.logical) {
            n -= 1;
            if n == 0 {
                break;
            }
            core::hint::spin_loop();
        }
        // If the channel never auto-cleared, quiesce it before the buffers escape —
        // otherwise a still-live engine could write a buffer the caller then frees.
        // (Copy `channel` out so the `&mut this.driver` calls don't borrow `this` whole.)
        let ch = this.channel.logical;
        let timed_out = this.driver.channel_enabled_raw(ch);
        if timed_out {
            quiesce_channel(&mut this.driver, ch);
        }
        // Drop the stale destination cache lines so the CPU re-reads what DMA wrote.
        #[cfg(feature = "chip-ws63")]
        unsafe {
            crate::cache::invalidate_range(this.dst_addr, this.bytes);
        }
        // SAFETY: `this` is ManuallyDrop (its Drop never runs) and each field is read
        // exactly once, so there is no double-read or double-drop.
        unsafe {
            let driver = core::ptr::read(&this.driver);
            let channel = core::ptr::read(&this.channel);
            let src = core::ptr::read(&this.src);
            let dst = core::ptr::read(&this.dst);
            if timed_out {
                Err(DmaWaitError { error: DmaError::Timeout, driver, channel, src, dst })
            } else {
                Ok((driver, channel, src, dst))
            }
        }
    }
}

/// Resources returned when a DMA transfer cannot be started.
#[instability::unstable]
pub struct DmaStartError<'d, SRC, DST> {
    error: DmaError,
    driver: DmaDriver<'d, Dma0>,
    channel: DmaChannel,
    src: SRC,
    dst: DST,
}

impl<SRC, DST> core::fmt::Debug for DmaStartError<'_, SRC, DST> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("DmaStartError").field("error", &self.error).finish_non_exhaustive()
    }
}

impl<'d, SRC, DST> DmaStartError<'d, SRC, DST> {
    /// The reason the transfer could not be started.
    pub const fn error(&self) -> DmaError {
        self.error
    }

    /// Recover the driver, claimed channel, and buffers.
    pub fn into_parts(self) -> (DmaError, DmaDriver<'d, Dma0>, DmaChannel, SRC, DST) {
        (self.error, self.driver, self.channel, self.src, self.dst)
    }
}

/// Resources returned when a started DMA transfer times out while waiting.
#[instability::unstable]
pub struct DmaWaitError<'d, SRC, DST> {
    error: DmaError,
    driver: DmaDriver<'d, Dma0>,
    channel: DmaChannel,
    src: SRC,
    dst: DST,
}

impl<SRC, DST> core::fmt::Debug for DmaWaitError<'_, SRC, DST> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("DmaWaitError").field("error", &self.error).finish_non_exhaustive()
    }
}

impl<'d, SRC, DST> DmaWaitError<'d, SRC, DST> {
    /// The reason waiting failed.
    pub const fn error(&self) -> DmaError {
        self.error
    }

    /// Recover the driver, claimed channel, and buffers after the channel has been
    /// quiesced and the destination cache maintenance has run.
    pub fn into_parts(self) -> (DmaError, DmaDriver<'d, Dma0>, DmaChannel, SRC, DST) {
        (self.error, self.driver, self.channel, self.src, self.dst)
    }
}

impl<SRC, DST> Drop for Transfer<'_, SRC, DST> {
    /// Abort the channel so the engine stops reading/writing the owned buffers before
    /// they are dropped — the use-after-free guard for an early-dropped transfer.
    ///
    /// Uses `quiesce_channel`: halt, then drain the `active` (FIFO/bus) bit, then
    /// clear `ch_enable` — clearing `ch_enable` mid-burst (the old behaviour) could
    /// let an outstanding bus write land after the buffer is freed.
    fn drop(&mut self) {
        quiesce_channel(&mut self.driver, self.channel.logical);
    }
}

// ── Channel teardown helpers (cancel-then-quiesce) ───────────────────────────

/// Halt a DMA channel, drain its in-flight beats (the `active`/FIFO-busy bit), then
/// clear `ch_enable`. This is the DesignWare quiesce sequence — clearing `ch_enable`
/// mid-burst (the original `Drop`) could let an outstanding bus write land after the
/// owned buffer is freed, a use-after-free window on the non-coherent core. The poll
/// is bounded by `DMA_WAIT_LOOPS`.
///
/// Factored as a free function over `&mut DmaDriver` so both the mem-to-mem
/// [`Transfer`] and the peripheral [`PeripheralTransfer`] share one teardown path.
fn quiesce_channel<T: DmaInstance>(driver: &mut DmaDriver<'_, T>, channel: u8) {
    driver.halt_channel_raw(channel);
    let mut n = DMA_WAIT_LOOPS;
    while driver.channel_active_raw(channel) {
        if n == 0 {
            break;
        }
        n -= 1;
        core::hint::spin_loop();
    }
    driver.disable_channel_raw(channel);
}

/// Cancel-then-quiesce for a peripheral-paced transfer: clear the peripheral's
/// DMA-enable **first** (so it stops asserting DMA requests), then
/// `quiesce_channel` (halt → drain `active` → disable). esp-hal's
/// cancel-then-quiesce (`spi/master/dma.rs:1608`) disables the peripheral DMA
/// first; doing it the other way leaves a spurious request latched.
#[cfg(feature = "chip-ws63")]
fn cancel_then_quiesce(peri_dis: &mut PeriDmaCtl, driver: &mut DmaDriver<'_, Dma0>, channel: u8) {
    peri_dis.disable();
    quiesce_channel(driver, channel);
}

// ── Peripheral-paced DMA (hisi-riscv-hal#6 / 0.5.1) ──────────────────────────

/// Errors from DMA transfer setup or completion.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum DmaError {
    /// The buffer exceeded the 12-bit `trans_size` field (4095 beats per single-block
    /// transfer). The caller must chunk it. (Multi-block LLI chunking is a follow-up;
    /// for 0.5.1 a single-block cap is correct and not a silent truncation.)
    TransferTooLarge,
    /// The channel did not auto-clear its enable bit within `DMA_WAIT_LOOPS` (a
    /// wedged transfer — e.g. a peripheral whose DMA-enable was never set, or a
    /// handshake mismatch). The guard quiesced the channel before returning, so the
    /// buffer is safe, but the transfer did not complete.
    Timeout,
}

/// The DMA beat (frame) width for a peripheral transfer — derived from the driver's
/// frame size (UART = byte, SPI = byte for `DataBits ≤ 8` else halfword), never the
/// mem-to-mem default `Width32`.
#[cfg(feature = "chip-ws63")]
#[instability::unstable]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DmaFrame {
    /// 8-bit beat (`TransferWidth::Width8`).
    Byte,
    /// 16-bit beat (`TransferWidth::Width16`).
    HalfWord,
}

#[cfg(feature = "chip-ws63")]
impl DmaFrame {
    /// The `TransferWidth` to program into the channel's source/destination width
    /// field for this frame size.
    pub const fn width(self) -> TransferWidth {
        match self {
            DmaFrame::Byte => TransferWidth::Width8,
            DmaFrame::HalfWord => TransferWidth::Width16,
        }
    }
}

/// Which kind of peripheral a [`PeriDmaCtl`] controls — selects the register layout
/// for clearing the peripheral-side DMA-enable on teardown.
#[cfg(feature = "chip-ws63")]
#[instability::unstable]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PeriKind {
    /// SPI (SSI v151): `spi_dcr` @ base+0x18, `tdmae`/`rdmae`.
    Spi,
    /// UART (DW 16550 v151): `uart_parameter` @ base+0x60, `dma_mode` bit 11.
    Uart,
}

/// A POD handle describing how to clear a peripheral's DMA-enable on teardown, so the
/// [`PeripheralTransfer`] guard can stop the peripheral asserting DMA requests
/// without being generic over the driver. Built by the driver's `with_dma`.
#[cfg(feature = "chip-ws63")]
#[instability::unstable]
#[derive(Debug, Clone, Copy)]
#[cfg_attr(not(target_arch = "riscv32"), allow(dead_code))]
pub struct PeriDmaCtl {
    /// The peripheral's register-block base (e.g. `0x4402_0060`-style — the exact
    /// data-register address is passed to `start_*` separately; this is the base the
    /// DMA-enable register lives at).
    base: usize,
    kind: PeriKind,
    dir: DmaDirection,
}

#[cfg(feature = "chip-ws63")]
impl PeriDmaCtl {
    /// Build a teardown handle for a peripheral at `base` with the given kind/dir.
    #[instability::unstable]
    pub const fn new(base: usize, kind: PeriKind, dir: DmaDirection) -> Self {
        Self { base, kind, dir }
    }

    /// Clear the peripheral's DMA-enable so it stops asserting DMA requests. Called
    /// by `cancel_then_quiesce` **before** halting the channel.
    ///
    /// On the host (non-riscv) build this is a no-op so unit tests don't touch MMIO;
    /// the teardown *order* is exercised by the `cancel_then_quiesce` host test.
    #[cfg(target_arch = "riscv32")]
    fn disable(&mut self) {
        match self.kind {
            PeriKind::Spi => {
                if let Some(r) = spi_regs_from_base(self.base) {
                    match self.dir {
                        DmaDirection::Tx => r.spi_dcr().modify(|_, w| w.tdmae().clear_bit()),
                        DmaDirection::Rx => r.spi_dcr().modify(|_, w| w.rdmae().clear_bit()),
                    };
                }
            }
            // UART DMA is paced by FIFO_CTL trigger levels; UART_PARAMETER.dma_mode is
            // read-only in the PAC, so there is no peripheral DMA-enable bit to clear.
            PeriKind::Uart => {}
        }
    }

    #[cfg(not(target_arch = "riscv32"))]
    fn disable(&mut self) {}
}

#[cfg(all(feature = "chip-ws63", target_arch = "riscv32"))]
fn spi_regs_from_base(base: usize) -> Option<&'static crate::soc::pac::spi0::RegisterBlock> {
    if base == crate::peripherals::Spi0::ptr() as usize {
        Some(unsafe { &*crate::peripherals::Spi0::ptr() })
    } else if base == crate::peripherals::Spi1::ptr() as usize {
        Some(unsafe { &*crate::peripherals::Spi1::ptr() })
    } else {
        None
    }
}

// ── Typed DMA channel tokens (runtime-claimed) ───────────────────────────────

/// A claimed Dma0 (MDMA) channel — a token proving the caller owns logical channel
/// `0..=3` for an in-flight transfer. At most one transfer may use a channel at a
/// time; the channel is released when the token (or its [`PeripheralTransfer`] /
/// [`Transfer`]) drops.
///
/// Tokens come from [`DmaDriver::split_channels`], which claims all four channels at
/// once (all-or-nothing). The claim is a runtime bitmask (no-atomics-safe via
/// `portable-atomic`'s critical-section polyfill); a second `split_channels` while
/// any channel is outstanding returns `None`.
pub struct DmaChannel {
    logical: u8,
}

impl DmaChannel {
    /// The logical channel number (0-3 on Dma0).
    pub const fn logical(&self) -> u8 {
        self.logical
    }
}

impl Drop for DmaChannel {
    fn drop(&mut self) {
        DMA0_CHANNELS_CLAIMED.fetch_and(!(1 << self.logical), portable_atomic::Ordering::AcqRel);
    }
}

/// The four Dma0 channel tokens, returned by [`DmaDriver::split_channels`].
pub struct DmaChannels {
    /// Logical channel 0.
    pub ch0: DmaChannel,
    /// Logical channel 1.
    pub ch1: DmaChannel,
    /// Logical channel 2.
    pub ch2: DmaChannel,
    /// Logical channel 3.
    pub ch3: DmaChannel,
}

/// Bitmask of claimed Dma0 channels (bit `n` = logical channel `n`). A channel token
/// claims its bit on `split_channels` and releases it on `Drop`.
static DMA0_CHANNELS_CLAIMED: portable_atomic::AtomicU8 = portable_atomic::AtomicU8::new(0);

impl DmaDriver<'_, Dma0> {
    /// Claim all four Dma0 channels at once, returning owned tokens. Returns `None`
    /// if any channel is already outstanding (call it once, after the previous
    /// transfer's guards have dropped). All-or-nothing so two callers can't race on
    /// a partial set.
    pub fn split_channels(&self) -> Option<DmaChannels> {
        match DMA0_CHANNELS_CLAIMED.compare_exchange(
            0,
            0b1111,
            portable_atomic::Ordering::AcqRel,
            portable_atomic::Ordering::Acquire,
        ) {
            Ok(_) => Some(DmaChannels {
                ch0: DmaChannel { logical: 0 },
                ch1: DmaChannel { logical: 1 },
                ch2: DmaChannel { logical: 2 },
                ch3: DmaChannel { logical: 3 },
            }),
            Err(_) => None,
        }
    }

    /// Enable a claimed DMA channel.
    pub fn enable_channel(&mut self, channel: &DmaChannel) {
        self.enable_channel_raw(channel.logical);
    }

    /// Disable a claimed DMA channel.
    pub fn disable_channel(&mut self, channel: &DmaChannel) {
        self.disable_channel_raw(channel.logical);
    }

    /// Check whether a claimed DMA channel is enabled.
    pub fn channel_enabled(&self, channel: &DmaChannel) -> bool {
        self.channel_enabled_raw(channel.logical)
    }

    /// Check whether a claimed DMA channel has data in its FIFO or bus pipeline.
    pub fn channel_active(&self, channel: &DmaChannel) -> bool {
        self.channel_active_raw(channel.logical)
    }

    /// Halt a claimed DMA channel.
    pub fn halt_channel(&mut self, channel: &DmaChannel) {
        self.halt_channel_raw(channel.logical);
    }

    /// Resume a halted claimed DMA channel.
    pub fn resume_channel(&mut self, channel: &DmaChannel) {
        self.resume_channel_raw(channel.logical);
    }

    /// Issue a software burst request for a claimed channel.
    pub fn burst_request(&mut self, channel: &DmaChannel) {
        self.burst_request_raw(channel.logical);
    }

    /// Issue a software single request for a claimed channel.
    pub fn single_request(&mut self, channel: &DmaChannel) {
        self.single_request_raw(channel.logical);
    }

    /// Clear transfer complete interrupt for a claimed channel.
    pub fn clear_transfer_interrupt(&mut self, channel: &DmaChannel) {
        self.clear_transfer_interrupt_raw(channel.logical);
    }

    /// Clear error interrupt for a claimed channel.
    pub fn clear_error_interrupt(&mut self, channel: &DmaChannel) {
        self.clear_error_interrupt_raw(channel.logical);
    }
}

// ── Peripheral-paced transfer guard ──────────────────────────────────────────

#[cfg(feature = "chip-ws63")]
impl<'d> DmaDriver<'d, Dma0> {
    /// Launch a **memory → peripheral** (TX) transfer on `channel` that OWNS `src`
    /// for its whole lifetime, returning a [`PeripheralTransfer`] guard you
    /// [`wait`](PeripheralTransfer::wait) to reclaim the driver + channel + buffer.
    ///
    /// Cleans the source cache before launch (non-coherent core) and pins
    /// `peri_data_addr` (a peripheral data register) as the fixed destination. The
    /// beat width is `frame` (not the mem-to-mem `Word = u32`). The peripheral-side
    /// DMA-enable is the caller's responsibility until P1's `with_dma` wires it
    /// (vendor order: configure channel → write watermark → clean → start → set
    /// peripheral DMA-enable); this guard tears it down on `Drop` via `peri_dis`.
    ///
    /// Returns [`DmaError::TransferTooLarge`] if `src` exceeds 4095 beats (a
    /// single-block DMA `trans_size` cap — chunk the buffer). On `Err` the driver,
    /// channel, and buffer are dropped (the channel claim is released).
    #[instability::unstable]
    pub fn start_mem_to_peripheral<SRC>(
        mut self,
        channel: DmaChannel,
        src: SRC,
        peri_data_addr: usize,
        peri: DmaPeripheral,
        frame: DmaFrame,
        peri_dis: PeriDmaCtl,
    ) -> Result<PeripheralTransfer<'d, SRC>, DmaError>
    where
        SRC: ReadBuffer<Word = u8>,
    {
        let ch = channel.logical;
        // SAFETY: `src` is moved into the returned guard and held until wait(), so the
        // pointer stays valid for the whole transfer.
        let (src_ptr, beats) = unsafe { src.read_buffer() };
        if beats > 0xFFF {
            // Drop the consumed driver (ZST), channel (releases claim), and buffer.
            drop(channel);
            return Err(DmaError::TransferTooLarge);
        }
        let bytes = beats;

        // Clean the source so the DMA master reads CPU-written data (non-coherent core).
        // The peripheral DATA register is uncached MMIO — never cache-maintain it.
        unsafe {
            crate::cache::clean_range(src_ptr as usize, bytes);
        }

        let size = DmaTransferSize::from_beats(beats).expect("beats checked above");
        let cfg = DmaChannelConfig::default().mem_to_peripheral(peri).with_width(frame).with_transfer_int(true);
        self.configure_channel_raw(ch, src_ptr as u32, peri_data_addr as u32, size, &cfg);

        Ok(PeripheralTransfer {
            driver: self,
            channel,
            buf: src,
            dir: DmaDirection::Tx,
            mem_addr: src_ptr as usize,
            bytes,
            peri_dis,
        })
    }

    /// Launch a **peripheral → memory** (RX) transfer on `channel` that OWNS `dst`,
    /// returning a [`PeripheralTransfer`] guard. The destination cache is invalidated
    /// on [`wait`](PeripheralTransfer::wait) AFTER completion. See
    /// [`start_mem_to_peripheral`] for the lifetime / cap / teardown contract.
    #[instability::unstable]
    pub fn start_peripheral_to_mem<DST>(
        mut self,
        channel: DmaChannel,
        mut dst: DST,
        peri_data_addr: usize,
        peri: DmaPeripheral,
        frame: DmaFrame,
        peri_dis: PeriDmaCtl,
    ) -> Result<PeripheralTransfer<'d, DST>, DmaError>
    where
        DST: WriteBuffer<Word = u8>,
    {
        let ch = channel.logical;
        // SAFETY: `dst` is moved into the returned guard and held until wait().
        let (dst_ptr, beats) = unsafe { dst.write_buffer() };
        if beats > 0xFFF {
            drop(channel);
            return Err(DmaError::TransferTooLarge);
        }
        let bytes = beats;

        // No clean before launch — the CPU hasn't written the RX destination; the DMA
        // will. invalidate_range runs on wait() AFTER completion. The peripheral DATA
        // register is uncached MMIO.
        let size = DmaTransferSize::from_beats(beats).expect("beats checked above");
        let cfg = DmaChannelConfig::default().peripheral_to_mem(peri).with_width(frame).with_transfer_int(true);
        self.configure_channel_raw(ch, peri_data_addr as u32, dst_ptr as u32, size, &cfg);

        Ok(PeripheralTransfer {
            driver: self,
            channel,
            buf: dst,
            dir: DmaDirection::Rx,
            mem_addr: dst_ptr as usize,
            bytes,
            peri_dis,
        })
    }
}

/// An in-flight peripheral-paced DMA transfer that **owns the driver, the channel
/// token, and the single memory buffer**. Reclaim them with
/// [`wait`](Self::wait); dropping the guard early runs cancel-then-quiesce: clear
/// the peripheral DMA-enable, halt, drain `active`, disable — so neither the channel
/// nor the peripheral keeps touching the buffer before it is freed.
///
/// Owning the buffer (by value, via `embedded_dma`) is what makes a use-after-free of
/// an active DMA region unrepresentable in safe code: the buffer cannot be touched,
/// moved, or freed while the DMA reads/writes it.
#[cfg(feature = "chip-ws63")]
#[instability::unstable]
pub struct PeripheralTransfer<'d, BUF> {
    driver: DmaDriver<'d, Dma0>,
    channel: DmaChannel,
    buf: BUF,
    dir: DmaDirection,
    mem_addr: usize,
    bytes: usize,
    peri_dis: PeriDmaCtl,
}

#[cfg(feature = "chip-ws63")]
impl<'d, BUF> PeripheralTransfer<'d, BUF> {
    /// True once the channel has auto-cleared its enable bit (single-block done).
    #[instability::unstable]
    pub fn is_done(&self) -> bool {
        !self.driver.channel_enabled_raw(self.channel.logical)
    }

    /// Block (bounded) until the transfer completes, do the direction-correct cache
    /// maintenance, and return the driver + channel + buffer. Consumes the guard, so
    /// no teardown `Drop` runs on the success path.
    ///
    /// On `DMA_WAIT_LOOPS` exhaustion the channel is cancel-then-quiesced first, then
    /// [`Err(DmaError::Timeout)`](DmaError::Timeout) is returned (the driver, channel,
    /// and buffer are dropped — the channel claim is released, the buffer is safe
    /// because the engine was stopped). A wedged transfer is therefore *observable*,
    /// not silently "done" — the UAF-on-timeout hole the mem-to-mem guard had.
    #[instability::unstable]
    pub fn wait(self) -> Result<(DmaDriver<'d, Dma0>, DmaChannel, BUF), DmaError> {
        // Skip the abort-on-drop: the transfer is completing normally.
        let mut this = ManuallyDrop::new(self);
        let ch = this.channel.logical;
        let mut n = DMA_WAIT_LOOPS;
        while this.driver.channel_enabled_raw(ch) {
            n -= 1;
            if n == 0 {
                break;
            }
            core::hint::spin_loop();
        }

        if this.driver.channel_enabled_raw(ch) {
            // Timed out: cancel-then-quiesce so the engine is stopped before any
            // buffer/cache teardown, then invalidate the RX destination (the DMA may
            // have written partial data) before the buffer drops. The two `&mut`
            // field borrows are sequential (not simultaneous) so they reborrow
            // `&mut *this` one at a time.
            this.peri_dis.disable();
            quiesce_channel(&mut this.driver, ch);
            if this.dir == DmaDirection::Rx {
                unsafe {
                    crate::cache::invalidate_range(this.mem_addr, this.bytes);
                }
            }
            // Drop driver + channel (its Drop releases the claim bitmask) + buf.
            // `into_inner` unwraps the ManuallyDrop so the inner values actually drop
            // (otherwise the channel claim would leak on the timeout path).
            drop(ManuallyDrop::into_inner(this));
            return Err(DmaError::Timeout);
        }

        // Completed: invalidate the RX destination so the CPU re-reads DMA-written data.
        // (TX needs no invalidate — the CPU didn't write the peripheral, and the source
        // was cleaned before launch.)
        if this.dir == DmaDirection::Rx {
            unsafe {
                crate::cache::invalidate_range(this.mem_addr, this.bytes);
            }
        }

        // SAFETY: `this` is ManuallyDrop (its Drop never runs) and each field is read
        // exactly once, so there is no double-read or double-drop.
        unsafe {
            let driver = core::ptr::read(&this.driver);
            let channel = core::ptr::read(&this.channel);
            let buf = core::ptr::read(&this.buf);
            Ok((driver, channel, buf))
        }
    }
}

#[cfg(feature = "chip-ws63")]
impl<BUF> Drop for PeripheralTransfer<'_, BUF> {
    /// Cancel-then-quiesce: clear the peripheral DMA-enable, halt, drain `active`,
    /// disable — so neither the channel nor the peripheral keeps asserting requests
    /// against a buffer that's about to be freed.
    fn drop(&mut self) {
        cancel_then_quiesce(&mut self.peri_dis, &mut self.driver, self.channel.logical);
    }
}

// ── Channel-config width/int helpers (typed-config on the config carrier) ────

#[cfg(feature = "chip-ws63")]
impl DmaChannelConfig {
    /// Set both source and destination transfer width (peripheral and memory sides
    /// share the frame width for a peripheral-paced transfer).
    #[instability::unstable]
    pub fn with_width(mut self, frame: DmaFrame) -> Self {
        let w = frame.width();
        self.src_width = w;
        self.dst_width = w;
        self
    }

    /// Enable the transfer-complete interrupt (required for the async `.await` path
    /// and so `dmac_int_st` reflects per-channel completion).
    #[instability::unstable]
    pub fn with_transfer_int(mut self, on: bool) -> Self {
        self.transfer_int = on;
        self
    }
}

#[cfg(all(test, not(target_arch = "riscv32")))]
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
        // (The PAC's Spi1 instance may be QSPI0_2CS = 9/10 — silicon-unverified;
        // the Qspi02cs variant is deferred to P2.)
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

    // ── P0 peripheral-DMA core (host-verifiable; silicon proof is the HIL suite) ──

    #[cfg(feature = "chip-ws63")]
    #[test]
    fn test_peripheral_config_tx_full() {
        // HOST-CFGWORD-TX + HOST-PERI-CONFIG-INT-ENABLED + HOST-WIDTH-FROM-DATABITS:
        // a TX peripheral config sets MemToPeripheral, the dst handshaking ID, holds
        // the peripheral address fixed, uses the frame width (not Width32), and enables
        // the transfer-complete interrupt (so dmac_int_st reflects completion).
        let cfg = DmaChannelConfig::default()
            .mem_to_peripheral(DmaPeripheral::Spi0Tx)
            .with_width(DmaFrame::Byte)
            .with_transfer_int(true);
        assert_eq!(cfg.flow_control, FlowControl::MemToPeripheral);
        assert_eq!(cfg.dst_peripheral, 7);
        assert!(!cfg.dst_inc);
        assert!(cfg.src_inc, "memory side still increments");
        assert_eq!(cfg.src_width, TransferWidth::Width8);
        assert_eq!(cfg.dst_width, TransferWidth::Width8);
        assert!(cfg.transfer_int);
    }

    #[cfg(feature = "chip-ws63")]
    #[test]
    fn test_peripheral_config_rx_full() {
        // HOST-CFGWORD-RX: PeripheralToMem, src handshaking ID, src fixed, frame width.
        let cfg = DmaChannelConfig::default().peripheral_to_mem(DmaPeripheral::Spi0Rx).with_width(DmaFrame::HalfWord);
        assert_eq!(cfg.flow_control, FlowControl::PeripheralToMem);
        assert_eq!(cfg.src_peripheral, 8);
        assert!(!cfg.src_inc);
        assert!(cfg.dst_inc);
        assert_eq!(cfg.src_width, TransferWidth::Width16);
        assert_eq!(cfg.dst_width, TransferWidth::Width16);
    }

    #[cfg(feature = "chip-ws63")]
    #[test]
    fn test_dma_frame_width_mapping() {
        // HOST-WIDTH-FROM-DATABITS: frame → TransferWidth, never the mem-to-mem Width32.
        assert_eq!(DmaFrame::Byte.width(), TransferWidth::Width8);
        assert_eq!(DmaFrame::HalfWord.width(), TransferWidth::Width16);
    }

    #[cfg(feature = "chip-ws63")]
    #[test]
    fn test_start_mem_to_peripheral_rejects_too_large() {
        // HOST-CHUNK-4095: a >4095-beat buffer is rejected with TransferTooLarge
        // (not silently truncated — the 12-bit trans_size field hazard). The check
        // runs before configure_channel, so this is host-safe (no MMIO touched).
        let dma: DmaDriver<'static, Dma0> = DmaDriver::new_for_test();
        // Construct a channel token directly (in-module, so the private field is
        // accessible) — avoids the global split_channels CAS so this test is
        // thread-safe alongside split_channels tests.
        let ch = DmaChannel { logical: 0 };
        // A 'static buffer (the 'static driver requires SRC: 'static).
        static BUF: [u8; 5000] = [0; 5000]; // 5000 > 0xFFF (4095)
        let peri_dis = PeriDmaCtl::new(0x4402_0000, PeriKind::Spi, DmaDirection::Tx);
        let r = dma.start_mem_to_peripheral(ch, &BUF[..], 0x4402_0060, DmaPeripheral::Spi0Tx, DmaFrame::Byte, peri_dis);
        assert!(matches!(r, Err(DmaError::TransferTooLarge)), "expected TransferTooLarge");
    }

    #[cfg(feature = "chip-ws63")]
    #[test]
    fn test_split_channels_once() {
        // HOST-DOUBLE-SPLIT: split_channels claims all 4 at once; a second split while
        // any channel is outstanding returns None. Serialized with a mutex because the
        // claim is a process-global static (tests run in parallel by default).
        use std::sync::Mutex;
        static LOCK: Mutex<()> = Mutex::new(());
        let _g = LOCK.lock().unwrap();
        // Reset any leftover claim from a previously-panicked test so this test is
        // independent (a poisoned/dropped token from another test could leave a bit set).
        DMA0_CHANNELS_CLAIMED.store(0, portable_atomic::Ordering::Release);

        let dma: DmaDriver<'static, Dma0> = DmaDriver::new_for_test();
        let chs = dma.split_channels();
        assert!(chs.is_some(), "first split must succeed");
        // While all four are outstanding, a second split fails.
        assert!(dma.split_channels().is_none(), "second split must fail");
        drop(chs); // releases all four bits
        assert!(dma.split_channels().is_some(), "split after drop must succeed");
    }
}

// ── Property-based fuzz tests ──────────────────────────────────

#[cfg(all(test, not(target_arch = "riscv32")))]
mod proptests {
    use super::{BurstSize, FlowControl, TransferWidth, physical_channel_index};
    use proptest::prelude::*;

    // Re-derive `configure_channel`'s control-word encoder (same masks/shifts as
    // the driver — pure arithmetic, no MMIO). Mirrors dma.rs lines 244-261.
    fn control_word(
        transfer_size: u16,
        src_burst: BurstSize,
        dst_burst: BurstSize,
        src_width: TransferWidth,
        dst_width: TransferWidth,
        src_inc: bool,
        dst_inc: bool,
        transfer_int: bool,
    ) -> u32 {
        let mut control: u32 = 0;
        control |= (transfer_size as u32) & 0xFFF; // trans_size [0:11]
        control |= ((src_burst as u32) & 0x07) << 12; // s_bsize [12:14]
        control |= ((dst_burst as u32) & 0x07) << 15; // d_bsize [15:17]
        control |= ((src_width as u32) & 0x07) << 18; // s_width [18:20]
        control |= ((dst_width as u32) & 0x07) << 21; // d_width [21:23]
        if src_inc {
            control |= 1 << 26;
        }
        if dst_inc {
            control |= 1 << 27;
        }
        if transfer_int {
            control |= 1 << 31;
        }
        control
    }

    // Re-derive `configure_channel`'s channel-config-word encoder. Mirrors
    // dma.rs lines 268-281.
    fn ch_cfg_word(
        src_peripheral: u8,
        dst_peripheral: u8,
        flow_control: FlowControl,
        error_int: bool,
        transfer_int: bool,
        bus_lock: bool,
    ) -> u32 {
        let mut ch_cfg: u32 = 0;
        ch_cfg |= 0x01; // chn_en
        ch_cfg |= ((src_peripheral as u32) & 0x0F) << 1; // s_peripheral [1:4]
        ch_cfg |= ((dst_peripheral as u32) & 0x0F) << 5; // d_peripheral [5:8]
        ch_cfg |= ((flow_control as u32) & 0x07) << 9; // flow_ctl [9:11]
        if error_int {
            ch_cfg |= 1 << 12; // int_en
        }
        if transfer_int {
            ch_cfg |= 1 << 13; // int_tc
        }
        if bus_lock {
            ch_cfg |= 1 << 14; // lock
        }
        ch_cfg
    }

    fn burst(v: u8) -> BurstSize {
        match v & 0x07 {
            0 => BurstSize::Beats1,
            1 => BurstSize::Beats4,
            2 => BurstSize::Beats8,
            3 => BurstSize::Beats16,
            4 => BurstSize::Beats32,
            5 => BurstSize::Beats64,
            6 => BurstSize::Beats128,
            _ => BurstSize::Beats256,
        }
    }

    fn width(v: u8) -> TransferWidth {
        match v % 3 {
            0 => TransferWidth::Width8,
            1 => TransferWidth::Width16,
            _ => TransferWidth::Width32,
        }
    }

    fn flow(v: u8) -> FlowControl {
        match v & 0x03 {
            0 => FlowControl::MemToMem,
            1 => FlowControl::MemToPeripheral,
            2 => FlowControl::PeripheralToMem,
            _ => FlowControl::PeripheralToPeripheral,
        }
    }

    proptest! {
        // ── physical_channel_index (logical→physical translation + range gate) ──

        /// Fuzz: any in-range logical channel maps to a physical index 0..=3.
        #[test]
        fn physical_index_in_range(base in 0u8..=200, off in 0u8..4) {
            let idx = physical_channel_index(base, base + off);
            prop_assert_eq!(idx, off as usize);
            prop_assert!(idx < 4);
        }

        /// Fuzz: translation is monotonic across the controller's 4 logical channels.
        #[test]
        fn physical_index_monotonic(base in 0u8..=200) {
            let mut prev = None;
            for off in 0u8..4 {
                let idx = physical_channel_index(base, base + off);
                if let Some(p) = prev {
                    prop_assert!(idx > p);
                }
                prev = Some(idx);
            }
        }

        /// Fuzz: a channel below the controller's base must panic, never silently
        /// alias physical channel 0 (the guard's lower bound).
        #[test]
        fn physical_index_below_base_panics(base in 1u8..=200, under in 1u8..=100) {
            let under = under.min(base);
            let ch = base - under;
            let r = std::panic::catch_unwind(|| physical_channel_index(base, ch));
            prop_assert!(r.is_err());
        }

        /// Fuzz: a channel at or above base+4 must panic (the guard's upper bound).
        #[test]
        fn physical_index_above_range_panics(base in 0u8..=200, over in 0u8..=50) {
            let ch = (base as u16 + 4 + over as u16).min(255) as u8;
            // Guard against wrap making ch land back in [base, base+4).
            prop_assume!(ch >= base.saturating_add(4) || ch < base);
            let r = std::panic::catch_unwind(|| physical_channel_index(base, ch));
            prop_assert!(r.is_err());
        }

        // ── control-word bit encoder ──────────────────────────────────────────

        /// Fuzz: the control word never sets a bit outside its documented fields.
        /// Reserved positions [24:25] (masters), [28:30] (prot/reserved) must be 0.
        #[test]
        fn control_word_no_stray_bits(
            ts in any::<u16>(),
            sb in any::<u8>(), db in any::<u8>(),
            sw in any::<u8>(), dw in any::<u8>(),
            si in any::<bool>(), di in any::<bool>(), ti in any::<bool>(),
        ) {
            let c = control_word(ts, burst(sb), burst(db), width(sw), width(dw), si, di, ti);
            // Defined bits: [0:11] trans, [12:14] sb, [15:17] db, [18:20] sw,
            // [21:23] dw, 26 s_inc, 27 d_inc, 31 t_int. Everything else is reserved 0.
            const DEFINED: u32 = 0x00FF_FFFF | (1 << 26) | (1 << 27) | (1 << 31);
            prop_assert_eq!(c & !DEFINED, 0, "stray bits in control word {:#010x}", c);
        }

        /// Fuzz: trans_size is a 12-bit field — the encoder truncates a u16 > 0xFFF
        /// (mask, not overflow). Verify the masked low 12 bits round-trip exactly.
        #[test]
        fn control_word_trans_size_field(ts in any::<u16>()) {
            let c = control_word(ts, BurstSize::Beats1, BurstSize::Beats1,
                TransferWidth::Width8, TransferWidth::Width8, false, false, false);
            prop_assert_eq!(c & 0xFFF, (ts as u32) & 0xFFF);
            prop_assert!((c & 0xFFF) <= 0xFFF);
        }

        /// Fuzz: every enum field round-trips out of the control word at its shift.
        #[test]
        fn control_word_fields_round_trip(
            sb in 0u8..8, db in 0u8..8, sw in 0u8..3, dw in 0u8..3,
        ) {
            let (sbe, dbe, swe, dwe) = (burst(sb), burst(db), width(sw), width(dw));
            let c = control_word(0, sbe, dbe, swe, dwe, false, false, false);
            prop_assert_eq!((c >> 12) & 0x07, sbe as u32);
            prop_assert_eq!((c >> 15) & 0x07, dbe as u32);
            prop_assert_eq!((c >> 18) & 0x07, swe as u32);
            prop_assert_eq!((c >> 21) & 0x07, dwe as u32);
        }

        /// Fuzz: each boolean control flag lands in exactly its documented bit.
        #[test]
        fn control_word_flag_bits(si in any::<bool>(), di in any::<bool>(), ti in any::<bool>()) {
            let c = control_word(0, BurstSize::Beats1, BurstSize::Beats1,
                TransferWidth::Width8, TransferWidth::Width8, si, di, ti);
            prop_assert_eq!((c >> 26) & 1 == 1, si);
            prop_assert_eq!((c >> 27) & 1 == 1, di);
            prop_assert_eq!((c >> 31) & 1 == 1, ti);
        }

        // ── channel-config-word bit encoder ───────────────────────────────────

        /// Fuzz: ch_cfg always has chn_en set and no bits above the lock bit (14).
        #[test]
        fn ch_cfg_word_no_stray_bits(
            sp in any::<u8>(), dp in any::<u8>(), fc in any::<u8>(),
            ei in any::<bool>(), ti in any::<bool>(), bl in any::<bool>(),
        ) {
            let w = ch_cfg_word(sp, dp, flow(fc), ei, ti, bl);
            prop_assert_eq!(w & 0x01, 1, "chn_en must always be set");
            // Defined: bit0 en, [1:4] s_peri, [5:8] d_peri, [9:11] flow, 12 int_en,
            // 13 int_tc, 14 lock → low 15 bits. Nothing above bit 14.
            prop_assert_eq!(w & !0x7FFF, 0, "stray bits in ch_cfg {:#010x}", w);
        }

        /// Fuzz: a 4-bit peripheral field truncates ids > 0x0F — verify both
        /// peripheral selects round-trip their low nibble and never collide.
        #[test]
        fn ch_cfg_word_peripheral_fields(sp in any::<u8>(), dp in any::<u8>()) {
            let w = ch_cfg_word(sp, dp, FlowControl::MemToMem, false, false, false);
            prop_assert_eq!((w >> 1) & 0x0F, (sp as u32) & 0x0F);
            prop_assert_eq!((w >> 5) & 0x0F, (dp as u32) & 0x0F);
        }

        /// Fuzz: flow control occupies bits [9:11] and round-trips its discriminant.
        #[test]
        fn ch_cfg_word_flow_field(fc in 0u8..4) {
            let f = flow(fc);
            let w = ch_cfg_word(0, 0, f, false, false, false);
            prop_assert_eq!((w >> 9) & 0x07, f as u32);
        }

        /// Fuzz: int_en / int_tc / lock flags map to bits 12 / 13 / 14 exactly.
        #[test]
        fn ch_cfg_word_flag_bits(ei in any::<bool>(), ti in any::<bool>(), bl in any::<bool>()) {
            let w = ch_cfg_word(0, 0, FlowControl::MemToMem, ei, ti, bl);
            prop_assert_eq!((w >> 12) & 1 == 1, ei);
            prop_assert_eq!((w >> 13) & 1 == 1, ti);
            prop_assert_eq!((w >> 14) & 1 == 1, bl);
        }
    }
}

// ── Async DMA completion (bespoke; DMA_INT = IRQ 59) ────────────────────────
#[cfg(all(feature = "chip-ws63", feature = "async", feature = "unstable"))]
mod asynch_impl {
    use super::{Dma0, DmaDriver, DmaInstance};
    use crate::asynch::IrqSignal;
    use crate::interrupt::{self, Interrupt};
    use core::future::Future;
    use core::pin::Pin;
    use core::task::{Context, Poll};

    // Per-physical-channel signal (Dma0 has 4 physical channels 0..=3). Replaces the
    // earlier single global `DMA_SIGNAL` so two concurrent transfers (e.g. SPI
    // full-duplex TX + RX on ch0 + ch1) don't false-wake each other.
    static DMA_SIGNAL: [IrqSignal; 4] = [IrqSignal::new(), IrqSignal::new(), IrqSignal::new(), IrqSignal::new()];

    /// DMA trap hook (IRQ 59): **demux per channel** — for each completed channel,
    /// signal its waker AND clear its `dmac_int_clr` done bit **in-ISR** (a
    /// level-triggered done line would re-fire forever otherwise), then clear the
    /// ECLIC pending bit. Reads the raw transfer-done mask (`dmac_ori_int_st[7:0]`).
    pub fn on_interrupt() {
        // SAFETY: Dma0::ptr() is a static physical MMIO address, always valid.
        let r = unsafe { &*Dma0::ptr() };
        let done = (r.dmac_ori_int_st().read().bits() & 0xFF) as u8;
        let mut clr = 0u32;
        for ch in 0..4u8 {
            if done & (1 << ch) != 0 {
                DMA_SIGNAL[ch as usize].signal();
                clr |= 1 << ch;
            }
        }
        if clr != 0 {
            // SAFETY: writing the per-channel done-clear bits (bit n = channel n).
            unsafe { r.dmac_int_clr().write(|w| w.bits(clr)) };
        }
        interrupt::clear_pending(Interrupt::DMA_INT);
    }

    /// Named device.x handler (DMA_INT = IRQ 59): the rt routes the DMA IRQ here by
    /// number, so an async DMA app needs no `mcause` trap.
    #[unsafe(no_mangle)]
    extern "C" fn DMA_INT() {
        on_interrupt();
    }

    /// A future that resolves once `DMA_SIGNAL[ch]` fires (the ISR signals it when
    /// channel `ch`'s transfer-done bit asserts).
    struct DmaDoneFuture {
        ch: u8,
    }
    impl Future for DmaDoneFuture {
        type Output = ();
        fn poll(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<()> {
            let sig = &DMA_SIGNAL[(self.ch as usize).min(3)];
            if sig.take_fired() {
                Poll::Ready(())
            } else {
                sig.register(cx.waker());
                Poll::Pending
            }
        }
    }

    impl DmaDriver<'_, Dma0> {
        /// Await transfer-complete for `channel` (after configuring + enabling it),
        /// then clear its interrupt. Parks on the DMA IRQ (IRQ 59) on hardware; the
        /// per-channel demux means a concurrent transfer on another channel won't
        /// false-wake this one. Silicon-verified: IRQ 59 fires for both mem-to-mem
        /// and peripheral-paced single-block completion (spi_dma_irq59 test).
        pub async fn wait_transfer_done(&mut self, channel: &super::DmaChannel) {
            let channel = channel.logical;
            let bit = 1u8 << channel; // Dma0: physical channel == logical channel
            if self.raw_interrupt_status().0 & bit == 0 {
                DMA_SIGNAL[(channel as usize).min(3)].reset();
                // SAFETY: enabling a known, fixed WS63 IRQ line; enable() also raises
                // its LOCIPRI above the reset-0 threshold so it is deliverable.
                unsafe { interrupt::enable(Interrupt::DMA_INT) };
                if self.raw_interrupt_status().0 & bit == 0 {
                    DmaDoneFuture { ch: channel }.await;
                }
            }
            self.clear_transfer_interrupt_raw(channel);
        }
    }
}

#[cfg(all(feature = "chip-ws63", feature = "async", feature = "unstable"))]
pub use asynch_impl::on_interrupt;
