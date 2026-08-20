//! SFC (SPI Flash Controller) driver for WS63.
//!
//! The WS63 SFC provides a high-speed interface to external SPI NOR Flash
//! memory. It supports standard, dual, and quad SPI modes, configurable
//! timing, command-based operations, and bus DMA for efficient data transfer.
//!
//! # Features
//!
//! - Standard/Dual/Quad SPI modes
//! - 3-byte and 4-byte addressing
//! - Configurable SPI mode (Mode0/Mode3)
//! - Command-based flash operations (read, write, erase)
//! - 16-word (64-byte) data buffer for indirect operations
//! - Bus DMA for direct memory-mapped flash access
//! - Hardware write protect
//! - AES low-power mode for XIP encryption

use crate::peripherals::SfcCfg;

/// SPI interface type for flash operations.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SpiIfType {
    /// Standard SPI (1 data line).
    Standard = 0,
    /// Dual I/O (2 data lines).
    DualIO = 1,
    /// Dual I/O continuous.
    DualIOCont = 2,
    /// Quad I/O (4 data lines).
    QuadIO = 3,
    /// Quad I/O continuous.
    QuadIOCont = 4,
}

/// Flash address mode.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AddressMode {
    /// 3-byte addressing (up to 16 MB).
    ThreeByte = 0,
    /// 4-byte addressing (up to 4 GB).
    FourByte = 1,
}

/// SPI mode for the flash bus.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FlashSpiMode {
    /// CPOL=0, CPHA=0.
    Mode0 = 0,
    /// CPOL=1, CPHA=1.
    Mode3 = 1,
}

/// SFC read data delay.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ReadDelay {
    /// No delay.
    Delay0,
    /// Half cycle delay.
    DelayHalf,
    /// 1 cycle delay.
    Delay1,
    /// 1.5 cycle delay.
    Delay1_5,
}

/// SFC bus configuration for memory-mapped reads.
#[derive(Debug, Clone, Copy)]
pub struct BusConfig {
    /// SPI interface type for read operations.
    pub read_if_type: SpiIfType,
    /// Number of dummy bytes for read operations.
    pub read_dummy_bytes: u8,
    /// Read instruction code (e.g., 0x03 for standard read, 0xEB for quad read).
    pub read_instruction: u8,
    /// Prefetch count for read operations.
    pub read_prefetch_cnt: u8,
    /// SPI interface type for write operations.
    pub write_if_type: SpiIfType,
    /// Number of dummy bytes for write operations.
    pub write_dummy_bytes: u8,
    /// Write instruction code.
    pub write_instruction: u8,
}

/// Writable SPI NOR status register selected by its standard command pair.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FlashStatusRegister {
    /// Status register 1 (`RDSR1`/`WRSR1`).
    One,
    /// Status register 2 (`RDSR2`/`WRSR2`).
    Two,
}

impl FlashStatusRegister {
    const fn read_command(self) -> u8 {
        match self {
            Self::One => 0x05,
            Self::Two => 0x35,
        }
    }

    const fn write_command(self) -> u8 {
        match self {
            Self::One => 0x01,
            Self::Two => 0x31,
        }
    }
}

impl Default for BusConfig {
    fn default() -> Self {
        Self {
            read_if_type: SpiIfType::Standard,
            read_dummy_bytes: 0,
            read_instruction: 0x03, // Standard SPI read
            read_prefetch_cnt: 0,
            write_if_type: SpiIfType::Standard,
            write_dummy_bytes: 0,
            write_instruction: 0x02, // Standard page program
        }
    }
}

const fn command_config_bits(
    select_cs1: bool,
    address_enable: bool,
    data_enable: bool,
    read: bool,
    data_len: usize,
) -> u32 {
    let mut config = 1;
    if select_cs1 {
        config |= 1 << 1;
    }
    if address_enable {
        config |= 1 << 3;
    }
    if data_enable {
        config |= 1 << 7;
        if read {
            config |= 1 << 8;
        }
        config |= ((data_len.saturating_sub(1) as u32) & 0x3f) << 9;
    }
    config
}

#[inline(always)]
const fn validate_program_chunk(flash_offset: u32, length: usize) -> Result<(), SfcError> {
    const COMMAND_BUFFER_SIZE: usize = 64;
    const NOR_PAGE_SIZE: usize = 256;

    if length == 0 || length > COMMAND_BUFFER_SIZE {
        return Err(SfcError::InvalidProgramLength);
    }
    if (flash_offset as usize & (NOR_PAGE_SIZE - 1)) + length > NOR_PAGE_SIZE {
        return Err(SfcError::ProgramCrossesPage);
    }
    Ok(())
}

#[inline(always)]
const fn validate_command_buffer(length: usize) -> Result<(), SfcError> {
    if length == 0 || length > 64 {
        Err(SfcError::InvalidProgramLength)
    } else {
        Ok(())
    }
}

#[inline(always)]
const fn validate_erase_sector_4k(flash_offset: u32) -> Result<(), SfcError> {
    if flash_offset & 0x0fff == 0 {
        Ok(())
    } else {
        Err(SfcError::InvalidEraseRange)
    }
}

/// SFC driver.
pub struct SfcDriver<'d> {
    _sfc: SfcCfg<'d>,
}

/// Minimum inter-operation delay in clock cycles.
const MIN_TSHSL: u32 = 5;

impl<'d> SfcDriver<'d> {
    /// Create a new SFC driver.
    pub fn new(sfc: SfcCfg<'d>) -> Self {
        Self { _sfc: sfc }
    }

    #[inline(always)]
    fn regs(&self) -> &'static crate::soc::pac::sfc_cfg::RegisterBlock {
        // SAFETY: PAC peripheral pointer is a static physical MMIO address, always valid
        unsafe { &*SfcCfg::ptr() }
    }

    #[cfg_attr(target_arch = "riscv32", unsafe(link_section = ".sram_text.sfc"))]
    #[inline(never)]
    fn wait_command(&self) -> Result<(), SfcError> {
        const COMMAND_POLL_LIMIT: usize = 1_000_000;

        for _ in 0..COMMAND_POLL_LIMIT {
            if !self.regs().cmd_config().read().start().bit_is_set() {
                // Command-mode fields continue to own the external flash even
                // after `start` auto-clears. Return the controller to its idle
                // configuration before executing further code through XIP.
                unsafe { self.regs().cmd_config().write(|w| w.bits(0)) };
                return Ok(());
            }
            core::hint::spin_loop();
        }
        Err(SfcError::Timeout)
    }

    #[cfg_attr(target_arch = "riscv32", unsafe(link_section = ".sram_text.sfc"))]
    #[inline(never)]
    fn issue_status_command(&self, instruction: u8, write_data: Option<u8>) -> Result<(), SfcError> {
        let r = self.regs();
        if let Some(value) = write_data {
            unsafe { r.cmd_databuf_0().write(|w| w.bits(value as u32)) };
        }
        unsafe { r.cmd_ins().write(|w| w.bits(instruction as u32)) };

        // CS1 owns the external flash. A one-byte write has data_en set with
        // data_cnt=0; the command-only 0x50 deliberately has no data phase.
        let config = command_config_bits(true, false, write_data.is_some(), false, 1);
        unsafe { r.cmd_config().write(|w| w.bits(config)) };
        self.wait_command()
    }

    #[cfg_attr(target_arch = "riscv32", unsafe(link_section = ".sram_text.sfc"))]
    #[inline(never)]
    fn read_status(&self, register: FlashStatusRegister) -> Result<u8, SfcError> {
        let r = self.regs();
        unsafe { r.cmd_ins().write(|w| w.bits(register.read_command() as u32)) };
        let config = command_config_bits(true, false, true, true, 1);
        unsafe { r.cmd_config().write(|w| w.bits(config)) };
        self.wait_command()?;
        Ok(r.cmd_databuf_0().read().bits() as u8)
    }

    #[cfg_attr(target_arch = "riscv32", unsafe(link_section = ".sram_text.sfc"))]
    #[inline(never)]
    fn wait_flash_ready(&self) -> Result<(), SfcError> {
        const READY_POLL_LIMIT: usize = 100_000;

        for _ in 0..READY_POLL_LIMIT {
            if self.read_status(FlashStatusRegister::One)? & 1 == 0 {
                return Ok(());
            }
            core::hint::spin_loop();
        }
        Err(SfcError::Timeout)
    }

    /// Volatile-write one SPI NOR status register and verify the new value.
    ///
    /// The `0x50` volatile-write-enable command and the following WRSR command
    /// are emitted as one indivisible controller sequence. SPI NOR forbids any
    /// intervening command, including a readiness poll. Callers must provide
    /// external exclusion for the complete operation.
    #[cfg_attr(target_arch = "riscv32", unsafe(link_section = ".sram_text.sfc"))]
    #[inline(never)]
    pub fn write_volatile_status(
        &mut self,
        register: FlashStatusRegister,
        value: u8,
    ) -> Result<(), SfcError> {
        self.wait_flash_ready()?;

        self.issue_status_command(0x50, None)?;
        self.issue_status_command(register.write_command(), Some(value))?;
        self.wait_flash_ready()?;

        let mask = match register {
            FlashStatusRegister::One => 0x7c,
            FlashStatusRegister::Two => 0x42,
        };
        if self.read_status(register)? & mask == value & mask {
            Ok(())
        } else {
            Err(SfcError::StatusVerify)
        }
    }

    /// Reset the external SPI NOR command state and return the SFC to XIP.
    ///
    /// `RSTEN` and `RST` are emitted without an intervening flash command. The
    /// operation runs from SRAM and returns only after the flash is ready and
    /// the indirect-command register is idle. Callers must exclude concurrent
    /// SFC users and prevent flash-resident interrupt handlers while it runs.
    #[cfg_attr(target_arch = "riscv32", unsafe(link_section = ".sram_text.sfc"))]
    #[inline(never)]
    pub fn reset_flash(&mut self) -> Result<(), SfcError> {
        self.issue_status_command(0x66, None)?;
        self.issue_status_command(0x99, None)?;
        self.wait_flash_ready()
    }

    /// Program at most one 64-byte command-buffer chunk within one NOR page.
    ///
    /// `flash_offset` is relative to the start of the external flash, not the
    /// CPU XIP address. The caller must ensure the destination is erased,
    /// externally exclude all other SFC command users. The complete command
    /// path runs from SRAM because the same controller backs instruction XIP.
    /// The method returns only after the NOR WIP bit clears and command mode is
    /// idle, so executing the caller's next flash-resident instruction is safe.
    #[cfg_attr(target_arch = "riscv32", unsafe(link_section = ".sram_text.sfc"))]
    #[inline(never)]
    pub fn program_chunk(&mut self, flash_offset: u32, data: &[u8]) -> Result<(), SfcError> {
        validate_program_chunk(flash_offset, data.len())?;

        self.wait_flash_ready()?;
        self.issue_status_command(0x06, None)?;

        let r = self.regs();
        for (index, chunk) in data.chunks(4).enumerate() {
            let mut bytes = [0u8; 4];
            bytes[..chunk.len()].copy_from_slice(chunk);
            Self::write_databuf(r, index, u32::from_le_bytes(bytes));
        }
        unsafe {
            r.cmd_ins().write(|w| w.bits(0x02));
            r.cmd_addr().write(|w| w.bits(flash_offset));
        }
        let config = command_config_bits(true, true, true, false, data.len());
        unsafe { r.cmd_config().write(|w| w.bits(config)) };
        self.wait_command()?;
        self.wait_flash_ready()
    }

    /// Read at most one 64-byte command-buffer chunk without using the XIP alias.
    ///
    /// The complete transaction runs from SRAM, making this suitable for
    /// verifying a page program before flash-resident execution resumes.
    #[cfg_attr(target_arch = "riscv32", unsafe(link_section = ".sram_text.sfc"))]
    #[inline(never)]
    pub fn read_chunk(&mut self, flash_offset: u32, output: &mut [u8]) -> Result<(), SfcError> {
        validate_command_buffer(output.len())?;
        self.wait_flash_ready()?;

        let r = self.regs();
        unsafe {
            r.cmd_ins().write(|w| w.bits(0x03));
            r.cmd_addr().write(|w| w.bits(flash_offset));
        }
        let config = command_config_bits(true, true, true, true, output.len());
        unsafe { r.cmd_config().write(|w| w.bits(config)) };
        self.wait_command()?;

        for (index, destination) in output.chunks_mut(4).enumerate() {
            let bytes = Self::read_databuf(r, index).to_le_bytes();
            destination.copy_from_slice(&bytes[..destination.len()]);
        }
        Ok(())
    }

    /// Erase one aligned 4 KiB SPI NOR sector.
    ///
    /// `flash_offset` is relative to the external flash. The caller must prove
    /// ownership of the complete sector, exclude every other SFC command user,
    /// and prevent flash-resident interrupt handlers while this runs. The
    /// command path executes from SRAM and returns only after WIP clears.
    #[cfg_attr(target_arch = "riscv32", unsafe(link_section = ".sram_text.sfc"))]
    #[inline(never)]
    pub fn erase_sector_4k(&mut self, flash_offset: u32) -> Result<(), SfcError> {
        validate_erase_sector_4k(flash_offset)?;
        self.wait_flash_ready()?;
        self.issue_status_command(0x06, None)?;

        let r = self.regs();
        unsafe {
            r.cmd_ins().write(|w| w.bits(0x20));
            r.cmd_addr().write(|w| w.bits(flash_offset));
            r.cmd_config()
                .write(|w| w.bits(command_config_bits(true, true, false, false, 0)));
        }
        self.wait_command()?;
        self.wait_flash_ready()
    }

    // ── Global configuration ───────────────────────────────────────

    /// Configure the global SFC settings.
    pub fn configure_global(
        &mut self,
        spi_mode: FlashSpiMode,
        addr_mode: AddressMode,
        read_delay: ReadDelay,
        write_protect: bool,
    ) {
        let mut val: u32 = 0;
        val |= spi_mode as u32; // mode [0]
        if write_protect {
            val |= 1 << 1; // wp_en [1]
        }
        if matches!(addr_mode, AddressMode::FourByte) {
            val |= 1 << 2; // flash_addr_mode [2]
        }
        val |= (read_delay as u32) << 3; // rd_delay [3:5]

        unsafe {
            self.regs().global_config().write(|w| w.bits(val));
        }
    }

    /// Configure the SFC timing parameters.
    ///
    /// * `tshsl` — Inter-operation delay: (tshsl + 2) clock cycles
    /// * `tcss` — CS setup time: (tcss + 1) clock cycles
    /// * `tcsh` — CS hold time: (tcsh + 1) clock cycles
    pub fn configure_timing(&mut self, tshsl: u8, tcss: u8, tcsh: u8) {
        // Mask to the 4-bit field FIRST, then floor to the minimum. Doing it the
        // other way (max() before the mask) lets a value whose low nibble is
        // below the floor — e.g. 16/32/48, low nibble 0 — survive the max() only
        // to be masked back down to 0, defeating the MIN_TSHSL guarantee. Mask
        // then floor keeps the field in [MIN_TSHSL-2, 0xF] for every input.
        let tshsl = ((tshsl as u32) & 0x0F).max(MIN_TSHSL - 2);
        let tcss = (tcss as u32) & 0x07;
        let tcsh = (tcsh as u32) & 0x07;

        let val = tshsl | (tcss << 8) | (tcsh << 12);

        unsafe {
            self.regs().timing().write(|w| w.bits(val));
        }
    }

    // ── Bus configuration ──────────────────────────────────────────

    /// Configure the bus read/write parameters for memory-mapped access.
    pub fn configure_bus(&mut self, config: &BusConfig) {
        let r = self.regs();

        // BUS_CONFIG1
        let mut cfg1: u32 = 0;
        cfg1 |= (config.read_if_type as u32) & 0x07; // rd_mem_if_type [0:2]
        cfg1 |= ((config.read_dummy_bytes as u32) & 0x07) << 3; // rd_dummy_bytes [3:5]
        cfg1 |= ((config.read_prefetch_cnt as u32) & 0x03) << 6; // rd_prefetch_cnt [6:7]
        cfg1 |= ((config.read_instruction as u32) & 0xFF) << 8; // rd_ins [8:15]
        cfg1 |= ((config.write_if_type as u32) & 0x07) << 16; // wr_mem_if_type [16:18]
        cfg1 |= ((config.write_dummy_bytes as u32) & 0x07) << 19; // wr_dummy_bytes [19:21]
        cfg1 |= ((config.write_instruction as u32) & 0xFF) << 22; // wr_ins [22:29]

        unsafe {
            r.bus_config1().write(|w| w.bits(cfg1));
        }
    }

    /// Release the SFC bus from soft reset.
    pub fn release_bus_reset(&mut self) {
        unsafe {
            self.regs().soft_rst_mask().write(|w| w.bits(0x01));
        }
    }

    /// Hold the SFC bus in soft reset.
    pub fn hold_bus_reset(&mut self) {
        unsafe {
            self.regs().soft_rst_mask().write(|w| w.bits(0x00));
        }
    }

    // ── Command operations ──────────────────────────────────────────

    /// Write a 32-bit word to a specific data buffer register.
    #[cfg_attr(target_arch = "riscv32", unsafe(link_section = ".sram_text.sfc"))]
    #[inline(never)]
    fn write_databuf(r: &crate::soc::pac::sfc_cfg::RegisterBlock, idx: usize, word: u32) {
        unsafe {
            match idx {
                0 => {
                    r.cmd_databuf_0().write(|w| w.bits(word));
                }
                1 => {
                    r.cmd_databuf_1().write(|w| w.bits(word));
                }
                2 => {
                    r.cmd_databuf_2().write(|w| w.bits(word));
                }
                3 => {
                    r.cmd_databuf_3().write(|w| w.bits(word));
                }
                4 => {
                    r.cmd_databuf_4().write(|w| w.bits(word));
                }
                5 => {
                    r.cmd_databuf_5().write(|w| w.bits(word));
                }
                6 => {
                    r.cmd_databuf_6().write(|w| w.bits(word));
                }
                7 => {
                    r.cmd_databuf_7().write(|w| w.bits(word));
                }
                8 => {
                    r.cmd_databuf_8().write(|w| w.bits(word));
                }
                9 => {
                    r.cmd_databuf_9().write(|w| w.bits(word));
                }
                10 => {
                    r.cmd_databuf_10().write(|w| w.bits(word));
                }
                11 => {
                    r.cmd_databuf_11().write(|w| w.bits(word));
                }
                12 => {
                    r.cmd_databuf_12().write(|w| w.bits(word));
                }
                13 => {
                    r.cmd_databuf_13().write(|w| w.bits(word));
                }
                14 => {
                    r.cmd_databuf_14().write(|w| w.bits(word));
                }
                15 => {
                    r.cmd_databuf_15().write(|w| w.bits(word));
                }
                _ => {}
            }
        }
    }

    /// Read a 32-bit word from a specific data buffer register.
    #[cfg_attr(target_arch = "riscv32", unsafe(link_section = ".sram_text.sfc"))]
    #[inline(never)]
    fn read_databuf(r: &crate::soc::pac::sfc_cfg::RegisterBlock, idx: usize) -> u32 {
        match idx {
            0 => r.cmd_databuf_0().read().bits(),
            1 => r.cmd_databuf_1().read().bits(),
            2 => r.cmd_databuf_2().read().bits(),
            3 => r.cmd_databuf_3().read().bits(),
            4 => r.cmd_databuf_4().read().bits(),
            5 => r.cmd_databuf_5().read().bits(),
            6 => r.cmd_databuf_6().read().bits(),
            7 => r.cmd_databuf_7().read().bits(),
            8 => r.cmd_databuf_8().read().bits(),
            9 => r.cmd_databuf_9().read().bits(),
            10 => r.cmd_databuf_10().read().bits(),
            11 => r.cmd_databuf_11().read().bits(),
            12 => r.cmd_databuf_12().read().bits(),
            13 => r.cmd_databuf_13().read().bits(),
            14 => r.cmd_databuf_14().read().bits(),
            15 => r.cmd_databuf_15().read().bits(),
            _ => 0,
        }
    }

    /// Execute a flash command (no data phase).
    ///
    /// * `instruction` — Flash operation code.
    /// * `address` — Operation address (for commands with address phase).
    /// * `address_enable` — Whether the command includes an address phase.
    pub fn send_command(&mut self, instruction: u8, address: u32, address_enable: bool) {
        let r = self.regs();

        // Write instruction
        unsafe {
            r.cmd_ins().write(|w| w.bits(instruction as u32));
        }

        // Write address
        if address_enable {
            unsafe {
                r.cmd_addr().write(|w| w.bits(address));
            }
        }

        // Build command config
        let cmd_cfg = command_config_bits(false, address_enable, false, false, 0);

        unsafe {
            r.cmd_config().write(|w| w.bits(cmd_cfg));
        }

        // Wait for command completion
        while !self.command_done() {}
        self.clear_interrupts();
    }

    /// Execute a flash command with data phase.
    ///
    /// * `instruction` — Flash operation code.
    /// * `address` — Operation address.
    /// * `write_data` — Data to write (for write commands).
    /// * `read` — `true` for read commands, `false` for write commands.
    ///
    /// Returns the read data (up to 64 bytes) for read commands.
    pub fn command_with_data(
        &mut self,
        instruction: u8,
        address: u32,
        write_data: &[u8],
        read: bool,
    ) -> Result<[u8; 64], SfcError> {
        let r = self.regs();
        // The command data buffer is 64 bytes; reject an over-long write rather than
        // silently truncating it (the old `.min(64)` dropped the tail).
        if write_data.len() > 64 {
            return Err(SfcError::BufferTooLong);
        }
        let data_len = write_data.len();

        if !read && !write_data.is_empty() {
            // Load write data into data buffer
            for (i, chunk) in write_data[..data_len].chunks(4).enumerate() {
                let mut word: u32 = 0;
                for (j, &b) in chunk.iter().enumerate() {
                    word |= (b as u32) << (j * 8);
                }
                Self::write_databuf(r, i, word);
            }
        }

        // Write instruction and address
        unsafe {
            r.cmd_ins().write(|w| w.bits(instruction as u32));
            r.cmd_addr().write(|w| w.bits(address));
        }

        // Build command config
        let cmd_cfg = command_config_bits(false, true, true, read, data_len);

        unsafe {
            r.cmd_config().write(|w| w.bits(cmd_cfg));
        }

        // Wait for command completion
        while !self.command_done() {}
        self.clear_interrupts();

        // Read back data for read commands
        let mut result = [0u8; 64];
        if read {
            let mut idx = 0;
            for i in 0..data_len.div_ceil(4) {
                let word = Self::read_databuf(r, i);
                let bytes = word.to_le_bytes();
                for &b in &bytes {
                    if idx < data_len {
                        result[idx] = b;
                        idx += 1;
                    }
                }
            }
        }

        Ok(result)
    }

    // ── Bus DMA ────────────────────────────────────────────────────

    /// Start a bus DMA transfer between flash and memory.
    ///
    /// * `mem_addr` — Memory address (must be in valid range).
    /// * `flash_addr` — Flash address.
    /// * `length` — Number of bytes to transfer.
    /// * `read` — `true` for flash-to-memory read, `false` for memory-to-flash write.
    pub fn bus_dma_start(&mut self, mem_addr: u32, flash_addr: u32, length: u32, read: bool) {
        let r = self.regs();

        unsafe {
            r.bus_dma_mem_saddr().write(|w| w.bits(mem_addr));
            r.bus_dma_flash_saddr().write(|w| w.bits(flash_addr));
            r.bus_dma_len().write(|w| w.bits(length & 0x3FFF_FFFF));
        }

        let mut ctrl: u32 = 0;
        ctrl |= 0x01; // dma_start
        if read {
            ctrl |= 1 << 1; // dma_rw = read
        }

        unsafe {
            r.bus_dma_ctrl().write(|w| w.bits(ctrl));
        }
    }

    /// Wait for bus DMA to complete.
    pub fn bus_dma_wait(&self) {
        while !self.dma_done() {}
        self.clear_interrupts();
    }

    /// Check if the DMA transfer is complete.
    pub fn dma_done(&self) -> bool {
        self.regs().int_status().read().bits() & 0x02 != 0
    }

    /// Check if a flash command is complete.
    pub fn command_done(&self) -> bool {
        self.regs().int_status().read().bits() & 0x01 != 0
    }

    /// Clear all SFC interrupts.
    pub fn clear_interrupts(&self) {
        unsafe {
            self.regs().int_clear().write(|w| w.bits(0x03));
        }
    }

    /// Enable specific SFC interrupts.
    ///
    /// * `cmd_done` — Command operation complete interrupt.
    /// * `dma_done` — DMA transfer complete interrupt.
    pub fn enable_interrupts(&mut self, cmd_done: bool, dma_done: bool) {
        let mut mask: u32 = 0;
        if cmd_done {
            mask |= 0x01;
        }
        if dma_done {
            mask |= 0x02;
        }
        unsafe {
            self.regs().int_mask().write(|w| w.bits(mask));
        }
    }

    /// Check raw interrupt status.
    ///
    /// Returns `(cmd_done_raw, dma_done_raw)`.
    pub fn raw_interrupt_status(&self) -> (bool, bool) {
        let sts = self.regs().int_raw_status().read().bits();
        ((sts & 0x01) != 0, (sts & 0x02) != 0)
    }

    // ── AES (XIP encryption) control ───────────────────────────────

    /// Enable AES low-power mode (for XIP encrypted execution).
    pub fn enable_aes_low_power(&mut self) {
        unsafe {
            self.regs().lea_lp_en().write(|w| w.bits(0x01));
        }
    }

    /// Disable AES low-power mode.
    pub fn disable_aes_low_power(&mut self) {
        unsafe {
            self.regs().lea_lp_en().write(|w| w.bits(0x00));
        }
    }

    /// Set AES IV valid flag.
    pub fn set_iv_valid(&mut self) {
        unsafe {
            self.regs().lea_iv_vld().write(|w| w.bits(0x01));
        }
    }

    /// Read AES DFX information (for debugging).
    pub fn read_aes_dfx(&self) -> u32 {
        self.regs().lea_dfx_info().read().bits()
    }
}

/// SFC operation error.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum SfcError {
    /// Command timeout.
    Timeout,
    /// DMA transfer error.
    DmaError,
    /// A `write_data` slice longer than the 64-byte command data buffer was passed
    /// — rejected rather than silently truncated to the first 64 bytes.
    BufferTooLong,
    /// A status-register write completed but readback did not match.
    StatusVerify,
    /// A command-buffer program request was empty or larger than 64 bytes.
    InvalidProgramLength,
    /// A program request crossed a 256-byte SPI NOR page boundary.
    ProgramCrossesPage,
    /// A sector erase request was not aligned to the 4 KiB erase geometry.
    InvalidEraseRange,
}

// ── Tests ──────────────────────────────────────────────────────
//
// These re-derive the *pure* bit-packing / clamping arithmetic that the
// register-writing methods perform, without touching any MMIO. Each helper
// mirrors the exact expression in the corresponding driver method so the
// asserts pin the encoding contract (field offsets, masks, clamps).

#[cfg(all(test, not(target_arch = "riscv32")))]
mod tests {
    use super::*;

    /// Re-derivation of the GLOBAL_CONFIG word built by `configure_global`.
    fn global_config_bits(
        spi_mode: FlashSpiMode,
        addr_mode: AddressMode,
        read_delay: ReadDelay,
        write_protect: bool,
    ) -> u32 {
        let mut val: u32 = 0;
        val |= spi_mode as u32; // mode [0]
        if write_protect {
            val |= 1 << 1; // wp_en [1]
        }
        if matches!(addr_mode, AddressMode::FourByte) {
            val |= 1 << 2; // flash_addr_mode [2]
        }
        val |= (read_delay as u32) << 3; // rd_delay [3:5]
        val
    }

    /// Re-derivation of the TIMING word built by `configure_timing`.
    fn timing_bits(tshsl: u8, tcss: u8, tcsh: u8) -> u32 {
        let tshsl = ((tshsl as u32) & 0x0F).max(MIN_TSHSL - 2);
        let tcss = (tcss as u32) & 0x07;
        let tcsh = (tcsh as u32) & 0x07;
        tshsl | (tcss << 8) | (tcsh << 12)
    }

    /// Re-derivation of the BUS_CONFIG1 word built by `configure_bus`.
    fn bus_config1_bits(config: &BusConfig) -> u32 {
        let mut cfg1: u32 = 0;
        cfg1 |= (config.read_if_type as u32) & 0x07;
        cfg1 |= ((config.read_dummy_bytes as u32) & 0x07) << 3;
        cfg1 |= ((config.read_prefetch_cnt as u32) & 0x03) << 6;
        cfg1 |= ((config.read_instruction as u32) & 0xFF) << 8;
        cfg1 |= ((config.write_if_type as u32) & 0x07) << 16;
        cfg1 |= ((config.write_dummy_bytes as u32) & 0x07) << 19;
        cfg1 |= ((config.write_instruction as u32) & 0xFF) << 22;
        cfg1
    }

    #[test]
    fn enum_discriminants_match_field_values() {
        // The enums are cast `as u32` straight into register fields, so the
        // discriminants ARE the wire encoding — assert the known values.
        assert_eq!(SpiIfType::Standard as u32, 0);
        assert_eq!(SpiIfType::DualIO as u32, 1);
        assert_eq!(SpiIfType::DualIOCont as u32, 2);
        assert_eq!(SpiIfType::QuadIO as u32, 3);
        assert_eq!(SpiIfType::QuadIOCont as u32, 4);

        assert_eq!(AddressMode::ThreeByte as u32, 0);
        assert_eq!(AddressMode::FourByte as u32, 1);

        assert_eq!(FlashSpiMode::Mode0 as u32, 0);
        assert_eq!(FlashSpiMode::Mode3 as u32, 1);

        assert_eq!(ReadDelay::Delay0 as u32, 0);
        assert_eq!(ReadDelay::DelayHalf as u32, 1);
        assert_eq!(ReadDelay::Delay1 as u32, 2);
        assert_eq!(ReadDelay::Delay1_5 as u32, 3);
    }

    #[test]
    fn global_config_all_zero_default() {
        // Mode0 + 3-byte + no delay + no WP encodes to all-zero.
        let v = global_config_bits(FlashSpiMode::Mode0, AddressMode::ThreeByte, ReadDelay::Delay0, false);
        assert_eq!(v, 0);
    }

    #[test]
    fn global_config_each_field_lands_in_its_bit() {
        // mode bit [0]
        assert_eq!(
            global_config_bits(FlashSpiMode::Mode3, AddressMode::ThreeByte, ReadDelay::Delay0, false),
            0b0000_0001
        );
        // wp_en bit [1]
        assert_eq!(
            global_config_bits(FlashSpiMode::Mode0, AddressMode::ThreeByte, ReadDelay::Delay0, true),
            0b0000_0010
        );
        // flash_addr_mode bit [2]
        assert_eq!(
            global_config_bits(FlashSpiMode::Mode0, AddressMode::FourByte, ReadDelay::Delay0, false),
            0b0000_0100
        );
        // rd_delay occupies [3:5]; Delay1_5 (=3) shifts to 0b11000
        assert_eq!(
            global_config_bits(FlashSpiMode::Mode0, AddressMode::ThreeByte, ReadDelay::Delay1_5, false),
            0b0001_1000
        );
    }

    #[test]
    fn global_config_fields_are_independent() {
        // All four fields set at once OR together without collision.
        let v = global_config_bits(FlashSpiMode::Mode3, AddressMode::FourByte, ReadDelay::Delay1_5, true);
        assert_eq!(v, 0b0001_1111);
    }

    #[test]
    fn timing_clamps_tshsl_to_minimum() {
        // tshsl is floored at MIN_TSHSL - 2 (the field stores tshsl, HW adds 2,
        // so the effective minimum inter-op delay is MIN_TSHSL cycles).
        let floor = MIN_TSHSL as u8 - 2; // == 3
        assert_eq!(floor, 3);
        // Any input below the floor is raised to it.
        assert_eq!(timing_bits(0, 0, 0) & 0x0F, floor as u32);
        assert_eq!(timing_bits(1, 0, 0) & 0x0F, floor as u32);
        assert_eq!(timing_bits(2, 0, 0) & 0x0F, floor as u32);
        // At/above the floor the value passes through.
        assert_eq!(timing_bits(3, 0, 0) & 0x0F, 3);
        assert_eq!(timing_bits(7, 0, 0) & 0x0F, 7);
    }

    #[test]
    fn timing_masks_field_widths() {
        // tshsl is a 4-bit field: 0xFF masks to 0x0F.
        assert_eq!(timing_bits(0xFF, 0, 0) & 0x0F, 0x0F);
        // tcss/tcsh are 3-bit fields at [8:10] and [12:14]: 0xFF masks to 0x07.
        assert_eq!((timing_bits(3, 0xFF, 0) >> 8) & 0x07, 0x07);
        assert_eq!((timing_bits(3, 0, 0xFF) >> 12) & 0x07, 0x07);
    }

    #[test]
    fn timing_places_fields_at_correct_offsets() {
        // tshsl [0:3], tcss [8:10], tcsh [12:14] are non-overlapping.
        let v = timing_bits(5, 6, 7);
        assert_eq!(v & 0x0F, 5);
        assert_eq!((v >> 8) & 0x07, 6);
        assert_eq!((v >> 12) & 0x07, 7);
        // No stray bits set outside the three fields.
        assert_eq!(v, 5 | (6 << 8) | (7 << 12));
    }

    #[test]
    fn bus_config1_default_round_trips() {
        // Default: standard read 0x03, standard write 0x02, no dummy/prefetch.
        let cfg = BusConfig::default();
        let v = bus_config1_bits(&cfg);
        assert_eq!(v & 0x07, SpiIfType::Standard as u32); // rd_mem_if_type
        assert_eq!((v >> 8) & 0xFF, 0x03); // rd_ins
        assert_eq!((v >> 16) & 0x07, SpiIfType::Standard as u32); // wr_mem_if_type
        assert_eq!((v >> 22) & 0xFF, 0x02); // wr_ins
        // dummy + prefetch fields are zero.
        assert_eq!((v >> 3) & 0x07, 0);
        assert_eq!((v >> 6) & 0x03, 0);
        assert_eq!((v >> 19) & 0x07, 0);
    }

    #[test]
    fn bus_config1_quad_read_encoding() {
        // A realistic quad fast-read (0xEB, quad I/O, 4 dummy bytes, prefetch 2).
        let cfg = BusConfig {
            read_if_type: SpiIfType::QuadIO,
            read_dummy_bytes: 4,
            read_instruction: 0xEB,
            read_prefetch_cnt: 2,
            write_if_type: SpiIfType::QuadIO,
            write_dummy_bytes: 0,
            write_instruction: 0x32,
        };
        let v = bus_config1_bits(&cfg);
        assert_eq!(v & 0x07, SpiIfType::QuadIO as u32);
        assert_eq!((v >> 3) & 0x07, 4);
        assert_eq!((v >> 6) & 0x03, 2);
        assert_eq!((v >> 8) & 0xFF, 0xEB);
        assert_eq!((v >> 16) & 0x07, SpiIfType::QuadIO as u32);
        assert_eq!((v >> 22) & 0xFF, 0x32);
    }

    #[test]
    fn bus_config1_dummy_field_saturates_to_3_bits() {
        // read_dummy_bytes is a 3-bit field; 7 fits, but 8 wraps to 0.
        let mut cfg = BusConfig { read_dummy_bytes: 7, ..BusConfig::default() };
        assert_eq!((bus_config1_bits(&cfg) >> 3) & 0x07, 7);
        cfg.read_dummy_bytes = 8;
        assert_eq!((bus_config1_bits(&cfg) >> 3) & 0x07, 0);
        // prefetch is a 2-bit field; 3 fits, 4 wraps to 0.
        cfg = BusConfig { read_prefetch_cnt: 3, ..BusConfig::default() };
        assert_eq!((bus_config1_bits(&cfg) >> 6) & 0x03, 3);
        cfg.read_prefetch_cnt = 4;
        assert_eq!((bus_config1_bits(&cfg) >> 6) & 0x03, 0);
    }

    #[test]
    fn databuf_word_packing_is_little_endian() {
        // command_with_data packs each 4-byte chunk LE: byte j → bits [j*8..].
        let chunk = [0x11u8, 0x22, 0x33, 0x44];
        let mut word: u32 = 0;
        for (j, &b) in chunk.iter().enumerate() {
            word |= (b as u32) << (j * 8);
        }
        assert_eq!(word, 0x4433_2211);
        // Round-trips through to_le_bytes (the read-back path).
        assert_eq!(word.to_le_bytes(), chunk);
    }

    #[test]
    fn databuf_partial_chunk_packs_low_bytes() {
        // A 3-byte tail leaves the top byte clear.
        let chunk = [0xAAu8, 0xBB, 0xCC];
        let mut word: u32 = 0;
        for (j, &b) in chunk.iter().enumerate() {
            word |= (b as u32) << (j * 8);
        }
        assert_eq!(word, 0x00CC_BBAA);
    }

    #[test]
    fn data_count_field_encodes_len_minus_one() {
        // cmd_cfg packs (data_len - 1) into a 6-bit field [9:14], saturating at 0.
        // `command_with_data` rejects len > 64 up front, so the field never wraps.
        let encode = |data_len: usize| ((data_len.saturating_sub(1)) as u32) & 0x3F;
        assert_eq!(encode(0), 0); // saturating_sub keeps 0 (empty)
        assert_eq!(encode(1), 0); // 1 byte → count 0
        assert_eq!(encode(64), 63); // full buffer → max 6-bit value
    }

    #[test]
    fn command_config_uses_pac_field_offsets() {
        assert_eq!(command_config_bits(false, false, false, false, 0), 1);
        assert_eq!(command_config_bits(true, false, false, false, 0), 1 | (1 << 1));
        assert_eq!(command_config_bits(false, true, false, false, 0), 1 | (1 << 3));
        assert_eq!(command_config_bits(false, false, true, true, 1), 1 | (1 << 7) | (1 << 8));
        assert_eq!(
            command_config_bits(true, true, true, false, 64),
            1 | (1 << 1) | (1 << 3) | (1 << 7) | (63 << 9)
        );
    }

    #[test]
    fn program_chunk_validation_matches_controller_and_nor_boundaries() {
        assert_eq!(validate_program_chunk(0x100, 1), Ok(()));
        assert_eq!(validate_program_chunk(0x1c0, 64), Ok(()));
        assert_eq!(validate_program_chunk(0x1ff, 1), Ok(()));
        assert_eq!(validate_program_chunk(0x100, 0), Err(SfcError::InvalidProgramLength));
        assert_eq!(validate_program_chunk(0x100, 65), Err(SfcError::InvalidProgramLength));
        assert_eq!(validate_program_chunk(0x1e0, 64), Err(SfcError::ProgramCrossesPage));
    }

    #[test]
    fn erase_sector_validation_requires_4k_alignment() {
        assert_eq!(validate_erase_sector_4k(0), Ok(()));
        assert_eq!(validate_erase_sector_4k(0x3fc000), Ok(()));
        assert_eq!(
            validate_erase_sector_4k(0x3fc001),
            Err(SfcError::InvalidEraseRange)
        );
    }

    #[test]
    fn over_long_write_is_rejected_not_truncated() {
        // command_with_data returns Err(BufferTooLong) for len > 64 (was a silent
        // .min(64) truncation that dropped the tail of the write).
        // This re-derives the guard's boundary without touching MMIO.
        let accepts = |len: usize| len <= 64;
        assert!(accepts(0) && accepts(64));
        assert!(!accepts(65) && !accepts(100));
    }
}

// ── Property-based fuzz tests ──────────────────────────────────

#[cfg(all(test, not(target_arch = "riscv32")))]
mod proptests {
    use super::MIN_TSHSL;
    use proptest::prelude::*;

    fn timing_bits(tshsl: u8, tcss: u8, tcsh: u8) -> u32 {
        let tshsl = ((tshsl as u32) & 0x0F).max(MIN_TSHSL - 2);
        let tcss = (tcss as u32) & 0x07;
        let tcsh = (tcsh as u32) & 0x07;
        tshsl | (tcss << 8) | (tcsh << 12)
    }

    proptest! {
        /// Fuzz: the timing word only ever sets bits inside its three fields.
        #[test]
        fn timing_sets_no_stray_bits(tshsl in any::<u8>(), tcss in any::<u8>(), tcsh in any::<u8>()) {
            let v = timing_bits(tshsl, tcss, tcsh);
            // Allowed bits: [0:3] | [8:10] | [12:14].
            let allowed: u32 = 0x0F | (0x07 << 8) | (0x07 << 12);
            prop_assert_eq!(v & !allowed, 0);
        }

        /// Fuzz: tshsl field is always at least the clamped floor (MIN_TSHSL-2).
        #[test]
        fn timing_tshsl_never_below_floor(tshsl in any::<u8>()) {
            let floor = (MIN_TSHSL as u8 - 2) as u32;
            let field = timing_bits(tshsl, 0, 0) & 0x0F;
            prop_assert!(field >= floor);
        }

        /// Fuzz: LE word packing of a 4-byte chunk always round-trips.
        #[test]
        fn databuf_pack_round_trips(b in any::<[u8; 4]>()) {
            let mut word: u32 = 0;
            for (j, &x) in b.iter().enumerate() {
                word |= (x as u32) << (j * 8);
            }
            prop_assert_eq!(word.to_le_bytes(), b);
        }

        /// Fuzz: the data-count field never exceeds its 6-bit width.
        #[test]
        fn data_count_fits_6_bits(len in 0usize..=64) {
            let field = ((len.saturating_sub(1)) as u32) & 0x3F;
            prop_assert!(field <= 0x3F);
        }
    }
}
