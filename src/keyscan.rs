//! BS2X KEYSCAN — key-matrix scanner (IP v150).
//!
//! BS2X-only (`chip-bs21`); no WS63 analogue. Scans a row/column key matrix and
//! reports pressed/released keys through a FIFO. Register map from fbb_bs2x
//! `hal_keyscan_v150`; bs2x-pac's `keyscan` block is a faithful match.
//!
//! Blocking model: `new()` configures + starts the scanner; `read_key()` polls the
//! event status and pops one decoded key from `key_value_fifo` (row/col/pressed).

use crate::peripherals::Keyscan as KeyscanPeriph;
use core::marker::PhantomData;

/// One decoded key event from the scan FIFO.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct KeyEvent {
    pub row: u8,
    pub col: u8,
    /// `true` = press, `false` = release.
    pub pressed: bool,
}

/// Empty-FIFO marker returned by `key_value_fifo` (regs_op.h).
const FIFO_EMPTY: u16 = 0x0FF;
const POLL_LIMIT: u32 = 1_000_000;

pub struct Keyscan<'d> {
    _k: PhantomData<KeyscanPeriph<'d>>,
}

impl<'d> Keyscan<'d> {
    fn regs(&self) -> &'static crate::soc::pac::keyscan::RegisterBlock {
        // SAFETY: static physical MMIO base (0x5208_d000) from bs2x-pac.
        unsafe { &*KeyscanPeriph::ptr() }
    }

    /// Configure a `rows` x `cols` matrix and start scanning
    /// (`hal_keyscan_v150_enable`).
    pub fn new(_k: KeyscanPeriph<'d>, rows: u8, cols: u8) -> Self {
        let this = Self { _k: PhantomData };
        let r = this.regs();
        unsafe {
            // Pin counts: row_pin_en[8:4], clo_pin_en[2:0] (count - 1).
            r.keyscan_pin_num().write(|w| {
                w.row_pin_en().bits(rows.saturating_sub(1));
                w.clo_pin_en().bits(cols.saturating_sub(1))
            });
        }
        // Enable -> FIFO read-clock -> start the scan task.
        r.keyscan_enable().write(|w| w.ena().set_bit());
        r.keyscan_clk_ena().write(|w| w.fifo_rd_clken().set_bit());
        r.keyscan_start().write(|w| w.task_start().set_bit());
        this
    }

    /// Poll for one key event; returns `None` if the scan FIFO yields the
    /// empty marker. Decodes row = key[7:3], col = key[2:0], pressed = key[8].
    pub fn read_key(&self) -> Option<KeyEvent> {
        let r = self.regs();
        // Wait for a value-ready (or press) event.
        for _ in 0..POLL_LIMIT {
            let s = r.keyscan_event_sts().read();
            if s.event_key_value_rdy().bit_is_set() || s.event_key_press().bit_is_set() {
                break;
            }
            core::hint::spin_loop();
        }
        let key = r.key_value_fifo().read().key_value().bits();
        // Acknowledge the value-ready event (write-1-to-clear).
        r.keyscan_event_clr().write(|w| w.event_key_value_rdy_clr().set_bit());
        if key == FIFO_EMPTY {
            return None;
        }
        let low = (key & 0xFF) as u8;
        Some(KeyEvent { row: low >> 3, col: low & 0x7, pressed: (key >> 8) & 0x1 != 0 })
    }
}
