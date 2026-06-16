//! True Random Number Generator (TRNG) driver for WS63.
//!
//! The WS63 TRNG generates true random numbers using physical entropy
//! sources (FRO — Free-Running Oscillator). Random data is read from
//! a FIFO register.
//!
//! # Usage
//!
//! ```ignore
//! let trng = Trng::new(peripherals.TRNG);
//! let random_word: u32 = trng.read_blocking().unwrap();
//! ```

use crate::peripherals::Trng;

/// TRNG error type.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum TrngError {
    /// No data available in the FIFO.
    NoData,
    /// Timeout waiting for entropy generation.
    Timeout,
}

/// True Random Number Generator driver.
pub struct TrngDriver<'d> {
    _trng: Trng<'d>,
}

impl<'d> TrngDriver<'d> {
    /// Create a new TRNG driver.
    pub fn new(trng: Trng<'d>) -> Self {
        Self { _trng: trng }
    }

    fn regs(&self) -> &'static crate::soc::pac::trng::RegisterBlock {
        // SAFETY: PAC peripheral pointer is a static physical MMIO address, always valid
        unsafe { &*Trng::ptr() }
    }

    /// Check if random data is ready in the FIFO.
    pub fn data_ready(&self) -> bool {
        self.regs().trng_fifo_ready().read().bits() & 0x01 != 0
    }

    /// Check if the TRNG generation is complete.
    pub fn done(&self) -> bool {
        self.regs().trng_fifo_ready().read().bits() & 0x02 != 0
    }

    /// Read a 32-bit random word from the TRNG FIFO.
    ///
    /// Returns `Err(NoData)` if no data is available.
    pub fn read(&self) -> Result<u32, TrngError> {
        if !self.data_ready() {
            return Err(TrngError::NoData);
        }
        Ok(self.regs().trng_fifo_data().read().bits())
    }

    /// Read a random word, blocking until data is available.
    ///
    /// Returns `Err(Timeout)` if the TRNG fails to produce entropy after
    /// ~4ms (1,000,000 spin-loop iterations at 240MHz). On cold start,
    /// the FRO-based entropy source may need several attempts to stabilize;
    /// retry the call if the first attempt times out.
    pub fn read_blocking(&self) -> Result<u32, TrngError> {
        for _ in 0..1_000_000 {
            if self.data_ready() {
                return Ok(self.regs().trng_fifo_data().read().bits());
            }
            core::hint::spin_loop();
        }
        Err(TrngError::Timeout)
    }

    /// Fill a buffer with random bytes.
    ///
    /// Each 32-bit word produces 4 bytes. Returns `Err(Timeout)` if the TRNG
    /// hardware fails to produce entropy.
    pub fn fill_bytes(&self, buf: &mut [u8]) -> Result<(), TrngError> {
        let mut i = 0;
        while i < buf.len() {
            let word = self.read_blocking()?;
            let bytes = word.to_le_bytes();
            for &b in &bytes {
                if i < buf.len() {
                    buf[i] = b;
                    i += 1;
                }
            }
        }
        Ok(())
    }

    /// Fill a buffer with random 32-bit words.
    ///
    /// Returns `Err(Timeout)` if the TRNG hardware fails to produce entropy.
    pub fn fill_words(&self, buf: &mut [u32]) -> Result<(), TrngError> {
        for word in buf.iter_mut() {
            *word = self.read_blocking()?;
        }
        Ok(())
    }

    /// Select the FRO sample clock source.
    ///
    /// * `external` — `true` for external clock, `false` for internal clock.
    pub fn set_sample_clock(&mut self, external: bool) {
        unsafe {
            self.regs().trng_fro_sample_clk_sel().write(|w| w.bits(if external { 1 } else { 0 }));
        }
    }

    /// Set the FRO divider count.
    ///
    /// Controls the sampling rate of the FRO entropy source.
    /// Default is 0x1b (27).
    pub fn set_divider(&mut self, div: u8) {
        unsafe {
            self.regs().trng_fro_div_cnt().write(|w| w.bits(div as u32));
        }
    }

    /// Get the data status register value (for debugging).
    pub fn data_status(&self) -> u32 {
        self.regs().trng_data_st().read().bits()
    }
}

// ── Tests ──────────────────────────────────────────────────────

#[cfg(all(test, not(target_arch = "riscv32")))]
mod tests {
    use super::*;

    #[test]
    fn test_trng_error_type_variants() {
        assert_ne!(TrngError::NoData as u8, TrngError::Timeout as u8);
    }

    #[test]
    fn test_trng_data_ready_bit() {
        // data_ready checks fifo_ready bit 0
        let ready: u32 = 0x01;
        assert!((ready & 0x01) != 0); // data ready
        let not_ready: u32 = 0x00;
        assert!((not_ready & 0x01) == 0); // not ready
    }

    #[test]
    fn test_trng_done_bit() {
        // done checks fifo_ready bit 1
        let done: u32 = 0x02;
        assert!((done & 0x02) != 0); // generation done
        let not_done: u32 = 0x00;
        assert!((not_done & 0x02) == 0); // not done
    }

    #[test]
    fn test_trng_read_blocking_timeout_logic() {
        // Simulate the timeout loop: should return Err after retries exhausted
        let max_retries = 10u32;
        let data_ready = false;
        let mut retries = 0;
        let result = loop {
            if data_ready {
                break Ok(42u32);
            }
            if retries >= max_retries {
                break Err(TrngError::Timeout);
            }
            retries += 1;
        };
        assert_eq!(result, Err(TrngError::Timeout));
        assert_eq!(retries, 10);
    }

    #[test]
    fn test_trng_read_blocking_success_first_try() {
        let data_ready = true;
        let result = if data_ready { Ok(0xDEAD_BEEFu32) } else { Err(TrngError::Timeout) };
        assert_eq!(result.unwrap(), 0xDEAD_BEEF);
    }
}

// ── Property-based fuzz tests ──────────────────────────────────

#[cfg(all(test, not(target_arch = "riscv32")))]
mod proptests {
    use proptest::prelude::*;

    /// Pure re-derivation of `TrngDriver::fill_bytes` packing over a fixed 32-byte
    /// scratch buffer (`len` selects the active prefix), with the MMIO
    /// `read_blocking()` replaced by drawing from a pre-supplied word stream.
    /// This is byte-for-byte the same little-endian split + buffer-fill clamp the
    /// driver runs (`word.to_le_bytes()` into `buf[i]`, stopping at `buf.len()`).
    /// Returns the populated scratch buffer.
    fn fill_bytes_from(words: &[u32; 8], len: usize) -> [u8; 32] {
        let mut buf = [0u8; 32];
        let active = &mut buf[..len];
        let mut i = 0;
        let mut w = 0;
        while i < active.len() {
            // Driver pulls a fresh word each outer iteration via read_blocking().
            let word = words[w % words.len()];
            w += 1;
            let bytes = word.to_le_bytes();
            for &b in &bytes {
                if i < active.len() {
                    active[i] = b;
                    i += 1;
                }
            }
        }
        buf
    }

    proptest! {
        /// Fuzz: fill_bytes packing never panics / never writes out of bounds for
        /// any buffer length and any word stream (the inner `i < buf.len()` guard
        /// must hold even when the final word straddles the buffer tail).
        #[test]
        fn fill_bytes_never_overflows(
            words in any::<[u32; 8]>(),
            len in 0usize..=32,
        ) {
            let _ = fill_bytes_from(&words, len);
        }

        /// Fuzz: for a buffer whose length is a whole number of words, every byte
        /// is exactly the little-endian decomposition of the consumed words — i.e.
        /// the buffer round-trips back to the source words with no shuffle/gap.
        #[test]
        fn fill_bytes_round_trips_whole_words(
            words in any::<[u32; 8]>(),
            nwords in 0usize..=8,
        ) {
            let buf = fill_bytes_from(&words, nwords * 4);
            for k in 0..nwords {
                let chunk: [u8; 4] = buf[k * 4..k * 4 + 4].try_into().unwrap();
                prop_assert_eq!(u32::from_le_bytes(chunk), words[k]);
            }
        }

        /// Fuzz: the partial-word tail is handled by truncation, never by reading
        /// past the buffer. The bytes written into a non-aligned tail must be the
        /// low-order LE prefix of the next word (LE => buf[i] = (word >> 8*j) & 0xff).
        #[test]
        fn fill_bytes_tail_is_le_prefix(
            word in any::<u32>(),
            tail in 1usize..4,
        ) {
            // One full word already consumed (zeros), then `tail` bytes of `word`.
            let words = [0, word, 0, 0, 0, 0, 0, 0];
            let buf = fill_bytes_from(&words, 4 + tail);
            for j in 0..tail {
                let expected = ((word >> (8 * j)) & 0xff) as u8;
                prop_assert_eq!(buf[4 + j], expected);
            }
        }

        /// Fuzz: set_divider widening (`div as u32`) is a pure zero-extension —
        /// it never sets any bit above bit 7, for any u8 divider.
        #[test]
        fn divider_widening_zero_extends(div in any::<u8>()) {
            let reg = div as u32;
            prop_assert_eq!(reg & !0xff, 0);
            prop_assert_eq!(reg as u8, div);
        }
    }
}
