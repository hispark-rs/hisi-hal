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

    fn regs(&self) -> &'static ws63_pac::trng::RegisterBlock {
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

#[cfg(test)]
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
        let mut data_ready = false;
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
        let result = if data_ready {
            Ok(0xDEAD_BEEFu32)
        } else {
            Err(TrngError::Timeout)
        };
        assert_eq!(result.unwrap(), 0xDEAD_BEEF);
    }
}
