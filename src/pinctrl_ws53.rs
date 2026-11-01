//! WS53 per-pad configuration through generated PAC fields.
//!
//! Ownership covers the physical pad, all aliases, and any signals being muxed.
//! Claiming a pad is unsafe until other cores/peripherals have quiesced it and
//! pad/GPIO clocks are guaranteed. This module does not negotiate ownership,
//! change clock trees or establish undocumented GPIO_SW_CTL source encodings.
//! Configure with external drivers quiesced: pad fields are updated before mux,
//! but two separate registers cannot form an electrically atomic transition.
use crate::gpio::{AnyPin, PinError};
use crate::soc::pac;
use crate::soc::ws53::pins::{DriveSetting, MuxMode, MuxSelection, Pad};
use core::marker::PhantomData;

/// Pad pull resistor selection (not the reversed legacy hal_pio_pull_t enum).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum Pull {
    /// Neither pull enabled.
    None = 0,
    /// Pull toward the pad supply.
    Up = 1,
    /// Pull toward ground.
    Down = 2,
}

/// An invalid or protected pad configuration.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error {
    /// SWD/reset configuration is intentionally unavailable here.
    ProtectedPad,
    /// Mux and drive configuration belong to different pads, or a different owner.
    MismatchedPad,
    /// Readback has a mode/pull encoding outside the audited SDK settings.
    UnsupportedReadback,
    /// Transfer to GPIO requires mode 0 and its input buffer enabled.
    GpioNotPrepared,
}

/// Fully validated pad settings, with no write occurring during construction.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Config {
    mode: MuxSelection,
    drive: DriveSetting,
    pull: Pull,
    input_enable: bool,
    schmitt_enable: bool,
}
impl Config {
    /// Combine pad-specific settings. Reject mismatched or protected pads before MMIO.
    /// Drive codes are not converted into unverified current ratings.
    pub fn new(
        mode: MuxSelection,
        drive: DriveSetting,
        pull: Pull,
        input_enable: bool,
        schmitt_enable: bool,
    ) -> Result<Self, Error> {
        if mode.pad() != drive.pad() {
            return Err(Error::MismatchedPad);
        }
        ensure_unprotected(mode.pad())?;
        Ok(Self { mode, drive, pull, input_enable, schmitt_enable })
    }
    /// Physical pad to which every setting belongs.
    pub fn pad(self) -> Pad {
        self.mode.pad()
    }
    /// Validated mux setting.
    pub fn mode(self) -> MuxSelection {
        self.mode
    }
    /// Validated drive code.
    pub fn drive(self) -> DriveSetting {
        self.drive
    }
    /// Pull selection.
    pub fn pull(self) -> Pull {
        self.pull
    }
    /// Whether the digital input buffer is enabled.
    pub fn input_enabled(self) -> bool {
        self.input_enable
    }
    /// Schmitt enable bit; no hysteresis voltages are inferred.
    pub fn schmitt_enabled(self) -> bool {
        self.schmitt_enable
    }
}

fn ensure_unprotected(pad: Pad) -> Result<(), Error> {
    if pad == Pad::Reset || pad.is_debug() { Err(Error::ProtectedPad) } else { Ok(()) }
}

/// Exclusive pad/mux permission, separate from the GPIO controller singleton.
///
/// ```compile_fail
/// use hisi_hal::pinctrl::PadControl;
/// fn copy(owner: PadControl<'_>) { let alias = owner.clone(); }
/// ```
pub struct PadControl<'d> {
    pad: Pad,
    _ownership: PhantomData<&'d mut ()>,
}
impl<'d> PadControl<'d> {
    /// Accept a quiesced pad from an external resource handoff; performs no MMIO.
    ///
    /// # Safety
    /// Own this physical pad and every alias exclusively for `'d`. Stop all old
    /// CPU, peripheral and control-core users, and reserve any signal to which
    /// you may route it. Old output drivers must be disabled before reconfiguring.
    /// Its pad, mux and GPIO register clocks/power must remain available. No
    /// concurrent raw/PAC/AnyPin/PadControl access is allowed. External wiring
    /// must tolerate the chosen settings and their non-atomic transition.
    /// For transfer to GPIO, its GPIO_SW_CTL source must already select software
    /// control (the selection encoding is not yet evidenced and is not written).
    pub unsafe fn claim(pad: Pad) -> Result<Self, Error> {
        ensure_unprotected(pad)?;
        Ok(Self { pad, _ownership: PhantomData })
    }
    /// Canonical identity; does not duplicate ownership.
    pub fn pad(&self) -> Pad {
        self.pad
    }
    /// Update pad fields, then mux, retaining reserved bits in both words.
    /// GPIO/peripheral output drivers must be quiesced by the ownership contract.
    /// Invalid pad association is rejected before accessing any registers.
    pub fn configure(&mut self, config: Config) -> Result<(), Error> {
        if config.pad() != self.pad {
            return Err(Error::MismatchedPad);
        }
        write_config(self.pad, config);
        Ok(())
    }
    /// Decode the currently configured fields, rejecting unsupported encodings.
    pub fn configuration(&self) -> Result<Config, Error> {
        read_config(self.pad)
    }
    /// Move ownership into the digital GPIO layer after checking mode and input.
    /// On failure the owner is returned for correction; no configuration changes occur.
    pub fn into_gpio(self) -> Result<AnyPin<'d>, (Error, Self)> {
        let config = match self.configuration() {
            Ok(config) => config,
            Err(error) => return Err((error, self)),
        };
        if config.mode().mode().code() != 0 || !config.input_enabled() {
            return Err((Error::GpioNotPrepared, self));
        }
        // claim's safety contract covers ownership, source/clock and external
        // wiring; configure/readback supply the previously missing pad setup.
        match unsafe { AnyPin::from_configured(self.pad) } {
            Ok(pin) => Ok(pin),
            Err(PinError::DebugReserved | PinError::NotGpio) => Err((Error::ProtectedPad, self)),
            Err(PinError::WrongBank) => unreachable!("AnyPin attachment does not select a bank"),
        }
    }
}

// Both drive portions are written in ONE RMW on extended pads. No partial
// write-before-validation behavior from the SDK setter is inherited.
macro_rules! write_pad {
    ($r:expr, $cfg:expr, ordinary) => {
        $r.modify(|_, w| unsafe {
            w.drive_low()
                .bits($cfg.drive.code())
                .pull()
                .bits($cfg.pull as u8)
                .input_enable()
                .bit($cfg.input_enable)
                .schmitt_enable()
                .bit($cfg.schmitt_enable)
        })
    };
    ($r:expr, $cfg:expr, extended) => {
        $r.modify(|_, w| unsafe {
            w.drive_low()
                .bits($cfg.drive.code() & 3)
                .drive_high()
                .bits($cfg.drive.code() >> 2)
                .pull()
                .bits($cfg.pull as u8)
                .input_enable()
                .bit($cfg.input_enable)
                .schmitt_enable()
                .bit($cfg.schmitt_enable)
        })
    };
}
macro_rules! drive_code {
    ($r:expr, ordinary) => {
        $r.drive_low().bits()
    };
    ($r:expr, extended) => {
        $r.drive_low().bits() | ($r.drive_high().bits() << 2)
    };
}
fn decode(pad: Pad, mode: u8, drive: u8, pull: u8, input: bool, schmitt: bool) -> Result<Config, Error> {
    let mode = MuxMode::from_code(mode).and_then(|mode| pad.select_mode(mode)).ok_or(Error::UnsupportedReadback)?;
    let drive = pad.drive_setting(drive).ok_or(Error::UnsupportedReadback)?;
    let pull = match pull {
        0 => Pull::None,
        1 => Pull::Up,
        2 => Pull::Down,
        _ => return Err(Error::UnsupportedReadback),
    };
    Config::new(mode, drive, pull, input, schmitt)
}

macro_rules! register_map {
    ($($pad:ident, $pad_block:ident, $pad_reg:ident, $drive:ident, $mux_block:ident, $mux_reg:ident;)+) => {
        fn write_config(pad: Pad, config: Config) {
            match pad { $(Pad::$pad => {
                let pad_regs = unsafe { &*pac::$pad_block::PTR };
                write_pad!(pad_regs.$pad_reg(), config, $drive);
                let mux_regs = unsafe { &*pac::$mux_block::PTR };
                mux_regs.$mux_reg().modify(|_, w| unsafe { w.mode().bits(config.mode.mode().code()) });
            },)+ _ => unreachable!("protected pads cannot be owned") }
        }
        fn read_config(pad: Pad) -> Result<Config, Error> {
            match pad { $(Pad::$pad => {
                let pad_regs = unsafe { &*pac::$pad_block::PTR };
                let r = pad_regs.$pad_reg().read();
                let mux_regs = unsafe { &*pac::$mux_block::PTR };
                let mode = mux_regs.$mux_reg().read().mode().bits();
                decode(pad, mode, drive_code!(r, $drive), r.pull().bits(),
                       r.input_enable().bit(), r.schmitt_enable().bit())
            },)+ _ => Err(Error::ProtectedPad) }
        }
    };
}
// PAC accessors, never numeric MMIO pointers. Aliases use one canonical arm.
register_map! {
    Mgpio0, PinmuxAon, mgpio0_pad, extended, PinmuxMcu, mgpio0_sel;
    Mgpio2, PinmuxAon, mgpio2_pad, extended, PinmuxMcu, mgpio2_sel;
    Mgpio3, PinmuxAon, mgpio3_pad, extended, PinmuxMcu, mgpio3_sel;
    Mgpio4, PinmuxAon, mgpio4_pad, extended, PinmuxMcu, mgpio4_sel;
    Mgpio5, PinmuxAon, mgpio5_pad, ordinary, PinmuxMcu, mgpio5_sel;
    Mgpio8, PinmuxAon, mgpio8_pad, ordinary, PinmuxMcu, mgpio8_sel;
    Mgpio9, PinmuxAon, mgpio9_pad, ordinary, PinmuxMcu, mgpio9_sel;
    Mgpio10, PinmuxAon, mgpio10_pad, ordinary, PinmuxMcu, mgpio10_sel;
    Mgpio12, PinmuxAon, mgpio12_pad, ordinary, PinmuxMcu, mgpio12_sel;
    Mgpio13, PinmuxAon, mgpio13_pad, ordinary, PinmuxMcu, mgpio13_sel;
    Mgpio17, PinmuxAon, mgpio17_pad, ordinary, PinmuxMcu, mgpio17_sel;
    Mgpio18, PinmuxAon, mgpio18_pad, ordinary, PinmuxMcu, mgpio18_sel;
    Mgpio19, PinmuxAon, mgpio19_pad, ordinary, PinmuxMcu, mgpio19_sel;
    Mgpio20, PinmuxAon, mgpio20_pad, ordinary, PinmuxMcu, mgpio20_sel;
    Mgpio22, PinmuxAon, mgpio22_pad, ordinary, PinmuxMcu, mgpio22_sel;
    Agpio0, PinmuxAon, agpio0_pad, ordinary, PinmuxAon, agpio0_sel;
    Agpio1, PinmuxAon, agpio1_pad, ordinary, PinmuxAon, agpio1_sel;
    Agpio2, PinmuxAon, agpio2_pad, ordinary, PinmuxAon, agpio2_sel;
    Agpio3, PinmuxAon, agpio3_pad, ordinary, PinmuxAon, agpio3_sel;
    Agpio4, PinmuxAon, agpio4_pad, ordinary, PinmuxAon, agpio4_sel;
    Agpio5, PinmuxAon, agpio5_pad, extended, PinmuxAon, agpio5_sel;
    Agpio6, PinmuxAon, agpio6_pad, extended, PinmuxAon, agpio6_sel;
    Agpio7, PinmuxAon, agpio7_pad, ordinary, PinmuxAon, agpio7_sel;
    Agpio8, PinmuxAon, agpio8_pad, ordinary, PinmuxAon, agpio8_sel;
    Agpio9, PinmuxAon, agpio9_pad, ordinary, PinmuxAon, agpio9_sel;
    Agpio10, PinmuxAon, agpio10_pad, ordinary, PinmuxAon, agpio10_sel;
    Agpio11, PinmuxAon, agpio11_pad, ordinary, PinmuxAon, agpio11_sel;
    Sgpio2, PinmuxTzpc, mgpio21_pad, ordinary, PinmuxMcu, mgpio21_sel;
}

#[cfg(all(test, not(target_arch = "riscv32")))]
mod tests {
    use super::*;
    #[test]
    fn mismatched_and_protected_settings_fail_without_mmio() {
        let gpio = MuxMode::from_code(0).unwrap();
        assert_eq!(
            Config::new(
                Pad::Agpio1.select_mode(gpio).unwrap(),
                Pad::Mgpio0.drive_setting(15).unwrap(),
                Pull::Up,
                true,
                true
            ),
            Err(Error::MismatchedPad)
        );
        assert_eq!(
            Config::new(
                Pad::Sgpio0.select_mode(gpio).unwrap(),
                Pad::Sgpio0.drive_setting(3).unwrap(),
                Pull::None,
                true,
                false
            ),
            Err(Error::ProtectedPad)
        );
        for pad in [Pad::Sgpio0, Pad::Sgpio1, Pad::Reset] {
            assert!(matches!(unsafe { PadControl::claim(pad) }, Err(Error::ProtectedPad)));
        }
        let mut owner = unsafe { PadControl::claim(Pad::Agpio1) }.unwrap();
        let other = Config::new(
            Pad::Mgpio0.select_mode(gpio).unwrap(),
            Pad::Mgpio0.drive_setting(15).unwrap(),
            Pull::Down,
            false,
            true,
        )
        .unwrap();
        assert_eq!(owner.configure(other), Err(Error::MismatchedPad));
    }
    #[test]
    fn decode_rejects_reserved_pull_mode_and_drive_before_changes() {
        assert_eq!(decode(Pad::Agpio1, 2, 0, 0, true, false), Err(Error::UnsupportedReadback));
        assert_eq!(decode(Pad::Agpio1, 0, 4, 0, true, false), Err(Error::UnsupportedReadback));
        assert_eq!(decode(Pad::Mgpio0, 0, 16, 0, true, false), Err(Error::UnsupportedReadback));
        assert_eq!(decode(Pad::Mgpio0, 0, 15, 3, true, false), Err(Error::UnsupportedReadback));
        let c = decode(Pad::Mgpio0, 7, 15, 2, true, true).unwrap();
        assert_eq!(c.drive().code(), 15);
        assert_eq!(c.pull(), Pull::Down);
        assert!(c.input_enabled() && c.schmitt_enabled());
    }
}
