//! WS53 pad identities from fbb_ws53 163d9150, application-core profile.
//!
//! `MGPIO`, `AGPIO`, SDK `pin_t`, GPIO bank bits, and board `MIO/AIO` labels
//! are different namespaces. In particular MGPIO1 = AGPIO6 = SDK 38 = GPIO1
//! bit6. A board label must be resolved by its board description, not cast to
//! an SDK number. These copyable values describe identity, not ownership;
//! they perform no MMIO and cannot grant access to shared pads or SWD pins.
//!
//! See `scripts/check-ws53-pins.py` for an optional comparison with the original
//! SDK enum/mux table. Physical bonding, clock state and electrical operation
//! still require board validation. Flash pads outside this profile are absent.

macro_rules! pin_numbers {
    ($($name:ident, $max:literal, $description:literal;)+) => {$(
        #[doc = $description]
        #[derive(Debug, Clone, Copy, PartialEq, Eq)]
        pub struct $name(u8);
        impl $name {
            /// Parse a chip pad suffix, not an SDK ID or board connector number.
            pub const fn from_index(index: u8) -> Option<Self> {
                if index <= $max { Some(Self(index)) } else { None }
            }
            /// The chip pad suffix in this namespace.
            pub const fn index(self) -> u8 { self.0 }
        }
    )+};
}
pin_numbers! {
    Mgpio, 22, "MGPIO0..22 chip names, including aliases routed through AON/TZPC.";
    Agpio, 12, "AGPIO0..12 chip names; AGPIO12 is reset, not GPIO.";
    Sgpio, 2, "SGPIO0..2 chip names; SGPIO0/1 share the SWD pads.";
}

macro_rules! pads {
    ($($pad:ident, $id:literal, $modes:literal, $drive_bits:literal;)+) => {
        /// Canonical physical pad identity within the selected SDK profile.
        ///
        /// Aliases normalize to a single value. Reset remains identifiable but
        /// has no GPIO line or mux selection. No value is an ownership token.
        #[derive(Debug, Clone, Copy, PartialEq, Eq)]
        #[repr(u8)]
        pub enum Pad { $(#[doc = concat!(stringify!($pad), ": SDK pin_t ", stringify!($id), ".")]
            $pad = $id,)+ }
        impl Pad {
            /// All 31 canonical pads, including reset and the two SWD pads.
            pub const ALL: [Self; 31] = [$(Self::$pad,)+];
            const fn from_sdk_raw(raw: u8) -> Option<Self> {
                match raw { $($id => Some(Self::$pad),)+ _ => None }
            }
            const fn mode_mask(self) -> u8 { match self { $(Self::$pad => $modes,)+ } }
            const fn drive_bits(self) -> u8 { match self { $(Self::$pad => $drive_bits,)+ } }
        }
    };
}
// Canonical name, SDK pin_t, porting mux acceptance mask, pad drive-code bits.
// GPIO0 holes are deliberately not represented by SDK IDs 1/6/7/11/14/15/16/21.
pads! {
    Mgpio0, 0, 0xff, 4;
    Mgpio2, 2, 0x87, 4;
    Mgpio3, 3, 0x87, 4;
    Mgpio4, 4, 0x87, 4;
    Mgpio5, 5, 0x83, 2;
    Mgpio8, 8, 0x9f, 2;
    Mgpio9, 9, 0x9f, 2;
    Mgpio10, 10, 0xf7, 2;
    Mgpio12, 12, 0xf7, 2;
    Mgpio13, 13, 0xbf, 2;
    Mgpio17, 17, 0xef, 2;
    Mgpio18, 18, 0xff, 2;
    Mgpio19, 19, 0xef, 2;
    Mgpio20, 20, 0xef, 2;
    Mgpio22, 22, 0xef, 2;
    Agpio0, 32, 0x9f, 2;
    Agpio1, 33, 0x83, 2;
    Agpio2, 34, 0x87, 2;
    Agpio3, 35, 0x87, 2;
    Agpio4, 36, 0x87, 2;
    Agpio5, 37, 0x87, 4;
    Agpio6, 38, 0xff, 4;
    Agpio7, 39, 0xb7, 2;
    Agpio8, 40, 0x87, 2;
    Agpio9, 41, 0xdf, 2;
    Agpio10, 42, 0x83, 2;
    Agpio11, 43, 0x83, 2;
    Reset, 44, 0, 2;
    Sgpio0, 45, 0xff, 2;
    Sgpio1, 46, 0xf7, 2;
    Sgpio2, 47, 0xff, 2;
}

/// A recognized SDK `pin_t` identity. Holes, flash pads and PIN_NONE are rejected.
///
/// SDK IDs cannot silently pass as MGPIO suffixes:
/// ```compile_fail
/// use hisi_hal::soc::ws53::pins::{Pad, SdkPinId};
/// let pad = Pad::from_mgpio(SdkPinId::from_raw(38).unwrap());
/// ```
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SdkPinId(Pad);
impl SdkPinId {
    /// Parse a numeric SDK ID; e.g. 38 is valid, 1 is not an alias for 38.
    pub const fn from_raw(raw: u8) -> Option<Self> {
        match Pad::from_sdk_raw(raw) {
            Some(pad) => Some(Self(pad)),
            None => None,
        }
    }
    /// Number for explicit SDK interoperability; not a GPIO bit number.
    pub const fn number(self) -> u8 {
        self.0 as u8
    }
    /// Canonical pad denoted by this SDK ID.
    pub const fn pad(self) -> Pad {
        self.0
    }
}

/// One of the three GPIO register banks (not a pad power or pinmux domain).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GpioBank {
    /// Application GPIO0, IRQ34.
    Bank0,
    /// AON GPIO1, IRQ33.
    Bank1,
    /// Secure GPIO2, IRQ29.
    Bank2,
}
impl GpioBank {
    /// Absolute local interrupt cause number.
    pub const fn interrupt(self) -> super::Interrupt {
        match self {
            Self::Bank0 => super::Interrupt::GPIO_0,
            Self::Bank1 => super::Interrupt::GPIO_1,
            Self::Bank2 => super::Interrupt::SGPIO_INT,
        }
    }
}

/// A GPIO line derived from a canonical pad; arbitrary register bits cannot be constructed.
/// ```compile_fail
/// use hisi_hal::soc::ws53::pins::{GpioBank, GpioLine};
/// let reserved = GpioLine { bank: GpioBank::Bank0, bit: 1 };
/// ```
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct GpioLine {
    bank: GpioBank,
    bit: u8,
}
impl GpioLine {
    /// Controller that owns this bit.
    pub const fn bank(self) -> GpioBank {
        self.bank
    }
    /// Position in that controller's bitmaps, not a chip pad suffix.
    pub const fn bit(self) -> u8 {
        self.bit
    }
    /// One-bit mask for this line.
    pub const fn mask(self) -> u32 {
        1 << self.bit
    }
}

/// Encoded mux mode; each pad accepts only a subset of 0..7.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MuxMode(u8);
impl MuxMode {
    /// Reject values outside the hardware's three-bit field.
    pub const fn from_code(code: u8) -> Option<Self> {
        if code < 8 { Some(Self(code)) } else { None }
    }
    /// Register encoding. Its signal name depends on the pad.
    pub const fn code(self) -> u8 {
        self.0
    }
}

/// A pad-specific mux choice that passed the SDK mode table, not an applied configuration.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MuxSelection {
    pad: Pad,
    mode: MuxMode,
}
impl MuxSelection {
    /// The pad for which this mode is valid.
    pub const fn pad(self) -> Pad {
        self.pad
    }
    /// The validated encoding.
    pub const fn mode(self) -> MuxMode {
        self.mode
    }
}

/// A pad-specific drive code; no current in mA is inferred from the code.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DriveSetting {
    pad: Pad,
    code: u8,
}
impl DriveSetting {
    /// Pad for which this drive code is representable.
    pub const fn pad(self) -> Pad {
        self.pad
    }
    /// Untruncated two- or four-bit drive code.
    pub const fn code(self) -> u8 {
        self.code
    }
}

impl Pad {
    /// Normalize a chip MGPIO name, including AON and secure aliases.
    pub const fn from_mgpio(pin: Mgpio) -> Self {
        let pads: [Pad; 23] = [
            Self::Mgpio0,
            Self::Agpio6,
            Self::Mgpio2,
            Self::Mgpio3,
            Self::Mgpio4,
            Self::Mgpio5,
            Self::Agpio0,
            Self::Agpio9,
            Self::Mgpio8,
            Self::Mgpio9,
            Self::Mgpio10,
            Self::Agpio7,
            Self::Mgpio12,
            Self::Mgpio13,
            Self::Sgpio0,
            Self::Sgpio1,
            Self::Agpio8,
            Self::Mgpio17,
            Self::Mgpio18,
            Self::Mgpio19,
            Self::Mgpio20,
            Self::Sgpio2,
            Self::Mgpio22,
        ];
        pads[pin.0 as usize]
    }
    /// Normalize a chip AGPIO name; AGPIO12 denotes the reset pad.
    pub const fn from_agpio(pin: Agpio) -> Self {
        let pads: [Pad; 13] = [
            Self::Agpio0,
            Self::Agpio1,
            Self::Agpio2,
            Self::Agpio3,
            Self::Agpio4,
            Self::Agpio5,
            Self::Agpio6,
            Self::Agpio7,
            Self::Agpio8,
            Self::Agpio9,
            Self::Agpio10,
            Self::Agpio11,
            Self::Reset,
        ];
        pads[pin.0 as usize]
    }
    /// Normalize a chip SGPIO name; 0/1 are also the SWD pads.
    pub const fn from_sgpio(pin: Sgpio) -> Self {
        [Self::Sgpio0, Self::Sgpio1, Self::Sgpio2][pin.0 as usize]
    }
    /// Explicit SDK identity, with all chip aliases normalized.
    pub const fn sdk_id(self) -> SdkPinId {
        SdkPinId(self)
    }
    /// Corresponding controller bit. Reset cannot be used as GPIO.
    pub const fn gpio_line(self) -> Option<GpioLine> {
        let id = self as u8;
        if id < 23 {
            Some(GpioLine { bank: GpioBank::Bank0, bit: id })
        } else if id < 44 {
            Some(GpioLine { bank: GpioBank::Bank1, bit: id - 32 })
        } else if id > 44 {
            Some(GpioLine { bank: GpioBank::Bank2, bit: id - 45 })
        } else {
            None
        }
    }
    /// True for SWDIO/SWCLK. This flag does not release the debugger's ownership.
    pub const fn is_debug(self) -> bool {
        matches!(self, Self::Sgpio0 | Self::Sgpio1)
    }
    /// Validate a pad-specific mux mode before any register write.
    pub const fn select_mode(self, mode: MuxMode) -> Option<MuxSelection> {
        if self.mode_mask() & (1 << mode.0) != 0 { Some(MuxSelection { pad: self, mode }) } else { None }
    }
    /// Validate the full drive code before any write; never partially update then reject.
    pub const fn drive_setting(self, code: u8) -> Option<DriveSetting> {
        if code < (1 << self.drive_bits()) { Some(DriveSetting { pad: self, code }) } else { None }
    }
}

#[cfg(all(test, not(target_arch = "riscv32")))]
mod tests;
