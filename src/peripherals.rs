//! Peripheral singletons wrapping the WS63 PAC.
//!
//! Each peripheral is a zero-sized type that grants safe, exclusive access to
//! the underlying hardware registers. The [`Peripherals`] struct is obtained
//! once via [`Peripherals::take()`].

pub use crate::soc::chip::Interrupt;
use core::marker::PhantomData;

macro_rules! peripheral {
    ($name:ident, $pac_ty:ty) => {
        #[doc = concat!("Peripheral singleton for ", stringify!($name))]
        #[derive(Debug)]
        pub struct $name<'d> {
            _marker: PhantomData<&'d ()>,
        }

        impl<'d> $name<'d> {
            /// Unsafely create a peripheral instance.
            ///
            /// # Safety
            /// Must not create multiple instances of the same peripheral.
            #[inline]
            pub unsafe fn steal() -> Self {
                Self { _marker: PhantomData }
            }

            /// Pointer to the PAC register block.
            #[inline]
            pub fn ptr() -> *const <$pac_ty as core::ops::Deref>::Target {
                <$pac_ty>::PTR
            }

            /// Return a reference to the PAC register block.
            ///
            /// # Safety
            /// The PAC pointer must be valid. It points to a static physical
            /// MMIO address provided by the svd2rust-generated PAC.
            #[inline]
            pub fn register_block(&self) -> &<$pac_ty as core::ops::Deref>::Target {
                // SAFETY: PAC peripheral pointer is a static physical MMIO address, always valid
                unsafe { &*Self::ptr() }
            }
        }

        unsafe impl<'d> Send for $name<'d> {}
    };
}

macro_rules! peripherals {
    ($($field:ident => $ty:ident),* $(,)?) => {
        #[allow(non_snake_case)]
        pub struct Peripherals {
            $(
                #[doc = concat!("`", stringify!($field), "` peripheral")]
                pub $field: $ty<'static>,
            )*
        }

        impl Peripherals {
            pub fn take() -> Option<Self> {
                let pac = crate::soc::pac::Peripherals::take()?;
                Some(Self::from_pac(pac))
            }

            /// Unchecked version of `take`. Does not check singleton.
            ///
            /// # Safety
            ///
            /// Each peripheral must be used at most once.
            pub unsafe fn steal() -> Self {
                let pac = unsafe { crate::soc::pac::Peripherals::steal() };
                Self::from_pac(pac)
            }

            fn from_pac(_pac: crate::soc::pac::Peripherals) -> Self {
                unsafe {
                    Self {
                        $(
                            $field: $ty::steal(),
                        )*
                    }
                }
            }
        }
    };
}

// ── Wrappers for PAC types present in EVERY chip (shared) ───────────────────
peripheral!(GlbCtlM, crate::soc::pac::GlbCtlM);
peripheral!(Gpio0, crate::soc::pac::Gpio0);
peripheral!(Gpio1, crate::soc::pac::Gpio1);
peripheral!(Gpio2, crate::soc::pac::Gpio2);
peripheral!(UlpGpio, crate::soc::pac::UlpGpio);
peripheral!(Uart0, crate::soc::pac::Uart0);
peripheral!(Uart1, crate::soc::pac::Uart1);
peripheral!(Uart2, crate::soc::pac::Uart2);
peripheral!(I2c0, crate::soc::pac::I2c0);
peripheral!(I2c1, crate::soc::pac::I2c1);
peripheral!(Spi0, crate::soc::pac::Spi0);
peripheral!(Spi1, crate::soc::pac::Spi1);
peripheral!(Pwm, crate::soc::pac::Pwm);
peripheral!(Dma, crate::soc::pac::Dma);
peripheral!(Sdma, crate::soc::pac::Sdma);
peripheral!(Timer, crate::soc::pac::Timer);
peripheral!(Wdt, crate::soc::pac::Wdt);
peripheral!(Rtc, crate::soc::pac::Rtc);
peripheral!(Tcxo, crate::soc::pac::Tcxo);
peripheral!(Trng, crate::soc::pac::Trng);

// ── WS63-only PAC types ─────────────────────────────────────────────────────
#[cfg(feature = "chip-ws63")]
mod ws63_only {
    use super::PhantomData;
    peripheral!(SysCtl0, crate::soc::pac::SysCtl0);
    peripheral!(SysCtl1, crate::soc::pac::SysCtl1);
    peripheral!(CldoCrg, crate::soc::pac::CldoCrg);
    peripheral!(IoConfig, crate::soc::pac::IoConfig);
    peripheral!(I2s, crate::soc::pac::I2s);
    peripheral!(Lsadc, crate::soc::pac::Lsadc);
    peripheral!(SfcCfg, crate::soc::pac::SfcCfg);
    peripheral!(Tsensor, crate::soc::pac::Tsensor);
    peripheral!(Efuse, crate::soc::pac::Efuse);
    peripheral!(Spacc, crate::soc::pac::Spacc);
    peripheral!(Pke, crate::soc::pac::Pke);
    peripheral!(Km, crate::soc::pac::Km);
    peripheral!(RfWbCtl, crate::soc::pac::RfWbCtl);
    peripheral!(ShareMemCtl, crate::soc::pac::ShareMemCtl);
    peripheral!(FamaRemap, crate::soc::pac::FamaRemap);
}
#[cfg(feature = "chip-ws63")]
pub use ws63_only::*;

// ── BS21-only PAC types (extra GPIO banks + SPI bus + the GADC) ──────────────
#[cfg(feature = "chip-bs21")]
mod bs21_only {
    use super::PhantomData;
    peripheral!(Gpio3, crate::soc::pac::Gpio3);
    peripheral!(Gpio4, crate::soc::pac::Gpio4);
    peripheral!(Spi2, crate::soc::pac::Spi2);
    // GADC — the BS2X 13-bit ADC (v153). No WS63 analogue (WS63 has LSADC v154),
    // so it is chip-bs21-only; the `gadc` driver wraps it.
    peripheral!(Gadc, crate::soc::pac::Gadc);
    // BS2X-only HID peripherals (no WS63 analogue): the key-matrix scanner and the
    // quadrature decoder. Drivers: `keyscan`, `qdec`.
    peripheral!(Keyscan, crate::soc::pac::Keyscan);
    peripheral!(Qdec, crate::soc::pac::Qdec);
    // PDM — the BS2X PDM-microphone audio front-end. Driver: `pdm`.
    peripheral!(Pdm, crate::soc::pac::Pdm);
    // USB 2.0 OTG (Synopsys DWC OTG). Driver: `usb` (config-level: core-ID).
    peripheral!(Usb, crate::soc::pac::Usb);
}
#[cfg(feature = "chip-bs21")]
pub use bs21_only::*;

#[cfg(feature = "chip-ws63")]
peripherals!(
    SYS_CTL0 => SysCtl0,
    SYS_CTL1 => SysCtl1,
    GLB_CTL_M => GlbCtlM,
    CLDO_CRG => CldoCrg,
    IO_CONFIG => IoConfig,
    GPIO0 => Gpio0,
    GPIO1 => Gpio1,
    GPIO2 => Gpio2,
    ULP_GPIO => UlpGpio,
    UART0 => Uart0,
    UART1 => Uart1,
    UART2 => Uart2,
    I2C0 => I2c0,
    I2C1 => I2c1,
    SPI0 => Spi0,
    SPI1 => Spi1,
    PWM => Pwm,
    I2S => I2s,
    LSADC => Lsadc,
    DMA => Dma,
    SDMA => Sdma,
    SFC_CFG => SfcCfg,
    TIMER => Timer,
    WDT => Wdt,
    RTC => Rtc,
    TCXO => Tcxo,
    TSENSOR => Tsensor,
    EFUSE => Efuse,
    SPACC => Spacc,
    PKE => Pke,
    KM => Km,
    TRNG => Trng,
    RF_WB_CTL => RfWbCtl,
    SHARE_MEM_CTL => ShareMemCtl,
    FAMA_REMAP => FamaRemap,
);

#[cfg(feature = "chip-bs21")]
peripherals!(
    GLB_CTL_M => GlbCtlM,
    GPIO0 => Gpio0,
    GPIO1 => Gpio1,
    GPIO2 => Gpio2,
    GPIO3 => Gpio3,
    GPIO4 => Gpio4,
    ULP_GPIO => UlpGpio,
    UART0 => Uart0,
    UART1 => Uart1,
    UART2 => Uart2,
    I2C0 => I2c0,
    I2C1 => I2c1,
    SPI0 => Spi0,
    SPI1 => Spi1,
    SPI2 => Spi2,
    PWM => Pwm,
    DMA => Dma,
    SDMA => Sdma,
    TIMER => Timer,
    WDT => Wdt,
    RTC => Rtc,
    TCXO => Tcxo,
    TRNG => Trng,
    GADC => Gadc,
    KEYSCAN => Keyscan,
    QDEC => Qdec,
    PDM => Pdm,
    USB => Usb,
);
