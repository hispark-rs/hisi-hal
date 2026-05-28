//! Peripheral singletons wrapping the WS63 PAC.
//!
//! Each peripheral is a zero-sized type that grants safe, exclusive access to
//! the underlying hardware registers. The [`Peripherals`] struct is obtained
//! once via [`Peripherals::take()`].

pub use crate::soc::ws63::Interrupt;
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
            #[inline]
            pub fn register_block(&self) -> &<$pac_ty as core::ops::Deref>::Target {
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
                let pac = ws63_pac::Peripherals::take()?;
                Some(Self::from_pac(pac))
            }

            /// Unchecked version of `take`. Does not check singleton.
            ///
            /// # Safety
            ///
            /// Each peripheral must be used at most once.
            pub unsafe fn steal() -> Self {
                let pac = unsafe { ws63_pac::Peripherals::steal() };
                Self::from_pac(pac)
            }

            fn from_pac(_pac: ws63_pac::Peripherals) -> Self {
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

peripheral!(SysCtl0, ws63_pac::SysCtl0);
peripheral!(SysCtl1, ws63_pac::SysCtl1);
peripheral!(GlbCtlM, ws63_pac::GlbCtlM);
peripheral!(CldoCrg, ws63_pac::CldoCrg);
peripheral!(IoConfig, ws63_pac::IoConfig);
peripheral!(Gpio0, ws63_pac::Gpio0);
peripheral!(Gpio1, ws63_pac::Gpio1);
peripheral!(Gpio2, ws63_pac::Gpio2);
peripheral!(UlpGpio, ws63_pac::UlpGpio);
peripheral!(Uart0, ws63_pac::Uart0);
peripheral!(Uart1, ws63_pac::Uart1);
peripheral!(Uart2, ws63_pac::Uart2);
peripheral!(I2c0, ws63_pac::I2c0);
peripheral!(I2c1, ws63_pac::I2c1);
peripheral!(Spi0, ws63_pac::Spi0);
peripheral!(Spi1, ws63_pac::Spi1);
peripheral!(Pwm, ws63_pac::Pwm);
peripheral!(I2s, ws63_pac::I2s);
peripheral!(Lsadc, ws63_pac::Lsadc);
peripheral!(Dma, ws63_pac::Dma);
peripheral!(Sdma, ws63_pac::Sdma);
peripheral!(SfcCfg, ws63_pac::SfcCfg);
peripheral!(Timer, ws63_pac::Timer);
peripheral!(Wdt, ws63_pac::Wdt);
peripheral!(Rtc, ws63_pac::Rtc);
peripheral!(Tcxo, ws63_pac::Tcxo);
peripheral!(Tsensor, ws63_pac::Tsensor);
peripheral!(Efuse, ws63_pac::Efuse);
peripheral!(Spacc, ws63_pac::Spacc);
peripheral!(Pke, ws63_pac::Pke);
peripheral!(Km, ws63_pac::Km);
peripheral!(Trng, ws63_pac::Trng);
peripheral!(RfWbCtl, ws63_pac::RfWbCtl);
peripheral!(ShareMemCtl, ws63_pac::ShareMemCtl);
peripheral!(FamaRemap, ws63_pac::FamaRemap);

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
