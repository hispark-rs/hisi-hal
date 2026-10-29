//! Peripheral singletons wrapping the WS63 PAC.
//!
//! Each peripheral is a zero-sized type that grants safe, exclusive access to
//! the underlying hardware registers. The [`Peripherals`] struct is obtained
//! once via [`Peripherals::take()`].

/// Chip-specific interrupt number enumeration re-exported from the SoC PAC.
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
            /// This is a raw PAC escape hatch: the caller must uphold the
            /// register block's aliasing, ordering, and peripheral-state
            /// invariants. Prefer the typed HAL driver APIs when available.
            #[inline]
            #[instability::unstable]
            pub unsafe fn register_block(&self) -> &<$pac_ty as core::ops::Deref>::Target {
                // SAFETY: PAC peripheral pointer is a static physical MMIO address, always valid
                unsafe { &*Self::ptr() }
            }

            /// Reborrow this peripheral with a shorter lifetime.
            ///
            /// Lets you hand the token to a driver constructor and keep using the
            /// peripheral afterwards, without `unsafe steal()` — the borrow checker
            /// enforces that the reborrow does not outlive `self`. (esp-hal
            /// `reborrow` idiom; SemVer-safe additive helper.)
            #[inline]
            pub fn reborrow(&mut self) -> $name<'_> {
                $name { _marker: PhantomData }
            }
        }

        unsafe impl<'d> Send for $name<'d> {}
    };
}

#[cfg(not(feature = "chip-ws53"))]
macro_rules! peripherals {
    ($($(#[$meta:meta])* $field:ident => $ty:ident),* $(,)?) => {
        /// Owns every peripheral singleton for the selected chip; obtain once via [`Peripherals::take()`].
        #[allow(non_snake_case)]
        pub struct Peripherals {
            $(
                $(#[$meta])*
                #[doc = concat!("`", stringify!($field), "` peripheral")]
                pub $field: $ty<'static>,
            )*
        }

        impl Peripherals {
            /// Take the peripheral singletons; returns `None` if already taken.
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
                            $(#[$meta])*
                            $field: $ty::steal(),
                        )*
                    }
                }
            }
        }
    };
}

/// A reserved Rust token whose ownership still belongs to the WS53 boot
/// environment. It cannot be used by a driver until explicitly handed over.
#[cfg(feature = "chip-ws53")]
pub struct Unclaimed<T> {
    token: T,
}

#[cfg(feature = "chip-ws53")]
impl<T> Unclaimed<T> {
    /// Accept an externally negotiated application-core resource handoff.
    ///
    /// # Safety
    /// Stop all previous CPU/DMA/IRQ users of this resource and establish that
    /// the vendor control core will not access it. Its clock must remain active
    /// while the returned token is used. TIMER requires the SDK's 32 MHz clock;
    /// UART attachment additionally requires configured pads (or explicitly
    /// owned internal loopback), divider and data mode. UART1 may instead be
    /// passed unclaimed to `uart::Uart::initialize_h0` under that constructor's
    /// separate shared clock/reset handoff; generic attachment does not enable it.
    /// I2C tokens may be passed directly to `I2c::new_i2c0/new_i2c1`: those
    /// constructors enable the audited instance gate before peripheral MMIO.
    /// Their bus/pad power, independent delay and stable upstream32MHz source
    /// must already exist; no raw register access may precede construction.
    /// GPIO requires all previous bank interrupt sources to be quiesced; physical
    /// pad ownership and mux/input/pull/drive configuration are handed over
    /// separately through `gpio::AnyPin::from_configured`.
    /// This call does not perform IPC negotiation or clock/pin configuration.
    /// SPACC requires coordinating HASH, SYMC and shared IRQ59 across both
    /// cores, draining all old DMA work and excluding other lock/error/IRQ word
    /// users. The HASH transfer constructor imposes additional configured
    /// profile and retained-memory obligations; this token alone proves none.
    /// KM requires exclusive ownership of the whole loader, all key slots,
    /// selectors and IRQ61. Stop prior common/root/clear routes and drain all
    /// previous key consumers, including Flash decryption. Its configured
    /// constructor separately requires a qualified completion/flush contract;
    /// this handoff neither loads nor erases keys and does not program OTP.
    /// ADC v155 requires exclusive analog/reset/IRQ80 ownership, powered analog
    /// circuitry and a working source clock. Stop all old scans and coordinate
    /// MCU_DIAG/CPU trace, their RAM writes and any other core before handoff.
    /// Serialize complete M_CLKEN1/GLOBAL_CLKEN halfword and SOFT_RESET1 word
    /// RMW operations with other users; retain bus access and independent delay
    /// through calibration/error cleanup. ADC calibration does not grant pad or
    /// diagnostic-buffer ownership and does not prove a capture writer drained.
    /// MCU_DIAG additionally requires all ADC, CPU-trace and control-core bus
    /// writers stopped and drained, CPU_TRACE and SAMPLE_MODE enable cleared,
    /// exclusive ownership of its entire monitor/source/address/clock register
    /// bank, and permission to use source6. Keep its bus/clock accessible through
    /// stop and Drop. Reservation alone neither stops nor drains an old writer.
    /// MDMA requires exclusive ownership of all eight channels, the controller
    /// and IRQ58, with old linked lists/requests/CPU/control-core users stopped
    /// and all source reads/destination writes drained. Establish released reset,
    /// working bus clock, little-endian master0 and the v151 memory-transfer
    /// profile (8/16/32-bit elements, increment bits, raw TC/error semantics).
    /// Clock-tree handoff remains external: the generic clock enums conflict
    /// with the selected ROM port. Keep MMIO/clock accessible through cleanup;
    /// prevent low-power transitions and shared reset while any channel exists.
    /// ENABLED/ACTIVE or CPU halt alone is not proof of physical bus quiescence.
    /// WDT additionally requires released reset, both clock gates enabled, and
    /// the clock frequency supplied to its Config to remain accurate. Its NMI
    /// source must be quiesced and exclusively handed over, including the NMI
    /// hook; ordinary IRQ masking does not exclude an NMI. The hook must not
    /// borrow a main-context driver or use locks held by interrupted code.
    /// TCXO requires exclusive use of the application snapshot window and
    /// coordination of the underlying counter/epoch with all other users,
    /// including NMI and the control core's different window. No other user may
    /// refresh, clear, change the clock or suspend the counter while it is used.
    /// RTC0/1 require the selected 32-bit profile, released reset/FSM, a working
    /// 32768 Hz input and accessible bus clock. Quiesce previous alarms/handlers
    /// and establish exclusive channel and local-IRQ ownership. The common bank
    /// and vendor sleep scheduler must not change the channel or its clock.
    /// PWM requires all eight outputs, DMA requests, groups and both local IRQs
    /// quiesced and exclusively handed over. Own/serialize the complete M_CLKEN0
    /// and PWM_M_DIV3 halfword transactions against all other cores/handlers;
    /// the driver provisions these clocks using the reference divider profile.
    /// Establish the source that yields 32 MHz at divider16, released reset and
    /// zero high count halves. Keep register clocks and an independent delay
    /// provider operational through Drop. Physical pad/routes are separate.
    /// I2C0/1 require the non-PRO v151 profile, 8-entry FIFOs, released reset,
    /// 32 MHz upstream clock and valid inherited SDA hold/setup timing. Verify
    /// the SDK's disabled-controller CON addressing selection is effective;
    /// dynamic TAR addressing selection is not qualified by this driver. Quiesce
    /// all old bus/IRQ/DMA users and disable optional SMBus/special modes. Own
    /// and serialize M_CLKEN1 halfword RMW against other cores/handlers. The
    /// driver controls its gate, IRQ and DMA requests; it does not negotiate
    /// pads or release a physically stuck bus. Keep bus MMIO and an independent
    /// delay provider accessible through bounded shutdown and external recovery.
    /// SPI0/QSPI1 require the v151_100 profile, released reset and a 32 MHz
    /// source after the documented port gate configuration. Own the complete
    /// M_CLKEN1/SPI_CLOCK/SPI_DEVICE_MODE or XIP_CLOCK_ENABLE/XIP_QSPI_CLOCK/
    /// GLOBAL_CLKEN halfword transactions, serialized against all other users.
    /// For QSPI1, no core, cache, DMA or instruction/data fetch may use its XIP
    /// path; the Rust program and recovery path must be independent of it.
    /// Quiesce bus/IRQ/DMA users and inherited optional extensions before handoff.
    /// Pad routing and chip-select management are separate. Bus MMIO and the
    /// independent delay provider must remain accessible through shutdown.
    /// TSENSOR requires exclusive thermal-service coordination, powered analog
    /// circuitry, released reset and the inherited 32kHz sampling clock. Confirm
    /// that CONTROL=3 denotes the complete source profile (sample time=2,
    /// refresh period=160, refresh configuration=0), or grant permission to
    /// initialize that profile when CONTROL=0. No evidenced stop/clear/IRQ
    /// operation exists; the sampler and its clocks remain active across Drop
    /// and token handback. Other cores/firmware must not concurrently reconfigure
    /// or rely on uninterrupted sampling during a permitted initialization.
    /// I2S requires the source v151 stereo/split-FIFO profile, eight entries per
    /// side, working bus/audio clocks, and exclusive I2S/IRQ67 ownership. Stop
    /// all previous CPU/IRQ/DMA users, especially DMA requests12/13, and keep
    /// them disabled throughout the driver's lifetime. Own/serialize complete
    /// 32-bit AUDIO_CMU.NEW_CFG0 and M_CLKEN0 transactions against other cores,
    /// interrupts and halfword users. Grant permission to release the shared
    /// audio-divider reset without pulsing it, and to reset this I2S block.
    /// Clocks are left enabled on Drop. Pad routes and external clock direction
    /// are handed over separately; keep MMIO/delay accessible through recovery.
    /// TRNG requires exclusive FIFO consumption and coordination with the SDK
    /// random-buffer IPC service and all security clients. No other core,
    /// handler or service may consume/reconfigure it during the handoff. Keep
    /// the unified-security source clock/reset/ring profile stable; the ring
    /// must be enabled and settled before use. This is not IPC negotiation or
    /// proof of entropy quality. See `trng::TrngDriver::from_configured` for
    /// the additional two-word FIFO admission obligation.
    /// EFUSE requires exclusive read-service ownership, no active programming,
    /// completed region0 boot and the settled SDK timing/read-mode profile.
    /// Coordinate boot/security/control-core users and protect the complete
    /// mode register transaction, including unmodeled upper bits. The read HAL
    /// does not power or initialize the controller and never programs OTP.
    /// SFC additionally requires the RAM-only, configured-controller, NOR and
    /// cross-core/cache/DMA/XIP obligations of `sfc::ReadOnly::from_configured`.
    /// This generic handoff alone does not establish those obligations; SFC
    /// construction remains unsafe and independently checks the readable state.
    /// SDIO requires exclusive application/control-core/IRQ46 ownership and all
    /// previous ADMA operations physically drained. Its own ADMA is independent
    /// of MDMA. Clock words, pads and host traffic need the additional handoff
    /// in `sdio::Initialization::begin`; the generic token does not establish it.
    /// IPC reserves only the application-side doorbell fields and IRQ26/27,
    /// not the rest of M_CRG or the peer's register view. Quiesce old local
    /// SDK/ISR producers and retain its configured clock/reset/power state.
    /// A coordinated vendor peer may use its own side; shared-ring/cache and
    /// buffer handoff are separate duties of `ipc::Doorbell::from_configured`
    /// and each unsafe notification. No ring is initialized by this handoff.
    pub unsafe fn assume_application_ownership(self) -> T {
        self.token
    }
}

/// WS53 singleton reservations. Each peripheral requires its own explicit
/// ownership handoff; shared system-memory/clock blocks are not exposed.
#[cfg(feature = "chip-ws53")]
#[allow(non_snake_case)]
pub struct Peripherals {
    /// UART0 (UARTH1) ownership reservation.
    pub UART0: Unclaimed<Uart0<'static>>,
    /// UART1 (UARTH0) ownership reservation.
    pub UART1: Unclaimed<Uart1<'static>>,
    /// UART2 (UARTL) ownership reservation.
    pub UART2: Unclaimed<Uart2<'static>>,
    /// TIMER0/1 ownership reservation.
    pub TIMER: Unclaimed<Timer<'static>>,
    /// Watchdog reservation; clock/reset and NMI ownership are separate handoff obligations.
    pub WDT: Unclaimed<Wdt<'static>>,
    /// TCXO application window reservation; not proof of a private timebase.
    pub TCXO: Unclaimed<Tcxo<'static>>,
    /// Application RTC0 reservation; not proof of a running low-speed clock.
    pub RTC0: Unclaimed<Rtc0<'static>>,
    /// Application RTC1 reservation, independent of RTC0.
    pub RTC1: Unclaimed<Rtc1<'static>>,
    /// Whole PWM block reservation; all groups/channels/IRQs share this owner.
    pub PWM: Unclaimed<Pwm<'static>>,
    /// I2C0 controller reservation; configured bus/pad permissions are separate.
    pub I2C0: Unclaimed<I2c0<'static>>,
    /// I2C1 controller reservation; independent of I2C0, shared clock halfword.
    pub I2C1: Unclaimed<I2c1<'static>>,
    /// SPI0 standard-layout controller reservation; pads/clock handoff is separate.
    pub SPI0: Unclaimed<Spi0<'static>>,
    /// QSPI1 controller reservation; does not grant ownership of its shared XIP path.
    pub QSPI1: Unclaimed<Qspi1<'static>>,
    /// Temperature sampler reservation; shared thermal services require explicit handoff.
    pub TSENSOR: Unclaimed<Tsensor<'static>>,
    /// I2S0 reservation; shared audio clocks and pads require explicit handoff.
    pub I2S: Unclaimed<I2s<'static>>,
    /// ADC v155 analog reservation; diagnostic RAM/pads need separate ownership.
    pub ADC: Unclaimed<Adc<'static>>,
    /// Shared ADC/CPU-trace RAM writer; requires independent exclusive handoff.
    pub MCU_DIAG: Unclaimed<McuDiag<'static>>,
    /// Whole MDMA/IRQ58 reservation; does not establish usable SDMA ownership.
    pub MDMA: Unclaimed<Mdma<'static>>,
    /// Shared security TRNG reservation; requires random-service coordination.
    pub TRNG: Unclaimed<Trng<'static>>,
    /// Whole KM reservation: global loader/selectors, all slots and IRQ61.
    pub KM: Unclaimed<Km<'static>>,
    /// Shared SPACC reservation; requires coordination of HASH, SYMC and IRQ59.
    pub SPACC: Unclaimed<Spacc<'static>>,
    /// eFuse read-service reservation; does not authorize OTP programming.
    pub EFUSE: Unclaimed<Efuse<'static>>,
    /// SFC CPU-command reservation, not ownership of the shared Flash/XIP bus.
    pub SFC: Unclaimed<SfcCfg<'static>>,
    /// SDIO controller reservation; not an ADMA buffer or host-protocol handoff.
    pub SDIO: Unclaimed<Sdio<'static>>,
    /// Application-side IPC doorbell reservation; not shared-ring ownership.
    pub IPC: Unclaimed<Ipc<'static>>,
    /// GPIO0 bank reservation; pad ownership/configuration is separate.
    pub GPIO0: Unclaimed<Gpio0<'static>>,
    /// GPIO1 bank reservation; pad ownership/configuration is separate.
    pub GPIO1: Unclaimed<Gpio1<'static>>,
    /// GPIO2 bank reservation; does not release the SWD pads.
    pub GPIO2: Unclaimed<Gpio2<'static>>,
}

#[cfg(feature = "chip-ws53")]
impl Peripherals {
    /// Reserve the PAC singleton once; performs no MMIO and grants no cross-core ownership.
    pub fn take() -> Option<Self> {
        let _pac = crate::soc::pac::Peripherals::take()?;
        Some(Self {
            UART0: Unclaimed { token: unsafe { Uart0::steal() } },
            UART1: Unclaimed { token: unsafe { Uart1::steal() } },
            UART2: Unclaimed { token: unsafe { Uart2::steal() } },
            TIMER: Unclaimed { token: unsafe { Timer::steal() } },
            WDT: Unclaimed { token: unsafe { Wdt::steal() } },
            TCXO: Unclaimed { token: unsafe { Tcxo::steal() } },
            RTC0: Unclaimed { token: unsafe { Rtc0::steal() } },
            RTC1: Unclaimed { token: unsafe { Rtc1::steal() } },
            PWM: Unclaimed { token: unsafe { Pwm::steal() } },
            I2C0: Unclaimed { token: unsafe { I2c0::steal() } },
            I2C1: Unclaimed { token: unsafe { I2c1::steal() } },
            SPI0: Unclaimed { token: unsafe { Spi0::steal() } },
            QSPI1: Unclaimed { token: unsafe { Qspi1::steal() } },
            TSENSOR: Unclaimed { token: unsafe { Tsensor::steal() } },
            I2S: Unclaimed { token: unsafe { I2s::steal() } },
            ADC: Unclaimed { token: unsafe { Adc::steal() } },
            MCU_DIAG: Unclaimed { token: unsafe { McuDiag::steal() } },
            MDMA: Unclaimed { token: unsafe { Mdma::steal() } },
            TRNG: Unclaimed { token: unsafe { Trng::steal() } },
            KM: Unclaimed { token: unsafe { Km::steal() } },
            SPACC: Unclaimed { token: unsafe { Spacc::steal() } },
            EFUSE: Unclaimed { token: unsafe { Efuse::steal() } },
            SFC: Unclaimed { token: unsafe { SfcCfg::steal() } },
            SDIO: Unclaimed { token: unsafe { Sdio::steal() } },
            IPC: Unclaimed { token: unsafe { Ipc::steal() } },
            GPIO0: Unclaimed { token: unsafe { Gpio0::steal() } },
            GPIO1: Unclaimed { token: unsafe { Gpio1::steal() } },
            GPIO2: Unclaimed { token: unsafe { Gpio2::steal() } },
        })
    }
}

// ── Wrappers for PAC types present in EVERY chip (shared) ───────────────────
#[cfg(not(feature = "chip-ws53"))]
peripheral!(GlbCtlM, crate::soc::pac::GlbCtlM);
peripheral!(Gpio0, crate::soc::pac::Gpio0);
peripheral!(Gpio1, crate::soc::pac::Gpio1);
peripheral!(Gpio2, crate::soc::pac::Gpio2);
#[cfg(not(feature = "chip-ws53"))]
peripheral!(UlpGpio, crate::soc::pac::UlpGpio);
peripheral!(Uart0, crate::soc::pac::Uart0);
peripheral!(Uart1, crate::soc::pac::Uart1);
peripheral!(Uart2, crate::soc::pac::Uart2);
peripheral!(I2c0, crate::soc::pac::I2c0);
peripheral!(I2c1, crate::soc::pac::I2c1);
peripheral!(Spi0, crate::soc::pac::Spi0);
#[cfg(feature = "chip-ws53")]
peripheral!(Qspi1, crate::soc::pac::Qspi1);
#[cfg(feature = "chip-ws53")]
peripheral!(Tsensor, crate::soc::pac::Tsensor);
#[cfg(feature = "chip-ws53")]
peripheral!(I2s, crate::soc::pac::I2s);
#[cfg(feature = "chip-ws53")]
peripheral!(Adc, crate::soc::pac::Adc);
#[cfg(feature = "chip-ws53")]
peripheral!(McuDiag, crate::soc::pac::McuDiag);
#[cfg(feature = "chip-ws53")]
peripheral!(Mdma, crate::soc::pac::Mdma);
#[cfg(not(feature = "chip-ws53"))]
peripheral!(Spi1, crate::soc::pac::Spi1);
peripheral!(Pwm, crate::soc::pac::Pwm);
#[cfg(not(feature = "chip-ws53"))]
peripheral!(Dma, crate::soc::pac::Dma);
#[cfg(not(feature = "chip-ws53"))]
peripheral!(Sdma, crate::soc::pac::Sdma);
peripheral!(Timer, crate::soc::pac::Timer);
peripheral!(Wdt, crate::soc::pac::Wdt);
#[cfg(not(feature = "chip-ws53"))]
peripheral!(Rtc, crate::soc::pac::Rtc);
#[cfg(feature = "chip-ws53")]
peripheral!(Rtc0, crate::soc::pac::Rtc0);
#[cfg(feature = "chip-ws53")]
peripheral!(Rtc1, crate::soc::pac::Rtc1);
peripheral!(Tcxo, crate::soc::pac::Tcxo);
peripheral!(Trng, crate::soc::pac::Trng);
#[cfg(feature = "chip-ws53")]
peripheral!(Efuse, crate::soc::pac::Efuse);
#[cfg(feature = "chip-ws53")]
peripheral!(SfcCfg, crate::soc::pac::SfcCfg);
#[cfg(feature = "chip-ws53")]
peripheral!(Sdio, crate::soc::pac::Sdio);
#[cfg(feature = "chip-ws53")]
peripheral!(Ipc, crate::soc::pac::MCrg);
#[cfg(feature = "chip-ws53")]
peripheral!(Spacc, crate::soc::pac::Spacc);
#[cfg(feature = "chip-ws53")]
peripheral!(Km, crate::soc::pac::Km);

// ── WS63-only PAC types ─────────────────────────────────────────────────────
#[cfg(feature = "chip-ws63")]
mod ws63_only {
    use super::PhantomData;
    peripheral!(SysCtl0, crate::soc::pac::SysCtl0);
    peripheral!(SysCtl1, crate::soc::pac::SysCtl1);
    #[cfg(feature = "unstable")]
    peripheral!(Cmu, crate::soc::pac::Cmu);
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
    peripheral!(BtEmCtl, crate::soc::pac::BtEmCtl);
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
    #[cfg(feature = "unstable")]
    CMU => Cmu,
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
    BT_EM_CTL => BtEmCtl,
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
