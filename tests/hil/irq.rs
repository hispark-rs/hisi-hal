use crate::hal;

/// Full device.x-named TIMER_INT0 interrupt routing.
#[cfg(all(feature = "chip-ws63", not(feature = "async")))]
pub(crate) fn timer_int0_named_routing() {
    use core::sync::atomic::{AtomicBool, Ordering};
    use hal::interrupt;
    use hal::timer::{TimerChannel, TimerDriver, TimerMode};

    static FIRED: AtomicBool = AtomicBool::new(false);

    #[unsafe(no_mangle)]
    extern "C" fn TIMER_INT0() {
        let t = TimerDriver::new(unsafe { hal::peripherals::Timer::steal() });
        t.clear_interrupt(TimerChannel::Channel0);
        t.disable(TimerChannel::Channel0);
        FIRED.store(true, Ordering::SeqCst);
    }

    let t = TimerDriver::new(unsafe { hal::peripherals::Timer::steal() });
    t.configure(TimerChannel::Channel0, TimerMode::Periodic, 24_000);
    unsafe {
        interrupt::enable(interrupt::Interrupt::TIMER_INT0);
        interrupt::enable_global();
    }
    t.enable(TimerChannel::Channel0);

    let mut spun = 0u32;
    while !FIRED.load(Ordering::SeqCst) && spun < 20_000_000 {
        spun += 1;
        core::hint::spin_loop();
    }
    unsafe { interrupt::disable(interrupt::Interrupt::TIMER_INT0) };
    t.disable(TimerChannel::Channel0);
    t.clear_interrupt(TimerChannel::Channel0);

    assert!(
        FIRED.load(Ordering::SeqCst),
        "named TIMER_INT0 (IRQ 26) handler never ran — rt device.x named routing broken"
    );
}

/// Application-owned GPIO_INT0 routing into the async HAL hook on silicon.
/// Requires GPIO0 → GPIO3 jumper.
#[cfg(all(feature = "chip-ws63", feature = "async", feature = "hil-loopback", feature = "unstable"))]
pub(crate) fn gpio_int0_app_owned_routing() {
    use crate::pac;
    use hal::gpio::{AnyPin, InputConfig, InterruptTrigger, OutputConfig};
    use hal::interrupt;
    use hal::io_config::{GpioPad, IoConfigDriver, MuxFunction};

    #[unsafe(no_mangle)]
    extern "C" fn GPIO_INT0() {
        hal::gpio::on_interrupt(hal::gpio::GpioBank::Bank0);
    }

    let mut io = IoConfigDriver::new(unsafe { hal::peripherals::IoConfig::steal() });
    io.set_gpio_mux(GpioPad::Gpio00, MuxFunction::F0);
    io.set_gpio_mux(GpioPad::Gpio03, MuxFunction::F0);

    // SAFETY: GPIO0/3 owned by this test; jumpered 0->3.
    let mut out = unsafe { AnyPin::steal(0) }.init_output(OutputConfig::new().with_initial(false));
    let inp = unsafe { AnyPin::steal(3) }.init_input(InputConfig::default());
    inp.set_interrupt_trigger(InterruptTrigger::RisingEdge);
    inp.enable_interrupt();

    unsafe {
        interrupt::enable(interrupt::Interrupt::GPIO_INT0);
        interrupt::enable_global();
    }

    let g0 = unsafe { &*pac::Gpio0::PTR };
    assert_ne!(g0.gpio_int_en().read().bits() & (1 << 3), 0, "GPIO3 int-enable not set");

    out.set_high();

    let mut spun = 0u32;
    while (g0.gpio_int_en().read().bits() & (1 << 3)) != 0 && spun < 20_000_000 {
        spun += 1;
        core::hint::spin_loop();
    }
    unsafe { interrupt::disable(interrupt::Interrupt::GPIO_INT0) };

    assert_eq!(
        g0.gpio_int_en().read().bits() & (1 << 3),
        0,
        "application GPIO_INT0 handler never ran — on_interrupt(Bank0) did not mask GPIO3"
    );
}
