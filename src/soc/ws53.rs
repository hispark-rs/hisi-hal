//! WS53 application-core profile; facts from fbb_ws53 163d9150.
//! SDK defaults describe a boot profile, not a measurement of the connected board.

/// Typed chip-pad identities and SDK mappings; these grant no pin ownership.
pub mod pins;

/// Absolute local interrupt cause numbers (not PLIC IDs).
pub use ws53_pac::interrupt::ExternalInterrupt as Interrupt;
/// TIMER0/1 clock after ws53_application/main.c::dev_work_clock_init.
/// The menuconfig initial value is 30 MHz, overwritten to 32 MHz at app startup.
pub const TIMER_CLOCK_HZ: u32 = 32_000_000;
/// SDK uart_porting.c default. No WS53 constructor programs a divider from this
/// constant until runtime clock selection is audited.
pub const UART_CLOCK_HZ: u32 = 32_000_000;
/// Application UART instances in the acore SDK map.
pub const UART_COUNT: usize = 3;
/// Confirmed application timer channels in this product profile.
pub const TIMER_COUNT: usize = 2;

#[cfg(all(test, not(target_arch = "riscv32")))]
mod tests {
    use super::*;
    #[test]
    fn irq_tiers_and_instances_match_acore_sdk() {
        assert_eq!([Interrupt::UART_0 as u16, Interrupt::UART_1 as u16, Interrupt::UART_2 as u16], [39, 41, 42]);
        assert_eq!([Interrupt::TIMER_0 as u16, Interrupt::TIMER_1 as u16], [53, 54]);
        assert_eq!(Interrupt::GPIO_0 as u16, 34);
        assert_eq!(Interrupt::GPIO_1 as u16, 33);
        assert_eq!(Interrupt::SGPIO_INT as u16, 29);
        assert_eq!(ws53_pac::Uart2::PTR as usize, 0x57014000);
    }
}
