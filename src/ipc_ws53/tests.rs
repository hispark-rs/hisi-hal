use super::*;

#[test]
fn channel_causes_and_hardware_formats_are_exact() {
    #[cfg(feature = "defmt")]
    {
        fn format<T: defmt::Format>() {}
        format::<ReceiveEnableState>();
        format::<Channel>();
        format::<NotificationPending>();
    }
    assert_eq!(Channel::Channel0.interrupt() as u16, 26);
    assert_eq!(Channel::Channel1.interrupt() as u16, 27);
}

fn store(r: &pac::m_crg::RegisterBlock, offset: usize, value: u32) {
    // Ordinary host RAM only, including the cells modeled RO by the PAC. No
    // hardware side effects are synthesized and no silicon MMIO is touched.
    let address = (r as *const _ as *mut u8).wrapping_add(offset).cast::<u32>();
    unsafe { address.write(value) };
}

#[test]
fn doorbell_fields_preserve_other_sources_and_do_not_consume_status() {
    let registers: pac::m_crg::RegisterBlock = unsafe { core::mem::zeroed() };
    let mut driver = Doorbell {
        _peripheral: unsafe { Ipc::steal() },
        registers: &registers,
        receive_mask: ReceiveEnableState::SdkAllEnabled.mask(),
    };
    store(&registers, 0x20, 0xffff);
    store(&registers, 0x28, 3);
    driver.set_receive_enabled(Channel::Channel1, true);
    driver.set_receive_enabled(Channel::Channel0, false);
    let enable = (&registers as *const _ as *const u8).wrapping_add(0x20).cast::<u32>();
    assert_eq!(unsafe { enable.read() }, 0xfffe);
    assert!(driver.receive_pending(Channel::Channel0));
    assert!(driver.receive_pending(Channel::Channel1));
    driver.acknowledge(Channel::Channel1);
    assert_eq!(registers.ipc_receive_ack_sync().read().bits(), 2);
    // The discarded command readback does not prove status cleared. This RAM
    // fixture deliberately keeps it high, and the HAL must still report it.
    assert!(driver.receive_pending(Channel::Channel1));
    assert_eq!(registers.ipc_receive_status().read().bits(), 3);
    driver.acknowledge(Channel::Channel0);
    assert_eq!(registers.ipc_receive_ack_sync().read().bits(), 1);
}

#[test]
fn pending_send_does_not_overwrite_and_new_send_is_halfword() {
    let registers: pac::m_crg::RegisterBlock = unsafe { core::mem::zeroed() };
    let mut driver = Doorbell {
        _peripheral: unsafe { Ipc::steal() },
        registers: &registers,
        receive_mask: ReceiveEnableState::Disabled.mask(),
    };
    store(&registers, 0x30, 1);
    store(&registers, 0x34, 0xa55a_face);
    assert_eq!(unsafe { driver.notify(Channel::Channel0) }, Err(NotificationPending));
    let command = (&registers as *const _ as *const u8).wrapping_add(0x34).cast::<u32>();
    assert_eq!(unsafe { command.read() }, 0xa55a_face);
    assert_eq!(unsafe { driver.notify(Channel::Channel1) }, Ok(()));
    assert_eq!(unsafe { command.read() }, 0xa55a_0002);
    // A successful command does not consume or clear the other pending bit.
    assert!(driver.send_pending(Channel::Channel0));
    assert!(!driver.send_pending(Channel::Channel1));
}
