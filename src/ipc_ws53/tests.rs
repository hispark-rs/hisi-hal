use super::*;

#[test]
fn channel_id_and_service_bounds_are_exact() {
    #[cfg(feature = "defmt")]
    {
        fn format<T: defmt::Format>() {}
        format::<ReceiveEnableState>();
        format::<Channel>();
        format::<MessageId>();
        format::<Priority>();
        format::<MessageError>();
        format::<Message<'static>>();
        format::<NotificationPending>();
        format::<mailbox::MailboxSize>();
        format::<mailbox::Route>();
        format::<mailbox::MailboxError>();
        format::<spsc::Snapshot>();
        format::<spsc::Error>();
    }
    assert_eq!(Channel::Channel0.interrupt() as u16, 26);
    assert_eq!(Channel::Channel1.interrupt() as u16, 27);
    assert_eq!(MessageId::UPDATE_SHARED_RANDOM_NUMBER.raw(), 18);
    for raw in 0..=u8::MAX {
        assert_eq!(MessageId::try_from_raw(raw).is_some(), raw < 19);
    }
}

#[test]
fn all_sdk_ids_priorities_and_payload_lengths_encode_exactly_and_preserve_tail() {
    let payload: [u8; MAX_PAYLOAD] = core::array::from_fn(|i| (i * 37 + 11) as u8);
    let mut cases = 0;
    for raw in 0..19 {
        for priority in [Priority::High, Priority::Low] {
            for length in 0..=MAX_PAYLOAD {
                let id = MessageId::try_from_raw(raw).unwrap();
                let message = Message::new(id, priority, &payload[..length]).unwrap();
                let mut output = [0xa5; MAX_MESSAGE];
                let used = message.encode(&mut output);
                assert_eq!(used, length + 4);
                assert_eq!(&output[..4], &[raw, priority as u8, length as u8, 0]);
                assert_eq!(&output[4..used], &payload[..length]);
                assert!(output[used..].iter().all(|n| *n == 0xa5));
                let decoded = Message::decode(&output[..used]).unwrap();
                assert_eq!(decoded.id(), id);
                assert_eq!(decoded.priority(), priority);
                assert_eq!(decoded.payload(), &payload[..length]);
                cases += 1;
            }
        }
    }
    assert_eq!(cases, 2318);
}

#[test]
fn every_u16_oversized_length_is_rejected_before_accessing_the_payload() {
    for length in 61..=u16::MAX {
        let [low, high] = length.to_le_bytes();
        assert_eq!(Message::decode(&[0, 0, low, high]), Err(MessageError::PayloadTooLong));
    }
    for length in [61, 64, 255, 256, 65535, 65536, 65540] {
        let bytes = std::vec![0; length];
        assert_eq!(Message::new(MessageId(0), Priority::Low, &bytes), Err(MessageError::PayloadTooLong));
    }
}

#[test]
fn malformed_ids_priority_truncation_and_trailing_bytes_are_not_dispatched() {
    for id in 19..=u8::MAX {
        assert_eq!(Message::decode(&[id, 0, 0, 0]), Err(MessageError::UnknownId));
    }
    for priority in 2..=u8::MAX {
        assert_eq!(Message::decode(&[18, priority, 0, 0]), Err(MessageError::UnknownPriority));
    }
    for length in 0..4 {
        assert_eq!(Message::decode(&[0; 4][..length]), Err(MessageError::Framing));
    }
    assert_eq!(Message::decode(&[18, 1, 1, 0]), Err(MessageError::Framing));
    assert_eq!(Message::decode(&[18, 1, 0, 0, 0xaa]), Err(MessageError::Framing));
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
