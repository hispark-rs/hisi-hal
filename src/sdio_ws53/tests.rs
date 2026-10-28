use super::*;

#[test]
fn rejects_rom_length_truncation_and_address_wrap() {
    for value in [0, 65536, 65537, u32::MAX] {
        assert_eq!(AdmaLength::try_new(value), None);
    }
    for value in [1, 4, 65535] {
        assert_eq!(u32::from(AdmaLength::try_new(value).unwrap().get()), value);
    }
    let one = AdmaLength::try_new(1).unwrap();
    let two = AdmaLength::try_new(2).unwrap();
    assert!(AdmaTransferImage::try_new(0, one).is_none());
    assert!(AdmaTransferImage::try_new(u32::MAX, one).is_some());
    assert!(AdmaTransferImage::try_new(u32::MAX, two).is_none());
}

#[test]
fn rom_descriptor_layout_and_last_slot() {
    let length = AdmaLength::try_new(65535).unwrap();
    // Audited ROM accepts unaligned software values; do not invent an alignment
    // guarantee. Bytes are serialized explicitly, independent of host ABI.
    assert_eq!(
        AdmaTransferImage::try_new(0x20050001, length).unwrap().to_le_bytes(),
        [0x21, 0, 0xff, 0xff, 1, 0, 5, 0x20]
    );
    assert_eq!(AdmaIndex::try_new(0).unwrap().byte_offset(), 0);
    assert_eq!(AdmaIndex::try_new(128).unwrap().byte_offset(), 1024);
    for value in [129, 255, u32::MAX] {
        assert_eq!(AdmaIndex::try_new(value), None);
    }
}

#[test]
fn matches_6144_audited_rom_cmd53_cases_without_error_sentinel() {
    for bytes in [0, 1, 64, 512, 2048, 4095] {
        for mode in [0, 1] {
            for count in 0..512 {
                let command = Cmd53Argument::from_bits(mode << 27 | count);
                let actual = command.byte_length(BlockSize::try_new(bytes));
                // Oracle from the retained unmodified ROM execution report,
                // with 0xffffffff converted to a typed error instead of length.
                let rom = if mode == 0 {
                    if count == 0 { 512 } else { count }
                } else if count * bytes == 0 {
                    u32::MAX
                } else {
                    count * bytes
                };
                if rom == u32::MAX {
                    assert!(actual.is_err());
                } else {
                    assert_eq!(actual, Ok(rom));
                }
            }
        }
    }
    assert_eq!(BlockSize::try_new(4096), None);
    assert_eq!(BlockSize::try_new(u32::MAX), None);
    assert_eq!(Cmd53Argument::from_bits(1 << 27).byte_length(None), Err(LengthError::ZeroBlockCount));
    assert_eq!(Cmd53Argument::from_bits((1 << 27) | 1).byte_length(None), Err(LengthError::MissingBlockSize));
    let max = Cmd53Argument::from_bits((1 << 27) | 511).byte_length(BlockSize::try_new(4095)).unwrap();
    assert_eq!(max, 2_092_545);
    assert_eq!(AdmaLength::try_new(max), None);
}

#[test]
fn command_fields_do_not_contaminate_length() {
    for function in 0..8 {
        for write in 0..2 {
            for increment in 0..2 {
                for address in [0, 1, 0x1ffff] {
                    let raw = write << 31 | function << 28 | increment << 26 | address << 9 | 511;
                    let cmd = Cmd53Argument::from_bits(raw);
                    assert_eq!(cmd.bits(), raw);
                    assert_eq!(u32::from(cmd.function()), function);
                    assert_eq!(cmd.address(), address);
                    assert_eq!(cmd.direction(), if write == 0 { Direction::HostRead } else { Direction::HostWrite });
                    assert_eq!(
                        cmd.address_mode(),
                        if increment == 0 { AddressMode::Fixed } else { AddressMode::Incrementing }
                    );
                    assert_eq!(cmd.byte_length(None), Ok(511));
                    assert!(!cmd.block_mode());
                }
            }
        }
    }
}
