use super::*;

const FIXTURE: &str = include_str!("../../../../tests/fixtures/ws53-pins.csv");

#[test]
fn every_canonical_pad_matches_compiled_sdk_modes_and_drive_codes() {
    let mut seen = [false; 256];
    let mut count = 0;
    for line in FIXTURE.lines().filter(|line| line.starts_with("PAD,")) {
        let columns: std::vec::Vec<_> = line.split(',').collect();
        let raw: u8 = columns[1].parse().unwrap();
        let mask: u8 = columns[2].parse().unwrap();
        let drive_bits: u8 = columns[3].parse().unwrap();
        let sdk = SdkPinId::from_raw(raw).unwrap();
        let pad = sdk.pad();
        assert_eq!(sdk.number(), raw);
        assert_eq!(pad.sdk_id(), sdk);
        assert!(!seen[raw as usize]);
        seen[raw as usize] = true;
        count += 1;
        for code in 0..=u8::MAX {
            let selection = MuxMode::from_code(code).and_then(|mode| pad.select_mode(mode));
            assert_eq!(selection.is_some(), code < 8 && mask & (1 << code) != 0, "{pad:?} mode {code}");
            if let Some(selection) = selection {
                assert_eq!(selection.pad(), pad);
                assert_eq!(selection.mode().code(), code);
            }
            let drive = pad.drive_setting(code);
            assert_eq!(drive.is_some(), code < (1 << drive_bits), "{pad:?} drive {code}");
            if let Some(drive) = drive {
                assert_eq!(drive.pad(), pad);
                assert_eq!(drive.code(), code);
            }
        }
    }
    assert_eq!(count, 31);
    for raw in 0..=u8::MAX {
        assert_eq!(SdkPinId::from_raw(raw).is_some(), seen[raw as usize], "SDK {raw}");
    }
    for pad in Pad::ALL {
        assert!(seen[pad.sdk_id().number() as usize]);
        seen[pad.sdk_id().number() as usize] = false;
    }
    assert!(!seen.into_iter().any(|value| value));
}

#[test]
fn all_chip_aliases_match_the_original_application_pin_enum() {
    let mut count = 0;
    for line in FIXTURE.lines().filter(|line| !line.starts_with('#') && !line.starts_with("PAD,")) {
        let columns: std::vec::Vec<_> = line.split(',').collect();
        let index: u8 = columns[1].parse().unwrap();
        let expected: u8 = columns[2].parse().unwrap();
        let pad = match columns[0] {
            "MGPIO" => Pad::from_mgpio(Mgpio::from_index(index).unwrap()),
            "AGPIO" => Pad::from_agpio(Agpio::from_index(index).unwrap()),
            "SGPIO" => Pad::from_sgpio(Sgpio::from_index(index).unwrap()),
            _ => panic!("unexpected fixture namespace"),
        };
        assert_eq!(pad.sdk_id().number(), expected, "{line}");
        count += 1;
    }
    assert_eq!(count, 39);
    assert_eq!(Pad::from_mgpio(Mgpio::from_index(1).unwrap()), Pad::Agpio6);
    assert_eq!(Pad::from_mgpio(Mgpio::from_index(14).unwrap()), Pad::Sgpio0);
}

#[test]
fn namespace_boundaries_reject_flash_absent_and_sentinel_numbers() {
    for index in 0..=u8::MAX {
        assert_eq!(Mgpio::from_index(index).is_some(), index <= 22);
        assert_eq!(Agpio::from_index(index).is_some(), index <= 12);
        assert_eq!(Sgpio::from_index(index).is_some(), index <= 2);
        assert_eq!(MuxMode::from_code(index).is_some(), index <= 7);
    }
    for id in [1, 6, 7, 11, 14, 15, 16, 21, 23, 24, 25, 26, 27, 28, 29, 30, 31, 48] {
        assert!(SdkPinId::from_raw(id).is_none(), "reserved SDK ID {id}");
    }
}

#[test]
fn gpio_lines_are_unique_with_holes_excluded_and_correct_irqs() {
    let mut masks = [0u32; 3];
    let mut counts = [0; 3];
    for pad in Pad::ALL {
        if let Some(line) = pad.gpio_line() {
            let bank = match line.bank() {
                GpioBank::Bank0 => 0,
                GpioBank::Bank1 => 1,
                GpioBank::Bank2 => 2,
            };
            assert_eq!(line.bank().interrupt() as u16, [34, 33, 29][bank]);
            assert_eq!(line.mask(), 1 << line.bit());
            assert_eq!(masks[bank] & line.mask(), 0, "duplicate line for {pad:?}");
            masks[bank] |= line.mask();
            counts[bank] += 1;
        } else {
            assert_eq!(pad, Pad::Reset);
        }
    }
    assert_eq!(counts, [15, 12, 3]);
    assert_eq!(
        masks[0],
        0x7fffff & !((1 << 1) | (1 << 6) | (1 << 7) | (1 << 11) | (1 << 14) | (1 << 15) | (1 << 16) | (1 << 21))
    );
    assert_eq!(masks[1..], [0xfff, 7]);
    let mio1 = Pad::from_mgpio(Mgpio::from_index(1).unwrap()).gpio_line().unwrap();
    assert_eq!(mio1.bank(), GpioBank::Bank1);
    assert_eq!(mio1.bit(), 6);
}

#[test]
fn reset_and_debug_identity_never_imply_general_gpio_ownership() {
    assert_eq!(Pad::from_agpio(Agpio::from_index(12).unwrap()), Pad::Reset);
    assert_eq!(Pad::Reset.gpio_line(), None);
    for mode in 0..8 {
        assert!(Pad::Reset.select_mode(MuxMode::from_code(mode).unwrap()).is_none());
    }
    let debug: std::vec::Vec<_> = Pad::ALL.into_iter().filter(|pad| pad.is_debug()).collect();
    assert_eq!(debug, [Pad::Sgpio0, Pad::Sgpio1]);
    assert_eq!(Pad::Sgpio0.gpio_line().unwrap().bit(), 0);
    assert_eq!(Pad::Sgpio1.gpio_line().unwrap().bit(), 1);
    // Copyable metadata grants neither a peripheral singleton nor a pin token.
    assert_eq!(Pad::Sgpio0.sdk_id().number(), 45);
}
