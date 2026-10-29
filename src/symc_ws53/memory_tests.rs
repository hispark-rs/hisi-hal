use super::*;
use crate::{
    km::{Algorithm, CipherSlot, Slot, Usage},
    symc::{Direction, Mode},
};

#[test]
fn ranges_are_whole_owned_lines_and_disjoint() {
    assert_eq!(Range::new(0x20010000, 0x50000), Ok(Range { start: 0x20010000, length: 0x50000 }));
    assert!(Range::new(0x2005ffe0, 32).is_ok());
    for (a, n) in [
        (0x20010000, 0),
        (0x20010001, 32),
        (0x20010000, 31),
        (0x2000ffe0, 32),
        (0x20060000, 32),
        (usize::MAX - 31, 64),
        (0x120010000, 32),
    ] {
        assert_eq!(Range::new(a, n), Err(MemoryError));
    }
    let r = |a, n| Range::new(a, n).unwrap();
    let length = BlockLength::from_bytes(48).unwrap();
    let input = r(0x20050000, 64);
    let output = r(0x20050040, 64);
    let table = r(0x20050080, 96);
    assert_eq!(validate(input, output, table, length), Ok(()));
    assert_eq!(validate(r(0x20050000, 32), output, table, length), Err(PrepareError::Capacity));
    assert_eq!(validate(input, r(0x20050040, 32), table, length), Err(PrepareError::Capacity));
    for (a, b, c) in [
        (input, input, table),
        (input, output, input),
        (input, output, output),
        (input, r(0x20050020, 64), table),
        (input, output, r(0x20050060, 96)),
    ] {
        assert_eq!(validate(a, b, c, length), Err(PrepareError::Overlap));
    }
}
#[test]
fn descriptor_abi_parity_iv_and_unused_entries() {
    assert_eq!(core::mem::size_of::<DescriptorStorage>(), 96);
    assert_eq!(core::mem::align_of::<DescriptorStorage>(), 32);
    assert_eq!(core::mem::offset_of!(DescriptorStorage, input), 0);
    assert_eq!(core::mem::offset_of!(DescriptorStorage, output), 64);
    let mut iv = [0; 16];
    for (i, byte) in iv.iter_mut().enumerate() {
        *byte = (i * 17) as u8;
    }
    for mode in [Mode::Ecb, Mode::Cbc(iv), Mode::Ctr(iv), Mode::Ofb(iv), Mode::Cfb128(iv)] {
        for parity in [Parity::Even, Parity::Odd] {
            for length in [16, 32, 48, 256, 0x50000] {
                let length = BlockLength::from_bytes(length).unwrap();
                let c = Configuration::new(
                    Slot::Cipher(CipherSlot::Seven),
                    Algorithm::Aes128,
                    parity,
                    Some(Usage::Both),
                    Direction::Encrypt,
                    mode,
                )
                .unwrap();
                let mut s = DescriptorStorage { input: [[u32::MAX; 8]; 2], output: [[u32::MAX; 4]; 2] };
                s.fill(0x20050000, 0x20051000, length, c);
                assert_eq!(
                    &s.input[0][..4],
                    &[3 | if parity == Parity::Odd { 512 } else { 0 }, length.bytes(), 0x20050000, 0]
                );
                let expected = if mode == Mode::Ecb { [0; 16] } else { iv };
                for (word, bytes) in s.input[0][4..].iter().zip(expected.as_chunks::<4>().0) {
                    assert_eq!(word.to_le_bytes(), *bytes);
                }
                assert_eq!(s.output[0], [0, length.bytes(), 0x20051000, 0]);
                assert_eq!(s.input[1], [0; 8]);
                assert_eq!(s.output[1], [0; 4]);
            }
        }
    }
}
