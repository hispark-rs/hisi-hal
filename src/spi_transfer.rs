//! Frame sequencing shared by blocking SPI backends; no hardware access here.

pub(crate) fn transfer<W: Copy + Default, E>(
    read: &mut [W],
    write: &[W],
    mut exchange: impl FnMut(W) -> Result<W, E>,
) -> Result<(), E> {
    for index in 0..read.len().max(write.len()) {
        let received = exchange(write.get(index).copied().unwrap_or_default())?;
        if let Some(slot) = read.get_mut(index) {
            *slot = received;
        }
    }
    Ok(())
}

#[cfg(all(test, not(target_arch = "riscv32")))]
mod tests {
    use super::transfer;
    use std::vec::Vec;

    #[test]
    fn read_write_padding_and_discard_exchange_one_frame_at_a_time() {
        let mut sent = Vec::new();
        let mut read = [0u32; 4];
        transfer(&mut read, &[3, 5], |word| {
            sent.push(word);
            Ok::<_, ()>(word + 10)
        })
        .unwrap();
        assert_eq!(sent, [3, 5, 0, 0]);
        assert_eq!(read, [13, 15, 10, 10]);

        let mut exchanges = 0;
        transfer(&mut [], &[1u8, 2, 3, 4], |word| {
            exchanges += 1;
            Ok::<_, ()>(word ^ 0xff)
        })
        .unwrap();
        assert_eq!(exchanges, 4);
    }

    #[test]
    fn failed_exchange_stops_without_overwriting_unreceived_words() {
        let mut read = [0xaau8; 4];
        let mut calls = 0;
        let result = transfer(&mut read, &[1, 2, 3, 4], |word| {
            calls += 1;
            if calls == 3 { Err("timeout") } else { Ok(word + 10) }
        });
        assert_eq!(result, Err("timeout"));
        assert_eq!(calls, 3);
        assert_eq!(read, [11, 12, 0xaa, 0xaa]);
    }
}
