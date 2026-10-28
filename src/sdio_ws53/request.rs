use super::{AddressMode, BlockSize, Cmd53Argument, Direction, LengthError};
use core::num::NonZeroU32;

/// An observed command cannot enter the Function1 fixed-address transfer path.
/// No descriptor or device register is changed by this validation.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RequestError {
    /// Only Function1 has an audited device transfer path; retain the observed ID.
    Function(u8),
    /// Command direction disagrees with the captured read/write-start event.
    Direction,
    /// The audited data path requires fixed addressing.
    IncrementingAddress,
    /// The audited data path only accepts function-relative address zero.
    Address(u32),
    /// The command's transfer size cannot be represented as a valid request.
    Length(LengthError),
}

/// Validated Function1 request metadata, not a controller or DMA reservation.
///
/// ROM read/write-start handlers reject nonzero address/increment bits, but do
/// not check function or direction themselves. This boundary additionally
/// requires Function1 and agreement with the caller's captured event. Invalid
/// block lengths never escape as the ROM's 0xffffffff callback sentinel.
/// Validation does not claim host enumeration, memory ownership or bus drain.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Function1Request {
    argument: Cmd53Argument,
    bytes: NonZeroU32,
}

impl Function1Request {
    /// Validate a captured CMD53 argument and separately observed event direction.
    /// `block_size` must describe the same command; it is ignored for byte mode.
    /// No default negotiated block size, truncation or descriptor split is used.
    /// The result may be larger than one `AdmaLength` and needs explicit splitting.
    pub const fn try_from_cmd53(
        argument: Cmd53Argument,
        block_size: Option<BlockSize>,
        event_direction: Direction,
    ) -> Result<Self, RequestError> {
        if argument.function() != 1 {
            return Err(RequestError::Function(argument.function()));
        }
        if !matches!(
            (argument.direction(), event_direction),
            (Direction::HostRead, Direction::HostRead) | (Direction::HostWrite, Direction::HostWrite)
        ) {
            return Err(RequestError::Direction);
        }
        if matches!(argument.address_mode(), AddressMode::Incrementing) {
            return Err(RequestError::IncrementingAddress);
        }
        if argument.address() != 0 {
            return Err(RequestError::Address(argument.address()));
        }
        let bytes = match argument.byte_length(block_size) {
            Ok(bytes) => bytes,
            Err(error) => return Err(RequestError::Length(error)),
        };
        // Both modes above guarantee a positive value. Keep the conversion
        // fallible if length decoding is ever extended to a zero-size command.
        match NonZeroU32::new(bytes) {
            Some(bytes) => Ok(Self { argument, bytes }),
            None => Err(RequestError::Length(LengthError::ZeroBlockCount)),
        }
    }

    /// Original command for diagnostics; the validated metadata is immutable.
    pub const fn argument(self) -> Cmd53Argument {
        self.argument
    }

    /// Direction from the external host's perspective.
    pub const fn direction(self) -> Direction {
        self.argument.direction()
    }

    /// Nonzero complete command length; never a narrowed descriptor length.
    pub const fn byte_length(self) -> NonZeroU32 {
        self.bytes
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn every_function_and_event_direction_is_checked_before_admission() {
        for function in 0..8 {
            for direction in [Direction::HostRead, Direction::HostWrite] {
                for event in [Direction::HostRead, Direction::HostWrite] {
                    let write = u32::from(direction == Direction::HostWrite);
                    let arg = Cmd53Argument::from_bits((write << 31) | (function << 28) | 17);
                    let result = Function1Request::try_from_cmd53(arg, None, event);
                    if function != 1 {
                        assert_eq!(result, Err(RequestError::Function(function as u8)));
                    } else if direction != event {
                        assert_eq!(result, Err(RequestError::Direction));
                    } else {
                        let req = result.unwrap();
                        assert_eq!(req.argument(), arg);
                        assert_eq!(req.direction(), event);
                        assert_eq!(req.byte_length().get(), 17);
                    }
                }
            }
        }
    }

    #[test]
    fn fixed_zero_address_is_the_only_audited_data_path() {
        for address in [1, 2, 511, 65536, 0x1ffff] {
            let arg = Cmd53Argument::from_bits(0x10000001 | (address << 9));
            assert_eq!(
                Function1Request::try_from_cmd53(arg, None, Direction::HostRead),
                Err(RequestError::Address(address))
            );
        }
        assert_eq!(
            Function1Request::try_from_cmd53(Cmd53Argument::from_bits(0x14000001), None, Direction::HostRead),
            Err(RequestError::IncrementingAddress)
        );
    }

    #[test]
    fn source_error_sentinel_is_rejected_and_large_commands_are_not_narrowed() {
        for count in [0, 1, 511] {
            let byte = Cmd53Argument::from_bits(0x10000000 | count);
            let req = Function1Request::try_from_cmd53(byte, None, Direction::HostRead).unwrap();
            assert_eq!(req.byte_length().get(), if count == 0 { 512 } else { count });
            let block = Cmd53Argument::from_bits(0x18000000 | count);
            assert!(Function1Request::try_from_cmd53(block, None, Direction::HostRead).is_err());
            let result = Function1Request::try_from_cmd53(block, BlockSize::try_new(4095), Direction::HostRead);
            if count == 0 {
                assert_eq!(result, Err(RequestError::Length(LengthError::ZeroBlockCount)));
            } else {
                assert_eq!(result.unwrap().byte_length().get(), count * 4095);
            }
        }
    }
}
