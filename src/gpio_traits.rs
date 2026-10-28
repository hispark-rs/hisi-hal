//! Shared infallible embedded-hal adapters; the pin types own all hardware behavior.

macro_rules! impl_error_type {
    ($pin:ty) => {
        impl embedded_hal::digital::ErrorType for $pin {
            type Error = core::convert::Infallible;
        }
    };
}

macro_rules! impl_input_pin {
    ($pin:ty, $driver:ident) => {
        impl embedded_hal::digital::InputPin for $pin {
            fn is_high(&mut self) -> Result<bool, Self::Error> {
                Ok($driver::is_high(self))
            }
            fn is_low(&mut self) -> Result<bool, Self::Error> {
                Ok(!$driver::is_high(self))
            }
        }
    };
}

macro_rules! impl_output_pin {
    ($pin:ty, $driver:ident) => {
        impl embedded_hal::digital::OutputPin for $pin {
            fn set_low(&mut self) -> Result<(), Self::Error> {
                $driver::set_low(self);
                Ok(())
            }
            fn set_high(&mut self) -> Result<(), Self::Error> {
                $driver::set_high(self);
                Ok(())
            }
        }
        impl embedded_hal::digital::StatefulOutputPin for $pin {
            fn is_set_high(&mut self) -> Result<bool, Self::Error> {
                Ok($driver::is_set_high(self))
            }
            fn is_set_low(&mut self) -> Result<bool, Self::Error> {
                Ok(!$driver::is_set_high(self))
            }
        }
    };
}

pub(crate) use {impl_error_type, impl_input_pin, impl_output_pin};

#[cfg(test)]
mod tests {
    use embedded_hal::digital::{InputPin, OutputPin, StatefulOutputPin};

    struct FakePin {
        input: bool,
        output: bool,
    }
    impl FakePin {
        fn is_high(&self) -> bool {
            self.input
        }
        fn set_high(&mut self) {
            self.output = true;
        }
        fn set_low(&mut self) {
            self.output = false;
        }
        fn is_set_high(&self) -> bool {
            self.output
        }
    }
    impl_error_type!(FakePin);
    impl_input_pin!(FakePin, FakePin);
    impl_output_pin!(FakePin, FakePin);

    #[test]
    fn delegates_input_and_output_independently() {
        let mut pin = FakePin { input: true, output: false };
        assert_eq!(InputPin::is_high(&mut pin), Ok(true));
        assert_eq!(InputPin::is_low(&mut pin), Ok(false));
        assert_eq!(StatefulOutputPin::is_set_low(&mut pin), Ok(true));
        OutputPin::set_high(&mut pin).unwrap();
        assert_eq!(StatefulOutputPin::is_set_high(&mut pin), Ok(true));
        pin.input = false;
        assert_eq!(InputPin::is_low(&mut pin), Ok(true));
        OutputPin::set_low(&mut pin).unwrap();
        assert_eq!(StatefulOutputPin::is_set_low(&mut pin), Ok(true));
    }
}
