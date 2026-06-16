//! Utility macros for the WS63 HAL.

/// Declare a driver as unstable (only available with the `unstable` feature).
/// When the feature is not enabled, the module is `pub(crate)`.
#[macro_export]
macro_rules! unstable_module {
    ($($tokens:tt)*) => {
        #[cfg(feature = "unstable")]
        $($tokens)*
        #[cfg(not(feature = "unstable"))]
        pub(crate) $($tokens)*
    };
}

/// Declare a driver that requires the `unstable` feature.
/// Without it, the driver is not compiled at all.
#[macro_export]
macro_rules! unstable_driver {
    ($($tokens:tt)*) => {
        #[cfg(feature = "unstable")]
        $($tokens)*
    };
}

/// Create a type-erased enum for a peripheral type.
/// Example:
/// ```ignore
/// any_peripheral! {
///     pub peripheral AnySpi<'d> {
///         Spi0(crate::peripherals::Spi0<'d>),
///         Spi1(crate::peripherals::Spi1<'d>),
///     }
/// }
/// ```
#[macro_export]
macro_rules! any_peripheral {
    (
        $(#[$outer:meta])*
        $vis:vis peripheral $name:ident <'d> {
            $(
                $(#[$inner:meta])*
                $variant:ident($ty:ty),
            )+
        }
    ) => {
        $(#[$outer])*
        $vis enum $name<'d> {
            $(
                $(#[$inner])*
                $variant($ty),
            )+
        }

        /// Trait for types that can be degraded into an `$name`.
        $vis trait Degrade<'d> {
            fn degrade(self) -> $name<'d>;
        }

        $(
            impl<'d> Degrade<'d> for $ty {
                fn degrade(self) -> $name<'d> {
                    $name::$variant(self)
                }
            }
        )+
    };
}

/// Infallible conversion — used when a conversion cannot fail.
#[macro_export]
macro_rules! infallible {
    ($from:ty => $to:ty, |$val:ident| $expr:expr) => {
        impl From<$from> for $to {
            fn from($val: $from) -> Self {
                $expr
            }
        }
    };
}
