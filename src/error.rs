//! Axp2101 Error.
use embedded_hal::i2c::{Error as I2cError, ErrorKind as I2cErrorKind};

/// AXP PMU errors.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum Error {
    /// An I2C error occurred during the transaction.
    I2cError(I2cErrorKind),
    /// Value(voltage, timer length) exceeds hardware limit.
    ValueOutOfRange,
    /// Failed to parse a value. E.g. failed to process a value read from chip.
    ParseError,
    /// A different error occurred. The original error(mapped from) may contain more information.
    Other,
}

impl<T: I2cError> From<T> for Error {
    fn from(value: T) -> Self {
        Self::I2cError(value.kind())
    }
}

impl embedded_hal::digital::Error for Error {
    fn kind(&self) -> embedded_hal::digital::ErrorKind {
        embedded_hal::digital::ErrorKind::Other
    }
}
