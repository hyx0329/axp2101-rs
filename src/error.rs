use embedded_hal::i2c::ErrorKind as I2cErrorKind;
use embedded_hal::i2c::Error as I2cError;

/// AXP PMU errors
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum Error {
    /// Detected chip version does not match driver.
    UnknownChip(u8),
    /// An I2C error occurred during the transaction.
    I2cError(I2cErrorKind),
    /// Voltage value exceeds hardware limit.
    UnsupportedVoltage,
    /// Failed to parse a value. E.g. failed to process a value read from chip.
    ParseError,
    /// Unsupported action. Something might be not able to be changed.
    /// A different error occurred. The original error(mapped from) may contain more information.
    Unknown,
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
