//! Watchdog feature.

use embedded_hal::i2c::I2c;
use num_enum::{FromPrimitive, IntoPrimitive};

use crate::core::Axp2101;
use crate::error::Error;
use crate::register_addresses::*;

/// Watchdog action.
///
/// Hardware defaults to [`WatchdogAction::IrqOnly`].
#[repr(u8)]
#[derive(IntoPrimitive, FromPrimitive, Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum WatchdogAction {
    /// Only send an IRQ signal.
    #[num_enum(default)]
    IrqOnly,
    /// Send an IRQ signal, and perform a system reset(reset all related PMU registers).
    ///
    /// The registers which have reset condition as "System Reset" will be reset.
    IrqSystemReset,
    /// Pull down PWROK for 1 second, and do what [`WatchdogAction::IrqSystemReset`] does.
    IrqSystemResetPullPwrok,
    /// Restart all DCDC/LDO(power off & power on), and do what [`WatchdogAction::IrqSystemResetPullPwrok`] does.
    IrqFullRestart,
}

impl<I2C: I2c> Axp2101<I2C> {
    /// Set `true` to enable watchdog.
    ///
    /// Chip defaults to disabled.
    pub fn set_watchdog_en(&mut self, value: bool) -> Result<(), Error> {
        self.write_bit(REG_CHARGER_GAUGE_WATCHDOG_SW, 0, value)
    }

    /// Sets what a watchdog reset triggers.
    ///
    /// Chip defaults to [`WatchdogAction::IrqOnly`].
    pub fn set_watchdog_action(&mut self, value: WatchdogAction) -> Result<(), Error> {
        self.write_bits(REG_WATCHDOG_CONTROL, 4..=5, value.into())
    }

    /// Feeds watchdog.
    pub fn feed_watchdog(&mut self) -> Result<(), Error> {
        self.write_bit(REG_WATCHDOG_CONTROL, 3, true)
    }

    /// Sets TWSI watchdog timer length.
    ///
    /// The actual time length is 2 ** raw_value. For example, write 0 for 1 second,
    /// 1 for 2 seconds, 4 for 16 seconds.
    ///
    /// The maximum timer length is 128 seconds, corresponding raw value is 7(0b111).
    pub fn set_watchdog_timer_length(&mut self, value: u8) -> Result<(), Error> {
        if value > 0b111 {
            Err(Error::ValueOutOfRange)
        } else {
            self.write_bits(REG_WATCHDOG_CONTROL, 0..2, value)
        }
    }
}
