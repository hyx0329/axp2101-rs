//! Power key/button(PWRON key) configuration.

use bit_field::BitField;
use embedded_hal::i2c::I2c;
use num_enum::{FromPrimitive, IntoPrimitive};

use crate::core::Axp2101;
use crate::error::Error;
use crate::register_addresses::*;

/// PWRON key active duration to trigger an IRQ event.
///
/// Hardware default value is 1.5s(0b01)
#[repr(u8)]
#[derive(IntoPrimitive, FromPrimitive, Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum KeyDurationIrq {
    /// 1 second.
    T1000MS,
    /// 1.5 seconds.
    #[num_enum(default)]
    T1500MS,
    /// 2 seconds.
    T2000MS,
    /// 2.5 seconds.
    T2500MS,
}

/// PWRON key active duration to trigger a power off event.
///
/// Hardware default value is 6s(0b01)
#[repr(u8)]
#[derive(IntoPrimitive, FromPrimitive, Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum KeyDurationPowerOff {
    /// 4 seconds.
    T4S,
    /// 6 seconds.
    #[num_enum(default)]
    T6S,
    /// 8 seconds.
    T8S,
    /// 10 seconds.
    T10S,
}

/// PWRON key active duration to trigger a power on event.
///
/// Hardware default value depends on efuse.
#[repr(u8)]
#[derive(IntoPrimitive, FromPrimitive, Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum KeyDurationPowerOn {
    /// 128 milliseconds.
    #[num_enum(default)]
    T128MS,
    /// 512 milliseconds.
    T512MS,
    /// 1 second.
    T1000MS,
    /// 2 seconds.
    T2000MS,
}

impl<I2C: I2c> Axp2101<I2C> {
    /// Set `true` to power off when PWRON active more than configured OFFLEVEL time.
    ///
    /// See also [`Axp2101::set_pwron_key_poweroff_restart`].
    ///
    /// This config is also referred as `btn_pwroff_en`
    ///
    /// Chip defaults to efuse value.
    pub fn set_pwron_key_poweroff(&mut self, value: bool) -> Result<(), Error> {
        self.write_bit(REG_POWEROFF_EN_BEHAVIOR, 1, value)
    }

    /// Sets the actual behavior of [`Axp2101::set_pwron_key_poweroff`].
    ///
    /// PMU will automatically restart, if set to `true`.
    ///
    /// Chip defaults to efuse value.
    pub fn set_pwron_key_poweroff_but_restart(&mut self, value: bool) -> Result<(), Error> {
        self.write_bit(REG_POWEROFF_EN_BEHAVIOR, 0, value)
    }

    /// Returns [`KeyDurationIrq`], the time duration PWRON key need to be active to trigger an IRQ event.
    pub fn key_duration_irq(&mut self) -> Result<KeyDurationIrq, Error> {
        let result: KeyDurationIrq = self.read_u8(REG_KEY_EVENT_TIME)?.get_bits(4..=5).into();
        Ok(result)
    }

    /// Sets PWRON key active duration for an IRQ event.
    pub fn set_key_duration_irq(&mut self, value: KeyDurationIrq) -> Result<(), Error> {
        self.write_bits(REG_KEY_EVENT_TIME, 4..=5, value.into())
    }

    /// Returns [`KeyDurationPowerOff`], the time duration PWRON key need to be active to trigger an power off event.
    pub fn key_duration_power_off(&mut self) -> Result<KeyDurationPowerOff, Error> {
        let reg_val = self.read_u8(REG_KEY_EVENT_TIME)?.get_bits(2..=3);
        Ok(KeyDurationPowerOff::from_primitive(reg_val))
    }

    /// Sets PWRON key active duration for a power off event.
    pub fn set_key_duration_power_off(&mut self, value: KeyDurationPowerOff) -> Result<(), Error> {
        self.write_bits(REG_KEY_EVENT_TIME, 2..=3, value.into())
    }

    /// Returns [`KeyDurationPowerOn`], the time duration PWRON key need to be active to trigger an power on event.
    pub fn key_duration_power_on(&mut self) -> Result<KeyDurationPowerOn, Error> {
        let reg_val = self.read_u8(REG_KEY_EVENT_TIME)?.get_bits(0..=1);
        Ok(KeyDurationPowerOn::from_primitive(reg_val))
    }

    /// Sets PWRON key active duration for a power on event.
    pub fn set_key_duration_power_on(&mut self, value: KeyDurationPowerOn) -> Result<(), Error> {
        self.write_bits(REG_KEY_EVENT_TIME, 0..=1, value.into())
    }
}
