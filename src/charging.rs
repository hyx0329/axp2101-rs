//! Charging control related features.

use bit_field::BitField;
use embedded_hal::i2c::I2c;
use num_enum::{FromPrimitive, IntoPrimitive};

use crate::core::Axp2101;
use crate::error::Error;
use crate::register_addresses::*;

/// Input current limit. Limit of the VBUS input current.
///
/// Hardware defaults to [`InputCurrentLimit::I500MA`].
#[repr(u8)]
#[derive(IntoPrimitive, FromPrimitive, Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum InputCurrentLimit {
    /// 100 mA.
    I100MA,
    #[num_enum(default)]
    /// 500 mA.
    I500MA,
    /// 900 mA.
    I900MA,
    /// 1000 mA.
    I1000MA,
    /// 1500 mA.
    I1500MA,
    /// 2000 mA.
    I2000MA,
}

/// Battery current flow direction.
#[allow(missing_docs)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum BatteryCurrentDirection {
    Standby,
    Charging,
    Discharging,
    Unknown,
}

/// Battery charging status.
#[allow(missing_docs)]
#[repr(u8)]
#[derive(IntoPrimitive, FromPrimitive, Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum BatteryChargingStatus {
    TryCharging,
    PreCharging,
    ConstantCurrent,
    ConstantVoltage,
    Charged,
    NotCharging,
    #[num_enum(default)]
    Unknown,
}

impl<I2C: I2c> Axp2101<I2C> {
    /// Returns current [`BatteryCurrentDirection`].
    pub fn battery_current_direction(&mut self) -> Result<BatteryCurrentDirection, Error> {
        match self.read_u8(REG_PMU_STATUS1)?.get_bits(5..=6) {
            0 => Ok(BatteryCurrentDirection::Standby),
            1 => Ok(BatteryCurrentDirection::Charging),
            2 => Ok(BatteryCurrentDirection::Discharging),
            _ => Ok(BatteryCurrentDirection::Unknown),
        }
    }

    /// Returns [`BatteryChargingStatus`].
    pub fn battery_charging_status(&mut self) -> Result<BatteryChargingStatus, Error> {
        let value = self.read_u8(REG_PMU_STATUS1)?.get_bits(0..=2);
        Ok(BatteryChargingStatus::from_primitive(value))
    }

    /// Sets the minimum VSYS that allows linear charger working, in millivolt.
    /// Ranging from 4100mV to 4800mV, both ends inclusive, step size 100mV.
    ///
    /// Chip defaults to 4700mV. This field is called `ln_vsys_dpm` in one datasheet.
    pub fn set_min_vsys_linear_charger(&mut self, value: u16) -> Result<(), Error> {
        if (4100..=4800).contains(&value) {
            let reg_val = ((value - 4100) / 100) as u8;
            self.write_bits(REG_VSYS_LOW_THRESH, 4..=6, reg_val)
        } else {
            Err(Error::ValueOutOfRange)
        }
    }

    /// Sets the voltage for VINDPM, in millivolt.
    /// Ranging from 3880mV to 5080mV, both ends inclusive, step size 80mV, total 16 levels.
    ///
    /// Chip defaults to 4360mV(0b0110)
    pub fn set_vindpm_thresh(&mut self, value: u16) -> Result<(), Error> {
        if (3880..=5080).contains(&value) {
            let reg_val = ((value - 3880) / 80) as u8;
            self.write_bits(REG_VIN_LOW_THRESH, 0..=3, reg_val)
        } else {
            Err(Error::ValueOutOfRange)
        }
    }

    /// Sets the input current limit [`InputCurrentLimit`].
    ///
    /// Chip defaults to [`InputCurrentLimit::I500MA`].
    pub fn set_input_current_limit(&mut self, value: InputCurrentLimit) -> Result<(), Error> {
        self.write_bits(REG_IIN_HIGH_THRESH, 0..=2, value.into())
    }

    /// Reset the gauge, the related registers will be reseted as well.
    ///
    /// The behavior is not tested on a real hardware!
    ///
    /// TODO: verify behavior
    pub fn reset_gauge(&mut self) -> Result<(), Error> {
        self.write_bit(REG_GAUGE_RST, 3, true)
    }

    /// Reset the gauge, but keep the registers.
    ///
    /// The behavior is not tested on a real hardware!
    ///
    /// TODO: verify behavior
    pub fn reset_gauge_keep_registers(&mut self) -> Result<(), Error> {
        self.write_bit(REG_GAUGE_RST, 2, true)
    }

    /// Set `true` to enable the fuel gauge.
    ///
    /// Chip defaults to enabled.
    pub fn set_gauge_en(&mut self, value: bool) -> Result<(), Error> {
        self.write_bit(REG_CHARGER_GAUGE_WATCHDOG_SW, 3, value)
    }

    /// Set `true` to enable charging coin battery/backup battery.
    ///
    /// Chip defaults to disabled.
    pub fn set_charging_backup_battery(&mut self, value: bool) -> Result<(), Error> {
        self.write_bit(REG_CHARGER_GAUGE_WATCHDOG_SW, 2, value)
    }

    /// Set `true` to enable charging cell/main battery.
    ///
    /// Chip defaults to enabled.
    pub fn set_charging(&mut self, value: bool) -> Result<(), Error> {
        self.write_bit(REG_CHARGER_GAUGE_WATCHDOG_SW, 1, value)
    }

    /// Reads battery voltage, in millivolts.
    pub fn battery_voltage(&mut self) -> Result<u16, Error> {
        let raw_value = self.read_u16(REG_ADC_VBAT_H)?;
        // 14 bit
        Ok(raw_value & 0x3fff)
    }

    /// Returns battery percentage.
    pub fn battery_percent(&mut self) -> Result<u8, Error> {
        self.read_u8(REG_BATTERY_PERCENT)
    }
}
