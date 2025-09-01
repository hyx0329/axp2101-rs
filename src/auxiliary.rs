//! Uncategorized AXP2101 functions.

use bit_field::BitField;
use embedded_hal::i2c::I2c;
use num_enum::{FromPrimitive, IntoPrimitive};

use crate::core::Axp2101;
use crate::error::Error;
use crate::register_addresses::*;

/// DIE Over-Temperature Protection temperatures, level 1.
///
/// Currently there's only one temperature level for AXP2101.
#[repr(u8)]
#[derive(IntoPrimitive, FromPrimitive, Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum DieOverheatTempL1 {
    /// 115 celsius degree
    #[num_enum(default)]
    Deg115C,
    /// 125 celsius degree
    Deg125C,
    /// 135 celsius degree
    Deg135C,
}

impl<I2C: I2c> Axp2101<I2C> {
    /// Read chip ID.
    ///
    /// It's revealed in the datasheet provided by M5Stack. On M5Stack Core2 V1.1, the raw
    /// reading is `74`(`0b01001010`), while the datasheet says it should be `0b01XX0111`.
    /// It may vary on different chip variants(e.g. customized ones).
    pub fn chip_id(&mut self) -> Result<u8, Error> {
        self.read_u8(REG_CHIP_ID)
    }

    /// Returns `true` if external power is connected.
    pub fn vbus_present(&mut self) -> Result<bool, Error> {
        Ok(self.read_u8(REG_PMU_STATUS0)?.get_bit(5))
    }

    /// Returns `true` if BATFET is ON, which means the power path from
    /// battery to system is OK.
    ///
    /// TODO: verify the actual behavior
    pub fn batfet_on(&mut self) -> Result<bool, Error> {
        Ok(self.read_u8(REG_PMU_STATUS0)?.get_bit(4))
    }

    /// Returns `true` if main battery is connected.
    pub fn battery_present(&mut self) -> Result<bool, Error> {
        Ok(self.read_u8(REG_PMU_STATUS0)?.get_bit(3))
    }

    /// Returns `true` if battery is in active mode.
    pub fn battery_active(&mut self) -> Result<bool, Error> {
        Ok(self.read_u8(REG_PMU_STATUS0)?.get_bit(2))
    }

    /// Returns `true` if in thermal regulation.
    pub fn thermal_regulated(&mut self) -> Result<bool, Error> {
        Ok(self.read_u8(REG_PMU_STATUS0)?.get_bit(1))
    }

    /// Returns `true` if in current limit state.
    pub fn current_limited(&mut self) -> Result<bool, Error> {
        Ok(self.read_u8(REG_PMU_STATUS0)?.get_bit(0))
    }

    /// Returns `true` if system power is turned on.
    pub fn system_on(&mut self) -> Result<bool, Error> {
        Ok(self.read_u8(REG_PMU_STATUS1)?.get_bit(4))
    }

    /// Returns `true` if the input voltage drops below the limit(VINDPM).
    ///
    /// VINDPM: VIN Dynamic Power Management
    ///
    /// The PMU will reduce charging current when in VINDPM status, until leaving VINDPM,
    /// or charging current reduced to zero.
    ///
    /// If the charging current is reduced to zero and ISYS is still too high
    /// that exceeds the input power supply capability, VSYS will drop.
    ///
    /// If VBAT becomes higher than VSYS, PMU will enter ther supplement mode,
    /// drawing power from the battery.
    ///
    /// TODO: verify the behavior
    pub fn vindpm_status(&mut self) -> Result<bool, Error> {
        Ok(self.read_u8(REG_PMU_STATUS1)?.get_bit(3))
    }

    /// Returns `true` if PMU draws all power from VBUS/VIN.
    ///
    /// This is effectively getting the results from [`Axp2101::vbus_present`]
    /// and [`Axp2101::vindpm_status`] with only one I2C read.
    pub fn vbus_serving(&mut self) -> Result<bool, Error> {
        let raw_value = self.read_u8(REG_PMU_STATUS1)?;
        Ok(!raw_value.get_bit(3) && raw_value.get_bit(5))
    }

    /// Writes to on-chip data buffer.
    ///
    /// The buffer size of AXP2101 is 4 bytes.
    pub fn write_data_buffer(&mut self, buf: &[u8]) -> Result<(), Error> {
        if buf.len() > 4 {
            Err(Error::Other)
        } else {
            let mut buffer: [u8; 5] = [0; 5];
            for (dest, src) in buffer[1..].iter_mut().zip(buf) {
                *dest = *src
            }
            let length = buf.len() + 1;
            Ok(self.i2c.write(AXP_CHIP_ADDR, &buffer[..length])?)
        }
    }

    /// Reads the on-chip data buffer.
    ///
    /// The buffer size of AXP2101 is 4 bytes.
    pub fn read_data_buffer(&mut self, buf: &mut [u8]) -> Result<(), Error> {
        if buf.len() > 4 {
            Err(Error::Other)
        } else {
            Ok(self
                .i2c
                .write_read(AXP_CHIP_ADDR, &[REG_DATABUF_START], buf)?)
        }
    }

    /// Sets `true` to utilize internal discharge after powering off.
    ///
    /// Chip defaults to `true`.
    pub fn set_off_discharge(&mut self, value: bool) -> Result<(), Error> {
        self.write_bit(REG_PMU_CONFIG, 5, value)
    }

    /// Sets `true` to restart when PWROK(Power good indication output) pin pulled low.
    ///
    /// All outputs except RTCLDO and VREF will be turned off at reset state, and then
    /// PMU initiates its startup sequence.
    ///
    /// This is one of the three PMU restart methods. See [Self::restart] and [Self::set_watchdog_action].
    ///
    /// Chip defaults to `false`.
    pub fn set_restart_on_pwrok_low(&mut self, value: bool) -> Result<(), Error> {
        self.write_bit(REG_PMU_CONFIG, 3, value)
    }

    /// Sets `true` to shut the PMIC when PWRON(Power On-Off key) enabled for 16 seconds.
    ///
    /// Chip defaults to `false`.
    pub fn set_pwron_shut_16s(&mut self, value: bool) -> Result<(), Error> {
        self.write_bit(REG_PMU_CONFIG, 2, value)
    }

    /// Sets the BATFET state in power-off state and battery-only state.
    /// `true` for enable/connected, and `false` for disable/disconnected.
    ///
    /// Chip defaults to efuse value.
    pub fn set_batfet_poweroff_state(&mut self, value: bool) -> Result<(), Error> {
        self.write_bit(REG_BATFET_CONTROL, 3, value)
    }

    /// Sets `true` to enable the BATFET over-current-protection.
    ///
    /// BATFET will cut the power if current exceeds 6 Amps for 100 microseconds.
    ///
    /// Chip defaults to efuse value.
    pub fn set_batfet_ocp(&mut self, value: bool) -> Result<(), Error> {
        self.write_bit(REG_BATFET_CONTROL, 3, value)
    }

    /// Sets DIE Over-Temperature Protection level 1 temperature.
    ///
    /// Chip defaults to [`DieOverheatTempL1::Deg125C`].
    pub fn set_die_temperature_l1(&mut self, value: DieOverheatTempL1) -> Result<(), Error> {
        self.write_bits(REG_TDIE_CONTROL, 1..=2, value.into())
    }

    /// Sets `true` to enable DIE temperature detection.
    ///
    /// Chip defaults to `true`.
    pub fn set_die_temperature_detect_en(&mut self, value: bool) -> Result<(), Error> {
        self.write_bit(REG_TDIE_CONTROL, 0, value)
    }

    /// Sets low-battery warning level 2.
    ///
    /// From 5% to 20%, both ends included, 1% per step.
    pub fn set_bat_low_level2(&mut self, value: u8) -> Result<(), Error> {
        if (5..=20).contains(&value) {
            let reg_val = value - 5;
            self.write_bits(REG_BAT_LOW_WARN_THRESH, 4..=7, reg_val)
        } else {
            Err(Error::ValueOutOfRange)
        }
    }

    /// Sets low-battery warning level 1.
    ///
    /// From 0% to 15%, both ends included, 1% per step.
    pub fn set_bat_low_level1(&mut self, value: u8) -> Result<(), Error> {
        if (0..=15).contains(&value) {
            self.write_bits(REG_BAT_LOW_WARN_THRESH, 0..=3, value)
        } else {
            Err(Error::ValueOutOfRange)
        }
    }

    /// Set `true` to power off when DIE temperature exceeds limit(LEVEL2).
    ///
    /// Chip defaults to enabled.
    pub fn set_die_overheat_poweroff(&mut self, value: bool) -> Result<(), Error> {
        self.write_bit(REG_POWEROFF_EN_BEHAVIOR, 2, value)
    }

    /// Sets 120%(130%) over voltage protection for all DCDC outputs.
    pub fn set_dcdc_ovp(&mut self, value: bool) -> Result<(), Error> {
        self.write_bit(REG_DCDC_PROTECT, 5, value)
    }

    /// Sets 85% under voltage protection for DCDC5.
    pub fn set_dcdc5_uvp(&mut self, value: bool) -> Result<(), Error> {
        self.write_bit(REG_DCDC_PROTECT, 4, value)
    }

    /// Sets 85% under voltage protection for DCDC4.
    pub fn set_dcdc4_uvp(&mut self, value: bool) -> Result<(), Error> {
        self.write_bit(REG_DCDC_PROTECT, 3, value)
    }

    /// Sets 85% under voltage protection for DCDC3.
    pub fn set_dcdc3_uvp(&mut self, value: bool) -> Result<(), Error> {
        self.write_bit(REG_DCDC_PROTECT, 2, value)
    }

    /// Sets 85% under voltage protection for DCDC2.
    pub fn set_dcdc2_uvp(&mut self, value: bool) -> Result<(), Error> {
        self.write_bit(REG_DCDC_PROTECT, 1, value)
    }

    /// Sets 85% under voltage protection for DCDC1.
    pub fn set_dcdc1_uvp(&mut self, value: bool) -> Result<(), Error> {
        self.write_bit(REG_DCDC_PROTECT, 0, value)
    }

    /// Returns the battery low threshold voltage triggering the power off in millivolt.
    pub fn vbat_poweroff_threshold(&mut self) -> Result<u16, Error> {
        let raw_value = self.read_u8(REG_POWEROFF_VBAT_LOW_THRESH)?.get_bits(0..=2) as u16;
        Ok(2600 + raw_value * 100)
    }

    /// Sets the battery low threshold voltage triggering the power off in millivolt.
    ///
    /// The range is [2600mV, 3300mV], total 8 steps.
    pub fn set_vbat_poweroff_threshold(&mut self, value: u16) -> Result<(), Error> {
        if (2600..=3300).contains(&value) {
            let reg_val = ((value - 2600) / 100) as u8;
            self.write_bits(REG_POWEROFF_VBAT_LOW_THRESH, 0..=2, reg_val)
        } else {
            Err(Error::ValueOutOfRange)
        }
    }

    // TODO: REG 0x25 power timing implementation

    /// Sets if to wake up the PMU from sleep mode when IRQ pin pulled low.
    ///
    /// Chip defaults to `false`.
    pub fn set_wakeup_by_irq_low(&mut self, value: bool) -> Result<(), Error> {
        self.write_bit(REG_SLEEP_WAKE_CONFIG, 4, value)
    }

    /// Sets if to set PWROK to low level when PMU woke.
    ///
    /// When [REG_PMU_CONFIG]/reg 0x10 bit 3 is 1, this action is effectively to restart the PMU.
    /// See [Self::set_restart_on_pwrok_low].
    ///
    /// Chip defaults to `true`.
    pub fn set_pwrok_low_when_wakeup(&mut self, value: bool) -> Result<(), Error> {
        self.write_bit(REG_SLEEP_WAKE_CONFIG, 3, value)
    }

    /// Sets whether to keep the voltages of DCDC/LDO outputs when wakeup.
    ///
    /// Set `true` to use the voltages configured before waking up.
    /// Set `false` to use chip's default voltages.
    ///
    /// Chip defaults to `false`.
    pub fn set_keep_voltage_when_wakeup(&mut self, value: bool) -> Result<(), Error> {
        self.write_bit(REG_SLEEP_WAKE_CONFIG, 2, value)
    }

    /// Wakes the PMU up. Read more detail at [`Axp2101::prepare_sleep`].
    pub fn wakeup(&mut self) -> Result<(), Error> {
        self.write_bit(REG_SLEEP_WAKE_CONFIG, 1, true)
    }

    /// Puts the PMU into sleep mode.
    ///
    /// When this method is called, the output enable bits at registers 0x80, 0x90 and 0x91(the
    /// switch bits for all outputs) are backed up. Then the host may disable the outputs, or
    /// configure the output voltages. When [`Axp2101::wakeup`] is called, the output enable bits
    /// are restored, and the voltages are either restored to the configured values or reset to the defaults,
    /// depending on how [`Axp2101::set_keep_voltage_when_wakeup`] is configured.
    pub fn prepare_sleep(&mut self) -> Result<(), Error> {
        self.write_bit(REG_SLEEP_WAKE_CONFIG, 0, true)
    }

    // TODO: REG 0x27 fast power on implementation

    /// Returns whether DIE temperature ADC channel is enabled.
    pub fn adc_status_die_temperature(&mut self) -> Result<bool, Error> {
        Ok(self.read_u8(REG_ADC_CONTROL)?.get_bit(4))
    }

    /// Sets whether to enable DIE temperature ADC channel
    pub fn set_adc_status_die_temperature(&mut self, value: bool) -> Result<(), Error> {
        self.write_bit(REG_ADC_CONTROL, 4, value)
    }

    /// Returns whether system voltage ADC channel is enabled.
    pub fn adc_status_system_voltage(&mut self) -> Result<bool, Error> {
        Ok(self.read_u8(REG_ADC_CONTROL)?.get_bit(3))
    }
    /// Sets whether to enable system voltage ADC channel
    pub fn set_adc_status_system_voltage(&mut self, value: bool) -> Result<(), Error> {
        self.write_bit(REG_ADC_CONTROL, 3, value)
    }

    /// Returns whether VBUS voltage ADC channel is enabled.
    pub fn adc_status_vbus_voltage(&mut self) -> Result<bool, Error> {
        Ok(self.read_u8(REG_ADC_CONTROL)?.get_bit(2))
    }
    /// Sets whether to enable VBUS voltage ADC channel
    pub fn set_adc_status_vbus_voltage(&mut self, value: bool) -> Result<(), Error> {
        self.write_bit(REG_ADC_CONTROL, 2, value)
    }

    /// Returns whether temperature sensor(TS pin) ADC channel is enabled.
    pub fn adc_status_ts_pin(&mut self) -> Result<bool, Error> {
        Ok(self.read_u8(REG_ADC_CONTROL)?.get_bit(1))
    }
    /// Sets whether to enable temperature sensor(TS pin) ADC channel
    pub fn set_adc_status_ts_pin(&mut self, value: bool) -> Result<(), Error> {
        self.write_bit(REG_ADC_CONTROL, 1, value)
    }

    /// Returns whether battery voltage ADC channel is enabled.
    pub fn adc_status_battery_voltage(&mut self) -> Result<bool, Error> {
        Ok(self.read_u8(REG_ADC_CONTROL)?.get_bit(0))
    }
    /// Sets whether to enable battery voltage ADC channel
    pub fn set_adc_status_battery_voltage(&mut self, value: bool) -> Result<(), Error> {
        self.write_bit(REG_ADC_CONTROL, 0, value)
    }

    // TODO: General purpose ADC implementation

    /// Reads raw TS pin ADC value, the unit is 0.5mV.
    pub fn ts_pin_voltage_raw(&mut self) -> Result<u16, Error> {
        let raw_value = self.read_u16(REG_ADC_TS_H)?;
        // 14 bit
        Ok(raw_value & 0x3fff)
    }

    /// Reads VBUS voltage, in millivolts.
    pub fn vbus_voltage(&mut self) -> Result<u16, Error> {
        let raw_value = self.read_u16(REG_ADC_VBUS_H)?;
        // 14 bit
        Ok(raw_value & 0x3fff)
    }

    /// Reads system voltage, in millivolts.
    pub fn vsys_voltage(&mut self) -> Result<u16, Error> {
        let raw_value = self.read_u16(REG_ADC_VSYS_H)?;
        // 14 bit
        Ok(raw_value & 0x3fff)
    }

    /// Reads raw value of DIE temperature sensor voltage, in 0.1 millivolts.
    pub fn tdie_voltage_raw(&mut self) -> Result<u16, Error> {
        let raw_value = self.read_u16(REG_ADC_TDIE_H)?;
        // 14 bit
        Ok(raw_value & 0x3fff)
    }

    // TODO: A lot.
    // TS(battery Temperature Sensor) control is to be implemented.
    // Li-bat charger controls(current, voltage) are to be implemented.
    // Button bat charger controls
}
