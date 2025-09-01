//! AXP2101 power on/off reason and related features.

use bit_field::BitField;
use embedded_hal::i2c::I2c;

use crate::core::Axp2101;
use crate::error::Error;
use crate::register_addresses::*;

/// PMU power on reason.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum PowerOnReason {
    /// PWRON pin pulled low for configured time. The same key can be used to trigger
    /// both power on and power off signal.
    PowerOnKey,
    /// IRQ pin pulled down.
    IrqPulledDown,
    /// VBUS power inserted and match GOOD condition.
    VbusInsertedGood,
    /// In charging mode and battery charged over 3.3V
    BatteryChargedOver3v3,
    /// External Li battery inserted.
    BatteryInserted,
    /// PWRON pin pulled HIGH, only when PWRON is configured in EN mode.
    PowerOnEnMode,
    /// Unknown power on reason, maybe a customized one.
    /// The raw value of the register is returned.
    Unknown(u8),
}

/// PMU power off reason.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum PowerOffReason {
    /// PWRON pin pulled low for configured time. The same key can be used to trigger
    /// both power on and power off signal.
    PowerOffKey,
    /// Software powered off by writing the power off register.
    Software,
    /// PWRON pin pulled LOW, only when PWRON is configured in EN mode.
    PowerOffEnMode,
    /// VSYS voltage dropped below limit.
    VsysUndervolt,
    /// VBUS voltage exceeds limit.
    VbusOvervolt,
    /// Any DCDC's voltage dropped below limit.
    DcdcUndervolt,
    /// Any DCDC's voltage exceeded limit.
    DcdcOvervolt,
    /// DIE temperature exceeded limit.
    DieOverheat,
    /// Unknown power off reason, should not occur in normal cases.
    /// The raw value of the register is returned.
    Unknown(u8),
}

impl<I2C: I2c> Axp2101<I2C> {
    /// Restarts the PMU, reset register values to default states.
    ///
    /// All outputs except RTCLDO and VREF will be turned off at reset state, and then
    /// PMU initiates its startup sequence.
    ///
    /// This is one of the three PMU restart methods. See [Self::set_restart_on_pwrok_low]
    /// and [Self::set_watchdog_action].
    pub fn restart(&mut self) -> Result<(), Error> {
        self.write_bit(REG_PMU_CONFIG, 1, true)
    }

    /// Shutdown the PMU, powering off everything except the RTCLDO.
    ///
    /// This method does not guarantee a correct shutdown sequence or setup.
    /// Care should be taken to manage the interrupt signals.
    pub fn shutdown(&mut self) -> Result<(), Error> {
        // write shutdown register
        self.write_bit(REG_PMU_CONFIG, 0, true)
    }

    /// Returns [`PowerOnReason`].
    ///
    /// This method assumes there's only one power on reason each time.
    pub fn power_on_reason(&mut self) -> Result<PowerOnReason, Error> {
        let raw_value = self.read_u8(REG_POWERON_REASON)?;
        if raw_value.get_bit(0) {
            Ok(PowerOnReason::PowerOnKey)
        } else if raw_value.get_bit(1) {
            Ok(PowerOnReason::IrqPulledDown)
        } else if raw_value.get_bit(2) {
            Ok(PowerOnReason::VbusInsertedGood)
        } else if raw_value.get_bit(3) {
            Ok(PowerOnReason::BatteryChargedOver3v3)
        } else if raw_value.get_bit(4) {
            Ok(PowerOnReason::BatteryInserted)
        } else if raw_value.get_bit(5) {
            Ok(PowerOnReason::PowerOnEnMode)
        } else {
            Ok(PowerOnReason::Unknown(raw_value))
        }
    }

    /// Returns [`PowerOffReason`]
    ///
    /// This method assumes there's only one power off reason each time.
    pub fn power_off_reason(&mut self) -> Result<PowerOffReason, Error> {
        let raw_value = self.read_u8(REG_POWEROFF_REASON)?;
        if raw_value.get_bit(0) {
            Ok(PowerOffReason::PowerOffKey)
        } else if raw_value.get_bit(1) {
            Ok(PowerOffReason::Software)
        } else if raw_value.get_bit(2) {
            Ok(PowerOffReason::PowerOffEnMode)
        } else if raw_value.get_bit(3) {
            Ok(PowerOffReason::VsysUndervolt)
        } else if raw_value.get_bit(4) {
            Ok(PowerOffReason::VbusOvervolt)
        } else if raw_value.get_bit(5) {
            Ok(PowerOffReason::DcdcUndervolt)
        } else if raw_value.get_bit(6) {
            Ok(PowerOffReason::DcdcOvervolt)
        } else if raw_value.get_bit(7) {
            Ok(PowerOffReason::DieOverheat)
        } else {
            Ok(PowerOffReason::Unknown(raw_value))
        }
    }
}
