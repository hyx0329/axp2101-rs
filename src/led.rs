//! Charge LED(CHGLED) behavior control.

use embedded_hal::i2c::I2c;
use num_enum::{FromPrimitive, IntoPrimitive};

use crate::core::Axp2101;
use crate::error::Error;
use crate::register_addresses::REG_CHGLED_CONTROL;

/// How CHGLED is controlled.
#[repr(u8)]
#[derive(IntoPrimitive, FromPrimitive, Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ChargeLedControl {
    /// Auto pattern A.
    /// - HiZ: No charging
    /// - 25% 1Hz pull low/Hi-Z jump: Charger internal abnormal alarm
    ///     - timeout
    ///     - die/battery overheat
    /// - 25% 4Hz pull low/Hi-Z jump: Input source or battery over voltage
    /// - Pulled LOW: Charging
    TypeA,
    /// Auto pattern B.
    /// - HiZ: No VBUS, on battery.
    /// - 25% 1Hz pull low/Hi-Z jump: Charging
    /// - 25% 4Hz pull low/Hi-Z jump: Alarm
    ///     - over voltage
    ///     - overheat
    ///     - timeout
    /// - Pulled LOW: on VBUS, charge finished or no battery present
    TypeB,
    /// Manual control via register 0x69 field chgled_out_ctrl()
    #[num_enum(default)]
    Manual,
}

/// Predefined CHGLED patterns.
#[repr(u8)]
#[derive(IntoPrimitive, FromPrimitive, Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ChargeLedPattern {
    /// Hi-Z
    #[num_enum(default)]
    HiZ,
    /// Low/Hi-Z 25%/75% duty 1Hz
    OneHertz,
    /// Low/Hi-Z 25%/75% duty 4Hz
    FourHertz,
    /// Pulled low
    Low,
}

impl<I2C: I2c> Axp2101<I2C> {
    /// Sets whether to enable CHGLED.
    pub fn set_chgled_en(&mut self, value: bool) -> Result<(), Error> {
        self.write_bit(REG_CHGLED_CONTROL, 0, value)
    }

    /// Sets CHGLED pin(normally on-board LED) control source.
    pub fn set_chgled_control(&mut self, value: ChargeLedControl) -> Result<(), Error> {
        self.write_bits(REG_CHGLED_CONTROL, 1..=2, value.into())
    }

    /// Sets CHGLED pin's status manually.
    pub fn set_chgled_manually(&mut self, value: ChargeLedPattern) -> Result<(), Error> {
        // self.set_chgled_control(ChargeLedControl::Manual)?;
        self.write_bits(REG_CHGLED_CONTROL, 4..=5, value.into())
    }
}
