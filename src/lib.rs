#![doc = include_str!("../README.md")]
#![warn(missing_docs)]
#![warn(unsafe_code)]
#![no_std]

pub mod irq;
mod pmu;

pub use pmu::{
    Aldo1, Aldo2, Aldo3, Aldo4, Axp2101, BatteryChargingStatus, BatteryCurrentDirection, Bldo1,
    Bldo2, ChargeLedControl, ChargeLedPattern, Cpusldo, Dcdc1, Dcdc2, Dcdc3, Dcdc4, Dcdc5,
    DieOverheatTempL1, Dldo1, Dldo2, Error, InputCurrentLimit, KeyDurationIrq, KeyDurationPowerOff,
    KeyDurationPowerOn, PowerOffReason, PowerOnReason, Regulator, RegulatorPin, WatchdogAction,
};
