#![doc = include_str!("../README.md")]
#![warn(missing_docs)]
#![warn(unsafe_code)]
#![no_std]

pub mod auxiliary;
pub mod charging;
pub mod core;
pub mod error;
pub mod irq;
pub mod led;
pub mod power_key;
pub mod power_onoff;
pub mod prelude;
pub mod register_addresses;
pub mod regulator;
pub mod watchdog;

#[cfg(test)]
mod test;
