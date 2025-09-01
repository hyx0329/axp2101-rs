#![cfg(test)]
#![allow(unused_mut)]
#![allow(dead_code)]
#![allow(unused_variables)]
#![allow(non_snake_case)]

use crate::core::Axp2101;
use crate::register_addresses::*;
use crate::regulator::*;

use bit_field::BitField;
use embedded_hal::i2c::{ErrorKind, I2c, NoAcknowledgeSource, Operation};
use paste::paste;

#[derive(Debug)]
struct FakeI2c<'a> {
    address: u8,
    data: &'a mut [u8],
}

impl embedded_hal::i2c::ErrorType for FakeI2c<'_> {
    type Error = ErrorKind;
}

impl I2c for FakeI2c<'_> {
    fn write(&mut self, address: u8, write: &[u8]) -> Result<(), Self::Error> {
        if self.address != address {
            Err(ErrorKind::NoAcknowledge(NoAcknowledgeSource::Address))
        } else if self.data[0] == write[0] {
            for (dst, src) in self.data.iter_mut().zip(write) {
                *dst = *src
            }
            Ok(())
        } else {
            // No action
            Ok(())
        }
    }

    fn write_read(
        &mut self,
        address: u8,
        write: &[u8],
        read: &mut [u8],
    ) -> Result<(), Self::Error> {
        if self.address != address {
            Err(ErrorKind::NoAcknowledge(NoAcknowledgeSource::Address))
        } else if self.data[0] == write[0] {
            for (src, dst) in self.data[1..].iter().zip(read) {
                *dst = *src;
            }
            Ok(())
        } else {
            // No action
            Ok(())
        }
    }

    /// unused dummy implementation
    fn transaction(
        &mut self,
        address: u8,
        operations: &mut [Operation<'_>],
    ) -> Result<(), Self::Error> {
        Err(ErrorKind::Other)
    }
}

#[test]
fn test_axp2101_write_bits() {
    let mut data = [00, 0b0101_0000];
    let addr = AXP_CHIP_ADDR;
    let i2c = FakeI2c {
        address: addr,
        data: &mut data,
    };
    let mut axp = Axp2101::new(i2c);
    let _ = axp.write_bits(0, ..=3, 0b1101);
    assert_eq!(0b0101_1101, data[1]);
}

#[test]
fn test_axp2101_read_u16() {
    let mut data = [00, 0xAB, 0xCD];
    let addr = AXP_CHIP_ADDR;
    let i2c = FakeI2c {
        address: addr,
        data: &mut data,
    };
    let mut axp = Axp2101::new(i2c);
    match axp.read_u16(0) {
        Ok(value) => assert_eq!(0xabcd, value),
        Err(e) => assert!(true, "Error: {:?}", e),
    };
}

#[test]
fn test_chip_id_read() {
    let chip_id = 0b0110_0111;
    let mut data = [REG_CHIP_ID, chip_id];
    let addr = AXP_CHIP_ADDR;
    let i2c = FakeI2c {
        address: addr,
        data: &mut data,
    };
    let mut axp = Axp2101::new(i2c);

    match axp.chip_id() {
        Ok(value) => assert_eq!(value, chip_id, "Chip ID error!"),
        Err(_) => panic!("Chip ID reading failed! But it shouldn't!"),
    }
}

macro_rules! test_set_voltage {
    ($mod_name:ident, $vaddr:literal; $volt:literal, $reg_val:literal $(;)? $($volt2:literal, $reg_val2:literal);*) => {
        let mut data = [$vaddr, 0];
        let i2c = FakeI2c{address: AXP_CHIP_ADDR, data: &mut data};
        let axp = Axp2101{i2c};
        let mut regulator = $mod_name{axp};
        match regulator.set_voltage($volt) {
            Ok(_) => {},
            Err(e) => panic!("Failed to set voltage value {} for {:?}, {:?}", $volt, regulator, e),
        }
        assert_eq!($reg_val, data[1], "Error occurs when setting {}mV!", $volt);
        test_set_voltage!($mod_name, $vaddr; $($volt2, $reg_val2);*);
    };
    ($mod_name:ident, $vaddr:literal; ) => {};
}

macro_rules! test_read_voltage {
    ($mod_name:ident, $vaddr:literal; $volt:literal, $reg_val:literal $(;)? $($volt2:literal, $reg_val2:literal);*) => {
        let mut data = [$vaddr, $reg_val];
        let i2c = FakeI2c{address: AXP_CHIP_ADDR, data: &mut data};
        let axp = Axp2101{i2c};
        let mut regulator = $mod_name{axp};
        match regulator.voltage() {
            Ok(value) => assert_eq!($volt, value, "Voltage mismatch! Expect {}mV, got {}mV!", $volt, value),
            Err(e) => panic!("Failed to get voltage from {:?}, {:?}", regulator, e),
        }
        test_read_voltage!($mod_name, $vaddr; $($volt2, $reg_val2);*);
    };
    ($mod_name:ident, $vaddr:literal; ) => {};
}

/// Generate tests for something has [`Regulator`] implemented.
///
/// - `mod_name`: the target struct to test.
/// - `vaddr`: the register address to set the voltage.
/// - `swaddr`: the register address to enable the regulator.
/// - `swbit`: the exact bit in `swaddr` to toggle the regulator.
/// - `volt, reg_val`: voltage and matching register value pairs, as the test case.
macro_rules! gen_regulator_test {
    ($mod_name:ident, $vaddr:literal, $swaddr:literal, $swbit:literal $(;)? $($volt:literal, $reg_val:literal);*) => {
        paste! {
            #[test]
            fn [< test_ $mod_name _set_voltage >]() {
                test_set_voltage!($mod_name, $vaddr; $($volt, $reg_val);*);
            }
        }

        paste! {
            #[test]
            fn [< test_ $mod_name _read_voltage >]() {
                test_read_voltage!($mod_name, $vaddr; $($volt, $reg_val);*);
            }
        }

        paste! {
            #[test]
            fn [< test_ $mod_name _switch_off >]() {
                let mut data = [$swaddr, 0b11111111];
                let i2c = FakeI2c{address: AXP_CHIP_ADDR, data: &mut data};
                let axp = Axp2101{i2c};
                let mut regulator = $mod_name{axp};
                match regulator.disable() {
                    Ok(_) => {},
                    Err(e) => panic!("Failed to disable regulator {:?}, {:?}", regulator, e),
                }
                assert_eq!(false, data[1].get_bit($swbit), "Abnormal register value for disabled state! reg_val {:#8b}", data[1]);
            }
        }

        paste! {
            #[test]
            fn [< test_ $mod_name _switch_on >]() {
                let mut data = [$swaddr, 0];
                let i2c = FakeI2c{address: AXP_CHIP_ADDR, data: &mut data};
                let axp = Axp2101{i2c};
                let mut regulator = $mod_name{axp};
                match regulator.enable() {
                    Ok(_) => {},
                    Err(e) => panic!("Failed to disable regulator {:?}, {:?}", regulator, e),
                }
                assert_eq!(true, data[1].get_bit($swbit), "Abnormal register value for enabled state! reg_val {:#8b}", data[1]);
            }
        }
    };
}

// Datasheet values may not match the actual product! Do not blindly trust the datasheet!

gen_regulator_test! {Dcdc1, 0x82, 0x80, 0;   1500, 0b00000; 1600, 0b00001; 3400, 0b10011}
gen_regulator_test! {Dcdc2, 0x83, 0x80, 1;    500, 0b0000000; 1200, 0b1000110; 1220, 0b1000111; 1540, 0b1010111}
gen_regulator_test! {Dcdc3, 0x84, 0x80, 2;    500, 0b0000000; 1200, 0b1000110; 1220, 0b1000111; 1540, 0b1010111; 3400, 0b1101010}
gen_regulator_test! {Dcdc4, 0x85, 0x80, 3;    500, 0b0000000; 1200, 0b1000110; 1220, 0b1000111; 1840, 0b1100110}
gen_regulator_test! {Dcdc5, 0x86, 0x80, 4;   1400, 0b00000; 3700, 0b10111; 1200, 0b11001}
gen_regulator_test!(Aldo1,  0x92, 0x90, 0;   500, 0b00000; 3500, 0b11110);
gen_regulator_test!(Aldo2,  0x93, 0x90, 1;   500, 0b00000; 3500, 0b11110);
gen_regulator_test!(Aldo3,  0x94, 0x90, 2;   500, 0b00000; 3500, 0b11110);
gen_regulator_test!(Aldo4,  0x95, 0x90, 3;   500, 0b00000; 3500, 0b11110);
gen_regulator_test!(Bldo1,  0x96, 0x90, 4;   500, 0b00000; 3500, 0b11110);
gen_regulator_test!(Bldo2,  0x97, 0x90, 5;   500, 0b00000; 3500, 0b11110);
gen_regulator_test!(Cpusldo,0x98, 0x90, 6;   500, 0b00000; 1400, 0b10010);
gen_regulator_test!(Dldo1,  0x99, 0x90, 7;   500, 0b00000; 3400, 0b11101);
gen_regulator_test!(Dldo2,  0x9A, 0x91, 0;   500, 0b00000; 1400, 0b10010);
