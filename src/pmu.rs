//! X-Powers PMICs HAL
//! X-Powers AXP2101 PMIC
//! Datasheet: https://wmsc.lcsc.com/wmsc/upload/file/pdf/v2/lcsc/2305060916_X-Powers-Tech-AXP2101_C3036461.pdf
//! The datasheet contains errors/contradictions!
//! 
//! 0xFFFF is a special step size to indicate skipping some raw values when mapping some undefined gaps in voltage ranges.
//! When a range is defined with step size 0xFFFF, its start voltage becomes the steps to skip.
//! 
//! [`embedded_hal::digital::OutputPin`] implemented for [`Regulator`], so each regulator can be used as an
//! output GPIO, which is the use case on M5Stack Core2.

use core::ops::RangeBounds;

use crate::error::Error;
use bit_field::BitField;
use embedded_hal::i2c::I2c;

/// AXP PMU I2C address, it's same for several AXP chips.
const AXP_CHIP_ADDR: u8 = 0x34;
/// AXP2101 chip ID
const AXP2101_ID: u8 = 0b01_0111;

/// Create registers consts, make the code more readable.
macro_rules! address {
    ($name:ident, $value:literal, $($key:ident, $val:literal),+) => {
        const $name: u8 = $value;
        address!($($key, $val),+);
    };
    ($name:ident, $value:literal) => {
        const $name: u8 = $value;
    };
}

address!(
    REG_COMMON_STATUS0, 0x00,
    REG_COMMON_STATUS1, 0x01,
    REG_CHIP_ID, 0x03);

/// Tool for quick implementation of [`Regulator::set_voltage`].
macro_rules! chained_set_voltage {
    ($self:ident, $val:ident, $vaddr:literal, $vbits:expr, $offset:expr; $start:literal, $end:literal, $stepsize:literal $(;)?
        $($startnext:literal, $endnext:literal, $stepsizenext:tt);*) => {
        if $val >= $start && $val <= $end {
            let reg_val = (($val - $start) / $stepsize + $offset) as u8;
            $self.axp.write_bits($vaddr, $vbits, reg_val)
        } else {
            chained_set_voltage!($self, $val, $vaddr, $vbits, $offset + ($end - $start) / $stepsize + 1;
                                $($startnext, $endnext, $stepsizenext);*)
        }
    };
    ($self:ident, $val:ident, $vaddr:literal, $vbits:expr, $offset:expr; $skipstep:literal, $unused:literal, SKIP $(;)?
    $($startnext:literal, $endnext:literal, $stepsizenext:tt);*) => {
        chained_set_voltage!($self, $val, $vaddr, $vbits, $offset + $skipstep;
                                $($startnext, $endnext, $stepsizenext);*)
    };
    ($self:ident, $val:ident, $vaddr:literal, $vbits:expr, $offset:expr $(;)?) => {
        Err(Error::UnsupportedVoltage)
    };
}

/// Tool for quick implementation of [`Regulator::get_voltage`].
macro_rules! chained_get_voltage {
    ($self:ident, $val:ident, $offset:expr; $start:literal, $end:literal, $stepsize:literal $(;)?
        $($startnext:literal, $endnext:literal, $stepsizenext:tt);*) => {
        if $val <= $offset + ($end - $start) / $stepsize {
            Ok($start + ($val - $offset) * $stepsize)
        } else {
            chained_get_voltage!($self, $val, $offset + ($end - $start) / $stepsize + 1;
                                $($startnext, $endnext, $stepsizenext);*)
        }
    };
    ($self:ident, $val:ident, $offset:expr; $skipstep:literal, $unused:literal, SKIP $(;)?
        $($startnext:literal, $endnext:literal, $stepsizenext:tt);*) => {
            chained_get_voltage!($self, $val, $offset + $skipstep;
                                $($startnext, $endnext, $stepsizenext);*)
    };
    ($self:ident, $val:ident, $offset:expr $(;)?) => {
        Err(Error::ParseError)
    };
}

/// Tool for quick implementation of [`Regulator::set_voltage`] and [`Regulator::get_voltage`].
macro_rules! impl_regulator_voltage_control {
    ($vaddr:literal, $vbits:expr;
        $($vstart:literal, $vend:literal, $vstepsize:tt);+) => {
        fn set_voltage(&mut self, value: u16) -> Result<(), Error> {
            chained_set_voltage!(self, value, $vaddr, $vbits, 0; $($vstart, $vend, $vstepsize);+)
        }

        fn get_voltage(&mut self) -> Result<u16, Error> {
            let raw_value = self.axp.read_u8($vaddr)?.get_bits($vbits) as u16;
            chained_get_voltage!(self, raw_value, 0; $($vstart, $vend, $vstepsize);+)
        }
    };
}

/// Tool for quick implementation of [`Regulator`], for arbitrary regulator on AXP2101 chip.
macro_rules! impl_regulator {
    ($regulator_name:ident, $swaddr:literal, $swbit:literal, $vaddr:literal, $vbits:expr;
        $($vstart:literal, $vend:literal, $vstepsize:tt);+
    ) => {
        /// Wrapper struct for $regulator
        #[derive(Debug)]
        pub struct $regulator_name<T> {
            axp: Axp2101<T>
        }

        impl<I: I2c> Regulator for $regulator_name<I> {
            impl_regulator_voltage_control!{$vaddr, $vbits; $($vstart, $vend, $vstepsize);+}

            fn enable(&mut self) -> Result<(), Error> {
                self.axp.write_bit($swaddr, $swbit, true)
            }

            fn disable(&mut self) -> Result<(), Error> {
                self.axp.write_bit($swaddr, $swbit, false)
            }
        }

        impl<I> From<Axp2101<I>> for $regulator_name<I> {
            fn from(value: Axp2101<I>) -> $regulator_name<I> {
                $regulator_name{ axp: value }
            }
        }

        impl<I> From<$regulator_name<I>> for Axp2101<I> {
            fn from(value: $regulator_name<I>) -> Axp2101<I> {
                let $regulator_name::<I>{ axp } = value;
                axp
            }
        }
    }
}

/// AXP2101 struct
#[derive(Debug)]
pub struct Axp2101<I2C> {
    i2c: I2C,
}

/// AXP2101 implementation
/// TODO: make it a trait, so other PMUs fit
impl<I2C: I2c> Axp2101<I2C>
{
    /// Side-effect-free constructor.
    /// Nothing will be read or written before `init()` call.
    pub fn new(i2c: I2C) -> Self {
        Axp2101 { i2c }
    }

    /// Consumes the driver and give i2c back, which is useful when there's no helper lib.
    pub fn destroy(self) -> I2C {
        self.i2c
    }

    /// Check bus connection and verify chip ID. Returns chip version.
    pub fn check(&mut self) -> Result<u8, Error> {
        let chip_info = self.read_u8(0x03)?;
        let chip_id = chip_info.get_bits(0..=3) | (chip_info.get_bits(6..=7) << 4);
        match chip_id {
            AXP2101_ID => Ok(chip_info.get_bits(4..=5)),
            _ => Err(Error::UnknownChip(chip_info)),
        }
    }

    /// Returns `true` if external power connected.
    pub fn vbus_present(&mut self) -> Result<bool, Error> {
        Ok(self.read_u8(REG_COMMON_STATUS0)?.get_bit(5))
    }

    /// Returns `true` if main battery connected.
    pub fn battery_present(&mut self) -> Result<bool, Error> {
        Ok(self.read_u8(REG_COMMON_STATUS0)?.get_bit(3))
    }

    /// Return `true` if BATFET is opened

    fn read_u8(&mut self, reg: u8) -> Result<u8, Error> {
        let mut buf: [u8; 1] = [0; 1];

        match self.i2c.write_read(AXP_CHIP_ADDR, &[reg], &mut buf) {
            Ok(_) => Ok(buf[0]),
            Err(e) => Err(e.into()),
        }
    }

    fn write_u8(&mut self, reg: u8, value: u8) -> Result<(), Error> {
        self.i2c.write(AXP_CHIP_ADDR, &[reg, value])?;
        Ok(())
    }

    fn write_bit(&mut self, reg: u8, bit: usize, value: bool) -> Result<(), Error> {
        let mut reg_val = self.read_u8(reg)?;
        if reg_val.get_bit(bit) == value {
            Ok(())
        } else {
            reg_val.set_bit(bit, value);
            self.write_u8(reg, reg_val)
        }
    }

    fn write_bits<T: RangeBounds<usize>>(
        &mut self,
        reg: u8,
        range: T,
        value: u8,
    ) -> Result<(), Error> {
        let mut reg_val = self.read_u8(reg)?;
        reg_val.set_bits(range, value);
        self.write_u8(reg, reg_val)
    }
}

pub trait Regulator {
    /// Set regulator voltage, in millivolt. Note the voltage value is always
    /// "floored" to the nearest value, within the supported range.
    fn set_voltage(&mut self, value: u16) -> Result<(), Error>;
    /// Get regulator voltage, in millivolt.
    fn get_voltage(&mut self) -> Result<u16, Error>;
    /// Turn on the regulator.
    fn enable(&mut self) -> Result<(), Error>;
    /// Turn off the regulator.
    fn disable(&mut self) -> Result<(), Error>;
}

impl embedded_hal::digital::ErrorType for dyn Regulator {
    type Error = Error;
}

impl embedded_hal::digital::OutputPin for dyn Regulator {
    fn set_high(&mut self) -> Result<(), Self::Error> {
        self.enable()
    }

    fn set_low(&mut self) -> Result<(), Self::Error> {
        self.disable()
    }
}

// All numbers are validated against XPowersLib(the cpp driver by LilyGO), the datasheet contains errors and only serves a reference.
impl_regulator!(Dcdc1,  0x80, 0, 0x82, 0..=4;   1500, 3400, 100);
impl_regulator!(Dcdc2,  0x80, 1, 0x83, 0..=6;   500, 1200, 10;      1220, 1540, 20);
impl_regulator!(Dcdc3,  0x80, 2, 0x84, 0..=6;   500, 1200, 10;      1220, 1540, 20;     1600, 3400, 100);
impl_regulator!(Dcdc4,  0x80, 3, 0x85, 0..=6;   500, 1200, 10;      1220, 1840, 20);
// DCDC5 shares the pin with GPIO1, the special case is 1200mV, which happens to fit in this macro, in a special way
impl_regulator!(Dcdc5,  0x80, 4, 0x86, 0..=4;   1400,3700, 100;     1, 0, SKIP;       1200, 1200, 1);
impl_regulator!(Aldo1,  0x90, 0, 0x92, 0..=4;   500, 3500, 100);
impl_regulator!(Aldo2,  0x90, 1, 0x93, 0..=4;   500, 3500, 100);
impl_regulator!(Aldo3,  0x90, 2, 0x94, 0..=4;   500, 3500, 100);
impl_regulator!(Aldo4,  0x90, 3, 0x95, 0..=4;   500, 3500, 100);
impl_regulator!(Bldo1,  0x90, 4, 0x96, 0..=4;   500, 3500, 100);
impl_regulator!(Bldo2,  0x90, 5, 0x97, 0..=4;   500, 3500, 100);
// datasheet should be wrong. depend on DCDC4
impl_regulator!(Cpusldo,0x90, 6, 0x98, 0..=4;   500, 1400, 50);
// datasheet should be wrong. depend on DCDC1
#[cfg(not(feature = "dldo1-1v4"))]
impl_regulator!(Dldo1,  0x90, 7, 0x99, 0..=4;   500, 3400, 100);
#[cfg(feature = "dldo1-1v4")]
impl_regulator!(Dldo1,  0x90, 7, 0x99, 0..=4;   500, 1400, 50);
// datasheet should be wrong. depend on DCDC4
#[cfg(not(feature = "dldo2-1v4"))]
impl_regulator!(Dldo2,  0x91, 0, 0x9A, 0..=4;   500, 3400, 100);
#[cfg(feature = "dldo2-1v4")]
impl_regulator!(Dldo2,  0x91, 0, 0x9A, 0..=4;   500, 1400, 50);



#[cfg(test)]
mod test{
    #![allow(unused_mut)]
    #![allow(dead_code)]
    #![allow(unused_variables)]
    #![allow(non_snake_case)]

    use paste::paste;
    use embedded_hal::i2c::{I2c, Operation, ErrorKind, NoAcknowledgeSource};
    use super::{*};

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

        fn write_read(&mut self, address: u8, write: &[u8], read: &mut [u8]) -> Result<(), Self::Error> {
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
        let i2c = FakeI2c {address: addr, data: &mut data};
        let mut axp = Axp2101::new(i2c);
        let _ = axp.write_bits(0, ..=3, 0b1101);
        assert_eq!(0b0101_1101, data[1]);
    }

    #[test]
    fn test_chip_id_check() {
        let mut data = [REG_CHIP_ID, 0b0110_0111];
        let addr = AXP_CHIP_ADDR;
        let i2c = FakeI2c {address: addr, data: &mut data};
        let mut axp = Axp2101::new(i2c);

        match axp.check() {
            Ok(value) => assert_eq!(value, 0b10, "Chip ID error!"),
            Err(_) => panic!("Chip ID checking failed! But it shouldn't!"),
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

    macro_rules! test_get_voltage {
        ($mod_name:ident, $vaddr:literal; $volt:literal, $reg_val:literal $(;)? $($volt2:literal, $reg_val2:literal);*) => {
            let mut data = [$vaddr, $reg_val];
            let i2c = FakeI2c{address: AXP_CHIP_ADDR, data: &mut data};
            let axp = Axp2101{i2c};
            let mut regulator = $mod_name{axp};
            match regulator.get_voltage() {
                Ok(value) => assert_eq!($volt, value, "Voltage mismatch! Expect {}mV, got {}mV!", $volt, value),
                Err(e) => panic!("Failed to get voltage from {:?}, {:?}", regulator, e),
            }
            test_get_voltage!($mod_name, $vaddr; $($volt2, $reg_val2);*);
        };
        ($mod_name:ident, $vaddr:literal; ) => {};
    }

    macro_rules! test_toggle_switch {
        ($mod_name:ident, $swaddr:literal; $swbit:literal $(;)? $($volt2:literal, $reg_val2:literal);*) => {
            let mut data = [$vaddr, $reg_val];
            let i2c = FakeI2c{address: AXP_CHIP_ADDR, data: &mut data};
            let axp = Axp2101{i2c};
            let mut regulator = $mod_name{axp};
            match regulator.get_voltage() {
                Ok(value) => assert_eq!($volt, value, "Voltage mismatch! Expect {}mV, got {}mV!", $volt, value),
                Err(e) => panic!("Failed to get voltage from {:?}, {:?}", regulator, e),
            }
            test_get_voltage!($mod_name, $vaddr; $($volt2, $reg_val2);*);
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
                fn [< test_ $mod_name _get_voltage >]() {
                    test_get_voltage!($mod_name, $vaddr; $($volt, $reg_val);*);
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

    gen_regulator_test!{Dcdc1, 0x82, 0x80, 0;   1500, 0b00000; 1600, 0b00001; 3400, 0b10011}
    gen_regulator_test!{Dcdc2, 0x83, 0x80, 1;    500, 0b0000000; 1200, 0b1000110; 1220, 0b1000111; 1540, 0b1010111}
    gen_regulator_test!{Dcdc3, 0x84, 0x80, 2;    500, 0b0000000; 1200, 0b1000110; 1220, 0b1000111; 1540, 0b1010111; 3400, 0b1101010}
    gen_regulator_test!{Dcdc4, 0x85, 0x80, 3;    500, 0b0000000; 1200, 0b1000110; 1220, 0b1000111; 1840, 0b1100110}
    gen_regulator_test!{Dcdc5, 0x86, 0x80, 4;   1400, 0b00000; 3700, 0b10111; 1200, 0b11001}
    gen_regulator_test!(Aldo1,  0x92, 0x90, 0;   500, 0b00000; 3500, 0b11110);
    gen_regulator_test!(Aldo2,  0x93, 0x90, 1;   500, 0b00000; 3500, 0b11110);
    gen_regulator_test!(Aldo3,  0x94, 0x90, 2;   500, 0b00000; 3500, 0b11110);
    gen_regulator_test!(Aldo4,  0x95, 0x90, 3;   500, 0b00000; 3500, 0b11110);
    gen_regulator_test!(Bldo1,  0x96, 0x90, 4;   500, 0b00000; 3500, 0b11110);
    gen_regulator_test!(Bldo2,  0x97, 0x90, 5;   500, 0b00000; 3500, 0b11110);
    gen_regulator_test!(Cpusldo,0x98, 0x90, 6;   500, 0b00000; 1400, 0b10010);
    #[cfg(not(feature = "dldo1-1v4"))]
    gen_regulator_test!(Dldo1,  0x99, 0x90, 7;   500, 0b00000; 3400, 0b11101);
    #[cfg(feature = "dldo1-1v4")]
    gen_regulator_test!(Dldo1,  0x99, 0x90, 7;   500, 0b00000; 1400, 0b10010);
    #[cfg(not(feature = "dldo2-1v4"))]
    gen_regulator_test!(Dldo2,  0x9A, 0x91, 0;   500, 0b00000; 3400, 0b11101);
    #[cfg(feature = "dldo2-1v4")]
    gen_regulator_test!(Dldo2,  0x9A, 0x91, 0;   500, 0b00000; 1400, 0b10010);

}