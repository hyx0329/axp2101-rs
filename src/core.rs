//! Core support functions and definition of AXP2101 struct.

use bit_field::BitField;
use core::ops::RangeBounds;
use embedded_hal::i2c::I2c;

use crate::error::Error;
use crate::register_addresses::*;

/// AXP2101 struct.
#[derive(Debug)]
pub struct Axp2101<I2C> {
    pub(crate) i2c: I2C,
}

impl<I2C> Axp2101<I2C> {
    /// Side-effect-free constructor.
    /// Nothing will be read or written.
    pub fn new(i2c: I2C) -> Self {
        Axp2101 { i2c }
    }

    /// Consumes the driver and gives i2c back, which is useful when there's no helper lib.
    pub fn destroy(self) -> I2C {
        self.i2c
    }
}

impl<I2C: I2c> Axp2101<I2C> {
    pub(crate) fn read_u8(&mut self, reg: u8) -> Result<u8, Error> {
        let mut buf: [u8; 1] = [0; 1];

        match self.i2c.write_read(AXP_CHIP_ADDR, &[reg], &mut buf) {
            Ok(_) => Ok(buf[0]),
            Err(e) => Err(e.into()),
        }
    }

    pub(crate) fn read_u16(&mut self, reg: u8) -> Result<u16, Error> {
        let mut buf: [u8; 2] = [0; 2];
        match self.i2c.write_read(AXP_CHIP_ADDR, &[reg], &mut buf) {
            Ok(_) => {
                let number = ((buf[0] as u16) << 8) + buf[1] as u16;
                Ok(number)
            }
            Err(e) => Err(e.into()),
        }
    }

    pub(crate) fn write_u8(&mut self, reg: u8, value: u8) -> Result<(), Error> {
        Ok(self.i2c.write(AXP_CHIP_ADDR, &[reg, value])?)
    }

    pub(crate) fn write_bit(&mut self, reg: u8, bit: usize, value: bool) -> Result<(), Error> {
        let mut reg_val = self.read_u8(reg)?;
        if reg_val.get_bit(bit) == value {
            Ok(())
        } else {
            reg_val.set_bit(bit, value);
            self.write_u8(reg, reg_val)
        }
    }

    pub(crate) fn write_bits<T: RangeBounds<usize>>(
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
