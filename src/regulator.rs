//! X-Powers AXP2101 PMIC driver implementation
//!
//! Multiple datasheets are required to fully understand this PMU.
//!
//! - Datasheet from M5Stack: <https://m5stack.oss-cn-shenzhen.aliyuncs.com/resource/docs/products/core/Core2%20v1.1/axp2101.pdf>
//! - Dev pack from a reseller(need free account) on OSHWHUB: <https://oshwhub.com/mondraker/axp2101_2023-11-18_20-15-19>
//! - AXP2585 shares a similar register layout: <https://bbs.aw-ol.com/assets/uploads/files/1650363815445-axp2585-datasheet-v1.2.pdf>
//!
//! The datasheet contains a lot of errors/contradictions! Be careful!
//!
//! Note: "RWAC" in datasheet probably means "Read & Write, Always Clear", and
//! "RW1C" possibily means "Read & Write, write 1 to Clear".
//!
//! [`embedded_hal::digital::OutputPin`] implemented for every regulator(DCDCx, xLDOx) so they can be used as
//! output GPIOs/switches, which is the use case on M5Stack Core2.
//!
//! Not every PMU feature is implemented. The register addresss consts not used indicate the features not implemented.
//!
#![allow(rustdoc::private_intra_doc_links)]

use bit_field::BitField;
use embedded_hal::i2c::I2c;

use crate::core::Axp2101;
use crate::error::Error;

/// Common trait for all regulators.
pub trait Regulator {
    /// Turn on the regulator.
    fn enable(&mut self) -> Result<(), Error>;
    /// Turn off the regulator.
    fn disable(&mut self) -> Result<(), Error>;
    /// Returns if the regulator is enabled.
    fn status(&mut self) -> Result<bool, Error>;
    /// Set regulator voltage, in millivolt. Note the voltage value is always
    /// "floored" to the nearest value, within the supported range.
    fn set_voltage(&mut self, value: u16) -> Result<(), Error>;
    /// Get regulator voltage, in millivolt.
    fn voltage(&mut self) -> Result<u16, Error>;
}

/// Wrapper for a regulator to make it compatible with [embedded_hal::digital::OutputPin].
///
/// Example:
/// ```rust,ignore
/// let r = Dcdc1::new(i2c);
/// let mut pin = RegulatorPin::new(r);
/// pin.set_high().unwrap();
/// ```
pub struct RegulatorPin<T> {
    regulator: T,
}

impl<T> RegulatorPin<T>
where
    T: Regulator,
{
    /// Create a RegulatorPin from a Regulator
    pub fn new(regulator: T) -> Self {
        Self { regulator }
    }
}

impl<T> embedded_hal::digital::OutputPin for RegulatorPin<T>
where
    T: Regulator,
{
    fn set_high(&mut self) -> Result<(), Self::Error> {
        self.regulator.enable()
    }

    fn set_low(&mut self) -> Result<(), Self::Error> {
        self.regulator.disable()
    }
}

impl<T> embedded_hal::digital::ErrorType for RegulatorPin<T>
where
    T: Regulator,
{
    type Error = Error;
}

/// Tool for quick implementation of [`Regulator::set_voltage`].
macro_rules! chained_set_voltage {
    ($self:ident, $val:ident, $vaddr:literal, $vbits:expr, $offset:expr; $start:literal, $end:literal, $stepsize:literal $(;)?
        $($startnext:literal, $endnext:literal, $stepsizenext:tt);*) => {
        if ($start..=$end).contains(&$val) {
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
        Err(Error::ValueOutOfRange)
    };
}

/// Tool for quick implementation of [`Regulator::voltage`].
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

/// Tool for quick implementation of [`Regulator::set_voltage`] and [`Regulator::voltage`].
macro_rules! impl_regulator_voltage_control {
    ($vaddr:literal, $vbits:expr;
        $($vstart:literal, $vend:literal, $vstepsize:tt);+) => {
        fn set_voltage(&mut self, value: u16) -> Result<(), Error> {
            chained_set_voltage!(self, value, $vaddr, $vbits, 0; $($vstart, $vend, $vstepsize);+)
        }

        fn voltage(&mut self) -> Result<u16, Error> {
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
        /// Wrapper struct for the corresponding regulator.
        #[derive(Debug)]
        pub struct $regulator_name<T> {
            pub(crate) axp: Axp2101<T>
        }

        impl<I2C: I2c> $regulator_name<I2C> {
            /// Creates the regulator directly from the I2C struct.
            pub fn new(i2c: I2C) -> Self {
                Self {
                    axp: Axp2101::new(i2c),
                }
            }
        }

        impl<I: I2c> Regulator for $regulator_name<I> {
            impl_regulator_voltage_control!{$vaddr, $vbits; $($vstart, $vend, $vstepsize);+}

            fn enable(&mut self) -> Result<(), Error> {
                self.axp.write_bit($swaddr, $swbit, true)
            }

            fn disable(&mut self) -> Result<(), Error> {
                self.axp.write_bit($swaddr, $swbit, false)
            }

            fn status(&mut self) -> Result<bool, Error> {
                Ok(self.axp.read_u8($swaddr)?.get_bit($swbit))
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
impl_regulator!(Dldo1,  0x90, 7, 0x99, 0..=4;   500, 3400, 100);
impl_regulator!(Dldo2,  0x91, 0, 0x9A, 0..=4;   500, 1400, 50);
