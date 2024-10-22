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
use core::ops::RangeBounds;
use embedded_hal::i2c::{Error as I2cError, ErrorKind as I2cErrorKind, I2c};
use num_enum::{FromPrimitive, IntoPrimitive};

use crate::irq::IrqStatus;

/// AXP PMU I2C address, it's same for several AXP chips.
const AXP_CHIP_ADDR: u8 = 0x34;

/// AXP PMU errors.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum Error {
    /// An I2C error occurred during the transaction.
    I2cError(I2cErrorKind),
    /// Value(voltage, timer length) exceeds hardware limit.
    ValueOutOfRange,
    /// Failed to parse a value. E.g. failed to process a value read from chip.
    ParseError,
    /// A different error occurred. The original error(mapped from) may contain more information.
    Other,
}

impl<T: I2cError> From<T> for Error {
    fn from(value: T) -> Self {
        Self::I2cError(value.kind())
    }
}

impl embedded_hal::digital::Error for Error {
    fn kind(&self) -> embedded_hal::digital::ErrorKind {
        embedded_hal::digital::ErrorKind::Other
    }
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
    Unknown,
}

/// PMU power off reason.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum PowerOffReason {
    /// PWRON pin pulled low for configured time. The same key can be used to trigger
    /// both power on and power off signal.
    PowerOffKey,
    /// Software power off by writing the power off register.
    Software,
    /// PWRON pin pulled LOW, only when PWRON is configured in EN mode.
    PowerOffEnMode,
    /// VSYS voltage dropped below limit.
    VsysUndervolt,
    /// VBUS voltage exceeds limit.
    VbusOvervolt,
    /// Any DCDC's voltage dropped below limit.
    DcdcUndervolt,
    /// Any DCDC's voltage exceeds limit.
    DcdcOvervolt,
    /// DIE temperature exceeds limit.
    DieOverheat,
    /// Unknown power on reason, should not occur in normal cases.
    Unknown,
}

/// PWRON key active duration to trigger an IRQ event.
///
/// Hardware default value is 1.5s(0b01)
#[repr(u8)]
#[derive(IntoPrimitive, FromPrimitive, Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum KeyDurationIrq {
    /// 1 second.
    T1000MS,
    /// 1.5 seconds.
    #[num_enum(default)]
    T1500MS,
    /// 2 seconds.
    T2000MS,
    /// 2.5 seconds.
    T2500MS,
}

/// PWRON key active duration to trigger a power off event.
///
/// Hardware default value is 6s(0b01)
#[repr(u8)]
#[derive(IntoPrimitive, FromPrimitive, Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum KeyDurationPowerOff {
    /// 4 seconds.
    T4S,
    /// 6 seconds.
    #[num_enum(default)]
    T6S,
    /// 8 seconds.
    T8S,
    /// 10 seconds.
    T10S,
}

/// PWRON key active duration to trigger a power on event.
///
/// Hardware default value depends on efuse.
#[repr(u8)]
#[derive(IntoPrimitive, FromPrimitive, Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum KeyDurationPowerOn {
    /// 128 milliseconds.
    #[num_enum(default)]
    T128MS,
    /// 512 milliseconds.
    T512MS,
    /// 1 second.
    T1000MS,
    /// 2 seconds.
    T2000MS,
}

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

/// Watchdog action.
///
/// Hardware defaults to [`WatchdogAction::IrqOnly`].
#[repr(u8)]
#[derive(IntoPrimitive, FromPrimitive, Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum WatchdogAction {
    /// Only send an IRQ signal.
    #[num_enum(default)]
    IrqOnly,
    /// Send an IRQ signal, and perform a system reset(reset all related PMU registers).
    ///
    /// The registers which have reset condition as "System Reset" will be reseted.
    IrqSystemReset,
    /// Pull down PWROK for 1 second, and do what [`WatchdogAction::IrqSystemReset`] does.
    IrqSystemResetPullPwrok,
    /// Restart all DCDC/LDO(power off & power on), and do what [`WatchdogAction::IrqSystemResetPullPwrok`] does.
    IrqFullRestart,
}

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
    fn get_voltage(&mut self) -> Result<u16, Error>;
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

/// Create registers consts, make the code more readable.
macro_rules! address {
    ($name:ident, $value:literal, $($key:ident, $val:literal),+ $(,)?) => {
        #[allow(dead_code)]
        const $name: u8 = $value;
        address!($($key, $val),+);
    };
    ($name:ident, $value:literal $(,)?) => {
        const $name: u8 = $value;
    };
}

#[rustfmt::skip]
address!{
    REG_PMU_STATUS0,    0x00,
    REG_PMU_STATUS1,    0x01,
    REG_CHIP_ID,        0x03,
    REG_DATABUF_START,  0x04,
    REG_PMU_CONFIG,     0x10,
    REG_BATFET_CONTROL, 0x12,
    REG_TDIE_CONTROL,   0x13,
    REG_VSYS_LOW_THRESH,0x14,
    REG_VIN_LOW_THRESH, 0x15,
    REG_IIN_HIGH_THRESH,0x16,
    REG_GAUGE_RST,      0x17,
    REG_CHARGER_GAUGE_WATCHDOG_SW,  0x18,
    REG_WATCHDOG_CONTROL,           0x19,
    REG_BAT_LOW_WARN_THRESH,        0x1A,
    REG_GPIO1_CONFIG,               0x1B,
    REG_POWERON_REASON,             0x20,
    REG_POWEROFF_REASON,            0x21,
    REG_POWEROFF_EN_BEHAVIOR,       0x22,
    REG_DCDC_PROTECT,               0x23,
    REG_POWEROFF_VBAT_LOW_THRESH,   0x24,
    REG_POWER_TIMING,               0x25,
    REG_SLEEP_WAKE_CONFIG,          0x26,
    REG_KEY_EVENT_TIME,             0x27,
    REG_FAST_PWRON_CONFIG0,         0x28,
    REG_FAST_PWRON_CONFIG1,         0x29,
    REG_FAST_PWRON_CONFIG2,         0x2A,
    REG_FAST_PWRON_CONFIG3,         0x2B,
    REG_ADC_CONTROL,    0x30,
    REG_ADC_VBAT_H,     0x34,
    REG_ADC_VBAT_L,     0x35,
    REG_ADC_TS_H,       0x36,
    REG_ADC_TS_L,       0x37,
    REG_ADC_VBUS_H,     0x38,
    REG_ADC_VBUS_L,     0x39,
    REG_ADC_VSYS_H,     0x3A,
    REG_ADC_VSYS_L,     0x3B,
    REG_ADC_TDIE_H,     0x3C,
    REG_ADC_TDIE_L,     0x3D,
    REG_ADC_GPADC_H,    0x3E,
    REG_ADC_GPADC_L,    0x3F,
    REG_IRQ_ENABLE0,    0x40,
    REG_IRQ_ENABLE1,    0x41,
    REG_IRQ_ENABLE2,    0x42,
    REG_IRQ_STATUS0,    0x48,
    REG_IRQ_STATUS1,    0x49,
    REG_IRQ_STATUS2,    0x4A,
    REG_TS_CONFIG,      0x50,
    REG_TS_HYSTER_L2H,  0x52,
    REG_TS_HYSTER_H2L,  0x53,
    REG_TSV_CHARGER_LOW,            0x54,
    REG_TSV_CHARGER_HIGH,           0x55,
    REG_TSV_WORK_LOW,               0x56,
    REG_TSV_WORK_HIGH,              0x57,
    REG_JEITA_EN,       0x58,
    REG_JEITA_IV_CONFIG,0x59,
    REG_JEITA_COOL,     0x5A,
    REG_JEITA_WARM,     0x5B,
    REG_TS_VOLT_H,      0x5C,
    REG_TS_VOLT_L,      0x5D,
    REG_RECHARGE_CONFIG,0x60,
    REG_CHARGER_IPRE,   0x61,
    REG_CHARGER_ICC,    0x62,
    REG_CHARGER_ITERM,  0x63,
    REG_CHARGER_CV,     0x64,
    REG_THERMAL_THRESH, 0x65,
    REG_CHARGER_TIMER,  0x67,
    REG_BAT_DETECT_EN,  0x68,
    REG_CHGLED_CONTROL, 0x69,
    REG_COIN_BAT_VTERM, 0x6A,
    REG_BATTERY_PERCENT,0xA4,
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
        /// Wrapper struct for the corresponding regulator.
        #[derive(Debug)]
        pub struct $regulator_name<T> {
            axp: Axp2101<T>
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

/// AXP2101 struct.
#[derive(Debug)]
pub struct Axp2101<I2C> {
    i2c: I2C,
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

/// AXP2101 implementation
///
/// TODO: make it a trait, so other PMUs fit
impl<I2C: I2c> Axp2101<I2C> {
    /// Read chip ID.
    ///
    /// It's revealed in the datasheet provided by M5Stack. On M5Stack Core2 V1.1, the raw
    /// reading is `74`(`0b01001010`), while the datasheet says it should be `0b01XX0111`.
    /// It may vary on different chip variants(e.g. customized ones).
    pub fn chip_id(&mut self) -> Result<u8, Error> {
        self.read_u8(REG_CHIP_ID)
    }

    /// Returns `true` if external power connected.
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

    /// Returns `true` if main battery connected.
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

    /// Returns current [`BatteryCurrentDirection`].
    pub fn battery_current_direction(&mut self) -> Result<BatteryCurrentDirection, Error> {
        match self.read_u8(REG_PMU_STATUS1)?.get_bits(5..=6) {
            0 => Ok(BatteryCurrentDirection::Standby),
            1 => Ok(BatteryCurrentDirection::Charging),
            2 => Ok(BatteryCurrentDirection::Discharging),
            _ => Ok(BatteryCurrentDirection::Unknown),
        }
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

    /// Returns [`BatteryChargingStatus`].
    pub fn battery_charging_status(&mut self) -> Result<BatteryChargingStatus, Error> {
        let value = self.read_u8(REG_PMU_STATUS1)?.get_bits(0..=2);
        Ok(BatteryChargingStatus::from_primitive(value))
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

    /// Shuts the PMU, powering off everything except the RTCLDO.
    ///
    /// This method does not guarantee a correct shutdown sequence or setup.
    /// Care should be taken to manage the interrupt signals.
    pub fn power_off(&mut self) -> Result<(), Error> {
        // write shutdown register
        self.write_bit(REG_PMU_CONFIG, 0, true)
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

    /// Set `true` to enable watchdog.
    ///
    /// Chip defaults to disabled.
    pub fn set_watchdog_en(&mut self, value: bool) -> Result<(), Error> {
        self.write_bit(REG_CHARGER_GAUGE_WATCHDOG_SW, 0, value)
    }

    /// Sets what a watchdog reset triggers.
    ///
    /// Chip defaults to [`WatchdogAction::IrqOnly`].
    pub fn set_watchdog_action(&mut self, value: WatchdogAction) -> Result<(), Error> {
        self.write_bits(REG_WATCHDOG_CONTROL, 4..=5, value.into())
    }

    /// Feeds watchdog.
    pub fn feed_watchdog(&mut self) -> Result<(), Error> {
        self.write_bit(REG_WATCHDOG_CONTROL, 3, true)
    }

    /// Sets TWSI watchdog timer length.
    ///
    /// The actual time length is 2 ** raw_value. For example, write 0 for 1 second,
    /// 1 for 2 seconds, 4 for 16 seconds.
    ///
    /// The maximum timer length is 128 seconds, corresponding raw value is 7(0b111).
    pub fn set_watchdog_timer_length(&mut self, value: u8) -> Result<(), Error> {
        if value > 0b111 {
            Err(Error::ValueOutOfRange)
        } else {
            self.write_bits(REG_WATCHDOG_CONTROL, 0..2, value)
        }
    }

    /// Sets low-battery warning level 2.
    ///
    /// From 5% to 20%, both end included, 1% per step.
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
    /// From 0% to 15%, both end included, 1% per step.
    pub fn set_bat_low_level1(&mut self, value: u8) -> Result<(), Error> {
        if value > 15 {
            Err(Error::ValueOutOfRange)
        } else {
            self.write_bits(REG_BAT_LOW_WARN_THRESH, 0..=3, value)
        }
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
            Ok(PowerOnReason::Unknown)
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
            Ok(PowerOffReason::Unknown)
        }
    }

    /// Set `true` to power off when DIE temperature exceeds limit(LEVEL2).
    ///
    /// Chip defaults to enabled.
    pub fn set_die_overheat_poweroff(&mut self, value: bool) -> Result<(), Error> {
        self.write_bit(REG_POWEROFF_EN_BEHAVIOR, 2, value)
    }

    /// Set `true` to power off when PWRON active more than configured OFFLEVEL time.
    ///
    /// This config is also referred as `btn_pwroff_en`
    ///
    /// Chip defaults to efuse value.
    pub fn set_pwron_key_poweroff(&mut self, value: bool) -> Result<(), Error> {
        self.write_bit(REG_POWEROFF_EN_BEHAVIOR, 1, value)
    }

    /// Sets the actual behavior of [`Axp2101::set_pwron_key_poweroff`].
    ///
    /// PMU will automatically restart, if set to `true`.
    ///
    /// Chip defaults to efuse value.
    pub fn set_pwron_key_poweroff_restart(&mut self, value: bool) -> Result<(), Error> {
        self.write_bit(REG_POWEROFF_EN_BEHAVIOR, 0, value)
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
    /// depending on how [`Axp2101::set_keep_voltage_when_wakeup`] configured.
    pub fn prepare_sleep(&mut self) -> Result<(), Error> {
        self.write_bit(REG_SLEEP_WAKE_CONFIG, 0, true)
    }

    /// Returns [`KeyDurationIrq`], the time duration PWRON key need to be active to trigger an IRQ event.
    pub fn key_duration_irq(&mut self) -> Result<KeyDurationIrq, Error> {
        let result: KeyDurationIrq = self.read_u8(REG_KEY_EVENT_TIME)?.get_bits(4..=5).into();
        Ok(result)
    }

    /// Sets PWRON key active duration for an IRQ event.
    pub fn set_key_duration_irq(&mut self, value: KeyDurationIrq) -> Result<(), Error> {
        self.write_bits(REG_KEY_EVENT_TIME, 4..=5, value.into())
    }

    /// Returns [`KeyDurationPowerOff`], the time duration PWRON key need to be active to trigger an power off event.
    pub fn key_duration_power_off(&mut self) -> Result<KeyDurationPowerOff, Error> {
        let reg_val = self.read_u8(REG_KEY_EVENT_TIME)?.get_bits(2..=3);
        Ok(KeyDurationPowerOff::from_primitive(reg_val))
    }

    /// Sets PWRON key active duration for a power off event.
    pub fn set_key_duration_power_off(&mut self, value: KeyDurationPowerOff) -> Result<(), Error> {
        self.write_bits(REG_KEY_EVENT_TIME, 2..=3, value.into())
    }

    /// Returns [`KeyDurationPowerOn`], the time duration PWRON key need to be active to trigger an power on event.
    pub fn key_duration_power_on(&mut self) -> Result<KeyDurationPowerOn, Error> {
        let reg_val = self.read_u8(REG_KEY_EVENT_TIME)?.get_bits(0..=1);
        Ok(KeyDurationPowerOn::from_primitive(reg_val))
    }

    /// Sets PWRON key active duration for a power on event.
    pub fn set_key_duration_power_on(&mut self, value: KeyDurationPowerOn) -> Result<(), Error> {
        self.write_bits(REG_KEY_EVENT_TIME, 0..=1, value.into())
    }

    // TODO: REG 0x27 fast power on implementation

    /// Returns whether DIE temperature ADC channel enabled.
    pub fn adc_status_die_temperature(&mut self) -> Result<bool, Error> {
        Ok(self.read_u8(REG_ADC_CONTROL)?.get_bit(4))
    }

    /// Sets whether to enable DIE temperature ADC channel
    pub fn set_adc_status_die_temperature(&mut self, value: bool) -> Result<(), Error> {
        self.write_bit(REG_ADC_CONTROL, 4, value)
    }

    /// Returns whether system voltage ADC channel enabled.
    pub fn adc_status_system_voltage(&mut self) -> Result<bool, Error> {
        Ok(self.read_u8(REG_ADC_CONTROL)?.get_bit(3))
    }
    /// Sets whether to enable system voltage ADC channel
    pub fn set_adc_status_system_voltage(&mut self, value: bool) -> Result<(), Error> {
        self.write_bit(REG_ADC_CONTROL, 3, value)
    }

    /// Returns whether VBUS voltage ADC channel enabled.
    pub fn adc_status_vbus_voltage(&mut self) -> Result<bool, Error> {
        Ok(self.read_u8(REG_ADC_CONTROL)?.get_bit(2))
    }
    /// Sets whether to enable VBUS voltage ADC channel
    pub fn set_adc_status_vbus_voltage(&mut self, value: bool) -> Result<(), Error> {
        self.write_bit(REG_ADC_CONTROL, 2, value)
    }

    /// Returns whether temperature sensor(TS pin) ADC channel enabled.
    pub fn adc_status_ts_pin(&mut self) -> Result<bool, Error> {
        Ok(self.read_u8(REG_ADC_CONTROL)?.get_bit(1))
    }
    /// Sets whether to enable temperature sensor(TS pin) ADC channel
    pub fn set_adc_status_ts_pin(&mut self, value: bool) -> Result<(), Error> {
        self.write_bit(REG_ADC_CONTROL, 1, value)
    }

    /// Returns whether battery voltage ADC channel enabled.
    pub fn adc_status_battery_voltage(&mut self) -> Result<bool, Error> {
        Ok(self.read_u8(REG_ADC_CONTROL)?.get_bit(0))
    }
    /// Sets whether to enable battery voltage ADC channel
    pub fn set_adc_status_battery_voltage(&mut self, value: bool) -> Result<(), Error> {
        self.write_bit(REG_ADC_CONTROL, 0, value)
    }

    // TODO: General purpose ADC implementation

    /// Reads battery voltage, in millivolts.
    pub fn battery_voltage(&mut self) -> Result<u16, Error> {
        let raw_value = self.read_u16(REG_ADC_VBAT_H)?;
        // 14 bit
        Ok(raw_value & 0x3fff)
    }

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

    /// Clears all IRQ status bits.
    pub fn irq_clear_all(&mut self) -> Result<(), Error> {
        let buf: [u8; 4] = [REG_IRQ_STATUS0, 0xFF, 0xFF, 0xFF];
        Ok(self.i2c.write(AXP_CHIP_ADDR, &buf)?)
    }

    /// Enables all IRQ signals.
    pub fn irq_enable_all(&mut self) -> Result<(), Error> {
        let buf: [u8; 4] = [REG_IRQ_ENABLE0, 0xFF, 0xFF, 0xFF];
        Ok(self.i2c.write(AXP_CHIP_ADDR, &buf)?)
    }

    /// Disables all IRQ signals.
    pub fn irq_disable_all(&mut self) -> Result<(), Error> {
        let buf: [u8; 4] = [REG_IRQ_ENABLE0, 0, 0, 0];
        Ok(self.i2c.write(AXP_CHIP_ADDR, &buf)?)
    }

    /// Returns current [`IrqStatus`]. Read more there for how to handle IRQ events.
    pub fn irq_status(&mut self) -> Result<IrqStatus, Error> {
        let mut buf: [u8; 3] = [0, 0, 0];
        self.i2c
            .write_read(AXP_CHIP_ADDR, &[REG_IRQ_STATUS0], &mut buf)?;
        Ok(IrqStatus(buf[0], buf[1], buf[2]))
    }

    /// Gets raw IRQ config registers.
    pub fn get_irq_config_raw(&mut self) -> Result<[u8; 3], Error> {
        let mut buf: [u8; 3] = [0, 0, 0];
        self.i2c
            .write_read(AXP_CHIP_ADDR, &[REG_IRQ_ENABLE0], &mut buf)?;
        Ok(buf)
    }

    /// Sets raw IRQ config registers.
    pub fn set_irq_config_raw(&mut self, buf: &[u8]) -> Result<(), Error> {
        if buf.len() != 3 {
            Err(Error::Other)
        } else {
            let write_buf: [u8; 4] = [REG_IRQ_ENABLE0, buf[0], buf[1], buf[2]];
            Ok(self.i2c.write(AXP_CHIP_ADDR, &write_buf)?)
        }
    }

    // TODO: A lot.
    // TS(battery Temperature Sensor) control is to be implemented.
    // Li-bat charger controls(current, voltage) are to be implemented.
    // Button bat charger controls

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

    /// Returns battery percentage.
    pub fn battery_percent(&mut self) -> Result<u8, Error> {
        self.read_u8(REG_BATTERY_PERCENT)
    }

    fn read_u8(&mut self, reg: u8) -> Result<u8, Error> {
        let mut buf: [u8; 1] = [0; 1];

        match self.i2c.write_read(AXP_CHIP_ADDR, &[reg], &mut buf) {
            Ok(_) => Ok(buf[0]),
            Err(e) => Err(e.into()),
        }
    }

    fn read_u16(&mut self, reg: u8) -> Result<u16, Error> {
        let mut buf: [u8; 2] = [0; 2];
        match self.i2c.write_read(AXP_CHIP_ADDR, &[reg], &mut buf) {
            Ok(_) => {
                let number = ((buf[0] as u16) << 8) + buf[1] as u16;
                Ok(number)
            }
            Err(e) => Err(e.into()),
        }
    }

    fn write_u8(&mut self, reg: u8, value: u8) -> Result<(), Error> {
        Ok(self.i2c.write(AXP_CHIP_ADDR, &[reg, value])?)
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

#[cfg(test)]
mod test {
    #![allow(unused_mut)]
    #![allow(dead_code)]
    #![allow(unused_variables)]
    #![allow(non_snake_case)]

    use super::*;
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
}
