# axp2101-rs

X-Power AXP2101 driver.

**WARNING:**
**You MUST verify the actual output of your chip before using this library to avoid killing your device!**
**AXP2101 chip can be customized(for bulk purchase) to support multiple voltage range/precision and different default states.**

For AXP2101 chips, whether DLDO1/DLDO2/RTCLDO2/GPIO1 are available depends
on the customization of chip, and they are NOT available by default.

Currently implementation status of the voltage regulators:

Feature | Implemented | Customized | Notes
:- | :- | :- | :-
DCDC1 | YES
DCDC2 | YES
DCDC3 | YES
DCDC4 | YES
DCDC5 | YES | YES | conflict with GPIO1/RTCLDO2
ALDO1 | YES
ALDO2 | YES
ALDO3 | YES
ALDO4 | YES
BLDO1 | YES
BLDO2 | YES
CPUSLDO | YES | | dependent on DC4
DLDO1(0.5V to 3.4V) | YES | YES | depend on(not exceed) DCDC1, 100mV/step
DLDO2(0.5V to 1.4V) | YES | YES | depend on(not exceed) DCDC4, 50mV/step
GPIO1(REG 0x1B) | NO | YES | hardware may support Hi-Z and LOW output
RTCLDO1 | NO |  | fixed output, nothing to implement
RTCLDO2 | NO | YES | possibly fixed output, nothing to implement

**All DCDC/LDO's default voltage and power on sequence may be customized!**
**Not knowing those values may destroy your device!**

*DLDO1 and DLDO2 defaults to switchable direct output route for DCDC1 and DCDC4, and may be customized(in factory, through efuse) as LDO outputs.*

## Example

```rust
// Assume there's one i2c struct having [`embedded_hal::i2c::I2c`] implemented.
use axp2101::pmu::{
    Axp2101,
    Dcdc1,
    Error as AxpError,
    Regulator,
};

fn main() {
    /* preparations */

    let mut axp = Axp2101::new(i2c);
    let mut dcdc1: Dcdc1<_> = axp.into();
    // or this way directly
    // let mut dcdc1 = Dcdc1::new(i2c);
    let _ = dcdc1.set_voltage(3300);
    let _ = dcdc1.enable();

    // take it back
    let mut axp: Axp2101<_> = dcdc1.into();

    if let Ok(value) = axp.battery_percent() {
        log::info!("Battery percent: {}%", value)
    }
    if let Ok(value) = axp.battery_voltage() {
        log::info!("Battery voltage: {}mV", value)
    }
    if let Ok(value) = axp.battery_charging_status() {
        log::info!("Battery charging status: {:?}", value)
    }
    if let Ok(value) = axp.get_irq_config_raw() {
        log::info!("IRQ settings: {:?}", value)
    };
    if let Ok(value) = axp.get_irq_bits_raw() {
        log::info!("IRQ status bits: {:?}", value)
    };

    /* end */
}
```

For more API information, read the generated doc.

## Other devices

### M5Stack Core2 1.1

- DLDO1 works at 3.4V range, defaults to 3.3V
- DLDO2 is NOT used
- DCDC5/GPIO1/RTCLDO2 is NOT used
- DCDC2 and DCDC4 are NOT used
- **RTCLDO1 is 3.3V**, and the built-in RTC battery is not a backup for PMU.
- PMU's IRQ pin is NOT connected to the MCU.

**The values in the table below are software detect values for reference only! They are NOT REAL values measured with a multimeter!**

Regulator| Default enabled| Default voltage(mV)| Note
:- | :- | :- | :-
DCDC1| true| 3300| Direct power source for ESP32.
DCDC2| false| 1200| Not used.
DCDC3| true| 3300| Power for speaker/MBUS 3V3/ESP32(secondary).
DCDC4| false| 1800| Not used.
DCDC5| false| 1400| Not used.
ALDO1| true| 1800| Not used.
ALDO2| true| 3300| For LCD/Touch enable.
ALDO3| false| 3300| For speaker enable.
ALDO4| true| 3300| For LCD/Touch/TF-card power.
BLDO1| false| 2800| For LCD backlight.
BLDO2| false| 1500| Set this to 3300 when 5V boost output is used.
DLDO1| false| 3300| For vibration motor.
DLDO2| false| 1200| Not used.
CPUSLDO| false| 1200| Not used.

## References

- [M5Stack Core2 1.1](https://docs.m5stack.com/en/core/Core2%20v1.1)
- [XPowersLib](https://github.com/lewisxhe/XPowersLib)
- [AXP2101 stamp module](https://oshwhub.com/mondraker/axp2101_2023-11-18_20-15-19)

## License

This project is released under the MIT license.
