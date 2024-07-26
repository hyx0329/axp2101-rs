# axp2101-rs

X-Power AXP2101 driver.

**WARNING:**
**You MUST verify the actual output of your chip before using this library to avoid killing your device!**
**AXP2101 chip can be customized(for bulk purchase) to support multiple voltage range/precision and different default states.**

For AXP2101 chips, whether DLDO1/DLDO2/RTCLDO2/GPIO1 are available depends
on the customization of chip, and they are NOT available by default.

Currently implementation status of the voltage regulators:

Feature | Implemented | Customed | Notes
:- | :- | :- | :-
DCDC1 | YES
DCDC2 | YES
DCDC3 | YES
DCDC4 | YES
DCDC5 | YES | | conflict with GPIO1/RTCLDO2
ALDO1 | YES
ALDO2 | YES
ALDO3 | YES
ALDO4 | YES
BLDO1 | YES
BLDO2 | YES
CPUSLDO | YES | | dependent on DC4
DLDO1(3.4V range) | YES | YES | default software implementation, depend on DCDC1, 100mV/step
DLDO1(1.4V range) | YES | YES | depend on DCDC1, 50mV/step
DLDO2(3.4V range) | YES | YES | default software implementation, depend on DCDC4, 100mV/step
DLDO2(1.4V range) | YES | YES | depend on DCDC4, 50mV/step
GPIO1(REG 0x1B) | NO | YES | hardware may support Hi-Z and LOW output
RTCLDO1 | NO |  | fixed output, nothing to implement
RTCLDO2 | NO | YES | possibly fixed output, nothing to implement

*DLDO1 and DLDO2 may be customized(in factory) as control switches for DCDC1 and DCDC4.*

To use DLDO1/DLDO2's 1.4V range mode, enable feature `dldo1-1v4` and `dldo2-1v4` respectively.

## Other devices

- M5Stack Core2 1.1
    - DLDO1 works at 3.4V range
    - DLDO2 is NOT used
    - DCDC5/GPIO1/RTCLDO2 is NOT used
    - DCDC2 and DCDC4 are NOT used

## References

- [XPowersLib](https://github.com/lewisxhe/XPowersLib)
- [AXP2101 stamp module](https://oshwhub.com/mondraker/axp2101_2023-11-18_20-15-19)
