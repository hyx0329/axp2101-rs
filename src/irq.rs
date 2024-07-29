//! IRQ stucture definitions.
//!
//! All IRQs are by default enabled, unless especially documented.
//!
//! All IRQ bits are toggled regardless of the IRQ configuration. The IRQ
//! configuration only affects the signals on the IRQ pin.
//!
//! When there's any unhandled IRQ bit, the IRQ pin is asserted. Write 1 to the
//! bits to clear them and PMU will deassert the IRQ signal.

/// AXP2101 IRQ reason.
///
/// All IRQs are by default enabled, unless especially documented.
///
/// All IRQ bits are toggled regardless of the IRQ configuration. The IRQ
/// configuration only affects the signals on the IRQ pin.
///
/// All IRQ status bits may be automatically cleared if condition changed.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Axp2101IrqReason {
    // REG 0x40 IRQ enable 0
    // REG 0x48 IRQ status 0, bits 0..7
    /// Battery underheat when discharging.
    ///
    /// "Battery Under Temperature in Work mode IRQ(bwut_irq)"
    BatteryUnderheatDischarging,
    /// Battery overheat when discharging.
    ///
    /// "Battery Over Temperature in Work mode IRQ(bwot_irq)"
    BatteryOverheatDischarging,
    /// Battery underheat when charging.
    ///
    /// "Battery Under Temperature in Charge mode IRQ(bcut_irq)"
    BatteryUnderheatCharging,
    /// Battery overheat when charging.
    ///
    /// "Battery Over Temperature in Charge mode IRQ(bcot_irq)"
    BatteryOverheatCharging,
    /// Possibly: Gauge reporting battery percentage changed.
    ///
    /// "Gauge New SOC IRQ(lowsoc_irq)"
    GaugeNewSoc,
    /// Have no idea what is this.
    ///
    /// "Gauge Watchdog Timeout IRQ(gwdt_irq)".
    GaugeWatchdogTimeout,
    /// Possibly: State Of Charge drops to Level 1(shutdown level).
    ///
    /// If SOC is higher than level1, this bit will be cleared.
    ///
    /// "SOC drop to Warning Level1 IRQ(socwl1_irq)"
    BatteryPercentWarnLevel1,
    /// Possibly: State Of Charge drops to Level 2(warning level).
    ///
    /// If SOC drops to level1(shutdown level) or higher than warning level,
    /// this bit be will cleared.
    ///
    /// "SOC drop to Warning Level2 IRQ(socwl2_irq)"
    BatteryPercentWarnLevel2,

    // REG 0x41 IRQ enable 1
    // REG 0x49 IRQ status 1, bits 0..7
    /// Power key positive edge signal. It defaults to disabled.
    ///
    /// "POWERON Positive Edge IRQ(ponpe_irq_en)"
    PowerKeyEdgePositive,
    /// Power key negative edge signal. It defaults to disabled.
    ///
    /// "POWERON Negative Edge IRQ(ponne_irq_en)"
    PowerKeyEdgeNegative,
    /// Power key long press event.
    ///
    /// "POWERON Long PRESS IRQ(ponlp_irq)"
    PowerKeyEventLong,
    /// Power key short press event.
    ///
    /// "POWERON Short PRESS IRQ(ponsp_irq_en)"
    PowerKeyEventShort,
    /// Battery removed.
    ///
    /// "Battery Remove IRQ(bremove_irq)"
    BatteryRemoved,
    /// Battery inserted.
    ///
    /// "Battery Insert IRQ(binsert_irq)"
    BatteryInserted,
    /// VBUS removed.
    ///
    /// "VBUS Remove IRQ(vremove_irq)"
    VbusRemoved,
    /// VBUS inserted.
    ///
    /// "VBUS Insert IRQ(vinsert_irq)"
    VbusInserted,

    // REG 0x42 IRQ enable 2
    // REG 0x4A IRQ status 2, bits 0..7
    /// Battery over voltage protection active.
    ///
    /// "Battery Over Voltage Protection IRQ(bovp_irq)"
    BatteryOvervoltage,
    /// Charger safety timer expired.
    ///
    /// Possibly means the battery cannot enter a working state during
    /// safe precharging stage, or the battery never fully charged during
    /// normal charging stage.
    ///
    /// "Charger Safety Timer1/2 expire IRQ(chgte_irq)"
    ChargerSafetyTimer,
    /// DIE overheat.
    ///
    /// "DIE Over Temperature level1 IRQ(dotl1_irq)"
    DieOverheat,
    /// Charger started.
    ///
    /// "Charger start IRQ(chgst_irq) enable"
    ChargingStart,
    /// Battery charging finished.
    ///
    /// "Battery charge done IRQ(chgdn_irq)"
    ChargingDone,
    /// BATFET over current protection active. It defaults to disabled.
    ///
    /// "BATFET Over Current Protection IRQ(bocp_irq)"
    BatfetOvercurrent,
    /// LDO over current protection active.
    ///
    /// "LDO Over Current IRQ(ldooc_irq)"
    LdoOvercurrent,
    /// Watchdog timer expired.
    ///
    /// "Watchdog Expire IRQ(wdexp_irq)". It defaults to disabled.
    WatchdogTimer,
}
