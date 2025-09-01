//! Common address constants.

/// AXP PMU I2C address, it's same for several AXP chips.
pub const AXP_CHIP_ADDR: u8 = 0x34;

/// Create registers consts, make the code more readable.
macro_rules! address {
    ($name:ident, $value:literal, $($key:ident, $val:literal),+ $(,)?) => {
        #[allow(dead_code, missing_docs)]
        pub const $name: u8 = $value;
        address!($($key, $val),+);
    };
    ($name:ident, $value:literal $(,)?) => {
        #[allow(missing_docs)]
        pub const $name: u8 = $value;
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
