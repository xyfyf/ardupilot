#include "AP_BattMonitor_config.h"

#if AP_BATTERY_ANALOG_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include <AP_BoardConfig/AP_BoardConfig.h>

#include "AP_BattMonitor_Analog.h"

extern const AP_HAL::HAL& hal;

#if defined(HAL_EFT_CAAC_BATT_VOLT_CALIB) && HAL_EFT_CAAC_BATT_VOLT_CALIB
/*
 * EFT_CAAC 板载分压采样非线性校正表
 * 输入：经 VOLT_MULT 换算后的软件实测电压 (V)
 * 输出：与标准表计一致的真实电压 (V)
 * 校准数据：12.3~80V 共 69 点（2026 实测，全范围逐伏校准）
 */
struct EFT_CAAC_BattVoltLUT {
    float measured_v;
    float actual_v;
};

static const EFT_CAAC_BattVoltLUT eft_caac_batt_volt_lut[] = {
    {11.88f, 12.3f},
    {11.98f, 12.4f},
    {12.60f, 13.0f},
    {13.64f, 14.0f},
    {14.67f, 15.0f},
    {15.69f, 16.0f},
    {16.72f, 17.0f},
    {17.73f, 18.0f},
    {18.75f, 19.0f},
    {19.74f, 20.0f},
    {20.76f, 21.0f},
    {21.78f, 22.0f},
    {22.78f, 23.0f},
    {23.81f, 24.0f},
    {24.83f, 25.0f},
    {25.85f, 26.0f},
    {26.87f, 27.0f},
    {27.88f, 28.0f},
    {28.91f, 29.0f},
    {29.92f, 30.0f},
    {30.94f, 31.0f},
    {31.95f, 32.0f},
    {32.95f, 33.0f},
    {33.96f, 34.0f},
    {34.99f, 35.0f},
    {36.00f, 36.0f},
    {37.03f, 37.0f},
    {38.04f, 38.0f},
    {39.09f, 39.0f},
    {40.10f, 40.0f},
    {41.13f, 41.0f},
    {42.14f, 42.0f},
    {43.16f, 43.0f},
    {44.19f, 44.0f},
    {45.18f, 45.0f},
    {46.20f, 46.0f},
    {47.22f, 47.0f},
    {48.24f, 48.0f},
    {49.26f, 49.0f},
    {50.28f, 50.0f},
    {51.35f, 51.0f},
    {52.36f, 52.0f},
    {53.39f, 53.0f},
    {54.41f, 54.0f},
    {55.43f, 55.0f},
    {56.45f, 56.0f},
    {57.47f, 57.0f},
    {58.47f, 58.0f},
    {59.49f, 59.0f},
    {60.51f, 60.0f},
    {61.53f, 61.0f},
    {62.56f, 62.0f},
    {63.58f, 63.0f},
    {64.63f, 64.0f},
    {65.65f, 65.0f},
    {66.68f, 66.0f},
    {67.69f, 67.0f},
    {68.72f, 68.0f},
    {69.74f, 69.0f},
    {70.74f, 70.0f},
    {71.76f, 71.0f},
    {72.79f, 72.0f},
    {73.81f, 73.0f},
    {74.85f, 74.0f},
    {75.87f, 75.0f},
    {76.89f, 76.0f},
    {77.92f, 77.0f},
    {78.94f, 78.0f},
    {79.97f, 79.0f},
    {81.00f, 80.0f},
};

static float eft_caac_correct_battery_voltage(float measured_v)
{
    const uint8_t lut_size = ARRAY_SIZE(eft_caac_batt_volt_lut);

    if (measured_v <= eft_caac_batt_volt_lut[0].measured_v) {
        return linear_interpolate(
            eft_caac_batt_volt_lut[0].actual_v,
            eft_caac_batt_volt_lut[1].actual_v,
            measured_v,
            eft_caac_batt_volt_lut[0].measured_v,
            eft_caac_batt_volt_lut[1].measured_v);
    }

    if (measured_v >= eft_caac_batt_volt_lut[lut_size - 1].measured_v) {
        return linear_interpolate(
            eft_caac_batt_volt_lut[lut_size - 2].actual_v,
            eft_caac_batt_volt_lut[lut_size - 1].actual_v,
            measured_v,
            eft_caac_batt_volt_lut[lut_size - 2].measured_v,
            eft_caac_batt_volt_lut[lut_size - 1].measured_v);
    }

    for (uint8_t i = 0; i < lut_size - 1; i++) {
        if (measured_v <= eft_caac_batt_volt_lut[i + 1].measured_v) {
            return linear_interpolate(
                eft_caac_batt_volt_lut[i].actual_v,
                eft_caac_batt_volt_lut[i + 1].actual_v,
                measured_v,
                eft_caac_batt_volt_lut[i].measured_v,
                eft_caac_batt_volt_lut[i + 1].measured_v);
        }
    }

    return measured_v;
}
#endif  // HAL_EFT_CAAC_BATT_VOLT_CALIB

const AP_Param::GroupInfo AP_BattMonitor_Analog::var_info[] = {

    // @Param: VOLT_PIN
    // @DisplayName: Battery Voltage sensing pin
    // @Description: Sets the analog input pin that should be used for voltage monitoring.
    // @Values: -1:Disabled, 2:Pixhawk/Pixracer/Navio2/Pixhawk2_PM1, 5:Navigator, 13:Pixhawk2_PM2/CubeOrange_PM2, 14:CubeOrange, 16:Durandal, 100:PX4-v1
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO("VOLT_PIN", 1, AP_BattMonitor_Analog, _volt_pin, AP_BATT_VOLT_PIN),

    // @Param: CURR_PIN
    // @DisplayName: Battery Current sensing pin
    // @Description: Sets the analog input pin that should be used for current monitoring.
    // @Values: -1:Disabled, 3:Pixhawk/Pixracer/Navio2/Pixhawk2_PM1, 4:CubeOrange_PM2/Navigator, 14:Pixhawk2_PM2, 15:CubeOrange, 17:Durandal, 101:PX4-v1
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO("CURR_PIN", 2, AP_BattMonitor_Analog, _curr_pin, AP_BATT_CURR_PIN),

    // @Param: VOLT_MULT
    // @DisplayName: Voltage Multiplier
    // @Description: Used to convert the voltage of the voltage sensing pin (@PREFIX@VOLT_PIN) to the actual battery's voltage (pin_voltage * VOLT_MULT). For the 3DR Power brick with a Pixhawk, this should be set to 10.1. For the Pixhawk with the 3DR 4in1 ESC this should be 12.02. For the PX using the PX4IO power supply this should be set to 1.
    // @User: Advanced
    AP_GROUPINFO("VOLT_MULT", 3, AP_BattMonitor_Analog, _volt_multiplier, AP_BATT_VOLTDIVIDER_DEFAULT),

    // @Param: AMP_PERVLT
    // @DisplayName: Amps per volt
    // @Description: Number of amps that a 1V reading on the current sensor corresponds to. With a Pixhawk using the 3DR Power brick this should be set to 17. For the Pixhawk with the 3DR 4in1 ESC this should be 17. For Synthetic Current sensor monitors, this is the maximum, full throttle current draw.
    // @Units: A/V
    // @User: Standard
    AP_GROUPINFO("AMP_PERVLT", 4, AP_BattMonitor_Analog, _curr_amp_per_volt, AP_BATT_CURR_AMP_PERVOLT_DEFAULT),

    // @Param: AMP_OFFSET
    // @DisplayName: AMP offset
    // @Description: Voltage offset at zero current on current sensor for Analog Sensors. For Synthetic Current sensor, this offset is the zero throttle system current and is added to the calculated throttle base current.
    // @Units: V
    // @User: Standard
    AP_GROUPINFO("AMP_OFFSET", 5, AP_BattMonitor_Analog, _curr_amp_offset, AP_BATT_CURR_AMP_OFFSET_DEFAULT),

    // @Param: VLT_OFFSET
    // @DisplayName: Voltage offset
    // @Description: Voltage offset on voltage pin. This allows for an offset due to a diode. This voltage is subtracted before the scaling is applied.
    // @Units: V
    // @User: Advanced
    AP_GROUPINFO("VLT_OFFSET", 6, AP_BattMonitor_Analog, _volt_offset, 0),
    
    // CHECK/UPDATE INDEX TABLE IN AP_BattMonitor_Backend.cpp WHEN CHANGING OR ADDING PARAMETERS

    AP_GROUPEND
};

/// Constructor
AP_BattMonitor_Analog::AP_BattMonitor_Analog(AP_BattMonitor &mon,
                                             AP_BattMonitor::BattMonitor_State &mon_state,
                                             AP_BattMonitor_Params &params) :
    AP_BattMonitor_Backend(mon, mon_state, params)
{
    AP_Param::setup_object_defaults(this, var_info);

    // no other good way of setting these defaults
#if AP_BATT_MONITOR_MAX_INSTANCES > 1
    if (mon_state.instance == 1) {
#ifdef HAL_BATT2_VOLT_PIN
        _volt_pin.set_default(HAL_BATT2_VOLT_PIN);
#endif
#ifdef HAL_BATT2_CURR_PIN
        _curr_pin.set_default(HAL_BATT2_CURR_PIN);
#endif
#ifdef HAL_BATT2_VOLT_SCALE
        _volt_multiplier.set_default(HAL_BATT2_VOLT_SCALE);
#endif
#ifdef HAL_BATT2_CURR_SCALE
        _curr_amp_per_volt.set_default(HAL_BATT2_CURR_SCALE);
#endif
    }
#endif
    _state.var_info = var_info;
    
    if (_params._type != AP_BattMonitor::Type::ANALOG_CURRENT_ONLY) {
        _volt_pin_analog_source = hal.analogin->channel(_volt_pin);
        if (_volt_pin_analog_source == nullptr) {
            AP_BoardConfig::config_error("No analog voltage channel for battery %d", mon_state.instance);
        }
    }
    if (_params._type == AP_BattMonitor::Type::ANALOG_VOLTAGE_AND_CURRENT ||
        _params._type == AP_BattMonitor::Type::ANALOG_CURRENT_ONLY) {
        _curr_pin_analog_source = hal.analogin->channel(_curr_pin);
        if (_curr_pin_analog_source == nullptr) {
            AP_BoardConfig::config_error("No analog current channel for battery %d", mon_state.instance);
        }
    }

}

// read - read the voltage and current
void
AP_BattMonitor_Analog::read()
{
    if (_state.type != AP_BattMonitor::Type::ANALOG_CURRENT_ONLY) {
        // this copes with changing the pin at runtime
        _state.healthy = _volt_pin_analog_source->set_pin(_volt_pin);

        // get voltage
        _state.voltage = (_volt_pin_analog_source->voltage_average() - _volt_offset) * _volt_multiplier;
#if defined(HAL_EFT_CAAC_BATT_VOLT_CALIB) && HAL_EFT_CAAC_BATT_VOLT_CALIB
        _state.voltage = eft_caac_correct_battery_voltage(_state.voltage);
#endif
    } else {
        _state.healthy = 1;
        _state.voltage = 0.0f;
    }

    // read current
    if (has_current()) {
        // calculate time since last current read
        const uint32_t tnow = AP_HAL::micros();
        const uint32_t dt_us = tnow - _state.last_time_micros;

        // this copes with changing the pin at runtime
        _state.healthy &= _curr_pin_analog_source->set_pin(_curr_pin);

        // read current
        _state.current_amps = (_curr_pin_analog_source->voltage_average() - _curr_amp_offset) * _curr_amp_per_volt;

        update_consumed(_state, dt_us);

        // record time
        _state.last_time_micros = tnow;
    }
}

/// return true if battery provides current info
bool AP_BattMonitor_Analog::has_current() const
{
    return (_curr_pin_analog_source != nullptr) &&
        (_state.type == AP_BattMonitor::Type::ANALOG_VOLTAGE_AND_CURRENT ||
         _state.type == AP_BattMonitor::Type::ANALOG_CURRENT_ONLY);
}

#endif  // AP_BATTERY_ANALOG_ENABLED
