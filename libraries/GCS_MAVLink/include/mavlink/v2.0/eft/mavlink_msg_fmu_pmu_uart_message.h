#pragma once
// MESSAGE FMU_PMU_UART_MESSAGE PACKING

#define MAVLINK_MSG_ID_FMU_PMU_UART_MESSAGE 515


typedef struct __mavlink_fmu_pmu_uart_message_t {
 uint32_t nozzle_control; /*<  Nozzle control bitmask. bit0 for nozzle 1, bit1 for nozzle 2, etc. (0: OFF, 1: ON)*/
 uint16_t pump_control; /*<  Pump control bitmask. bit0 for pump 1, bit1 for pump 2. (0: OFF, 1: ON)*/
 uint16_t horizontal_speed; /*< [cm/s] Horizontal speed calculated from sqrt(vel.x^2 + vel.y^2).*/
 uint16_t spray_rate; /*< [mL/mu] Spray rate, e.g., 3000 for 3000mL/mu (Chinese acre).*/
 uint16_t spray_width; /*< [cm] Spray width, e.g., 600 for 600cm.*/
 uint16_t calibration_weight; /*<  Calibration weight value.*/
 uint16_t k_values[3]; /*<  Array of 3 K-values.*/
 uint16_t spreader_motor_pwm; /*<  Spreader motor PWM value (default 1000).*/
 uint16_t spreader_valve_pwm; /*<  Spreader valve PWM value (default 1000).*/
 uint8_t control_mode; /*<  Control mode. Fixed value 0x00 for PWM mode.*/
 uint8_t pump_calibration_cmd; /*<  Pump calibration command. 0x11 for pump 1, 0x31 for pump 2, 0x00 otherwise.*/
 uint8_t led_control_cmd; /*<  LED control command identifier (e.g., 0xE1).*/
 uint8_t led_brightness_right; /*< [%] Right front LED brightness (0-100).*/
 uint8_t led_brightness_left; /*< [%] Left front LED brightness (0-100).*/
 uint8_t tare_calibration_cmd; /*<  Tare calibration command. 0xF6 to execute, 0x00 otherwise.*/
 uint8_t weight_calibration_cmd; /*<  Weight calibration command identifier (e.g., 0xF7).*/
 uint8_t k_value_calibration_cmd; /*<  K-value calibration command identifier (e.g., 0xFC).*/
 uint8_t spreader_control_cmd; /*<  Spreader motor control command identifier (e.g., 0xF1).*/
 uint8_t signal_source_cmd; /*<  Signal source selection command identifier (e.g., 0xF2).*/
 uint8_t signal_source; /*<  Signal source selection (0: PWM, 1: CANBUS).*/
 uint8_t alarm_config_cmd; /*<  Alarm configuration command identifier (e.g., 0xF3).*/
 uint8_t alarm_config[3]; /*<  Alarm configuration array for material shortage, motor stall, and hall sensor fault alarms.*/
 uint8_t factory_reset_cmd; /*<  Factory reset command. 0xF5 to execute, 0x00 otherwise.*/
 uint8_t spray_spreader_mode; /*<  Spray/Spreader mode switch. 0x01 to toggle, 0x00 otherwise.*/
} mavlink_fmu_pmu_uart_message_t;

#define MAVLINK_MSG_ID_FMU_PMU_UART_MESSAGE_LEN 41
#define MAVLINK_MSG_ID_FMU_PMU_UART_MESSAGE_MIN_LEN 41
#define MAVLINK_MSG_ID_515_LEN 41
#define MAVLINK_MSG_ID_515_MIN_LEN 41

#define MAVLINK_MSG_ID_FMU_PMU_UART_MESSAGE_CRC 66
#define MAVLINK_MSG_ID_515_CRC 66

#define MAVLINK_MSG_FMU_PMU_UART_MESSAGE_FIELD_K_VALUES_LEN 3
#define MAVLINK_MSG_FMU_PMU_UART_MESSAGE_FIELD_ALARM_CONFIG_LEN 3

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_FMU_PMU_UART_MESSAGE { \
    515, \
    "FMU_PMU_UART_MESSAGE", \
    24, \
    {  { "pump_control", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_fmu_pmu_uart_message_t, pump_control) }, \
         { "nozzle_control", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_fmu_pmu_uart_message_t, nozzle_control) }, \
         { "control_mode", NULL, MAVLINK_TYPE_UINT8_T, 0, 24, offsetof(mavlink_fmu_pmu_uart_message_t, control_mode) }, \
         { "horizontal_speed", NULL, MAVLINK_TYPE_UINT16_T, 0, 6, offsetof(mavlink_fmu_pmu_uart_message_t, horizontal_speed) }, \
         { "spray_rate", NULL, MAVLINK_TYPE_UINT16_T, 0, 8, offsetof(mavlink_fmu_pmu_uart_message_t, spray_rate) }, \
         { "spray_width", NULL, MAVLINK_TYPE_UINT16_T, 0, 10, offsetof(mavlink_fmu_pmu_uart_message_t, spray_width) }, \
         { "pump_calibration_cmd", NULL, MAVLINK_TYPE_UINT8_T, 0, 25, offsetof(mavlink_fmu_pmu_uart_message_t, pump_calibration_cmd) }, \
         { "led_control_cmd", NULL, MAVLINK_TYPE_UINT8_T, 0, 26, offsetof(mavlink_fmu_pmu_uart_message_t, led_control_cmd) }, \
         { "led_brightness_right", NULL, MAVLINK_TYPE_UINT8_T, 0, 27, offsetof(mavlink_fmu_pmu_uart_message_t, led_brightness_right) }, \
         { "led_brightness_left", NULL, MAVLINK_TYPE_UINT8_T, 0, 28, offsetof(mavlink_fmu_pmu_uart_message_t, led_brightness_left) }, \
         { "tare_calibration_cmd", NULL, MAVLINK_TYPE_UINT8_T, 0, 29, offsetof(mavlink_fmu_pmu_uart_message_t, tare_calibration_cmd) }, \
         { "weight_calibration_cmd", NULL, MAVLINK_TYPE_UINT8_T, 0, 30, offsetof(mavlink_fmu_pmu_uart_message_t, weight_calibration_cmd) }, \
         { "calibration_weight", NULL, MAVLINK_TYPE_UINT16_T, 0, 12, offsetof(mavlink_fmu_pmu_uart_message_t, calibration_weight) }, \
         { "k_value_calibration_cmd", NULL, MAVLINK_TYPE_UINT8_T, 0, 31, offsetof(mavlink_fmu_pmu_uart_message_t, k_value_calibration_cmd) }, \
         { "k_values", NULL, MAVLINK_TYPE_UINT16_T, 3, 14, offsetof(mavlink_fmu_pmu_uart_message_t, k_values) }, \
         { "spreader_control_cmd", NULL, MAVLINK_TYPE_UINT8_T, 0, 32, offsetof(mavlink_fmu_pmu_uart_message_t, spreader_control_cmd) }, \
         { "spreader_motor_pwm", NULL, MAVLINK_TYPE_UINT16_T, 0, 20, offsetof(mavlink_fmu_pmu_uart_message_t, spreader_motor_pwm) }, \
         { "spreader_valve_pwm", NULL, MAVLINK_TYPE_UINT16_T, 0, 22, offsetof(mavlink_fmu_pmu_uart_message_t, spreader_valve_pwm) }, \
         { "signal_source_cmd", NULL, MAVLINK_TYPE_UINT8_T, 0, 33, offsetof(mavlink_fmu_pmu_uart_message_t, signal_source_cmd) }, \
         { "signal_source", NULL, MAVLINK_TYPE_UINT8_T, 0, 34, offsetof(mavlink_fmu_pmu_uart_message_t, signal_source) }, \
         { "alarm_config_cmd", NULL, MAVLINK_TYPE_UINT8_T, 0, 35, offsetof(mavlink_fmu_pmu_uart_message_t, alarm_config_cmd) }, \
         { "alarm_config", NULL, MAVLINK_TYPE_UINT8_T, 3, 36, offsetof(mavlink_fmu_pmu_uart_message_t, alarm_config) }, \
         { "factory_reset_cmd", NULL, MAVLINK_TYPE_UINT8_T, 0, 39, offsetof(mavlink_fmu_pmu_uart_message_t, factory_reset_cmd) }, \
         { "spray_spreader_mode", NULL, MAVLINK_TYPE_UINT8_T, 0, 40, offsetof(mavlink_fmu_pmu_uart_message_t, spray_spreader_mode) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_FMU_PMU_UART_MESSAGE { \
    "FMU_PMU_UART_MESSAGE", \
    24, \
    {  { "pump_control", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_fmu_pmu_uart_message_t, pump_control) }, \
         { "nozzle_control", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_fmu_pmu_uart_message_t, nozzle_control) }, \
         { "control_mode", NULL, MAVLINK_TYPE_UINT8_T, 0, 24, offsetof(mavlink_fmu_pmu_uart_message_t, control_mode) }, \
         { "horizontal_speed", NULL, MAVLINK_TYPE_UINT16_T, 0, 6, offsetof(mavlink_fmu_pmu_uart_message_t, horizontal_speed) }, \
         { "spray_rate", NULL, MAVLINK_TYPE_UINT16_T, 0, 8, offsetof(mavlink_fmu_pmu_uart_message_t, spray_rate) }, \
         { "spray_width", NULL, MAVLINK_TYPE_UINT16_T, 0, 10, offsetof(mavlink_fmu_pmu_uart_message_t, spray_width) }, \
         { "pump_calibration_cmd", NULL, MAVLINK_TYPE_UINT8_T, 0, 25, offsetof(mavlink_fmu_pmu_uart_message_t, pump_calibration_cmd) }, \
         { "led_control_cmd", NULL, MAVLINK_TYPE_UINT8_T, 0, 26, offsetof(mavlink_fmu_pmu_uart_message_t, led_control_cmd) }, \
         { "led_brightness_right", NULL, MAVLINK_TYPE_UINT8_T, 0, 27, offsetof(mavlink_fmu_pmu_uart_message_t, led_brightness_right) }, \
         { "led_brightness_left", NULL, MAVLINK_TYPE_UINT8_T, 0, 28, offsetof(mavlink_fmu_pmu_uart_message_t, led_brightness_left) }, \
         { "tare_calibration_cmd", NULL, MAVLINK_TYPE_UINT8_T, 0, 29, offsetof(mavlink_fmu_pmu_uart_message_t, tare_calibration_cmd) }, \
         { "weight_calibration_cmd", NULL, MAVLINK_TYPE_UINT8_T, 0, 30, offsetof(mavlink_fmu_pmu_uart_message_t, weight_calibration_cmd) }, \
         { "calibration_weight", NULL, MAVLINK_TYPE_UINT16_T, 0, 12, offsetof(mavlink_fmu_pmu_uart_message_t, calibration_weight) }, \
         { "k_value_calibration_cmd", NULL, MAVLINK_TYPE_UINT8_T, 0, 31, offsetof(mavlink_fmu_pmu_uart_message_t, k_value_calibration_cmd) }, \
         { "k_values", NULL, MAVLINK_TYPE_UINT16_T, 3, 14, offsetof(mavlink_fmu_pmu_uart_message_t, k_values) }, \
         { "spreader_control_cmd", NULL, MAVLINK_TYPE_UINT8_T, 0, 32, offsetof(mavlink_fmu_pmu_uart_message_t, spreader_control_cmd) }, \
         { "spreader_motor_pwm", NULL, MAVLINK_TYPE_UINT16_T, 0, 20, offsetof(mavlink_fmu_pmu_uart_message_t, spreader_motor_pwm) }, \
         { "spreader_valve_pwm", NULL, MAVLINK_TYPE_UINT16_T, 0, 22, offsetof(mavlink_fmu_pmu_uart_message_t, spreader_valve_pwm) }, \
         { "signal_source_cmd", NULL, MAVLINK_TYPE_UINT8_T, 0, 33, offsetof(mavlink_fmu_pmu_uart_message_t, signal_source_cmd) }, \
         { "signal_source", NULL, MAVLINK_TYPE_UINT8_T, 0, 34, offsetof(mavlink_fmu_pmu_uart_message_t, signal_source) }, \
         { "alarm_config_cmd", NULL, MAVLINK_TYPE_UINT8_T, 0, 35, offsetof(mavlink_fmu_pmu_uart_message_t, alarm_config_cmd) }, \
         { "alarm_config", NULL, MAVLINK_TYPE_UINT8_T, 3, 36, offsetof(mavlink_fmu_pmu_uart_message_t, alarm_config) }, \
         { "factory_reset_cmd", NULL, MAVLINK_TYPE_UINT8_T, 0, 39, offsetof(mavlink_fmu_pmu_uart_message_t, factory_reset_cmd) }, \
         { "spray_spreader_mode", NULL, MAVLINK_TYPE_UINT8_T, 0, 40, offsetof(mavlink_fmu_pmu_uart_message_t, spray_spreader_mode) }, \
         } \
}
#endif

/**
 * @brief Pack a fmu_pmu_uart_message message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param pump_control  Pump control bitmask. bit0 for pump 1, bit1 for pump 2. (0: OFF, 1: ON)
 * @param nozzle_control  Nozzle control bitmask. bit0 for nozzle 1, bit1 for nozzle 2, etc. (0: OFF, 1: ON)
 * @param control_mode  Control mode. Fixed value 0x00 for PWM mode.
 * @param horizontal_speed [cm/s] Horizontal speed calculated from sqrt(vel.x^2 + vel.y^2).
 * @param spray_rate [mL/mu] Spray rate, e.g., 3000 for 3000mL/mu (Chinese acre).
 * @param spray_width [cm] Spray width, e.g., 600 for 600cm.
 * @param pump_calibration_cmd  Pump calibration command. 0x11 for pump 1, 0x31 for pump 2, 0x00 otherwise.
 * @param led_control_cmd  LED control command identifier (e.g., 0xE1).
 * @param led_brightness_right [%] Right front LED brightness (0-100).
 * @param led_brightness_left [%] Left front LED brightness (0-100).
 * @param tare_calibration_cmd  Tare calibration command. 0xF6 to execute, 0x00 otherwise.
 * @param weight_calibration_cmd  Weight calibration command identifier (e.g., 0xF7).
 * @param calibration_weight  Calibration weight value.
 * @param k_value_calibration_cmd  K-value calibration command identifier (e.g., 0xFC).
 * @param k_values  Array of 3 K-values.
 * @param spreader_control_cmd  Spreader motor control command identifier (e.g., 0xF1).
 * @param spreader_motor_pwm  Spreader motor PWM value (default 1000).
 * @param spreader_valve_pwm  Spreader valve PWM value (default 1000).
 * @param signal_source_cmd  Signal source selection command identifier (e.g., 0xF2).
 * @param signal_source  Signal source selection (0: PWM, 1: CANBUS).
 * @param alarm_config_cmd  Alarm configuration command identifier (e.g., 0xF3).
 * @param alarm_config  Alarm configuration array for material shortage, motor stall, and hall sensor fault alarms.
 * @param factory_reset_cmd  Factory reset command. 0xF5 to execute, 0x00 otherwise.
 * @param spray_spreader_mode  Spray/Spreader mode switch. 0x01 to toggle, 0x00 otherwise.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_fmu_pmu_uart_message_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint16_t pump_control, uint32_t nozzle_control, uint8_t control_mode, uint16_t horizontal_speed, uint16_t spray_rate, uint16_t spray_width, uint8_t pump_calibration_cmd, uint8_t led_control_cmd, uint8_t led_brightness_right, uint8_t led_brightness_left, uint8_t tare_calibration_cmd, uint8_t weight_calibration_cmd, uint16_t calibration_weight, uint8_t k_value_calibration_cmd, const uint16_t *k_values, uint8_t spreader_control_cmd, uint16_t spreader_motor_pwm, uint16_t spreader_valve_pwm, uint8_t signal_source_cmd, uint8_t signal_source, uint8_t alarm_config_cmd, const uint8_t *alarm_config, uint8_t factory_reset_cmd, uint8_t spray_spreader_mode)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_FMU_PMU_UART_MESSAGE_LEN];
    _mav_put_uint32_t(buf, 0, nozzle_control);
    _mav_put_uint16_t(buf, 4, pump_control);
    _mav_put_uint16_t(buf, 6, horizontal_speed);
    _mav_put_uint16_t(buf, 8, spray_rate);
    _mav_put_uint16_t(buf, 10, spray_width);
    _mav_put_uint16_t(buf, 12, calibration_weight);
    _mav_put_uint16_t(buf, 20, spreader_motor_pwm);
    _mav_put_uint16_t(buf, 22, spreader_valve_pwm);
    _mav_put_uint8_t(buf, 24, control_mode);
    _mav_put_uint8_t(buf, 25, pump_calibration_cmd);
    _mav_put_uint8_t(buf, 26, led_control_cmd);
    _mav_put_uint8_t(buf, 27, led_brightness_right);
    _mav_put_uint8_t(buf, 28, led_brightness_left);
    _mav_put_uint8_t(buf, 29, tare_calibration_cmd);
    _mav_put_uint8_t(buf, 30, weight_calibration_cmd);
    _mav_put_uint8_t(buf, 31, k_value_calibration_cmd);
    _mav_put_uint8_t(buf, 32, spreader_control_cmd);
    _mav_put_uint8_t(buf, 33, signal_source_cmd);
    _mav_put_uint8_t(buf, 34, signal_source);
    _mav_put_uint8_t(buf, 35, alarm_config_cmd);
    _mav_put_uint8_t(buf, 39, factory_reset_cmd);
    _mav_put_uint8_t(buf, 40, spray_spreader_mode);
    _mav_put_uint16_t_array(buf, 14, k_values, 3);
    _mav_put_uint8_t_array(buf, 36, alarm_config, 3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_FMU_PMU_UART_MESSAGE_LEN);
#else
    mavlink_fmu_pmu_uart_message_t packet;
    packet.nozzle_control = nozzle_control;
    packet.pump_control = pump_control;
    packet.horizontal_speed = horizontal_speed;
    packet.spray_rate = spray_rate;
    packet.spray_width = spray_width;
    packet.calibration_weight = calibration_weight;
    packet.spreader_motor_pwm = spreader_motor_pwm;
    packet.spreader_valve_pwm = spreader_valve_pwm;
    packet.control_mode = control_mode;
    packet.pump_calibration_cmd = pump_calibration_cmd;
    packet.led_control_cmd = led_control_cmd;
    packet.led_brightness_right = led_brightness_right;
    packet.led_brightness_left = led_brightness_left;
    packet.tare_calibration_cmd = tare_calibration_cmd;
    packet.weight_calibration_cmd = weight_calibration_cmd;
    packet.k_value_calibration_cmd = k_value_calibration_cmd;
    packet.spreader_control_cmd = spreader_control_cmd;
    packet.signal_source_cmd = signal_source_cmd;
    packet.signal_source = signal_source;
    packet.alarm_config_cmd = alarm_config_cmd;
    packet.factory_reset_cmd = factory_reset_cmd;
    packet.spray_spreader_mode = spray_spreader_mode;
    mav_array_assign_uint16_t(packet.k_values, k_values, 3);
    mav_array_assign_uint8_t(packet.alarm_config, alarm_config, 3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_FMU_PMU_UART_MESSAGE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_FMU_PMU_UART_MESSAGE;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_FMU_PMU_UART_MESSAGE_MIN_LEN, MAVLINK_MSG_ID_FMU_PMU_UART_MESSAGE_LEN, MAVLINK_MSG_ID_FMU_PMU_UART_MESSAGE_CRC);
}

/**
 * @brief Pack a fmu_pmu_uart_message message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param pump_control  Pump control bitmask. bit0 for pump 1, bit1 for pump 2. (0: OFF, 1: ON)
 * @param nozzle_control  Nozzle control bitmask. bit0 for nozzle 1, bit1 for nozzle 2, etc. (0: OFF, 1: ON)
 * @param control_mode  Control mode. Fixed value 0x00 for PWM mode.
 * @param horizontal_speed [cm/s] Horizontal speed calculated from sqrt(vel.x^2 + vel.y^2).
 * @param spray_rate [mL/mu] Spray rate, e.g., 3000 for 3000mL/mu (Chinese acre).
 * @param spray_width [cm] Spray width, e.g., 600 for 600cm.
 * @param pump_calibration_cmd  Pump calibration command. 0x11 for pump 1, 0x31 for pump 2, 0x00 otherwise.
 * @param led_control_cmd  LED control command identifier (e.g., 0xE1).
 * @param led_brightness_right [%] Right front LED brightness (0-100).
 * @param led_brightness_left [%] Left front LED brightness (0-100).
 * @param tare_calibration_cmd  Tare calibration command. 0xF6 to execute, 0x00 otherwise.
 * @param weight_calibration_cmd  Weight calibration command identifier (e.g., 0xF7).
 * @param calibration_weight  Calibration weight value.
 * @param k_value_calibration_cmd  K-value calibration command identifier (e.g., 0xFC).
 * @param k_values  Array of 3 K-values.
 * @param spreader_control_cmd  Spreader motor control command identifier (e.g., 0xF1).
 * @param spreader_motor_pwm  Spreader motor PWM value (default 1000).
 * @param spreader_valve_pwm  Spreader valve PWM value (default 1000).
 * @param signal_source_cmd  Signal source selection command identifier (e.g., 0xF2).
 * @param signal_source  Signal source selection (0: PWM, 1: CANBUS).
 * @param alarm_config_cmd  Alarm configuration command identifier (e.g., 0xF3).
 * @param alarm_config  Alarm configuration array for material shortage, motor stall, and hall sensor fault alarms.
 * @param factory_reset_cmd  Factory reset command. 0xF5 to execute, 0x00 otherwise.
 * @param spray_spreader_mode  Spray/Spreader mode switch. 0x01 to toggle, 0x00 otherwise.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_fmu_pmu_uart_message_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint16_t pump_control, uint32_t nozzle_control, uint8_t control_mode, uint16_t horizontal_speed, uint16_t spray_rate, uint16_t spray_width, uint8_t pump_calibration_cmd, uint8_t led_control_cmd, uint8_t led_brightness_right, uint8_t led_brightness_left, uint8_t tare_calibration_cmd, uint8_t weight_calibration_cmd, uint16_t calibration_weight, uint8_t k_value_calibration_cmd, const uint16_t *k_values, uint8_t spreader_control_cmd, uint16_t spreader_motor_pwm, uint16_t spreader_valve_pwm, uint8_t signal_source_cmd, uint8_t signal_source, uint8_t alarm_config_cmd, const uint8_t *alarm_config, uint8_t factory_reset_cmd, uint8_t spray_spreader_mode)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_FMU_PMU_UART_MESSAGE_LEN];
    _mav_put_uint32_t(buf, 0, nozzle_control);
    _mav_put_uint16_t(buf, 4, pump_control);
    _mav_put_uint16_t(buf, 6, horizontal_speed);
    _mav_put_uint16_t(buf, 8, spray_rate);
    _mav_put_uint16_t(buf, 10, spray_width);
    _mav_put_uint16_t(buf, 12, calibration_weight);
    _mav_put_uint16_t(buf, 20, spreader_motor_pwm);
    _mav_put_uint16_t(buf, 22, spreader_valve_pwm);
    _mav_put_uint8_t(buf, 24, control_mode);
    _mav_put_uint8_t(buf, 25, pump_calibration_cmd);
    _mav_put_uint8_t(buf, 26, led_control_cmd);
    _mav_put_uint8_t(buf, 27, led_brightness_right);
    _mav_put_uint8_t(buf, 28, led_brightness_left);
    _mav_put_uint8_t(buf, 29, tare_calibration_cmd);
    _mav_put_uint8_t(buf, 30, weight_calibration_cmd);
    _mav_put_uint8_t(buf, 31, k_value_calibration_cmd);
    _mav_put_uint8_t(buf, 32, spreader_control_cmd);
    _mav_put_uint8_t(buf, 33, signal_source_cmd);
    _mav_put_uint8_t(buf, 34, signal_source);
    _mav_put_uint8_t(buf, 35, alarm_config_cmd);
    _mav_put_uint8_t(buf, 39, factory_reset_cmd);
    _mav_put_uint8_t(buf, 40, spray_spreader_mode);
    _mav_put_uint16_t_array(buf, 14, k_values, 3);
    _mav_put_uint8_t_array(buf, 36, alarm_config, 3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_FMU_PMU_UART_MESSAGE_LEN);
#else
    mavlink_fmu_pmu_uart_message_t packet;
    packet.nozzle_control = nozzle_control;
    packet.pump_control = pump_control;
    packet.horizontal_speed = horizontal_speed;
    packet.spray_rate = spray_rate;
    packet.spray_width = spray_width;
    packet.calibration_weight = calibration_weight;
    packet.spreader_motor_pwm = spreader_motor_pwm;
    packet.spreader_valve_pwm = spreader_valve_pwm;
    packet.control_mode = control_mode;
    packet.pump_calibration_cmd = pump_calibration_cmd;
    packet.led_control_cmd = led_control_cmd;
    packet.led_brightness_right = led_brightness_right;
    packet.led_brightness_left = led_brightness_left;
    packet.tare_calibration_cmd = tare_calibration_cmd;
    packet.weight_calibration_cmd = weight_calibration_cmd;
    packet.k_value_calibration_cmd = k_value_calibration_cmd;
    packet.spreader_control_cmd = spreader_control_cmd;
    packet.signal_source_cmd = signal_source_cmd;
    packet.signal_source = signal_source;
    packet.alarm_config_cmd = alarm_config_cmd;
    packet.factory_reset_cmd = factory_reset_cmd;
    packet.spray_spreader_mode = spray_spreader_mode;
    mav_array_memcpy(packet.k_values, k_values, sizeof(uint16_t)*3);
    mav_array_memcpy(packet.alarm_config, alarm_config, sizeof(uint8_t)*3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_FMU_PMU_UART_MESSAGE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_FMU_PMU_UART_MESSAGE;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_FMU_PMU_UART_MESSAGE_MIN_LEN, MAVLINK_MSG_ID_FMU_PMU_UART_MESSAGE_LEN, MAVLINK_MSG_ID_FMU_PMU_UART_MESSAGE_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_FMU_PMU_UART_MESSAGE_MIN_LEN, MAVLINK_MSG_ID_FMU_PMU_UART_MESSAGE_LEN);
#endif
}

/**
 * @brief Pack a fmu_pmu_uart_message message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param pump_control  Pump control bitmask. bit0 for pump 1, bit1 for pump 2. (0: OFF, 1: ON)
 * @param nozzle_control  Nozzle control bitmask. bit0 for nozzle 1, bit1 for nozzle 2, etc. (0: OFF, 1: ON)
 * @param control_mode  Control mode. Fixed value 0x00 for PWM mode.
 * @param horizontal_speed [cm/s] Horizontal speed calculated from sqrt(vel.x^2 + vel.y^2).
 * @param spray_rate [mL/mu] Spray rate, e.g., 3000 for 3000mL/mu (Chinese acre).
 * @param spray_width [cm] Spray width, e.g., 600 for 600cm.
 * @param pump_calibration_cmd  Pump calibration command. 0x11 for pump 1, 0x31 for pump 2, 0x00 otherwise.
 * @param led_control_cmd  LED control command identifier (e.g., 0xE1).
 * @param led_brightness_right [%] Right front LED brightness (0-100).
 * @param led_brightness_left [%] Left front LED brightness (0-100).
 * @param tare_calibration_cmd  Tare calibration command. 0xF6 to execute, 0x00 otherwise.
 * @param weight_calibration_cmd  Weight calibration command identifier (e.g., 0xF7).
 * @param calibration_weight  Calibration weight value.
 * @param k_value_calibration_cmd  K-value calibration command identifier (e.g., 0xFC).
 * @param k_values  Array of 3 K-values.
 * @param spreader_control_cmd  Spreader motor control command identifier (e.g., 0xF1).
 * @param spreader_motor_pwm  Spreader motor PWM value (default 1000).
 * @param spreader_valve_pwm  Spreader valve PWM value (default 1000).
 * @param signal_source_cmd  Signal source selection command identifier (e.g., 0xF2).
 * @param signal_source  Signal source selection (0: PWM, 1: CANBUS).
 * @param alarm_config_cmd  Alarm configuration command identifier (e.g., 0xF3).
 * @param alarm_config  Alarm configuration array for material shortage, motor stall, and hall sensor fault alarms.
 * @param factory_reset_cmd  Factory reset command. 0xF5 to execute, 0x00 otherwise.
 * @param spray_spreader_mode  Spray/Spreader mode switch. 0x01 to toggle, 0x00 otherwise.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_fmu_pmu_uart_message_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint16_t pump_control,uint32_t nozzle_control,uint8_t control_mode,uint16_t horizontal_speed,uint16_t spray_rate,uint16_t spray_width,uint8_t pump_calibration_cmd,uint8_t led_control_cmd,uint8_t led_brightness_right,uint8_t led_brightness_left,uint8_t tare_calibration_cmd,uint8_t weight_calibration_cmd,uint16_t calibration_weight,uint8_t k_value_calibration_cmd,const uint16_t *k_values,uint8_t spreader_control_cmd,uint16_t spreader_motor_pwm,uint16_t spreader_valve_pwm,uint8_t signal_source_cmd,uint8_t signal_source,uint8_t alarm_config_cmd,const uint8_t *alarm_config,uint8_t factory_reset_cmd,uint8_t spray_spreader_mode)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_FMU_PMU_UART_MESSAGE_LEN];
    _mav_put_uint32_t(buf, 0, nozzle_control);
    _mav_put_uint16_t(buf, 4, pump_control);
    _mav_put_uint16_t(buf, 6, horizontal_speed);
    _mav_put_uint16_t(buf, 8, spray_rate);
    _mav_put_uint16_t(buf, 10, spray_width);
    _mav_put_uint16_t(buf, 12, calibration_weight);
    _mav_put_uint16_t(buf, 20, spreader_motor_pwm);
    _mav_put_uint16_t(buf, 22, spreader_valve_pwm);
    _mav_put_uint8_t(buf, 24, control_mode);
    _mav_put_uint8_t(buf, 25, pump_calibration_cmd);
    _mav_put_uint8_t(buf, 26, led_control_cmd);
    _mav_put_uint8_t(buf, 27, led_brightness_right);
    _mav_put_uint8_t(buf, 28, led_brightness_left);
    _mav_put_uint8_t(buf, 29, tare_calibration_cmd);
    _mav_put_uint8_t(buf, 30, weight_calibration_cmd);
    _mav_put_uint8_t(buf, 31, k_value_calibration_cmd);
    _mav_put_uint8_t(buf, 32, spreader_control_cmd);
    _mav_put_uint8_t(buf, 33, signal_source_cmd);
    _mav_put_uint8_t(buf, 34, signal_source);
    _mav_put_uint8_t(buf, 35, alarm_config_cmd);
    _mav_put_uint8_t(buf, 39, factory_reset_cmd);
    _mav_put_uint8_t(buf, 40, spray_spreader_mode);
    _mav_put_uint16_t_array(buf, 14, k_values, 3);
    _mav_put_uint8_t_array(buf, 36, alarm_config, 3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_FMU_PMU_UART_MESSAGE_LEN);
#else
    mavlink_fmu_pmu_uart_message_t packet;
    packet.nozzle_control = nozzle_control;
    packet.pump_control = pump_control;
    packet.horizontal_speed = horizontal_speed;
    packet.spray_rate = spray_rate;
    packet.spray_width = spray_width;
    packet.calibration_weight = calibration_weight;
    packet.spreader_motor_pwm = spreader_motor_pwm;
    packet.spreader_valve_pwm = spreader_valve_pwm;
    packet.control_mode = control_mode;
    packet.pump_calibration_cmd = pump_calibration_cmd;
    packet.led_control_cmd = led_control_cmd;
    packet.led_brightness_right = led_brightness_right;
    packet.led_brightness_left = led_brightness_left;
    packet.tare_calibration_cmd = tare_calibration_cmd;
    packet.weight_calibration_cmd = weight_calibration_cmd;
    packet.k_value_calibration_cmd = k_value_calibration_cmd;
    packet.spreader_control_cmd = spreader_control_cmd;
    packet.signal_source_cmd = signal_source_cmd;
    packet.signal_source = signal_source;
    packet.alarm_config_cmd = alarm_config_cmd;
    packet.factory_reset_cmd = factory_reset_cmd;
    packet.spray_spreader_mode = spray_spreader_mode;
    mav_array_assign_uint16_t(packet.k_values, k_values, 3);
    mav_array_assign_uint8_t(packet.alarm_config, alarm_config, 3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_FMU_PMU_UART_MESSAGE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_FMU_PMU_UART_MESSAGE;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_FMU_PMU_UART_MESSAGE_MIN_LEN, MAVLINK_MSG_ID_FMU_PMU_UART_MESSAGE_LEN, MAVLINK_MSG_ID_FMU_PMU_UART_MESSAGE_CRC);
}

/**
 * @brief Encode a fmu_pmu_uart_message struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param fmu_pmu_uart_message C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_fmu_pmu_uart_message_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_fmu_pmu_uart_message_t* fmu_pmu_uart_message)
{
    return mavlink_msg_fmu_pmu_uart_message_pack(system_id, component_id, msg, fmu_pmu_uart_message->pump_control, fmu_pmu_uart_message->nozzle_control, fmu_pmu_uart_message->control_mode, fmu_pmu_uart_message->horizontal_speed, fmu_pmu_uart_message->spray_rate, fmu_pmu_uart_message->spray_width, fmu_pmu_uart_message->pump_calibration_cmd, fmu_pmu_uart_message->led_control_cmd, fmu_pmu_uart_message->led_brightness_right, fmu_pmu_uart_message->led_brightness_left, fmu_pmu_uart_message->tare_calibration_cmd, fmu_pmu_uart_message->weight_calibration_cmd, fmu_pmu_uart_message->calibration_weight, fmu_pmu_uart_message->k_value_calibration_cmd, fmu_pmu_uart_message->k_values, fmu_pmu_uart_message->spreader_control_cmd, fmu_pmu_uart_message->spreader_motor_pwm, fmu_pmu_uart_message->spreader_valve_pwm, fmu_pmu_uart_message->signal_source_cmd, fmu_pmu_uart_message->signal_source, fmu_pmu_uart_message->alarm_config_cmd, fmu_pmu_uart_message->alarm_config, fmu_pmu_uart_message->factory_reset_cmd, fmu_pmu_uart_message->spray_spreader_mode);
}

/**
 * @brief Encode a fmu_pmu_uart_message struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param fmu_pmu_uart_message C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_fmu_pmu_uart_message_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_fmu_pmu_uart_message_t* fmu_pmu_uart_message)
{
    return mavlink_msg_fmu_pmu_uart_message_pack_chan(system_id, component_id, chan, msg, fmu_pmu_uart_message->pump_control, fmu_pmu_uart_message->nozzle_control, fmu_pmu_uart_message->control_mode, fmu_pmu_uart_message->horizontal_speed, fmu_pmu_uart_message->spray_rate, fmu_pmu_uart_message->spray_width, fmu_pmu_uart_message->pump_calibration_cmd, fmu_pmu_uart_message->led_control_cmd, fmu_pmu_uart_message->led_brightness_right, fmu_pmu_uart_message->led_brightness_left, fmu_pmu_uart_message->tare_calibration_cmd, fmu_pmu_uart_message->weight_calibration_cmd, fmu_pmu_uart_message->calibration_weight, fmu_pmu_uart_message->k_value_calibration_cmd, fmu_pmu_uart_message->k_values, fmu_pmu_uart_message->spreader_control_cmd, fmu_pmu_uart_message->spreader_motor_pwm, fmu_pmu_uart_message->spreader_valve_pwm, fmu_pmu_uart_message->signal_source_cmd, fmu_pmu_uart_message->signal_source, fmu_pmu_uart_message->alarm_config_cmd, fmu_pmu_uart_message->alarm_config, fmu_pmu_uart_message->factory_reset_cmd, fmu_pmu_uart_message->spray_spreader_mode);
}

/**
 * @brief Encode a fmu_pmu_uart_message struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param fmu_pmu_uart_message C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_fmu_pmu_uart_message_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_fmu_pmu_uart_message_t* fmu_pmu_uart_message)
{
    return mavlink_msg_fmu_pmu_uart_message_pack_status(system_id, component_id, _status, msg,  fmu_pmu_uart_message->pump_control, fmu_pmu_uart_message->nozzle_control, fmu_pmu_uart_message->control_mode, fmu_pmu_uart_message->horizontal_speed, fmu_pmu_uart_message->spray_rate, fmu_pmu_uart_message->spray_width, fmu_pmu_uart_message->pump_calibration_cmd, fmu_pmu_uart_message->led_control_cmd, fmu_pmu_uart_message->led_brightness_right, fmu_pmu_uart_message->led_brightness_left, fmu_pmu_uart_message->tare_calibration_cmd, fmu_pmu_uart_message->weight_calibration_cmd, fmu_pmu_uart_message->calibration_weight, fmu_pmu_uart_message->k_value_calibration_cmd, fmu_pmu_uart_message->k_values, fmu_pmu_uart_message->spreader_control_cmd, fmu_pmu_uart_message->spreader_motor_pwm, fmu_pmu_uart_message->spreader_valve_pwm, fmu_pmu_uart_message->signal_source_cmd, fmu_pmu_uart_message->signal_source, fmu_pmu_uart_message->alarm_config_cmd, fmu_pmu_uart_message->alarm_config, fmu_pmu_uart_message->factory_reset_cmd, fmu_pmu_uart_message->spray_spreader_mode);
}

/**
 * @brief Send a fmu_pmu_uart_message message
 * @param chan MAVLink channel to send the message
 *
 * @param pump_control  Pump control bitmask. bit0 for pump 1, bit1 for pump 2. (0: OFF, 1: ON)
 * @param nozzle_control  Nozzle control bitmask. bit0 for nozzle 1, bit1 for nozzle 2, etc. (0: OFF, 1: ON)
 * @param control_mode  Control mode. Fixed value 0x00 for PWM mode.
 * @param horizontal_speed [cm/s] Horizontal speed calculated from sqrt(vel.x^2 + vel.y^2).
 * @param spray_rate [mL/mu] Spray rate, e.g., 3000 for 3000mL/mu (Chinese acre).
 * @param spray_width [cm] Spray width, e.g., 600 for 600cm.
 * @param pump_calibration_cmd  Pump calibration command. 0x11 for pump 1, 0x31 for pump 2, 0x00 otherwise.
 * @param led_control_cmd  LED control command identifier (e.g., 0xE1).
 * @param led_brightness_right [%] Right front LED brightness (0-100).
 * @param led_brightness_left [%] Left front LED brightness (0-100).
 * @param tare_calibration_cmd  Tare calibration command. 0xF6 to execute, 0x00 otherwise.
 * @param weight_calibration_cmd  Weight calibration command identifier (e.g., 0xF7).
 * @param calibration_weight  Calibration weight value.
 * @param k_value_calibration_cmd  K-value calibration command identifier (e.g., 0xFC).
 * @param k_values  Array of 3 K-values.
 * @param spreader_control_cmd  Spreader motor control command identifier (e.g., 0xF1).
 * @param spreader_motor_pwm  Spreader motor PWM value (default 1000).
 * @param spreader_valve_pwm  Spreader valve PWM value (default 1000).
 * @param signal_source_cmd  Signal source selection command identifier (e.g., 0xF2).
 * @param signal_source  Signal source selection (0: PWM, 1: CANBUS).
 * @param alarm_config_cmd  Alarm configuration command identifier (e.g., 0xF3).
 * @param alarm_config  Alarm configuration array for material shortage, motor stall, and hall sensor fault alarms.
 * @param factory_reset_cmd  Factory reset command. 0xF5 to execute, 0x00 otherwise.
 * @param spray_spreader_mode  Spray/Spreader mode switch. 0x01 to toggle, 0x00 otherwise.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_fmu_pmu_uart_message_send(mavlink_channel_t chan, uint16_t pump_control, uint32_t nozzle_control, uint8_t control_mode, uint16_t horizontal_speed, uint16_t spray_rate, uint16_t spray_width, uint8_t pump_calibration_cmd, uint8_t led_control_cmd, uint8_t led_brightness_right, uint8_t led_brightness_left, uint8_t tare_calibration_cmd, uint8_t weight_calibration_cmd, uint16_t calibration_weight, uint8_t k_value_calibration_cmd, const uint16_t *k_values, uint8_t spreader_control_cmd, uint16_t spreader_motor_pwm, uint16_t spreader_valve_pwm, uint8_t signal_source_cmd, uint8_t signal_source, uint8_t alarm_config_cmd, const uint8_t *alarm_config, uint8_t factory_reset_cmd, uint8_t spray_spreader_mode)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_FMU_PMU_UART_MESSAGE_LEN];
    _mav_put_uint32_t(buf, 0, nozzle_control);
    _mav_put_uint16_t(buf, 4, pump_control);
    _mav_put_uint16_t(buf, 6, horizontal_speed);
    _mav_put_uint16_t(buf, 8, spray_rate);
    _mav_put_uint16_t(buf, 10, spray_width);
    _mav_put_uint16_t(buf, 12, calibration_weight);
    _mav_put_uint16_t(buf, 20, spreader_motor_pwm);
    _mav_put_uint16_t(buf, 22, spreader_valve_pwm);
    _mav_put_uint8_t(buf, 24, control_mode);
    _mav_put_uint8_t(buf, 25, pump_calibration_cmd);
    _mav_put_uint8_t(buf, 26, led_control_cmd);
    _mav_put_uint8_t(buf, 27, led_brightness_right);
    _mav_put_uint8_t(buf, 28, led_brightness_left);
    _mav_put_uint8_t(buf, 29, tare_calibration_cmd);
    _mav_put_uint8_t(buf, 30, weight_calibration_cmd);
    _mav_put_uint8_t(buf, 31, k_value_calibration_cmd);
    _mav_put_uint8_t(buf, 32, spreader_control_cmd);
    _mav_put_uint8_t(buf, 33, signal_source_cmd);
    _mav_put_uint8_t(buf, 34, signal_source);
    _mav_put_uint8_t(buf, 35, alarm_config_cmd);
    _mav_put_uint8_t(buf, 39, factory_reset_cmd);
    _mav_put_uint8_t(buf, 40, spray_spreader_mode);
    _mav_put_uint16_t_array(buf, 14, k_values, 3);
    _mav_put_uint8_t_array(buf, 36, alarm_config, 3);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FMU_PMU_UART_MESSAGE, buf, MAVLINK_MSG_ID_FMU_PMU_UART_MESSAGE_MIN_LEN, MAVLINK_MSG_ID_FMU_PMU_UART_MESSAGE_LEN, MAVLINK_MSG_ID_FMU_PMU_UART_MESSAGE_CRC);
#else
    mavlink_fmu_pmu_uart_message_t packet;
    packet.nozzle_control = nozzle_control;
    packet.pump_control = pump_control;
    packet.horizontal_speed = horizontal_speed;
    packet.spray_rate = spray_rate;
    packet.spray_width = spray_width;
    packet.calibration_weight = calibration_weight;
    packet.spreader_motor_pwm = spreader_motor_pwm;
    packet.spreader_valve_pwm = spreader_valve_pwm;
    packet.control_mode = control_mode;
    packet.pump_calibration_cmd = pump_calibration_cmd;
    packet.led_control_cmd = led_control_cmd;
    packet.led_brightness_right = led_brightness_right;
    packet.led_brightness_left = led_brightness_left;
    packet.tare_calibration_cmd = tare_calibration_cmd;
    packet.weight_calibration_cmd = weight_calibration_cmd;
    packet.k_value_calibration_cmd = k_value_calibration_cmd;
    packet.spreader_control_cmd = spreader_control_cmd;
    packet.signal_source_cmd = signal_source_cmd;
    packet.signal_source = signal_source;
    packet.alarm_config_cmd = alarm_config_cmd;
    packet.factory_reset_cmd = factory_reset_cmd;
    packet.spray_spreader_mode = spray_spreader_mode;
    mav_array_assign_uint16_t(packet.k_values, k_values, 3);
    mav_array_assign_uint8_t(packet.alarm_config, alarm_config, 3);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FMU_PMU_UART_MESSAGE, (const char *)&packet, MAVLINK_MSG_ID_FMU_PMU_UART_MESSAGE_MIN_LEN, MAVLINK_MSG_ID_FMU_PMU_UART_MESSAGE_LEN, MAVLINK_MSG_ID_FMU_PMU_UART_MESSAGE_CRC);
#endif
}

/**
 * @brief Send a fmu_pmu_uart_message message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_fmu_pmu_uart_message_send_struct(mavlink_channel_t chan, const mavlink_fmu_pmu_uart_message_t* fmu_pmu_uart_message)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_fmu_pmu_uart_message_send(chan, fmu_pmu_uart_message->pump_control, fmu_pmu_uart_message->nozzle_control, fmu_pmu_uart_message->control_mode, fmu_pmu_uart_message->horizontal_speed, fmu_pmu_uart_message->spray_rate, fmu_pmu_uart_message->spray_width, fmu_pmu_uart_message->pump_calibration_cmd, fmu_pmu_uart_message->led_control_cmd, fmu_pmu_uart_message->led_brightness_right, fmu_pmu_uart_message->led_brightness_left, fmu_pmu_uart_message->tare_calibration_cmd, fmu_pmu_uart_message->weight_calibration_cmd, fmu_pmu_uart_message->calibration_weight, fmu_pmu_uart_message->k_value_calibration_cmd, fmu_pmu_uart_message->k_values, fmu_pmu_uart_message->spreader_control_cmd, fmu_pmu_uart_message->spreader_motor_pwm, fmu_pmu_uart_message->spreader_valve_pwm, fmu_pmu_uart_message->signal_source_cmd, fmu_pmu_uart_message->signal_source, fmu_pmu_uart_message->alarm_config_cmd, fmu_pmu_uart_message->alarm_config, fmu_pmu_uart_message->factory_reset_cmd, fmu_pmu_uart_message->spray_spreader_mode);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FMU_PMU_UART_MESSAGE, (const char *)fmu_pmu_uart_message, MAVLINK_MSG_ID_FMU_PMU_UART_MESSAGE_MIN_LEN, MAVLINK_MSG_ID_FMU_PMU_UART_MESSAGE_LEN, MAVLINK_MSG_ID_FMU_PMU_UART_MESSAGE_CRC);
#endif
}

#if MAVLINK_MSG_ID_FMU_PMU_UART_MESSAGE_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by reusing
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_fmu_pmu_uart_message_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint16_t pump_control, uint32_t nozzle_control, uint8_t control_mode, uint16_t horizontal_speed, uint16_t spray_rate, uint16_t spray_width, uint8_t pump_calibration_cmd, uint8_t led_control_cmd, uint8_t led_brightness_right, uint8_t led_brightness_left, uint8_t tare_calibration_cmd, uint8_t weight_calibration_cmd, uint16_t calibration_weight, uint8_t k_value_calibration_cmd, const uint16_t *k_values, uint8_t spreader_control_cmd, uint16_t spreader_motor_pwm, uint16_t spreader_valve_pwm, uint8_t signal_source_cmd, uint8_t signal_source, uint8_t alarm_config_cmd, const uint8_t *alarm_config, uint8_t factory_reset_cmd, uint8_t spray_spreader_mode)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, nozzle_control);
    _mav_put_uint16_t(buf, 4, pump_control);
    _mav_put_uint16_t(buf, 6, horizontal_speed);
    _mav_put_uint16_t(buf, 8, spray_rate);
    _mav_put_uint16_t(buf, 10, spray_width);
    _mav_put_uint16_t(buf, 12, calibration_weight);
    _mav_put_uint16_t(buf, 20, spreader_motor_pwm);
    _mav_put_uint16_t(buf, 22, spreader_valve_pwm);
    _mav_put_uint8_t(buf, 24, control_mode);
    _mav_put_uint8_t(buf, 25, pump_calibration_cmd);
    _mav_put_uint8_t(buf, 26, led_control_cmd);
    _mav_put_uint8_t(buf, 27, led_brightness_right);
    _mav_put_uint8_t(buf, 28, led_brightness_left);
    _mav_put_uint8_t(buf, 29, tare_calibration_cmd);
    _mav_put_uint8_t(buf, 30, weight_calibration_cmd);
    _mav_put_uint8_t(buf, 31, k_value_calibration_cmd);
    _mav_put_uint8_t(buf, 32, spreader_control_cmd);
    _mav_put_uint8_t(buf, 33, signal_source_cmd);
    _mav_put_uint8_t(buf, 34, signal_source);
    _mav_put_uint8_t(buf, 35, alarm_config_cmd);
    _mav_put_uint8_t(buf, 39, factory_reset_cmd);
    _mav_put_uint8_t(buf, 40, spray_spreader_mode);
    _mav_put_uint16_t_array(buf, 14, k_values, 3);
    _mav_put_uint8_t_array(buf, 36, alarm_config, 3);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FMU_PMU_UART_MESSAGE, buf, MAVLINK_MSG_ID_FMU_PMU_UART_MESSAGE_MIN_LEN, MAVLINK_MSG_ID_FMU_PMU_UART_MESSAGE_LEN, MAVLINK_MSG_ID_FMU_PMU_UART_MESSAGE_CRC);
#else
    mavlink_fmu_pmu_uart_message_t *packet = (mavlink_fmu_pmu_uart_message_t *)msgbuf;
    packet->nozzle_control = nozzle_control;
    packet->pump_control = pump_control;
    packet->horizontal_speed = horizontal_speed;
    packet->spray_rate = spray_rate;
    packet->spray_width = spray_width;
    packet->calibration_weight = calibration_weight;
    packet->spreader_motor_pwm = spreader_motor_pwm;
    packet->spreader_valve_pwm = spreader_valve_pwm;
    packet->control_mode = control_mode;
    packet->pump_calibration_cmd = pump_calibration_cmd;
    packet->led_control_cmd = led_control_cmd;
    packet->led_brightness_right = led_brightness_right;
    packet->led_brightness_left = led_brightness_left;
    packet->tare_calibration_cmd = tare_calibration_cmd;
    packet->weight_calibration_cmd = weight_calibration_cmd;
    packet->k_value_calibration_cmd = k_value_calibration_cmd;
    packet->spreader_control_cmd = spreader_control_cmd;
    packet->signal_source_cmd = signal_source_cmd;
    packet->signal_source = signal_source;
    packet->alarm_config_cmd = alarm_config_cmd;
    packet->factory_reset_cmd = factory_reset_cmd;
    packet->spray_spreader_mode = spray_spreader_mode;
    mav_array_assign_uint16_t(packet->k_values, k_values, 3);
    mav_array_assign_uint8_t(packet->alarm_config, alarm_config, 3);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FMU_PMU_UART_MESSAGE, (const char *)packet, MAVLINK_MSG_ID_FMU_PMU_UART_MESSAGE_MIN_LEN, MAVLINK_MSG_ID_FMU_PMU_UART_MESSAGE_LEN, MAVLINK_MSG_ID_FMU_PMU_UART_MESSAGE_CRC);
#endif
}
#endif

#endif

// MESSAGE FMU_PMU_UART_MESSAGE UNPACKING


/**
 * @brief Get field pump_control from fmu_pmu_uart_message message
 *
 * @return  Pump control bitmask. bit0 for pump 1, bit1 for pump 2. (0: OFF, 1: ON)
 */
static inline uint16_t mavlink_msg_fmu_pmu_uart_message_get_pump_control(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  4);
}

/**
 * @brief Get field nozzle_control from fmu_pmu_uart_message message
 *
 * @return  Nozzle control bitmask. bit0 for nozzle 1, bit1 for nozzle 2, etc. (0: OFF, 1: ON)
 */
static inline uint32_t mavlink_msg_fmu_pmu_uart_message_get_nozzle_control(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field control_mode from fmu_pmu_uart_message message
 *
 * @return  Control mode. Fixed value 0x00 for PWM mode.
 */
static inline uint8_t mavlink_msg_fmu_pmu_uart_message_get_control_mode(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  24);
}

/**
 * @brief Get field horizontal_speed from fmu_pmu_uart_message message
 *
 * @return [cm/s] Horizontal speed calculated from sqrt(vel.x^2 + vel.y^2).
 */
static inline uint16_t mavlink_msg_fmu_pmu_uart_message_get_horizontal_speed(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  6);
}

/**
 * @brief Get field spray_rate from fmu_pmu_uart_message message
 *
 * @return [mL/mu] Spray rate, e.g., 3000 for 3000mL/mu (Chinese acre).
 */
static inline uint16_t mavlink_msg_fmu_pmu_uart_message_get_spray_rate(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  8);
}

/**
 * @brief Get field spray_width from fmu_pmu_uart_message message
 *
 * @return [cm] Spray width, e.g., 600 for 600cm.
 */
static inline uint16_t mavlink_msg_fmu_pmu_uart_message_get_spray_width(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  10);
}

/**
 * @brief Get field pump_calibration_cmd from fmu_pmu_uart_message message
 *
 * @return  Pump calibration command. 0x11 for pump 1, 0x31 for pump 2, 0x00 otherwise.
 */
static inline uint8_t mavlink_msg_fmu_pmu_uart_message_get_pump_calibration_cmd(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  25);
}

/**
 * @brief Get field led_control_cmd from fmu_pmu_uart_message message
 *
 * @return  LED control command identifier (e.g., 0xE1).
 */
static inline uint8_t mavlink_msg_fmu_pmu_uart_message_get_led_control_cmd(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  26);
}

/**
 * @brief Get field led_brightness_right from fmu_pmu_uart_message message
 *
 * @return [%] Right front LED brightness (0-100).
 */
static inline uint8_t mavlink_msg_fmu_pmu_uart_message_get_led_brightness_right(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  27);
}

/**
 * @brief Get field led_brightness_left from fmu_pmu_uart_message message
 *
 * @return [%] Left front LED brightness (0-100).
 */
static inline uint8_t mavlink_msg_fmu_pmu_uart_message_get_led_brightness_left(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  28);
}

/**
 * @brief Get field tare_calibration_cmd from fmu_pmu_uart_message message
 *
 * @return  Tare calibration command. 0xF6 to execute, 0x00 otherwise.
 */
static inline uint8_t mavlink_msg_fmu_pmu_uart_message_get_tare_calibration_cmd(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  29);
}

/**
 * @brief Get field weight_calibration_cmd from fmu_pmu_uart_message message
 *
 * @return  Weight calibration command identifier (e.g., 0xF7).
 */
static inline uint8_t mavlink_msg_fmu_pmu_uart_message_get_weight_calibration_cmd(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  30);
}

/**
 * @brief Get field calibration_weight from fmu_pmu_uart_message message
 *
 * @return  Calibration weight value.
 */
static inline uint16_t mavlink_msg_fmu_pmu_uart_message_get_calibration_weight(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  12);
}

/**
 * @brief Get field k_value_calibration_cmd from fmu_pmu_uart_message message
 *
 * @return  K-value calibration command identifier (e.g., 0xFC).
 */
static inline uint8_t mavlink_msg_fmu_pmu_uart_message_get_k_value_calibration_cmd(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  31);
}

/**
 * @brief Get field k_values from fmu_pmu_uart_message message
 *
 * @return  Array of 3 K-values.
 */
static inline uint16_t mavlink_msg_fmu_pmu_uart_message_get_k_values(const mavlink_message_t* msg, uint16_t *k_values)
{
    return _MAV_RETURN_uint16_t_array(msg, k_values, 3,  14);
}

/**
 * @brief Get field spreader_control_cmd from fmu_pmu_uart_message message
 *
 * @return  Spreader motor control command identifier (e.g., 0xF1).
 */
static inline uint8_t mavlink_msg_fmu_pmu_uart_message_get_spreader_control_cmd(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  32);
}

/**
 * @brief Get field spreader_motor_pwm from fmu_pmu_uart_message message
 *
 * @return  Spreader motor PWM value (default 1000).
 */
static inline uint16_t mavlink_msg_fmu_pmu_uart_message_get_spreader_motor_pwm(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  20);
}

/**
 * @brief Get field spreader_valve_pwm from fmu_pmu_uart_message message
 *
 * @return  Spreader valve PWM value (default 1000).
 */
static inline uint16_t mavlink_msg_fmu_pmu_uart_message_get_spreader_valve_pwm(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  22);
}

/**
 * @brief Get field signal_source_cmd from fmu_pmu_uart_message message
 *
 * @return  Signal source selection command identifier (e.g., 0xF2).
 */
static inline uint8_t mavlink_msg_fmu_pmu_uart_message_get_signal_source_cmd(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  33);
}

/**
 * @brief Get field signal_source from fmu_pmu_uart_message message
 *
 * @return  Signal source selection (0: PWM, 1: CANBUS).
 */
static inline uint8_t mavlink_msg_fmu_pmu_uart_message_get_signal_source(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  34);
}

/**
 * @brief Get field alarm_config_cmd from fmu_pmu_uart_message message
 *
 * @return  Alarm configuration command identifier (e.g., 0xF3).
 */
static inline uint8_t mavlink_msg_fmu_pmu_uart_message_get_alarm_config_cmd(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  35);
}

/**
 * @brief Get field alarm_config from fmu_pmu_uart_message message
 *
 * @return  Alarm configuration array for material shortage, motor stall, and hall sensor fault alarms.
 */
static inline uint16_t mavlink_msg_fmu_pmu_uart_message_get_alarm_config(const mavlink_message_t* msg, uint8_t *alarm_config)
{
    return _MAV_RETURN_uint8_t_array(msg, alarm_config, 3,  36);
}

/**
 * @brief Get field factory_reset_cmd from fmu_pmu_uart_message message
 *
 * @return  Factory reset command. 0xF5 to execute, 0x00 otherwise.
 */
static inline uint8_t mavlink_msg_fmu_pmu_uart_message_get_factory_reset_cmd(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  39);
}

/**
 * @brief Get field spray_spreader_mode from fmu_pmu_uart_message message
 *
 * @return  Spray/Spreader mode switch. 0x01 to toggle, 0x00 otherwise.
 */
static inline uint8_t mavlink_msg_fmu_pmu_uart_message_get_spray_spreader_mode(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  40);
}

/**
 * @brief Decode a fmu_pmu_uart_message message into a struct
 *
 * @param msg The message to decode
 * @param fmu_pmu_uart_message C-struct to decode the message contents into
 */
static inline void mavlink_msg_fmu_pmu_uart_message_decode(const mavlink_message_t* msg, mavlink_fmu_pmu_uart_message_t* fmu_pmu_uart_message)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    fmu_pmu_uart_message->nozzle_control = mavlink_msg_fmu_pmu_uart_message_get_nozzle_control(msg);
    fmu_pmu_uart_message->pump_control = mavlink_msg_fmu_pmu_uart_message_get_pump_control(msg);
    fmu_pmu_uart_message->horizontal_speed = mavlink_msg_fmu_pmu_uart_message_get_horizontal_speed(msg);
    fmu_pmu_uart_message->spray_rate = mavlink_msg_fmu_pmu_uart_message_get_spray_rate(msg);
    fmu_pmu_uart_message->spray_width = mavlink_msg_fmu_pmu_uart_message_get_spray_width(msg);
    fmu_pmu_uart_message->calibration_weight = mavlink_msg_fmu_pmu_uart_message_get_calibration_weight(msg);
    mavlink_msg_fmu_pmu_uart_message_get_k_values(msg, fmu_pmu_uart_message->k_values);
    fmu_pmu_uart_message->spreader_motor_pwm = mavlink_msg_fmu_pmu_uart_message_get_spreader_motor_pwm(msg);
    fmu_pmu_uart_message->spreader_valve_pwm = mavlink_msg_fmu_pmu_uart_message_get_spreader_valve_pwm(msg);
    fmu_pmu_uart_message->control_mode = mavlink_msg_fmu_pmu_uart_message_get_control_mode(msg);
    fmu_pmu_uart_message->pump_calibration_cmd = mavlink_msg_fmu_pmu_uart_message_get_pump_calibration_cmd(msg);
    fmu_pmu_uart_message->led_control_cmd = mavlink_msg_fmu_pmu_uart_message_get_led_control_cmd(msg);
    fmu_pmu_uart_message->led_brightness_right = mavlink_msg_fmu_pmu_uart_message_get_led_brightness_right(msg);
    fmu_pmu_uart_message->led_brightness_left = mavlink_msg_fmu_pmu_uart_message_get_led_brightness_left(msg);
    fmu_pmu_uart_message->tare_calibration_cmd = mavlink_msg_fmu_pmu_uart_message_get_tare_calibration_cmd(msg);
    fmu_pmu_uart_message->weight_calibration_cmd = mavlink_msg_fmu_pmu_uart_message_get_weight_calibration_cmd(msg);
    fmu_pmu_uart_message->k_value_calibration_cmd = mavlink_msg_fmu_pmu_uart_message_get_k_value_calibration_cmd(msg);
    fmu_pmu_uart_message->spreader_control_cmd = mavlink_msg_fmu_pmu_uart_message_get_spreader_control_cmd(msg);
    fmu_pmu_uart_message->signal_source_cmd = mavlink_msg_fmu_pmu_uart_message_get_signal_source_cmd(msg);
    fmu_pmu_uart_message->signal_source = mavlink_msg_fmu_pmu_uart_message_get_signal_source(msg);
    fmu_pmu_uart_message->alarm_config_cmd = mavlink_msg_fmu_pmu_uart_message_get_alarm_config_cmd(msg);
    mavlink_msg_fmu_pmu_uart_message_get_alarm_config(msg, fmu_pmu_uart_message->alarm_config);
    fmu_pmu_uart_message->factory_reset_cmd = mavlink_msg_fmu_pmu_uart_message_get_factory_reset_cmd(msg);
    fmu_pmu_uart_message->spray_spreader_mode = mavlink_msg_fmu_pmu_uart_message_get_spray_spreader_mode(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_FMU_PMU_UART_MESSAGE_LEN? msg->len : MAVLINK_MSG_ID_FMU_PMU_UART_MESSAGE_LEN;
        memset(fmu_pmu_uart_message, 0, MAVLINK_MSG_ID_FMU_PMU_UART_MESSAGE_LEN);
    memcpy(fmu_pmu_uart_message, _MAV_PAYLOAD(msg), len);
#endif
}
