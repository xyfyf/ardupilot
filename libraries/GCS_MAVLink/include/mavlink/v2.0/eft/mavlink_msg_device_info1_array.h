#pragma once
// MESSAGE DEVICE_INFO1_ARRAY PACKING

#define MAVLINK_MSG_ID_DEVICE_INFO1_ARRAY 502


typedef struct __mavlink_device_info1_array_t {
 uint16_t hours_0; /*<  Device 0 total runtime hours*/
 uint16_t pwm_lost_count_0; /*<  Device 0 PWM signal loss count*/
 uint16_t hours_1; /*<  Device 1 total runtime hours*/
 uint16_t pwm_lost_count_1; /*<  Device 1 PWM signal loss count*/
 uint16_t hours_2; /*<  Device 2 total runtime hours*/
 uint16_t pwm_lost_count_2; /*<  Device 2 PWM signal loss count*/
 uint8_t type_0; /*<  Device 0 type identifier (A-E)*/
 uint8_t year_0; /*<  Device 0 manufacturing year*/
 uint8_t month_0; /*<  Device 0 manufacturing month (1-12)*/
 uint8_t day_0; /*<  Device 0 manufacturing day (1-31)*/
 uint8_t number_0; /*<  Device 0 serial number*/
 uint8_t hw_major_0; /*<  Device 0 hardware major version*/
 uint8_t hw_minor_0; /*<  Device 0 hardware minor version*/
 uint8_t sw_major_0; /*<  Device 0 software major version*/
 uint8_t sw_minor_0; /*<  Device 0 software minor version*/
 uint8_t minutes_0; /*<  Device 0 runtime minutes (0-59)*/
 uint8_t type_1; /*<  Device 1 type identifier (A-E)*/
 uint8_t year_1; /*<  Device 1 manufacturing year*/
 uint8_t month_1; /*<  Device 1 manufacturing month (1-12)*/
 uint8_t day_1; /*<  Device 1 manufacturing day (1-31)*/
 uint8_t number_1; /*<  Device 1 serial number*/
 uint8_t hw_major_1; /*<  Device 1 hardware major version*/
 uint8_t hw_minor_1; /*<  Device 1 hardware minor version*/
 uint8_t sw_major_1; /*<  Device 1 software major version*/
 uint8_t sw_minor_1; /*<  Device 1 software minor version*/
 uint8_t minutes_1; /*<  Device 1 runtime minutes (0-59)*/
 uint8_t type_2; /*<  Device 2 type identifier (A-E)*/
 uint8_t year_2; /*<  Device 2 manufacturing year*/
 uint8_t month_2; /*<  Device 2 manufacturing month (1-12)*/
 uint8_t day_2; /*<  Device 2 manufacturing day (1-31)*/
 uint8_t number_2; /*<  Device 2 serial number*/
 uint8_t hw_major_2; /*<  Device 2 hardware major version*/
 uint8_t hw_minor_2; /*<  Device 2 hardware minor version*/
 uint8_t sw_major_2; /*<  Device 2 software major version*/
 uint8_t sw_minor_2; /*<  Device 2 software minor version*/
 uint8_t minutes_2; /*<  Device 2 runtime minutes (0-59)*/
} mavlink_device_info1_array_t;

#define MAVLINK_MSG_ID_DEVICE_INFO1_ARRAY_LEN 42
#define MAVLINK_MSG_ID_DEVICE_INFO1_ARRAY_MIN_LEN 42
#define MAVLINK_MSG_ID_502_LEN 42
#define MAVLINK_MSG_ID_502_MIN_LEN 42

#define MAVLINK_MSG_ID_DEVICE_INFO1_ARRAY_CRC 152
#define MAVLINK_MSG_ID_502_CRC 152



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_DEVICE_INFO1_ARRAY { \
    502, \
    "DEVICE_INFO1_ARRAY", \
    36, \
    {  { "type_0", NULL, MAVLINK_TYPE_UINT8_T, 0, 12, offsetof(mavlink_device_info1_array_t, type_0) }, \
         { "year_0", NULL, MAVLINK_TYPE_UINT8_T, 0, 13, offsetof(mavlink_device_info1_array_t, year_0) }, \
         { "month_0", NULL, MAVLINK_TYPE_UINT8_T, 0, 14, offsetof(mavlink_device_info1_array_t, month_0) }, \
         { "day_0", NULL, MAVLINK_TYPE_UINT8_T, 0, 15, offsetof(mavlink_device_info1_array_t, day_0) }, \
         { "number_0", NULL, MAVLINK_TYPE_UINT8_T, 0, 16, offsetof(mavlink_device_info1_array_t, number_0) }, \
         { "hw_major_0", NULL, MAVLINK_TYPE_UINT8_T, 0, 17, offsetof(mavlink_device_info1_array_t, hw_major_0) }, \
         { "hw_minor_0", NULL, MAVLINK_TYPE_UINT8_T, 0, 18, offsetof(mavlink_device_info1_array_t, hw_minor_0) }, \
         { "sw_major_0", NULL, MAVLINK_TYPE_UINT8_T, 0, 19, offsetof(mavlink_device_info1_array_t, sw_major_0) }, \
         { "sw_minor_0", NULL, MAVLINK_TYPE_UINT8_T, 0, 20, offsetof(mavlink_device_info1_array_t, sw_minor_0) }, \
         { "minutes_0", NULL, MAVLINK_TYPE_UINT8_T, 0, 21, offsetof(mavlink_device_info1_array_t, minutes_0) }, \
         { "hours_0", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_device_info1_array_t, hours_0) }, \
         { "pwm_lost_count_0", NULL, MAVLINK_TYPE_UINT16_T, 0, 2, offsetof(mavlink_device_info1_array_t, pwm_lost_count_0) }, \
         { "type_1", NULL, MAVLINK_TYPE_UINT8_T, 0, 22, offsetof(mavlink_device_info1_array_t, type_1) }, \
         { "year_1", NULL, MAVLINK_TYPE_UINT8_T, 0, 23, offsetof(mavlink_device_info1_array_t, year_1) }, \
         { "month_1", NULL, MAVLINK_TYPE_UINT8_T, 0, 24, offsetof(mavlink_device_info1_array_t, month_1) }, \
         { "day_1", NULL, MAVLINK_TYPE_UINT8_T, 0, 25, offsetof(mavlink_device_info1_array_t, day_1) }, \
         { "number_1", NULL, MAVLINK_TYPE_UINT8_T, 0, 26, offsetof(mavlink_device_info1_array_t, number_1) }, \
         { "hw_major_1", NULL, MAVLINK_TYPE_UINT8_T, 0, 27, offsetof(mavlink_device_info1_array_t, hw_major_1) }, \
         { "hw_minor_1", NULL, MAVLINK_TYPE_UINT8_T, 0, 28, offsetof(mavlink_device_info1_array_t, hw_minor_1) }, \
         { "sw_major_1", NULL, MAVLINK_TYPE_UINT8_T, 0, 29, offsetof(mavlink_device_info1_array_t, sw_major_1) }, \
         { "sw_minor_1", NULL, MAVLINK_TYPE_UINT8_T, 0, 30, offsetof(mavlink_device_info1_array_t, sw_minor_1) }, \
         { "minutes_1", NULL, MAVLINK_TYPE_UINT8_T, 0, 31, offsetof(mavlink_device_info1_array_t, minutes_1) }, \
         { "hours_1", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_device_info1_array_t, hours_1) }, \
         { "pwm_lost_count_1", NULL, MAVLINK_TYPE_UINT16_T, 0, 6, offsetof(mavlink_device_info1_array_t, pwm_lost_count_1) }, \
         { "type_2", NULL, MAVLINK_TYPE_UINT8_T, 0, 32, offsetof(mavlink_device_info1_array_t, type_2) }, \
         { "year_2", NULL, MAVLINK_TYPE_UINT8_T, 0, 33, offsetof(mavlink_device_info1_array_t, year_2) }, \
         { "month_2", NULL, MAVLINK_TYPE_UINT8_T, 0, 34, offsetof(mavlink_device_info1_array_t, month_2) }, \
         { "day_2", NULL, MAVLINK_TYPE_UINT8_T, 0, 35, offsetof(mavlink_device_info1_array_t, day_2) }, \
         { "number_2", NULL, MAVLINK_TYPE_UINT8_T, 0, 36, offsetof(mavlink_device_info1_array_t, number_2) }, \
         { "hw_major_2", NULL, MAVLINK_TYPE_UINT8_T, 0, 37, offsetof(mavlink_device_info1_array_t, hw_major_2) }, \
         { "hw_minor_2", NULL, MAVLINK_TYPE_UINT8_T, 0, 38, offsetof(mavlink_device_info1_array_t, hw_minor_2) }, \
         { "sw_major_2", NULL, MAVLINK_TYPE_UINT8_T, 0, 39, offsetof(mavlink_device_info1_array_t, sw_major_2) }, \
         { "sw_minor_2", NULL, MAVLINK_TYPE_UINT8_T, 0, 40, offsetof(mavlink_device_info1_array_t, sw_minor_2) }, \
         { "minutes_2", NULL, MAVLINK_TYPE_UINT8_T, 0, 41, offsetof(mavlink_device_info1_array_t, minutes_2) }, \
         { "hours_2", NULL, MAVLINK_TYPE_UINT16_T, 0, 8, offsetof(mavlink_device_info1_array_t, hours_2) }, \
         { "pwm_lost_count_2", NULL, MAVLINK_TYPE_UINT16_T, 0, 10, offsetof(mavlink_device_info1_array_t, pwm_lost_count_2) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_DEVICE_INFO1_ARRAY { \
    "DEVICE_INFO1_ARRAY", \
    36, \
    {  { "type_0", NULL, MAVLINK_TYPE_UINT8_T, 0, 12, offsetof(mavlink_device_info1_array_t, type_0) }, \
         { "year_0", NULL, MAVLINK_TYPE_UINT8_T, 0, 13, offsetof(mavlink_device_info1_array_t, year_0) }, \
         { "month_0", NULL, MAVLINK_TYPE_UINT8_T, 0, 14, offsetof(mavlink_device_info1_array_t, month_0) }, \
         { "day_0", NULL, MAVLINK_TYPE_UINT8_T, 0, 15, offsetof(mavlink_device_info1_array_t, day_0) }, \
         { "number_0", NULL, MAVLINK_TYPE_UINT8_T, 0, 16, offsetof(mavlink_device_info1_array_t, number_0) }, \
         { "hw_major_0", NULL, MAVLINK_TYPE_UINT8_T, 0, 17, offsetof(mavlink_device_info1_array_t, hw_major_0) }, \
         { "hw_minor_0", NULL, MAVLINK_TYPE_UINT8_T, 0, 18, offsetof(mavlink_device_info1_array_t, hw_minor_0) }, \
         { "sw_major_0", NULL, MAVLINK_TYPE_UINT8_T, 0, 19, offsetof(mavlink_device_info1_array_t, sw_major_0) }, \
         { "sw_minor_0", NULL, MAVLINK_TYPE_UINT8_T, 0, 20, offsetof(mavlink_device_info1_array_t, sw_minor_0) }, \
         { "minutes_0", NULL, MAVLINK_TYPE_UINT8_T, 0, 21, offsetof(mavlink_device_info1_array_t, minutes_0) }, \
         { "hours_0", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_device_info1_array_t, hours_0) }, \
         { "pwm_lost_count_0", NULL, MAVLINK_TYPE_UINT16_T, 0, 2, offsetof(mavlink_device_info1_array_t, pwm_lost_count_0) }, \
         { "type_1", NULL, MAVLINK_TYPE_UINT8_T, 0, 22, offsetof(mavlink_device_info1_array_t, type_1) }, \
         { "year_1", NULL, MAVLINK_TYPE_UINT8_T, 0, 23, offsetof(mavlink_device_info1_array_t, year_1) }, \
         { "month_1", NULL, MAVLINK_TYPE_UINT8_T, 0, 24, offsetof(mavlink_device_info1_array_t, month_1) }, \
         { "day_1", NULL, MAVLINK_TYPE_UINT8_T, 0, 25, offsetof(mavlink_device_info1_array_t, day_1) }, \
         { "number_1", NULL, MAVLINK_TYPE_UINT8_T, 0, 26, offsetof(mavlink_device_info1_array_t, number_1) }, \
         { "hw_major_1", NULL, MAVLINK_TYPE_UINT8_T, 0, 27, offsetof(mavlink_device_info1_array_t, hw_major_1) }, \
         { "hw_minor_1", NULL, MAVLINK_TYPE_UINT8_T, 0, 28, offsetof(mavlink_device_info1_array_t, hw_minor_1) }, \
         { "sw_major_1", NULL, MAVLINK_TYPE_UINT8_T, 0, 29, offsetof(mavlink_device_info1_array_t, sw_major_1) }, \
         { "sw_minor_1", NULL, MAVLINK_TYPE_UINT8_T, 0, 30, offsetof(mavlink_device_info1_array_t, sw_minor_1) }, \
         { "minutes_1", NULL, MAVLINK_TYPE_UINT8_T, 0, 31, offsetof(mavlink_device_info1_array_t, minutes_1) }, \
         { "hours_1", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_device_info1_array_t, hours_1) }, \
         { "pwm_lost_count_1", NULL, MAVLINK_TYPE_UINT16_T, 0, 6, offsetof(mavlink_device_info1_array_t, pwm_lost_count_1) }, \
         { "type_2", NULL, MAVLINK_TYPE_UINT8_T, 0, 32, offsetof(mavlink_device_info1_array_t, type_2) }, \
         { "year_2", NULL, MAVLINK_TYPE_UINT8_T, 0, 33, offsetof(mavlink_device_info1_array_t, year_2) }, \
         { "month_2", NULL, MAVLINK_TYPE_UINT8_T, 0, 34, offsetof(mavlink_device_info1_array_t, month_2) }, \
         { "day_2", NULL, MAVLINK_TYPE_UINT8_T, 0, 35, offsetof(mavlink_device_info1_array_t, day_2) }, \
         { "number_2", NULL, MAVLINK_TYPE_UINT8_T, 0, 36, offsetof(mavlink_device_info1_array_t, number_2) }, \
         { "hw_major_2", NULL, MAVLINK_TYPE_UINT8_T, 0, 37, offsetof(mavlink_device_info1_array_t, hw_major_2) }, \
         { "hw_minor_2", NULL, MAVLINK_TYPE_UINT8_T, 0, 38, offsetof(mavlink_device_info1_array_t, hw_minor_2) }, \
         { "sw_major_2", NULL, MAVLINK_TYPE_UINT8_T, 0, 39, offsetof(mavlink_device_info1_array_t, sw_major_2) }, \
         { "sw_minor_2", NULL, MAVLINK_TYPE_UINT8_T, 0, 40, offsetof(mavlink_device_info1_array_t, sw_minor_2) }, \
         { "minutes_2", NULL, MAVLINK_TYPE_UINT8_T, 0, 41, offsetof(mavlink_device_info1_array_t, minutes_2) }, \
         { "hours_2", NULL, MAVLINK_TYPE_UINT16_T, 0, 8, offsetof(mavlink_device_info1_array_t, hours_2) }, \
         { "pwm_lost_count_2", NULL, MAVLINK_TYPE_UINT16_T, 0, 10, offsetof(mavlink_device_info1_array_t, pwm_lost_count_2) }, \
         } \
}
#endif

/**
 * @brief Pack a device_info1_array message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param type_0  Device 0 type identifier (A-E)
 * @param year_0  Device 0 manufacturing year
 * @param month_0  Device 0 manufacturing month (1-12)
 * @param day_0  Device 0 manufacturing day (1-31)
 * @param number_0  Device 0 serial number
 * @param hw_major_0  Device 0 hardware major version
 * @param hw_minor_0  Device 0 hardware minor version
 * @param sw_major_0  Device 0 software major version
 * @param sw_minor_0  Device 0 software minor version
 * @param minutes_0  Device 0 runtime minutes (0-59)
 * @param hours_0  Device 0 total runtime hours
 * @param pwm_lost_count_0  Device 0 PWM signal loss count
 * @param type_1  Device 1 type identifier (A-E)
 * @param year_1  Device 1 manufacturing year
 * @param month_1  Device 1 manufacturing month (1-12)
 * @param day_1  Device 1 manufacturing day (1-31)
 * @param number_1  Device 1 serial number
 * @param hw_major_1  Device 1 hardware major version
 * @param hw_minor_1  Device 1 hardware minor version
 * @param sw_major_1  Device 1 software major version
 * @param sw_minor_1  Device 1 software minor version
 * @param minutes_1  Device 1 runtime minutes (0-59)
 * @param hours_1  Device 1 total runtime hours
 * @param pwm_lost_count_1  Device 1 PWM signal loss count
 * @param type_2  Device 2 type identifier (A-E)
 * @param year_2  Device 2 manufacturing year
 * @param month_2  Device 2 manufacturing month (1-12)
 * @param day_2  Device 2 manufacturing day (1-31)
 * @param number_2  Device 2 serial number
 * @param hw_major_2  Device 2 hardware major version
 * @param hw_minor_2  Device 2 hardware minor version
 * @param sw_major_2  Device 2 software major version
 * @param sw_minor_2  Device 2 software minor version
 * @param minutes_2  Device 2 runtime minutes (0-59)
 * @param hours_2  Device 2 total runtime hours
 * @param pwm_lost_count_2  Device 2 PWM signal loss count
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_device_info1_array_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t type_0, uint8_t year_0, uint8_t month_0, uint8_t day_0, uint8_t number_0, uint8_t hw_major_0, uint8_t hw_minor_0, uint8_t sw_major_0, uint8_t sw_minor_0, uint8_t minutes_0, uint16_t hours_0, uint16_t pwm_lost_count_0, uint8_t type_1, uint8_t year_1, uint8_t month_1, uint8_t day_1, uint8_t number_1, uint8_t hw_major_1, uint8_t hw_minor_1, uint8_t sw_major_1, uint8_t sw_minor_1, uint8_t minutes_1, uint16_t hours_1, uint16_t pwm_lost_count_1, uint8_t type_2, uint8_t year_2, uint8_t month_2, uint8_t day_2, uint8_t number_2, uint8_t hw_major_2, uint8_t hw_minor_2, uint8_t sw_major_2, uint8_t sw_minor_2, uint8_t minutes_2, uint16_t hours_2, uint16_t pwm_lost_count_2)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_DEVICE_INFO1_ARRAY_LEN];
    _mav_put_uint16_t(buf, 0, hours_0);
    _mav_put_uint16_t(buf, 2, pwm_lost_count_0);
    _mav_put_uint16_t(buf, 4, hours_1);
    _mav_put_uint16_t(buf, 6, pwm_lost_count_1);
    _mav_put_uint16_t(buf, 8, hours_2);
    _mav_put_uint16_t(buf, 10, pwm_lost_count_2);
    _mav_put_uint8_t(buf, 12, type_0);
    _mav_put_uint8_t(buf, 13, year_0);
    _mav_put_uint8_t(buf, 14, month_0);
    _mav_put_uint8_t(buf, 15, day_0);
    _mav_put_uint8_t(buf, 16, number_0);
    _mav_put_uint8_t(buf, 17, hw_major_0);
    _mav_put_uint8_t(buf, 18, hw_minor_0);
    _mav_put_uint8_t(buf, 19, sw_major_0);
    _mav_put_uint8_t(buf, 20, sw_minor_0);
    _mav_put_uint8_t(buf, 21, minutes_0);
    _mav_put_uint8_t(buf, 22, type_1);
    _mav_put_uint8_t(buf, 23, year_1);
    _mav_put_uint8_t(buf, 24, month_1);
    _mav_put_uint8_t(buf, 25, day_1);
    _mav_put_uint8_t(buf, 26, number_1);
    _mav_put_uint8_t(buf, 27, hw_major_1);
    _mav_put_uint8_t(buf, 28, hw_minor_1);
    _mav_put_uint8_t(buf, 29, sw_major_1);
    _mav_put_uint8_t(buf, 30, sw_minor_1);
    _mav_put_uint8_t(buf, 31, minutes_1);
    _mav_put_uint8_t(buf, 32, type_2);
    _mav_put_uint8_t(buf, 33, year_2);
    _mav_put_uint8_t(buf, 34, month_2);
    _mav_put_uint8_t(buf, 35, day_2);
    _mav_put_uint8_t(buf, 36, number_2);
    _mav_put_uint8_t(buf, 37, hw_major_2);
    _mav_put_uint8_t(buf, 38, hw_minor_2);
    _mav_put_uint8_t(buf, 39, sw_major_2);
    _mav_put_uint8_t(buf, 40, sw_minor_2);
    _mav_put_uint8_t(buf, 41, minutes_2);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_DEVICE_INFO1_ARRAY_LEN);
#else
    mavlink_device_info1_array_t packet;
    packet.hours_0 = hours_0;
    packet.pwm_lost_count_0 = pwm_lost_count_0;
    packet.hours_1 = hours_1;
    packet.pwm_lost_count_1 = pwm_lost_count_1;
    packet.hours_2 = hours_2;
    packet.pwm_lost_count_2 = pwm_lost_count_2;
    packet.type_0 = type_0;
    packet.year_0 = year_0;
    packet.month_0 = month_0;
    packet.day_0 = day_0;
    packet.number_0 = number_0;
    packet.hw_major_0 = hw_major_0;
    packet.hw_minor_0 = hw_minor_0;
    packet.sw_major_0 = sw_major_0;
    packet.sw_minor_0 = sw_minor_0;
    packet.minutes_0 = minutes_0;
    packet.type_1 = type_1;
    packet.year_1 = year_1;
    packet.month_1 = month_1;
    packet.day_1 = day_1;
    packet.number_1 = number_1;
    packet.hw_major_1 = hw_major_1;
    packet.hw_minor_1 = hw_minor_1;
    packet.sw_major_1 = sw_major_1;
    packet.sw_minor_1 = sw_minor_1;
    packet.minutes_1 = minutes_1;
    packet.type_2 = type_2;
    packet.year_2 = year_2;
    packet.month_2 = month_2;
    packet.day_2 = day_2;
    packet.number_2 = number_2;
    packet.hw_major_2 = hw_major_2;
    packet.hw_minor_2 = hw_minor_2;
    packet.sw_major_2 = sw_major_2;
    packet.sw_minor_2 = sw_minor_2;
    packet.minutes_2 = minutes_2;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_DEVICE_INFO1_ARRAY_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_DEVICE_INFO1_ARRAY;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_DEVICE_INFO1_ARRAY_MIN_LEN, MAVLINK_MSG_ID_DEVICE_INFO1_ARRAY_LEN, MAVLINK_MSG_ID_DEVICE_INFO1_ARRAY_CRC);
}

/**
 * @brief Pack a device_info1_array message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param type_0  Device 0 type identifier (A-E)
 * @param year_0  Device 0 manufacturing year
 * @param month_0  Device 0 manufacturing month (1-12)
 * @param day_0  Device 0 manufacturing day (1-31)
 * @param number_0  Device 0 serial number
 * @param hw_major_0  Device 0 hardware major version
 * @param hw_minor_0  Device 0 hardware minor version
 * @param sw_major_0  Device 0 software major version
 * @param sw_minor_0  Device 0 software minor version
 * @param minutes_0  Device 0 runtime minutes (0-59)
 * @param hours_0  Device 0 total runtime hours
 * @param pwm_lost_count_0  Device 0 PWM signal loss count
 * @param type_1  Device 1 type identifier (A-E)
 * @param year_1  Device 1 manufacturing year
 * @param month_1  Device 1 manufacturing month (1-12)
 * @param day_1  Device 1 manufacturing day (1-31)
 * @param number_1  Device 1 serial number
 * @param hw_major_1  Device 1 hardware major version
 * @param hw_minor_1  Device 1 hardware minor version
 * @param sw_major_1  Device 1 software major version
 * @param sw_minor_1  Device 1 software minor version
 * @param minutes_1  Device 1 runtime minutes (0-59)
 * @param hours_1  Device 1 total runtime hours
 * @param pwm_lost_count_1  Device 1 PWM signal loss count
 * @param type_2  Device 2 type identifier (A-E)
 * @param year_2  Device 2 manufacturing year
 * @param month_2  Device 2 manufacturing month (1-12)
 * @param day_2  Device 2 manufacturing day (1-31)
 * @param number_2  Device 2 serial number
 * @param hw_major_2  Device 2 hardware major version
 * @param hw_minor_2  Device 2 hardware minor version
 * @param sw_major_2  Device 2 software major version
 * @param sw_minor_2  Device 2 software minor version
 * @param minutes_2  Device 2 runtime minutes (0-59)
 * @param hours_2  Device 2 total runtime hours
 * @param pwm_lost_count_2  Device 2 PWM signal loss count
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_device_info1_array_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint8_t type_0, uint8_t year_0, uint8_t month_0, uint8_t day_0, uint8_t number_0, uint8_t hw_major_0, uint8_t hw_minor_0, uint8_t sw_major_0, uint8_t sw_minor_0, uint8_t minutes_0, uint16_t hours_0, uint16_t pwm_lost_count_0, uint8_t type_1, uint8_t year_1, uint8_t month_1, uint8_t day_1, uint8_t number_1, uint8_t hw_major_1, uint8_t hw_minor_1, uint8_t sw_major_1, uint8_t sw_minor_1, uint8_t minutes_1, uint16_t hours_1, uint16_t pwm_lost_count_1, uint8_t type_2, uint8_t year_2, uint8_t month_2, uint8_t day_2, uint8_t number_2, uint8_t hw_major_2, uint8_t hw_minor_2, uint8_t sw_major_2, uint8_t sw_minor_2, uint8_t minutes_2, uint16_t hours_2, uint16_t pwm_lost_count_2)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_DEVICE_INFO1_ARRAY_LEN];
    _mav_put_uint16_t(buf, 0, hours_0);
    _mav_put_uint16_t(buf, 2, pwm_lost_count_0);
    _mav_put_uint16_t(buf, 4, hours_1);
    _mav_put_uint16_t(buf, 6, pwm_lost_count_1);
    _mav_put_uint16_t(buf, 8, hours_2);
    _mav_put_uint16_t(buf, 10, pwm_lost_count_2);
    _mav_put_uint8_t(buf, 12, type_0);
    _mav_put_uint8_t(buf, 13, year_0);
    _mav_put_uint8_t(buf, 14, month_0);
    _mav_put_uint8_t(buf, 15, day_0);
    _mav_put_uint8_t(buf, 16, number_0);
    _mav_put_uint8_t(buf, 17, hw_major_0);
    _mav_put_uint8_t(buf, 18, hw_minor_0);
    _mav_put_uint8_t(buf, 19, sw_major_0);
    _mav_put_uint8_t(buf, 20, sw_minor_0);
    _mav_put_uint8_t(buf, 21, minutes_0);
    _mav_put_uint8_t(buf, 22, type_1);
    _mav_put_uint8_t(buf, 23, year_1);
    _mav_put_uint8_t(buf, 24, month_1);
    _mav_put_uint8_t(buf, 25, day_1);
    _mav_put_uint8_t(buf, 26, number_1);
    _mav_put_uint8_t(buf, 27, hw_major_1);
    _mav_put_uint8_t(buf, 28, hw_minor_1);
    _mav_put_uint8_t(buf, 29, sw_major_1);
    _mav_put_uint8_t(buf, 30, sw_minor_1);
    _mav_put_uint8_t(buf, 31, minutes_1);
    _mav_put_uint8_t(buf, 32, type_2);
    _mav_put_uint8_t(buf, 33, year_2);
    _mav_put_uint8_t(buf, 34, month_2);
    _mav_put_uint8_t(buf, 35, day_2);
    _mav_put_uint8_t(buf, 36, number_2);
    _mav_put_uint8_t(buf, 37, hw_major_2);
    _mav_put_uint8_t(buf, 38, hw_minor_2);
    _mav_put_uint8_t(buf, 39, sw_major_2);
    _mav_put_uint8_t(buf, 40, sw_minor_2);
    _mav_put_uint8_t(buf, 41, minutes_2);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_DEVICE_INFO1_ARRAY_LEN);
#else
    mavlink_device_info1_array_t packet;
    packet.hours_0 = hours_0;
    packet.pwm_lost_count_0 = pwm_lost_count_0;
    packet.hours_1 = hours_1;
    packet.pwm_lost_count_1 = pwm_lost_count_1;
    packet.hours_2 = hours_2;
    packet.pwm_lost_count_2 = pwm_lost_count_2;
    packet.type_0 = type_0;
    packet.year_0 = year_0;
    packet.month_0 = month_0;
    packet.day_0 = day_0;
    packet.number_0 = number_0;
    packet.hw_major_0 = hw_major_0;
    packet.hw_minor_0 = hw_minor_0;
    packet.sw_major_0 = sw_major_0;
    packet.sw_minor_0 = sw_minor_0;
    packet.minutes_0 = minutes_0;
    packet.type_1 = type_1;
    packet.year_1 = year_1;
    packet.month_1 = month_1;
    packet.day_1 = day_1;
    packet.number_1 = number_1;
    packet.hw_major_1 = hw_major_1;
    packet.hw_minor_1 = hw_minor_1;
    packet.sw_major_1 = sw_major_1;
    packet.sw_minor_1 = sw_minor_1;
    packet.minutes_1 = minutes_1;
    packet.type_2 = type_2;
    packet.year_2 = year_2;
    packet.month_2 = month_2;
    packet.day_2 = day_2;
    packet.number_2 = number_2;
    packet.hw_major_2 = hw_major_2;
    packet.hw_minor_2 = hw_minor_2;
    packet.sw_major_2 = sw_major_2;
    packet.sw_minor_2 = sw_minor_2;
    packet.minutes_2 = minutes_2;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_DEVICE_INFO1_ARRAY_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_DEVICE_INFO1_ARRAY;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_DEVICE_INFO1_ARRAY_MIN_LEN, MAVLINK_MSG_ID_DEVICE_INFO1_ARRAY_LEN, MAVLINK_MSG_ID_DEVICE_INFO1_ARRAY_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_DEVICE_INFO1_ARRAY_MIN_LEN, MAVLINK_MSG_ID_DEVICE_INFO1_ARRAY_LEN);
#endif
}

/**
 * @brief Pack a device_info1_array message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param type_0  Device 0 type identifier (A-E)
 * @param year_0  Device 0 manufacturing year
 * @param month_0  Device 0 manufacturing month (1-12)
 * @param day_0  Device 0 manufacturing day (1-31)
 * @param number_0  Device 0 serial number
 * @param hw_major_0  Device 0 hardware major version
 * @param hw_minor_0  Device 0 hardware minor version
 * @param sw_major_0  Device 0 software major version
 * @param sw_minor_0  Device 0 software minor version
 * @param minutes_0  Device 0 runtime minutes (0-59)
 * @param hours_0  Device 0 total runtime hours
 * @param pwm_lost_count_0  Device 0 PWM signal loss count
 * @param type_1  Device 1 type identifier (A-E)
 * @param year_1  Device 1 manufacturing year
 * @param month_1  Device 1 manufacturing month (1-12)
 * @param day_1  Device 1 manufacturing day (1-31)
 * @param number_1  Device 1 serial number
 * @param hw_major_1  Device 1 hardware major version
 * @param hw_minor_1  Device 1 hardware minor version
 * @param sw_major_1  Device 1 software major version
 * @param sw_minor_1  Device 1 software minor version
 * @param minutes_1  Device 1 runtime minutes (0-59)
 * @param hours_1  Device 1 total runtime hours
 * @param pwm_lost_count_1  Device 1 PWM signal loss count
 * @param type_2  Device 2 type identifier (A-E)
 * @param year_2  Device 2 manufacturing year
 * @param month_2  Device 2 manufacturing month (1-12)
 * @param day_2  Device 2 manufacturing day (1-31)
 * @param number_2  Device 2 serial number
 * @param hw_major_2  Device 2 hardware major version
 * @param hw_minor_2  Device 2 hardware minor version
 * @param sw_major_2  Device 2 software major version
 * @param sw_minor_2  Device 2 software minor version
 * @param minutes_2  Device 2 runtime minutes (0-59)
 * @param hours_2  Device 2 total runtime hours
 * @param pwm_lost_count_2  Device 2 PWM signal loss count
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_device_info1_array_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t type_0,uint8_t year_0,uint8_t month_0,uint8_t day_0,uint8_t number_0,uint8_t hw_major_0,uint8_t hw_minor_0,uint8_t sw_major_0,uint8_t sw_minor_0,uint8_t minutes_0,uint16_t hours_0,uint16_t pwm_lost_count_0,uint8_t type_1,uint8_t year_1,uint8_t month_1,uint8_t day_1,uint8_t number_1,uint8_t hw_major_1,uint8_t hw_minor_1,uint8_t sw_major_1,uint8_t sw_minor_1,uint8_t minutes_1,uint16_t hours_1,uint16_t pwm_lost_count_1,uint8_t type_2,uint8_t year_2,uint8_t month_2,uint8_t day_2,uint8_t number_2,uint8_t hw_major_2,uint8_t hw_minor_2,uint8_t sw_major_2,uint8_t sw_minor_2,uint8_t minutes_2,uint16_t hours_2,uint16_t pwm_lost_count_2)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_DEVICE_INFO1_ARRAY_LEN];
    _mav_put_uint16_t(buf, 0, hours_0);
    _mav_put_uint16_t(buf, 2, pwm_lost_count_0);
    _mav_put_uint16_t(buf, 4, hours_1);
    _mav_put_uint16_t(buf, 6, pwm_lost_count_1);
    _mav_put_uint16_t(buf, 8, hours_2);
    _mav_put_uint16_t(buf, 10, pwm_lost_count_2);
    _mav_put_uint8_t(buf, 12, type_0);
    _mav_put_uint8_t(buf, 13, year_0);
    _mav_put_uint8_t(buf, 14, month_0);
    _mav_put_uint8_t(buf, 15, day_0);
    _mav_put_uint8_t(buf, 16, number_0);
    _mav_put_uint8_t(buf, 17, hw_major_0);
    _mav_put_uint8_t(buf, 18, hw_minor_0);
    _mav_put_uint8_t(buf, 19, sw_major_0);
    _mav_put_uint8_t(buf, 20, sw_minor_0);
    _mav_put_uint8_t(buf, 21, minutes_0);
    _mav_put_uint8_t(buf, 22, type_1);
    _mav_put_uint8_t(buf, 23, year_1);
    _mav_put_uint8_t(buf, 24, month_1);
    _mav_put_uint8_t(buf, 25, day_1);
    _mav_put_uint8_t(buf, 26, number_1);
    _mav_put_uint8_t(buf, 27, hw_major_1);
    _mav_put_uint8_t(buf, 28, hw_minor_1);
    _mav_put_uint8_t(buf, 29, sw_major_1);
    _mav_put_uint8_t(buf, 30, sw_minor_1);
    _mav_put_uint8_t(buf, 31, minutes_1);
    _mav_put_uint8_t(buf, 32, type_2);
    _mav_put_uint8_t(buf, 33, year_2);
    _mav_put_uint8_t(buf, 34, month_2);
    _mav_put_uint8_t(buf, 35, day_2);
    _mav_put_uint8_t(buf, 36, number_2);
    _mav_put_uint8_t(buf, 37, hw_major_2);
    _mav_put_uint8_t(buf, 38, hw_minor_2);
    _mav_put_uint8_t(buf, 39, sw_major_2);
    _mav_put_uint8_t(buf, 40, sw_minor_2);
    _mav_put_uint8_t(buf, 41, minutes_2);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_DEVICE_INFO1_ARRAY_LEN);
#else
    mavlink_device_info1_array_t packet;
    packet.hours_0 = hours_0;
    packet.pwm_lost_count_0 = pwm_lost_count_0;
    packet.hours_1 = hours_1;
    packet.pwm_lost_count_1 = pwm_lost_count_1;
    packet.hours_2 = hours_2;
    packet.pwm_lost_count_2 = pwm_lost_count_2;
    packet.type_0 = type_0;
    packet.year_0 = year_0;
    packet.month_0 = month_0;
    packet.day_0 = day_0;
    packet.number_0 = number_0;
    packet.hw_major_0 = hw_major_0;
    packet.hw_minor_0 = hw_minor_0;
    packet.sw_major_0 = sw_major_0;
    packet.sw_minor_0 = sw_minor_0;
    packet.minutes_0 = minutes_0;
    packet.type_1 = type_1;
    packet.year_1 = year_1;
    packet.month_1 = month_1;
    packet.day_1 = day_1;
    packet.number_1 = number_1;
    packet.hw_major_1 = hw_major_1;
    packet.hw_minor_1 = hw_minor_1;
    packet.sw_major_1 = sw_major_1;
    packet.sw_minor_1 = sw_minor_1;
    packet.minutes_1 = minutes_1;
    packet.type_2 = type_2;
    packet.year_2 = year_2;
    packet.month_2 = month_2;
    packet.day_2 = day_2;
    packet.number_2 = number_2;
    packet.hw_major_2 = hw_major_2;
    packet.hw_minor_2 = hw_minor_2;
    packet.sw_major_2 = sw_major_2;
    packet.sw_minor_2 = sw_minor_2;
    packet.minutes_2 = minutes_2;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_DEVICE_INFO1_ARRAY_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_DEVICE_INFO1_ARRAY;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_DEVICE_INFO1_ARRAY_MIN_LEN, MAVLINK_MSG_ID_DEVICE_INFO1_ARRAY_LEN, MAVLINK_MSG_ID_DEVICE_INFO1_ARRAY_CRC);
}

/**
 * @brief Encode a device_info1_array struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param device_info1_array C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_device_info1_array_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_device_info1_array_t* device_info1_array)
{
    return mavlink_msg_device_info1_array_pack(system_id, component_id, msg, device_info1_array->type_0, device_info1_array->year_0, device_info1_array->month_0, device_info1_array->day_0, device_info1_array->number_0, device_info1_array->hw_major_0, device_info1_array->hw_minor_0, device_info1_array->sw_major_0, device_info1_array->sw_minor_0, device_info1_array->minutes_0, device_info1_array->hours_0, device_info1_array->pwm_lost_count_0, device_info1_array->type_1, device_info1_array->year_1, device_info1_array->month_1, device_info1_array->day_1, device_info1_array->number_1, device_info1_array->hw_major_1, device_info1_array->hw_minor_1, device_info1_array->sw_major_1, device_info1_array->sw_minor_1, device_info1_array->minutes_1, device_info1_array->hours_1, device_info1_array->pwm_lost_count_1, device_info1_array->type_2, device_info1_array->year_2, device_info1_array->month_2, device_info1_array->day_2, device_info1_array->number_2, device_info1_array->hw_major_2, device_info1_array->hw_minor_2, device_info1_array->sw_major_2, device_info1_array->sw_minor_2, device_info1_array->minutes_2, device_info1_array->hours_2, device_info1_array->pwm_lost_count_2);
}

/**
 * @brief Encode a device_info1_array struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param device_info1_array C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_device_info1_array_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_device_info1_array_t* device_info1_array)
{
    return mavlink_msg_device_info1_array_pack_chan(system_id, component_id, chan, msg, device_info1_array->type_0, device_info1_array->year_0, device_info1_array->month_0, device_info1_array->day_0, device_info1_array->number_0, device_info1_array->hw_major_0, device_info1_array->hw_minor_0, device_info1_array->sw_major_0, device_info1_array->sw_minor_0, device_info1_array->minutes_0, device_info1_array->hours_0, device_info1_array->pwm_lost_count_0, device_info1_array->type_1, device_info1_array->year_1, device_info1_array->month_1, device_info1_array->day_1, device_info1_array->number_1, device_info1_array->hw_major_1, device_info1_array->hw_minor_1, device_info1_array->sw_major_1, device_info1_array->sw_minor_1, device_info1_array->minutes_1, device_info1_array->hours_1, device_info1_array->pwm_lost_count_1, device_info1_array->type_2, device_info1_array->year_2, device_info1_array->month_2, device_info1_array->day_2, device_info1_array->number_2, device_info1_array->hw_major_2, device_info1_array->hw_minor_2, device_info1_array->sw_major_2, device_info1_array->sw_minor_2, device_info1_array->minutes_2, device_info1_array->hours_2, device_info1_array->pwm_lost_count_2);
}

/**
 * @brief Encode a device_info1_array struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param device_info1_array C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_device_info1_array_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_device_info1_array_t* device_info1_array)
{
    return mavlink_msg_device_info1_array_pack_status(system_id, component_id, _status, msg,  device_info1_array->type_0, device_info1_array->year_0, device_info1_array->month_0, device_info1_array->day_0, device_info1_array->number_0, device_info1_array->hw_major_0, device_info1_array->hw_minor_0, device_info1_array->sw_major_0, device_info1_array->sw_minor_0, device_info1_array->minutes_0, device_info1_array->hours_0, device_info1_array->pwm_lost_count_0, device_info1_array->type_1, device_info1_array->year_1, device_info1_array->month_1, device_info1_array->day_1, device_info1_array->number_1, device_info1_array->hw_major_1, device_info1_array->hw_minor_1, device_info1_array->sw_major_1, device_info1_array->sw_minor_1, device_info1_array->minutes_1, device_info1_array->hours_1, device_info1_array->pwm_lost_count_1, device_info1_array->type_2, device_info1_array->year_2, device_info1_array->month_2, device_info1_array->day_2, device_info1_array->number_2, device_info1_array->hw_major_2, device_info1_array->hw_minor_2, device_info1_array->sw_major_2, device_info1_array->sw_minor_2, device_info1_array->minutes_2, device_info1_array->hours_2, device_info1_array->pwm_lost_count_2);
}

/**
 * @brief Send a device_info1_array message
 * @param chan MAVLink channel to send the message
 *
 * @param type_0  Device 0 type identifier (A-E)
 * @param year_0  Device 0 manufacturing year
 * @param month_0  Device 0 manufacturing month (1-12)
 * @param day_0  Device 0 manufacturing day (1-31)
 * @param number_0  Device 0 serial number
 * @param hw_major_0  Device 0 hardware major version
 * @param hw_minor_0  Device 0 hardware minor version
 * @param sw_major_0  Device 0 software major version
 * @param sw_minor_0  Device 0 software minor version
 * @param minutes_0  Device 0 runtime minutes (0-59)
 * @param hours_0  Device 0 total runtime hours
 * @param pwm_lost_count_0  Device 0 PWM signal loss count
 * @param type_1  Device 1 type identifier (A-E)
 * @param year_1  Device 1 manufacturing year
 * @param month_1  Device 1 manufacturing month (1-12)
 * @param day_1  Device 1 manufacturing day (1-31)
 * @param number_1  Device 1 serial number
 * @param hw_major_1  Device 1 hardware major version
 * @param hw_minor_1  Device 1 hardware minor version
 * @param sw_major_1  Device 1 software major version
 * @param sw_minor_1  Device 1 software minor version
 * @param minutes_1  Device 1 runtime minutes (0-59)
 * @param hours_1  Device 1 total runtime hours
 * @param pwm_lost_count_1  Device 1 PWM signal loss count
 * @param type_2  Device 2 type identifier (A-E)
 * @param year_2  Device 2 manufacturing year
 * @param month_2  Device 2 manufacturing month (1-12)
 * @param day_2  Device 2 manufacturing day (1-31)
 * @param number_2  Device 2 serial number
 * @param hw_major_2  Device 2 hardware major version
 * @param hw_minor_2  Device 2 hardware minor version
 * @param sw_major_2  Device 2 software major version
 * @param sw_minor_2  Device 2 software minor version
 * @param minutes_2  Device 2 runtime minutes (0-59)
 * @param hours_2  Device 2 total runtime hours
 * @param pwm_lost_count_2  Device 2 PWM signal loss count
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_device_info1_array_send(mavlink_channel_t chan, uint8_t type_0, uint8_t year_0, uint8_t month_0, uint8_t day_0, uint8_t number_0, uint8_t hw_major_0, uint8_t hw_minor_0, uint8_t sw_major_0, uint8_t sw_minor_0, uint8_t minutes_0, uint16_t hours_0, uint16_t pwm_lost_count_0, uint8_t type_1, uint8_t year_1, uint8_t month_1, uint8_t day_1, uint8_t number_1, uint8_t hw_major_1, uint8_t hw_minor_1, uint8_t sw_major_1, uint8_t sw_minor_1, uint8_t minutes_1, uint16_t hours_1, uint16_t pwm_lost_count_1, uint8_t type_2, uint8_t year_2, uint8_t month_2, uint8_t day_2, uint8_t number_2, uint8_t hw_major_2, uint8_t hw_minor_2, uint8_t sw_major_2, uint8_t sw_minor_2, uint8_t minutes_2, uint16_t hours_2, uint16_t pwm_lost_count_2)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_DEVICE_INFO1_ARRAY_LEN];
    _mav_put_uint16_t(buf, 0, hours_0);
    _mav_put_uint16_t(buf, 2, pwm_lost_count_0);
    _mav_put_uint16_t(buf, 4, hours_1);
    _mav_put_uint16_t(buf, 6, pwm_lost_count_1);
    _mav_put_uint16_t(buf, 8, hours_2);
    _mav_put_uint16_t(buf, 10, pwm_lost_count_2);
    _mav_put_uint8_t(buf, 12, type_0);
    _mav_put_uint8_t(buf, 13, year_0);
    _mav_put_uint8_t(buf, 14, month_0);
    _mav_put_uint8_t(buf, 15, day_0);
    _mav_put_uint8_t(buf, 16, number_0);
    _mav_put_uint8_t(buf, 17, hw_major_0);
    _mav_put_uint8_t(buf, 18, hw_minor_0);
    _mav_put_uint8_t(buf, 19, sw_major_0);
    _mav_put_uint8_t(buf, 20, sw_minor_0);
    _mav_put_uint8_t(buf, 21, minutes_0);
    _mav_put_uint8_t(buf, 22, type_1);
    _mav_put_uint8_t(buf, 23, year_1);
    _mav_put_uint8_t(buf, 24, month_1);
    _mav_put_uint8_t(buf, 25, day_1);
    _mav_put_uint8_t(buf, 26, number_1);
    _mav_put_uint8_t(buf, 27, hw_major_1);
    _mav_put_uint8_t(buf, 28, hw_minor_1);
    _mav_put_uint8_t(buf, 29, sw_major_1);
    _mav_put_uint8_t(buf, 30, sw_minor_1);
    _mav_put_uint8_t(buf, 31, minutes_1);
    _mav_put_uint8_t(buf, 32, type_2);
    _mav_put_uint8_t(buf, 33, year_2);
    _mav_put_uint8_t(buf, 34, month_2);
    _mav_put_uint8_t(buf, 35, day_2);
    _mav_put_uint8_t(buf, 36, number_2);
    _mav_put_uint8_t(buf, 37, hw_major_2);
    _mav_put_uint8_t(buf, 38, hw_minor_2);
    _mav_put_uint8_t(buf, 39, sw_major_2);
    _mav_put_uint8_t(buf, 40, sw_minor_2);
    _mav_put_uint8_t(buf, 41, minutes_2);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DEVICE_INFO1_ARRAY, buf, MAVLINK_MSG_ID_DEVICE_INFO1_ARRAY_MIN_LEN, MAVLINK_MSG_ID_DEVICE_INFO1_ARRAY_LEN, MAVLINK_MSG_ID_DEVICE_INFO1_ARRAY_CRC);
#else
    mavlink_device_info1_array_t packet;
    packet.hours_0 = hours_0;
    packet.pwm_lost_count_0 = pwm_lost_count_0;
    packet.hours_1 = hours_1;
    packet.pwm_lost_count_1 = pwm_lost_count_1;
    packet.hours_2 = hours_2;
    packet.pwm_lost_count_2 = pwm_lost_count_2;
    packet.type_0 = type_0;
    packet.year_0 = year_0;
    packet.month_0 = month_0;
    packet.day_0 = day_0;
    packet.number_0 = number_0;
    packet.hw_major_0 = hw_major_0;
    packet.hw_minor_0 = hw_minor_0;
    packet.sw_major_0 = sw_major_0;
    packet.sw_minor_0 = sw_minor_0;
    packet.minutes_0 = minutes_0;
    packet.type_1 = type_1;
    packet.year_1 = year_1;
    packet.month_1 = month_1;
    packet.day_1 = day_1;
    packet.number_1 = number_1;
    packet.hw_major_1 = hw_major_1;
    packet.hw_minor_1 = hw_minor_1;
    packet.sw_major_1 = sw_major_1;
    packet.sw_minor_1 = sw_minor_1;
    packet.minutes_1 = minutes_1;
    packet.type_2 = type_2;
    packet.year_2 = year_2;
    packet.month_2 = month_2;
    packet.day_2 = day_2;
    packet.number_2 = number_2;
    packet.hw_major_2 = hw_major_2;
    packet.hw_minor_2 = hw_minor_2;
    packet.sw_major_2 = sw_major_2;
    packet.sw_minor_2 = sw_minor_2;
    packet.minutes_2 = minutes_2;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DEVICE_INFO1_ARRAY, (const char *)&packet, MAVLINK_MSG_ID_DEVICE_INFO1_ARRAY_MIN_LEN, MAVLINK_MSG_ID_DEVICE_INFO1_ARRAY_LEN, MAVLINK_MSG_ID_DEVICE_INFO1_ARRAY_CRC);
#endif
}

/**
 * @brief Send a device_info1_array message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_device_info1_array_send_struct(mavlink_channel_t chan, const mavlink_device_info1_array_t* device_info1_array)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_device_info1_array_send(chan, device_info1_array->type_0, device_info1_array->year_0, device_info1_array->month_0, device_info1_array->day_0, device_info1_array->number_0, device_info1_array->hw_major_0, device_info1_array->hw_minor_0, device_info1_array->sw_major_0, device_info1_array->sw_minor_0, device_info1_array->minutes_0, device_info1_array->hours_0, device_info1_array->pwm_lost_count_0, device_info1_array->type_1, device_info1_array->year_1, device_info1_array->month_1, device_info1_array->day_1, device_info1_array->number_1, device_info1_array->hw_major_1, device_info1_array->hw_minor_1, device_info1_array->sw_major_1, device_info1_array->sw_minor_1, device_info1_array->minutes_1, device_info1_array->hours_1, device_info1_array->pwm_lost_count_1, device_info1_array->type_2, device_info1_array->year_2, device_info1_array->month_2, device_info1_array->day_2, device_info1_array->number_2, device_info1_array->hw_major_2, device_info1_array->hw_minor_2, device_info1_array->sw_major_2, device_info1_array->sw_minor_2, device_info1_array->minutes_2, device_info1_array->hours_2, device_info1_array->pwm_lost_count_2);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DEVICE_INFO1_ARRAY, (const char *)device_info1_array, MAVLINK_MSG_ID_DEVICE_INFO1_ARRAY_MIN_LEN, MAVLINK_MSG_ID_DEVICE_INFO1_ARRAY_LEN, MAVLINK_MSG_ID_DEVICE_INFO1_ARRAY_CRC);
#endif
}

#if MAVLINK_MSG_ID_DEVICE_INFO1_ARRAY_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by reusing
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_device_info1_array_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t type_0, uint8_t year_0, uint8_t month_0, uint8_t day_0, uint8_t number_0, uint8_t hw_major_0, uint8_t hw_minor_0, uint8_t sw_major_0, uint8_t sw_minor_0, uint8_t minutes_0, uint16_t hours_0, uint16_t pwm_lost_count_0, uint8_t type_1, uint8_t year_1, uint8_t month_1, uint8_t day_1, uint8_t number_1, uint8_t hw_major_1, uint8_t hw_minor_1, uint8_t sw_major_1, uint8_t sw_minor_1, uint8_t minutes_1, uint16_t hours_1, uint16_t pwm_lost_count_1, uint8_t type_2, uint8_t year_2, uint8_t month_2, uint8_t day_2, uint8_t number_2, uint8_t hw_major_2, uint8_t hw_minor_2, uint8_t sw_major_2, uint8_t sw_minor_2, uint8_t minutes_2, uint16_t hours_2, uint16_t pwm_lost_count_2)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint16_t(buf, 0, hours_0);
    _mav_put_uint16_t(buf, 2, pwm_lost_count_0);
    _mav_put_uint16_t(buf, 4, hours_1);
    _mav_put_uint16_t(buf, 6, pwm_lost_count_1);
    _mav_put_uint16_t(buf, 8, hours_2);
    _mav_put_uint16_t(buf, 10, pwm_lost_count_2);
    _mav_put_uint8_t(buf, 12, type_0);
    _mav_put_uint8_t(buf, 13, year_0);
    _mav_put_uint8_t(buf, 14, month_0);
    _mav_put_uint8_t(buf, 15, day_0);
    _mav_put_uint8_t(buf, 16, number_0);
    _mav_put_uint8_t(buf, 17, hw_major_0);
    _mav_put_uint8_t(buf, 18, hw_minor_0);
    _mav_put_uint8_t(buf, 19, sw_major_0);
    _mav_put_uint8_t(buf, 20, sw_minor_0);
    _mav_put_uint8_t(buf, 21, minutes_0);
    _mav_put_uint8_t(buf, 22, type_1);
    _mav_put_uint8_t(buf, 23, year_1);
    _mav_put_uint8_t(buf, 24, month_1);
    _mav_put_uint8_t(buf, 25, day_1);
    _mav_put_uint8_t(buf, 26, number_1);
    _mav_put_uint8_t(buf, 27, hw_major_1);
    _mav_put_uint8_t(buf, 28, hw_minor_1);
    _mav_put_uint8_t(buf, 29, sw_major_1);
    _mav_put_uint8_t(buf, 30, sw_minor_1);
    _mav_put_uint8_t(buf, 31, minutes_1);
    _mav_put_uint8_t(buf, 32, type_2);
    _mav_put_uint8_t(buf, 33, year_2);
    _mav_put_uint8_t(buf, 34, month_2);
    _mav_put_uint8_t(buf, 35, day_2);
    _mav_put_uint8_t(buf, 36, number_2);
    _mav_put_uint8_t(buf, 37, hw_major_2);
    _mav_put_uint8_t(buf, 38, hw_minor_2);
    _mav_put_uint8_t(buf, 39, sw_major_2);
    _mav_put_uint8_t(buf, 40, sw_minor_2);
    _mav_put_uint8_t(buf, 41, minutes_2);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DEVICE_INFO1_ARRAY, buf, MAVLINK_MSG_ID_DEVICE_INFO1_ARRAY_MIN_LEN, MAVLINK_MSG_ID_DEVICE_INFO1_ARRAY_LEN, MAVLINK_MSG_ID_DEVICE_INFO1_ARRAY_CRC);
#else
    mavlink_device_info1_array_t *packet = (mavlink_device_info1_array_t *)msgbuf;
    packet->hours_0 = hours_0;
    packet->pwm_lost_count_0 = pwm_lost_count_0;
    packet->hours_1 = hours_1;
    packet->pwm_lost_count_1 = pwm_lost_count_1;
    packet->hours_2 = hours_2;
    packet->pwm_lost_count_2 = pwm_lost_count_2;
    packet->type_0 = type_0;
    packet->year_0 = year_0;
    packet->month_0 = month_0;
    packet->day_0 = day_0;
    packet->number_0 = number_0;
    packet->hw_major_0 = hw_major_0;
    packet->hw_minor_0 = hw_minor_0;
    packet->sw_major_0 = sw_major_0;
    packet->sw_minor_0 = sw_minor_0;
    packet->minutes_0 = minutes_0;
    packet->type_1 = type_1;
    packet->year_1 = year_1;
    packet->month_1 = month_1;
    packet->day_1 = day_1;
    packet->number_1 = number_1;
    packet->hw_major_1 = hw_major_1;
    packet->hw_minor_1 = hw_minor_1;
    packet->sw_major_1 = sw_major_1;
    packet->sw_minor_1 = sw_minor_1;
    packet->minutes_1 = minutes_1;
    packet->type_2 = type_2;
    packet->year_2 = year_2;
    packet->month_2 = month_2;
    packet->day_2 = day_2;
    packet->number_2 = number_2;
    packet->hw_major_2 = hw_major_2;
    packet->hw_minor_2 = hw_minor_2;
    packet->sw_major_2 = sw_major_2;
    packet->sw_minor_2 = sw_minor_2;
    packet->minutes_2 = minutes_2;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DEVICE_INFO1_ARRAY, (const char *)packet, MAVLINK_MSG_ID_DEVICE_INFO1_ARRAY_MIN_LEN, MAVLINK_MSG_ID_DEVICE_INFO1_ARRAY_LEN, MAVLINK_MSG_ID_DEVICE_INFO1_ARRAY_CRC);
#endif
}
#endif

#endif

// MESSAGE DEVICE_INFO1_ARRAY UNPACKING


/**
 * @brief Get field type_0 from device_info1_array message
 *
 * @return  Device 0 type identifier (A-E)
 */
static inline uint8_t mavlink_msg_device_info1_array_get_type_0(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  12);
}

/**
 * @brief Get field year_0 from device_info1_array message
 *
 * @return  Device 0 manufacturing year
 */
static inline uint8_t mavlink_msg_device_info1_array_get_year_0(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  13);
}

/**
 * @brief Get field month_0 from device_info1_array message
 *
 * @return  Device 0 manufacturing month (1-12)
 */
static inline uint8_t mavlink_msg_device_info1_array_get_month_0(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  14);
}

/**
 * @brief Get field day_0 from device_info1_array message
 *
 * @return  Device 0 manufacturing day (1-31)
 */
static inline uint8_t mavlink_msg_device_info1_array_get_day_0(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  15);
}

/**
 * @brief Get field number_0 from device_info1_array message
 *
 * @return  Device 0 serial number
 */
static inline uint8_t mavlink_msg_device_info1_array_get_number_0(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  16);
}

/**
 * @brief Get field hw_major_0 from device_info1_array message
 *
 * @return  Device 0 hardware major version
 */
static inline uint8_t mavlink_msg_device_info1_array_get_hw_major_0(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  17);
}

/**
 * @brief Get field hw_minor_0 from device_info1_array message
 *
 * @return  Device 0 hardware minor version
 */
static inline uint8_t mavlink_msg_device_info1_array_get_hw_minor_0(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  18);
}

/**
 * @brief Get field sw_major_0 from device_info1_array message
 *
 * @return  Device 0 software major version
 */
static inline uint8_t mavlink_msg_device_info1_array_get_sw_major_0(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  19);
}

/**
 * @brief Get field sw_minor_0 from device_info1_array message
 *
 * @return  Device 0 software minor version
 */
static inline uint8_t mavlink_msg_device_info1_array_get_sw_minor_0(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  20);
}

/**
 * @brief Get field minutes_0 from device_info1_array message
 *
 * @return  Device 0 runtime minutes (0-59)
 */
static inline uint8_t mavlink_msg_device_info1_array_get_minutes_0(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  21);
}

/**
 * @brief Get field hours_0 from device_info1_array message
 *
 * @return  Device 0 total runtime hours
 */
static inline uint16_t mavlink_msg_device_info1_array_get_hours_0(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  0);
}

/**
 * @brief Get field pwm_lost_count_0 from device_info1_array message
 *
 * @return  Device 0 PWM signal loss count
 */
static inline uint16_t mavlink_msg_device_info1_array_get_pwm_lost_count_0(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  2);
}

/**
 * @brief Get field type_1 from device_info1_array message
 *
 * @return  Device 1 type identifier (A-E)
 */
static inline uint8_t mavlink_msg_device_info1_array_get_type_1(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  22);
}

/**
 * @brief Get field year_1 from device_info1_array message
 *
 * @return  Device 1 manufacturing year
 */
static inline uint8_t mavlink_msg_device_info1_array_get_year_1(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  23);
}

/**
 * @brief Get field month_1 from device_info1_array message
 *
 * @return  Device 1 manufacturing month (1-12)
 */
static inline uint8_t mavlink_msg_device_info1_array_get_month_1(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  24);
}

/**
 * @brief Get field day_1 from device_info1_array message
 *
 * @return  Device 1 manufacturing day (1-31)
 */
static inline uint8_t mavlink_msg_device_info1_array_get_day_1(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  25);
}

/**
 * @brief Get field number_1 from device_info1_array message
 *
 * @return  Device 1 serial number
 */
static inline uint8_t mavlink_msg_device_info1_array_get_number_1(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  26);
}

/**
 * @brief Get field hw_major_1 from device_info1_array message
 *
 * @return  Device 1 hardware major version
 */
static inline uint8_t mavlink_msg_device_info1_array_get_hw_major_1(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  27);
}

/**
 * @brief Get field hw_minor_1 from device_info1_array message
 *
 * @return  Device 1 hardware minor version
 */
static inline uint8_t mavlink_msg_device_info1_array_get_hw_minor_1(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  28);
}

/**
 * @brief Get field sw_major_1 from device_info1_array message
 *
 * @return  Device 1 software major version
 */
static inline uint8_t mavlink_msg_device_info1_array_get_sw_major_1(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  29);
}

/**
 * @brief Get field sw_minor_1 from device_info1_array message
 *
 * @return  Device 1 software minor version
 */
static inline uint8_t mavlink_msg_device_info1_array_get_sw_minor_1(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  30);
}

/**
 * @brief Get field minutes_1 from device_info1_array message
 *
 * @return  Device 1 runtime minutes (0-59)
 */
static inline uint8_t mavlink_msg_device_info1_array_get_minutes_1(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  31);
}

/**
 * @brief Get field hours_1 from device_info1_array message
 *
 * @return  Device 1 total runtime hours
 */
static inline uint16_t mavlink_msg_device_info1_array_get_hours_1(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  4);
}

/**
 * @brief Get field pwm_lost_count_1 from device_info1_array message
 *
 * @return  Device 1 PWM signal loss count
 */
static inline uint16_t mavlink_msg_device_info1_array_get_pwm_lost_count_1(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  6);
}

/**
 * @brief Get field type_2 from device_info1_array message
 *
 * @return  Device 2 type identifier (A-E)
 */
static inline uint8_t mavlink_msg_device_info1_array_get_type_2(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  32);
}

/**
 * @brief Get field year_2 from device_info1_array message
 *
 * @return  Device 2 manufacturing year
 */
static inline uint8_t mavlink_msg_device_info1_array_get_year_2(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  33);
}

/**
 * @brief Get field month_2 from device_info1_array message
 *
 * @return  Device 2 manufacturing month (1-12)
 */
static inline uint8_t mavlink_msg_device_info1_array_get_month_2(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  34);
}

/**
 * @brief Get field day_2 from device_info1_array message
 *
 * @return  Device 2 manufacturing day (1-31)
 */
static inline uint8_t mavlink_msg_device_info1_array_get_day_2(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  35);
}

/**
 * @brief Get field number_2 from device_info1_array message
 *
 * @return  Device 2 serial number
 */
static inline uint8_t mavlink_msg_device_info1_array_get_number_2(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  36);
}

/**
 * @brief Get field hw_major_2 from device_info1_array message
 *
 * @return  Device 2 hardware major version
 */
static inline uint8_t mavlink_msg_device_info1_array_get_hw_major_2(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  37);
}

/**
 * @brief Get field hw_minor_2 from device_info1_array message
 *
 * @return  Device 2 hardware minor version
 */
static inline uint8_t mavlink_msg_device_info1_array_get_hw_minor_2(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  38);
}

/**
 * @brief Get field sw_major_2 from device_info1_array message
 *
 * @return  Device 2 software major version
 */
static inline uint8_t mavlink_msg_device_info1_array_get_sw_major_2(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  39);
}

/**
 * @brief Get field sw_minor_2 from device_info1_array message
 *
 * @return  Device 2 software minor version
 */
static inline uint8_t mavlink_msg_device_info1_array_get_sw_minor_2(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  40);
}

/**
 * @brief Get field minutes_2 from device_info1_array message
 *
 * @return  Device 2 runtime minutes (0-59)
 */
static inline uint8_t mavlink_msg_device_info1_array_get_minutes_2(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  41);
}

/**
 * @brief Get field hours_2 from device_info1_array message
 *
 * @return  Device 2 total runtime hours
 */
static inline uint16_t mavlink_msg_device_info1_array_get_hours_2(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  8);
}

/**
 * @brief Get field pwm_lost_count_2 from device_info1_array message
 *
 * @return  Device 2 PWM signal loss count
 */
static inline uint16_t mavlink_msg_device_info1_array_get_pwm_lost_count_2(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  10);
}

/**
 * @brief Decode a device_info1_array message into a struct
 *
 * @param msg The message to decode
 * @param device_info1_array C-struct to decode the message contents into
 */
static inline void mavlink_msg_device_info1_array_decode(const mavlink_message_t* msg, mavlink_device_info1_array_t* device_info1_array)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    device_info1_array->hours_0 = mavlink_msg_device_info1_array_get_hours_0(msg);
    device_info1_array->pwm_lost_count_0 = mavlink_msg_device_info1_array_get_pwm_lost_count_0(msg);
    device_info1_array->hours_1 = mavlink_msg_device_info1_array_get_hours_1(msg);
    device_info1_array->pwm_lost_count_1 = mavlink_msg_device_info1_array_get_pwm_lost_count_1(msg);
    device_info1_array->hours_2 = mavlink_msg_device_info1_array_get_hours_2(msg);
    device_info1_array->pwm_lost_count_2 = mavlink_msg_device_info1_array_get_pwm_lost_count_2(msg);
    device_info1_array->type_0 = mavlink_msg_device_info1_array_get_type_0(msg);
    device_info1_array->year_0 = mavlink_msg_device_info1_array_get_year_0(msg);
    device_info1_array->month_0 = mavlink_msg_device_info1_array_get_month_0(msg);
    device_info1_array->day_0 = mavlink_msg_device_info1_array_get_day_0(msg);
    device_info1_array->number_0 = mavlink_msg_device_info1_array_get_number_0(msg);
    device_info1_array->hw_major_0 = mavlink_msg_device_info1_array_get_hw_major_0(msg);
    device_info1_array->hw_minor_0 = mavlink_msg_device_info1_array_get_hw_minor_0(msg);
    device_info1_array->sw_major_0 = mavlink_msg_device_info1_array_get_sw_major_0(msg);
    device_info1_array->sw_minor_0 = mavlink_msg_device_info1_array_get_sw_minor_0(msg);
    device_info1_array->minutes_0 = mavlink_msg_device_info1_array_get_minutes_0(msg);
    device_info1_array->type_1 = mavlink_msg_device_info1_array_get_type_1(msg);
    device_info1_array->year_1 = mavlink_msg_device_info1_array_get_year_1(msg);
    device_info1_array->month_1 = mavlink_msg_device_info1_array_get_month_1(msg);
    device_info1_array->day_1 = mavlink_msg_device_info1_array_get_day_1(msg);
    device_info1_array->number_1 = mavlink_msg_device_info1_array_get_number_1(msg);
    device_info1_array->hw_major_1 = mavlink_msg_device_info1_array_get_hw_major_1(msg);
    device_info1_array->hw_minor_1 = mavlink_msg_device_info1_array_get_hw_minor_1(msg);
    device_info1_array->sw_major_1 = mavlink_msg_device_info1_array_get_sw_major_1(msg);
    device_info1_array->sw_minor_1 = mavlink_msg_device_info1_array_get_sw_minor_1(msg);
    device_info1_array->minutes_1 = mavlink_msg_device_info1_array_get_minutes_1(msg);
    device_info1_array->type_2 = mavlink_msg_device_info1_array_get_type_2(msg);
    device_info1_array->year_2 = mavlink_msg_device_info1_array_get_year_2(msg);
    device_info1_array->month_2 = mavlink_msg_device_info1_array_get_month_2(msg);
    device_info1_array->day_2 = mavlink_msg_device_info1_array_get_day_2(msg);
    device_info1_array->number_2 = mavlink_msg_device_info1_array_get_number_2(msg);
    device_info1_array->hw_major_2 = mavlink_msg_device_info1_array_get_hw_major_2(msg);
    device_info1_array->hw_minor_2 = mavlink_msg_device_info1_array_get_hw_minor_2(msg);
    device_info1_array->sw_major_2 = mavlink_msg_device_info1_array_get_sw_major_2(msg);
    device_info1_array->sw_minor_2 = mavlink_msg_device_info1_array_get_sw_minor_2(msg);
    device_info1_array->minutes_2 = mavlink_msg_device_info1_array_get_minutes_2(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_DEVICE_INFO1_ARRAY_LEN? msg->len : MAVLINK_MSG_ID_DEVICE_INFO1_ARRAY_LEN;
        memset(device_info1_array, 0, MAVLINK_MSG_ID_DEVICE_INFO1_ARRAY_LEN);
    memcpy(device_info1_array, _MAV_PAYLOAD(msg), len);
#endif
}
