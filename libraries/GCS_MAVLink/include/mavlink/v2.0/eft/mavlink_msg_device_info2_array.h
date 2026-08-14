#pragma once
// MESSAGE DEVICE_INFO2_ARRAY PACKING

#define MAVLINK_MSG_ID_DEVICE_INFO2_ARRAY 503


typedef struct __mavlink_device_info2_array_t {
 uint16_t hours_3; /*<  Device 3 total runtime hours*/
 uint16_t pwm_lost_count_3; /*<  Device 3 PWM signal loss count*/
 uint16_t hours_4; /*<  Device 4 total runtime hours*/
 uint16_t pwm_lost_count_4; /*<  Device 4 PWM signal loss count*/
 uint16_t hours_5; /*<  Device 5 total runtime hours*/
 uint16_t pwm_lost_count_5; /*<  Device 5 PWM signal loss count*/
 uint8_t type_3; /*<  Device 3 type identifier (A-E)*/
 uint8_t year_3; /*<  Device 3 manufacturing year*/
 uint8_t month_3; /*<  Device 3 manufacturing month (1-12)*/
 uint8_t day_3; /*<  Device 3 manufacturing day (1-31)*/
 uint8_t number_3; /*<  Device 3 serial number*/
 uint8_t hw_major_3; /*<  Device 3 hardware major version*/
 uint8_t hw_minor_3; /*<  Device 3 hardware minor version*/
 uint8_t sw_major_3; /*<  Device 3 software major version*/
 uint8_t sw_minor_3; /*<  Device 3 software minor version*/
 uint8_t minutes_3; /*<  Device 3 runtime minutes (0-59)*/
 uint8_t type_4; /*<  Device 4 type identifier (A-E)*/
 uint8_t year_4; /*<  Device 4 manufacturing year*/
 uint8_t month_4; /*<  Device 4 manufacturing month (1-12)*/
 uint8_t day_4; /*<  Device 4 manufacturing day (1-31)*/
 uint8_t number_4; /*<  Device 4 serial number*/
 uint8_t hw_major_4; /*<  Device 4 hardware major version*/
 uint8_t hw_minor_4; /*<  Device 4 hardware minor version*/
 uint8_t sw_major_4; /*<  Device 4 software major version*/
 uint8_t sw_minor_4; /*<  Device 4 software minor version*/
 uint8_t minutes_4; /*<  Device 4 runtime minutes (0-59)*/
 uint8_t type_5; /*<  Device 5 type identifier (A-E)*/
 uint8_t year_5; /*<  Device 5 manufacturing year*/
 uint8_t month_5; /*<  Device 5 manufacturing month (1-12)*/
 uint8_t day_5; /*<  Device 5 manufacturing day (1-31)*/
 uint8_t number_5; /*<  Device 5 serial number*/
 uint8_t hw_major_5; /*<  Device 5 hardware major version*/
 uint8_t hw_minor_5; /*<  Device 5 hardware minor version*/
 uint8_t sw_major_5; /*<  Device 5 software major version*/
 uint8_t sw_minor_5; /*<  Device 5 software minor version*/
 uint8_t minutes_5; /*<  Device 5 runtime minutes (0-59)*/
} mavlink_device_info2_array_t;

#define MAVLINK_MSG_ID_DEVICE_INFO2_ARRAY_LEN 42
#define MAVLINK_MSG_ID_DEVICE_INFO2_ARRAY_MIN_LEN 42
#define MAVLINK_MSG_ID_503_LEN 42
#define MAVLINK_MSG_ID_503_MIN_LEN 42

#define MAVLINK_MSG_ID_DEVICE_INFO2_ARRAY_CRC 161
#define MAVLINK_MSG_ID_503_CRC 161



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_DEVICE_INFO2_ARRAY { \
    503, \
    "DEVICE_INFO2_ARRAY", \
    36, \
    {  { "type_3", NULL, MAVLINK_TYPE_UINT8_T, 0, 12, offsetof(mavlink_device_info2_array_t, type_3) }, \
         { "year_3", NULL, MAVLINK_TYPE_UINT8_T, 0, 13, offsetof(mavlink_device_info2_array_t, year_3) }, \
         { "month_3", NULL, MAVLINK_TYPE_UINT8_T, 0, 14, offsetof(mavlink_device_info2_array_t, month_3) }, \
         { "day_3", NULL, MAVLINK_TYPE_UINT8_T, 0, 15, offsetof(mavlink_device_info2_array_t, day_3) }, \
         { "number_3", NULL, MAVLINK_TYPE_UINT8_T, 0, 16, offsetof(mavlink_device_info2_array_t, number_3) }, \
         { "hw_major_3", NULL, MAVLINK_TYPE_UINT8_T, 0, 17, offsetof(mavlink_device_info2_array_t, hw_major_3) }, \
         { "hw_minor_3", NULL, MAVLINK_TYPE_UINT8_T, 0, 18, offsetof(mavlink_device_info2_array_t, hw_minor_3) }, \
         { "sw_major_3", NULL, MAVLINK_TYPE_UINT8_T, 0, 19, offsetof(mavlink_device_info2_array_t, sw_major_3) }, \
         { "sw_minor_3", NULL, MAVLINK_TYPE_UINT8_T, 0, 20, offsetof(mavlink_device_info2_array_t, sw_minor_3) }, \
         { "minutes_3", NULL, MAVLINK_TYPE_UINT8_T, 0, 21, offsetof(mavlink_device_info2_array_t, minutes_3) }, \
         { "hours_3", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_device_info2_array_t, hours_3) }, \
         { "pwm_lost_count_3", NULL, MAVLINK_TYPE_UINT16_T, 0, 2, offsetof(mavlink_device_info2_array_t, pwm_lost_count_3) }, \
         { "type_4", NULL, MAVLINK_TYPE_UINT8_T, 0, 22, offsetof(mavlink_device_info2_array_t, type_4) }, \
         { "year_4", NULL, MAVLINK_TYPE_UINT8_T, 0, 23, offsetof(mavlink_device_info2_array_t, year_4) }, \
         { "month_4", NULL, MAVLINK_TYPE_UINT8_T, 0, 24, offsetof(mavlink_device_info2_array_t, month_4) }, \
         { "day_4", NULL, MAVLINK_TYPE_UINT8_T, 0, 25, offsetof(mavlink_device_info2_array_t, day_4) }, \
         { "number_4", NULL, MAVLINK_TYPE_UINT8_T, 0, 26, offsetof(mavlink_device_info2_array_t, number_4) }, \
         { "hw_major_4", NULL, MAVLINK_TYPE_UINT8_T, 0, 27, offsetof(mavlink_device_info2_array_t, hw_major_4) }, \
         { "hw_minor_4", NULL, MAVLINK_TYPE_UINT8_T, 0, 28, offsetof(mavlink_device_info2_array_t, hw_minor_4) }, \
         { "sw_major_4", NULL, MAVLINK_TYPE_UINT8_T, 0, 29, offsetof(mavlink_device_info2_array_t, sw_major_4) }, \
         { "sw_minor_4", NULL, MAVLINK_TYPE_UINT8_T, 0, 30, offsetof(mavlink_device_info2_array_t, sw_minor_4) }, \
         { "minutes_4", NULL, MAVLINK_TYPE_UINT8_T, 0, 31, offsetof(mavlink_device_info2_array_t, minutes_4) }, \
         { "hours_4", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_device_info2_array_t, hours_4) }, \
         { "pwm_lost_count_4", NULL, MAVLINK_TYPE_UINT16_T, 0, 6, offsetof(mavlink_device_info2_array_t, pwm_lost_count_4) }, \
         { "type_5", NULL, MAVLINK_TYPE_UINT8_T, 0, 32, offsetof(mavlink_device_info2_array_t, type_5) }, \
         { "year_5", NULL, MAVLINK_TYPE_UINT8_T, 0, 33, offsetof(mavlink_device_info2_array_t, year_5) }, \
         { "month_5", NULL, MAVLINK_TYPE_UINT8_T, 0, 34, offsetof(mavlink_device_info2_array_t, month_5) }, \
         { "day_5", NULL, MAVLINK_TYPE_UINT8_T, 0, 35, offsetof(mavlink_device_info2_array_t, day_5) }, \
         { "number_5", NULL, MAVLINK_TYPE_UINT8_T, 0, 36, offsetof(mavlink_device_info2_array_t, number_5) }, \
         { "hw_major_5", NULL, MAVLINK_TYPE_UINT8_T, 0, 37, offsetof(mavlink_device_info2_array_t, hw_major_5) }, \
         { "hw_minor_5", NULL, MAVLINK_TYPE_UINT8_T, 0, 38, offsetof(mavlink_device_info2_array_t, hw_minor_5) }, \
         { "sw_major_5", NULL, MAVLINK_TYPE_UINT8_T, 0, 39, offsetof(mavlink_device_info2_array_t, sw_major_5) }, \
         { "sw_minor_5", NULL, MAVLINK_TYPE_UINT8_T, 0, 40, offsetof(mavlink_device_info2_array_t, sw_minor_5) }, \
         { "minutes_5", NULL, MAVLINK_TYPE_UINT8_T, 0, 41, offsetof(mavlink_device_info2_array_t, minutes_5) }, \
         { "hours_5", NULL, MAVLINK_TYPE_UINT16_T, 0, 8, offsetof(mavlink_device_info2_array_t, hours_5) }, \
         { "pwm_lost_count_5", NULL, MAVLINK_TYPE_UINT16_T, 0, 10, offsetof(mavlink_device_info2_array_t, pwm_lost_count_5) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_DEVICE_INFO2_ARRAY { \
    "DEVICE_INFO2_ARRAY", \
    36, \
    {  { "type_3", NULL, MAVLINK_TYPE_UINT8_T, 0, 12, offsetof(mavlink_device_info2_array_t, type_3) }, \
         { "year_3", NULL, MAVLINK_TYPE_UINT8_T, 0, 13, offsetof(mavlink_device_info2_array_t, year_3) }, \
         { "month_3", NULL, MAVLINK_TYPE_UINT8_T, 0, 14, offsetof(mavlink_device_info2_array_t, month_3) }, \
         { "day_3", NULL, MAVLINK_TYPE_UINT8_T, 0, 15, offsetof(mavlink_device_info2_array_t, day_3) }, \
         { "number_3", NULL, MAVLINK_TYPE_UINT8_T, 0, 16, offsetof(mavlink_device_info2_array_t, number_3) }, \
         { "hw_major_3", NULL, MAVLINK_TYPE_UINT8_T, 0, 17, offsetof(mavlink_device_info2_array_t, hw_major_3) }, \
         { "hw_minor_3", NULL, MAVLINK_TYPE_UINT8_T, 0, 18, offsetof(mavlink_device_info2_array_t, hw_minor_3) }, \
         { "sw_major_3", NULL, MAVLINK_TYPE_UINT8_T, 0, 19, offsetof(mavlink_device_info2_array_t, sw_major_3) }, \
         { "sw_minor_3", NULL, MAVLINK_TYPE_UINT8_T, 0, 20, offsetof(mavlink_device_info2_array_t, sw_minor_3) }, \
         { "minutes_3", NULL, MAVLINK_TYPE_UINT8_T, 0, 21, offsetof(mavlink_device_info2_array_t, minutes_3) }, \
         { "hours_3", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_device_info2_array_t, hours_3) }, \
         { "pwm_lost_count_3", NULL, MAVLINK_TYPE_UINT16_T, 0, 2, offsetof(mavlink_device_info2_array_t, pwm_lost_count_3) }, \
         { "type_4", NULL, MAVLINK_TYPE_UINT8_T, 0, 22, offsetof(mavlink_device_info2_array_t, type_4) }, \
         { "year_4", NULL, MAVLINK_TYPE_UINT8_T, 0, 23, offsetof(mavlink_device_info2_array_t, year_4) }, \
         { "month_4", NULL, MAVLINK_TYPE_UINT8_T, 0, 24, offsetof(mavlink_device_info2_array_t, month_4) }, \
         { "day_4", NULL, MAVLINK_TYPE_UINT8_T, 0, 25, offsetof(mavlink_device_info2_array_t, day_4) }, \
         { "number_4", NULL, MAVLINK_TYPE_UINT8_T, 0, 26, offsetof(mavlink_device_info2_array_t, number_4) }, \
         { "hw_major_4", NULL, MAVLINK_TYPE_UINT8_T, 0, 27, offsetof(mavlink_device_info2_array_t, hw_major_4) }, \
         { "hw_minor_4", NULL, MAVLINK_TYPE_UINT8_T, 0, 28, offsetof(mavlink_device_info2_array_t, hw_minor_4) }, \
         { "sw_major_4", NULL, MAVLINK_TYPE_UINT8_T, 0, 29, offsetof(mavlink_device_info2_array_t, sw_major_4) }, \
         { "sw_minor_4", NULL, MAVLINK_TYPE_UINT8_T, 0, 30, offsetof(mavlink_device_info2_array_t, sw_minor_4) }, \
         { "minutes_4", NULL, MAVLINK_TYPE_UINT8_T, 0, 31, offsetof(mavlink_device_info2_array_t, minutes_4) }, \
         { "hours_4", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_device_info2_array_t, hours_4) }, \
         { "pwm_lost_count_4", NULL, MAVLINK_TYPE_UINT16_T, 0, 6, offsetof(mavlink_device_info2_array_t, pwm_lost_count_4) }, \
         { "type_5", NULL, MAVLINK_TYPE_UINT8_T, 0, 32, offsetof(mavlink_device_info2_array_t, type_5) }, \
         { "year_5", NULL, MAVLINK_TYPE_UINT8_T, 0, 33, offsetof(mavlink_device_info2_array_t, year_5) }, \
         { "month_5", NULL, MAVLINK_TYPE_UINT8_T, 0, 34, offsetof(mavlink_device_info2_array_t, month_5) }, \
         { "day_5", NULL, MAVLINK_TYPE_UINT8_T, 0, 35, offsetof(mavlink_device_info2_array_t, day_5) }, \
         { "number_5", NULL, MAVLINK_TYPE_UINT8_T, 0, 36, offsetof(mavlink_device_info2_array_t, number_5) }, \
         { "hw_major_5", NULL, MAVLINK_TYPE_UINT8_T, 0, 37, offsetof(mavlink_device_info2_array_t, hw_major_5) }, \
         { "hw_minor_5", NULL, MAVLINK_TYPE_UINT8_T, 0, 38, offsetof(mavlink_device_info2_array_t, hw_minor_5) }, \
         { "sw_major_5", NULL, MAVLINK_TYPE_UINT8_T, 0, 39, offsetof(mavlink_device_info2_array_t, sw_major_5) }, \
         { "sw_minor_5", NULL, MAVLINK_TYPE_UINT8_T, 0, 40, offsetof(mavlink_device_info2_array_t, sw_minor_5) }, \
         { "minutes_5", NULL, MAVLINK_TYPE_UINT8_T, 0, 41, offsetof(mavlink_device_info2_array_t, minutes_5) }, \
         { "hours_5", NULL, MAVLINK_TYPE_UINT16_T, 0, 8, offsetof(mavlink_device_info2_array_t, hours_5) }, \
         { "pwm_lost_count_5", NULL, MAVLINK_TYPE_UINT16_T, 0, 10, offsetof(mavlink_device_info2_array_t, pwm_lost_count_5) }, \
         } \
}
#endif

/**
 * @brief Pack a device_info2_array message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param type_3  Device 3 type identifier (A-E)
 * @param year_3  Device 3 manufacturing year
 * @param month_3  Device 3 manufacturing month (1-12)
 * @param day_3  Device 3 manufacturing day (1-31)
 * @param number_3  Device 3 serial number
 * @param hw_major_3  Device 3 hardware major version
 * @param hw_minor_3  Device 3 hardware minor version
 * @param sw_major_3  Device 3 software major version
 * @param sw_minor_3  Device 3 software minor version
 * @param minutes_3  Device 3 runtime minutes (0-59)
 * @param hours_3  Device 3 total runtime hours
 * @param pwm_lost_count_3  Device 3 PWM signal loss count
 * @param type_4  Device 4 type identifier (A-E)
 * @param year_4  Device 4 manufacturing year
 * @param month_4  Device 4 manufacturing month (1-12)
 * @param day_4  Device 4 manufacturing day (1-31)
 * @param number_4  Device 4 serial number
 * @param hw_major_4  Device 4 hardware major version
 * @param hw_minor_4  Device 4 hardware minor version
 * @param sw_major_4  Device 4 software major version
 * @param sw_minor_4  Device 4 software minor version
 * @param minutes_4  Device 4 runtime minutes (0-59)
 * @param hours_4  Device 4 total runtime hours
 * @param pwm_lost_count_4  Device 4 PWM signal loss count
 * @param type_5  Device 5 type identifier (A-E)
 * @param year_5  Device 5 manufacturing year
 * @param month_5  Device 5 manufacturing month (1-12)
 * @param day_5  Device 5 manufacturing day (1-31)
 * @param number_5  Device 5 serial number
 * @param hw_major_5  Device 5 hardware major version
 * @param hw_minor_5  Device 5 hardware minor version
 * @param sw_major_5  Device 5 software major version
 * @param sw_minor_5  Device 5 software minor version
 * @param minutes_5  Device 5 runtime minutes (0-59)
 * @param hours_5  Device 5 total runtime hours
 * @param pwm_lost_count_5  Device 5 PWM signal loss count
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_device_info2_array_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t type_3, uint8_t year_3, uint8_t month_3, uint8_t day_3, uint8_t number_3, uint8_t hw_major_3, uint8_t hw_minor_3, uint8_t sw_major_3, uint8_t sw_minor_3, uint8_t minutes_3, uint16_t hours_3, uint16_t pwm_lost_count_3, uint8_t type_4, uint8_t year_4, uint8_t month_4, uint8_t day_4, uint8_t number_4, uint8_t hw_major_4, uint8_t hw_minor_4, uint8_t sw_major_4, uint8_t sw_minor_4, uint8_t minutes_4, uint16_t hours_4, uint16_t pwm_lost_count_4, uint8_t type_5, uint8_t year_5, uint8_t month_5, uint8_t day_5, uint8_t number_5, uint8_t hw_major_5, uint8_t hw_minor_5, uint8_t sw_major_5, uint8_t sw_minor_5, uint8_t minutes_5, uint16_t hours_5, uint16_t pwm_lost_count_5)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_DEVICE_INFO2_ARRAY_LEN];
    _mav_put_uint16_t(buf, 0, hours_3);
    _mav_put_uint16_t(buf, 2, pwm_lost_count_3);
    _mav_put_uint16_t(buf, 4, hours_4);
    _mav_put_uint16_t(buf, 6, pwm_lost_count_4);
    _mav_put_uint16_t(buf, 8, hours_5);
    _mav_put_uint16_t(buf, 10, pwm_lost_count_5);
    _mav_put_uint8_t(buf, 12, type_3);
    _mav_put_uint8_t(buf, 13, year_3);
    _mav_put_uint8_t(buf, 14, month_3);
    _mav_put_uint8_t(buf, 15, day_3);
    _mav_put_uint8_t(buf, 16, number_3);
    _mav_put_uint8_t(buf, 17, hw_major_3);
    _mav_put_uint8_t(buf, 18, hw_minor_3);
    _mav_put_uint8_t(buf, 19, sw_major_3);
    _mav_put_uint8_t(buf, 20, sw_minor_3);
    _mav_put_uint8_t(buf, 21, minutes_3);
    _mav_put_uint8_t(buf, 22, type_4);
    _mav_put_uint8_t(buf, 23, year_4);
    _mav_put_uint8_t(buf, 24, month_4);
    _mav_put_uint8_t(buf, 25, day_4);
    _mav_put_uint8_t(buf, 26, number_4);
    _mav_put_uint8_t(buf, 27, hw_major_4);
    _mav_put_uint8_t(buf, 28, hw_minor_4);
    _mav_put_uint8_t(buf, 29, sw_major_4);
    _mav_put_uint8_t(buf, 30, sw_minor_4);
    _mav_put_uint8_t(buf, 31, minutes_4);
    _mav_put_uint8_t(buf, 32, type_5);
    _mav_put_uint8_t(buf, 33, year_5);
    _mav_put_uint8_t(buf, 34, month_5);
    _mav_put_uint8_t(buf, 35, day_5);
    _mav_put_uint8_t(buf, 36, number_5);
    _mav_put_uint8_t(buf, 37, hw_major_5);
    _mav_put_uint8_t(buf, 38, hw_minor_5);
    _mav_put_uint8_t(buf, 39, sw_major_5);
    _mav_put_uint8_t(buf, 40, sw_minor_5);
    _mav_put_uint8_t(buf, 41, minutes_5);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_DEVICE_INFO2_ARRAY_LEN);
#else
    mavlink_device_info2_array_t packet;
    packet.hours_3 = hours_3;
    packet.pwm_lost_count_3 = pwm_lost_count_3;
    packet.hours_4 = hours_4;
    packet.pwm_lost_count_4 = pwm_lost_count_4;
    packet.hours_5 = hours_5;
    packet.pwm_lost_count_5 = pwm_lost_count_5;
    packet.type_3 = type_3;
    packet.year_3 = year_3;
    packet.month_3 = month_3;
    packet.day_3 = day_3;
    packet.number_3 = number_3;
    packet.hw_major_3 = hw_major_3;
    packet.hw_minor_3 = hw_minor_3;
    packet.sw_major_3 = sw_major_3;
    packet.sw_minor_3 = sw_minor_3;
    packet.minutes_3 = minutes_3;
    packet.type_4 = type_4;
    packet.year_4 = year_4;
    packet.month_4 = month_4;
    packet.day_4 = day_4;
    packet.number_4 = number_4;
    packet.hw_major_4 = hw_major_4;
    packet.hw_minor_4 = hw_minor_4;
    packet.sw_major_4 = sw_major_4;
    packet.sw_minor_4 = sw_minor_4;
    packet.minutes_4 = minutes_4;
    packet.type_5 = type_5;
    packet.year_5 = year_5;
    packet.month_5 = month_5;
    packet.day_5 = day_5;
    packet.number_5 = number_5;
    packet.hw_major_5 = hw_major_5;
    packet.hw_minor_5 = hw_minor_5;
    packet.sw_major_5 = sw_major_5;
    packet.sw_minor_5 = sw_minor_5;
    packet.minutes_5 = minutes_5;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_DEVICE_INFO2_ARRAY_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_DEVICE_INFO2_ARRAY;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_DEVICE_INFO2_ARRAY_MIN_LEN, MAVLINK_MSG_ID_DEVICE_INFO2_ARRAY_LEN, MAVLINK_MSG_ID_DEVICE_INFO2_ARRAY_CRC);
}

/**
 * @brief Pack a device_info2_array message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param type_3  Device 3 type identifier (A-E)
 * @param year_3  Device 3 manufacturing year
 * @param month_3  Device 3 manufacturing month (1-12)
 * @param day_3  Device 3 manufacturing day (1-31)
 * @param number_3  Device 3 serial number
 * @param hw_major_3  Device 3 hardware major version
 * @param hw_minor_3  Device 3 hardware minor version
 * @param sw_major_3  Device 3 software major version
 * @param sw_minor_3  Device 3 software minor version
 * @param minutes_3  Device 3 runtime minutes (0-59)
 * @param hours_3  Device 3 total runtime hours
 * @param pwm_lost_count_3  Device 3 PWM signal loss count
 * @param type_4  Device 4 type identifier (A-E)
 * @param year_4  Device 4 manufacturing year
 * @param month_4  Device 4 manufacturing month (1-12)
 * @param day_4  Device 4 manufacturing day (1-31)
 * @param number_4  Device 4 serial number
 * @param hw_major_4  Device 4 hardware major version
 * @param hw_minor_4  Device 4 hardware minor version
 * @param sw_major_4  Device 4 software major version
 * @param sw_minor_4  Device 4 software minor version
 * @param minutes_4  Device 4 runtime minutes (0-59)
 * @param hours_4  Device 4 total runtime hours
 * @param pwm_lost_count_4  Device 4 PWM signal loss count
 * @param type_5  Device 5 type identifier (A-E)
 * @param year_5  Device 5 manufacturing year
 * @param month_5  Device 5 manufacturing month (1-12)
 * @param day_5  Device 5 manufacturing day (1-31)
 * @param number_5  Device 5 serial number
 * @param hw_major_5  Device 5 hardware major version
 * @param hw_minor_5  Device 5 hardware minor version
 * @param sw_major_5  Device 5 software major version
 * @param sw_minor_5  Device 5 software minor version
 * @param minutes_5  Device 5 runtime minutes (0-59)
 * @param hours_5  Device 5 total runtime hours
 * @param pwm_lost_count_5  Device 5 PWM signal loss count
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_device_info2_array_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint8_t type_3, uint8_t year_3, uint8_t month_3, uint8_t day_3, uint8_t number_3, uint8_t hw_major_3, uint8_t hw_minor_3, uint8_t sw_major_3, uint8_t sw_minor_3, uint8_t minutes_3, uint16_t hours_3, uint16_t pwm_lost_count_3, uint8_t type_4, uint8_t year_4, uint8_t month_4, uint8_t day_4, uint8_t number_4, uint8_t hw_major_4, uint8_t hw_minor_4, uint8_t sw_major_4, uint8_t sw_minor_4, uint8_t minutes_4, uint16_t hours_4, uint16_t pwm_lost_count_4, uint8_t type_5, uint8_t year_5, uint8_t month_5, uint8_t day_5, uint8_t number_5, uint8_t hw_major_5, uint8_t hw_minor_5, uint8_t sw_major_5, uint8_t sw_minor_5, uint8_t minutes_5, uint16_t hours_5, uint16_t pwm_lost_count_5)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_DEVICE_INFO2_ARRAY_LEN];
    _mav_put_uint16_t(buf, 0, hours_3);
    _mav_put_uint16_t(buf, 2, pwm_lost_count_3);
    _mav_put_uint16_t(buf, 4, hours_4);
    _mav_put_uint16_t(buf, 6, pwm_lost_count_4);
    _mav_put_uint16_t(buf, 8, hours_5);
    _mav_put_uint16_t(buf, 10, pwm_lost_count_5);
    _mav_put_uint8_t(buf, 12, type_3);
    _mav_put_uint8_t(buf, 13, year_3);
    _mav_put_uint8_t(buf, 14, month_3);
    _mav_put_uint8_t(buf, 15, day_3);
    _mav_put_uint8_t(buf, 16, number_3);
    _mav_put_uint8_t(buf, 17, hw_major_3);
    _mav_put_uint8_t(buf, 18, hw_minor_3);
    _mav_put_uint8_t(buf, 19, sw_major_3);
    _mav_put_uint8_t(buf, 20, sw_minor_3);
    _mav_put_uint8_t(buf, 21, minutes_3);
    _mav_put_uint8_t(buf, 22, type_4);
    _mav_put_uint8_t(buf, 23, year_4);
    _mav_put_uint8_t(buf, 24, month_4);
    _mav_put_uint8_t(buf, 25, day_4);
    _mav_put_uint8_t(buf, 26, number_4);
    _mav_put_uint8_t(buf, 27, hw_major_4);
    _mav_put_uint8_t(buf, 28, hw_minor_4);
    _mav_put_uint8_t(buf, 29, sw_major_4);
    _mav_put_uint8_t(buf, 30, sw_minor_4);
    _mav_put_uint8_t(buf, 31, minutes_4);
    _mav_put_uint8_t(buf, 32, type_5);
    _mav_put_uint8_t(buf, 33, year_5);
    _mav_put_uint8_t(buf, 34, month_5);
    _mav_put_uint8_t(buf, 35, day_5);
    _mav_put_uint8_t(buf, 36, number_5);
    _mav_put_uint8_t(buf, 37, hw_major_5);
    _mav_put_uint8_t(buf, 38, hw_minor_5);
    _mav_put_uint8_t(buf, 39, sw_major_5);
    _mav_put_uint8_t(buf, 40, sw_minor_5);
    _mav_put_uint8_t(buf, 41, minutes_5);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_DEVICE_INFO2_ARRAY_LEN);
#else
    mavlink_device_info2_array_t packet;
    packet.hours_3 = hours_3;
    packet.pwm_lost_count_3 = pwm_lost_count_3;
    packet.hours_4 = hours_4;
    packet.pwm_lost_count_4 = pwm_lost_count_4;
    packet.hours_5 = hours_5;
    packet.pwm_lost_count_5 = pwm_lost_count_5;
    packet.type_3 = type_3;
    packet.year_3 = year_3;
    packet.month_3 = month_3;
    packet.day_3 = day_3;
    packet.number_3 = number_3;
    packet.hw_major_3 = hw_major_3;
    packet.hw_minor_3 = hw_minor_3;
    packet.sw_major_3 = sw_major_3;
    packet.sw_minor_3 = sw_minor_3;
    packet.minutes_3 = minutes_3;
    packet.type_4 = type_4;
    packet.year_4 = year_4;
    packet.month_4 = month_4;
    packet.day_4 = day_4;
    packet.number_4 = number_4;
    packet.hw_major_4 = hw_major_4;
    packet.hw_minor_4 = hw_minor_4;
    packet.sw_major_4 = sw_major_4;
    packet.sw_minor_4 = sw_minor_4;
    packet.minutes_4 = minutes_4;
    packet.type_5 = type_5;
    packet.year_5 = year_5;
    packet.month_5 = month_5;
    packet.day_5 = day_5;
    packet.number_5 = number_5;
    packet.hw_major_5 = hw_major_5;
    packet.hw_minor_5 = hw_minor_5;
    packet.sw_major_5 = sw_major_5;
    packet.sw_minor_5 = sw_minor_5;
    packet.minutes_5 = minutes_5;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_DEVICE_INFO2_ARRAY_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_DEVICE_INFO2_ARRAY;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_DEVICE_INFO2_ARRAY_MIN_LEN, MAVLINK_MSG_ID_DEVICE_INFO2_ARRAY_LEN, MAVLINK_MSG_ID_DEVICE_INFO2_ARRAY_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_DEVICE_INFO2_ARRAY_MIN_LEN, MAVLINK_MSG_ID_DEVICE_INFO2_ARRAY_LEN);
#endif
}

/**
 * @brief Pack a device_info2_array message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param type_3  Device 3 type identifier (A-E)
 * @param year_3  Device 3 manufacturing year
 * @param month_3  Device 3 manufacturing month (1-12)
 * @param day_3  Device 3 manufacturing day (1-31)
 * @param number_3  Device 3 serial number
 * @param hw_major_3  Device 3 hardware major version
 * @param hw_minor_3  Device 3 hardware minor version
 * @param sw_major_3  Device 3 software major version
 * @param sw_minor_3  Device 3 software minor version
 * @param minutes_3  Device 3 runtime minutes (0-59)
 * @param hours_3  Device 3 total runtime hours
 * @param pwm_lost_count_3  Device 3 PWM signal loss count
 * @param type_4  Device 4 type identifier (A-E)
 * @param year_4  Device 4 manufacturing year
 * @param month_4  Device 4 manufacturing month (1-12)
 * @param day_4  Device 4 manufacturing day (1-31)
 * @param number_4  Device 4 serial number
 * @param hw_major_4  Device 4 hardware major version
 * @param hw_minor_4  Device 4 hardware minor version
 * @param sw_major_4  Device 4 software major version
 * @param sw_minor_4  Device 4 software minor version
 * @param minutes_4  Device 4 runtime minutes (0-59)
 * @param hours_4  Device 4 total runtime hours
 * @param pwm_lost_count_4  Device 4 PWM signal loss count
 * @param type_5  Device 5 type identifier (A-E)
 * @param year_5  Device 5 manufacturing year
 * @param month_5  Device 5 manufacturing month (1-12)
 * @param day_5  Device 5 manufacturing day (1-31)
 * @param number_5  Device 5 serial number
 * @param hw_major_5  Device 5 hardware major version
 * @param hw_minor_5  Device 5 hardware minor version
 * @param sw_major_5  Device 5 software major version
 * @param sw_minor_5  Device 5 software minor version
 * @param minutes_5  Device 5 runtime minutes (0-59)
 * @param hours_5  Device 5 total runtime hours
 * @param pwm_lost_count_5  Device 5 PWM signal loss count
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_device_info2_array_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t type_3,uint8_t year_3,uint8_t month_3,uint8_t day_3,uint8_t number_3,uint8_t hw_major_3,uint8_t hw_minor_3,uint8_t sw_major_3,uint8_t sw_minor_3,uint8_t minutes_3,uint16_t hours_3,uint16_t pwm_lost_count_3,uint8_t type_4,uint8_t year_4,uint8_t month_4,uint8_t day_4,uint8_t number_4,uint8_t hw_major_4,uint8_t hw_minor_4,uint8_t sw_major_4,uint8_t sw_minor_4,uint8_t minutes_4,uint16_t hours_4,uint16_t pwm_lost_count_4,uint8_t type_5,uint8_t year_5,uint8_t month_5,uint8_t day_5,uint8_t number_5,uint8_t hw_major_5,uint8_t hw_minor_5,uint8_t sw_major_5,uint8_t sw_minor_5,uint8_t minutes_5,uint16_t hours_5,uint16_t pwm_lost_count_5)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_DEVICE_INFO2_ARRAY_LEN];
    _mav_put_uint16_t(buf, 0, hours_3);
    _mav_put_uint16_t(buf, 2, pwm_lost_count_3);
    _mav_put_uint16_t(buf, 4, hours_4);
    _mav_put_uint16_t(buf, 6, pwm_lost_count_4);
    _mav_put_uint16_t(buf, 8, hours_5);
    _mav_put_uint16_t(buf, 10, pwm_lost_count_5);
    _mav_put_uint8_t(buf, 12, type_3);
    _mav_put_uint8_t(buf, 13, year_3);
    _mav_put_uint8_t(buf, 14, month_3);
    _mav_put_uint8_t(buf, 15, day_3);
    _mav_put_uint8_t(buf, 16, number_3);
    _mav_put_uint8_t(buf, 17, hw_major_3);
    _mav_put_uint8_t(buf, 18, hw_minor_3);
    _mav_put_uint8_t(buf, 19, sw_major_3);
    _mav_put_uint8_t(buf, 20, sw_minor_3);
    _mav_put_uint8_t(buf, 21, minutes_3);
    _mav_put_uint8_t(buf, 22, type_4);
    _mav_put_uint8_t(buf, 23, year_4);
    _mav_put_uint8_t(buf, 24, month_4);
    _mav_put_uint8_t(buf, 25, day_4);
    _mav_put_uint8_t(buf, 26, number_4);
    _mav_put_uint8_t(buf, 27, hw_major_4);
    _mav_put_uint8_t(buf, 28, hw_minor_4);
    _mav_put_uint8_t(buf, 29, sw_major_4);
    _mav_put_uint8_t(buf, 30, sw_minor_4);
    _mav_put_uint8_t(buf, 31, minutes_4);
    _mav_put_uint8_t(buf, 32, type_5);
    _mav_put_uint8_t(buf, 33, year_5);
    _mav_put_uint8_t(buf, 34, month_5);
    _mav_put_uint8_t(buf, 35, day_5);
    _mav_put_uint8_t(buf, 36, number_5);
    _mav_put_uint8_t(buf, 37, hw_major_5);
    _mav_put_uint8_t(buf, 38, hw_minor_5);
    _mav_put_uint8_t(buf, 39, sw_major_5);
    _mav_put_uint8_t(buf, 40, sw_minor_5);
    _mav_put_uint8_t(buf, 41, minutes_5);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_DEVICE_INFO2_ARRAY_LEN);
#else
    mavlink_device_info2_array_t packet;
    packet.hours_3 = hours_3;
    packet.pwm_lost_count_3 = pwm_lost_count_3;
    packet.hours_4 = hours_4;
    packet.pwm_lost_count_4 = pwm_lost_count_4;
    packet.hours_5 = hours_5;
    packet.pwm_lost_count_5 = pwm_lost_count_5;
    packet.type_3 = type_3;
    packet.year_3 = year_3;
    packet.month_3 = month_3;
    packet.day_3 = day_3;
    packet.number_3 = number_3;
    packet.hw_major_3 = hw_major_3;
    packet.hw_minor_3 = hw_minor_3;
    packet.sw_major_3 = sw_major_3;
    packet.sw_minor_3 = sw_minor_3;
    packet.minutes_3 = minutes_3;
    packet.type_4 = type_4;
    packet.year_4 = year_4;
    packet.month_4 = month_4;
    packet.day_4 = day_4;
    packet.number_4 = number_4;
    packet.hw_major_4 = hw_major_4;
    packet.hw_minor_4 = hw_minor_4;
    packet.sw_major_4 = sw_major_4;
    packet.sw_minor_4 = sw_minor_4;
    packet.minutes_4 = minutes_4;
    packet.type_5 = type_5;
    packet.year_5 = year_5;
    packet.month_5 = month_5;
    packet.day_5 = day_5;
    packet.number_5 = number_5;
    packet.hw_major_5 = hw_major_5;
    packet.hw_minor_5 = hw_minor_5;
    packet.sw_major_5 = sw_major_5;
    packet.sw_minor_5 = sw_minor_5;
    packet.minutes_5 = minutes_5;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_DEVICE_INFO2_ARRAY_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_DEVICE_INFO2_ARRAY;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_DEVICE_INFO2_ARRAY_MIN_LEN, MAVLINK_MSG_ID_DEVICE_INFO2_ARRAY_LEN, MAVLINK_MSG_ID_DEVICE_INFO2_ARRAY_CRC);
}

/**
 * @brief Encode a device_info2_array struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param device_info2_array C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_device_info2_array_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_device_info2_array_t* device_info2_array)
{
    return mavlink_msg_device_info2_array_pack(system_id, component_id, msg, device_info2_array->type_3, device_info2_array->year_3, device_info2_array->month_3, device_info2_array->day_3, device_info2_array->number_3, device_info2_array->hw_major_3, device_info2_array->hw_minor_3, device_info2_array->sw_major_3, device_info2_array->sw_minor_3, device_info2_array->minutes_3, device_info2_array->hours_3, device_info2_array->pwm_lost_count_3, device_info2_array->type_4, device_info2_array->year_4, device_info2_array->month_4, device_info2_array->day_4, device_info2_array->number_4, device_info2_array->hw_major_4, device_info2_array->hw_minor_4, device_info2_array->sw_major_4, device_info2_array->sw_minor_4, device_info2_array->minutes_4, device_info2_array->hours_4, device_info2_array->pwm_lost_count_4, device_info2_array->type_5, device_info2_array->year_5, device_info2_array->month_5, device_info2_array->day_5, device_info2_array->number_5, device_info2_array->hw_major_5, device_info2_array->hw_minor_5, device_info2_array->sw_major_5, device_info2_array->sw_minor_5, device_info2_array->minutes_5, device_info2_array->hours_5, device_info2_array->pwm_lost_count_5);
}

/**
 * @brief Encode a device_info2_array struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param device_info2_array C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_device_info2_array_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_device_info2_array_t* device_info2_array)
{
    return mavlink_msg_device_info2_array_pack_chan(system_id, component_id, chan, msg, device_info2_array->type_3, device_info2_array->year_3, device_info2_array->month_3, device_info2_array->day_3, device_info2_array->number_3, device_info2_array->hw_major_3, device_info2_array->hw_minor_3, device_info2_array->sw_major_3, device_info2_array->sw_minor_3, device_info2_array->minutes_3, device_info2_array->hours_3, device_info2_array->pwm_lost_count_3, device_info2_array->type_4, device_info2_array->year_4, device_info2_array->month_4, device_info2_array->day_4, device_info2_array->number_4, device_info2_array->hw_major_4, device_info2_array->hw_minor_4, device_info2_array->sw_major_4, device_info2_array->sw_minor_4, device_info2_array->minutes_4, device_info2_array->hours_4, device_info2_array->pwm_lost_count_4, device_info2_array->type_5, device_info2_array->year_5, device_info2_array->month_5, device_info2_array->day_5, device_info2_array->number_5, device_info2_array->hw_major_5, device_info2_array->hw_minor_5, device_info2_array->sw_major_5, device_info2_array->sw_minor_5, device_info2_array->minutes_5, device_info2_array->hours_5, device_info2_array->pwm_lost_count_5);
}

/**
 * @brief Encode a device_info2_array struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param device_info2_array C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_device_info2_array_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_device_info2_array_t* device_info2_array)
{
    return mavlink_msg_device_info2_array_pack_status(system_id, component_id, _status, msg,  device_info2_array->type_3, device_info2_array->year_3, device_info2_array->month_3, device_info2_array->day_3, device_info2_array->number_3, device_info2_array->hw_major_3, device_info2_array->hw_minor_3, device_info2_array->sw_major_3, device_info2_array->sw_minor_3, device_info2_array->minutes_3, device_info2_array->hours_3, device_info2_array->pwm_lost_count_3, device_info2_array->type_4, device_info2_array->year_4, device_info2_array->month_4, device_info2_array->day_4, device_info2_array->number_4, device_info2_array->hw_major_4, device_info2_array->hw_minor_4, device_info2_array->sw_major_4, device_info2_array->sw_minor_4, device_info2_array->minutes_4, device_info2_array->hours_4, device_info2_array->pwm_lost_count_4, device_info2_array->type_5, device_info2_array->year_5, device_info2_array->month_5, device_info2_array->day_5, device_info2_array->number_5, device_info2_array->hw_major_5, device_info2_array->hw_minor_5, device_info2_array->sw_major_5, device_info2_array->sw_minor_5, device_info2_array->minutes_5, device_info2_array->hours_5, device_info2_array->pwm_lost_count_5);
}

/**
 * @brief Send a device_info2_array message
 * @param chan MAVLink channel to send the message
 *
 * @param type_3  Device 3 type identifier (A-E)
 * @param year_3  Device 3 manufacturing year
 * @param month_3  Device 3 manufacturing month (1-12)
 * @param day_3  Device 3 manufacturing day (1-31)
 * @param number_3  Device 3 serial number
 * @param hw_major_3  Device 3 hardware major version
 * @param hw_minor_3  Device 3 hardware minor version
 * @param sw_major_3  Device 3 software major version
 * @param sw_minor_3  Device 3 software minor version
 * @param minutes_3  Device 3 runtime minutes (0-59)
 * @param hours_3  Device 3 total runtime hours
 * @param pwm_lost_count_3  Device 3 PWM signal loss count
 * @param type_4  Device 4 type identifier (A-E)
 * @param year_4  Device 4 manufacturing year
 * @param month_4  Device 4 manufacturing month (1-12)
 * @param day_4  Device 4 manufacturing day (1-31)
 * @param number_4  Device 4 serial number
 * @param hw_major_4  Device 4 hardware major version
 * @param hw_minor_4  Device 4 hardware minor version
 * @param sw_major_4  Device 4 software major version
 * @param sw_minor_4  Device 4 software minor version
 * @param minutes_4  Device 4 runtime minutes (0-59)
 * @param hours_4  Device 4 total runtime hours
 * @param pwm_lost_count_4  Device 4 PWM signal loss count
 * @param type_5  Device 5 type identifier (A-E)
 * @param year_5  Device 5 manufacturing year
 * @param month_5  Device 5 manufacturing month (1-12)
 * @param day_5  Device 5 manufacturing day (1-31)
 * @param number_5  Device 5 serial number
 * @param hw_major_5  Device 5 hardware major version
 * @param hw_minor_5  Device 5 hardware minor version
 * @param sw_major_5  Device 5 software major version
 * @param sw_minor_5  Device 5 software minor version
 * @param minutes_5  Device 5 runtime minutes (0-59)
 * @param hours_5  Device 5 total runtime hours
 * @param pwm_lost_count_5  Device 5 PWM signal loss count
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_device_info2_array_send(mavlink_channel_t chan, uint8_t type_3, uint8_t year_3, uint8_t month_3, uint8_t day_3, uint8_t number_3, uint8_t hw_major_3, uint8_t hw_minor_3, uint8_t sw_major_3, uint8_t sw_minor_3, uint8_t minutes_3, uint16_t hours_3, uint16_t pwm_lost_count_3, uint8_t type_4, uint8_t year_4, uint8_t month_4, uint8_t day_4, uint8_t number_4, uint8_t hw_major_4, uint8_t hw_minor_4, uint8_t sw_major_4, uint8_t sw_minor_4, uint8_t minutes_4, uint16_t hours_4, uint16_t pwm_lost_count_4, uint8_t type_5, uint8_t year_5, uint8_t month_5, uint8_t day_5, uint8_t number_5, uint8_t hw_major_5, uint8_t hw_minor_5, uint8_t sw_major_5, uint8_t sw_minor_5, uint8_t minutes_5, uint16_t hours_5, uint16_t pwm_lost_count_5)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_DEVICE_INFO2_ARRAY_LEN];
    _mav_put_uint16_t(buf, 0, hours_3);
    _mav_put_uint16_t(buf, 2, pwm_lost_count_3);
    _mav_put_uint16_t(buf, 4, hours_4);
    _mav_put_uint16_t(buf, 6, pwm_lost_count_4);
    _mav_put_uint16_t(buf, 8, hours_5);
    _mav_put_uint16_t(buf, 10, pwm_lost_count_5);
    _mav_put_uint8_t(buf, 12, type_3);
    _mav_put_uint8_t(buf, 13, year_3);
    _mav_put_uint8_t(buf, 14, month_3);
    _mav_put_uint8_t(buf, 15, day_3);
    _mav_put_uint8_t(buf, 16, number_3);
    _mav_put_uint8_t(buf, 17, hw_major_3);
    _mav_put_uint8_t(buf, 18, hw_minor_3);
    _mav_put_uint8_t(buf, 19, sw_major_3);
    _mav_put_uint8_t(buf, 20, sw_minor_3);
    _mav_put_uint8_t(buf, 21, minutes_3);
    _mav_put_uint8_t(buf, 22, type_4);
    _mav_put_uint8_t(buf, 23, year_4);
    _mav_put_uint8_t(buf, 24, month_4);
    _mav_put_uint8_t(buf, 25, day_4);
    _mav_put_uint8_t(buf, 26, number_4);
    _mav_put_uint8_t(buf, 27, hw_major_4);
    _mav_put_uint8_t(buf, 28, hw_minor_4);
    _mav_put_uint8_t(buf, 29, sw_major_4);
    _mav_put_uint8_t(buf, 30, sw_minor_4);
    _mav_put_uint8_t(buf, 31, minutes_4);
    _mav_put_uint8_t(buf, 32, type_5);
    _mav_put_uint8_t(buf, 33, year_5);
    _mav_put_uint8_t(buf, 34, month_5);
    _mav_put_uint8_t(buf, 35, day_5);
    _mav_put_uint8_t(buf, 36, number_5);
    _mav_put_uint8_t(buf, 37, hw_major_5);
    _mav_put_uint8_t(buf, 38, hw_minor_5);
    _mav_put_uint8_t(buf, 39, sw_major_5);
    _mav_put_uint8_t(buf, 40, sw_minor_5);
    _mav_put_uint8_t(buf, 41, minutes_5);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DEVICE_INFO2_ARRAY, buf, MAVLINK_MSG_ID_DEVICE_INFO2_ARRAY_MIN_LEN, MAVLINK_MSG_ID_DEVICE_INFO2_ARRAY_LEN, MAVLINK_MSG_ID_DEVICE_INFO2_ARRAY_CRC);
#else
    mavlink_device_info2_array_t packet;
    packet.hours_3 = hours_3;
    packet.pwm_lost_count_3 = pwm_lost_count_3;
    packet.hours_4 = hours_4;
    packet.pwm_lost_count_4 = pwm_lost_count_4;
    packet.hours_5 = hours_5;
    packet.pwm_lost_count_5 = pwm_lost_count_5;
    packet.type_3 = type_3;
    packet.year_3 = year_3;
    packet.month_3 = month_3;
    packet.day_3 = day_3;
    packet.number_3 = number_3;
    packet.hw_major_3 = hw_major_3;
    packet.hw_minor_3 = hw_minor_3;
    packet.sw_major_3 = sw_major_3;
    packet.sw_minor_3 = sw_minor_3;
    packet.minutes_3 = minutes_3;
    packet.type_4 = type_4;
    packet.year_4 = year_4;
    packet.month_4 = month_4;
    packet.day_4 = day_4;
    packet.number_4 = number_4;
    packet.hw_major_4 = hw_major_4;
    packet.hw_minor_4 = hw_minor_4;
    packet.sw_major_4 = sw_major_4;
    packet.sw_minor_4 = sw_minor_4;
    packet.minutes_4 = minutes_4;
    packet.type_5 = type_5;
    packet.year_5 = year_5;
    packet.month_5 = month_5;
    packet.day_5 = day_5;
    packet.number_5 = number_5;
    packet.hw_major_5 = hw_major_5;
    packet.hw_minor_5 = hw_minor_5;
    packet.sw_major_5 = sw_major_5;
    packet.sw_minor_5 = sw_minor_5;
    packet.minutes_5 = minutes_5;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DEVICE_INFO2_ARRAY, (const char *)&packet, MAVLINK_MSG_ID_DEVICE_INFO2_ARRAY_MIN_LEN, MAVLINK_MSG_ID_DEVICE_INFO2_ARRAY_LEN, MAVLINK_MSG_ID_DEVICE_INFO2_ARRAY_CRC);
#endif
}

/**
 * @brief Send a device_info2_array message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_device_info2_array_send_struct(mavlink_channel_t chan, const mavlink_device_info2_array_t* device_info2_array)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_device_info2_array_send(chan, device_info2_array->type_3, device_info2_array->year_3, device_info2_array->month_3, device_info2_array->day_3, device_info2_array->number_3, device_info2_array->hw_major_3, device_info2_array->hw_minor_3, device_info2_array->sw_major_3, device_info2_array->sw_minor_3, device_info2_array->minutes_3, device_info2_array->hours_3, device_info2_array->pwm_lost_count_3, device_info2_array->type_4, device_info2_array->year_4, device_info2_array->month_4, device_info2_array->day_4, device_info2_array->number_4, device_info2_array->hw_major_4, device_info2_array->hw_minor_4, device_info2_array->sw_major_4, device_info2_array->sw_minor_4, device_info2_array->minutes_4, device_info2_array->hours_4, device_info2_array->pwm_lost_count_4, device_info2_array->type_5, device_info2_array->year_5, device_info2_array->month_5, device_info2_array->day_5, device_info2_array->number_5, device_info2_array->hw_major_5, device_info2_array->hw_minor_5, device_info2_array->sw_major_5, device_info2_array->sw_minor_5, device_info2_array->minutes_5, device_info2_array->hours_5, device_info2_array->pwm_lost_count_5);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DEVICE_INFO2_ARRAY, (const char *)device_info2_array, MAVLINK_MSG_ID_DEVICE_INFO2_ARRAY_MIN_LEN, MAVLINK_MSG_ID_DEVICE_INFO2_ARRAY_LEN, MAVLINK_MSG_ID_DEVICE_INFO2_ARRAY_CRC);
#endif
}

#if MAVLINK_MSG_ID_DEVICE_INFO2_ARRAY_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by reusing
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_device_info2_array_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t type_3, uint8_t year_3, uint8_t month_3, uint8_t day_3, uint8_t number_3, uint8_t hw_major_3, uint8_t hw_minor_3, uint8_t sw_major_3, uint8_t sw_minor_3, uint8_t minutes_3, uint16_t hours_3, uint16_t pwm_lost_count_3, uint8_t type_4, uint8_t year_4, uint8_t month_4, uint8_t day_4, uint8_t number_4, uint8_t hw_major_4, uint8_t hw_minor_4, uint8_t sw_major_4, uint8_t sw_minor_4, uint8_t minutes_4, uint16_t hours_4, uint16_t pwm_lost_count_4, uint8_t type_5, uint8_t year_5, uint8_t month_5, uint8_t day_5, uint8_t number_5, uint8_t hw_major_5, uint8_t hw_minor_5, uint8_t sw_major_5, uint8_t sw_minor_5, uint8_t minutes_5, uint16_t hours_5, uint16_t pwm_lost_count_5)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint16_t(buf, 0, hours_3);
    _mav_put_uint16_t(buf, 2, pwm_lost_count_3);
    _mav_put_uint16_t(buf, 4, hours_4);
    _mav_put_uint16_t(buf, 6, pwm_lost_count_4);
    _mav_put_uint16_t(buf, 8, hours_5);
    _mav_put_uint16_t(buf, 10, pwm_lost_count_5);
    _mav_put_uint8_t(buf, 12, type_3);
    _mav_put_uint8_t(buf, 13, year_3);
    _mav_put_uint8_t(buf, 14, month_3);
    _mav_put_uint8_t(buf, 15, day_3);
    _mav_put_uint8_t(buf, 16, number_3);
    _mav_put_uint8_t(buf, 17, hw_major_3);
    _mav_put_uint8_t(buf, 18, hw_minor_3);
    _mav_put_uint8_t(buf, 19, sw_major_3);
    _mav_put_uint8_t(buf, 20, sw_minor_3);
    _mav_put_uint8_t(buf, 21, minutes_3);
    _mav_put_uint8_t(buf, 22, type_4);
    _mav_put_uint8_t(buf, 23, year_4);
    _mav_put_uint8_t(buf, 24, month_4);
    _mav_put_uint8_t(buf, 25, day_4);
    _mav_put_uint8_t(buf, 26, number_4);
    _mav_put_uint8_t(buf, 27, hw_major_4);
    _mav_put_uint8_t(buf, 28, hw_minor_4);
    _mav_put_uint8_t(buf, 29, sw_major_4);
    _mav_put_uint8_t(buf, 30, sw_minor_4);
    _mav_put_uint8_t(buf, 31, minutes_4);
    _mav_put_uint8_t(buf, 32, type_5);
    _mav_put_uint8_t(buf, 33, year_5);
    _mav_put_uint8_t(buf, 34, month_5);
    _mav_put_uint8_t(buf, 35, day_5);
    _mav_put_uint8_t(buf, 36, number_5);
    _mav_put_uint8_t(buf, 37, hw_major_5);
    _mav_put_uint8_t(buf, 38, hw_minor_5);
    _mav_put_uint8_t(buf, 39, sw_major_5);
    _mav_put_uint8_t(buf, 40, sw_minor_5);
    _mav_put_uint8_t(buf, 41, minutes_5);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DEVICE_INFO2_ARRAY, buf, MAVLINK_MSG_ID_DEVICE_INFO2_ARRAY_MIN_LEN, MAVLINK_MSG_ID_DEVICE_INFO2_ARRAY_LEN, MAVLINK_MSG_ID_DEVICE_INFO2_ARRAY_CRC);
#else
    mavlink_device_info2_array_t *packet = (mavlink_device_info2_array_t *)msgbuf;
    packet->hours_3 = hours_3;
    packet->pwm_lost_count_3 = pwm_lost_count_3;
    packet->hours_4 = hours_4;
    packet->pwm_lost_count_4 = pwm_lost_count_4;
    packet->hours_5 = hours_5;
    packet->pwm_lost_count_5 = pwm_lost_count_5;
    packet->type_3 = type_3;
    packet->year_3 = year_3;
    packet->month_3 = month_3;
    packet->day_3 = day_3;
    packet->number_3 = number_3;
    packet->hw_major_3 = hw_major_3;
    packet->hw_minor_3 = hw_minor_3;
    packet->sw_major_3 = sw_major_3;
    packet->sw_minor_3 = sw_minor_3;
    packet->minutes_3 = minutes_3;
    packet->type_4 = type_4;
    packet->year_4 = year_4;
    packet->month_4 = month_4;
    packet->day_4 = day_4;
    packet->number_4 = number_4;
    packet->hw_major_4 = hw_major_4;
    packet->hw_minor_4 = hw_minor_4;
    packet->sw_major_4 = sw_major_4;
    packet->sw_minor_4 = sw_minor_4;
    packet->minutes_4 = minutes_4;
    packet->type_5 = type_5;
    packet->year_5 = year_5;
    packet->month_5 = month_5;
    packet->day_5 = day_5;
    packet->number_5 = number_5;
    packet->hw_major_5 = hw_major_5;
    packet->hw_minor_5 = hw_minor_5;
    packet->sw_major_5 = sw_major_5;
    packet->sw_minor_5 = sw_minor_5;
    packet->minutes_5 = minutes_5;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DEVICE_INFO2_ARRAY, (const char *)packet, MAVLINK_MSG_ID_DEVICE_INFO2_ARRAY_MIN_LEN, MAVLINK_MSG_ID_DEVICE_INFO2_ARRAY_LEN, MAVLINK_MSG_ID_DEVICE_INFO2_ARRAY_CRC);
#endif
}
#endif

#endif

// MESSAGE DEVICE_INFO2_ARRAY UNPACKING


/**
 * @brief Get field type_3 from device_info2_array message
 *
 * @return  Device 3 type identifier (A-E)
 */
static inline uint8_t mavlink_msg_device_info2_array_get_type_3(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  12);
}

/**
 * @brief Get field year_3 from device_info2_array message
 *
 * @return  Device 3 manufacturing year
 */
static inline uint8_t mavlink_msg_device_info2_array_get_year_3(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  13);
}

/**
 * @brief Get field month_3 from device_info2_array message
 *
 * @return  Device 3 manufacturing month (1-12)
 */
static inline uint8_t mavlink_msg_device_info2_array_get_month_3(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  14);
}

/**
 * @brief Get field day_3 from device_info2_array message
 *
 * @return  Device 3 manufacturing day (1-31)
 */
static inline uint8_t mavlink_msg_device_info2_array_get_day_3(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  15);
}

/**
 * @brief Get field number_3 from device_info2_array message
 *
 * @return  Device 3 serial number
 */
static inline uint8_t mavlink_msg_device_info2_array_get_number_3(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  16);
}

/**
 * @brief Get field hw_major_3 from device_info2_array message
 *
 * @return  Device 3 hardware major version
 */
static inline uint8_t mavlink_msg_device_info2_array_get_hw_major_3(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  17);
}

/**
 * @brief Get field hw_minor_3 from device_info2_array message
 *
 * @return  Device 3 hardware minor version
 */
static inline uint8_t mavlink_msg_device_info2_array_get_hw_minor_3(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  18);
}

/**
 * @brief Get field sw_major_3 from device_info2_array message
 *
 * @return  Device 3 software major version
 */
static inline uint8_t mavlink_msg_device_info2_array_get_sw_major_3(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  19);
}

/**
 * @brief Get field sw_minor_3 from device_info2_array message
 *
 * @return  Device 3 software minor version
 */
static inline uint8_t mavlink_msg_device_info2_array_get_sw_minor_3(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  20);
}

/**
 * @brief Get field minutes_3 from device_info2_array message
 *
 * @return  Device 3 runtime minutes (0-59)
 */
static inline uint8_t mavlink_msg_device_info2_array_get_minutes_3(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  21);
}

/**
 * @brief Get field hours_3 from device_info2_array message
 *
 * @return  Device 3 total runtime hours
 */
static inline uint16_t mavlink_msg_device_info2_array_get_hours_3(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  0);
}

/**
 * @brief Get field pwm_lost_count_3 from device_info2_array message
 *
 * @return  Device 3 PWM signal loss count
 */
static inline uint16_t mavlink_msg_device_info2_array_get_pwm_lost_count_3(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  2);
}

/**
 * @brief Get field type_4 from device_info2_array message
 *
 * @return  Device 4 type identifier (A-E)
 */
static inline uint8_t mavlink_msg_device_info2_array_get_type_4(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  22);
}

/**
 * @brief Get field year_4 from device_info2_array message
 *
 * @return  Device 4 manufacturing year
 */
static inline uint8_t mavlink_msg_device_info2_array_get_year_4(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  23);
}

/**
 * @brief Get field month_4 from device_info2_array message
 *
 * @return  Device 4 manufacturing month (1-12)
 */
static inline uint8_t mavlink_msg_device_info2_array_get_month_4(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  24);
}

/**
 * @brief Get field day_4 from device_info2_array message
 *
 * @return  Device 4 manufacturing day (1-31)
 */
static inline uint8_t mavlink_msg_device_info2_array_get_day_4(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  25);
}

/**
 * @brief Get field number_4 from device_info2_array message
 *
 * @return  Device 4 serial number
 */
static inline uint8_t mavlink_msg_device_info2_array_get_number_4(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  26);
}

/**
 * @brief Get field hw_major_4 from device_info2_array message
 *
 * @return  Device 4 hardware major version
 */
static inline uint8_t mavlink_msg_device_info2_array_get_hw_major_4(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  27);
}

/**
 * @brief Get field hw_minor_4 from device_info2_array message
 *
 * @return  Device 4 hardware minor version
 */
static inline uint8_t mavlink_msg_device_info2_array_get_hw_minor_4(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  28);
}

/**
 * @brief Get field sw_major_4 from device_info2_array message
 *
 * @return  Device 4 software major version
 */
static inline uint8_t mavlink_msg_device_info2_array_get_sw_major_4(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  29);
}

/**
 * @brief Get field sw_minor_4 from device_info2_array message
 *
 * @return  Device 4 software minor version
 */
static inline uint8_t mavlink_msg_device_info2_array_get_sw_minor_4(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  30);
}

/**
 * @brief Get field minutes_4 from device_info2_array message
 *
 * @return  Device 4 runtime minutes (0-59)
 */
static inline uint8_t mavlink_msg_device_info2_array_get_minutes_4(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  31);
}

/**
 * @brief Get field hours_4 from device_info2_array message
 *
 * @return  Device 4 total runtime hours
 */
static inline uint16_t mavlink_msg_device_info2_array_get_hours_4(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  4);
}

/**
 * @brief Get field pwm_lost_count_4 from device_info2_array message
 *
 * @return  Device 4 PWM signal loss count
 */
static inline uint16_t mavlink_msg_device_info2_array_get_pwm_lost_count_4(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  6);
}

/**
 * @brief Get field type_5 from device_info2_array message
 *
 * @return  Device 5 type identifier (A-E)
 */
static inline uint8_t mavlink_msg_device_info2_array_get_type_5(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  32);
}

/**
 * @brief Get field year_5 from device_info2_array message
 *
 * @return  Device 5 manufacturing year
 */
static inline uint8_t mavlink_msg_device_info2_array_get_year_5(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  33);
}

/**
 * @brief Get field month_5 from device_info2_array message
 *
 * @return  Device 5 manufacturing month (1-12)
 */
static inline uint8_t mavlink_msg_device_info2_array_get_month_5(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  34);
}

/**
 * @brief Get field day_5 from device_info2_array message
 *
 * @return  Device 5 manufacturing day (1-31)
 */
static inline uint8_t mavlink_msg_device_info2_array_get_day_5(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  35);
}

/**
 * @brief Get field number_5 from device_info2_array message
 *
 * @return  Device 5 serial number
 */
static inline uint8_t mavlink_msg_device_info2_array_get_number_5(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  36);
}

/**
 * @brief Get field hw_major_5 from device_info2_array message
 *
 * @return  Device 5 hardware major version
 */
static inline uint8_t mavlink_msg_device_info2_array_get_hw_major_5(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  37);
}

/**
 * @brief Get field hw_minor_5 from device_info2_array message
 *
 * @return  Device 5 hardware minor version
 */
static inline uint8_t mavlink_msg_device_info2_array_get_hw_minor_5(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  38);
}

/**
 * @brief Get field sw_major_5 from device_info2_array message
 *
 * @return  Device 5 software major version
 */
static inline uint8_t mavlink_msg_device_info2_array_get_sw_major_5(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  39);
}

/**
 * @brief Get field sw_minor_5 from device_info2_array message
 *
 * @return  Device 5 software minor version
 */
static inline uint8_t mavlink_msg_device_info2_array_get_sw_minor_5(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  40);
}

/**
 * @brief Get field minutes_5 from device_info2_array message
 *
 * @return  Device 5 runtime minutes (0-59)
 */
static inline uint8_t mavlink_msg_device_info2_array_get_minutes_5(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  41);
}

/**
 * @brief Get field hours_5 from device_info2_array message
 *
 * @return  Device 5 total runtime hours
 */
static inline uint16_t mavlink_msg_device_info2_array_get_hours_5(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  8);
}

/**
 * @brief Get field pwm_lost_count_5 from device_info2_array message
 *
 * @return  Device 5 PWM signal loss count
 */
static inline uint16_t mavlink_msg_device_info2_array_get_pwm_lost_count_5(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  10);
}

/**
 * @brief Decode a device_info2_array message into a struct
 *
 * @param msg The message to decode
 * @param device_info2_array C-struct to decode the message contents into
 */
static inline void mavlink_msg_device_info2_array_decode(const mavlink_message_t* msg, mavlink_device_info2_array_t* device_info2_array)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    device_info2_array->hours_3 = mavlink_msg_device_info2_array_get_hours_3(msg);
    device_info2_array->pwm_lost_count_3 = mavlink_msg_device_info2_array_get_pwm_lost_count_3(msg);
    device_info2_array->hours_4 = mavlink_msg_device_info2_array_get_hours_4(msg);
    device_info2_array->pwm_lost_count_4 = mavlink_msg_device_info2_array_get_pwm_lost_count_4(msg);
    device_info2_array->hours_5 = mavlink_msg_device_info2_array_get_hours_5(msg);
    device_info2_array->pwm_lost_count_5 = mavlink_msg_device_info2_array_get_pwm_lost_count_5(msg);
    device_info2_array->type_3 = mavlink_msg_device_info2_array_get_type_3(msg);
    device_info2_array->year_3 = mavlink_msg_device_info2_array_get_year_3(msg);
    device_info2_array->month_3 = mavlink_msg_device_info2_array_get_month_3(msg);
    device_info2_array->day_3 = mavlink_msg_device_info2_array_get_day_3(msg);
    device_info2_array->number_3 = mavlink_msg_device_info2_array_get_number_3(msg);
    device_info2_array->hw_major_3 = mavlink_msg_device_info2_array_get_hw_major_3(msg);
    device_info2_array->hw_minor_3 = mavlink_msg_device_info2_array_get_hw_minor_3(msg);
    device_info2_array->sw_major_3 = mavlink_msg_device_info2_array_get_sw_major_3(msg);
    device_info2_array->sw_minor_3 = mavlink_msg_device_info2_array_get_sw_minor_3(msg);
    device_info2_array->minutes_3 = mavlink_msg_device_info2_array_get_minutes_3(msg);
    device_info2_array->type_4 = mavlink_msg_device_info2_array_get_type_4(msg);
    device_info2_array->year_4 = mavlink_msg_device_info2_array_get_year_4(msg);
    device_info2_array->month_4 = mavlink_msg_device_info2_array_get_month_4(msg);
    device_info2_array->day_4 = mavlink_msg_device_info2_array_get_day_4(msg);
    device_info2_array->number_4 = mavlink_msg_device_info2_array_get_number_4(msg);
    device_info2_array->hw_major_4 = mavlink_msg_device_info2_array_get_hw_major_4(msg);
    device_info2_array->hw_minor_4 = mavlink_msg_device_info2_array_get_hw_minor_4(msg);
    device_info2_array->sw_major_4 = mavlink_msg_device_info2_array_get_sw_major_4(msg);
    device_info2_array->sw_minor_4 = mavlink_msg_device_info2_array_get_sw_minor_4(msg);
    device_info2_array->minutes_4 = mavlink_msg_device_info2_array_get_minutes_4(msg);
    device_info2_array->type_5 = mavlink_msg_device_info2_array_get_type_5(msg);
    device_info2_array->year_5 = mavlink_msg_device_info2_array_get_year_5(msg);
    device_info2_array->month_5 = mavlink_msg_device_info2_array_get_month_5(msg);
    device_info2_array->day_5 = mavlink_msg_device_info2_array_get_day_5(msg);
    device_info2_array->number_5 = mavlink_msg_device_info2_array_get_number_5(msg);
    device_info2_array->hw_major_5 = mavlink_msg_device_info2_array_get_hw_major_5(msg);
    device_info2_array->hw_minor_5 = mavlink_msg_device_info2_array_get_hw_minor_5(msg);
    device_info2_array->sw_major_5 = mavlink_msg_device_info2_array_get_sw_major_5(msg);
    device_info2_array->sw_minor_5 = mavlink_msg_device_info2_array_get_sw_minor_5(msg);
    device_info2_array->minutes_5 = mavlink_msg_device_info2_array_get_minutes_5(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_DEVICE_INFO2_ARRAY_LEN? msg->len : MAVLINK_MSG_ID_DEVICE_INFO2_ARRAY_LEN;
        memset(device_info2_array, 0, MAVLINK_MSG_ID_DEVICE_INFO2_ARRAY_LEN);
    memcpy(device_info2_array, _MAV_PAYLOAD(msg), len);
#endif
}
