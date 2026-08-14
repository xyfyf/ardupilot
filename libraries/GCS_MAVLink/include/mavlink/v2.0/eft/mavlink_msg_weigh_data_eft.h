#pragma once
// MESSAGE WEIGH_DATA_EFT PACKING

#define MAVLINK_MSG_ID_WEIGH_DATA_EFT 506


typedef struct __mavlink_weigh_data_eft_t {
 uint16_t weight; /*<  Current weight value in grams*/
 uint16_t hours; /*<  Runtime hours*/
 uint16_t sensor1_k; /*<  Weight sensor 1 K value*/
 uint16_t sensor2_k; /*<  Weight sensor 2 K value*/
 uint16_t sensor3_k; /*<  Weight sensor 3 K value*/
 uint8_t liquid_level; /*<  Liquid level sensor status (0: No liquid, 1: Liquid detected)*/
 uint8_t sensor_status; /*<  Arm lock sensor status:- bit 0-3: Arm1-4 status (0: Locked, 1: Unlocked)- bit 4-7: Reserved*/
 uint8_t right_led_temp; /*<  Right LED temperature*/
 uint8_t left_led_temp; /*<  Left LED temperature*/
 uint8_t led_status; /*<  LED status (bit0-1: over temp, bit2-3: switch status)*/
 uint8_t battery_temp; /*<  Battery connector temperature*/
 uint8_t device_type; /*<  Device type (EB)*/
 uint8_t year; /*<  Manufacturing year*/
 uint8_t month; /*<  Manufacturing month*/
 uint8_t day; /*<  Manufacturing day*/
 uint8_t number; /*<  Device serial number*/
 uint8_t hw_major; /*<  Hardware major version*/
 uint8_t hw_minor; /*<  Hardware minor version*/
 uint8_t sw_major; /*<  Software major version*/
 uint8_t sw_minor; /*<  Software minor version*/
 uint8_t minutes; /*<  Runtime minutes*/
} mavlink_weigh_data_eft_t;

#define MAVLINK_MSG_ID_WEIGH_DATA_EFT_LEN 26
#define MAVLINK_MSG_ID_WEIGH_DATA_EFT_MIN_LEN 26
#define MAVLINK_MSG_ID_506_LEN 26
#define MAVLINK_MSG_ID_506_MIN_LEN 26

#define MAVLINK_MSG_ID_WEIGH_DATA_EFT_CRC 132
#define MAVLINK_MSG_ID_506_CRC 132



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_WEIGH_DATA_EFT { \
    506, \
    "WEIGH_DATA_EFT", \
    21, \
    {  { "liquid_level", NULL, MAVLINK_TYPE_UINT8_T, 0, 10, offsetof(mavlink_weigh_data_eft_t, liquid_level) }, \
         { "sensor_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 11, offsetof(mavlink_weigh_data_eft_t, sensor_status) }, \
         { "weight", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_weigh_data_eft_t, weight) }, \
         { "right_led_temp", NULL, MAVLINK_TYPE_UINT8_T, 0, 12, offsetof(mavlink_weigh_data_eft_t, right_led_temp) }, \
         { "left_led_temp", NULL, MAVLINK_TYPE_UINT8_T, 0, 13, offsetof(mavlink_weigh_data_eft_t, left_led_temp) }, \
         { "led_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 14, offsetof(mavlink_weigh_data_eft_t, led_status) }, \
         { "battery_temp", NULL, MAVLINK_TYPE_UINT8_T, 0, 15, offsetof(mavlink_weigh_data_eft_t, battery_temp) }, \
         { "device_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 16, offsetof(mavlink_weigh_data_eft_t, device_type) }, \
         { "year", NULL, MAVLINK_TYPE_UINT8_T, 0, 17, offsetof(mavlink_weigh_data_eft_t, year) }, \
         { "month", NULL, MAVLINK_TYPE_UINT8_T, 0, 18, offsetof(mavlink_weigh_data_eft_t, month) }, \
         { "day", NULL, MAVLINK_TYPE_UINT8_T, 0, 19, offsetof(mavlink_weigh_data_eft_t, day) }, \
         { "number", NULL, MAVLINK_TYPE_UINT8_T, 0, 20, offsetof(mavlink_weigh_data_eft_t, number) }, \
         { "hw_major", NULL, MAVLINK_TYPE_UINT8_T, 0, 21, offsetof(mavlink_weigh_data_eft_t, hw_major) }, \
         { "hw_minor", NULL, MAVLINK_TYPE_UINT8_T, 0, 22, offsetof(mavlink_weigh_data_eft_t, hw_minor) }, \
         { "sw_major", NULL, MAVLINK_TYPE_UINT8_T, 0, 23, offsetof(mavlink_weigh_data_eft_t, sw_major) }, \
         { "sw_minor", NULL, MAVLINK_TYPE_UINT8_T, 0, 24, offsetof(mavlink_weigh_data_eft_t, sw_minor) }, \
         { "minutes", NULL, MAVLINK_TYPE_UINT8_T, 0, 25, offsetof(mavlink_weigh_data_eft_t, minutes) }, \
         { "hours", NULL, MAVLINK_TYPE_UINT16_T, 0, 2, offsetof(mavlink_weigh_data_eft_t, hours) }, \
         { "sensor1_k", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_weigh_data_eft_t, sensor1_k) }, \
         { "sensor2_k", NULL, MAVLINK_TYPE_UINT16_T, 0, 6, offsetof(mavlink_weigh_data_eft_t, sensor2_k) }, \
         { "sensor3_k", NULL, MAVLINK_TYPE_UINT16_T, 0, 8, offsetof(mavlink_weigh_data_eft_t, sensor3_k) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_WEIGH_DATA_EFT { \
    "WEIGH_DATA_EFT", \
    21, \
    {  { "liquid_level", NULL, MAVLINK_TYPE_UINT8_T, 0, 10, offsetof(mavlink_weigh_data_eft_t, liquid_level) }, \
         { "sensor_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 11, offsetof(mavlink_weigh_data_eft_t, sensor_status) }, \
         { "weight", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_weigh_data_eft_t, weight) }, \
         { "right_led_temp", NULL, MAVLINK_TYPE_UINT8_T, 0, 12, offsetof(mavlink_weigh_data_eft_t, right_led_temp) }, \
         { "left_led_temp", NULL, MAVLINK_TYPE_UINT8_T, 0, 13, offsetof(mavlink_weigh_data_eft_t, left_led_temp) }, \
         { "led_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 14, offsetof(mavlink_weigh_data_eft_t, led_status) }, \
         { "battery_temp", NULL, MAVLINK_TYPE_UINT8_T, 0, 15, offsetof(mavlink_weigh_data_eft_t, battery_temp) }, \
         { "device_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 16, offsetof(mavlink_weigh_data_eft_t, device_type) }, \
         { "year", NULL, MAVLINK_TYPE_UINT8_T, 0, 17, offsetof(mavlink_weigh_data_eft_t, year) }, \
         { "month", NULL, MAVLINK_TYPE_UINT8_T, 0, 18, offsetof(mavlink_weigh_data_eft_t, month) }, \
         { "day", NULL, MAVLINK_TYPE_UINT8_T, 0, 19, offsetof(mavlink_weigh_data_eft_t, day) }, \
         { "number", NULL, MAVLINK_TYPE_UINT8_T, 0, 20, offsetof(mavlink_weigh_data_eft_t, number) }, \
         { "hw_major", NULL, MAVLINK_TYPE_UINT8_T, 0, 21, offsetof(mavlink_weigh_data_eft_t, hw_major) }, \
         { "hw_minor", NULL, MAVLINK_TYPE_UINT8_T, 0, 22, offsetof(mavlink_weigh_data_eft_t, hw_minor) }, \
         { "sw_major", NULL, MAVLINK_TYPE_UINT8_T, 0, 23, offsetof(mavlink_weigh_data_eft_t, sw_major) }, \
         { "sw_minor", NULL, MAVLINK_TYPE_UINT8_T, 0, 24, offsetof(mavlink_weigh_data_eft_t, sw_minor) }, \
         { "minutes", NULL, MAVLINK_TYPE_UINT8_T, 0, 25, offsetof(mavlink_weigh_data_eft_t, minutes) }, \
         { "hours", NULL, MAVLINK_TYPE_UINT16_T, 0, 2, offsetof(mavlink_weigh_data_eft_t, hours) }, \
         { "sensor1_k", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_weigh_data_eft_t, sensor1_k) }, \
         { "sensor2_k", NULL, MAVLINK_TYPE_UINT16_T, 0, 6, offsetof(mavlink_weigh_data_eft_t, sensor2_k) }, \
         { "sensor3_k", NULL, MAVLINK_TYPE_UINT16_T, 0, 8, offsetof(mavlink_weigh_data_eft_t, sensor3_k) }, \
         } \
}
#endif

/**
 * @brief Pack a weigh_data_eft message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param liquid_level  Liquid level sensor status (0: No liquid, 1: Liquid detected)
 * @param sensor_status  Arm lock sensor status:- bit 0-3: Arm1-4 status (0: Locked, 1: Unlocked)- bit 4-7: Reserved
 * @param weight  Current weight value in grams
 * @param right_led_temp  Right LED temperature
 * @param left_led_temp  Left LED temperature
 * @param led_status  LED status (bit0-1: over temp, bit2-3: switch status)
 * @param battery_temp  Battery connector temperature
 * @param device_type  Device type (EB)
 * @param year  Manufacturing year
 * @param month  Manufacturing month
 * @param day  Manufacturing day
 * @param number  Device serial number
 * @param hw_major  Hardware major version
 * @param hw_minor  Hardware minor version
 * @param sw_major  Software major version
 * @param sw_minor  Software minor version
 * @param minutes  Runtime minutes
 * @param hours  Runtime hours
 * @param sensor1_k  Weight sensor 1 K value
 * @param sensor2_k  Weight sensor 2 K value
 * @param sensor3_k  Weight sensor 3 K value
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_weigh_data_eft_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t liquid_level, uint8_t sensor_status, uint16_t weight, uint8_t right_led_temp, uint8_t left_led_temp, uint8_t led_status, uint8_t battery_temp, uint8_t device_type, uint8_t year, uint8_t month, uint8_t day, uint8_t number, uint8_t hw_major, uint8_t hw_minor, uint8_t sw_major, uint8_t sw_minor, uint8_t minutes, uint16_t hours, uint16_t sensor1_k, uint16_t sensor2_k, uint16_t sensor3_k)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_WEIGH_DATA_EFT_LEN];
    _mav_put_uint16_t(buf, 0, weight);
    _mav_put_uint16_t(buf, 2, hours);
    _mav_put_uint16_t(buf, 4, sensor1_k);
    _mav_put_uint16_t(buf, 6, sensor2_k);
    _mav_put_uint16_t(buf, 8, sensor3_k);
    _mav_put_uint8_t(buf, 10, liquid_level);
    _mav_put_uint8_t(buf, 11, sensor_status);
    _mav_put_uint8_t(buf, 12, right_led_temp);
    _mav_put_uint8_t(buf, 13, left_led_temp);
    _mav_put_uint8_t(buf, 14, led_status);
    _mav_put_uint8_t(buf, 15, battery_temp);
    _mav_put_uint8_t(buf, 16, device_type);
    _mav_put_uint8_t(buf, 17, year);
    _mav_put_uint8_t(buf, 18, month);
    _mav_put_uint8_t(buf, 19, day);
    _mav_put_uint8_t(buf, 20, number);
    _mav_put_uint8_t(buf, 21, hw_major);
    _mav_put_uint8_t(buf, 22, hw_minor);
    _mav_put_uint8_t(buf, 23, sw_major);
    _mav_put_uint8_t(buf, 24, sw_minor);
    _mav_put_uint8_t(buf, 25, minutes);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_WEIGH_DATA_EFT_LEN);
#else
    mavlink_weigh_data_eft_t packet;
    packet.weight = weight;
    packet.hours = hours;
    packet.sensor1_k = sensor1_k;
    packet.sensor2_k = sensor2_k;
    packet.sensor3_k = sensor3_k;
    packet.liquid_level = liquid_level;
    packet.sensor_status = sensor_status;
    packet.right_led_temp = right_led_temp;
    packet.left_led_temp = left_led_temp;
    packet.led_status = led_status;
    packet.battery_temp = battery_temp;
    packet.device_type = device_type;
    packet.year = year;
    packet.month = month;
    packet.day = day;
    packet.number = number;
    packet.hw_major = hw_major;
    packet.hw_minor = hw_minor;
    packet.sw_major = sw_major;
    packet.sw_minor = sw_minor;
    packet.minutes = minutes;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_WEIGH_DATA_EFT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_WEIGH_DATA_EFT;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_WEIGH_DATA_EFT_MIN_LEN, MAVLINK_MSG_ID_WEIGH_DATA_EFT_LEN, MAVLINK_MSG_ID_WEIGH_DATA_EFT_CRC);
}

/**
 * @brief Pack a weigh_data_eft message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param liquid_level  Liquid level sensor status (0: No liquid, 1: Liquid detected)
 * @param sensor_status  Arm lock sensor status:- bit 0-3: Arm1-4 status (0: Locked, 1: Unlocked)- bit 4-7: Reserved
 * @param weight  Current weight value in grams
 * @param right_led_temp  Right LED temperature
 * @param left_led_temp  Left LED temperature
 * @param led_status  LED status (bit0-1: over temp, bit2-3: switch status)
 * @param battery_temp  Battery connector temperature
 * @param device_type  Device type (EB)
 * @param year  Manufacturing year
 * @param month  Manufacturing month
 * @param day  Manufacturing day
 * @param number  Device serial number
 * @param hw_major  Hardware major version
 * @param hw_minor  Hardware minor version
 * @param sw_major  Software major version
 * @param sw_minor  Software minor version
 * @param minutes  Runtime minutes
 * @param hours  Runtime hours
 * @param sensor1_k  Weight sensor 1 K value
 * @param sensor2_k  Weight sensor 2 K value
 * @param sensor3_k  Weight sensor 3 K value
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_weigh_data_eft_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint8_t liquid_level, uint8_t sensor_status, uint16_t weight, uint8_t right_led_temp, uint8_t left_led_temp, uint8_t led_status, uint8_t battery_temp, uint8_t device_type, uint8_t year, uint8_t month, uint8_t day, uint8_t number, uint8_t hw_major, uint8_t hw_minor, uint8_t sw_major, uint8_t sw_minor, uint8_t minutes, uint16_t hours, uint16_t sensor1_k, uint16_t sensor2_k, uint16_t sensor3_k)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_WEIGH_DATA_EFT_LEN];
    _mav_put_uint16_t(buf, 0, weight);
    _mav_put_uint16_t(buf, 2, hours);
    _mav_put_uint16_t(buf, 4, sensor1_k);
    _mav_put_uint16_t(buf, 6, sensor2_k);
    _mav_put_uint16_t(buf, 8, sensor3_k);
    _mav_put_uint8_t(buf, 10, liquid_level);
    _mav_put_uint8_t(buf, 11, sensor_status);
    _mav_put_uint8_t(buf, 12, right_led_temp);
    _mav_put_uint8_t(buf, 13, left_led_temp);
    _mav_put_uint8_t(buf, 14, led_status);
    _mav_put_uint8_t(buf, 15, battery_temp);
    _mav_put_uint8_t(buf, 16, device_type);
    _mav_put_uint8_t(buf, 17, year);
    _mav_put_uint8_t(buf, 18, month);
    _mav_put_uint8_t(buf, 19, day);
    _mav_put_uint8_t(buf, 20, number);
    _mav_put_uint8_t(buf, 21, hw_major);
    _mav_put_uint8_t(buf, 22, hw_minor);
    _mav_put_uint8_t(buf, 23, sw_major);
    _mav_put_uint8_t(buf, 24, sw_minor);
    _mav_put_uint8_t(buf, 25, minutes);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_WEIGH_DATA_EFT_LEN);
#else
    mavlink_weigh_data_eft_t packet;
    packet.weight = weight;
    packet.hours = hours;
    packet.sensor1_k = sensor1_k;
    packet.sensor2_k = sensor2_k;
    packet.sensor3_k = sensor3_k;
    packet.liquid_level = liquid_level;
    packet.sensor_status = sensor_status;
    packet.right_led_temp = right_led_temp;
    packet.left_led_temp = left_led_temp;
    packet.led_status = led_status;
    packet.battery_temp = battery_temp;
    packet.device_type = device_type;
    packet.year = year;
    packet.month = month;
    packet.day = day;
    packet.number = number;
    packet.hw_major = hw_major;
    packet.hw_minor = hw_minor;
    packet.sw_major = sw_major;
    packet.sw_minor = sw_minor;
    packet.minutes = minutes;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_WEIGH_DATA_EFT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_WEIGH_DATA_EFT;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_WEIGH_DATA_EFT_MIN_LEN, MAVLINK_MSG_ID_WEIGH_DATA_EFT_LEN, MAVLINK_MSG_ID_WEIGH_DATA_EFT_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_WEIGH_DATA_EFT_MIN_LEN, MAVLINK_MSG_ID_WEIGH_DATA_EFT_LEN);
#endif
}

/**
 * @brief Pack a weigh_data_eft message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param liquid_level  Liquid level sensor status (0: No liquid, 1: Liquid detected)
 * @param sensor_status  Arm lock sensor status:- bit 0-3: Arm1-4 status (0: Locked, 1: Unlocked)- bit 4-7: Reserved
 * @param weight  Current weight value in grams
 * @param right_led_temp  Right LED temperature
 * @param left_led_temp  Left LED temperature
 * @param led_status  LED status (bit0-1: over temp, bit2-3: switch status)
 * @param battery_temp  Battery connector temperature
 * @param device_type  Device type (EB)
 * @param year  Manufacturing year
 * @param month  Manufacturing month
 * @param day  Manufacturing day
 * @param number  Device serial number
 * @param hw_major  Hardware major version
 * @param hw_minor  Hardware minor version
 * @param sw_major  Software major version
 * @param sw_minor  Software minor version
 * @param minutes  Runtime minutes
 * @param hours  Runtime hours
 * @param sensor1_k  Weight sensor 1 K value
 * @param sensor2_k  Weight sensor 2 K value
 * @param sensor3_k  Weight sensor 3 K value
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_weigh_data_eft_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t liquid_level,uint8_t sensor_status,uint16_t weight,uint8_t right_led_temp,uint8_t left_led_temp,uint8_t led_status,uint8_t battery_temp,uint8_t device_type,uint8_t year,uint8_t month,uint8_t day,uint8_t number,uint8_t hw_major,uint8_t hw_minor,uint8_t sw_major,uint8_t sw_minor,uint8_t minutes,uint16_t hours,uint16_t sensor1_k,uint16_t sensor2_k,uint16_t sensor3_k)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_WEIGH_DATA_EFT_LEN];
    _mav_put_uint16_t(buf, 0, weight);
    _mav_put_uint16_t(buf, 2, hours);
    _mav_put_uint16_t(buf, 4, sensor1_k);
    _mav_put_uint16_t(buf, 6, sensor2_k);
    _mav_put_uint16_t(buf, 8, sensor3_k);
    _mav_put_uint8_t(buf, 10, liquid_level);
    _mav_put_uint8_t(buf, 11, sensor_status);
    _mav_put_uint8_t(buf, 12, right_led_temp);
    _mav_put_uint8_t(buf, 13, left_led_temp);
    _mav_put_uint8_t(buf, 14, led_status);
    _mav_put_uint8_t(buf, 15, battery_temp);
    _mav_put_uint8_t(buf, 16, device_type);
    _mav_put_uint8_t(buf, 17, year);
    _mav_put_uint8_t(buf, 18, month);
    _mav_put_uint8_t(buf, 19, day);
    _mav_put_uint8_t(buf, 20, number);
    _mav_put_uint8_t(buf, 21, hw_major);
    _mav_put_uint8_t(buf, 22, hw_minor);
    _mav_put_uint8_t(buf, 23, sw_major);
    _mav_put_uint8_t(buf, 24, sw_minor);
    _mav_put_uint8_t(buf, 25, minutes);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_WEIGH_DATA_EFT_LEN);
#else
    mavlink_weigh_data_eft_t packet;
    packet.weight = weight;
    packet.hours = hours;
    packet.sensor1_k = sensor1_k;
    packet.sensor2_k = sensor2_k;
    packet.sensor3_k = sensor3_k;
    packet.liquid_level = liquid_level;
    packet.sensor_status = sensor_status;
    packet.right_led_temp = right_led_temp;
    packet.left_led_temp = left_led_temp;
    packet.led_status = led_status;
    packet.battery_temp = battery_temp;
    packet.device_type = device_type;
    packet.year = year;
    packet.month = month;
    packet.day = day;
    packet.number = number;
    packet.hw_major = hw_major;
    packet.hw_minor = hw_minor;
    packet.sw_major = sw_major;
    packet.sw_minor = sw_minor;
    packet.minutes = minutes;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_WEIGH_DATA_EFT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_WEIGH_DATA_EFT;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_WEIGH_DATA_EFT_MIN_LEN, MAVLINK_MSG_ID_WEIGH_DATA_EFT_LEN, MAVLINK_MSG_ID_WEIGH_DATA_EFT_CRC);
}

/**
 * @brief Encode a weigh_data_eft struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param weigh_data_eft C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_weigh_data_eft_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_weigh_data_eft_t* weigh_data_eft)
{
    return mavlink_msg_weigh_data_eft_pack(system_id, component_id, msg, weigh_data_eft->liquid_level, weigh_data_eft->sensor_status, weigh_data_eft->weight, weigh_data_eft->right_led_temp, weigh_data_eft->left_led_temp, weigh_data_eft->led_status, weigh_data_eft->battery_temp, weigh_data_eft->device_type, weigh_data_eft->year, weigh_data_eft->month, weigh_data_eft->day, weigh_data_eft->number, weigh_data_eft->hw_major, weigh_data_eft->hw_minor, weigh_data_eft->sw_major, weigh_data_eft->sw_minor, weigh_data_eft->minutes, weigh_data_eft->hours, weigh_data_eft->sensor1_k, weigh_data_eft->sensor2_k, weigh_data_eft->sensor3_k);
}

/**
 * @brief Encode a weigh_data_eft struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param weigh_data_eft C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_weigh_data_eft_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_weigh_data_eft_t* weigh_data_eft)
{
    return mavlink_msg_weigh_data_eft_pack_chan(system_id, component_id, chan, msg, weigh_data_eft->liquid_level, weigh_data_eft->sensor_status, weigh_data_eft->weight, weigh_data_eft->right_led_temp, weigh_data_eft->left_led_temp, weigh_data_eft->led_status, weigh_data_eft->battery_temp, weigh_data_eft->device_type, weigh_data_eft->year, weigh_data_eft->month, weigh_data_eft->day, weigh_data_eft->number, weigh_data_eft->hw_major, weigh_data_eft->hw_minor, weigh_data_eft->sw_major, weigh_data_eft->sw_minor, weigh_data_eft->minutes, weigh_data_eft->hours, weigh_data_eft->sensor1_k, weigh_data_eft->sensor2_k, weigh_data_eft->sensor3_k);
}

/**
 * @brief Encode a weigh_data_eft struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param weigh_data_eft C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_weigh_data_eft_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_weigh_data_eft_t* weigh_data_eft)
{
    return mavlink_msg_weigh_data_eft_pack_status(system_id, component_id, _status, msg,  weigh_data_eft->liquid_level, weigh_data_eft->sensor_status, weigh_data_eft->weight, weigh_data_eft->right_led_temp, weigh_data_eft->left_led_temp, weigh_data_eft->led_status, weigh_data_eft->battery_temp, weigh_data_eft->device_type, weigh_data_eft->year, weigh_data_eft->month, weigh_data_eft->day, weigh_data_eft->number, weigh_data_eft->hw_major, weigh_data_eft->hw_minor, weigh_data_eft->sw_major, weigh_data_eft->sw_minor, weigh_data_eft->minutes, weigh_data_eft->hours, weigh_data_eft->sensor1_k, weigh_data_eft->sensor2_k, weigh_data_eft->sensor3_k);
}

/**
 * @brief Send a weigh_data_eft message
 * @param chan MAVLink channel to send the message
 *
 * @param liquid_level  Liquid level sensor status (0: No liquid, 1: Liquid detected)
 * @param sensor_status  Arm lock sensor status:- bit 0-3: Arm1-4 status (0: Locked, 1: Unlocked)- bit 4-7: Reserved
 * @param weight  Current weight value in grams
 * @param right_led_temp  Right LED temperature
 * @param left_led_temp  Left LED temperature
 * @param led_status  LED status (bit0-1: over temp, bit2-3: switch status)
 * @param battery_temp  Battery connector temperature
 * @param device_type  Device type (EB)
 * @param year  Manufacturing year
 * @param month  Manufacturing month
 * @param day  Manufacturing day
 * @param number  Device serial number
 * @param hw_major  Hardware major version
 * @param hw_minor  Hardware minor version
 * @param sw_major  Software major version
 * @param sw_minor  Software minor version
 * @param minutes  Runtime minutes
 * @param hours  Runtime hours
 * @param sensor1_k  Weight sensor 1 K value
 * @param sensor2_k  Weight sensor 2 K value
 * @param sensor3_k  Weight sensor 3 K value
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_weigh_data_eft_send(mavlink_channel_t chan, uint8_t liquid_level, uint8_t sensor_status, uint16_t weight, uint8_t right_led_temp, uint8_t left_led_temp, uint8_t led_status, uint8_t battery_temp, uint8_t device_type, uint8_t year, uint8_t month, uint8_t day, uint8_t number, uint8_t hw_major, uint8_t hw_minor, uint8_t sw_major, uint8_t sw_minor, uint8_t minutes, uint16_t hours, uint16_t sensor1_k, uint16_t sensor2_k, uint16_t sensor3_k)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_WEIGH_DATA_EFT_LEN];
    _mav_put_uint16_t(buf, 0, weight);
    _mav_put_uint16_t(buf, 2, hours);
    _mav_put_uint16_t(buf, 4, sensor1_k);
    _mav_put_uint16_t(buf, 6, sensor2_k);
    _mav_put_uint16_t(buf, 8, sensor3_k);
    _mav_put_uint8_t(buf, 10, liquid_level);
    _mav_put_uint8_t(buf, 11, sensor_status);
    _mav_put_uint8_t(buf, 12, right_led_temp);
    _mav_put_uint8_t(buf, 13, left_led_temp);
    _mav_put_uint8_t(buf, 14, led_status);
    _mav_put_uint8_t(buf, 15, battery_temp);
    _mav_put_uint8_t(buf, 16, device_type);
    _mav_put_uint8_t(buf, 17, year);
    _mav_put_uint8_t(buf, 18, month);
    _mav_put_uint8_t(buf, 19, day);
    _mav_put_uint8_t(buf, 20, number);
    _mav_put_uint8_t(buf, 21, hw_major);
    _mav_put_uint8_t(buf, 22, hw_minor);
    _mav_put_uint8_t(buf, 23, sw_major);
    _mav_put_uint8_t(buf, 24, sw_minor);
    _mav_put_uint8_t(buf, 25, minutes);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_WEIGH_DATA_EFT, buf, MAVLINK_MSG_ID_WEIGH_DATA_EFT_MIN_LEN, MAVLINK_MSG_ID_WEIGH_DATA_EFT_LEN, MAVLINK_MSG_ID_WEIGH_DATA_EFT_CRC);
#else
    mavlink_weigh_data_eft_t packet;
    packet.weight = weight;
    packet.hours = hours;
    packet.sensor1_k = sensor1_k;
    packet.sensor2_k = sensor2_k;
    packet.sensor3_k = sensor3_k;
    packet.liquid_level = liquid_level;
    packet.sensor_status = sensor_status;
    packet.right_led_temp = right_led_temp;
    packet.left_led_temp = left_led_temp;
    packet.led_status = led_status;
    packet.battery_temp = battery_temp;
    packet.device_type = device_type;
    packet.year = year;
    packet.month = month;
    packet.day = day;
    packet.number = number;
    packet.hw_major = hw_major;
    packet.hw_minor = hw_minor;
    packet.sw_major = sw_major;
    packet.sw_minor = sw_minor;
    packet.minutes = minutes;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_WEIGH_DATA_EFT, (const char *)&packet, MAVLINK_MSG_ID_WEIGH_DATA_EFT_MIN_LEN, MAVLINK_MSG_ID_WEIGH_DATA_EFT_LEN, MAVLINK_MSG_ID_WEIGH_DATA_EFT_CRC);
#endif
}

/**
 * @brief Send a weigh_data_eft message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_weigh_data_eft_send_struct(mavlink_channel_t chan, const mavlink_weigh_data_eft_t* weigh_data_eft)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_weigh_data_eft_send(chan, weigh_data_eft->liquid_level, weigh_data_eft->sensor_status, weigh_data_eft->weight, weigh_data_eft->right_led_temp, weigh_data_eft->left_led_temp, weigh_data_eft->led_status, weigh_data_eft->battery_temp, weigh_data_eft->device_type, weigh_data_eft->year, weigh_data_eft->month, weigh_data_eft->day, weigh_data_eft->number, weigh_data_eft->hw_major, weigh_data_eft->hw_minor, weigh_data_eft->sw_major, weigh_data_eft->sw_minor, weigh_data_eft->minutes, weigh_data_eft->hours, weigh_data_eft->sensor1_k, weigh_data_eft->sensor2_k, weigh_data_eft->sensor3_k);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_WEIGH_DATA_EFT, (const char *)weigh_data_eft, MAVLINK_MSG_ID_WEIGH_DATA_EFT_MIN_LEN, MAVLINK_MSG_ID_WEIGH_DATA_EFT_LEN, MAVLINK_MSG_ID_WEIGH_DATA_EFT_CRC);
#endif
}

#if MAVLINK_MSG_ID_WEIGH_DATA_EFT_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by reusing
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_weigh_data_eft_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t liquid_level, uint8_t sensor_status, uint16_t weight, uint8_t right_led_temp, uint8_t left_led_temp, uint8_t led_status, uint8_t battery_temp, uint8_t device_type, uint8_t year, uint8_t month, uint8_t day, uint8_t number, uint8_t hw_major, uint8_t hw_minor, uint8_t sw_major, uint8_t sw_minor, uint8_t minutes, uint16_t hours, uint16_t sensor1_k, uint16_t sensor2_k, uint16_t sensor3_k)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint16_t(buf, 0, weight);
    _mav_put_uint16_t(buf, 2, hours);
    _mav_put_uint16_t(buf, 4, sensor1_k);
    _mav_put_uint16_t(buf, 6, sensor2_k);
    _mav_put_uint16_t(buf, 8, sensor3_k);
    _mav_put_uint8_t(buf, 10, liquid_level);
    _mav_put_uint8_t(buf, 11, sensor_status);
    _mav_put_uint8_t(buf, 12, right_led_temp);
    _mav_put_uint8_t(buf, 13, left_led_temp);
    _mav_put_uint8_t(buf, 14, led_status);
    _mav_put_uint8_t(buf, 15, battery_temp);
    _mav_put_uint8_t(buf, 16, device_type);
    _mav_put_uint8_t(buf, 17, year);
    _mav_put_uint8_t(buf, 18, month);
    _mav_put_uint8_t(buf, 19, day);
    _mav_put_uint8_t(buf, 20, number);
    _mav_put_uint8_t(buf, 21, hw_major);
    _mav_put_uint8_t(buf, 22, hw_minor);
    _mav_put_uint8_t(buf, 23, sw_major);
    _mav_put_uint8_t(buf, 24, sw_minor);
    _mav_put_uint8_t(buf, 25, minutes);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_WEIGH_DATA_EFT, buf, MAVLINK_MSG_ID_WEIGH_DATA_EFT_MIN_LEN, MAVLINK_MSG_ID_WEIGH_DATA_EFT_LEN, MAVLINK_MSG_ID_WEIGH_DATA_EFT_CRC);
#else
    mavlink_weigh_data_eft_t *packet = (mavlink_weigh_data_eft_t *)msgbuf;
    packet->weight = weight;
    packet->hours = hours;
    packet->sensor1_k = sensor1_k;
    packet->sensor2_k = sensor2_k;
    packet->sensor3_k = sensor3_k;
    packet->liquid_level = liquid_level;
    packet->sensor_status = sensor_status;
    packet->right_led_temp = right_led_temp;
    packet->left_led_temp = left_led_temp;
    packet->led_status = led_status;
    packet->battery_temp = battery_temp;
    packet->device_type = device_type;
    packet->year = year;
    packet->month = month;
    packet->day = day;
    packet->number = number;
    packet->hw_major = hw_major;
    packet->hw_minor = hw_minor;
    packet->sw_major = sw_major;
    packet->sw_minor = sw_minor;
    packet->minutes = minutes;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_WEIGH_DATA_EFT, (const char *)packet, MAVLINK_MSG_ID_WEIGH_DATA_EFT_MIN_LEN, MAVLINK_MSG_ID_WEIGH_DATA_EFT_LEN, MAVLINK_MSG_ID_WEIGH_DATA_EFT_CRC);
#endif
}
#endif

#endif

// MESSAGE WEIGH_DATA_EFT UNPACKING


/**
 * @brief Get field liquid_level from weigh_data_eft message
 *
 * @return  Liquid level sensor status (0: No liquid, 1: Liquid detected)
 */
static inline uint8_t mavlink_msg_weigh_data_eft_get_liquid_level(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  10);
}

/**
 * @brief Get field sensor_status from weigh_data_eft message
 *
 * @return  Arm lock sensor status:- bit 0-3: Arm1-4 status (0: Locked, 1: Unlocked)- bit 4-7: Reserved
 */
static inline uint8_t mavlink_msg_weigh_data_eft_get_sensor_status(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  11);
}

/**
 * @brief Get field weight from weigh_data_eft message
 *
 * @return  Current weight value in grams
 */
static inline uint16_t mavlink_msg_weigh_data_eft_get_weight(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  0);
}

/**
 * @brief Get field right_led_temp from weigh_data_eft message
 *
 * @return  Right LED temperature
 */
static inline uint8_t mavlink_msg_weigh_data_eft_get_right_led_temp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  12);
}

/**
 * @brief Get field left_led_temp from weigh_data_eft message
 *
 * @return  Left LED temperature
 */
static inline uint8_t mavlink_msg_weigh_data_eft_get_left_led_temp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  13);
}

/**
 * @brief Get field led_status from weigh_data_eft message
 *
 * @return  LED status (bit0-1: over temp, bit2-3: switch status)
 */
static inline uint8_t mavlink_msg_weigh_data_eft_get_led_status(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  14);
}

/**
 * @brief Get field battery_temp from weigh_data_eft message
 *
 * @return  Battery connector temperature
 */
static inline uint8_t mavlink_msg_weigh_data_eft_get_battery_temp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  15);
}

/**
 * @brief Get field device_type from weigh_data_eft message
 *
 * @return  Device type (EB)
 */
static inline uint8_t mavlink_msg_weigh_data_eft_get_device_type(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  16);
}

/**
 * @brief Get field year from weigh_data_eft message
 *
 * @return  Manufacturing year
 */
static inline uint8_t mavlink_msg_weigh_data_eft_get_year(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  17);
}

/**
 * @brief Get field month from weigh_data_eft message
 *
 * @return  Manufacturing month
 */
static inline uint8_t mavlink_msg_weigh_data_eft_get_month(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  18);
}

/**
 * @brief Get field day from weigh_data_eft message
 *
 * @return  Manufacturing day
 */
static inline uint8_t mavlink_msg_weigh_data_eft_get_day(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  19);
}

/**
 * @brief Get field number from weigh_data_eft message
 *
 * @return  Device serial number
 */
static inline uint8_t mavlink_msg_weigh_data_eft_get_number(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  20);
}

/**
 * @brief Get field hw_major from weigh_data_eft message
 *
 * @return  Hardware major version
 */
static inline uint8_t mavlink_msg_weigh_data_eft_get_hw_major(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  21);
}

/**
 * @brief Get field hw_minor from weigh_data_eft message
 *
 * @return  Hardware minor version
 */
static inline uint8_t mavlink_msg_weigh_data_eft_get_hw_minor(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  22);
}

/**
 * @brief Get field sw_major from weigh_data_eft message
 *
 * @return  Software major version
 */
static inline uint8_t mavlink_msg_weigh_data_eft_get_sw_major(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  23);
}

/**
 * @brief Get field sw_minor from weigh_data_eft message
 *
 * @return  Software minor version
 */
static inline uint8_t mavlink_msg_weigh_data_eft_get_sw_minor(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  24);
}

/**
 * @brief Get field minutes from weigh_data_eft message
 *
 * @return  Runtime minutes
 */
static inline uint8_t mavlink_msg_weigh_data_eft_get_minutes(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  25);
}

/**
 * @brief Get field hours from weigh_data_eft message
 *
 * @return  Runtime hours
 */
static inline uint16_t mavlink_msg_weigh_data_eft_get_hours(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  2);
}

/**
 * @brief Get field sensor1_k from weigh_data_eft message
 *
 * @return  Weight sensor 1 K value
 */
static inline uint16_t mavlink_msg_weigh_data_eft_get_sensor1_k(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  4);
}

/**
 * @brief Get field sensor2_k from weigh_data_eft message
 *
 * @return  Weight sensor 2 K value
 */
static inline uint16_t mavlink_msg_weigh_data_eft_get_sensor2_k(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  6);
}

/**
 * @brief Get field sensor3_k from weigh_data_eft message
 *
 * @return  Weight sensor 3 K value
 */
static inline uint16_t mavlink_msg_weigh_data_eft_get_sensor3_k(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  8);
}

/**
 * @brief Decode a weigh_data_eft message into a struct
 *
 * @param msg The message to decode
 * @param weigh_data_eft C-struct to decode the message contents into
 */
static inline void mavlink_msg_weigh_data_eft_decode(const mavlink_message_t* msg, mavlink_weigh_data_eft_t* weigh_data_eft)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    weigh_data_eft->weight = mavlink_msg_weigh_data_eft_get_weight(msg);
    weigh_data_eft->hours = mavlink_msg_weigh_data_eft_get_hours(msg);
    weigh_data_eft->sensor1_k = mavlink_msg_weigh_data_eft_get_sensor1_k(msg);
    weigh_data_eft->sensor2_k = mavlink_msg_weigh_data_eft_get_sensor2_k(msg);
    weigh_data_eft->sensor3_k = mavlink_msg_weigh_data_eft_get_sensor3_k(msg);
    weigh_data_eft->liquid_level = mavlink_msg_weigh_data_eft_get_liquid_level(msg);
    weigh_data_eft->sensor_status = mavlink_msg_weigh_data_eft_get_sensor_status(msg);
    weigh_data_eft->right_led_temp = mavlink_msg_weigh_data_eft_get_right_led_temp(msg);
    weigh_data_eft->left_led_temp = mavlink_msg_weigh_data_eft_get_left_led_temp(msg);
    weigh_data_eft->led_status = mavlink_msg_weigh_data_eft_get_led_status(msg);
    weigh_data_eft->battery_temp = mavlink_msg_weigh_data_eft_get_battery_temp(msg);
    weigh_data_eft->device_type = mavlink_msg_weigh_data_eft_get_device_type(msg);
    weigh_data_eft->year = mavlink_msg_weigh_data_eft_get_year(msg);
    weigh_data_eft->month = mavlink_msg_weigh_data_eft_get_month(msg);
    weigh_data_eft->day = mavlink_msg_weigh_data_eft_get_day(msg);
    weigh_data_eft->number = mavlink_msg_weigh_data_eft_get_number(msg);
    weigh_data_eft->hw_major = mavlink_msg_weigh_data_eft_get_hw_major(msg);
    weigh_data_eft->hw_minor = mavlink_msg_weigh_data_eft_get_hw_minor(msg);
    weigh_data_eft->sw_major = mavlink_msg_weigh_data_eft_get_sw_major(msg);
    weigh_data_eft->sw_minor = mavlink_msg_weigh_data_eft_get_sw_minor(msg);
    weigh_data_eft->minutes = mavlink_msg_weigh_data_eft_get_minutes(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_WEIGH_DATA_EFT_LEN? msg->len : MAVLINK_MSG_ID_WEIGH_DATA_EFT_LEN;
        memset(weigh_data_eft, 0, MAVLINK_MSG_ID_WEIGH_DATA_EFT_LEN);
    memcpy(weigh_data_eft, _MAV_PAYLOAD(msg), len);
#endif
}
