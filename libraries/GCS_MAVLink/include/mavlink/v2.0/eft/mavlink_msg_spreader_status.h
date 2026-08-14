#pragma once
// MESSAGE SPREADER_STATUS PACKING

#define MAVLINK_MSG_ID_SPREADER_STATUS 512


typedef struct __mavlink_spreader_status_t {
 uint16_t spreader_can_baudrate; /*<  CAN baudrate*/
 uint16_t spreader_sequence; /*<  Spreader sequence number*/
 uint16_t spreader_firmware_version; /*<  Firmware version number*/
 uint8_t spreader_servo_angle; /*<  Servo angle (0-100%, 0x00-0x64)*/
 uint8_t spreader_sensor_status; /*<  
        Sensor status bitmap:
        bit0: Material sensor (0: Not detected, 1: Detected)
        bit1: Disk rotation (0: Not detected, 1: Detected)
        bit2: Servo fault (0: Not detected, 1: Detected)
        bit3: Temperature (0: Not detected, 1: Detected)
        bit4: Power rate (0: Not detected, 1: Detected)
        bit5: Hall sensor (0: Not detected, 1: Detected)
        bit6-7: Reserved
        */
 uint8_t spreader_can_enable; /*<  CAN function status (0: Disabled, 1: Enabled)*/
 uint8_t spreader_speed; /*<  Spreader actual speed (Value = 0-256, actual speed = Value * 10)*/
 uint8_t spreader_function_status; /*<  
        Function status bitmap:
        bit0: Material shortage alarm (0: OFF, 1: ON)
        bit1: spreader stall alarm (0: OFF, 1: ON)
        bit2-7: Reserved
        */
 uint8_t spreader_life_signal; /*<  Life signal cycling from 0x00 to 0xFF*/
 uint8_t spreader_year; /*<  Manufacturing year*/
 uint8_t spreader_month; /*<  Manufacturing month*/
 uint8_t spreader_day; /*<  Manufacturing day*/
} mavlink_spreader_status_t;

#define MAVLINK_MSG_ID_SPREADER_STATUS_LEN 15
#define MAVLINK_MSG_ID_SPREADER_STATUS_MIN_LEN 15
#define MAVLINK_MSG_ID_512_LEN 15
#define MAVLINK_MSG_ID_512_MIN_LEN 15

#define MAVLINK_MSG_ID_SPREADER_STATUS_CRC 167
#define MAVLINK_MSG_ID_512_CRC 167



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_SPREADER_STATUS { \
    512, \
    "SPREADER_STATUS", \
    12, \
    {  { "spreader_servo_angle", NULL, MAVLINK_TYPE_UINT8_T, 0, 6, offsetof(mavlink_spreader_status_t, spreader_servo_angle) }, \
         { "spreader_sensor_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 7, offsetof(mavlink_spreader_status_t, spreader_sensor_status) }, \
         { "spreader_can_enable", NULL, MAVLINK_TYPE_UINT8_T, 0, 8, offsetof(mavlink_spreader_status_t, spreader_can_enable) }, \
         { "spreader_can_baudrate", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_spreader_status_t, spreader_can_baudrate) }, \
         { "spreader_speed", NULL, MAVLINK_TYPE_UINT8_T, 0, 9, offsetof(mavlink_spreader_status_t, spreader_speed) }, \
         { "spreader_function_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 10, offsetof(mavlink_spreader_status_t, spreader_function_status) }, \
         { "spreader_life_signal", NULL, MAVLINK_TYPE_UINT8_T, 0, 11, offsetof(mavlink_spreader_status_t, spreader_life_signal) }, \
         { "spreader_year", NULL, MAVLINK_TYPE_UINT8_T, 0, 12, offsetof(mavlink_spreader_status_t, spreader_year) }, \
         { "spreader_month", NULL, MAVLINK_TYPE_UINT8_T, 0, 13, offsetof(mavlink_spreader_status_t, spreader_month) }, \
         { "spreader_day", NULL, MAVLINK_TYPE_UINT8_T, 0, 14, offsetof(mavlink_spreader_status_t, spreader_day) }, \
         { "spreader_sequence", NULL, MAVLINK_TYPE_UINT16_T, 0, 2, offsetof(mavlink_spreader_status_t, spreader_sequence) }, \
         { "spreader_firmware_version", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_spreader_status_t, spreader_firmware_version) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_SPREADER_STATUS { \
    "SPREADER_STATUS", \
    12, \
    {  { "spreader_servo_angle", NULL, MAVLINK_TYPE_UINT8_T, 0, 6, offsetof(mavlink_spreader_status_t, spreader_servo_angle) }, \
         { "spreader_sensor_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 7, offsetof(mavlink_spreader_status_t, spreader_sensor_status) }, \
         { "spreader_can_enable", NULL, MAVLINK_TYPE_UINT8_T, 0, 8, offsetof(mavlink_spreader_status_t, spreader_can_enable) }, \
         { "spreader_can_baudrate", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_spreader_status_t, spreader_can_baudrate) }, \
         { "spreader_speed", NULL, MAVLINK_TYPE_UINT8_T, 0, 9, offsetof(mavlink_spreader_status_t, spreader_speed) }, \
         { "spreader_function_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 10, offsetof(mavlink_spreader_status_t, spreader_function_status) }, \
         { "spreader_life_signal", NULL, MAVLINK_TYPE_UINT8_T, 0, 11, offsetof(mavlink_spreader_status_t, spreader_life_signal) }, \
         { "spreader_year", NULL, MAVLINK_TYPE_UINT8_T, 0, 12, offsetof(mavlink_spreader_status_t, spreader_year) }, \
         { "spreader_month", NULL, MAVLINK_TYPE_UINT8_T, 0, 13, offsetof(mavlink_spreader_status_t, spreader_month) }, \
         { "spreader_day", NULL, MAVLINK_TYPE_UINT8_T, 0, 14, offsetof(mavlink_spreader_status_t, spreader_day) }, \
         { "spreader_sequence", NULL, MAVLINK_TYPE_UINT16_T, 0, 2, offsetof(mavlink_spreader_status_t, spreader_sequence) }, \
         { "spreader_firmware_version", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_spreader_status_t, spreader_firmware_version) }, \
         } \
}
#endif

/**
 * @brief Pack a spreader_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param spreader_servo_angle  Servo angle (0-100%, 0x00-0x64)
 * @param spreader_sensor_status  
        Sensor status bitmap:
        bit0: Material sensor (0: Not detected, 1: Detected)
        bit1: Disk rotation (0: Not detected, 1: Detected)
        bit2: Servo fault (0: Not detected, 1: Detected)
        bit3: Temperature (0: Not detected, 1: Detected)
        bit4: Power rate (0: Not detected, 1: Detected)
        bit5: Hall sensor (0: Not detected, 1: Detected)
        bit6-7: Reserved
        
 * @param spreader_can_enable  CAN function status (0: Disabled, 1: Enabled)
 * @param spreader_can_baudrate  CAN baudrate
 * @param spreader_speed  Spreader actual speed (Value = 0-256, actual speed = Value * 10)
 * @param spreader_function_status  
        Function status bitmap:
        bit0: Material shortage alarm (0: OFF, 1: ON)
        bit1: spreader stall alarm (0: OFF, 1: ON)
        bit2-7: Reserved
        
 * @param spreader_life_signal  Life signal cycling from 0x00 to 0xFF
 * @param spreader_year  Manufacturing year
 * @param spreader_month  Manufacturing month
 * @param spreader_day  Manufacturing day
 * @param spreader_sequence  Spreader sequence number
 * @param spreader_firmware_version  Firmware version number
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_spreader_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t spreader_servo_angle, uint8_t spreader_sensor_status, uint8_t spreader_can_enable, uint16_t spreader_can_baudrate, uint8_t spreader_speed, uint8_t spreader_function_status, uint8_t spreader_life_signal, uint8_t spreader_year, uint8_t spreader_month, uint8_t spreader_day, uint16_t spreader_sequence, uint16_t spreader_firmware_version)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SPREADER_STATUS_LEN];
    _mav_put_uint16_t(buf, 0, spreader_can_baudrate);
    _mav_put_uint16_t(buf, 2, spreader_sequence);
    _mav_put_uint16_t(buf, 4, spreader_firmware_version);
    _mav_put_uint8_t(buf, 6, spreader_servo_angle);
    _mav_put_uint8_t(buf, 7, spreader_sensor_status);
    _mav_put_uint8_t(buf, 8, spreader_can_enable);
    _mav_put_uint8_t(buf, 9, spreader_speed);
    _mav_put_uint8_t(buf, 10, spreader_function_status);
    _mav_put_uint8_t(buf, 11, spreader_life_signal);
    _mav_put_uint8_t(buf, 12, spreader_year);
    _mav_put_uint8_t(buf, 13, spreader_month);
    _mav_put_uint8_t(buf, 14, spreader_day);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SPREADER_STATUS_LEN);
#else
    mavlink_spreader_status_t packet;
    packet.spreader_can_baudrate = spreader_can_baudrate;
    packet.spreader_sequence = spreader_sequence;
    packet.spreader_firmware_version = spreader_firmware_version;
    packet.spreader_servo_angle = spreader_servo_angle;
    packet.spreader_sensor_status = spreader_sensor_status;
    packet.spreader_can_enable = spreader_can_enable;
    packet.spreader_speed = spreader_speed;
    packet.spreader_function_status = spreader_function_status;
    packet.spreader_life_signal = spreader_life_signal;
    packet.spreader_year = spreader_year;
    packet.spreader_month = spreader_month;
    packet.spreader_day = spreader_day;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SPREADER_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SPREADER_STATUS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SPREADER_STATUS_MIN_LEN, MAVLINK_MSG_ID_SPREADER_STATUS_LEN, MAVLINK_MSG_ID_SPREADER_STATUS_CRC);
}

/**
 * @brief Pack a spreader_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param spreader_servo_angle  Servo angle (0-100%, 0x00-0x64)
 * @param spreader_sensor_status  
        Sensor status bitmap:
        bit0: Material sensor (0: Not detected, 1: Detected)
        bit1: Disk rotation (0: Not detected, 1: Detected)
        bit2: Servo fault (0: Not detected, 1: Detected)
        bit3: Temperature (0: Not detected, 1: Detected)
        bit4: Power rate (0: Not detected, 1: Detected)
        bit5: Hall sensor (0: Not detected, 1: Detected)
        bit6-7: Reserved
        
 * @param spreader_can_enable  CAN function status (0: Disabled, 1: Enabled)
 * @param spreader_can_baudrate  CAN baudrate
 * @param spreader_speed  Spreader actual speed (Value = 0-256, actual speed = Value * 10)
 * @param spreader_function_status  
        Function status bitmap:
        bit0: Material shortage alarm (0: OFF, 1: ON)
        bit1: spreader stall alarm (0: OFF, 1: ON)
        bit2-7: Reserved
        
 * @param spreader_life_signal  Life signal cycling from 0x00 to 0xFF
 * @param spreader_year  Manufacturing year
 * @param spreader_month  Manufacturing month
 * @param spreader_day  Manufacturing day
 * @param spreader_sequence  Spreader sequence number
 * @param spreader_firmware_version  Firmware version number
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_spreader_status_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint8_t spreader_servo_angle, uint8_t spreader_sensor_status, uint8_t spreader_can_enable, uint16_t spreader_can_baudrate, uint8_t spreader_speed, uint8_t spreader_function_status, uint8_t spreader_life_signal, uint8_t spreader_year, uint8_t spreader_month, uint8_t spreader_day, uint16_t spreader_sequence, uint16_t spreader_firmware_version)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SPREADER_STATUS_LEN];
    _mav_put_uint16_t(buf, 0, spreader_can_baudrate);
    _mav_put_uint16_t(buf, 2, spreader_sequence);
    _mav_put_uint16_t(buf, 4, spreader_firmware_version);
    _mav_put_uint8_t(buf, 6, spreader_servo_angle);
    _mav_put_uint8_t(buf, 7, spreader_sensor_status);
    _mav_put_uint8_t(buf, 8, spreader_can_enable);
    _mav_put_uint8_t(buf, 9, spreader_speed);
    _mav_put_uint8_t(buf, 10, spreader_function_status);
    _mav_put_uint8_t(buf, 11, spreader_life_signal);
    _mav_put_uint8_t(buf, 12, spreader_year);
    _mav_put_uint8_t(buf, 13, spreader_month);
    _mav_put_uint8_t(buf, 14, spreader_day);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SPREADER_STATUS_LEN);
#else
    mavlink_spreader_status_t packet;
    packet.spreader_can_baudrate = spreader_can_baudrate;
    packet.spreader_sequence = spreader_sequence;
    packet.spreader_firmware_version = spreader_firmware_version;
    packet.spreader_servo_angle = spreader_servo_angle;
    packet.spreader_sensor_status = spreader_sensor_status;
    packet.spreader_can_enable = spreader_can_enable;
    packet.spreader_speed = spreader_speed;
    packet.spreader_function_status = spreader_function_status;
    packet.spreader_life_signal = spreader_life_signal;
    packet.spreader_year = spreader_year;
    packet.spreader_month = spreader_month;
    packet.spreader_day = spreader_day;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SPREADER_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SPREADER_STATUS;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_SPREADER_STATUS_MIN_LEN, MAVLINK_MSG_ID_SPREADER_STATUS_LEN, MAVLINK_MSG_ID_SPREADER_STATUS_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_SPREADER_STATUS_MIN_LEN, MAVLINK_MSG_ID_SPREADER_STATUS_LEN);
#endif
}

/**
 * @brief Pack a spreader_status message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param spreader_servo_angle  Servo angle (0-100%, 0x00-0x64)
 * @param spreader_sensor_status  
        Sensor status bitmap:
        bit0: Material sensor (0: Not detected, 1: Detected)
        bit1: Disk rotation (0: Not detected, 1: Detected)
        bit2: Servo fault (0: Not detected, 1: Detected)
        bit3: Temperature (0: Not detected, 1: Detected)
        bit4: Power rate (0: Not detected, 1: Detected)
        bit5: Hall sensor (0: Not detected, 1: Detected)
        bit6-7: Reserved
        
 * @param spreader_can_enable  CAN function status (0: Disabled, 1: Enabled)
 * @param spreader_can_baudrate  CAN baudrate
 * @param spreader_speed  Spreader actual speed (Value = 0-256, actual speed = Value * 10)
 * @param spreader_function_status  
        Function status bitmap:
        bit0: Material shortage alarm (0: OFF, 1: ON)
        bit1: spreader stall alarm (0: OFF, 1: ON)
        bit2-7: Reserved
        
 * @param spreader_life_signal  Life signal cycling from 0x00 to 0xFF
 * @param spreader_year  Manufacturing year
 * @param spreader_month  Manufacturing month
 * @param spreader_day  Manufacturing day
 * @param spreader_sequence  Spreader sequence number
 * @param spreader_firmware_version  Firmware version number
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_spreader_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t spreader_servo_angle,uint8_t spreader_sensor_status,uint8_t spreader_can_enable,uint16_t spreader_can_baudrate,uint8_t spreader_speed,uint8_t spreader_function_status,uint8_t spreader_life_signal,uint8_t spreader_year,uint8_t spreader_month,uint8_t spreader_day,uint16_t spreader_sequence,uint16_t spreader_firmware_version)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SPREADER_STATUS_LEN];
    _mav_put_uint16_t(buf, 0, spreader_can_baudrate);
    _mav_put_uint16_t(buf, 2, spreader_sequence);
    _mav_put_uint16_t(buf, 4, spreader_firmware_version);
    _mav_put_uint8_t(buf, 6, spreader_servo_angle);
    _mav_put_uint8_t(buf, 7, spreader_sensor_status);
    _mav_put_uint8_t(buf, 8, spreader_can_enable);
    _mav_put_uint8_t(buf, 9, spreader_speed);
    _mav_put_uint8_t(buf, 10, spreader_function_status);
    _mav_put_uint8_t(buf, 11, spreader_life_signal);
    _mav_put_uint8_t(buf, 12, spreader_year);
    _mav_put_uint8_t(buf, 13, spreader_month);
    _mav_put_uint8_t(buf, 14, spreader_day);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SPREADER_STATUS_LEN);
#else
    mavlink_spreader_status_t packet;
    packet.spreader_can_baudrate = spreader_can_baudrate;
    packet.spreader_sequence = spreader_sequence;
    packet.spreader_firmware_version = spreader_firmware_version;
    packet.spreader_servo_angle = spreader_servo_angle;
    packet.spreader_sensor_status = spreader_sensor_status;
    packet.spreader_can_enable = spreader_can_enable;
    packet.spreader_speed = spreader_speed;
    packet.spreader_function_status = spreader_function_status;
    packet.spreader_life_signal = spreader_life_signal;
    packet.spreader_year = spreader_year;
    packet.spreader_month = spreader_month;
    packet.spreader_day = spreader_day;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SPREADER_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SPREADER_STATUS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SPREADER_STATUS_MIN_LEN, MAVLINK_MSG_ID_SPREADER_STATUS_LEN, MAVLINK_MSG_ID_SPREADER_STATUS_CRC);
}

/**
 * @brief Encode a spreader_status struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param spreader_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_spreader_status_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_spreader_status_t* spreader_status)
{
    return mavlink_msg_spreader_status_pack(system_id, component_id, msg, spreader_status->spreader_servo_angle, spreader_status->spreader_sensor_status, spreader_status->spreader_can_enable, spreader_status->spreader_can_baudrate, spreader_status->spreader_speed, spreader_status->spreader_function_status, spreader_status->spreader_life_signal, spreader_status->spreader_year, spreader_status->spreader_month, spreader_status->spreader_day, spreader_status->spreader_sequence, spreader_status->spreader_firmware_version);
}

/**
 * @brief Encode a spreader_status struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param spreader_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_spreader_status_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_spreader_status_t* spreader_status)
{
    return mavlink_msg_spreader_status_pack_chan(system_id, component_id, chan, msg, spreader_status->spreader_servo_angle, spreader_status->spreader_sensor_status, spreader_status->spreader_can_enable, spreader_status->spreader_can_baudrate, spreader_status->spreader_speed, spreader_status->spreader_function_status, spreader_status->spreader_life_signal, spreader_status->spreader_year, spreader_status->spreader_month, spreader_status->spreader_day, spreader_status->spreader_sequence, spreader_status->spreader_firmware_version);
}

/**
 * @brief Encode a spreader_status struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param spreader_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_spreader_status_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_spreader_status_t* spreader_status)
{
    return mavlink_msg_spreader_status_pack_status(system_id, component_id, _status, msg,  spreader_status->spreader_servo_angle, spreader_status->spreader_sensor_status, spreader_status->spreader_can_enable, spreader_status->spreader_can_baudrate, spreader_status->spreader_speed, spreader_status->spreader_function_status, spreader_status->spreader_life_signal, spreader_status->spreader_year, spreader_status->spreader_month, spreader_status->spreader_day, spreader_status->spreader_sequence, spreader_status->spreader_firmware_version);
}

/**
 * @brief Send a spreader_status message
 * @param chan MAVLink channel to send the message
 *
 * @param spreader_servo_angle  Servo angle (0-100%, 0x00-0x64)
 * @param spreader_sensor_status  
        Sensor status bitmap:
        bit0: Material sensor (0: Not detected, 1: Detected)
        bit1: Disk rotation (0: Not detected, 1: Detected)
        bit2: Servo fault (0: Not detected, 1: Detected)
        bit3: Temperature (0: Not detected, 1: Detected)
        bit4: Power rate (0: Not detected, 1: Detected)
        bit5: Hall sensor (0: Not detected, 1: Detected)
        bit6-7: Reserved
        
 * @param spreader_can_enable  CAN function status (0: Disabled, 1: Enabled)
 * @param spreader_can_baudrate  CAN baudrate
 * @param spreader_speed  Spreader actual speed (Value = 0-256, actual speed = Value * 10)
 * @param spreader_function_status  
        Function status bitmap:
        bit0: Material shortage alarm (0: OFF, 1: ON)
        bit1: spreader stall alarm (0: OFF, 1: ON)
        bit2-7: Reserved
        
 * @param spreader_life_signal  Life signal cycling from 0x00 to 0xFF
 * @param spreader_year  Manufacturing year
 * @param spreader_month  Manufacturing month
 * @param spreader_day  Manufacturing day
 * @param spreader_sequence  Spreader sequence number
 * @param spreader_firmware_version  Firmware version number
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_spreader_status_send(mavlink_channel_t chan, uint8_t spreader_servo_angle, uint8_t spreader_sensor_status, uint8_t spreader_can_enable, uint16_t spreader_can_baudrate, uint8_t spreader_speed, uint8_t spreader_function_status, uint8_t spreader_life_signal, uint8_t spreader_year, uint8_t spreader_month, uint8_t spreader_day, uint16_t spreader_sequence, uint16_t spreader_firmware_version)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SPREADER_STATUS_LEN];
    _mav_put_uint16_t(buf, 0, spreader_can_baudrate);
    _mav_put_uint16_t(buf, 2, spreader_sequence);
    _mav_put_uint16_t(buf, 4, spreader_firmware_version);
    _mav_put_uint8_t(buf, 6, spreader_servo_angle);
    _mav_put_uint8_t(buf, 7, spreader_sensor_status);
    _mav_put_uint8_t(buf, 8, spreader_can_enable);
    _mav_put_uint8_t(buf, 9, spreader_speed);
    _mav_put_uint8_t(buf, 10, spreader_function_status);
    _mav_put_uint8_t(buf, 11, spreader_life_signal);
    _mav_put_uint8_t(buf, 12, spreader_year);
    _mav_put_uint8_t(buf, 13, spreader_month);
    _mav_put_uint8_t(buf, 14, spreader_day);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SPREADER_STATUS, buf, MAVLINK_MSG_ID_SPREADER_STATUS_MIN_LEN, MAVLINK_MSG_ID_SPREADER_STATUS_LEN, MAVLINK_MSG_ID_SPREADER_STATUS_CRC);
#else
    mavlink_spreader_status_t packet;
    packet.spreader_can_baudrate = spreader_can_baudrate;
    packet.spreader_sequence = spreader_sequence;
    packet.spreader_firmware_version = spreader_firmware_version;
    packet.spreader_servo_angle = spreader_servo_angle;
    packet.spreader_sensor_status = spreader_sensor_status;
    packet.spreader_can_enable = spreader_can_enable;
    packet.spreader_speed = spreader_speed;
    packet.spreader_function_status = spreader_function_status;
    packet.spreader_life_signal = spreader_life_signal;
    packet.spreader_year = spreader_year;
    packet.spreader_month = spreader_month;
    packet.spreader_day = spreader_day;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SPREADER_STATUS, (const char *)&packet, MAVLINK_MSG_ID_SPREADER_STATUS_MIN_LEN, MAVLINK_MSG_ID_SPREADER_STATUS_LEN, MAVLINK_MSG_ID_SPREADER_STATUS_CRC);
#endif
}

/**
 * @brief Send a spreader_status message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_spreader_status_send_struct(mavlink_channel_t chan, const mavlink_spreader_status_t* spreader_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_spreader_status_send(chan, spreader_status->spreader_servo_angle, spreader_status->spreader_sensor_status, spreader_status->spreader_can_enable, spreader_status->spreader_can_baudrate, spreader_status->spreader_speed, spreader_status->spreader_function_status, spreader_status->spreader_life_signal, spreader_status->spreader_year, spreader_status->spreader_month, spreader_status->spreader_day, spreader_status->spreader_sequence, spreader_status->spreader_firmware_version);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SPREADER_STATUS, (const char *)spreader_status, MAVLINK_MSG_ID_SPREADER_STATUS_MIN_LEN, MAVLINK_MSG_ID_SPREADER_STATUS_LEN, MAVLINK_MSG_ID_SPREADER_STATUS_CRC);
#endif
}

#if MAVLINK_MSG_ID_SPREADER_STATUS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by reusing
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_spreader_status_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t spreader_servo_angle, uint8_t spreader_sensor_status, uint8_t spreader_can_enable, uint16_t spreader_can_baudrate, uint8_t spreader_speed, uint8_t spreader_function_status, uint8_t spreader_life_signal, uint8_t spreader_year, uint8_t spreader_month, uint8_t spreader_day, uint16_t spreader_sequence, uint16_t spreader_firmware_version)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint16_t(buf, 0, spreader_can_baudrate);
    _mav_put_uint16_t(buf, 2, spreader_sequence);
    _mav_put_uint16_t(buf, 4, spreader_firmware_version);
    _mav_put_uint8_t(buf, 6, spreader_servo_angle);
    _mav_put_uint8_t(buf, 7, spreader_sensor_status);
    _mav_put_uint8_t(buf, 8, spreader_can_enable);
    _mav_put_uint8_t(buf, 9, spreader_speed);
    _mav_put_uint8_t(buf, 10, spreader_function_status);
    _mav_put_uint8_t(buf, 11, spreader_life_signal);
    _mav_put_uint8_t(buf, 12, spreader_year);
    _mav_put_uint8_t(buf, 13, spreader_month);
    _mav_put_uint8_t(buf, 14, spreader_day);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SPREADER_STATUS, buf, MAVLINK_MSG_ID_SPREADER_STATUS_MIN_LEN, MAVLINK_MSG_ID_SPREADER_STATUS_LEN, MAVLINK_MSG_ID_SPREADER_STATUS_CRC);
#else
    mavlink_spreader_status_t *packet = (mavlink_spreader_status_t *)msgbuf;
    packet->spreader_can_baudrate = spreader_can_baudrate;
    packet->spreader_sequence = spreader_sequence;
    packet->spreader_firmware_version = spreader_firmware_version;
    packet->spreader_servo_angle = spreader_servo_angle;
    packet->spreader_sensor_status = spreader_sensor_status;
    packet->spreader_can_enable = spreader_can_enable;
    packet->spreader_speed = spreader_speed;
    packet->spreader_function_status = spreader_function_status;
    packet->spreader_life_signal = spreader_life_signal;
    packet->spreader_year = spreader_year;
    packet->spreader_month = spreader_month;
    packet->spreader_day = spreader_day;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SPREADER_STATUS, (const char *)packet, MAVLINK_MSG_ID_SPREADER_STATUS_MIN_LEN, MAVLINK_MSG_ID_SPREADER_STATUS_LEN, MAVLINK_MSG_ID_SPREADER_STATUS_CRC);
#endif
}
#endif

#endif

// MESSAGE SPREADER_STATUS UNPACKING


/**
 * @brief Get field spreader_servo_angle from spreader_status message
 *
 * @return  Servo angle (0-100%, 0x00-0x64)
 */
static inline uint8_t mavlink_msg_spreader_status_get_spreader_servo_angle(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  6);
}

/**
 * @brief Get field spreader_sensor_status from spreader_status message
 *
 * @return  
        Sensor status bitmap:
        bit0: Material sensor (0: Not detected, 1: Detected)
        bit1: Disk rotation (0: Not detected, 1: Detected)
        bit2: Servo fault (0: Not detected, 1: Detected)
        bit3: Temperature (0: Not detected, 1: Detected)
        bit4: Power rate (0: Not detected, 1: Detected)
        bit5: Hall sensor (0: Not detected, 1: Detected)
        bit6-7: Reserved
        
 */
static inline uint8_t mavlink_msg_spreader_status_get_spreader_sensor_status(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  7);
}

/**
 * @brief Get field spreader_can_enable from spreader_status message
 *
 * @return  CAN function status (0: Disabled, 1: Enabled)
 */
static inline uint8_t mavlink_msg_spreader_status_get_spreader_can_enable(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  8);
}

/**
 * @brief Get field spreader_can_baudrate from spreader_status message
 *
 * @return  CAN baudrate
 */
static inline uint16_t mavlink_msg_spreader_status_get_spreader_can_baudrate(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  0);
}

/**
 * @brief Get field spreader_speed from spreader_status message
 *
 * @return  Spreader actual speed (Value = 0-256, actual speed = Value * 10)
 */
static inline uint8_t mavlink_msg_spreader_status_get_spreader_speed(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  9);
}

/**
 * @brief Get field spreader_function_status from spreader_status message
 *
 * @return  
        Function status bitmap:
        bit0: Material shortage alarm (0: OFF, 1: ON)
        bit1: spreader stall alarm (0: OFF, 1: ON)
        bit2-7: Reserved
        
 */
static inline uint8_t mavlink_msg_spreader_status_get_spreader_function_status(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  10);
}

/**
 * @brief Get field spreader_life_signal from spreader_status message
 *
 * @return  Life signal cycling from 0x00 to 0xFF
 */
static inline uint8_t mavlink_msg_spreader_status_get_spreader_life_signal(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  11);
}

/**
 * @brief Get field spreader_year from spreader_status message
 *
 * @return  Manufacturing year
 */
static inline uint8_t mavlink_msg_spreader_status_get_spreader_year(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  12);
}

/**
 * @brief Get field spreader_month from spreader_status message
 *
 * @return  Manufacturing month
 */
static inline uint8_t mavlink_msg_spreader_status_get_spreader_month(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  13);
}

/**
 * @brief Get field spreader_day from spreader_status message
 *
 * @return  Manufacturing day
 */
static inline uint8_t mavlink_msg_spreader_status_get_spreader_day(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  14);
}

/**
 * @brief Get field spreader_sequence from spreader_status message
 *
 * @return  Spreader sequence number
 */
static inline uint16_t mavlink_msg_spreader_status_get_spreader_sequence(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  2);
}

/**
 * @brief Get field spreader_firmware_version from spreader_status message
 *
 * @return  Firmware version number
 */
static inline uint16_t mavlink_msg_spreader_status_get_spreader_firmware_version(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  4);
}

/**
 * @brief Decode a spreader_status message into a struct
 *
 * @param msg The message to decode
 * @param spreader_status C-struct to decode the message contents into
 */
static inline void mavlink_msg_spreader_status_decode(const mavlink_message_t* msg, mavlink_spreader_status_t* spreader_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    spreader_status->spreader_can_baudrate = mavlink_msg_spreader_status_get_spreader_can_baudrate(msg);
    spreader_status->spreader_sequence = mavlink_msg_spreader_status_get_spreader_sequence(msg);
    spreader_status->spreader_firmware_version = mavlink_msg_spreader_status_get_spreader_firmware_version(msg);
    spreader_status->spreader_servo_angle = mavlink_msg_spreader_status_get_spreader_servo_angle(msg);
    spreader_status->spreader_sensor_status = mavlink_msg_spreader_status_get_spreader_sensor_status(msg);
    spreader_status->spreader_can_enable = mavlink_msg_spreader_status_get_spreader_can_enable(msg);
    spreader_status->spreader_speed = mavlink_msg_spreader_status_get_spreader_speed(msg);
    spreader_status->spreader_function_status = mavlink_msg_spreader_status_get_spreader_function_status(msg);
    spreader_status->spreader_life_signal = mavlink_msg_spreader_status_get_spreader_life_signal(msg);
    spreader_status->spreader_year = mavlink_msg_spreader_status_get_spreader_year(msg);
    spreader_status->spreader_month = mavlink_msg_spreader_status_get_spreader_month(msg);
    spreader_status->spreader_day = mavlink_msg_spreader_status_get_spreader_day(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_SPREADER_STATUS_LEN? msg->len : MAVLINK_MSG_ID_SPREADER_STATUS_LEN;
        memset(spreader_status, 0, MAVLINK_MSG_ID_SPREADER_STATUS_LEN);
    memcpy(spreader_status, _MAV_PAYLOAD(msg), len);
#endif
}
