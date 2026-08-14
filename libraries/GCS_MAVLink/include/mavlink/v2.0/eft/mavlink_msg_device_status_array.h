#pragma once
// MESSAGE DEVICE_STATUS_ARRAY PACKING

#define MAVLINK_MSG_ID_DEVICE_STATUS_ARRAY 501


typedef struct __mavlink_device_status_array_t {
 uint16_t baud_rate_0; /*<  Device 0 CAN bus baud rate*/
 uint16_t speed_0; /*<  Device 0 rotation speed*/
 uint16_t baud_rate_1; /*<  Device 1 CAN bus baud rate*/
 uint16_t speed_1; /*<  Device 1 rotation speed*/
 uint16_t baud_rate_2; /*<  Device 2 CAN bus baud rate*/
 uint16_t speed_2; /*<  Device 2 rotation speed*/
 uint16_t baud_rate_3; /*<  Device 3 CAN bus baud rate*/
 uint16_t speed_3; /*<  Device 3 rotation speed*/
 uint16_t baud_rate_4; /*<  Device 4 CAN bus baud rate*/
 uint16_t speed_4; /*<  Device 4 rotation speed*/
 uint16_t baud_rate_5; /*<  Device 5 CAN bus baud rate*/
 uint16_t speed_5; /*<  Device 5 rotation speed*/
 uint8_t fault_status_0; /*<  Device 0 fault status flags*/
 uint8_t control_mode_0; /*<  Device 0 control mode*/
 uint8_t reserved_0; /*<  Device 0 reserved field*/
 uint8_t life_signal_0; /*<  Device 0 life signal counter*/
 uint8_t fault_status_1; /*<  Device 1 fault status flags*/
 uint8_t control_mode_1; /*<  Device 1 control mode*/
 uint8_t reserved_1; /*<  Device 1 reserved field*/
 uint8_t life_signal_1; /*<  Device 1 life signal counter*/
 uint8_t fault_status_2; /*<  Device 2 fault status flags*/
 uint8_t control_mode_2; /*<  Device 2 control mode*/
 uint8_t reserved_2; /*<  Device 2 reserved field*/
 uint8_t life_signal_2; /*<  Device 2 life signal counter*/
 uint8_t fault_status_3; /*<  Device 3 fault status flags*/
 uint8_t control_mode_3; /*<  Device 3 control mode*/
 uint8_t reserved_3; /*<  Device 3 reserved field*/
 uint8_t life_signal_3; /*<  Device 3 life signal counter*/
 uint8_t fault_status_4; /*<  Device 4 fault status flags*/
 uint8_t control_mode_4; /*<  Device 4 control mode*/
 uint8_t reserved_4; /*<  Device 4 reserved field*/
 uint8_t life_signal_4; /*<  Device 4 life signal counter*/
 uint8_t fault_status_5; /*<  Device 5 fault status flags*/
 uint8_t control_mode_5; /*<  Device 5 control mode*/
 uint8_t reserved_5; /*<  Device 5 reserved field*/
 uint8_t life_signal_5; /*<  Device 5 life signal counter*/
} mavlink_device_status_array_t;

#define MAVLINK_MSG_ID_DEVICE_STATUS_ARRAY_LEN 48
#define MAVLINK_MSG_ID_DEVICE_STATUS_ARRAY_MIN_LEN 48
#define MAVLINK_MSG_ID_501_LEN 48
#define MAVLINK_MSG_ID_501_MIN_LEN 48

#define MAVLINK_MSG_ID_DEVICE_STATUS_ARRAY_CRC 137
#define MAVLINK_MSG_ID_501_CRC 137



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_DEVICE_STATUS_ARRAY { \
    501, \
    "DEVICE_STATUS_ARRAY", \
    36, \
    {  { "fault_status_0", NULL, MAVLINK_TYPE_UINT8_T, 0, 24, offsetof(mavlink_device_status_array_t, fault_status_0) }, \
         { "control_mode_0", NULL, MAVLINK_TYPE_UINT8_T, 0, 25, offsetof(mavlink_device_status_array_t, control_mode_0) }, \
         { "baud_rate_0", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_device_status_array_t, baud_rate_0) }, \
         { "speed_0", NULL, MAVLINK_TYPE_UINT16_T, 0, 2, offsetof(mavlink_device_status_array_t, speed_0) }, \
         { "reserved_0", NULL, MAVLINK_TYPE_UINT8_T, 0, 26, offsetof(mavlink_device_status_array_t, reserved_0) }, \
         { "life_signal_0", NULL, MAVLINK_TYPE_UINT8_T, 0, 27, offsetof(mavlink_device_status_array_t, life_signal_0) }, \
         { "fault_status_1", NULL, MAVLINK_TYPE_UINT8_T, 0, 28, offsetof(mavlink_device_status_array_t, fault_status_1) }, \
         { "control_mode_1", NULL, MAVLINK_TYPE_UINT8_T, 0, 29, offsetof(mavlink_device_status_array_t, control_mode_1) }, \
         { "baud_rate_1", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_device_status_array_t, baud_rate_1) }, \
         { "speed_1", NULL, MAVLINK_TYPE_UINT16_T, 0, 6, offsetof(mavlink_device_status_array_t, speed_1) }, \
         { "reserved_1", NULL, MAVLINK_TYPE_UINT8_T, 0, 30, offsetof(mavlink_device_status_array_t, reserved_1) }, \
         { "life_signal_1", NULL, MAVLINK_TYPE_UINT8_T, 0, 31, offsetof(mavlink_device_status_array_t, life_signal_1) }, \
         { "fault_status_2", NULL, MAVLINK_TYPE_UINT8_T, 0, 32, offsetof(mavlink_device_status_array_t, fault_status_2) }, \
         { "control_mode_2", NULL, MAVLINK_TYPE_UINT8_T, 0, 33, offsetof(mavlink_device_status_array_t, control_mode_2) }, \
         { "baud_rate_2", NULL, MAVLINK_TYPE_UINT16_T, 0, 8, offsetof(mavlink_device_status_array_t, baud_rate_2) }, \
         { "speed_2", NULL, MAVLINK_TYPE_UINT16_T, 0, 10, offsetof(mavlink_device_status_array_t, speed_2) }, \
         { "reserved_2", NULL, MAVLINK_TYPE_UINT8_T, 0, 34, offsetof(mavlink_device_status_array_t, reserved_2) }, \
         { "life_signal_2", NULL, MAVLINK_TYPE_UINT8_T, 0, 35, offsetof(mavlink_device_status_array_t, life_signal_2) }, \
         { "fault_status_3", NULL, MAVLINK_TYPE_UINT8_T, 0, 36, offsetof(mavlink_device_status_array_t, fault_status_3) }, \
         { "control_mode_3", NULL, MAVLINK_TYPE_UINT8_T, 0, 37, offsetof(mavlink_device_status_array_t, control_mode_3) }, \
         { "baud_rate_3", NULL, MAVLINK_TYPE_UINT16_T, 0, 12, offsetof(mavlink_device_status_array_t, baud_rate_3) }, \
         { "speed_3", NULL, MAVLINK_TYPE_UINT16_T, 0, 14, offsetof(mavlink_device_status_array_t, speed_3) }, \
         { "reserved_3", NULL, MAVLINK_TYPE_UINT8_T, 0, 38, offsetof(mavlink_device_status_array_t, reserved_3) }, \
         { "life_signal_3", NULL, MAVLINK_TYPE_UINT8_T, 0, 39, offsetof(mavlink_device_status_array_t, life_signal_3) }, \
         { "fault_status_4", NULL, MAVLINK_TYPE_UINT8_T, 0, 40, offsetof(mavlink_device_status_array_t, fault_status_4) }, \
         { "control_mode_4", NULL, MAVLINK_TYPE_UINT8_T, 0, 41, offsetof(mavlink_device_status_array_t, control_mode_4) }, \
         { "baud_rate_4", NULL, MAVLINK_TYPE_UINT16_T, 0, 16, offsetof(mavlink_device_status_array_t, baud_rate_4) }, \
         { "speed_4", NULL, MAVLINK_TYPE_UINT16_T, 0, 18, offsetof(mavlink_device_status_array_t, speed_4) }, \
         { "reserved_4", NULL, MAVLINK_TYPE_UINT8_T, 0, 42, offsetof(mavlink_device_status_array_t, reserved_4) }, \
         { "life_signal_4", NULL, MAVLINK_TYPE_UINT8_T, 0, 43, offsetof(mavlink_device_status_array_t, life_signal_4) }, \
         { "fault_status_5", NULL, MAVLINK_TYPE_UINT8_T, 0, 44, offsetof(mavlink_device_status_array_t, fault_status_5) }, \
         { "control_mode_5", NULL, MAVLINK_TYPE_UINT8_T, 0, 45, offsetof(mavlink_device_status_array_t, control_mode_5) }, \
         { "baud_rate_5", NULL, MAVLINK_TYPE_UINT16_T, 0, 20, offsetof(mavlink_device_status_array_t, baud_rate_5) }, \
         { "speed_5", NULL, MAVLINK_TYPE_UINT16_T, 0, 22, offsetof(mavlink_device_status_array_t, speed_5) }, \
         { "reserved_5", NULL, MAVLINK_TYPE_UINT8_T, 0, 46, offsetof(mavlink_device_status_array_t, reserved_5) }, \
         { "life_signal_5", NULL, MAVLINK_TYPE_UINT8_T, 0, 47, offsetof(mavlink_device_status_array_t, life_signal_5) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_DEVICE_STATUS_ARRAY { \
    "DEVICE_STATUS_ARRAY", \
    36, \
    {  { "fault_status_0", NULL, MAVLINK_TYPE_UINT8_T, 0, 24, offsetof(mavlink_device_status_array_t, fault_status_0) }, \
         { "control_mode_0", NULL, MAVLINK_TYPE_UINT8_T, 0, 25, offsetof(mavlink_device_status_array_t, control_mode_0) }, \
         { "baud_rate_0", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_device_status_array_t, baud_rate_0) }, \
         { "speed_0", NULL, MAVLINK_TYPE_UINT16_T, 0, 2, offsetof(mavlink_device_status_array_t, speed_0) }, \
         { "reserved_0", NULL, MAVLINK_TYPE_UINT8_T, 0, 26, offsetof(mavlink_device_status_array_t, reserved_0) }, \
         { "life_signal_0", NULL, MAVLINK_TYPE_UINT8_T, 0, 27, offsetof(mavlink_device_status_array_t, life_signal_0) }, \
         { "fault_status_1", NULL, MAVLINK_TYPE_UINT8_T, 0, 28, offsetof(mavlink_device_status_array_t, fault_status_1) }, \
         { "control_mode_1", NULL, MAVLINK_TYPE_UINT8_T, 0, 29, offsetof(mavlink_device_status_array_t, control_mode_1) }, \
         { "baud_rate_1", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_device_status_array_t, baud_rate_1) }, \
         { "speed_1", NULL, MAVLINK_TYPE_UINT16_T, 0, 6, offsetof(mavlink_device_status_array_t, speed_1) }, \
         { "reserved_1", NULL, MAVLINK_TYPE_UINT8_T, 0, 30, offsetof(mavlink_device_status_array_t, reserved_1) }, \
         { "life_signal_1", NULL, MAVLINK_TYPE_UINT8_T, 0, 31, offsetof(mavlink_device_status_array_t, life_signal_1) }, \
         { "fault_status_2", NULL, MAVLINK_TYPE_UINT8_T, 0, 32, offsetof(mavlink_device_status_array_t, fault_status_2) }, \
         { "control_mode_2", NULL, MAVLINK_TYPE_UINT8_T, 0, 33, offsetof(mavlink_device_status_array_t, control_mode_2) }, \
         { "baud_rate_2", NULL, MAVLINK_TYPE_UINT16_T, 0, 8, offsetof(mavlink_device_status_array_t, baud_rate_2) }, \
         { "speed_2", NULL, MAVLINK_TYPE_UINT16_T, 0, 10, offsetof(mavlink_device_status_array_t, speed_2) }, \
         { "reserved_2", NULL, MAVLINK_TYPE_UINT8_T, 0, 34, offsetof(mavlink_device_status_array_t, reserved_2) }, \
         { "life_signal_2", NULL, MAVLINK_TYPE_UINT8_T, 0, 35, offsetof(mavlink_device_status_array_t, life_signal_2) }, \
         { "fault_status_3", NULL, MAVLINK_TYPE_UINT8_T, 0, 36, offsetof(mavlink_device_status_array_t, fault_status_3) }, \
         { "control_mode_3", NULL, MAVLINK_TYPE_UINT8_T, 0, 37, offsetof(mavlink_device_status_array_t, control_mode_3) }, \
         { "baud_rate_3", NULL, MAVLINK_TYPE_UINT16_T, 0, 12, offsetof(mavlink_device_status_array_t, baud_rate_3) }, \
         { "speed_3", NULL, MAVLINK_TYPE_UINT16_T, 0, 14, offsetof(mavlink_device_status_array_t, speed_3) }, \
         { "reserved_3", NULL, MAVLINK_TYPE_UINT8_T, 0, 38, offsetof(mavlink_device_status_array_t, reserved_3) }, \
         { "life_signal_3", NULL, MAVLINK_TYPE_UINT8_T, 0, 39, offsetof(mavlink_device_status_array_t, life_signal_3) }, \
         { "fault_status_4", NULL, MAVLINK_TYPE_UINT8_T, 0, 40, offsetof(mavlink_device_status_array_t, fault_status_4) }, \
         { "control_mode_4", NULL, MAVLINK_TYPE_UINT8_T, 0, 41, offsetof(mavlink_device_status_array_t, control_mode_4) }, \
         { "baud_rate_4", NULL, MAVLINK_TYPE_UINT16_T, 0, 16, offsetof(mavlink_device_status_array_t, baud_rate_4) }, \
         { "speed_4", NULL, MAVLINK_TYPE_UINT16_T, 0, 18, offsetof(mavlink_device_status_array_t, speed_4) }, \
         { "reserved_4", NULL, MAVLINK_TYPE_UINT8_T, 0, 42, offsetof(mavlink_device_status_array_t, reserved_4) }, \
         { "life_signal_4", NULL, MAVLINK_TYPE_UINT8_T, 0, 43, offsetof(mavlink_device_status_array_t, life_signal_4) }, \
         { "fault_status_5", NULL, MAVLINK_TYPE_UINT8_T, 0, 44, offsetof(mavlink_device_status_array_t, fault_status_5) }, \
         { "control_mode_5", NULL, MAVLINK_TYPE_UINT8_T, 0, 45, offsetof(mavlink_device_status_array_t, control_mode_5) }, \
         { "baud_rate_5", NULL, MAVLINK_TYPE_UINT16_T, 0, 20, offsetof(mavlink_device_status_array_t, baud_rate_5) }, \
         { "speed_5", NULL, MAVLINK_TYPE_UINT16_T, 0, 22, offsetof(mavlink_device_status_array_t, speed_5) }, \
         { "reserved_5", NULL, MAVLINK_TYPE_UINT8_T, 0, 46, offsetof(mavlink_device_status_array_t, reserved_5) }, \
         { "life_signal_5", NULL, MAVLINK_TYPE_UINT8_T, 0, 47, offsetof(mavlink_device_status_array_t, life_signal_5) }, \
         } \
}
#endif

/**
 * @brief Pack a device_status_array message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param fault_status_0  Device 0 fault status flags
 * @param control_mode_0  Device 0 control mode
 * @param baud_rate_0  Device 0 CAN bus baud rate
 * @param speed_0  Device 0 rotation speed
 * @param reserved_0  Device 0 reserved field
 * @param life_signal_0  Device 0 life signal counter
 * @param fault_status_1  Device 1 fault status flags
 * @param control_mode_1  Device 1 control mode
 * @param baud_rate_1  Device 1 CAN bus baud rate
 * @param speed_1  Device 1 rotation speed
 * @param reserved_1  Device 1 reserved field
 * @param life_signal_1  Device 1 life signal counter
 * @param fault_status_2  Device 2 fault status flags
 * @param control_mode_2  Device 2 control mode
 * @param baud_rate_2  Device 2 CAN bus baud rate
 * @param speed_2  Device 2 rotation speed
 * @param reserved_2  Device 2 reserved field
 * @param life_signal_2  Device 2 life signal counter
 * @param fault_status_3  Device 3 fault status flags
 * @param control_mode_3  Device 3 control mode
 * @param baud_rate_3  Device 3 CAN bus baud rate
 * @param speed_3  Device 3 rotation speed
 * @param reserved_3  Device 3 reserved field
 * @param life_signal_3  Device 3 life signal counter
 * @param fault_status_4  Device 4 fault status flags
 * @param control_mode_4  Device 4 control mode
 * @param baud_rate_4  Device 4 CAN bus baud rate
 * @param speed_4  Device 4 rotation speed
 * @param reserved_4  Device 4 reserved field
 * @param life_signal_4  Device 4 life signal counter
 * @param fault_status_5  Device 5 fault status flags
 * @param control_mode_5  Device 5 control mode
 * @param baud_rate_5  Device 5 CAN bus baud rate
 * @param speed_5  Device 5 rotation speed
 * @param reserved_5  Device 5 reserved field
 * @param life_signal_5  Device 5 life signal counter
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_device_status_array_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t fault_status_0, uint8_t control_mode_0, uint16_t baud_rate_0, uint16_t speed_0, uint8_t reserved_0, uint8_t life_signal_0, uint8_t fault_status_1, uint8_t control_mode_1, uint16_t baud_rate_1, uint16_t speed_1, uint8_t reserved_1, uint8_t life_signal_1, uint8_t fault_status_2, uint8_t control_mode_2, uint16_t baud_rate_2, uint16_t speed_2, uint8_t reserved_2, uint8_t life_signal_2, uint8_t fault_status_3, uint8_t control_mode_3, uint16_t baud_rate_3, uint16_t speed_3, uint8_t reserved_3, uint8_t life_signal_3, uint8_t fault_status_4, uint8_t control_mode_4, uint16_t baud_rate_4, uint16_t speed_4, uint8_t reserved_4, uint8_t life_signal_4, uint8_t fault_status_5, uint8_t control_mode_5, uint16_t baud_rate_5, uint16_t speed_5, uint8_t reserved_5, uint8_t life_signal_5)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_DEVICE_STATUS_ARRAY_LEN];
    _mav_put_uint16_t(buf, 0, baud_rate_0);
    _mav_put_uint16_t(buf, 2, speed_0);
    _mav_put_uint16_t(buf, 4, baud_rate_1);
    _mav_put_uint16_t(buf, 6, speed_1);
    _mav_put_uint16_t(buf, 8, baud_rate_2);
    _mav_put_uint16_t(buf, 10, speed_2);
    _mav_put_uint16_t(buf, 12, baud_rate_3);
    _mav_put_uint16_t(buf, 14, speed_3);
    _mav_put_uint16_t(buf, 16, baud_rate_4);
    _mav_put_uint16_t(buf, 18, speed_4);
    _mav_put_uint16_t(buf, 20, baud_rate_5);
    _mav_put_uint16_t(buf, 22, speed_5);
    _mav_put_uint8_t(buf, 24, fault_status_0);
    _mav_put_uint8_t(buf, 25, control_mode_0);
    _mav_put_uint8_t(buf, 26, reserved_0);
    _mav_put_uint8_t(buf, 27, life_signal_0);
    _mav_put_uint8_t(buf, 28, fault_status_1);
    _mav_put_uint8_t(buf, 29, control_mode_1);
    _mav_put_uint8_t(buf, 30, reserved_1);
    _mav_put_uint8_t(buf, 31, life_signal_1);
    _mav_put_uint8_t(buf, 32, fault_status_2);
    _mav_put_uint8_t(buf, 33, control_mode_2);
    _mav_put_uint8_t(buf, 34, reserved_2);
    _mav_put_uint8_t(buf, 35, life_signal_2);
    _mav_put_uint8_t(buf, 36, fault_status_3);
    _mav_put_uint8_t(buf, 37, control_mode_3);
    _mav_put_uint8_t(buf, 38, reserved_3);
    _mav_put_uint8_t(buf, 39, life_signal_3);
    _mav_put_uint8_t(buf, 40, fault_status_4);
    _mav_put_uint8_t(buf, 41, control_mode_4);
    _mav_put_uint8_t(buf, 42, reserved_4);
    _mav_put_uint8_t(buf, 43, life_signal_4);
    _mav_put_uint8_t(buf, 44, fault_status_5);
    _mav_put_uint8_t(buf, 45, control_mode_5);
    _mav_put_uint8_t(buf, 46, reserved_5);
    _mav_put_uint8_t(buf, 47, life_signal_5);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_DEVICE_STATUS_ARRAY_LEN);
#else
    mavlink_device_status_array_t packet;
    packet.baud_rate_0 = baud_rate_0;
    packet.speed_0 = speed_0;
    packet.baud_rate_1 = baud_rate_1;
    packet.speed_1 = speed_1;
    packet.baud_rate_2 = baud_rate_2;
    packet.speed_2 = speed_2;
    packet.baud_rate_3 = baud_rate_3;
    packet.speed_3 = speed_3;
    packet.baud_rate_4 = baud_rate_4;
    packet.speed_4 = speed_4;
    packet.baud_rate_5 = baud_rate_5;
    packet.speed_5 = speed_5;
    packet.fault_status_0 = fault_status_0;
    packet.control_mode_0 = control_mode_0;
    packet.reserved_0 = reserved_0;
    packet.life_signal_0 = life_signal_0;
    packet.fault_status_1 = fault_status_1;
    packet.control_mode_1 = control_mode_1;
    packet.reserved_1 = reserved_1;
    packet.life_signal_1 = life_signal_1;
    packet.fault_status_2 = fault_status_2;
    packet.control_mode_2 = control_mode_2;
    packet.reserved_2 = reserved_2;
    packet.life_signal_2 = life_signal_2;
    packet.fault_status_3 = fault_status_3;
    packet.control_mode_3 = control_mode_3;
    packet.reserved_3 = reserved_3;
    packet.life_signal_3 = life_signal_3;
    packet.fault_status_4 = fault_status_4;
    packet.control_mode_4 = control_mode_4;
    packet.reserved_4 = reserved_4;
    packet.life_signal_4 = life_signal_4;
    packet.fault_status_5 = fault_status_5;
    packet.control_mode_5 = control_mode_5;
    packet.reserved_5 = reserved_5;
    packet.life_signal_5 = life_signal_5;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_DEVICE_STATUS_ARRAY_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_DEVICE_STATUS_ARRAY;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_DEVICE_STATUS_ARRAY_MIN_LEN, MAVLINK_MSG_ID_DEVICE_STATUS_ARRAY_LEN, MAVLINK_MSG_ID_DEVICE_STATUS_ARRAY_CRC);
}

/**
 * @brief Pack a device_status_array message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param fault_status_0  Device 0 fault status flags
 * @param control_mode_0  Device 0 control mode
 * @param baud_rate_0  Device 0 CAN bus baud rate
 * @param speed_0  Device 0 rotation speed
 * @param reserved_0  Device 0 reserved field
 * @param life_signal_0  Device 0 life signal counter
 * @param fault_status_1  Device 1 fault status flags
 * @param control_mode_1  Device 1 control mode
 * @param baud_rate_1  Device 1 CAN bus baud rate
 * @param speed_1  Device 1 rotation speed
 * @param reserved_1  Device 1 reserved field
 * @param life_signal_1  Device 1 life signal counter
 * @param fault_status_2  Device 2 fault status flags
 * @param control_mode_2  Device 2 control mode
 * @param baud_rate_2  Device 2 CAN bus baud rate
 * @param speed_2  Device 2 rotation speed
 * @param reserved_2  Device 2 reserved field
 * @param life_signal_2  Device 2 life signal counter
 * @param fault_status_3  Device 3 fault status flags
 * @param control_mode_3  Device 3 control mode
 * @param baud_rate_3  Device 3 CAN bus baud rate
 * @param speed_3  Device 3 rotation speed
 * @param reserved_3  Device 3 reserved field
 * @param life_signal_3  Device 3 life signal counter
 * @param fault_status_4  Device 4 fault status flags
 * @param control_mode_4  Device 4 control mode
 * @param baud_rate_4  Device 4 CAN bus baud rate
 * @param speed_4  Device 4 rotation speed
 * @param reserved_4  Device 4 reserved field
 * @param life_signal_4  Device 4 life signal counter
 * @param fault_status_5  Device 5 fault status flags
 * @param control_mode_5  Device 5 control mode
 * @param baud_rate_5  Device 5 CAN bus baud rate
 * @param speed_5  Device 5 rotation speed
 * @param reserved_5  Device 5 reserved field
 * @param life_signal_5  Device 5 life signal counter
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_device_status_array_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint8_t fault_status_0, uint8_t control_mode_0, uint16_t baud_rate_0, uint16_t speed_0, uint8_t reserved_0, uint8_t life_signal_0, uint8_t fault_status_1, uint8_t control_mode_1, uint16_t baud_rate_1, uint16_t speed_1, uint8_t reserved_1, uint8_t life_signal_1, uint8_t fault_status_2, uint8_t control_mode_2, uint16_t baud_rate_2, uint16_t speed_2, uint8_t reserved_2, uint8_t life_signal_2, uint8_t fault_status_3, uint8_t control_mode_3, uint16_t baud_rate_3, uint16_t speed_3, uint8_t reserved_3, uint8_t life_signal_3, uint8_t fault_status_4, uint8_t control_mode_4, uint16_t baud_rate_4, uint16_t speed_4, uint8_t reserved_4, uint8_t life_signal_4, uint8_t fault_status_5, uint8_t control_mode_5, uint16_t baud_rate_5, uint16_t speed_5, uint8_t reserved_5, uint8_t life_signal_5)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_DEVICE_STATUS_ARRAY_LEN];
    _mav_put_uint16_t(buf, 0, baud_rate_0);
    _mav_put_uint16_t(buf, 2, speed_0);
    _mav_put_uint16_t(buf, 4, baud_rate_1);
    _mav_put_uint16_t(buf, 6, speed_1);
    _mav_put_uint16_t(buf, 8, baud_rate_2);
    _mav_put_uint16_t(buf, 10, speed_2);
    _mav_put_uint16_t(buf, 12, baud_rate_3);
    _mav_put_uint16_t(buf, 14, speed_3);
    _mav_put_uint16_t(buf, 16, baud_rate_4);
    _mav_put_uint16_t(buf, 18, speed_4);
    _mav_put_uint16_t(buf, 20, baud_rate_5);
    _mav_put_uint16_t(buf, 22, speed_5);
    _mav_put_uint8_t(buf, 24, fault_status_0);
    _mav_put_uint8_t(buf, 25, control_mode_0);
    _mav_put_uint8_t(buf, 26, reserved_0);
    _mav_put_uint8_t(buf, 27, life_signal_0);
    _mav_put_uint8_t(buf, 28, fault_status_1);
    _mav_put_uint8_t(buf, 29, control_mode_1);
    _mav_put_uint8_t(buf, 30, reserved_1);
    _mav_put_uint8_t(buf, 31, life_signal_1);
    _mav_put_uint8_t(buf, 32, fault_status_2);
    _mav_put_uint8_t(buf, 33, control_mode_2);
    _mav_put_uint8_t(buf, 34, reserved_2);
    _mav_put_uint8_t(buf, 35, life_signal_2);
    _mav_put_uint8_t(buf, 36, fault_status_3);
    _mav_put_uint8_t(buf, 37, control_mode_3);
    _mav_put_uint8_t(buf, 38, reserved_3);
    _mav_put_uint8_t(buf, 39, life_signal_3);
    _mav_put_uint8_t(buf, 40, fault_status_4);
    _mav_put_uint8_t(buf, 41, control_mode_4);
    _mav_put_uint8_t(buf, 42, reserved_4);
    _mav_put_uint8_t(buf, 43, life_signal_4);
    _mav_put_uint8_t(buf, 44, fault_status_5);
    _mav_put_uint8_t(buf, 45, control_mode_5);
    _mav_put_uint8_t(buf, 46, reserved_5);
    _mav_put_uint8_t(buf, 47, life_signal_5);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_DEVICE_STATUS_ARRAY_LEN);
#else
    mavlink_device_status_array_t packet;
    packet.baud_rate_0 = baud_rate_0;
    packet.speed_0 = speed_0;
    packet.baud_rate_1 = baud_rate_1;
    packet.speed_1 = speed_1;
    packet.baud_rate_2 = baud_rate_2;
    packet.speed_2 = speed_2;
    packet.baud_rate_3 = baud_rate_3;
    packet.speed_3 = speed_3;
    packet.baud_rate_4 = baud_rate_4;
    packet.speed_4 = speed_4;
    packet.baud_rate_5 = baud_rate_5;
    packet.speed_5 = speed_5;
    packet.fault_status_0 = fault_status_0;
    packet.control_mode_0 = control_mode_0;
    packet.reserved_0 = reserved_0;
    packet.life_signal_0 = life_signal_0;
    packet.fault_status_1 = fault_status_1;
    packet.control_mode_1 = control_mode_1;
    packet.reserved_1 = reserved_1;
    packet.life_signal_1 = life_signal_1;
    packet.fault_status_2 = fault_status_2;
    packet.control_mode_2 = control_mode_2;
    packet.reserved_2 = reserved_2;
    packet.life_signal_2 = life_signal_2;
    packet.fault_status_3 = fault_status_3;
    packet.control_mode_3 = control_mode_3;
    packet.reserved_3 = reserved_3;
    packet.life_signal_3 = life_signal_3;
    packet.fault_status_4 = fault_status_4;
    packet.control_mode_4 = control_mode_4;
    packet.reserved_4 = reserved_4;
    packet.life_signal_4 = life_signal_4;
    packet.fault_status_5 = fault_status_5;
    packet.control_mode_5 = control_mode_5;
    packet.reserved_5 = reserved_5;
    packet.life_signal_5 = life_signal_5;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_DEVICE_STATUS_ARRAY_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_DEVICE_STATUS_ARRAY;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_DEVICE_STATUS_ARRAY_MIN_LEN, MAVLINK_MSG_ID_DEVICE_STATUS_ARRAY_LEN, MAVLINK_MSG_ID_DEVICE_STATUS_ARRAY_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_DEVICE_STATUS_ARRAY_MIN_LEN, MAVLINK_MSG_ID_DEVICE_STATUS_ARRAY_LEN);
#endif
}

/**
 * @brief Pack a device_status_array message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param fault_status_0  Device 0 fault status flags
 * @param control_mode_0  Device 0 control mode
 * @param baud_rate_0  Device 0 CAN bus baud rate
 * @param speed_0  Device 0 rotation speed
 * @param reserved_0  Device 0 reserved field
 * @param life_signal_0  Device 0 life signal counter
 * @param fault_status_1  Device 1 fault status flags
 * @param control_mode_1  Device 1 control mode
 * @param baud_rate_1  Device 1 CAN bus baud rate
 * @param speed_1  Device 1 rotation speed
 * @param reserved_1  Device 1 reserved field
 * @param life_signal_1  Device 1 life signal counter
 * @param fault_status_2  Device 2 fault status flags
 * @param control_mode_2  Device 2 control mode
 * @param baud_rate_2  Device 2 CAN bus baud rate
 * @param speed_2  Device 2 rotation speed
 * @param reserved_2  Device 2 reserved field
 * @param life_signal_2  Device 2 life signal counter
 * @param fault_status_3  Device 3 fault status flags
 * @param control_mode_3  Device 3 control mode
 * @param baud_rate_3  Device 3 CAN bus baud rate
 * @param speed_3  Device 3 rotation speed
 * @param reserved_3  Device 3 reserved field
 * @param life_signal_3  Device 3 life signal counter
 * @param fault_status_4  Device 4 fault status flags
 * @param control_mode_4  Device 4 control mode
 * @param baud_rate_4  Device 4 CAN bus baud rate
 * @param speed_4  Device 4 rotation speed
 * @param reserved_4  Device 4 reserved field
 * @param life_signal_4  Device 4 life signal counter
 * @param fault_status_5  Device 5 fault status flags
 * @param control_mode_5  Device 5 control mode
 * @param baud_rate_5  Device 5 CAN bus baud rate
 * @param speed_5  Device 5 rotation speed
 * @param reserved_5  Device 5 reserved field
 * @param life_signal_5  Device 5 life signal counter
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_device_status_array_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t fault_status_0,uint8_t control_mode_0,uint16_t baud_rate_0,uint16_t speed_0,uint8_t reserved_0,uint8_t life_signal_0,uint8_t fault_status_1,uint8_t control_mode_1,uint16_t baud_rate_1,uint16_t speed_1,uint8_t reserved_1,uint8_t life_signal_1,uint8_t fault_status_2,uint8_t control_mode_2,uint16_t baud_rate_2,uint16_t speed_2,uint8_t reserved_2,uint8_t life_signal_2,uint8_t fault_status_3,uint8_t control_mode_3,uint16_t baud_rate_3,uint16_t speed_3,uint8_t reserved_3,uint8_t life_signal_3,uint8_t fault_status_4,uint8_t control_mode_4,uint16_t baud_rate_4,uint16_t speed_4,uint8_t reserved_4,uint8_t life_signal_4,uint8_t fault_status_5,uint8_t control_mode_5,uint16_t baud_rate_5,uint16_t speed_5,uint8_t reserved_5,uint8_t life_signal_5)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_DEVICE_STATUS_ARRAY_LEN];
    _mav_put_uint16_t(buf, 0, baud_rate_0);
    _mav_put_uint16_t(buf, 2, speed_0);
    _mav_put_uint16_t(buf, 4, baud_rate_1);
    _mav_put_uint16_t(buf, 6, speed_1);
    _mav_put_uint16_t(buf, 8, baud_rate_2);
    _mav_put_uint16_t(buf, 10, speed_2);
    _mav_put_uint16_t(buf, 12, baud_rate_3);
    _mav_put_uint16_t(buf, 14, speed_3);
    _mav_put_uint16_t(buf, 16, baud_rate_4);
    _mav_put_uint16_t(buf, 18, speed_4);
    _mav_put_uint16_t(buf, 20, baud_rate_5);
    _mav_put_uint16_t(buf, 22, speed_5);
    _mav_put_uint8_t(buf, 24, fault_status_0);
    _mav_put_uint8_t(buf, 25, control_mode_0);
    _mav_put_uint8_t(buf, 26, reserved_0);
    _mav_put_uint8_t(buf, 27, life_signal_0);
    _mav_put_uint8_t(buf, 28, fault_status_1);
    _mav_put_uint8_t(buf, 29, control_mode_1);
    _mav_put_uint8_t(buf, 30, reserved_1);
    _mav_put_uint8_t(buf, 31, life_signal_1);
    _mav_put_uint8_t(buf, 32, fault_status_2);
    _mav_put_uint8_t(buf, 33, control_mode_2);
    _mav_put_uint8_t(buf, 34, reserved_2);
    _mav_put_uint8_t(buf, 35, life_signal_2);
    _mav_put_uint8_t(buf, 36, fault_status_3);
    _mav_put_uint8_t(buf, 37, control_mode_3);
    _mav_put_uint8_t(buf, 38, reserved_3);
    _mav_put_uint8_t(buf, 39, life_signal_3);
    _mav_put_uint8_t(buf, 40, fault_status_4);
    _mav_put_uint8_t(buf, 41, control_mode_4);
    _mav_put_uint8_t(buf, 42, reserved_4);
    _mav_put_uint8_t(buf, 43, life_signal_4);
    _mav_put_uint8_t(buf, 44, fault_status_5);
    _mav_put_uint8_t(buf, 45, control_mode_5);
    _mav_put_uint8_t(buf, 46, reserved_5);
    _mav_put_uint8_t(buf, 47, life_signal_5);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_DEVICE_STATUS_ARRAY_LEN);
#else
    mavlink_device_status_array_t packet;
    packet.baud_rate_0 = baud_rate_0;
    packet.speed_0 = speed_0;
    packet.baud_rate_1 = baud_rate_1;
    packet.speed_1 = speed_1;
    packet.baud_rate_2 = baud_rate_2;
    packet.speed_2 = speed_2;
    packet.baud_rate_3 = baud_rate_3;
    packet.speed_3 = speed_3;
    packet.baud_rate_4 = baud_rate_4;
    packet.speed_4 = speed_4;
    packet.baud_rate_5 = baud_rate_5;
    packet.speed_5 = speed_5;
    packet.fault_status_0 = fault_status_0;
    packet.control_mode_0 = control_mode_0;
    packet.reserved_0 = reserved_0;
    packet.life_signal_0 = life_signal_0;
    packet.fault_status_1 = fault_status_1;
    packet.control_mode_1 = control_mode_1;
    packet.reserved_1 = reserved_1;
    packet.life_signal_1 = life_signal_1;
    packet.fault_status_2 = fault_status_2;
    packet.control_mode_2 = control_mode_2;
    packet.reserved_2 = reserved_2;
    packet.life_signal_2 = life_signal_2;
    packet.fault_status_3 = fault_status_3;
    packet.control_mode_3 = control_mode_3;
    packet.reserved_3 = reserved_3;
    packet.life_signal_3 = life_signal_3;
    packet.fault_status_4 = fault_status_4;
    packet.control_mode_4 = control_mode_4;
    packet.reserved_4 = reserved_4;
    packet.life_signal_4 = life_signal_4;
    packet.fault_status_5 = fault_status_5;
    packet.control_mode_5 = control_mode_5;
    packet.reserved_5 = reserved_5;
    packet.life_signal_5 = life_signal_5;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_DEVICE_STATUS_ARRAY_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_DEVICE_STATUS_ARRAY;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_DEVICE_STATUS_ARRAY_MIN_LEN, MAVLINK_MSG_ID_DEVICE_STATUS_ARRAY_LEN, MAVLINK_MSG_ID_DEVICE_STATUS_ARRAY_CRC);
}

/**
 * @brief Encode a device_status_array struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param device_status_array C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_device_status_array_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_device_status_array_t* device_status_array)
{
    return mavlink_msg_device_status_array_pack(system_id, component_id, msg, device_status_array->fault_status_0, device_status_array->control_mode_0, device_status_array->baud_rate_0, device_status_array->speed_0, device_status_array->reserved_0, device_status_array->life_signal_0, device_status_array->fault_status_1, device_status_array->control_mode_1, device_status_array->baud_rate_1, device_status_array->speed_1, device_status_array->reserved_1, device_status_array->life_signal_1, device_status_array->fault_status_2, device_status_array->control_mode_2, device_status_array->baud_rate_2, device_status_array->speed_2, device_status_array->reserved_2, device_status_array->life_signal_2, device_status_array->fault_status_3, device_status_array->control_mode_3, device_status_array->baud_rate_3, device_status_array->speed_3, device_status_array->reserved_3, device_status_array->life_signal_3, device_status_array->fault_status_4, device_status_array->control_mode_4, device_status_array->baud_rate_4, device_status_array->speed_4, device_status_array->reserved_4, device_status_array->life_signal_4, device_status_array->fault_status_5, device_status_array->control_mode_5, device_status_array->baud_rate_5, device_status_array->speed_5, device_status_array->reserved_5, device_status_array->life_signal_5);
}

/**
 * @brief Encode a device_status_array struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param device_status_array C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_device_status_array_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_device_status_array_t* device_status_array)
{
    return mavlink_msg_device_status_array_pack_chan(system_id, component_id, chan, msg, device_status_array->fault_status_0, device_status_array->control_mode_0, device_status_array->baud_rate_0, device_status_array->speed_0, device_status_array->reserved_0, device_status_array->life_signal_0, device_status_array->fault_status_1, device_status_array->control_mode_1, device_status_array->baud_rate_1, device_status_array->speed_1, device_status_array->reserved_1, device_status_array->life_signal_1, device_status_array->fault_status_2, device_status_array->control_mode_2, device_status_array->baud_rate_2, device_status_array->speed_2, device_status_array->reserved_2, device_status_array->life_signal_2, device_status_array->fault_status_3, device_status_array->control_mode_3, device_status_array->baud_rate_3, device_status_array->speed_3, device_status_array->reserved_3, device_status_array->life_signal_3, device_status_array->fault_status_4, device_status_array->control_mode_4, device_status_array->baud_rate_4, device_status_array->speed_4, device_status_array->reserved_4, device_status_array->life_signal_4, device_status_array->fault_status_5, device_status_array->control_mode_5, device_status_array->baud_rate_5, device_status_array->speed_5, device_status_array->reserved_5, device_status_array->life_signal_5);
}

/**
 * @brief Encode a device_status_array struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param device_status_array C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_device_status_array_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_device_status_array_t* device_status_array)
{
    return mavlink_msg_device_status_array_pack_status(system_id, component_id, _status, msg,  device_status_array->fault_status_0, device_status_array->control_mode_0, device_status_array->baud_rate_0, device_status_array->speed_0, device_status_array->reserved_0, device_status_array->life_signal_0, device_status_array->fault_status_1, device_status_array->control_mode_1, device_status_array->baud_rate_1, device_status_array->speed_1, device_status_array->reserved_1, device_status_array->life_signal_1, device_status_array->fault_status_2, device_status_array->control_mode_2, device_status_array->baud_rate_2, device_status_array->speed_2, device_status_array->reserved_2, device_status_array->life_signal_2, device_status_array->fault_status_3, device_status_array->control_mode_3, device_status_array->baud_rate_3, device_status_array->speed_3, device_status_array->reserved_3, device_status_array->life_signal_3, device_status_array->fault_status_4, device_status_array->control_mode_4, device_status_array->baud_rate_4, device_status_array->speed_4, device_status_array->reserved_4, device_status_array->life_signal_4, device_status_array->fault_status_5, device_status_array->control_mode_5, device_status_array->baud_rate_5, device_status_array->speed_5, device_status_array->reserved_5, device_status_array->life_signal_5);
}

/**
 * @brief Send a device_status_array message
 * @param chan MAVLink channel to send the message
 *
 * @param fault_status_0  Device 0 fault status flags
 * @param control_mode_0  Device 0 control mode
 * @param baud_rate_0  Device 0 CAN bus baud rate
 * @param speed_0  Device 0 rotation speed
 * @param reserved_0  Device 0 reserved field
 * @param life_signal_0  Device 0 life signal counter
 * @param fault_status_1  Device 1 fault status flags
 * @param control_mode_1  Device 1 control mode
 * @param baud_rate_1  Device 1 CAN bus baud rate
 * @param speed_1  Device 1 rotation speed
 * @param reserved_1  Device 1 reserved field
 * @param life_signal_1  Device 1 life signal counter
 * @param fault_status_2  Device 2 fault status flags
 * @param control_mode_2  Device 2 control mode
 * @param baud_rate_2  Device 2 CAN bus baud rate
 * @param speed_2  Device 2 rotation speed
 * @param reserved_2  Device 2 reserved field
 * @param life_signal_2  Device 2 life signal counter
 * @param fault_status_3  Device 3 fault status flags
 * @param control_mode_3  Device 3 control mode
 * @param baud_rate_3  Device 3 CAN bus baud rate
 * @param speed_3  Device 3 rotation speed
 * @param reserved_3  Device 3 reserved field
 * @param life_signal_3  Device 3 life signal counter
 * @param fault_status_4  Device 4 fault status flags
 * @param control_mode_4  Device 4 control mode
 * @param baud_rate_4  Device 4 CAN bus baud rate
 * @param speed_4  Device 4 rotation speed
 * @param reserved_4  Device 4 reserved field
 * @param life_signal_4  Device 4 life signal counter
 * @param fault_status_5  Device 5 fault status flags
 * @param control_mode_5  Device 5 control mode
 * @param baud_rate_5  Device 5 CAN bus baud rate
 * @param speed_5  Device 5 rotation speed
 * @param reserved_5  Device 5 reserved field
 * @param life_signal_5  Device 5 life signal counter
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_device_status_array_send(mavlink_channel_t chan, uint8_t fault_status_0, uint8_t control_mode_0, uint16_t baud_rate_0, uint16_t speed_0, uint8_t reserved_0, uint8_t life_signal_0, uint8_t fault_status_1, uint8_t control_mode_1, uint16_t baud_rate_1, uint16_t speed_1, uint8_t reserved_1, uint8_t life_signal_1, uint8_t fault_status_2, uint8_t control_mode_2, uint16_t baud_rate_2, uint16_t speed_2, uint8_t reserved_2, uint8_t life_signal_2, uint8_t fault_status_3, uint8_t control_mode_3, uint16_t baud_rate_3, uint16_t speed_3, uint8_t reserved_3, uint8_t life_signal_3, uint8_t fault_status_4, uint8_t control_mode_4, uint16_t baud_rate_4, uint16_t speed_4, uint8_t reserved_4, uint8_t life_signal_4, uint8_t fault_status_5, uint8_t control_mode_5, uint16_t baud_rate_5, uint16_t speed_5, uint8_t reserved_5, uint8_t life_signal_5)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_DEVICE_STATUS_ARRAY_LEN];
    _mav_put_uint16_t(buf, 0, baud_rate_0);
    _mav_put_uint16_t(buf, 2, speed_0);
    _mav_put_uint16_t(buf, 4, baud_rate_1);
    _mav_put_uint16_t(buf, 6, speed_1);
    _mav_put_uint16_t(buf, 8, baud_rate_2);
    _mav_put_uint16_t(buf, 10, speed_2);
    _mav_put_uint16_t(buf, 12, baud_rate_3);
    _mav_put_uint16_t(buf, 14, speed_3);
    _mav_put_uint16_t(buf, 16, baud_rate_4);
    _mav_put_uint16_t(buf, 18, speed_4);
    _mav_put_uint16_t(buf, 20, baud_rate_5);
    _mav_put_uint16_t(buf, 22, speed_5);
    _mav_put_uint8_t(buf, 24, fault_status_0);
    _mav_put_uint8_t(buf, 25, control_mode_0);
    _mav_put_uint8_t(buf, 26, reserved_0);
    _mav_put_uint8_t(buf, 27, life_signal_0);
    _mav_put_uint8_t(buf, 28, fault_status_1);
    _mav_put_uint8_t(buf, 29, control_mode_1);
    _mav_put_uint8_t(buf, 30, reserved_1);
    _mav_put_uint8_t(buf, 31, life_signal_1);
    _mav_put_uint8_t(buf, 32, fault_status_2);
    _mav_put_uint8_t(buf, 33, control_mode_2);
    _mav_put_uint8_t(buf, 34, reserved_2);
    _mav_put_uint8_t(buf, 35, life_signal_2);
    _mav_put_uint8_t(buf, 36, fault_status_3);
    _mav_put_uint8_t(buf, 37, control_mode_3);
    _mav_put_uint8_t(buf, 38, reserved_3);
    _mav_put_uint8_t(buf, 39, life_signal_3);
    _mav_put_uint8_t(buf, 40, fault_status_4);
    _mav_put_uint8_t(buf, 41, control_mode_4);
    _mav_put_uint8_t(buf, 42, reserved_4);
    _mav_put_uint8_t(buf, 43, life_signal_4);
    _mav_put_uint8_t(buf, 44, fault_status_5);
    _mav_put_uint8_t(buf, 45, control_mode_5);
    _mav_put_uint8_t(buf, 46, reserved_5);
    _mav_put_uint8_t(buf, 47, life_signal_5);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DEVICE_STATUS_ARRAY, buf, MAVLINK_MSG_ID_DEVICE_STATUS_ARRAY_MIN_LEN, MAVLINK_MSG_ID_DEVICE_STATUS_ARRAY_LEN, MAVLINK_MSG_ID_DEVICE_STATUS_ARRAY_CRC);
#else
    mavlink_device_status_array_t packet;
    packet.baud_rate_0 = baud_rate_0;
    packet.speed_0 = speed_0;
    packet.baud_rate_1 = baud_rate_1;
    packet.speed_1 = speed_1;
    packet.baud_rate_2 = baud_rate_2;
    packet.speed_2 = speed_2;
    packet.baud_rate_3 = baud_rate_3;
    packet.speed_3 = speed_3;
    packet.baud_rate_4 = baud_rate_4;
    packet.speed_4 = speed_4;
    packet.baud_rate_5 = baud_rate_5;
    packet.speed_5 = speed_5;
    packet.fault_status_0 = fault_status_0;
    packet.control_mode_0 = control_mode_0;
    packet.reserved_0 = reserved_0;
    packet.life_signal_0 = life_signal_0;
    packet.fault_status_1 = fault_status_1;
    packet.control_mode_1 = control_mode_1;
    packet.reserved_1 = reserved_1;
    packet.life_signal_1 = life_signal_1;
    packet.fault_status_2 = fault_status_2;
    packet.control_mode_2 = control_mode_2;
    packet.reserved_2 = reserved_2;
    packet.life_signal_2 = life_signal_2;
    packet.fault_status_3 = fault_status_3;
    packet.control_mode_3 = control_mode_3;
    packet.reserved_3 = reserved_3;
    packet.life_signal_3 = life_signal_3;
    packet.fault_status_4 = fault_status_4;
    packet.control_mode_4 = control_mode_4;
    packet.reserved_4 = reserved_4;
    packet.life_signal_4 = life_signal_4;
    packet.fault_status_5 = fault_status_5;
    packet.control_mode_5 = control_mode_5;
    packet.reserved_5 = reserved_5;
    packet.life_signal_5 = life_signal_5;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DEVICE_STATUS_ARRAY, (const char *)&packet, MAVLINK_MSG_ID_DEVICE_STATUS_ARRAY_MIN_LEN, MAVLINK_MSG_ID_DEVICE_STATUS_ARRAY_LEN, MAVLINK_MSG_ID_DEVICE_STATUS_ARRAY_CRC);
#endif
}

/**
 * @brief Send a device_status_array message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_device_status_array_send_struct(mavlink_channel_t chan, const mavlink_device_status_array_t* device_status_array)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_device_status_array_send(chan, device_status_array->fault_status_0, device_status_array->control_mode_0, device_status_array->baud_rate_0, device_status_array->speed_0, device_status_array->reserved_0, device_status_array->life_signal_0, device_status_array->fault_status_1, device_status_array->control_mode_1, device_status_array->baud_rate_1, device_status_array->speed_1, device_status_array->reserved_1, device_status_array->life_signal_1, device_status_array->fault_status_2, device_status_array->control_mode_2, device_status_array->baud_rate_2, device_status_array->speed_2, device_status_array->reserved_2, device_status_array->life_signal_2, device_status_array->fault_status_3, device_status_array->control_mode_3, device_status_array->baud_rate_3, device_status_array->speed_3, device_status_array->reserved_3, device_status_array->life_signal_3, device_status_array->fault_status_4, device_status_array->control_mode_4, device_status_array->baud_rate_4, device_status_array->speed_4, device_status_array->reserved_4, device_status_array->life_signal_4, device_status_array->fault_status_5, device_status_array->control_mode_5, device_status_array->baud_rate_5, device_status_array->speed_5, device_status_array->reserved_5, device_status_array->life_signal_5);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DEVICE_STATUS_ARRAY, (const char *)device_status_array, MAVLINK_MSG_ID_DEVICE_STATUS_ARRAY_MIN_LEN, MAVLINK_MSG_ID_DEVICE_STATUS_ARRAY_LEN, MAVLINK_MSG_ID_DEVICE_STATUS_ARRAY_CRC);
#endif
}

#if MAVLINK_MSG_ID_DEVICE_STATUS_ARRAY_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by reusing
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_device_status_array_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t fault_status_0, uint8_t control_mode_0, uint16_t baud_rate_0, uint16_t speed_0, uint8_t reserved_0, uint8_t life_signal_0, uint8_t fault_status_1, uint8_t control_mode_1, uint16_t baud_rate_1, uint16_t speed_1, uint8_t reserved_1, uint8_t life_signal_1, uint8_t fault_status_2, uint8_t control_mode_2, uint16_t baud_rate_2, uint16_t speed_2, uint8_t reserved_2, uint8_t life_signal_2, uint8_t fault_status_3, uint8_t control_mode_3, uint16_t baud_rate_3, uint16_t speed_3, uint8_t reserved_3, uint8_t life_signal_3, uint8_t fault_status_4, uint8_t control_mode_4, uint16_t baud_rate_4, uint16_t speed_4, uint8_t reserved_4, uint8_t life_signal_4, uint8_t fault_status_5, uint8_t control_mode_5, uint16_t baud_rate_5, uint16_t speed_5, uint8_t reserved_5, uint8_t life_signal_5)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint16_t(buf, 0, baud_rate_0);
    _mav_put_uint16_t(buf, 2, speed_0);
    _mav_put_uint16_t(buf, 4, baud_rate_1);
    _mav_put_uint16_t(buf, 6, speed_1);
    _mav_put_uint16_t(buf, 8, baud_rate_2);
    _mav_put_uint16_t(buf, 10, speed_2);
    _mav_put_uint16_t(buf, 12, baud_rate_3);
    _mav_put_uint16_t(buf, 14, speed_3);
    _mav_put_uint16_t(buf, 16, baud_rate_4);
    _mav_put_uint16_t(buf, 18, speed_4);
    _mav_put_uint16_t(buf, 20, baud_rate_5);
    _mav_put_uint16_t(buf, 22, speed_5);
    _mav_put_uint8_t(buf, 24, fault_status_0);
    _mav_put_uint8_t(buf, 25, control_mode_0);
    _mav_put_uint8_t(buf, 26, reserved_0);
    _mav_put_uint8_t(buf, 27, life_signal_0);
    _mav_put_uint8_t(buf, 28, fault_status_1);
    _mav_put_uint8_t(buf, 29, control_mode_1);
    _mav_put_uint8_t(buf, 30, reserved_1);
    _mav_put_uint8_t(buf, 31, life_signal_1);
    _mav_put_uint8_t(buf, 32, fault_status_2);
    _mav_put_uint8_t(buf, 33, control_mode_2);
    _mav_put_uint8_t(buf, 34, reserved_2);
    _mav_put_uint8_t(buf, 35, life_signal_2);
    _mav_put_uint8_t(buf, 36, fault_status_3);
    _mav_put_uint8_t(buf, 37, control_mode_3);
    _mav_put_uint8_t(buf, 38, reserved_3);
    _mav_put_uint8_t(buf, 39, life_signal_3);
    _mav_put_uint8_t(buf, 40, fault_status_4);
    _mav_put_uint8_t(buf, 41, control_mode_4);
    _mav_put_uint8_t(buf, 42, reserved_4);
    _mav_put_uint8_t(buf, 43, life_signal_4);
    _mav_put_uint8_t(buf, 44, fault_status_5);
    _mav_put_uint8_t(buf, 45, control_mode_5);
    _mav_put_uint8_t(buf, 46, reserved_5);
    _mav_put_uint8_t(buf, 47, life_signal_5);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DEVICE_STATUS_ARRAY, buf, MAVLINK_MSG_ID_DEVICE_STATUS_ARRAY_MIN_LEN, MAVLINK_MSG_ID_DEVICE_STATUS_ARRAY_LEN, MAVLINK_MSG_ID_DEVICE_STATUS_ARRAY_CRC);
#else
    mavlink_device_status_array_t *packet = (mavlink_device_status_array_t *)msgbuf;
    packet->baud_rate_0 = baud_rate_0;
    packet->speed_0 = speed_0;
    packet->baud_rate_1 = baud_rate_1;
    packet->speed_1 = speed_1;
    packet->baud_rate_2 = baud_rate_2;
    packet->speed_2 = speed_2;
    packet->baud_rate_3 = baud_rate_3;
    packet->speed_3 = speed_3;
    packet->baud_rate_4 = baud_rate_4;
    packet->speed_4 = speed_4;
    packet->baud_rate_5 = baud_rate_5;
    packet->speed_5 = speed_5;
    packet->fault_status_0 = fault_status_0;
    packet->control_mode_0 = control_mode_0;
    packet->reserved_0 = reserved_0;
    packet->life_signal_0 = life_signal_0;
    packet->fault_status_1 = fault_status_1;
    packet->control_mode_1 = control_mode_1;
    packet->reserved_1 = reserved_1;
    packet->life_signal_1 = life_signal_1;
    packet->fault_status_2 = fault_status_2;
    packet->control_mode_2 = control_mode_2;
    packet->reserved_2 = reserved_2;
    packet->life_signal_2 = life_signal_2;
    packet->fault_status_3 = fault_status_3;
    packet->control_mode_3 = control_mode_3;
    packet->reserved_3 = reserved_3;
    packet->life_signal_3 = life_signal_3;
    packet->fault_status_4 = fault_status_4;
    packet->control_mode_4 = control_mode_4;
    packet->reserved_4 = reserved_4;
    packet->life_signal_4 = life_signal_4;
    packet->fault_status_5 = fault_status_5;
    packet->control_mode_5 = control_mode_5;
    packet->reserved_5 = reserved_5;
    packet->life_signal_5 = life_signal_5;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DEVICE_STATUS_ARRAY, (const char *)packet, MAVLINK_MSG_ID_DEVICE_STATUS_ARRAY_MIN_LEN, MAVLINK_MSG_ID_DEVICE_STATUS_ARRAY_LEN, MAVLINK_MSG_ID_DEVICE_STATUS_ARRAY_CRC);
#endif
}
#endif

#endif

// MESSAGE DEVICE_STATUS_ARRAY UNPACKING


/**
 * @brief Get field fault_status_0 from device_status_array message
 *
 * @return  Device 0 fault status flags
 */
static inline uint8_t mavlink_msg_device_status_array_get_fault_status_0(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  24);
}

/**
 * @brief Get field control_mode_0 from device_status_array message
 *
 * @return  Device 0 control mode
 */
static inline uint8_t mavlink_msg_device_status_array_get_control_mode_0(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  25);
}

/**
 * @brief Get field baud_rate_0 from device_status_array message
 *
 * @return  Device 0 CAN bus baud rate
 */
static inline uint16_t mavlink_msg_device_status_array_get_baud_rate_0(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  0);
}

/**
 * @brief Get field speed_0 from device_status_array message
 *
 * @return  Device 0 rotation speed
 */
static inline uint16_t mavlink_msg_device_status_array_get_speed_0(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  2);
}

/**
 * @brief Get field reserved_0 from device_status_array message
 *
 * @return  Device 0 reserved field
 */
static inline uint8_t mavlink_msg_device_status_array_get_reserved_0(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  26);
}

/**
 * @brief Get field life_signal_0 from device_status_array message
 *
 * @return  Device 0 life signal counter
 */
static inline uint8_t mavlink_msg_device_status_array_get_life_signal_0(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  27);
}

/**
 * @brief Get field fault_status_1 from device_status_array message
 *
 * @return  Device 1 fault status flags
 */
static inline uint8_t mavlink_msg_device_status_array_get_fault_status_1(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  28);
}

/**
 * @brief Get field control_mode_1 from device_status_array message
 *
 * @return  Device 1 control mode
 */
static inline uint8_t mavlink_msg_device_status_array_get_control_mode_1(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  29);
}

/**
 * @brief Get field baud_rate_1 from device_status_array message
 *
 * @return  Device 1 CAN bus baud rate
 */
static inline uint16_t mavlink_msg_device_status_array_get_baud_rate_1(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  4);
}

/**
 * @brief Get field speed_1 from device_status_array message
 *
 * @return  Device 1 rotation speed
 */
static inline uint16_t mavlink_msg_device_status_array_get_speed_1(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  6);
}

/**
 * @brief Get field reserved_1 from device_status_array message
 *
 * @return  Device 1 reserved field
 */
static inline uint8_t mavlink_msg_device_status_array_get_reserved_1(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  30);
}

/**
 * @brief Get field life_signal_1 from device_status_array message
 *
 * @return  Device 1 life signal counter
 */
static inline uint8_t mavlink_msg_device_status_array_get_life_signal_1(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  31);
}

/**
 * @brief Get field fault_status_2 from device_status_array message
 *
 * @return  Device 2 fault status flags
 */
static inline uint8_t mavlink_msg_device_status_array_get_fault_status_2(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  32);
}

/**
 * @brief Get field control_mode_2 from device_status_array message
 *
 * @return  Device 2 control mode
 */
static inline uint8_t mavlink_msg_device_status_array_get_control_mode_2(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  33);
}

/**
 * @brief Get field baud_rate_2 from device_status_array message
 *
 * @return  Device 2 CAN bus baud rate
 */
static inline uint16_t mavlink_msg_device_status_array_get_baud_rate_2(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  8);
}

/**
 * @brief Get field speed_2 from device_status_array message
 *
 * @return  Device 2 rotation speed
 */
static inline uint16_t mavlink_msg_device_status_array_get_speed_2(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  10);
}

/**
 * @brief Get field reserved_2 from device_status_array message
 *
 * @return  Device 2 reserved field
 */
static inline uint8_t mavlink_msg_device_status_array_get_reserved_2(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  34);
}

/**
 * @brief Get field life_signal_2 from device_status_array message
 *
 * @return  Device 2 life signal counter
 */
static inline uint8_t mavlink_msg_device_status_array_get_life_signal_2(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  35);
}

/**
 * @brief Get field fault_status_3 from device_status_array message
 *
 * @return  Device 3 fault status flags
 */
static inline uint8_t mavlink_msg_device_status_array_get_fault_status_3(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  36);
}

/**
 * @brief Get field control_mode_3 from device_status_array message
 *
 * @return  Device 3 control mode
 */
static inline uint8_t mavlink_msg_device_status_array_get_control_mode_3(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  37);
}

/**
 * @brief Get field baud_rate_3 from device_status_array message
 *
 * @return  Device 3 CAN bus baud rate
 */
static inline uint16_t mavlink_msg_device_status_array_get_baud_rate_3(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  12);
}

/**
 * @brief Get field speed_3 from device_status_array message
 *
 * @return  Device 3 rotation speed
 */
static inline uint16_t mavlink_msg_device_status_array_get_speed_3(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  14);
}

/**
 * @brief Get field reserved_3 from device_status_array message
 *
 * @return  Device 3 reserved field
 */
static inline uint8_t mavlink_msg_device_status_array_get_reserved_3(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  38);
}

/**
 * @brief Get field life_signal_3 from device_status_array message
 *
 * @return  Device 3 life signal counter
 */
static inline uint8_t mavlink_msg_device_status_array_get_life_signal_3(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  39);
}

/**
 * @brief Get field fault_status_4 from device_status_array message
 *
 * @return  Device 4 fault status flags
 */
static inline uint8_t mavlink_msg_device_status_array_get_fault_status_4(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  40);
}

/**
 * @brief Get field control_mode_4 from device_status_array message
 *
 * @return  Device 4 control mode
 */
static inline uint8_t mavlink_msg_device_status_array_get_control_mode_4(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  41);
}

/**
 * @brief Get field baud_rate_4 from device_status_array message
 *
 * @return  Device 4 CAN bus baud rate
 */
static inline uint16_t mavlink_msg_device_status_array_get_baud_rate_4(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  16);
}

/**
 * @brief Get field speed_4 from device_status_array message
 *
 * @return  Device 4 rotation speed
 */
static inline uint16_t mavlink_msg_device_status_array_get_speed_4(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  18);
}

/**
 * @brief Get field reserved_4 from device_status_array message
 *
 * @return  Device 4 reserved field
 */
static inline uint8_t mavlink_msg_device_status_array_get_reserved_4(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  42);
}

/**
 * @brief Get field life_signal_4 from device_status_array message
 *
 * @return  Device 4 life signal counter
 */
static inline uint8_t mavlink_msg_device_status_array_get_life_signal_4(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  43);
}

/**
 * @brief Get field fault_status_5 from device_status_array message
 *
 * @return  Device 5 fault status flags
 */
static inline uint8_t mavlink_msg_device_status_array_get_fault_status_5(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  44);
}

/**
 * @brief Get field control_mode_5 from device_status_array message
 *
 * @return  Device 5 control mode
 */
static inline uint8_t mavlink_msg_device_status_array_get_control_mode_5(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  45);
}

/**
 * @brief Get field baud_rate_5 from device_status_array message
 *
 * @return  Device 5 CAN bus baud rate
 */
static inline uint16_t mavlink_msg_device_status_array_get_baud_rate_5(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  20);
}

/**
 * @brief Get field speed_5 from device_status_array message
 *
 * @return  Device 5 rotation speed
 */
static inline uint16_t mavlink_msg_device_status_array_get_speed_5(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  22);
}

/**
 * @brief Get field reserved_5 from device_status_array message
 *
 * @return  Device 5 reserved field
 */
static inline uint8_t mavlink_msg_device_status_array_get_reserved_5(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  46);
}

/**
 * @brief Get field life_signal_5 from device_status_array message
 *
 * @return  Device 5 life signal counter
 */
static inline uint8_t mavlink_msg_device_status_array_get_life_signal_5(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  47);
}

/**
 * @brief Decode a device_status_array message into a struct
 *
 * @param msg The message to decode
 * @param device_status_array C-struct to decode the message contents into
 */
static inline void mavlink_msg_device_status_array_decode(const mavlink_message_t* msg, mavlink_device_status_array_t* device_status_array)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    device_status_array->baud_rate_0 = mavlink_msg_device_status_array_get_baud_rate_0(msg);
    device_status_array->speed_0 = mavlink_msg_device_status_array_get_speed_0(msg);
    device_status_array->baud_rate_1 = mavlink_msg_device_status_array_get_baud_rate_1(msg);
    device_status_array->speed_1 = mavlink_msg_device_status_array_get_speed_1(msg);
    device_status_array->baud_rate_2 = mavlink_msg_device_status_array_get_baud_rate_2(msg);
    device_status_array->speed_2 = mavlink_msg_device_status_array_get_speed_2(msg);
    device_status_array->baud_rate_3 = mavlink_msg_device_status_array_get_baud_rate_3(msg);
    device_status_array->speed_3 = mavlink_msg_device_status_array_get_speed_3(msg);
    device_status_array->baud_rate_4 = mavlink_msg_device_status_array_get_baud_rate_4(msg);
    device_status_array->speed_4 = mavlink_msg_device_status_array_get_speed_4(msg);
    device_status_array->baud_rate_5 = mavlink_msg_device_status_array_get_baud_rate_5(msg);
    device_status_array->speed_5 = mavlink_msg_device_status_array_get_speed_5(msg);
    device_status_array->fault_status_0 = mavlink_msg_device_status_array_get_fault_status_0(msg);
    device_status_array->control_mode_0 = mavlink_msg_device_status_array_get_control_mode_0(msg);
    device_status_array->reserved_0 = mavlink_msg_device_status_array_get_reserved_0(msg);
    device_status_array->life_signal_0 = mavlink_msg_device_status_array_get_life_signal_0(msg);
    device_status_array->fault_status_1 = mavlink_msg_device_status_array_get_fault_status_1(msg);
    device_status_array->control_mode_1 = mavlink_msg_device_status_array_get_control_mode_1(msg);
    device_status_array->reserved_1 = mavlink_msg_device_status_array_get_reserved_1(msg);
    device_status_array->life_signal_1 = mavlink_msg_device_status_array_get_life_signal_1(msg);
    device_status_array->fault_status_2 = mavlink_msg_device_status_array_get_fault_status_2(msg);
    device_status_array->control_mode_2 = mavlink_msg_device_status_array_get_control_mode_2(msg);
    device_status_array->reserved_2 = mavlink_msg_device_status_array_get_reserved_2(msg);
    device_status_array->life_signal_2 = mavlink_msg_device_status_array_get_life_signal_2(msg);
    device_status_array->fault_status_3 = mavlink_msg_device_status_array_get_fault_status_3(msg);
    device_status_array->control_mode_3 = mavlink_msg_device_status_array_get_control_mode_3(msg);
    device_status_array->reserved_3 = mavlink_msg_device_status_array_get_reserved_3(msg);
    device_status_array->life_signal_3 = mavlink_msg_device_status_array_get_life_signal_3(msg);
    device_status_array->fault_status_4 = mavlink_msg_device_status_array_get_fault_status_4(msg);
    device_status_array->control_mode_4 = mavlink_msg_device_status_array_get_control_mode_4(msg);
    device_status_array->reserved_4 = mavlink_msg_device_status_array_get_reserved_4(msg);
    device_status_array->life_signal_4 = mavlink_msg_device_status_array_get_life_signal_4(msg);
    device_status_array->fault_status_5 = mavlink_msg_device_status_array_get_fault_status_5(msg);
    device_status_array->control_mode_5 = mavlink_msg_device_status_array_get_control_mode_5(msg);
    device_status_array->reserved_5 = mavlink_msg_device_status_array_get_reserved_5(msg);
    device_status_array->life_signal_5 = mavlink_msg_device_status_array_get_life_signal_5(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_DEVICE_STATUS_ARRAY_LEN? msg->len : MAVLINK_MSG_ID_DEVICE_STATUS_ARRAY_LEN;
        memset(device_status_array, 0, MAVLINK_MSG_ID_DEVICE_STATUS_ARRAY_LEN);
    memcpy(device_status_array, _MAV_PAYLOAD(msg), len);
#endif
}
