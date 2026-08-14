#pragma once
// MESSAGE BATTERY_DATA PACKING

#define MAVLINK_MSG_ID_BATTERY_DATA 510


typedef struct __mavlink_battery_data_t {
 uint32_t current; /*< [mA] Battery current in milliamps*/
 uint16_t voltage; /*< [mV] Battery voltage in millivolts*/
 uint16_t cell_temp; /*< [degC] Battery cell temperature in degrees Celsius*/
 uint16_t mosfet_temp; /*< [degC] Battery MOSFET temperature in degrees Celsius*/
 uint16_t capacity_percent; /*< [%] Battery capacity percentage (0-100)*/
} mavlink_battery_data_t;

#define MAVLINK_MSG_ID_BATTERY_DATA_LEN 12
#define MAVLINK_MSG_ID_BATTERY_DATA_MIN_LEN 12
#define MAVLINK_MSG_ID_510_LEN 12
#define MAVLINK_MSG_ID_510_MIN_LEN 12

#define MAVLINK_MSG_ID_BATTERY_DATA_CRC 29
#define MAVLINK_MSG_ID_510_CRC 29



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_BATTERY_DATA { \
    510, \
    "BATTERY_DATA", \
    5, \
    {  { "voltage", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_battery_data_t, voltage) }, \
         { "current", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_battery_data_t, current) }, \
         { "cell_temp", NULL, MAVLINK_TYPE_UINT16_T, 0, 6, offsetof(mavlink_battery_data_t, cell_temp) }, \
         { "mosfet_temp", NULL, MAVLINK_TYPE_UINT16_T, 0, 8, offsetof(mavlink_battery_data_t, mosfet_temp) }, \
         { "capacity_percent", NULL, MAVLINK_TYPE_UINT16_T, 0, 10, offsetof(mavlink_battery_data_t, capacity_percent) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_BATTERY_DATA { \
    "BATTERY_DATA", \
    5, \
    {  { "voltage", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_battery_data_t, voltage) }, \
         { "current", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_battery_data_t, current) }, \
         { "cell_temp", NULL, MAVLINK_TYPE_UINT16_T, 0, 6, offsetof(mavlink_battery_data_t, cell_temp) }, \
         { "mosfet_temp", NULL, MAVLINK_TYPE_UINT16_T, 0, 8, offsetof(mavlink_battery_data_t, mosfet_temp) }, \
         { "capacity_percent", NULL, MAVLINK_TYPE_UINT16_T, 0, 10, offsetof(mavlink_battery_data_t, capacity_percent) }, \
         } \
}
#endif

/**
 * @brief Pack a battery_data message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param voltage [mV] Battery voltage in millivolts
 * @param current [mA] Battery current in milliamps
 * @param cell_temp [degC] Battery cell temperature in degrees Celsius
 * @param mosfet_temp [degC] Battery MOSFET temperature in degrees Celsius
 * @param capacity_percent [%] Battery capacity percentage (0-100)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_battery_data_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint16_t voltage, uint32_t current, uint16_t cell_temp, uint16_t mosfet_temp, uint16_t capacity_percent)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_BATTERY_DATA_LEN];
    _mav_put_uint32_t(buf, 0, current);
    _mav_put_uint16_t(buf, 4, voltage);
    _mav_put_uint16_t(buf, 6, cell_temp);
    _mav_put_uint16_t(buf, 8, mosfet_temp);
    _mav_put_uint16_t(buf, 10, capacity_percent);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_BATTERY_DATA_LEN);
#else
    mavlink_battery_data_t packet;
    packet.current = current;
    packet.voltage = voltage;
    packet.cell_temp = cell_temp;
    packet.mosfet_temp = mosfet_temp;
    packet.capacity_percent = capacity_percent;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_BATTERY_DATA_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_BATTERY_DATA;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_BATTERY_DATA_MIN_LEN, MAVLINK_MSG_ID_BATTERY_DATA_LEN, MAVLINK_MSG_ID_BATTERY_DATA_CRC);
}

/**
 * @brief Pack a battery_data message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param voltage [mV] Battery voltage in millivolts
 * @param current [mA] Battery current in milliamps
 * @param cell_temp [degC] Battery cell temperature in degrees Celsius
 * @param mosfet_temp [degC] Battery MOSFET temperature in degrees Celsius
 * @param capacity_percent [%] Battery capacity percentage (0-100)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_battery_data_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint16_t voltage, uint32_t current, uint16_t cell_temp, uint16_t mosfet_temp, uint16_t capacity_percent)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_BATTERY_DATA_LEN];
    _mav_put_uint32_t(buf, 0, current);
    _mav_put_uint16_t(buf, 4, voltage);
    _mav_put_uint16_t(buf, 6, cell_temp);
    _mav_put_uint16_t(buf, 8, mosfet_temp);
    _mav_put_uint16_t(buf, 10, capacity_percent);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_BATTERY_DATA_LEN);
#else
    mavlink_battery_data_t packet;
    packet.current = current;
    packet.voltage = voltage;
    packet.cell_temp = cell_temp;
    packet.mosfet_temp = mosfet_temp;
    packet.capacity_percent = capacity_percent;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_BATTERY_DATA_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_BATTERY_DATA;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_BATTERY_DATA_MIN_LEN, MAVLINK_MSG_ID_BATTERY_DATA_LEN, MAVLINK_MSG_ID_BATTERY_DATA_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_BATTERY_DATA_MIN_LEN, MAVLINK_MSG_ID_BATTERY_DATA_LEN);
#endif
}

/**
 * @brief Pack a battery_data message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param voltage [mV] Battery voltage in millivolts
 * @param current [mA] Battery current in milliamps
 * @param cell_temp [degC] Battery cell temperature in degrees Celsius
 * @param mosfet_temp [degC] Battery MOSFET temperature in degrees Celsius
 * @param capacity_percent [%] Battery capacity percentage (0-100)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_battery_data_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint16_t voltage,uint32_t current,uint16_t cell_temp,uint16_t mosfet_temp,uint16_t capacity_percent)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_BATTERY_DATA_LEN];
    _mav_put_uint32_t(buf, 0, current);
    _mav_put_uint16_t(buf, 4, voltage);
    _mav_put_uint16_t(buf, 6, cell_temp);
    _mav_put_uint16_t(buf, 8, mosfet_temp);
    _mav_put_uint16_t(buf, 10, capacity_percent);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_BATTERY_DATA_LEN);
#else
    mavlink_battery_data_t packet;
    packet.current = current;
    packet.voltage = voltage;
    packet.cell_temp = cell_temp;
    packet.mosfet_temp = mosfet_temp;
    packet.capacity_percent = capacity_percent;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_BATTERY_DATA_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_BATTERY_DATA;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_BATTERY_DATA_MIN_LEN, MAVLINK_MSG_ID_BATTERY_DATA_LEN, MAVLINK_MSG_ID_BATTERY_DATA_CRC);
}

/**
 * @brief Encode a battery_data struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param battery_data C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_battery_data_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_battery_data_t* battery_data)
{
    return mavlink_msg_battery_data_pack(system_id, component_id, msg, battery_data->voltage, battery_data->current, battery_data->cell_temp, battery_data->mosfet_temp, battery_data->capacity_percent);
}

/**
 * @brief Encode a battery_data struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param battery_data C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_battery_data_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_battery_data_t* battery_data)
{
    return mavlink_msg_battery_data_pack_chan(system_id, component_id, chan, msg, battery_data->voltage, battery_data->current, battery_data->cell_temp, battery_data->mosfet_temp, battery_data->capacity_percent);
}

/**
 * @brief Encode a battery_data struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param battery_data C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_battery_data_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_battery_data_t* battery_data)
{
    return mavlink_msg_battery_data_pack_status(system_id, component_id, _status, msg,  battery_data->voltage, battery_data->current, battery_data->cell_temp, battery_data->mosfet_temp, battery_data->capacity_percent);
}

/**
 * @brief Send a battery_data message
 * @param chan MAVLink channel to send the message
 *
 * @param voltage [mV] Battery voltage in millivolts
 * @param current [mA] Battery current in milliamps
 * @param cell_temp [degC] Battery cell temperature in degrees Celsius
 * @param mosfet_temp [degC] Battery MOSFET temperature in degrees Celsius
 * @param capacity_percent [%] Battery capacity percentage (0-100)
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_battery_data_send(mavlink_channel_t chan, uint16_t voltage, uint32_t current, uint16_t cell_temp, uint16_t mosfet_temp, uint16_t capacity_percent)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_BATTERY_DATA_LEN];
    _mav_put_uint32_t(buf, 0, current);
    _mav_put_uint16_t(buf, 4, voltage);
    _mav_put_uint16_t(buf, 6, cell_temp);
    _mav_put_uint16_t(buf, 8, mosfet_temp);
    _mav_put_uint16_t(buf, 10, capacity_percent);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BATTERY_DATA, buf, MAVLINK_MSG_ID_BATTERY_DATA_MIN_LEN, MAVLINK_MSG_ID_BATTERY_DATA_LEN, MAVLINK_MSG_ID_BATTERY_DATA_CRC);
#else
    mavlink_battery_data_t packet;
    packet.current = current;
    packet.voltage = voltage;
    packet.cell_temp = cell_temp;
    packet.mosfet_temp = mosfet_temp;
    packet.capacity_percent = capacity_percent;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BATTERY_DATA, (const char *)&packet, MAVLINK_MSG_ID_BATTERY_DATA_MIN_LEN, MAVLINK_MSG_ID_BATTERY_DATA_LEN, MAVLINK_MSG_ID_BATTERY_DATA_CRC);
#endif
}

/**
 * @brief Send a battery_data message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_battery_data_send_struct(mavlink_channel_t chan, const mavlink_battery_data_t* battery_data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_battery_data_send(chan, battery_data->voltage, battery_data->current, battery_data->cell_temp, battery_data->mosfet_temp, battery_data->capacity_percent);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BATTERY_DATA, (const char *)battery_data, MAVLINK_MSG_ID_BATTERY_DATA_MIN_LEN, MAVLINK_MSG_ID_BATTERY_DATA_LEN, MAVLINK_MSG_ID_BATTERY_DATA_CRC);
#endif
}

#if MAVLINK_MSG_ID_BATTERY_DATA_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by reusing
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_battery_data_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint16_t voltage, uint32_t current, uint16_t cell_temp, uint16_t mosfet_temp, uint16_t capacity_percent)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, current);
    _mav_put_uint16_t(buf, 4, voltage);
    _mav_put_uint16_t(buf, 6, cell_temp);
    _mav_put_uint16_t(buf, 8, mosfet_temp);
    _mav_put_uint16_t(buf, 10, capacity_percent);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BATTERY_DATA, buf, MAVLINK_MSG_ID_BATTERY_DATA_MIN_LEN, MAVLINK_MSG_ID_BATTERY_DATA_LEN, MAVLINK_MSG_ID_BATTERY_DATA_CRC);
#else
    mavlink_battery_data_t *packet = (mavlink_battery_data_t *)msgbuf;
    packet->current = current;
    packet->voltage = voltage;
    packet->cell_temp = cell_temp;
    packet->mosfet_temp = mosfet_temp;
    packet->capacity_percent = capacity_percent;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BATTERY_DATA, (const char *)packet, MAVLINK_MSG_ID_BATTERY_DATA_MIN_LEN, MAVLINK_MSG_ID_BATTERY_DATA_LEN, MAVLINK_MSG_ID_BATTERY_DATA_CRC);
#endif
}
#endif

#endif

// MESSAGE BATTERY_DATA UNPACKING


/**
 * @brief Get field voltage from battery_data message
 *
 * @return [mV] Battery voltage in millivolts
 */
static inline uint16_t mavlink_msg_battery_data_get_voltage(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  4);
}

/**
 * @brief Get field current from battery_data message
 *
 * @return [mA] Battery current in milliamps
 */
static inline uint32_t mavlink_msg_battery_data_get_current(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field cell_temp from battery_data message
 *
 * @return [degC] Battery cell temperature in degrees Celsius
 */
static inline uint16_t mavlink_msg_battery_data_get_cell_temp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  6);
}

/**
 * @brief Get field mosfet_temp from battery_data message
 *
 * @return [degC] Battery MOSFET temperature in degrees Celsius
 */
static inline uint16_t mavlink_msg_battery_data_get_mosfet_temp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  8);
}

/**
 * @brief Get field capacity_percent from battery_data message
 *
 * @return [%] Battery capacity percentage (0-100)
 */
static inline uint16_t mavlink_msg_battery_data_get_capacity_percent(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  10);
}

/**
 * @brief Decode a battery_data message into a struct
 *
 * @param msg The message to decode
 * @param battery_data C-struct to decode the message contents into
 */
static inline void mavlink_msg_battery_data_decode(const mavlink_message_t* msg, mavlink_battery_data_t* battery_data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    battery_data->current = mavlink_msg_battery_data_get_current(msg);
    battery_data->voltage = mavlink_msg_battery_data_get_voltage(msg);
    battery_data->cell_temp = mavlink_msg_battery_data_get_cell_temp(msg);
    battery_data->mosfet_temp = mavlink_msg_battery_data_get_mosfet_temp(msg);
    battery_data->capacity_percent = mavlink_msg_battery_data_get_capacity_percent(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_BATTERY_DATA_LEN? msg->len : MAVLINK_MSG_ID_BATTERY_DATA_LEN;
        memset(battery_data, 0, MAVLINK_MSG_ID_BATTERY_DATA_LEN);
    memcpy(battery_data, _MAV_PAYLOAD(msg), len);
#endif
}
