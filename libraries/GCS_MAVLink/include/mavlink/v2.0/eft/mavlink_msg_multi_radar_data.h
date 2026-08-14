#pragma once
// MESSAGE MULTI_RADAR_DATA PACKING

#define MAVLINK_MSG_ID_MULTI_RADAR_DATA 514


typedef struct __mavlink_multi_radar_data_t {
 uint16_t ground_distance; /*< [cm] Ground radar distance in centimeters*/
 uint16_t forward_distance; /*< [cm] Forward radar distance in centimeters*/
 uint16_t backward_distance; /*< [cm] Backward radar distance in centimeters*/
} mavlink_multi_radar_data_t;

#define MAVLINK_MSG_ID_MULTI_RADAR_DATA_LEN 6
#define MAVLINK_MSG_ID_MULTI_RADAR_DATA_MIN_LEN 6
#define MAVLINK_MSG_ID_514_LEN 6
#define MAVLINK_MSG_ID_514_MIN_LEN 6

#define MAVLINK_MSG_ID_MULTI_RADAR_DATA_CRC 62
#define MAVLINK_MSG_ID_514_CRC 62



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_MULTI_RADAR_DATA { \
    514, \
    "MULTI_RADAR_DATA", \
    3, \
    {  { "ground_distance", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_multi_radar_data_t, ground_distance) }, \
         { "forward_distance", NULL, MAVLINK_TYPE_UINT16_T, 0, 2, offsetof(mavlink_multi_radar_data_t, forward_distance) }, \
         { "backward_distance", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_multi_radar_data_t, backward_distance) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_MULTI_RADAR_DATA { \
    "MULTI_RADAR_DATA", \
    3, \
    {  { "ground_distance", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_multi_radar_data_t, ground_distance) }, \
         { "forward_distance", NULL, MAVLINK_TYPE_UINT16_T, 0, 2, offsetof(mavlink_multi_radar_data_t, forward_distance) }, \
         { "backward_distance", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_multi_radar_data_t, backward_distance) }, \
         } \
}
#endif

/**
 * @brief Pack a multi_radar_data message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param ground_distance [cm] Ground radar distance in centimeters
 * @param forward_distance [cm] Forward radar distance in centimeters
 * @param backward_distance [cm] Backward radar distance in centimeters
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_multi_radar_data_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint16_t ground_distance, uint16_t forward_distance, uint16_t backward_distance)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MULTI_RADAR_DATA_LEN];
    _mav_put_uint16_t(buf, 0, ground_distance);
    _mav_put_uint16_t(buf, 2, forward_distance);
    _mav_put_uint16_t(buf, 4, backward_distance);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MULTI_RADAR_DATA_LEN);
#else
    mavlink_multi_radar_data_t packet;
    packet.ground_distance = ground_distance;
    packet.forward_distance = forward_distance;
    packet.backward_distance = backward_distance;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MULTI_RADAR_DATA_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MULTI_RADAR_DATA;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MULTI_RADAR_DATA_MIN_LEN, MAVLINK_MSG_ID_MULTI_RADAR_DATA_LEN, MAVLINK_MSG_ID_MULTI_RADAR_DATA_CRC);
}

/**
 * @brief Pack a multi_radar_data message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param ground_distance [cm] Ground radar distance in centimeters
 * @param forward_distance [cm] Forward radar distance in centimeters
 * @param backward_distance [cm] Backward radar distance in centimeters
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_multi_radar_data_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint16_t ground_distance, uint16_t forward_distance, uint16_t backward_distance)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MULTI_RADAR_DATA_LEN];
    _mav_put_uint16_t(buf, 0, ground_distance);
    _mav_put_uint16_t(buf, 2, forward_distance);
    _mav_put_uint16_t(buf, 4, backward_distance);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MULTI_RADAR_DATA_LEN);
#else
    mavlink_multi_radar_data_t packet;
    packet.ground_distance = ground_distance;
    packet.forward_distance = forward_distance;
    packet.backward_distance = backward_distance;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MULTI_RADAR_DATA_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MULTI_RADAR_DATA;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_MULTI_RADAR_DATA_MIN_LEN, MAVLINK_MSG_ID_MULTI_RADAR_DATA_LEN, MAVLINK_MSG_ID_MULTI_RADAR_DATA_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_MULTI_RADAR_DATA_MIN_LEN, MAVLINK_MSG_ID_MULTI_RADAR_DATA_LEN);
#endif
}

/**
 * @brief Pack a multi_radar_data message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param ground_distance [cm] Ground radar distance in centimeters
 * @param forward_distance [cm] Forward radar distance in centimeters
 * @param backward_distance [cm] Backward radar distance in centimeters
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_multi_radar_data_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint16_t ground_distance,uint16_t forward_distance,uint16_t backward_distance)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MULTI_RADAR_DATA_LEN];
    _mav_put_uint16_t(buf, 0, ground_distance);
    _mav_put_uint16_t(buf, 2, forward_distance);
    _mav_put_uint16_t(buf, 4, backward_distance);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MULTI_RADAR_DATA_LEN);
#else
    mavlink_multi_radar_data_t packet;
    packet.ground_distance = ground_distance;
    packet.forward_distance = forward_distance;
    packet.backward_distance = backward_distance;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MULTI_RADAR_DATA_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MULTI_RADAR_DATA;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_MULTI_RADAR_DATA_MIN_LEN, MAVLINK_MSG_ID_MULTI_RADAR_DATA_LEN, MAVLINK_MSG_ID_MULTI_RADAR_DATA_CRC);
}

/**
 * @brief Encode a multi_radar_data struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param multi_radar_data C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_multi_radar_data_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_multi_radar_data_t* multi_radar_data)
{
    return mavlink_msg_multi_radar_data_pack(system_id, component_id, msg, multi_radar_data->ground_distance, multi_radar_data->forward_distance, multi_radar_data->backward_distance);
}

/**
 * @brief Encode a multi_radar_data struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param multi_radar_data C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_multi_radar_data_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_multi_radar_data_t* multi_radar_data)
{
    return mavlink_msg_multi_radar_data_pack_chan(system_id, component_id, chan, msg, multi_radar_data->ground_distance, multi_radar_data->forward_distance, multi_radar_data->backward_distance);
}

/**
 * @brief Encode a multi_radar_data struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param multi_radar_data C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_multi_radar_data_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_multi_radar_data_t* multi_radar_data)
{
    return mavlink_msg_multi_radar_data_pack_status(system_id, component_id, _status, msg,  multi_radar_data->ground_distance, multi_radar_data->forward_distance, multi_radar_data->backward_distance);
}

/**
 * @brief Send a multi_radar_data message
 * @param chan MAVLink channel to send the message
 *
 * @param ground_distance [cm] Ground radar distance in centimeters
 * @param forward_distance [cm] Forward radar distance in centimeters
 * @param backward_distance [cm] Backward radar distance in centimeters
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_multi_radar_data_send(mavlink_channel_t chan, uint16_t ground_distance, uint16_t forward_distance, uint16_t backward_distance)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MULTI_RADAR_DATA_LEN];
    _mav_put_uint16_t(buf, 0, ground_distance);
    _mav_put_uint16_t(buf, 2, forward_distance);
    _mav_put_uint16_t(buf, 4, backward_distance);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MULTI_RADAR_DATA, buf, MAVLINK_MSG_ID_MULTI_RADAR_DATA_MIN_LEN, MAVLINK_MSG_ID_MULTI_RADAR_DATA_LEN, MAVLINK_MSG_ID_MULTI_RADAR_DATA_CRC);
#else
    mavlink_multi_radar_data_t packet;
    packet.ground_distance = ground_distance;
    packet.forward_distance = forward_distance;
    packet.backward_distance = backward_distance;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MULTI_RADAR_DATA, (const char *)&packet, MAVLINK_MSG_ID_MULTI_RADAR_DATA_MIN_LEN, MAVLINK_MSG_ID_MULTI_RADAR_DATA_LEN, MAVLINK_MSG_ID_MULTI_RADAR_DATA_CRC);
#endif
}

/**
 * @brief Send a multi_radar_data message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_multi_radar_data_send_struct(mavlink_channel_t chan, const mavlink_multi_radar_data_t* multi_radar_data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_multi_radar_data_send(chan, multi_radar_data->ground_distance, multi_radar_data->forward_distance, multi_radar_data->backward_distance);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MULTI_RADAR_DATA, (const char *)multi_radar_data, MAVLINK_MSG_ID_MULTI_RADAR_DATA_MIN_LEN, MAVLINK_MSG_ID_MULTI_RADAR_DATA_LEN, MAVLINK_MSG_ID_MULTI_RADAR_DATA_CRC);
#endif
}

#if MAVLINK_MSG_ID_MULTI_RADAR_DATA_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by reusing
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_multi_radar_data_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint16_t ground_distance, uint16_t forward_distance, uint16_t backward_distance)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint16_t(buf, 0, ground_distance);
    _mav_put_uint16_t(buf, 2, forward_distance);
    _mav_put_uint16_t(buf, 4, backward_distance);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MULTI_RADAR_DATA, buf, MAVLINK_MSG_ID_MULTI_RADAR_DATA_MIN_LEN, MAVLINK_MSG_ID_MULTI_RADAR_DATA_LEN, MAVLINK_MSG_ID_MULTI_RADAR_DATA_CRC);
#else
    mavlink_multi_radar_data_t *packet = (mavlink_multi_radar_data_t *)msgbuf;
    packet->ground_distance = ground_distance;
    packet->forward_distance = forward_distance;
    packet->backward_distance = backward_distance;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MULTI_RADAR_DATA, (const char *)packet, MAVLINK_MSG_ID_MULTI_RADAR_DATA_MIN_LEN, MAVLINK_MSG_ID_MULTI_RADAR_DATA_LEN, MAVLINK_MSG_ID_MULTI_RADAR_DATA_CRC);
#endif
}
#endif

#endif

// MESSAGE MULTI_RADAR_DATA UNPACKING


/**
 * @brief Get field ground_distance from multi_radar_data message
 *
 * @return [cm] Ground radar distance in centimeters
 */
static inline uint16_t mavlink_msg_multi_radar_data_get_ground_distance(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  0);
}

/**
 * @brief Get field forward_distance from multi_radar_data message
 *
 * @return [cm] Forward radar distance in centimeters
 */
static inline uint16_t mavlink_msg_multi_radar_data_get_forward_distance(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  2);
}

/**
 * @brief Get field backward_distance from multi_radar_data message
 *
 * @return [cm] Backward radar distance in centimeters
 */
static inline uint16_t mavlink_msg_multi_radar_data_get_backward_distance(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  4);
}

/**
 * @brief Decode a multi_radar_data message into a struct
 *
 * @param msg The message to decode
 * @param multi_radar_data C-struct to decode the message contents into
 */
static inline void mavlink_msg_multi_radar_data_decode(const mavlink_message_t* msg, mavlink_multi_radar_data_t* multi_radar_data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    multi_radar_data->ground_distance = mavlink_msg_multi_radar_data_get_ground_distance(msg);
    multi_radar_data->forward_distance = mavlink_msg_multi_radar_data_get_forward_distance(msg);
    multi_radar_data->backward_distance = mavlink_msg_multi_radar_data_get_backward_distance(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_MULTI_RADAR_DATA_LEN? msg->len : MAVLINK_MSG_ID_MULTI_RADAR_DATA_LEN;
        memset(multi_radar_data, 0, MAVLINK_MSG_ID_MULTI_RADAR_DATA_LEN);
    memcpy(multi_radar_data, _MAV_PAYLOAD(msg), len);
#endif
}
