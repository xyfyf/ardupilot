#pragma once
// MESSAGE UOM_FC_STATUS PACKING

#define MAVLINK_MSG_ID_UOM_FC_STATUS 520


typedef struct __mavlink_uom_fc_status_t {
 uint16_t status_code; /*<  Last known UOM platform status code (0=unknown)*/
 uint16_t status_age_s; /*<  Seconds since last GCS update (0xFFFF=never received since boot)*/
 uint8_t allow_arm; /*<  Arming permission: 0=prohibited, 1=allowed*/
 uint8_t is_armed; /*<  FC armed state: 0=disarmed, 1=armed*/
} mavlink_uom_fc_status_t;

#define MAVLINK_MSG_ID_UOM_FC_STATUS_LEN 6
#define MAVLINK_MSG_ID_UOM_FC_STATUS_MIN_LEN 6
#define MAVLINK_MSG_ID_520_LEN 6
#define MAVLINK_MSG_ID_520_MIN_LEN 6

#define MAVLINK_MSG_ID_UOM_FC_STATUS_CRC 205
#define MAVLINK_MSG_ID_520_CRC 205



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_UOM_FC_STATUS { \
    520, \
    "UOM_FC_STATUS", \
    4, \
    {  { "status_code", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_uom_fc_status_t, status_code) }, \
         { "status_age_s", NULL, MAVLINK_TYPE_UINT16_T, 0, 2, offsetof(mavlink_uom_fc_status_t, status_age_s) }, \
         { "allow_arm", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_uom_fc_status_t, allow_arm) }, \
         { "is_armed", NULL, MAVLINK_TYPE_UINT8_T, 0, 5, offsetof(mavlink_uom_fc_status_t, is_armed) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_UOM_FC_STATUS { \
    "UOM_FC_STATUS", \
    4, \
    {  { "status_code", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_uom_fc_status_t, status_code) }, \
         { "status_age_s", NULL, MAVLINK_TYPE_UINT16_T, 0, 2, offsetof(mavlink_uom_fc_status_t, status_age_s) }, \
         { "allow_arm", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_uom_fc_status_t, allow_arm) }, \
         { "is_armed", NULL, MAVLINK_TYPE_UINT8_T, 0, 5, offsetof(mavlink_uom_fc_status_t, is_armed) }, \
         } \
}
#endif

/**
 * @brief Pack a uom_fc_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param status_code  Last known UOM platform status code (0=unknown)
 * @param status_age_s  Seconds since last GCS update (0xFFFF=never received since boot)
 * @param allow_arm  Arming permission: 0=prohibited, 1=allowed
 * @param is_armed  FC armed state: 0=disarmed, 1=armed
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_uom_fc_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint16_t status_code, uint16_t status_age_s, uint8_t allow_arm, uint8_t is_armed)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_UOM_FC_STATUS_LEN];
    _mav_put_uint16_t(buf, 0, status_code);
    _mav_put_uint16_t(buf, 2, status_age_s);
    _mav_put_uint8_t(buf, 4, allow_arm);
    _mav_put_uint8_t(buf, 5, is_armed);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_UOM_FC_STATUS_LEN);
#else
    mavlink_uom_fc_status_t packet;
    packet.status_code = status_code;
    packet.status_age_s = status_age_s;
    packet.allow_arm = allow_arm;
    packet.is_armed = is_armed;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_UOM_FC_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_UOM_FC_STATUS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_UOM_FC_STATUS_MIN_LEN, MAVLINK_MSG_ID_UOM_FC_STATUS_LEN, MAVLINK_MSG_ID_UOM_FC_STATUS_CRC);
}

/**
 * @brief Pack a uom_fc_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param status_code  Last known UOM platform status code (0=unknown)
 * @param status_age_s  Seconds since last GCS update (0xFFFF=never received since boot)
 * @param allow_arm  Arming permission: 0=prohibited, 1=allowed
 * @param is_armed  FC armed state: 0=disarmed, 1=armed
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_uom_fc_status_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint16_t status_code, uint16_t status_age_s, uint8_t allow_arm, uint8_t is_armed)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_UOM_FC_STATUS_LEN];
    _mav_put_uint16_t(buf, 0, status_code);
    _mav_put_uint16_t(buf, 2, status_age_s);
    _mav_put_uint8_t(buf, 4, allow_arm);
    _mav_put_uint8_t(buf, 5, is_armed);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_UOM_FC_STATUS_LEN);
#else
    mavlink_uom_fc_status_t packet;
    packet.status_code = status_code;
    packet.status_age_s = status_age_s;
    packet.allow_arm = allow_arm;
    packet.is_armed = is_armed;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_UOM_FC_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_UOM_FC_STATUS;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_UOM_FC_STATUS_MIN_LEN, MAVLINK_MSG_ID_UOM_FC_STATUS_LEN, MAVLINK_MSG_ID_UOM_FC_STATUS_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_UOM_FC_STATUS_MIN_LEN, MAVLINK_MSG_ID_UOM_FC_STATUS_LEN);
#endif
}

/**
 * @brief Pack a uom_fc_status message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param status_code  Last known UOM platform status code (0=unknown)
 * @param status_age_s  Seconds since last GCS update (0xFFFF=never received since boot)
 * @param allow_arm  Arming permission: 0=prohibited, 1=allowed
 * @param is_armed  FC armed state: 0=disarmed, 1=armed
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_uom_fc_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint16_t status_code,uint16_t status_age_s,uint8_t allow_arm,uint8_t is_armed)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_UOM_FC_STATUS_LEN];
    _mav_put_uint16_t(buf, 0, status_code);
    _mav_put_uint16_t(buf, 2, status_age_s);
    _mav_put_uint8_t(buf, 4, allow_arm);
    _mav_put_uint8_t(buf, 5, is_armed);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_UOM_FC_STATUS_LEN);
#else
    mavlink_uom_fc_status_t packet;
    packet.status_code = status_code;
    packet.status_age_s = status_age_s;
    packet.allow_arm = allow_arm;
    packet.is_armed = is_armed;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_UOM_FC_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_UOM_FC_STATUS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_UOM_FC_STATUS_MIN_LEN, MAVLINK_MSG_ID_UOM_FC_STATUS_LEN, MAVLINK_MSG_ID_UOM_FC_STATUS_CRC);
}

/**
 * @brief Encode a uom_fc_status struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param uom_fc_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_uom_fc_status_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_uom_fc_status_t* uom_fc_status)
{
    return mavlink_msg_uom_fc_status_pack(system_id, component_id, msg, uom_fc_status->status_code, uom_fc_status->status_age_s, uom_fc_status->allow_arm, uom_fc_status->is_armed);
}

/**
 * @brief Encode a uom_fc_status struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param uom_fc_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_uom_fc_status_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_uom_fc_status_t* uom_fc_status)
{
    return mavlink_msg_uom_fc_status_pack_chan(system_id, component_id, chan, msg, uom_fc_status->status_code, uom_fc_status->status_age_s, uom_fc_status->allow_arm, uom_fc_status->is_armed);
}

/**
 * @brief Encode a uom_fc_status struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param uom_fc_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_uom_fc_status_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_uom_fc_status_t* uom_fc_status)
{
    return mavlink_msg_uom_fc_status_pack_status(system_id, component_id, _status, msg,  uom_fc_status->status_code, uom_fc_status->status_age_s, uom_fc_status->allow_arm, uom_fc_status->is_armed);
}

/**
 * @brief Send a uom_fc_status message
 * @param chan MAVLink channel to send the message
 *
 * @param status_code  Last known UOM platform status code (0=unknown)
 * @param status_age_s  Seconds since last GCS update (0xFFFF=never received since boot)
 * @param allow_arm  Arming permission: 0=prohibited, 1=allowed
 * @param is_armed  FC armed state: 0=disarmed, 1=armed
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_uom_fc_status_send(mavlink_channel_t chan, uint16_t status_code, uint16_t status_age_s, uint8_t allow_arm, uint8_t is_armed)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_UOM_FC_STATUS_LEN];
    _mav_put_uint16_t(buf, 0, status_code);
    _mav_put_uint16_t(buf, 2, status_age_s);
    _mav_put_uint8_t(buf, 4, allow_arm);
    _mav_put_uint8_t(buf, 5, is_armed);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UOM_FC_STATUS, buf, MAVLINK_MSG_ID_UOM_FC_STATUS_MIN_LEN, MAVLINK_MSG_ID_UOM_FC_STATUS_LEN, MAVLINK_MSG_ID_UOM_FC_STATUS_CRC);
#else
    mavlink_uom_fc_status_t packet;
    packet.status_code = status_code;
    packet.status_age_s = status_age_s;
    packet.allow_arm = allow_arm;
    packet.is_armed = is_armed;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UOM_FC_STATUS, (const char *)&packet, MAVLINK_MSG_ID_UOM_FC_STATUS_MIN_LEN, MAVLINK_MSG_ID_UOM_FC_STATUS_LEN, MAVLINK_MSG_ID_UOM_FC_STATUS_CRC);
#endif
}

/**
 * @brief Send a uom_fc_status message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_uom_fc_status_send_struct(mavlink_channel_t chan, const mavlink_uom_fc_status_t* uom_fc_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_uom_fc_status_send(chan, uom_fc_status->status_code, uom_fc_status->status_age_s, uom_fc_status->allow_arm, uom_fc_status->is_armed);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UOM_FC_STATUS, (const char *)uom_fc_status, MAVLINK_MSG_ID_UOM_FC_STATUS_MIN_LEN, MAVLINK_MSG_ID_UOM_FC_STATUS_LEN, MAVLINK_MSG_ID_UOM_FC_STATUS_CRC);
#endif
}

#if MAVLINK_MSG_ID_UOM_FC_STATUS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by reusing
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_uom_fc_status_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint16_t status_code, uint16_t status_age_s, uint8_t allow_arm, uint8_t is_armed)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint16_t(buf, 0, status_code);
    _mav_put_uint16_t(buf, 2, status_age_s);
    _mav_put_uint8_t(buf, 4, allow_arm);
    _mav_put_uint8_t(buf, 5, is_armed);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UOM_FC_STATUS, buf, MAVLINK_MSG_ID_UOM_FC_STATUS_MIN_LEN, MAVLINK_MSG_ID_UOM_FC_STATUS_LEN, MAVLINK_MSG_ID_UOM_FC_STATUS_CRC);
#else
    mavlink_uom_fc_status_t *packet = (mavlink_uom_fc_status_t *)msgbuf;
    packet->status_code = status_code;
    packet->status_age_s = status_age_s;
    packet->allow_arm = allow_arm;
    packet->is_armed = is_armed;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UOM_FC_STATUS, (const char *)packet, MAVLINK_MSG_ID_UOM_FC_STATUS_MIN_LEN, MAVLINK_MSG_ID_UOM_FC_STATUS_LEN, MAVLINK_MSG_ID_UOM_FC_STATUS_CRC);
#endif
}
#endif

#endif

// MESSAGE UOM_FC_STATUS UNPACKING


/**
 * @brief Get field status_code from uom_fc_status message
 *
 * @return  Last known UOM platform status code (0=unknown)
 */
static inline uint16_t mavlink_msg_uom_fc_status_get_status_code(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  0);
}

/**
 * @brief Get field status_age_s from uom_fc_status message
 *
 * @return  Seconds since last GCS update (0xFFFF=never received since boot)
 */
static inline uint16_t mavlink_msg_uom_fc_status_get_status_age_s(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  2);
}

/**
 * @brief Get field allow_arm from uom_fc_status message
 *
 * @return  Arming permission: 0=prohibited, 1=allowed
 */
static inline uint8_t mavlink_msg_uom_fc_status_get_allow_arm(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Get field is_armed from uom_fc_status message
 *
 * @return  FC armed state: 0=disarmed, 1=armed
 */
static inline uint8_t mavlink_msg_uom_fc_status_get_is_armed(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  5);
}

/**
 * @brief Decode a uom_fc_status message into a struct
 *
 * @param msg The message to decode
 * @param uom_fc_status C-struct to decode the message contents into
 */
static inline void mavlink_msg_uom_fc_status_decode(const mavlink_message_t* msg, mavlink_uom_fc_status_t* uom_fc_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    uom_fc_status->status_code = mavlink_msg_uom_fc_status_get_status_code(msg);
    uom_fc_status->status_age_s = mavlink_msg_uom_fc_status_get_status_age_s(msg);
    uom_fc_status->allow_arm = mavlink_msg_uom_fc_status_get_allow_arm(msg);
    uom_fc_status->is_armed = mavlink_msg_uom_fc_status_get_is_armed(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_UOM_FC_STATUS_LEN? msg->len : MAVLINK_MSG_ID_UOM_FC_STATUS_LEN;
        memset(uom_fc_status, 0, MAVLINK_MSG_ID_UOM_FC_STATUS_LEN);
    memcpy(uom_fc_status, _MAV_PAYLOAD(msg), len);
#endif
}
