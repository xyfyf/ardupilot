#pragma once
// MESSAGE UOM_ARM_STATUS PACKING

#define MAVLINK_MSG_ID_UOM_ARM_STATUS 519


typedef struct __mavlink_uom_arm_status_t {
 uint16_t status_code; /*<  UOM platform status code*/
} mavlink_uom_arm_status_t;

#define MAVLINK_MSG_ID_UOM_ARM_STATUS_LEN 2
#define MAVLINK_MSG_ID_UOM_ARM_STATUS_MIN_LEN 2
#define MAVLINK_MSG_ID_519_LEN 2
#define MAVLINK_MSG_ID_519_MIN_LEN 2

#define MAVLINK_MSG_ID_UOM_ARM_STATUS_CRC 133
#define MAVLINK_MSG_ID_519_CRC 133



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_UOM_ARM_STATUS { \
    519, \
    "UOM_ARM_STATUS", \
    1, \
    {  { "status_code", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_uom_arm_status_t, status_code) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_UOM_ARM_STATUS { \
    "UOM_ARM_STATUS", \
    1, \
    {  { "status_code", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_uom_arm_status_t, status_code) }, \
         } \
}
#endif

/**
 * @brief Pack a uom_arm_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param status_code  UOM platform status code
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_uom_arm_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint16_t status_code)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_UOM_ARM_STATUS_LEN];
    _mav_put_uint16_t(buf, 0, status_code);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_UOM_ARM_STATUS_LEN);
#else
    mavlink_uom_arm_status_t packet;
    packet.status_code = status_code;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_UOM_ARM_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_UOM_ARM_STATUS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_UOM_ARM_STATUS_MIN_LEN, MAVLINK_MSG_ID_UOM_ARM_STATUS_LEN, MAVLINK_MSG_ID_UOM_ARM_STATUS_CRC);
}

/**
 * @brief Pack a uom_arm_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param status_code  UOM platform status code
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_uom_arm_status_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint16_t status_code)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_UOM_ARM_STATUS_LEN];
    _mav_put_uint16_t(buf, 0, status_code);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_UOM_ARM_STATUS_LEN);
#else
    mavlink_uom_arm_status_t packet;
    packet.status_code = status_code;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_UOM_ARM_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_UOM_ARM_STATUS;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_UOM_ARM_STATUS_MIN_LEN, MAVLINK_MSG_ID_UOM_ARM_STATUS_LEN, MAVLINK_MSG_ID_UOM_ARM_STATUS_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_UOM_ARM_STATUS_MIN_LEN, MAVLINK_MSG_ID_UOM_ARM_STATUS_LEN);
#endif
}

/**
 * @brief Pack a uom_arm_status message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param status_code  UOM platform status code
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_uom_arm_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint16_t status_code)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_UOM_ARM_STATUS_LEN];
    _mav_put_uint16_t(buf, 0, status_code);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_UOM_ARM_STATUS_LEN);
#else
    mavlink_uom_arm_status_t packet;
    packet.status_code = status_code;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_UOM_ARM_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_UOM_ARM_STATUS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_UOM_ARM_STATUS_MIN_LEN, MAVLINK_MSG_ID_UOM_ARM_STATUS_LEN, MAVLINK_MSG_ID_UOM_ARM_STATUS_CRC);
}

/**
 * @brief Encode a uom_arm_status struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param uom_arm_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_uom_arm_status_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_uom_arm_status_t* uom_arm_status)
{
    return mavlink_msg_uom_arm_status_pack(system_id, component_id, msg, uom_arm_status->status_code);
}

/**
 * @brief Encode a uom_arm_status struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param uom_arm_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_uom_arm_status_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_uom_arm_status_t* uom_arm_status)
{
    return mavlink_msg_uom_arm_status_pack_chan(system_id, component_id, chan, msg, uom_arm_status->status_code);
}

/**
 * @brief Encode a uom_arm_status struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param uom_arm_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_uom_arm_status_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_uom_arm_status_t* uom_arm_status)
{
    return mavlink_msg_uom_arm_status_pack_status(system_id, component_id, _status, msg,  uom_arm_status->status_code);
}

/**
 * @brief Send a uom_arm_status message
 * @param chan MAVLink channel to send the message
 *
 * @param status_code  UOM platform status code
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_uom_arm_status_send(mavlink_channel_t chan, uint16_t status_code)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_UOM_ARM_STATUS_LEN];
    _mav_put_uint16_t(buf, 0, status_code);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UOM_ARM_STATUS, buf, MAVLINK_MSG_ID_UOM_ARM_STATUS_MIN_LEN, MAVLINK_MSG_ID_UOM_ARM_STATUS_LEN, MAVLINK_MSG_ID_UOM_ARM_STATUS_CRC);
#else
    mavlink_uom_arm_status_t packet;
    packet.status_code = status_code;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UOM_ARM_STATUS, (const char *)&packet, MAVLINK_MSG_ID_UOM_ARM_STATUS_MIN_LEN, MAVLINK_MSG_ID_UOM_ARM_STATUS_LEN, MAVLINK_MSG_ID_UOM_ARM_STATUS_CRC);
#endif
}

/**
 * @brief Send a uom_arm_status message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_uom_arm_status_send_struct(mavlink_channel_t chan, const mavlink_uom_arm_status_t* uom_arm_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_uom_arm_status_send(chan, uom_arm_status->status_code);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UOM_ARM_STATUS, (const char *)uom_arm_status, MAVLINK_MSG_ID_UOM_ARM_STATUS_MIN_LEN, MAVLINK_MSG_ID_UOM_ARM_STATUS_LEN, MAVLINK_MSG_ID_UOM_ARM_STATUS_CRC);
#endif
}

#if MAVLINK_MSG_ID_UOM_ARM_STATUS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by reusing
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_uom_arm_status_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint16_t status_code)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint16_t(buf, 0, status_code);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UOM_ARM_STATUS, buf, MAVLINK_MSG_ID_UOM_ARM_STATUS_MIN_LEN, MAVLINK_MSG_ID_UOM_ARM_STATUS_LEN, MAVLINK_MSG_ID_UOM_ARM_STATUS_CRC);
#else
    mavlink_uom_arm_status_t *packet = (mavlink_uom_arm_status_t *)msgbuf;
    packet->status_code = status_code;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UOM_ARM_STATUS, (const char *)packet, MAVLINK_MSG_ID_UOM_ARM_STATUS_MIN_LEN, MAVLINK_MSG_ID_UOM_ARM_STATUS_LEN, MAVLINK_MSG_ID_UOM_ARM_STATUS_CRC);
#endif
}
#endif

#endif

// MESSAGE UOM_ARM_STATUS UNPACKING


/**
 * @brief Get field status_code from uom_arm_status message
 *
 * @return  UOM platform status code
 */
static inline uint16_t mavlink_msg_uom_arm_status_get_status_code(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  0);
}

/**
 * @brief Decode a uom_arm_status message into a struct
 *
 * @param msg The message to decode
 * @param uom_arm_status C-struct to decode the message contents into
 */
static inline void mavlink_msg_uom_arm_status_decode(const mavlink_message_t* msg, mavlink_uom_arm_status_t* uom_arm_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    uom_arm_status->status_code = mavlink_msg_uom_arm_status_get_status_code(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_UOM_ARM_STATUS_LEN? msg->len : MAVLINK_MSG_ID_UOM_ARM_STATUS_LEN;
        memset(uom_arm_status, 0, MAVLINK_MSG_ID_UOM_ARM_STATUS_LEN);
    memcpy(uom_arm_status, _MAV_PAYLOAD(msg), len);
#endif
}
