#pragma once
// MESSAGE UOM_OPERATOR_ID PACKING

#define MAVLINK_MSG_ID_UOM_OPERATOR_ID 521


typedef struct __mavlink_uom_operator_id_t {
 char operator_id[20]; /*<  UOM operator ID string (ASCII, null-padded to 20 bytes)*/
 uint8_t operator_id_type; /*<  Operator ID type (MAV_ODID_OPERATOR_ID_TYPE)*/
} mavlink_uom_operator_id_t;

#define MAVLINK_MSG_ID_UOM_OPERATOR_ID_LEN 21
#define MAVLINK_MSG_ID_UOM_OPERATOR_ID_MIN_LEN 21
#define MAVLINK_MSG_ID_521_LEN 21
#define MAVLINK_MSG_ID_521_MIN_LEN 21

#define MAVLINK_MSG_ID_UOM_OPERATOR_ID_CRC 34
#define MAVLINK_MSG_ID_521_CRC 34

#define MAVLINK_MSG_UOM_OPERATOR_ID_FIELD_OPERATOR_ID_LEN 20

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_UOM_OPERATOR_ID { \
    521, \
    "UOM_OPERATOR_ID", \
    2, \
    {  { "operator_id", NULL, MAVLINK_TYPE_CHAR, 20, 0, offsetof(mavlink_uom_operator_id_t, operator_id) }, \
         { "operator_id_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 20, offsetof(mavlink_uom_operator_id_t, operator_id_type) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_UOM_OPERATOR_ID { \
    "UOM_OPERATOR_ID", \
    2, \
    {  { "operator_id", NULL, MAVLINK_TYPE_CHAR, 20, 0, offsetof(mavlink_uom_operator_id_t, operator_id) }, \
         { "operator_id_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 20, offsetof(mavlink_uom_operator_id_t, operator_id_type) }, \
         } \
}
#endif

/**
 * @brief Pack a uom_operator_id message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param operator_id  UOM operator ID string (ASCII, null-padded to 20 bytes)
 * @param operator_id_type  Operator ID type (MAV_ODID_OPERATOR_ID_TYPE)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_uom_operator_id_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               const char *operator_id, uint8_t operator_id_type)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_UOM_OPERATOR_ID_LEN];
    _mav_put_uint8_t(buf, 20, operator_id_type);
    _mav_put_char_array(buf, 0, operator_id, 20);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_UOM_OPERATOR_ID_LEN);
#else
    mavlink_uom_operator_id_t packet;
    packet.operator_id_type = operator_id_type;
    mav_array_assign_char(packet.operator_id, operator_id, 20);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_UOM_OPERATOR_ID_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_UOM_OPERATOR_ID;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_UOM_OPERATOR_ID_MIN_LEN, MAVLINK_MSG_ID_UOM_OPERATOR_ID_LEN, MAVLINK_MSG_ID_UOM_OPERATOR_ID_CRC);
}

/**
 * @brief Pack a uom_operator_id message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param operator_id  UOM operator ID string (ASCII, null-padded to 20 bytes)
 * @param operator_id_type  Operator ID type (MAV_ODID_OPERATOR_ID_TYPE)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_uom_operator_id_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               const char *operator_id, uint8_t operator_id_type)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_UOM_OPERATOR_ID_LEN];
    _mav_put_uint8_t(buf, 20, operator_id_type);
    _mav_put_char_array(buf, 0, operator_id, 20);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_UOM_OPERATOR_ID_LEN);
#else
    mavlink_uom_operator_id_t packet;
    packet.operator_id_type = operator_id_type;
    mav_array_memcpy(packet.operator_id, operator_id, sizeof(char)*20);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_UOM_OPERATOR_ID_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_UOM_OPERATOR_ID;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_UOM_OPERATOR_ID_MIN_LEN, MAVLINK_MSG_ID_UOM_OPERATOR_ID_LEN, MAVLINK_MSG_ID_UOM_OPERATOR_ID_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_UOM_OPERATOR_ID_MIN_LEN, MAVLINK_MSG_ID_UOM_OPERATOR_ID_LEN);
#endif
}

/**
 * @brief Pack a uom_operator_id message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param operator_id  UOM operator ID string (ASCII, null-padded to 20 bytes)
 * @param operator_id_type  Operator ID type (MAV_ODID_OPERATOR_ID_TYPE)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_uom_operator_id_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   const char *operator_id,uint8_t operator_id_type)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_UOM_OPERATOR_ID_LEN];
    _mav_put_uint8_t(buf, 20, operator_id_type);
    _mav_put_char_array(buf, 0, operator_id, 20);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_UOM_OPERATOR_ID_LEN);
#else
    mavlink_uom_operator_id_t packet;
    packet.operator_id_type = operator_id_type;
    mav_array_assign_char(packet.operator_id, operator_id, 20);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_UOM_OPERATOR_ID_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_UOM_OPERATOR_ID;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_UOM_OPERATOR_ID_MIN_LEN, MAVLINK_MSG_ID_UOM_OPERATOR_ID_LEN, MAVLINK_MSG_ID_UOM_OPERATOR_ID_CRC);
}

/**
 * @brief Encode a uom_operator_id struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param uom_operator_id C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_uom_operator_id_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_uom_operator_id_t* uom_operator_id)
{
    return mavlink_msg_uom_operator_id_pack(system_id, component_id, msg, uom_operator_id->operator_id, uom_operator_id->operator_id_type);
}

/**
 * @brief Encode a uom_operator_id struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param uom_operator_id C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_uom_operator_id_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_uom_operator_id_t* uom_operator_id)
{
    return mavlink_msg_uom_operator_id_pack_chan(system_id, component_id, chan, msg, uom_operator_id->operator_id, uom_operator_id->operator_id_type);
}

/**
 * @brief Encode a uom_operator_id struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param uom_operator_id C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_uom_operator_id_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_uom_operator_id_t* uom_operator_id)
{
    return mavlink_msg_uom_operator_id_pack_status(system_id, component_id, _status, msg,  uom_operator_id->operator_id, uom_operator_id->operator_id_type);
}

/**
 * @brief Send a uom_operator_id message
 * @param chan MAVLink channel to send the message
 *
 * @param operator_id  UOM operator ID string (ASCII, null-padded to 20 bytes)
 * @param operator_id_type  Operator ID type (MAV_ODID_OPERATOR_ID_TYPE)
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_uom_operator_id_send(mavlink_channel_t chan, const char *operator_id, uint8_t operator_id_type)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_UOM_OPERATOR_ID_LEN];
    _mav_put_uint8_t(buf, 20, operator_id_type);
    _mav_put_char_array(buf, 0, operator_id, 20);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UOM_OPERATOR_ID, buf, MAVLINK_MSG_ID_UOM_OPERATOR_ID_MIN_LEN, MAVLINK_MSG_ID_UOM_OPERATOR_ID_LEN, MAVLINK_MSG_ID_UOM_OPERATOR_ID_CRC);
#else
    mavlink_uom_operator_id_t packet;
    packet.operator_id_type = operator_id_type;
    mav_array_assign_char(packet.operator_id, operator_id, 20);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UOM_OPERATOR_ID, (const char *)&packet, MAVLINK_MSG_ID_UOM_OPERATOR_ID_MIN_LEN, MAVLINK_MSG_ID_UOM_OPERATOR_ID_LEN, MAVLINK_MSG_ID_UOM_OPERATOR_ID_CRC);
#endif
}

/**
 * @brief Send a uom_operator_id message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_uom_operator_id_send_struct(mavlink_channel_t chan, const mavlink_uom_operator_id_t* uom_operator_id)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_uom_operator_id_send(chan, uom_operator_id->operator_id, uom_operator_id->operator_id_type);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UOM_OPERATOR_ID, (const char *)uom_operator_id, MAVLINK_MSG_ID_UOM_OPERATOR_ID_MIN_LEN, MAVLINK_MSG_ID_UOM_OPERATOR_ID_LEN, MAVLINK_MSG_ID_UOM_OPERATOR_ID_CRC);
#endif
}

#if MAVLINK_MSG_ID_UOM_OPERATOR_ID_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by reusing
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_uom_operator_id_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  const char *operator_id, uint8_t operator_id_type)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint8_t(buf, 20, operator_id_type);
    _mav_put_char_array(buf, 0, operator_id, 20);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UOM_OPERATOR_ID, buf, MAVLINK_MSG_ID_UOM_OPERATOR_ID_MIN_LEN, MAVLINK_MSG_ID_UOM_OPERATOR_ID_LEN, MAVLINK_MSG_ID_UOM_OPERATOR_ID_CRC);
#else
    mavlink_uom_operator_id_t *packet = (mavlink_uom_operator_id_t *)msgbuf;
    packet->operator_id_type = operator_id_type;
    mav_array_assign_char(packet->operator_id, operator_id, 20);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UOM_OPERATOR_ID, (const char *)packet, MAVLINK_MSG_ID_UOM_OPERATOR_ID_MIN_LEN, MAVLINK_MSG_ID_UOM_OPERATOR_ID_LEN, MAVLINK_MSG_ID_UOM_OPERATOR_ID_CRC);
#endif
}
#endif

#endif

// MESSAGE UOM_OPERATOR_ID UNPACKING


/**
 * @brief Get field operator_id from uom_operator_id message
 *
 * @return  UOM operator ID string (ASCII, null-padded to 20 bytes)
 */
static inline uint16_t mavlink_msg_uom_operator_id_get_operator_id(const mavlink_message_t* msg, char *operator_id)
{
    return _MAV_RETURN_char_array(msg, operator_id, 20,  0);
}

/**
 * @brief Get field operator_id_type from uom_operator_id message
 *
 * @return  Operator ID type (MAV_ODID_OPERATOR_ID_TYPE)
 */
static inline uint8_t mavlink_msg_uom_operator_id_get_operator_id_type(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  20);
}

/**
 * @brief Decode a uom_operator_id message into a struct
 *
 * @param msg The message to decode
 * @param uom_operator_id C-struct to decode the message contents into
 */
static inline void mavlink_msg_uom_operator_id_decode(const mavlink_message_t* msg, mavlink_uom_operator_id_t* uom_operator_id)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_uom_operator_id_get_operator_id(msg, uom_operator_id->operator_id);
    uom_operator_id->operator_id_type = mavlink_msg_uom_operator_id_get_operator_id_type(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_UOM_OPERATOR_ID_LEN? msg->len : MAVLINK_MSG_ID_UOM_OPERATOR_ID_LEN;
        memset(uom_operator_id, 0, MAVLINK_MSG_ID_UOM_OPERATOR_ID_LEN);
    memcpy(uom_operator_id, _MAV_PAYLOAD(msg), len);
#endif
}
