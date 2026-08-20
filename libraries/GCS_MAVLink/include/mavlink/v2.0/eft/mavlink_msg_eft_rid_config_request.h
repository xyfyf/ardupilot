#pragma once
// MESSAGE EFT_RID_CONFIG_REQUEST PACKING

#define MAVLINK_MSG_ID_EFT_RID_CONFIG_REQUEST 517


typedef struct __mavlink_eft_rid_config_request_t {
 uint8_t target_system; /*<  System ID.*/
 uint8_t target_component; /*<  Component ID.*/
 uint8_t seq; /*<  Request sequence number echoed in the reply.*/
 uint8_t type; /*<  0: query current status; 1: clear RID config/status data then reply; 2: clear FactorySN (four SN groups) then reply.*/
} mavlink_eft_rid_config_request_t;

#define MAVLINK_MSG_ID_EFT_RID_CONFIG_REQUEST_LEN 4
#define MAVLINK_MSG_ID_EFT_RID_CONFIG_REQUEST_MIN_LEN 4
#define MAVLINK_MSG_ID_517_LEN 4
#define MAVLINK_MSG_ID_517_MIN_LEN 4

#define MAVLINK_MSG_ID_EFT_RID_CONFIG_REQUEST_CRC 209
#define MAVLINK_MSG_ID_517_CRC 209



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_EFT_RID_CONFIG_REQUEST { \
    517, \
    "EFT_RID_CONFIG_REQUEST", \
    4, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_eft_rid_config_request_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_eft_rid_config_request_t, target_component) }, \
         { "seq", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_eft_rid_config_request_t, seq) }, \
         { "type", NULL, MAVLINK_TYPE_UINT8_T, 0, 3, offsetof(mavlink_eft_rid_config_request_t, type) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_EFT_RID_CONFIG_REQUEST { \
    "EFT_RID_CONFIG_REQUEST", \
    4, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_eft_rid_config_request_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_eft_rid_config_request_t, target_component) }, \
         { "seq", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_eft_rid_config_request_t, seq) }, \
         { "type", NULL, MAVLINK_TYPE_UINT8_T, 0, 3, offsetof(mavlink_eft_rid_config_request_t, type) }, \
         } \
}
#endif

/**
 * @brief Pack a eft_rid_config_request message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system  System ID.
 * @param target_component  Component ID.
 * @param seq  Request sequence number echoed in the reply.
 * @param type  0: query current status; 1: clear RID config/status data then reply; 2: clear FactorySN (four SN groups) then reply.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_eft_rid_config_request_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t target_system, uint8_t target_component, uint8_t seq, uint8_t type)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_EFT_RID_CONFIG_REQUEST_LEN];
    _mav_put_uint8_t(buf, 0, target_system);
    _mav_put_uint8_t(buf, 1, target_component);
    _mav_put_uint8_t(buf, 2, seq);
    _mav_put_uint8_t(buf, 3, type);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_EFT_RID_CONFIG_REQUEST_LEN);
#else
    mavlink_eft_rid_config_request_t packet;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.seq = seq;
    packet.type = type;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_EFT_RID_CONFIG_REQUEST_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_EFT_RID_CONFIG_REQUEST;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_EFT_RID_CONFIG_REQUEST_MIN_LEN, MAVLINK_MSG_ID_EFT_RID_CONFIG_REQUEST_LEN, MAVLINK_MSG_ID_EFT_RID_CONFIG_REQUEST_CRC);
}

/**
 * @brief Pack a eft_rid_config_request message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system  System ID.
 * @param target_component  Component ID.
 * @param seq  Request sequence number echoed in the reply.
 * @param type  0: query current status; 1: clear RID config/status data then reply; 2: clear FactorySN (four SN groups) then reply.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_eft_rid_config_request_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint8_t target_system, uint8_t target_component, uint8_t seq, uint8_t type)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_EFT_RID_CONFIG_REQUEST_LEN];
    _mav_put_uint8_t(buf, 0, target_system);
    _mav_put_uint8_t(buf, 1, target_component);
    _mav_put_uint8_t(buf, 2, seq);
    _mav_put_uint8_t(buf, 3, type);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_EFT_RID_CONFIG_REQUEST_LEN);
#else
    mavlink_eft_rid_config_request_t packet;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.seq = seq;
    packet.type = type;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_EFT_RID_CONFIG_REQUEST_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_EFT_RID_CONFIG_REQUEST;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_EFT_RID_CONFIG_REQUEST_MIN_LEN, MAVLINK_MSG_ID_EFT_RID_CONFIG_REQUEST_LEN, MAVLINK_MSG_ID_EFT_RID_CONFIG_REQUEST_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_EFT_RID_CONFIG_REQUEST_MIN_LEN, MAVLINK_MSG_ID_EFT_RID_CONFIG_REQUEST_LEN);
#endif
}

/**
 * @brief Pack a eft_rid_config_request message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system  System ID.
 * @param target_component  Component ID.
 * @param seq  Request sequence number echoed in the reply.
 * @param type  0: query current status; 1: clear RID config/status data then reply; 2: clear FactorySN (four SN groups) then reply.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_eft_rid_config_request_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t target_system,uint8_t target_component,uint8_t seq,uint8_t type)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_EFT_RID_CONFIG_REQUEST_LEN];
    _mav_put_uint8_t(buf, 0, target_system);
    _mav_put_uint8_t(buf, 1, target_component);
    _mav_put_uint8_t(buf, 2, seq);
    _mav_put_uint8_t(buf, 3, type);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_EFT_RID_CONFIG_REQUEST_LEN);
#else
    mavlink_eft_rid_config_request_t packet;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.seq = seq;
    packet.type = type;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_EFT_RID_CONFIG_REQUEST_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_EFT_RID_CONFIG_REQUEST;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_EFT_RID_CONFIG_REQUEST_MIN_LEN, MAVLINK_MSG_ID_EFT_RID_CONFIG_REQUEST_LEN, MAVLINK_MSG_ID_EFT_RID_CONFIG_REQUEST_CRC);
}

/**
 * @brief Encode a eft_rid_config_request struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param eft_rid_config_request C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_eft_rid_config_request_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_eft_rid_config_request_t* eft_rid_config_request)
{
    return mavlink_msg_eft_rid_config_request_pack(system_id, component_id, msg, eft_rid_config_request->target_system, eft_rid_config_request->target_component, eft_rid_config_request->seq, eft_rid_config_request->type);
}

/**
 * @brief Encode a eft_rid_config_request struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param eft_rid_config_request C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_eft_rid_config_request_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_eft_rid_config_request_t* eft_rid_config_request)
{
    return mavlink_msg_eft_rid_config_request_pack_chan(system_id, component_id, chan, msg, eft_rid_config_request->target_system, eft_rid_config_request->target_component, eft_rid_config_request->seq, eft_rid_config_request->type);
}

/**
 * @brief Encode a eft_rid_config_request struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param eft_rid_config_request C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_eft_rid_config_request_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_eft_rid_config_request_t* eft_rid_config_request)
{
    return mavlink_msg_eft_rid_config_request_pack_status(system_id, component_id, _status, msg,  eft_rid_config_request->target_system, eft_rid_config_request->target_component, eft_rid_config_request->seq, eft_rid_config_request->type);
}

/**
 * @brief Send a eft_rid_config_request message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system  System ID.
 * @param target_component  Component ID.
 * @param seq  Request sequence number echoed in the reply.
 * @param type  0: query current status; 1: clear RID config/status data then reply; 2: clear FactorySN (four SN groups) then reply.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_eft_rid_config_request_send(mavlink_channel_t chan, uint8_t target_system, uint8_t target_component, uint8_t seq, uint8_t type)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_EFT_RID_CONFIG_REQUEST_LEN];
    _mav_put_uint8_t(buf, 0, target_system);
    _mav_put_uint8_t(buf, 1, target_component);
    _mav_put_uint8_t(buf, 2, seq);
    _mav_put_uint8_t(buf, 3, type);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_EFT_RID_CONFIG_REQUEST, buf, MAVLINK_MSG_ID_EFT_RID_CONFIG_REQUEST_MIN_LEN, MAVLINK_MSG_ID_EFT_RID_CONFIG_REQUEST_LEN, MAVLINK_MSG_ID_EFT_RID_CONFIG_REQUEST_CRC);
#else
    mavlink_eft_rid_config_request_t packet;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.seq = seq;
    packet.type = type;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_EFT_RID_CONFIG_REQUEST, (const char *)&packet, MAVLINK_MSG_ID_EFT_RID_CONFIG_REQUEST_MIN_LEN, MAVLINK_MSG_ID_EFT_RID_CONFIG_REQUEST_LEN, MAVLINK_MSG_ID_EFT_RID_CONFIG_REQUEST_CRC);
#endif
}

/**
 * @brief Send a eft_rid_config_request message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_eft_rid_config_request_send_struct(mavlink_channel_t chan, const mavlink_eft_rid_config_request_t* eft_rid_config_request)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_eft_rid_config_request_send(chan, eft_rid_config_request->target_system, eft_rid_config_request->target_component, eft_rid_config_request->seq, eft_rid_config_request->type);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_EFT_RID_CONFIG_REQUEST, (const char *)eft_rid_config_request, MAVLINK_MSG_ID_EFT_RID_CONFIG_REQUEST_MIN_LEN, MAVLINK_MSG_ID_EFT_RID_CONFIG_REQUEST_LEN, MAVLINK_MSG_ID_EFT_RID_CONFIG_REQUEST_CRC);
#endif
}

#if MAVLINK_MSG_ID_EFT_RID_CONFIG_REQUEST_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by reusing
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_eft_rid_config_request_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t target_system, uint8_t target_component, uint8_t seq, uint8_t type)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint8_t(buf, 0, target_system);
    _mav_put_uint8_t(buf, 1, target_component);
    _mav_put_uint8_t(buf, 2, seq);
    _mav_put_uint8_t(buf, 3, type);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_EFT_RID_CONFIG_REQUEST, buf, MAVLINK_MSG_ID_EFT_RID_CONFIG_REQUEST_MIN_LEN, MAVLINK_MSG_ID_EFT_RID_CONFIG_REQUEST_LEN, MAVLINK_MSG_ID_EFT_RID_CONFIG_REQUEST_CRC);
#else
    mavlink_eft_rid_config_request_t *packet = (mavlink_eft_rid_config_request_t *)msgbuf;
    packet->target_system = target_system;
    packet->target_component = target_component;
    packet->seq = seq;
    packet->type = type;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_EFT_RID_CONFIG_REQUEST, (const char *)packet, MAVLINK_MSG_ID_EFT_RID_CONFIG_REQUEST_MIN_LEN, MAVLINK_MSG_ID_EFT_RID_CONFIG_REQUEST_LEN, MAVLINK_MSG_ID_EFT_RID_CONFIG_REQUEST_CRC);
#endif
}
#endif

#endif

// MESSAGE EFT_RID_CONFIG_REQUEST UNPACKING


/**
 * @brief Get field target_system from eft_rid_config_request message
 *
 * @return  System ID.
 */
static inline uint8_t mavlink_msg_eft_rid_config_request_get_target_system(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field target_component from eft_rid_config_request message
 *
 * @return  Component ID.
 */
static inline uint8_t mavlink_msg_eft_rid_config_request_get_target_component(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Get field seq from eft_rid_config_request message
 *
 * @return  Request sequence number echoed in the reply.
 */
static inline uint8_t mavlink_msg_eft_rid_config_request_get_seq(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  2);
}

/**
 * @brief Get field type from eft_rid_config_request message
 *
 * @return  0: query current status; 1: clear RID config/status data then reply; 2: clear FactorySN (four SN groups) then reply.
 */
static inline uint8_t mavlink_msg_eft_rid_config_request_get_type(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  3);
}

/**
 * @brief Decode a eft_rid_config_request message into a struct
 *
 * @param msg The message to decode
 * @param eft_rid_config_request C-struct to decode the message contents into
 */
static inline void mavlink_msg_eft_rid_config_request_decode(const mavlink_message_t* msg, mavlink_eft_rid_config_request_t* eft_rid_config_request)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    eft_rid_config_request->target_system = mavlink_msg_eft_rid_config_request_get_target_system(msg);
    eft_rid_config_request->target_component = mavlink_msg_eft_rid_config_request_get_target_component(msg);
    eft_rid_config_request->seq = mavlink_msg_eft_rid_config_request_get_seq(msg);
    eft_rid_config_request->type = mavlink_msg_eft_rid_config_request_get_type(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_EFT_RID_CONFIG_REQUEST_LEN? msg->len : MAVLINK_MSG_ID_EFT_RID_CONFIG_REQUEST_LEN;
        memset(eft_rid_config_request, 0, MAVLINK_MSG_ID_EFT_RID_CONFIG_REQUEST_LEN);
    memcpy(eft_rid_config_request, _MAV_PAYLOAD(msg), len);
#endif
}
