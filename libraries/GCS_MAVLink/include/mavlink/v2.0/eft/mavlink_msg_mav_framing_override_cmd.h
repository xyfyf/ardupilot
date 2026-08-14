#pragma once
// MESSAGE MAV_FRAMING_OVERRIDE_CMD PACKING

#define MAVLINK_MSG_ID_MAV_FRAMING_OVERRIDE_CMD 516


typedef struct __mavlink_mav_framing_override_cmd_t {
 uint16_t crc; /*<  CRC value (little-endian) to force into the 2-byte checksum field when bit0 of cmd is set. Ignored when bit0 is clear.*/
 uint8_t cmd; /*<  Override command (0-3): bit0=force CRC to the crc field, bit1=use the magic field byte instead of 0xFD.*/
 uint8_t magic; /*<  Start-of-frame magic byte to use when bit1 of cmd is set (e.g. 0xEF). Ignored when bit1 is clear.*/
} mavlink_mav_framing_override_cmd_t;

#define MAVLINK_MSG_ID_MAV_FRAMING_OVERRIDE_CMD_LEN 4
#define MAVLINK_MSG_ID_MAV_FRAMING_OVERRIDE_CMD_MIN_LEN 4
#define MAVLINK_MSG_ID_516_LEN 4
#define MAVLINK_MSG_ID_516_MIN_LEN 4

#define MAVLINK_MSG_ID_MAV_FRAMING_OVERRIDE_CMD_CRC 253
#define MAVLINK_MSG_ID_516_CRC 253



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_MAV_FRAMING_OVERRIDE_CMD { \
    516, \
    "MAV_FRAMING_OVERRIDE_CMD", \
    3, \
    {  { "cmd", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_mav_framing_override_cmd_t, cmd) }, \
         { "magic", NULL, MAVLINK_TYPE_UINT8_T, 0, 3, offsetof(mavlink_mav_framing_override_cmd_t, magic) }, \
         { "crc", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_mav_framing_override_cmd_t, crc) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_MAV_FRAMING_OVERRIDE_CMD { \
    "MAV_FRAMING_OVERRIDE_CMD", \
    3, \
    {  { "cmd", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_mav_framing_override_cmd_t, cmd) }, \
         { "magic", NULL, MAVLINK_TYPE_UINT8_T, 0, 3, offsetof(mavlink_mav_framing_override_cmd_t, magic) }, \
         { "crc", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_mav_framing_override_cmd_t, crc) }, \
         } \
}
#endif

/**
 * @brief Pack a mav_framing_override_cmd message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param cmd  Override command (0-3): bit0=force CRC to the crc field, bit1=use the magic field byte instead of 0xFD.
 * @param magic  Start-of-frame magic byte to use when bit1 of cmd is set (e.g. 0xEF). Ignored when bit1 is clear.
 * @param crc  CRC value (little-endian) to force into the 2-byte checksum field when bit0 of cmd is set. Ignored when bit0 is clear.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mav_framing_override_cmd_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t cmd, uint8_t magic, uint16_t crc)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MAV_FRAMING_OVERRIDE_CMD_LEN];
    _mav_put_uint16_t(buf, 0, crc);
    _mav_put_uint8_t(buf, 2, cmd);
    _mav_put_uint8_t(buf, 3, magic);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MAV_FRAMING_OVERRIDE_CMD_LEN);
#else
    mavlink_mav_framing_override_cmd_t packet;
    packet.crc = crc;
    packet.cmd = cmd;
    packet.magic = magic;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MAV_FRAMING_OVERRIDE_CMD_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MAV_FRAMING_OVERRIDE_CMD;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MAV_FRAMING_OVERRIDE_CMD_MIN_LEN, MAVLINK_MSG_ID_MAV_FRAMING_OVERRIDE_CMD_LEN, MAVLINK_MSG_ID_MAV_FRAMING_OVERRIDE_CMD_CRC);
}

/**
 * @brief Pack a mav_framing_override_cmd message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param cmd  Override command (0-3): bit0=force CRC to the crc field, bit1=use the magic field byte instead of 0xFD.
 * @param magic  Start-of-frame magic byte to use when bit1 of cmd is set (e.g. 0xEF). Ignored when bit1 is clear.
 * @param crc  CRC value (little-endian) to force into the 2-byte checksum field when bit0 of cmd is set. Ignored when bit0 is clear.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mav_framing_override_cmd_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint8_t cmd, uint8_t magic, uint16_t crc)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MAV_FRAMING_OVERRIDE_CMD_LEN];
    _mav_put_uint16_t(buf, 0, crc);
    _mav_put_uint8_t(buf, 2, cmd);
    _mav_put_uint8_t(buf, 3, magic);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MAV_FRAMING_OVERRIDE_CMD_LEN);
#else
    mavlink_mav_framing_override_cmd_t packet;
    packet.crc = crc;
    packet.cmd = cmd;
    packet.magic = magic;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MAV_FRAMING_OVERRIDE_CMD_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MAV_FRAMING_OVERRIDE_CMD;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_MAV_FRAMING_OVERRIDE_CMD_MIN_LEN, MAVLINK_MSG_ID_MAV_FRAMING_OVERRIDE_CMD_LEN, MAVLINK_MSG_ID_MAV_FRAMING_OVERRIDE_CMD_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_MAV_FRAMING_OVERRIDE_CMD_MIN_LEN, MAVLINK_MSG_ID_MAV_FRAMING_OVERRIDE_CMD_LEN);
#endif
}

/**
 * @brief Pack a mav_framing_override_cmd message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param cmd  Override command (0-3): bit0=force CRC to the crc field, bit1=use the magic field byte instead of 0xFD.
 * @param magic  Start-of-frame magic byte to use when bit1 of cmd is set (e.g. 0xEF). Ignored when bit1 is clear.
 * @param crc  CRC value (little-endian) to force into the 2-byte checksum field when bit0 of cmd is set. Ignored when bit0 is clear.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mav_framing_override_cmd_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t cmd,uint8_t magic,uint16_t crc)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MAV_FRAMING_OVERRIDE_CMD_LEN];
    _mav_put_uint16_t(buf, 0, crc);
    _mav_put_uint8_t(buf, 2, cmd);
    _mav_put_uint8_t(buf, 3, magic);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MAV_FRAMING_OVERRIDE_CMD_LEN);
#else
    mavlink_mav_framing_override_cmd_t packet;
    packet.crc = crc;
    packet.cmd = cmd;
    packet.magic = magic;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MAV_FRAMING_OVERRIDE_CMD_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MAV_FRAMING_OVERRIDE_CMD;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_MAV_FRAMING_OVERRIDE_CMD_MIN_LEN, MAVLINK_MSG_ID_MAV_FRAMING_OVERRIDE_CMD_LEN, MAVLINK_MSG_ID_MAV_FRAMING_OVERRIDE_CMD_CRC);
}

/**
 * @brief Encode a mav_framing_override_cmd struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param mav_framing_override_cmd C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mav_framing_override_cmd_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_mav_framing_override_cmd_t* mav_framing_override_cmd)
{
    return mavlink_msg_mav_framing_override_cmd_pack(system_id, component_id, msg, mav_framing_override_cmd->cmd, mav_framing_override_cmd->magic, mav_framing_override_cmd->crc);
}

/**
 * @brief Encode a mav_framing_override_cmd struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param mav_framing_override_cmd C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mav_framing_override_cmd_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_mav_framing_override_cmd_t* mav_framing_override_cmd)
{
    return mavlink_msg_mav_framing_override_cmd_pack_chan(system_id, component_id, chan, msg, mav_framing_override_cmd->cmd, mav_framing_override_cmd->magic, mav_framing_override_cmd->crc);
}

/**
 * @brief Encode a mav_framing_override_cmd struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param mav_framing_override_cmd C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mav_framing_override_cmd_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_mav_framing_override_cmd_t* mav_framing_override_cmd)
{
    return mavlink_msg_mav_framing_override_cmd_pack_status(system_id, component_id, _status, msg,  mav_framing_override_cmd->cmd, mav_framing_override_cmd->magic, mav_framing_override_cmd->crc);
}

/**
 * @brief Send a mav_framing_override_cmd message
 * @param chan MAVLink channel to send the message
 *
 * @param cmd  Override command (0-3): bit0=force CRC to the crc field, bit1=use the magic field byte instead of 0xFD.
 * @param magic  Start-of-frame magic byte to use when bit1 of cmd is set (e.g. 0xEF). Ignored when bit1 is clear.
 * @param crc  CRC value (little-endian) to force into the 2-byte checksum field when bit0 of cmd is set. Ignored when bit0 is clear.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_mav_framing_override_cmd_send(mavlink_channel_t chan, uint8_t cmd, uint8_t magic, uint16_t crc)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MAV_FRAMING_OVERRIDE_CMD_LEN];
    _mav_put_uint16_t(buf, 0, crc);
    _mav_put_uint8_t(buf, 2, cmd);
    _mav_put_uint8_t(buf, 3, magic);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAV_FRAMING_OVERRIDE_CMD, buf, MAVLINK_MSG_ID_MAV_FRAMING_OVERRIDE_CMD_MIN_LEN, MAVLINK_MSG_ID_MAV_FRAMING_OVERRIDE_CMD_LEN, MAVLINK_MSG_ID_MAV_FRAMING_OVERRIDE_CMD_CRC);
#else
    mavlink_mav_framing_override_cmd_t packet;
    packet.crc = crc;
    packet.cmd = cmd;
    packet.magic = magic;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAV_FRAMING_OVERRIDE_CMD, (const char *)&packet, MAVLINK_MSG_ID_MAV_FRAMING_OVERRIDE_CMD_MIN_LEN, MAVLINK_MSG_ID_MAV_FRAMING_OVERRIDE_CMD_LEN, MAVLINK_MSG_ID_MAV_FRAMING_OVERRIDE_CMD_CRC);
#endif
}

/**
 * @brief Send a mav_framing_override_cmd message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_mav_framing_override_cmd_send_struct(mavlink_channel_t chan, const mavlink_mav_framing_override_cmd_t* mav_framing_override_cmd)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_mav_framing_override_cmd_send(chan, mav_framing_override_cmd->cmd, mav_framing_override_cmd->magic, mav_framing_override_cmd->crc);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAV_FRAMING_OVERRIDE_CMD, (const char *)mav_framing_override_cmd, MAVLINK_MSG_ID_MAV_FRAMING_OVERRIDE_CMD_MIN_LEN, MAVLINK_MSG_ID_MAV_FRAMING_OVERRIDE_CMD_LEN, MAVLINK_MSG_ID_MAV_FRAMING_OVERRIDE_CMD_CRC);
#endif
}

#if MAVLINK_MSG_ID_MAV_FRAMING_OVERRIDE_CMD_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by reusing
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_mav_framing_override_cmd_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t cmd, uint8_t magic, uint16_t crc)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint16_t(buf, 0, crc);
    _mav_put_uint8_t(buf, 2, cmd);
    _mav_put_uint8_t(buf, 3, magic);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAV_FRAMING_OVERRIDE_CMD, buf, MAVLINK_MSG_ID_MAV_FRAMING_OVERRIDE_CMD_MIN_LEN, MAVLINK_MSG_ID_MAV_FRAMING_OVERRIDE_CMD_LEN, MAVLINK_MSG_ID_MAV_FRAMING_OVERRIDE_CMD_CRC);
#else
    mavlink_mav_framing_override_cmd_t *packet = (mavlink_mav_framing_override_cmd_t *)msgbuf;
    packet->crc = crc;
    packet->cmd = cmd;
    packet->magic = magic;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAV_FRAMING_OVERRIDE_CMD, (const char *)packet, MAVLINK_MSG_ID_MAV_FRAMING_OVERRIDE_CMD_MIN_LEN, MAVLINK_MSG_ID_MAV_FRAMING_OVERRIDE_CMD_LEN, MAVLINK_MSG_ID_MAV_FRAMING_OVERRIDE_CMD_CRC);
#endif
}
#endif

#endif

// MESSAGE MAV_FRAMING_OVERRIDE_CMD UNPACKING


/**
 * @brief Get field cmd from mav_framing_override_cmd message
 *
 * @return  Override command (0-3): bit0=force CRC to the crc field, bit1=use the magic field byte instead of 0xFD.
 */
static inline uint8_t mavlink_msg_mav_framing_override_cmd_get_cmd(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  2);
}

/**
 * @brief Get field magic from mav_framing_override_cmd message
 *
 * @return  Start-of-frame magic byte to use when bit1 of cmd is set (e.g. 0xEF). Ignored when bit1 is clear.
 */
static inline uint8_t mavlink_msg_mav_framing_override_cmd_get_magic(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  3);
}

/**
 * @brief Get field crc from mav_framing_override_cmd message
 *
 * @return  CRC value (little-endian) to force into the 2-byte checksum field when bit0 of cmd is set. Ignored when bit0 is clear.
 */
static inline uint16_t mavlink_msg_mav_framing_override_cmd_get_crc(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  0);
}

/**
 * @brief Decode a mav_framing_override_cmd message into a struct
 *
 * @param msg The message to decode
 * @param mav_framing_override_cmd C-struct to decode the message contents into
 */
static inline void mavlink_msg_mav_framing_override_cmd_decode(const mavlink_message_t* msg, mavlink_mav_framing_override_cmd_t* mav_framing_override_cmd)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mav_framing_override_cmd->crc = mavlink_msg_mav_framing_override_cmd_get_crc(msg);
    mav_framing_override_cmd->cmd = mavlink_msg_mav_framing_override_cmd_get_cmd(msg);
    mav_framing_override_cmd->magic = mavlink_msg_mav_framing_override_cmd_get_magic(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_MAV_FRAMING_OVERRIDE_CMD_LEN? msg->len : MAVLINK_MSG_ID_MAV_FRAMING_OVERRIDE_CMD_LEN;
        memset(mav_framing_override_cmd, 0, MAVLINK_MSG_ID_MAV_FRAMING_OVERRIDE_CMD_LEN);
    memcpy(mav_framing_override_cmd, _MAV_PAYLOAD(msg), len);
#endif
}
