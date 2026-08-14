#pragma once
// MESSAGE PUMP_CALIBRATION_CMD PACKING

#define MAVLINK_MSG_ID_PUMP_CALIBRATION_CMD 507


typedef struct __mavlink_pump_calibration_cmd_t {
 uint8_t calibration_cmd; /*<  Calibration command:11: Start pump1 calibration 12: Stop pump1 calibration 
      21: Start pump2 calibration 22: Stop pump2 calibration 31: Start spreader calibration 32: Stop spreader calibration*/
} mavlink_pump_calibration_cmd_t;

#define MAVLINK_MSG_ID_PUMP_CALIBRATION_CMD_LEN 1
#define MAVLINK_MSG_ID_PUMP_CALIBRATION_CMD_MIN_LEN 1
#define MAVLINK_MSG_ID_507_LEN 1
#define MAVLINK_MSG_ID_507_MIN_LEN 1

#define MAVLINK_MSG_ID_PUMP_CALIBRATION_CMD_CRC 83
#define MAVLINK_MSG_ID_507_CRC 83



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_PUMP_CALIBRATION_CMD { \
    507, \
    "PUMP_CALIBRATION_CMD", \
    1, \
    {  { "calibration_cmd", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_pump_calibration_cmd_t, calibration_cmd) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_PUMP_CALIBRATION_CMD { \
    "PUMP_CALIBRATION_CMD", \
    1, \
    {  { "calibration_cmd", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_pump_calibration_cmd_t, calibration_cmd) }, \
         } \
}
#endif

/**
 * @brief Pack a pump_calibration_cmd message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param calibration_cmd  Calibration command:11: Start pump1 calibration 12: Stop pump1 calibration 
      21: Start pump2 calibration 22: Stop pump2 calibration 31: Start spreader calibration 32: Stop spreader calibration
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_pump_calibration_cmd_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t calibration_cmd)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_PUMP_CALIBRATION_CMD_LEN];
    _mav_put_uint8_t(buf, 0, calibration_cmd);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_PUMP_CALIBRATION_CMD_LEN);
#else
    mavlink_pump_calibration_cmd_t packet;
    packet.calibration_cmd = calibration_cmd;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_PUMP_CALIBRATION_CMD_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_PUMP_CALIBRATION_CMD;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_PUMP_CALIBRATION_CMD_MIN_LEN, MAVLINK_MSG_ID_PUMP_CALIBRATION_CMD_LEN, MAVLINK_MSG_ID_PUMP_CALIBRATION_CMD_CRC);
}

/**
 * @brief Pack a pump_calibration_cmd message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param calibration_cmd  Calibration command:11: Start pump1 calibration 12: Stop pump1 calibration 
      21: Start pump2 calibration 22: Stop pump2 calibration 31: Start spreader calibration 32: Stop spreader calibration
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_pump_calibration_cmd_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint8_t calibration_cmd)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_PUMP_CALIBRATION_CMD_LEN];
    _mav_put_uint8_t(buf, 0, calibration_cmd);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_PUMP_CALIBRATION_CMD_LEN);
#else
    mavlink_pump_calibration_cmd_t packet;
    packet.calibration_cmd = calibration_cmd;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_PUMP_CALIBRATION_CMD_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_PUMP_CALIBRATION_CMD;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_PUMP_CALIBRATION_CMD_MIN_LEN, MAVLINK_MSG_ID_PUMP_CALIBRATION_CMD_LEN, MAVLINK_MSG_ID_PUMP_CALIBRATION_CMD_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_PUMP_CALIBRATION_CMD_MIN_LEN, MAVLINK_MSG_ID_PUMP_CALIBRATION_CMD_LEN);
#endif
}

/**
 * @brief Pack a pump_calibration_cmd message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param calibration_cmd  Calibration command:11: Start pump1 calibration 12: Stop pump1 calibration 
      21: Start pump2 calibration 22: Stop pump2 calibration 31: Start spreader calibration 32: Stop spreader calibration
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_pump_calibration_cmd_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t calibration_cmd)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_PUMP_CALIBRATION_CMD_LEN];
    _mav_put_uint8_t(buf, 0, calibration_cmd);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_PUMP_CALIBRATION_CMD_LEN);
#else
    mavlink_pump_calibration_cmd_t packet;
    packet.calibration_cmd = calibration_cmd;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_PUMP_CALIBRATION_CMD_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_PUMP_CALIBRATION_CMD;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_PUMP_CALIBRATION_CMD_MIN_LEN, MAVLINK_MSG_ID_PUMP_CALIBRATION_CMD_LEN, MAVLINK_MSG_ID_PUMP_CALIBRATION_CMD_CRC);
}

/**
 * @brief Encode a pump_calibration_cmd struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param pump_calibration_cmd C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_pump_calibration_cmd_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_pump_calibration_cmd_t* pump_calibration_cmd)
{
    return mavlink_msg_pump_calibration_cmd_pack(system_id, component_id, msg, pump_calibration_cmd->calibration_cmd);
}

/**
 * @brief Encode a pump_calibration_cmd struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param pump_calibration_cmd C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_pump_calibration_cmd_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_pump_calibration_cmd_t* pump_calibration_cmd)
{
    return mavlink_msg_pump_calibration_cmd_pack_chan(system_id, component_id, chan, msg, pump_calibration_cmd->calibration_cmd);
}

/**
 * @brief Encode a pump_calibration_cmd struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param pump_calibration_cmd C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_pump_calibration_cmd_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_pump_calibration_cmd_t* pump_calibration_cmd)
{
    return mavlink_msg_pump_calibration_cmd_pack_status(system_id, component_id, _status, msg,  pump_calibration_cmd->calibration_cmd);
}

/**
 * @brief Send a pump_calibration_cmd message
 * @param chan MAVLink channel to send the message
 *
 * @param calibration_cmd  Calibration command:11: Start pump1 calibration 12: Stop pump1 calibration 
      21: Start pump2 calibration 22: Stop pump2 calibration 31: Start spreader calibration 32: Stop spreader calibration
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_pump_calibration_cmd_send(mavlink_channel_t chan, uint8_t calibration_cmd)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_PUMP_CALIBRATION_CMD_LEN];
    _mav_put_uint8_t(buf, 0, calibration_cmd);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PUMP_CALIBRATION_CMD, buf, MAVLINK_MSG_ID_PUMP_CALIBRATION_CMD_MIN_LEN, MAVLINK_MSG_ID_PUMP_CALIBRATION_CMD_LEN, MAVLINK_MSG_ID_PUMP_CALIBRATION_CMD_CRC);
#else
    mavlink_pump_calibration_cmd_t packet;
    packet.calibration_cmd = calibration_cmd;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PUMP_CALIBRATION_CMD, (const char *)&packet, MAVLINK_MSG_ID_PUMP_CALIBRATION_CMD_MIN_LEN, MAVLINK_MSG_ID_PUMP_CALIBRATION_CMD_LEN, MAVLINK_MSG_ID_PUMP_CALIBRATION_CMD_CRC);
#endif
}

/**
 * @brief Send a pump_calibration_cmd message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_pump_calibration_cmd_send_struct(mavlink_channel_t chan, const mavlink_pump_calibration_cmd_t* pump_calibration_cmd)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_pump_calibration_cmd_send(chan, pump_calibration_cmd->calibration_cmd);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PUMP_CALIBRATION_CMD, (const char *)pump_calibration_cmd, MAVLINK_MSG_ID_PUMP_CALIBRATION_CMD_MIN_LEN, MAVLINK_MSG_ID_PUMP_CALIBRATION_CMD_LEN, MAVLINK_MSG_ID_PUMP_CALIBRATION_CMD_CRC);
#endif
}

#if MAVLINK_MSG_ID_PUMP_CALIBRATION_CMD_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by reusing
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_pump_calibration_cmd_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t calibration_cmd)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint8_t(buf, 0, calibration_cmd);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PUMP_CALIBRATION_CMD, buf, MAVLINK_MSG_ID_PUMP_CALIBRATION_CMD_MIN_LEN, MAVLINK_MSG_ID_PUMP_CALIBRATION_CMD_LEN, MAVLINK_MSG_ID_PUMP_CALIBRATION_CMD_CRC);
#else
    mavlink_pump_calibration_cmd_t *packet = (mavlink_pump_calibration_cmd_t *)msgbuf;
    packet->calibration_cmd = calibration_cmd;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PUMP_CALIBRATION_CMD, (const char *)packet, MAVLINK_MSG_ID_PUMP_CALIBRATION_CMD_MIN_LEN, MAVLINK_MSG_ID_PUMP_CALIBRATION_CMD_LEN, MAVLINK_MSG_ID_PUMP_CALIBRATION_CMD_CRC);
#endif
}
#endif

#endif

// MESSAGE PUMP_CALIBRATION_CMD UNPACKING


/**
 * @brief Get field calibration_cmd from pump_calibration_cmd message
 *
 * @return  Calibration command:11: Start pump1 calibration 12: Stop pump1 calibration 
      21: Start pump2 calibration 22: Stop pump2 calibration 31: Start spreader calibration 32: Stop spreader calibration
 */
static inline uint8_t mavlink_msg_pump_calibration_cmd_get_calibration_cmd(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Decode a pump_calibration_cmd message into a struct
 *
 * @param msg The message to decode
 * @param pump_calibration_cmd C-struct to decode the message contents into
 */
static inline void mavlink_msg_pump_calibration_cmd_decode(const mavlink_message_t* msg, mavlink_pump_calibration_cmd_t* pump_calibration_cmd)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    pump_calibration_cmd->calibration_cmd = mavlink_msg_pump_calibration_cmd_get_calibration_cmd(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_PUMP_CALIBRATION_CMD_LEN? msg->len : MAVLINK_MSG_ID_PUMP_CALIBRATION_CMD_LEN;
        memset(pump_calibration_cmd, 0, MAVLINK_MSG_ID_PUMP_CALIBRATION_CMD_LEN);
    memcpy(pump_calibration_cmd, _MAV_PAYLOAD(msg), len);
#endif
}
