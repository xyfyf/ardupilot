#pragma once
// MESSAGE PUMP_CALIBRATION_RESULTS PACKING

#define MAVLINK_MSG_ID_PUMP_CALIBRATION_RESULTS 508


typedef struct __mavlink_pump_calibration_results_t {
 uint16_t pwm[6]; /*<  PWM value for each pump (1050-1950)*/
 uint16_t speed[6]; /*<  Motor speed in RPM (0-20000)*/
 uint16_t flow[6]; /*<  Flow rate in L/min*/
} mavlink_pump_calibration_results_t;

#define MAVLINK_MSG_ID_PUMP_CALIBRATION_RESULTS_LEN 36
#define MAVLINK_MSG_ID_PUMP_CALIBRATION_RESULTS_MIN_LEN 36
#define MAVLINK_MSG_ID_508_LEN 36
#define MAVLINK_MSG_ID_508_MIN_LEN 36

#define MAVLINK_MSG_ID_PUMP_CALIBRATION_RESULTS_CRC 65
#define MAVLINK_MSG_ID_508_CRC 65

#define MAVLINK_MSG_PUMP_CALIBRATION_RESULTS_FIELD_PWM_LEN 6
#define MAVLINK_MSG_PUMP_CALIBRATION_RESULTS_FIELD_SPEED_LEN 6
#define MAVLINK_MSG_PUMP_CALIBRATION_RESULTS_FIELD_FLOW_LEN 6

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_PUMP_CALIBRATION_RESULTS { \
    508, \
    "PUMP_CALIBRATION_RESULTS", \
    3, \
    {  { "pwm", NULL, MAVLINK_TYPE_UINT16_T, 6, 0, offsetof(mavlink_pump_calibration_results_t, pwm) }, \
         { "speed", NULL, MAVLINK_TYPE_UINT16_T, 6, 12, offsetof(mavlink_pump_calibration_results_t, speed) }, \
         { "flow", NULL, MAVLINK_TYPE_UINT16_T, 6, 24, offsetof(mavlink_pump_calibration_results_t, flow) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_PUMP_CALIBRATION_RESULTS { \
    "PUMP_CALIBRATION_RESULTS", \
    3, \
    {  { "pwm", NULL, MAVLINK_TYPE_UINT16_T, 6, 0, offsetof(mavlink_pump_calibration_results_t, pwm) }, \
         { "speed", NULL, MAVLINK_TYPE_UINT16_T, 6, 12, offsetof(mavlink_pump_calibration_results_t, speed) }, \
         { "flow", NULL, MAVLINK_TYPE_UINT16_T, 6, 24, offsetof(mavlink_pump_calibration_results_t, flow) }, \
         } \
}
#endif

/**
 * @brief Pack a pump_calibration_results message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param pwm  PWM value for each pump (1050-1950)
 * @param speed  Motor speed in RPM (0-20000)
 * @param flow  Flow rate in L/min
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_pump_calibration_results_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               const uint16_t *pwm, const uint16_t *speed, const uint16_t *flow)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_PUMP_CALIBRATION_RESULTS_LEN];

    _mav_put_uint16_t_array(buf, 0, pwm, 6);
    _mav_put_uint16_t_array(buf, 12, speed, 6);
    _mav_put_uint16_t_array(buf, 24, flow, 6);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_PUMP_CALIBRATION_RESULTS_LEN);
#else
    mavlink_pump_calibration_results_t packet;

    mav_array_assign_uint16_t(packet.pwm, pwm, 6);
    mav_array_assign_uint16_t(packet.speed, speed, 6);
    mav_array_assign_uint16_t(packet.flow, flow, 6);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_PUMP_CALIBRATION_RESULTS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_PUMP_CALIBRATION_RESULTS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_PUMP_CALIBRATION_RESULTS_MIN_LEN, MAVLINK_MSG_ID_PUMP_CALIBRATION_RESULTS_LEN, MAVLINK_MSG_ID_PUMP_CALIBRATION_RESULTS_CRC);
}

/**
 * @brief Pack a pump_calibration_results message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param pwm  PWM value for each pump (1050-1950)
 * @param speed  Motor speed in RPM (0-20000)
 * @param flow  Flow rate in L/min
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_pump_calibration_results_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               const uint16_t *pwm, const uint16_t *speed, const uint16_t *flow)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_PUMP_CALIBRATION_RESULTS_LEN];

    _mav_put_uint16_t_array(buf, 0, pwm, 6);
    _mav_put_uint16_t_array(buf, 12, speed, 6);
    _mav_put_uint16_t_array(buf, 24, flow, 6);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_PUMP_CALIBRATION_RESULTS_LEN);
#else
    mavlink_pump_calibration_results_t packet;

    mav_array_memcpy(packet.pwm, pwm, sizeof(uint16_t)*6);
    mav_array_memcpy(packet.speed, speed, sizeof(uint16_t)*6);
    mav_array_memcpy(packet.flow, flow, sizeof(uint16_t)*6);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_PUMP_CALIBRATION_RESULTS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_PUMP_CALIBRATION_RESULTS;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_PUMP_CALIBRATION_RESULTS_MIN_LEN, MAVLINK_MSG_ID_PUMP_CALIBRATION_RESULTS_LEN, MAVLINK_MSG_ID_PUMP_CALIBRATION_RESULTS_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_PUMP_CALIBRATION_RESULTS_MIN_LEN, MAVLINK_MSG_ID_PUMP_CALIBRATION_RESULTS_LEN);
#endif
}

/**
 * @brief Pack a pump_calibration_results message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param pwm  PWM value for each pump (1050-1950)
 * @param speed  Motor speed in RPM (0-20000)
 * @param flow  Flow rate in L/min
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_pump_calibration_results_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   const uint16_t *pwm,const uint16_t *speed,const uint16_t *flow)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_PUMP_CALIBRATION_RESULTS_LEN];

    _mav_put_uint16_t_array(buf, 0, pwm, 6);
    _mav_put_uint16_t_array(buf, 12, speed, 6);
    _mav_put_uint16_t_array(buf, 24, flow, 6);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_PUMP_CALIBRATION_RESULTS_LEN);
#else
    mavlink_pump_calibration_results_t packet;

    mav_array_assign_uint16_t(packet.pwm, pwm, 6);
    mav_array_assign_uint16_t(packet.speed, speed, 6);
    mav_array_assign_uint16_t(packet.flow, flow, 6);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_PUMP_CALIBRATION_RESULTS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_PUMP_CALIBRATION_RESULTS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_PUMP_CALIBRATION_RESULTS_MIN_LEN, MAVLINK_MSG_ID_PUMP_CALIBRATION_RESULTS_LEN, MAVLINK_MSG_ID_PUMP_CALIBRATION_RESULTS_CRC);
}

/**
 * @brief Encode a pump_calibration_results struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param pump_calibration_results C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_pump_calibration_results_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_pump_calibration_results_t* pump_calibration_results)
{
    return mavlink_msg_pump_calibration_results_pack(system_id, component_id, msg, pump_calibration_results->pwm, pump_calibration_results->speed, pump_calibration_results->flow);
}

/**
 * @brief Encode a pump_calibration_results struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param pump_calibration_results C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_pump_calibration_results_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_pump_calibration_results_t* pump_calibration_results)
{
    return mavlink_msg_pump_calibration_results_pack_chan(system_id, component_id, chan, msg, pump_calibration_results->pwm, pump_calibration_results->speed, pump_calibration_results->flow);
}

/**
 * @brief Encode a pump_calibration_results struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param pump_calibration_results C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_pump_calibration_results_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_pump_calibration_results_t* pump_calibration_results)
{
    return mavlink_msg_pump_calibration_results_pack_status(system_id, component_id, _status, msg,  pump_calibration_results->pwm, pump_calibration_results->speed, pump_calibration_results->flow);
}

/**
 * @brief Send a pump_calibration_results message
 * @param chan MAVLink channel to send the message
 *
 * @param pwm  PWM value for each pump (1050-1950)
 * @param speed  Motor speed in RPM (0-20000)
 * @param flow  Flow rate in L/min
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_pump_calibration_results_send(mavlink_channel_t chan, const uint16_t *pwm, const uint16_t *speed, const uint16_t *flow)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_PUMP_CALIBRATION_RESULTS_LEN];

    _mav_put_uint16_t_array(buf, 0, pwm, 6);
    _mav_put_uint16_t_array(buf, 12, speed, 6);
    _mav_put_uint16_t_array(buf, 24, flow, 6);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PUMP_CALIBRATION_RESULTS, buf, MAVLINK_MSG_ID_PUMP_CALIBRATION_RESULTS_MIN_LEN, MAVLINK_MSG_ID_PUMP_CALIBRATION_RESULTS_LEN, MAVLINK_MSG_ID_PUMP_CALIBRATION_RESULTS_CRC);
#else
    mavlink_pump_calibration_results_t packet;

    mav_array_assign_uint16_t(packet.pwm, pwm, 6);
    mav_array_assign_uint16_t(packet.speed, speed, 6);
    mav_array_assign_uint16_t(packet.flow, flow, 6);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PUMP_CALIBRATION_RESULTS, (const char *)&packet, MAVLINK_MSG_ID_PUMP_CALIBRATION_RESULTS_MIN_LEN, MAVLINK_MSG_ID_PUMP_CALIBRATION_RESULTS_LEN, MAVLINK_MSG_ID_PUMP_CALIBRATION_RESULTS_CRC);
#endif
}

/**
 * @brief Send a pump_calibration_results message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_pump_calibration_results_send_struct(mavlink_channel_t chan, const mavlink_pump_calibration_results_t* pump_calibration_results)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_pump_calibration_results_send(chan, pump_calibration_results->pwm, pump_calibration_results->speed, pump_calibration_results->flow);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PUMP_CALIBRATION_RESULTS, (const char *)pump_calibration_results, MAVLINK_MSG_ID_PUMP_CALIBRATION_RESULTS_MIN_LEN, MAVLINK_MSG_ID_PUMP_CALIBRATION_RESULTS_LEN, MAVLINK_MSG_ID_PUMP_CALIBRATION_RESULTS_CRC);
#endif
}

#if MAVLINK_MSG_ID_PUMP_CALIBRATION_RESULTS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by reusing
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_pump_calibration_results_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  const uint16_t *pwm, const uint16_t *speed, const uint16_t *flow)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;

    _mav_put_uint16_t_array(buf, 0, pwm, 6);
    _mav_put_uint16_t_array(buf, 12, speed, 6);
    _mav_put_uint16_t_array(buf, 24, flow, 6);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PUMP_CALIBRATION_RESULTS, buf, MAVLINK_MSG_ID_PUMP_CALIBRATION_RESULTS_MIN_LEN, MAVLINK_MSG_ID_PUMP_CALIBRATION_RESULTS_LEN, MAVLINK_MSG_ID_PUMP_CALIBRATION_RESULTS_CRC);
#else
    mavlink_pump_calibration_results_t *packet = (mavlink_pump_calibration_results_t *)msgbuf;

    mav_array_assign_uint16_t(packet->pwm, pwm, 6);
    mav_array_assign_uint16_t(packet->speed, speed, 6);
    mav_array_assign_uint16_t(packet->flow, flow, 6);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PUMP_CALIBRATION_RESULTS, (const char *)packet, MAVLINK_MSG_ID_PUMP_CALIBRATION_RESULTS_MIN_LEN, MAVLINK_MSG_ID_PUMP_CALIBRATION_RESULTS_LEN, MAVLINK_MSG_ID_PUMP_CALIBRATION_RESULTS_CRC);
#endif
}
#endif

#endif

// MESSAGE PUMP_CALIBRATION_RESULTS UNPACKING


/**
 * @brief Get field pwm from pump_calibration_results message
 *
 * @return  PWM value for each pump (1050-1950)
 */
static inline uint16_t mavlink_msg_pump_calibration_results_get_pwm(const mavlink_message_t* msg, uint16_t *pwm)
{
    return _MAV_RETURN_uint16_t_array(msg, pwm, 6,  0);
}

/**
 * @brief Get field speed from pump_calibration_results message
 *
 * @return  Motor speed in RPM (0-20000)
 */
static inline uint16_t mavlink_msg_pump_calibration_results_get_speed(const mavlink_message_t* msg, uint16_t *speed)
{
    return _MAV_RETURN_uint16_t_array(msg, speed, 6,  12);
}

/**
 * @brief Get field flow from pump_calibration_results message
 *
 * @return  Flow rate in L/min
 */
static inline uint16_t mavlink_msg_pump_calibration_results_get_flow(const mavlink_message_t* msg, uint16_t *flow)
{
    return _MAV_RETURN_uint16_t_array(msg, flow, 6,  24);
}

/**
 * @brief Decode a pump_calibration_results message into a struct
 *
 * @param msg The message to decode
 * @param pump_calibration_results C-struct to decode the message contents into
 */
static inline void mavlink_msg_pump_calibration_results_decode(const mavlink_message_t* msg, mavlink_pump_calibration_results_t* pump_calibration_results)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_pump_calibration_results_get_pwm(msg, pump_calibration_results->pwm);
    mavlink_msg_pump_calibration_results_get_speed(msg, pump_calibration_results->speed);
    mavlink_msg_pump_calibration_results_get_flow(msg, pump_calibration_results->flow);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_PUMP_CALIBRATION_RESULTS_LEN? msg->len : MAVLINK_MSG_ID_PUMP_CALIBRATION_RESULTS_LEN;
        memset(pump_calibration_results, 0, MAVLINK_MSG_ID_PUMP_CALIBRATION_RESULTS_LEN);
    memcpy(pump_calibration_results, _MAV_PAYLOAD(msg), len);
#endif
}
