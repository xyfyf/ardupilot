#pragma once
// MESSAGE WEIGHT_CALIBRATION PACKING

#define MAVLINK_MSG_ID_WEIGHT_CALIBRATION 505


typedef struct __mavlink_weight_calibration_t {
 uint16_t calibration_weight; /*<  Calibration weight value in grams*/
 uint16_t k_values[3]; /*<  K values for three weight sensors*/
 uint8_t led_control; /*<  LED control flag (0: no control, 1: start control)*/
 uint8_t right_led_brightness; /*<  Right front LED brightness (0-100: 0% to 100%)*/
 uint8_t left_led_brightness; /*<  Left front LED brightness (0-100: 0% to 100%)*/
 uint8_t tare_calibration; /*<  Tare calibration flag (0: no calibration, 1: start calibration)*/
 uint8_t weight_calibration; /*<  Weight calibration flag (0: no calibration, 1: start calibration)*/
 uint8_t k_calibration; /*<  K-value calibration flag (0: no calibration, 1: start calibration)*/
} mavlink_weight_calibration_t;

#define MAVLINK_MSG_ID_WEIGHT_CALIBRATION_LEN 14
#define MAVLINK_MSG_ID_WEIGHT_CALIBRATION_MIN_LEN 14
#define MAVLINK_MSG_ID_505_LEN 14
#define MAVLINK_MSG_ID_505_MIN_LEN 14

#define MAVLINK_MSG_ID_WEIGHT_CALIBRATION_CRC 203
#define MAVLINK_MSG_ID_505_CRC 203

#define MAVLINK_MSG_WEIGHT_CALIBRATION_FIELD_K_VALUES_LEN 3

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_WEIGHT_CALIBRATION { \
    505, \
    "WEIGHT_CALIBRATION", \
    8, \
    {  { "led_control", NULL, MAVLINK_TYPE_UINT8_T, 0, 8, offsetof(mavlink_weight_calibration_t, led_control) }, \
         { "right_led_brightness", NULL, MAVLINK_TYPE_UINT8_T, 0, 9, offsetof(mavlink_weight_calibration_t, right_led_brightness) }, \
         { "left_led_brightness", NULL, MAVLINK_TYPE_UINT8_T, 0, 10, offsetof(mavlink_weight_calibration_t, left_led_brightness) }, \
         { "tare_calibration", NULL, MAVLINK_TYPE_UINT8_T, 0, 11, offsetof(mavlink_weight_calibration_t, tare_calibration) }, \
         { "weight_calibration", NULL, MAVLINK_TYPE_UINT8_T, 0, 12, offsetof(mavlink_weight_calibration_t, weight_calibration) }, \
         { "calibration_weight", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_weight_calibration_t, calibration_weight) }, \
         { "k_calibration", NULL, MAVLINK_TYPE_UINT8_T, 0, 13, offsetof(mavlink_weight_calibration_t, k_calibration) }, \
         { "k_values", NULL, MAVLINK_TYPE_UINT16_T, 3, 2, offsetof(mavlink_weight_calibration_t, k_values) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_WEIGHT_CALIBRATION { \
    "WEIGHT_CALIBRATION", \
    8, \
    {  { "led_control", NULL, MAVLINK_TYPE_UINT8_T, 0, 8, offsetof(mavlink_weight_calibration_t, led_control) }, \
         { "right_led_brightness", NULL, MAVLINK_TYPE_UINT8_T, 0, 9, offsetof(mavlink_weight_calibration_t, right_led_brightness) }, \
         { "left_led_brightness", NULL, MAVLINK_TYPE_UINT8_T, 0, 10, offsetof(mavlink_weight_calibration_t, left_led_brightness) }, \
         { "tare_calibration", NULL, MAVLINK_TYPE_UINT8_T, 0, 11, offsetof(mavlink_weight_calibration_t, tare_calibration) }, \
         { "weight_calibration", NULL, MAVLINK_TYPE_UINT8_T, 0, 12, offsetof(mavlink_weight_calibration_t, weight_calibration) }, \
         { "calibration_weight", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_weight_calibration_t, calibration_weight) }, \
         { "k_calibration", NULL, MAVLINK_TYPE_UINT8_T, 0, 13, offsetof(mavlink_weight_calibration_t, k_calibration) }, \
         { "k_values", NULL, MAVLINK_TYPE_UINT16_T, 3, 2, offsetof(mavlink_weight_calibration_t, k_values) }, \
         } \
}
#endif

/**
 * @brief Pack a weight_calibration message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param led_control  LED control flag (0: no control, 1: start control)
 * @param right_led_brightness  Right front LED brightness (0-100: 0% to 100%)
 * @param left_led_brightness  Left front LED brightness (0-100: 0% to 100%)
 * @param tare_calibration  Tare calibration flag (0: no calibration, 1: start calibration)
 * @param weight_calibration  Weight calibration flag (0: no calibration, 1: start calibration)
 * @param calibration_weight  Calibration weight value in grams
 * @param k_calibration  K-value calibration flag (0: no calibration, 1: start calibration)
 * @param k_values  K values for three weight sensors
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_weight_calibration_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t led_control, uint8_t right_led_brightness, uint8_t left_led_brightness, uint8_t tare_calibration, uint8_t weight_calibration, uint16_t calibration_weight, uint8_t k_calibration, const uint16_t *k_values)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_WEIGHT_CALIBRATION_LEN];
    _mav_put_uint16_t(buf, 0, calibration_weight);
    _mav_put_uint8_t(buf, 8, led_control);
    _mav_put_uint8_t(buf, 9, right_led_brightness);
    _mav_put_uint8_t(buf, 10, left_led_brightness);
    _mav_put_uint8_t(buf, 11, tare_calibration);
    _mav_put_uint8_t(buf, 12, weight_calibration);
    _mav_put_uint8_t(buf, 13, k_calibration);
    _mav_put_uint16_t_array(buf, 2, k_values, 3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_WEIGHT_CALIBRATION_LEN);
#else
    mavlink_weight_calibration_t packet;
    packet.calibration_weight = calibration_weight;
    packet.led_control = led_control;
    packet.right_led_brightness = right_led_brightness;
    packet.left_led_brightness = left_led_brightness;
    packet.tare_calibration = tare_calibration;
    packet.weight_calibration = weight_calibration;
    packet.k_calibration = k_calibration;
    mav_array_assign_uint16_t(packet.k_values, k_values, 3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_WEIGHT_CALIBRATION_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_WEIGHT_CALIBRATION;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_WEIGHT_CALIBRATION_MIN_LEN, MAVLINK_MSG_ID_WEIGHT_CALIBRATION_LEN, MAVLINK_MSG_ID_WEIGHT_CALIBRATION_CRC);
}

/**
 * @brief Pack a weight_calibration message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param led_control  LED control flag (0: no control, 1: start control)
 * @param right_led_brightness  Right front LED brightness (0-100: 0% to 100%)
 * @param left_led_brightness  Left front LED brightness (0-100: 0% to 100%)
 * @param tare_calibration  Tare calibration flag (0: no calibration, 1: start calibration)
 * @param weight_calibration  Weight calibration flag (0: no calibration, 1: start calibration)
 * @param calibration_weight  Calibration weight value in grams
 * @param k_calibration  K-value calibration flag (0: no calibration, 1: start calibration)
 * @param k_values  K values for three weight sensors
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_weight_calibration_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint8_t led_control, uint8_t right_led_brightness, uint8_t left_led_brightness, uint8_t tare_calibration, uint8_t weight_calibration, uint16_t calibration_weight, uint8_t k_calibration, const uint16_t *k_values)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_WEIGHT_CALIBRATION_LEN];
    _mav_put_uint16_t(buf, 0, calibration_weight);
    _mav_put_uint8_t(buf, 8, led_control);
    _mav_put_uint8_t(buf, 9, right_led_brightness);
    _mav_put_uint8_t(buf, 10, left_led_brightness);
    _mav_put_uint8_t(buf, 11, tare_calibration);
    _mav_put_uint8_t(buf, 12, weight_calibration);
    _mav_put_uint8_t(buf, 13, k_calibration);
    _mav_put_uint16_t_array(buf, 2, k_values, 3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_WEIGHT_CALIBRATION_LEN);
#else
    mavlink_weight_calibration_t packet;
    packet.calibration_weight = calibration_weight;
    packet.led_control = led_control;
    packet.right_led_brightness = right_led_brightness;
    packet.left_led_brightness = left_led_brightness;
    packet.tare_calibration = tare_calibration;
    packet.weight_calibration = weight_calibration;
    packet.k_calibration = k_calibration;
    mav_array_memcpy(packet.k_values, k_values, sizeof(uint16_t)*3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_WEIGHT_CALIBRATION_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_WEIGHT_CALIBRATION;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_WEIGHT_CALIBRATION_MIN_LEN, MAVLINK_MSG_ID_WEIGHT_CALIBRATION_LEN, MAVLINK_MSG_ID_WEIGHT_CALIBRATION_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_WEIGHT_CALIBRATION_MIN_LEN, MAVLINK_MSG_ID_WEIGHT_CALIBRATION_LEN);
#endif
}

/**
 * @brief Pack a weight_calibration message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param led_control  LED control flag (0: no control, 1: start control)
 * @param right_led_brightness  Right front LED brightness (0-100: 0% to 100%)
 * @param left_led_brightness  Left front LED brightness (0-100: 0% to 100%)
 * @param tare_calibration  Tare calibration flag (0: no calibration, 1: start calibration)
 * @param weight_calibration  Weight calibration flag (0: no calibration, 1: start calibration)
 * @param calibration_weight  Calibration weight value in grams
 * @param k_calibration  K-value calibration flag (0: no calibration, 1: start calibration)
 * @param k_values  K values for three weight sensors
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_weight_calibration_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t led_control,uint8_t right_led_brightness,uint8_t left_led_brightness,uint8_t tare_calibration,uint8_t weight_calibration,uint16_t calibration_weight,uint8_t k_calibration,const uint16_t *k_values)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_WEIGHT_CALIBRATION_LEN];
    _mav_put_uint16_t(buf, 0, calibration_weight);
    _mav_put_uint8_t(buf, 8, led_control);
    _mav_put_uint8_t(buf, 9, right_led_brightness);
    _mav_put_uint8_t(buf, 10, left_led_brightness);
    _mav_put_uint8_t(buf, 11, tare_calibration);
    _mav_put_uint8_t(buf, 12, weight_calibration);
    _mav_put_uint8_t(buf, 13, k_calibration);
    _mav_put_uint16_t_array(buf, 2, k_values, 3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_WEIGHT_CALIBRATION_LEN);
#else
    mavlink_weight_calibration_t packet;
    packet.calibration_weight = calibration_weight;
    packet.led_control = led_control;
    packet.right_led_brightness = right_led_brightness;
    packet.left_led_brightness = left_led_brightness;
    packet.tare_calibration = tare_calibration;
    packet.weight_calibration = weight_calibration;
    packet.k_calibration = k_calibration;
    mav_array_assign_uint16_t(packet.k_values, k_values, 3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_WEIGHT_CALIBRATION_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_WEIGHT_CALIBRATION;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_WEIGHT_CALIBRATION_MIN_LEN, MAVLINK_MSG_ID_WEIGHT_CALIBRATION_LEN, MAVLINK_MSG_ID_WEIGHT_CALIBRATION_CRC);
}

/**
 * @brief Encode a weight_calibration struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param weight_calibration C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_weight_calibration_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_weight_calibration_t* weight_calibration)
{
    return mavlink_msg_weight_calibration_pack(system_id, component_id, msg, weight_calibration->led_control, weight_calibration->right_led_brightness, weight_calibration->left_led_brightness, weight_calibration->tare_calibration, weight_calibration->weight_calibration, weight_calibration->calibration_weight, weight_calibration->k_calibration, weight_calibration->k_values);
}

/**
 * @brief Encode a weight_calibration struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param weight_calibration C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_weight_calibration_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_weight_calibration_t* weight_calibration)
{
    return mavlink_msg_weight_calibration_pack_chan(system_id, component_id, chan, msg, weight_calibration->led_control, weight_calibration->right_led_brightness, weight_calibration->left_led_brightness, weight_calibration->tare_calibration, weight_calibration->weight_calibration, weight_calibration->calibration_weight, weight_calibration->k_calibration, weight_calibration->k_values);
}

/**
 * @brief Encode a weight_calibration struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param weight_calibration C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_weight_calibration_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_weight_calibration_t* weight_calibration)
{
    return mavlink_msg_weight_calibration_pack_status(system_id, component_id, _status, msg,  weight_calibration->led_control, weight_calibration->right_led_brightness, weight_calibration->left_led_brightness, weight_calibration->tare_calibration, weight_calibration->weight_calibration, weight_calibration->calibration_weight, weight_calibration->k_calibration, weight_calibration->k_values);
}

/**
 * @brief Send a weight_calibration message
 * @param chan MAVLink channel to send the message
 *
 * @param led_control  LED control flag (0: no control, 1: start control)
 * @param right_led_brightness  Right front LED brightness (0-100: 0% to 100%)
 * @param left_led_brightness  Left front LED brightness (0-100: 0% to 100%)
 * @param tare_calibration  Tare calibration flag (0: no calibration, 1: start calibration)
 * @param weight_calibration  Weight calibration flag (0: no calibration, 1: start calibration)
 * @param calibration_weight  Calibration weight value in grams
 * @param k_calibration  K-value calibration flag (0: no calibration, 1: start calibration)
 * @param k_values  K values for three weight sensors
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_weight_calibration_send(mavlink_channel_t chan, uint8_t led_control, uint8_t right_led_brightness, uint8_t left_led_brightness, uint8_t tare_calibration, uint8_t weight_calibration, uint16_t calibration_weight, uint8_t k_calibration, const uint16_t *k_values)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_WEIGHT_CALIBRATION_LEN];
    _mav_put_uint16_t(buf, 0, calibration_weight);
    _mav_put_uint8_t(buf, 8, led_control);
    _mav_put_uint8_t(buf, 9, right_led_brightness);
    _mav_put_uint8_t(buf, 10, left_led_brightness);
    _mav_put_uint8_t(buf, 11, tare_calibration);
    _mav_put_uint8_t(buf, 12, weight_calibration);
    _mav_put_uint8_t(buf, 13, k_calibration);
    _mav_put_uint16_t_array(buf, 2, k_values, 3);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_WEIGHT_CALIBRATION, buf, MAVLINK_MSG_ID_WEIGHT_CALIBRATION_MIN_LEN, MAVLINK_MSG_ID_WEIGHT_CALIBRATION_LEN, MAVLINK_MSG_ID_WEIGHT_CALIBRATION_CRC);
#else
    mavlink_weight_calibration_t packet;
    packet.calibration_weight = calibration_weight;
    packet.led_control = led_control;
    packet.right_led_brightness = right_led_brightness;
    packet.left_led_brightness = left_led_brightness;
    packet.tare_calibration = tare_calibration;
    packet.weight_calibration = weight_calibration;
    packet.k_calibration = k_calibration;
    mav_array_assign_uint16_t(packet.k_values, k_values, 3);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_WEIGHT_CALIBRATION, (const char *)&packet, MAVLINK_MSG_ID_WEIGHT_CALIBRATION_MIN_LEN, MAVLINK_MSG_ID_WEIGHT_CALIBRATION_LEN, MAVLINK_MSG_ID_WEIGHT_CALIBRATION_CRC);
#endif
}

/**
 * @brief Send a weight_calibration message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_weight_calibration_send_struct(mavlink_channel_t chan, const mavlink_weight_calibration_t* weight_calibration)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_weight_calibration_send(chan, weight_calibration->led_control, weight_calibration->right_led_brightness, weight_calibration->left_led_brightness, weight_calibration->tare_calibration, weight_calibration->weight_calibration, weight_calibration->calibration_weight, weight_calibration->k_calibration, weight_calibration->k_values);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_WEIGHT_CALIBRATION, (const char *)weight_calibration, MAVLINK_MSG_ID_WEIGHT_CALIBRATION_MIN_LEN, MAVLINK_MSG_ID_WEIGHT_CALIBRATION_LEN, MAVLINK_MSG_ID_WEIGHT_CALIBRATION_CRC);
#endif
}

#if MAVLINK_MSG_ID_WEIGHT_CALIBRATION_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by reusing
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_weight_calibration_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t led_control, uint8_t right_led_brightness, uint8_t left_led_brightness, uint8_t tare_calibration, uint8_t weight_calibration, uint16_t calibration_weight, uint8_t k_calibration, const uint16_t *k_values)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint16_t(buf, 0, calibration_weight);
    _mav_put_uint8_t(buf, 8, led_control);
    _mav_put_uint8_t(buf, 9, right_led_brightness);
    _mav_put_uint8_t(buf, 10, left_led_brightness);
    _mav_put_uint8_t(buf, 11, tare_calibration);
    _mav_put_uint8_t(buf, 12, weight_calibration);
    _mav_put_uint8_t(buf, 13, k_calibration);
    _mav_put_uint16_t_array(buf, 2, k_values, 3);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_WEIGHT_CALIBRATION, buf, MAVLINK_MSG_ID_WEIGHT_CALIBRATION_MIN_LEN, MAVLINK_MSG_ID_WEIGHT_CALIBRATION_LEN, MAVLINK_MSG_ID_WEIGHT_CALIBRATION_CRC);
#else
    mavlink_weight_calibration_t *packet = (mavlink_weight_calibration_t *)msgbuf;
    packet->calibration_weight = calibration_weight;
    packet->led_control = led_control;
    packet->right_led_brightness = right_led_brightness;
    packet->left_led_brightness = left_led_brightness;
    packet->tare_calibration = tare_calibration;
    packet->weight_calibration = weight_calibration;
    packet->k_calibration = k_calibration;
    mav_array_assign_uint16_t(packet->k_values, k_values, 3);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_WEIGHT_CALIBRATION, (const char *)packet, MAVLINK_MSG_ID_WEIGHT_CALIBRATION_MIN_LEN, MAVLINK_MSG_ID_WEIGHT_CALIBRATION_LEN, MAVLINK_MSG_ID_WEIGHT_CALIBRATION_CRC);
#endif
}
#endif

#endif

// MESSAGE WEIGHT_CALIBRATION UNPACKING


/**
 * @brief Get field led_control from weight_calibration message
 *
 * @return  LED control flag (0: no control, 1: start control)
 */
static inline uint8_t mavlink_msg_weight_calibration_get_led_control(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  8);
}

/**
 * @brief Get field right_led_brightness from weight_calibration message
 *
 * @return  Right front LED brightness (0-100: 0% to 100%)
 */
static inline uint8_t mavlink_msg_weight_calibration_get_right_led_brightness(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  9);
}

/**
 * @brief Get field left_led_brightness from weight_calibration message
 *
 * @return  Left front LED brightness (0-100: 0% to 100%)
 */
static inline uint8_t mavlink_msg_weight_calibration_get_left_led_brightness(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  10);
}

/**
 * @brief Get field tare_calibration from weight_calibration message
 *
 * @return  Tare calibration flag (0: no calibration, 1: start calibration)
 */
static inline uint8_t mavlink_msg_weight_calibration_get_tare_calibration(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  11);
}

/**
 * @brief Get field weight_calibration from weight_calibration message
 *
 * @return  Weight calibration flag (0: no calibration, 1: start calibration)
 */
static inline uint8_t mavlink_msg_weight_calibration_get_weight_calibration(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  12);
}

/**
 * @brief Get field calibration_weight from weight_calibration message
 *
 * @return  Calibration weight value in grams
 */
static inline uint16_t mavlink_msg_weight_calibration_get_calibration_weight(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  0);
}

/**
 * @brief Get field k_calibration from weight_calibration message
 *
 * @return  K-value calibration flag (0: no calibration, 1: start calibration)
 */
static inline uint8_t mavlink_msg_weight_calibration_get_k_calibration(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  13);
}

/**
 * @brief Get field k_values from weight_calibration message
 *
 * @return  K values for three weight sensors
 */
static inline uint16_t mavlink_msg_weight_calibration_get_k_values(const mavlink_message_t* msg, uint16_t *k_values)
{
    return _MAV_RETURN_uint16_t_array(msg, k_values, 3,  2);
}

/**
 * @brief Decode a weight_calibration message into a struct
 *
 * @param msg The message to decode
 * @param weight_calibration C-struct to decode the message contents into
 */
static inline void mavlink_msg_weight_calibration_decode(const mavlink_message_t* msg, mavlink_weight_calibration_t* weight_calibration)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    weight_calibration->calibration_weight = mavlink_msg_weight_calibration_get_calibration_weight(msg);
    mavlink_msg_weight_calibration_get_k_values(msg, weight_calibration->k_values);
    weight_calibration->led_control = mavlink_msg_weight_calibration_get_led_control(msg);
    weight_calibration->right_led_brightness = mavlink_msg_weight_calibration_get_right_led_brightness(msg);
    weight_calibration->left_led_brightness = mavlink_msg_weight_calibration_get_left_led_brightness(msg);
    weight_calibration->tare_calibration = mavlink_msg_weight_calibration_get_tare_calibration(msg);
    weight_calibration->weight_calibration = mavlink_msg_weight_calibration_get_weight_calibration(msg);
    weight_calibration->k_calibration = mavlink_msg_weight_calibration_get_k_calibration(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_WEIGHT_CALIBRATION_LEN? msg->len : MAVLINK_MSG_ID_WEIGHT_CALIBRATION_LEN;
        memset(weight_calibration, 0, MAVLINK_MSG_ID_WEIGHT_CALIBRATION_LEN);
    memcpy(weight_calibration, _MAV_PAYLOAD(msg), len);
#endif
}
