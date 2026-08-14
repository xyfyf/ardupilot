#pragma once
// MESSAGE EFT_RID_CONFIG_STATUS PACKING

#define MAVLINK_MSG_ID_EFT_RID_CONFIG_STATUS 518


typedef struct __mavlink_eft_rid_config_status_t {
 int32_t operator_latitude; /*< [degE7] Operator latitude.*/
 int32_t operator_longitude; /*< [degE7] Operator longitude.*/
 float operator_altitude_geo; /*< [m] Operator geodetic altitude.*/
 uint32_t status_flags; /*<  Remote ID status bitmask.*/
 uint16_t arm_status_age_ms; /*< [ms] Milliseconds since last ARM_STATUS (65535 if never received).*/
 uint16_t system_age_ms; /*< [ms] Milliseconds since last SYSTEM/SYSTEM_UPDATE (65535 if never received).*/
 uint8_t target_system; /*<  Reply target system ID.*/
 uint8_t target_component; /*<  Reply target component ID.*/
 uint8_t seq; /*<  Echo of request sequence number.*/
 uint8_t did_enable; /*<  DID_ENABLE (0=disabled, 1=enabled).*/
 int8_t did_mavport; /*<  DID_MAVPORT serial port index (-1=disabled).*/
 uint8_t did_options; /*<  DID_OPTIONS bitmask.*/
 uint8_t did_can_driver; /*<  DID_CANDRIVER (0=disabled).*/
 uint8_t ua_type; /*<  UA type.*/
 uint8_t id_type; /*<  UAS ID type.*/
 char uas_id[20]; /*<  UAS ID string.*/
 uint8_t op_id_type; /*<  Operator ID type.*/
 char operator_id[20]; /*<  Operator ID string.*/
 uint8_t desc_type; /*<  Self ID description type.*/
 char self_desc[23]; /*<  Self ID description text.*/
 uint8_t arm_status; /*<  ARM_STATUS from RID transmitter.*/
 char arm_error[32]; /*<  Arm failure reason (truncated).*/
} mavlink_eft_rid_config_status_t;

#define MAVLINK_MSG_ID_EFT_RID_CONFIG_STATUS_LEN 127
#define MAVLINK_MSG_ID_EFT_RID_CONFIG_STATUS_MIN_LEN 127
#define MAVLINK_MSG_ID_518_LEN 127
#define MAVLINK_MSG_ID_518_MIN_LEN 127

#define MAVLINK_MSG_ID_EFT_RID_CONFIG_STATUS_CRC 197
#define MAVLINK_MSG_ID_518_CRC 197

#define MAVLINK_MSG_EFT_RID_CONFIG_STATUS_FIELD_UAS_ID_LEN 20
#define MAVLINK_MSG_EFT_RID_CONFIG_STATUS_FIELD_OPERATOR_ID_LEN 20
#define MAVLINK_MSG_EFT_RID_CONFIG_STATUS_FIELD_SELF_DESC_LEN 23
#define MAVLINK_MSG_EFT_RID_CONFIG_STATUS_FIELD_ARM_ERROR_LEN 32

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_EFT_RID_CONFIG_STATUS { \
    518, \
    "EFT_RID_CONFIG_STATUS", \
    22, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 20, offsetof(mavlink_eft_rid_config_status_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 21, offsetof(mavlink_eft_rid_config_status_t, target_component) }, \
         { "seq", NULL, MAVLINK_TYPE_UINT8_T, 0, 22, offsetof(mavlink_eft_rid_config_status_t, seq) }, \
         { "did_enable", NULL, MAVLINK_TYPE_UINT8_T, 0, 23, offsetof(mavlink_eft_rid_config_status_t, did_enable) }, \
         { "did_mavport", NULL, MAVLINK_TYPE_INT8_T, 0, 24, offsetof(mavlink_eft_rid_config_status_t, did_mavport) }, \
         { "did_options", NULL, MAVLINK_TYPE_UINT8_T, 0, 25, offsetof(mavlink_eft_rid_config_status_t, did_options) }, \
         { "did_can_driver", NULL, MAVLINK_TYPE_UINT8_T, 0, 26, offsetof(mavlink_eft_rid_config_status_t, did_can_driver) }, \
         { "ua_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 27, offsetof(mavlink_eft_rid_config_status_t, ua_type) }, \
         { "id_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 28, offsetof(mavlink_eft_rid_config_status_t, id_type) }, \
         { "uas_id", NULL, MAVLINK_TYPE_CHAR, 20, 29, offsetof(mavlink_eft_rid_config_status_t, uas_id) }, \
         { "op_id_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 49, offsetof(mavlink_eft_rid_config_status_t, op_id_type) }, \
         { "operator_id", NULL, MAVLINK_TYPE_CHAR, 20, 50, offsetof(mavlink_eft_rid_config_status_t, operator_id) }, \
         { "desc_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 70, offsetof(mavlink_eft_rid_config_status_t, desc_type) }, \
         { "self_desc", NULL, MAVLINK_TYPE_CHAR, 23, 71, offsetof(mavlink_eft_rid_config_status_t, self_desc) }, \
         { "operator_latitude", NULL, MAVLINK_TYPE_INT32_T, 0, 0, offsetof(mavlink_eft_rid_config_status_t, operator_latitude) }, \
         { "operator_longitude", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_eft_rid_config_status_t, operator_longitude) }, \
         { "operator_altitude_geo", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_eft_rid_config_status_t, operator_altitude_geo) }, \
         { "arm_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 94, offsetof(mavlink_eft_rid_config_status_t, arm_status) }, \
         { "arm_error", NULL, MAVLINK_TYPE_CHAR, 32, 95, offsetof(mavlink_eft_rid_config_status_t, arm_error) }, \
         { "arm_status_age_ms", NULL, MAVLINK_TYPE_UINT16_T, 0, 16, offsetof(mavlink_eft_rid_config_status_t, arm_status_age_ms) }, \
         { "system_age_ms", NULL, MAVLINK_TYPE_UINT16_T, 0, 18, offsetof(mavlink_eft_rid_config_status_t, system_age_ms) }, \
         { "status_flags", NULL, MAVLINK_TYPE_UINT32_T, 0, 12, offsetof(mavlink_eft_rid_config_status_t, status_flags) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_EFT_RID_CONFIG_STATUS { \
    "EFT_RID_CONFIG_STATUS", \
    22, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 20, offsetof(mavlink_eft_rid_config_status_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 21, offsetof(mavlink_eft_rid_config_status_t, target_component) }, \
         { "seq", NULL, MAVLINK_TYPE_UINT8_T, 0, 22, offsetof(mavlink_eft_rid_config_status_t, seq) }, \
         { "did_enable", NULL, MAVLINK_TYPE_UINT8_T, 0, 23, offsetof(mavlink_eft_rid_config_status_t, did_enable) }, \
         { "did_mavport", NULL, MAVLINK_TYPE_INT8_T, 0, 24, offsetof(mavlink_eft_rid_config_status_t, did_mavport) }, \
         { "did_options", NULL, MAVLINK_TYPE_UINT8_T, 0, 25, offsetof(mavlink_eft_rid_config_status_t, did_options) }, \
         { "did_can_driver", NULL, MAVLINK_TYPE_UINT8_T, 0, 26, offsetof(mavlink_eft_rid_config_status_t, did_can_driver) }, \
         { "ua_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 27, offsetof(mavlink_eft_rid_config_status_t, ua_type) }, \
         { "id_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 28, offsetof(mavlink_eft_rid_config_status_t, id_type) }, \
         { "uas_id", NULL, MAVLINK_TYPE_CHAR, 20, 29, offsetof(mavlink_eft_rid_config_status_t, uas_id) }, \
         { "op_id_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 49, offsetof(mavlink_eft_rid_config_status_t, op_id_type) }, \
         { "operator_id", NULL, MAVLINK_TYPE_CHAR, 20, 50, offsetof(mavlink_eft_rid_config_status_t, operator_id) }, \
         { "desc_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 70, offsetof(mavlink_eft_rid_config_status_t, desc_type) }, \
         { "self_desc", NULL, MAVLINK_TYPE_CHAR, 23, 71, offsetof(mavlink_eft_rid_config_status_t, self_desc) }, \
         { "operator_latitude", NULL, MAVLINK_TYPE_INT32_T, 0, 0, offsetof(mavlink_eft_rid_config_status_t, operator_latitude) }, \
         { "operator_longitude", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_eft_rid_config_status_t, operator_longitude) }, \
         { "operator_altitude_geo", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_eft_rid_config_status_t, operator_altitude_geo) }, \
         { "arm_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 94, offsetof(mavlink_eft_rid_config_status_t, arm_status) }, \
         { "arm_error", NULL, MAVLINK_TYPE_CHAR, 32, 95, offsetof(mavlink_eft_rid_config_status_t, arm_error) }, \
         { "arm_status_age_ms", NULL, MAVLINK_TYPE_UINT16_T, 0, 16, offsetof(mavlink_eft_rid_config_status_t, arm_status_age_ms) }, \
         { "system_age_ms", NULL, MAVLINK_TYPE_UINT16_T, 0, 18, offsetof(mavlink_eft_rid_config_status_t, system_age_ms) }, \
         { "status_flags", NULL, MAVLINK_TYPE_UINT32_T, 0, 12, offsetof(mavlink_eft_rid_config_status_t, status_flags) }, \
         } \
}
#endif

/**
 * @brief Pack a eft_rid_config_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system  Reply target system ID.
 * @param target_component  Reply target component ID.
 * @param seq  Echo of request sequence number.
 * @param did_enable  DID_ENABLE (0=disabled, 1=enabled).
 * @param did_mavport  DID_MAVPORT serial port index (-1=disabled).
 * @param did_options  DID_OPTIONS bitmask.
 * @param did_can_driver  DID_CANDRIVER (0=disabled).
 * @param ua_type  UA type.
 * @param id_type  UAS ID type.
 * @param uas_id  UAS ID string.
 * @param op_id_type  Operator ID type.
 * @param operator_id  Operator ID string.
 * @param desc_type  Self ID description type.
 * @param self_desc  Self ID description text.
 * @param operator_latitude [degE7] Operator latitude.
 * @param operator_longitude [degE7] Operator longitude.
 * @param operator_altitude_geo [m] Operator geodetic altitude.
 * @param arm_status  ARM_STATUS from RID transmitter.
 * @param arm_error  Arm failure reason (truncated).
 * @param arm_status_age_ms [ms] Milliseconds since last ARM_STATUS (65535 if never received).
 * @param system_age_ms [ms] Milliseconds since last SYSTEM/SYSTEM_UPDATE (65535 if never received).
 * @param status_flags  Remote ID status bitmask.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_eft_rid_config_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t target_system, uint8_t target_component, uint8_t seq, uint8_t did_enable, int8_t did_mavport, uint8_t did_options, uint8_t did_can_driver, uint8_t ua_type, uint8_t id_type, const char *uas_id, uint8_t op_id_type, const char *operator_id, uint8_t desc_type, const char *self_desc, int32_t operator_latitude, int32_t operator_longitude, float operator_altitude_geo, uint8_t arm_status, const char *arm_error, uint16_t arm_status_age_ms, uint16_t system_age_ms, uint32_t status_flags)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_EFT_RID_CONFIG_STATUS_LEN];
    _mav_put_int32_t(buf, 0, operator_latitude);
    _mav_put_int32_t(buf, 4, operator_longitude);
    _mav_put_float(buf, 8, operator_altitude_geo);
    _mav_put_uint32_t(buf, 12, status_flags);
    _mav_put_uint16_t(buf, 16, arm_status_age_ms);
    _mav_put_uint16_t(buf, 18, system_age_ms);
    _mav_put_uint8_t(buf, 20, target_system);
    _mav_put_uint8_t(buf, 21, target_component);
    _mav_put_uint8_t(buf, 22, seq);
    _mav_put_uint8_t(buf, 23, did_enable);
    _mav_put_int8_t(buf, 24, did_mavport);
    _mav_put_uint8_t(buf, 25, did_options);
    _mav_put_uint8_t(buf, 26, did_can_driver);
    _mav_put_uint8_t(buf, 27, ua_type);
    _mav_put_uint8_t(buf, 28, id_type);
    _mav_put_uint8_t(buf, 49, op_id_type);
    _mav_put_uint8_t(buf, 70, desc_type);
    _mav_put_uint8_t(buf, 94, arm_status);
    _mav_put_char_array(buf, 29, uas_id, 20);
    _mav_put_char_array(buf, 50, operator_id, 20);
    _mav_put_char_array(buf, 71, self_desc, 23);
    _mav_put_char_array(buf, 95, arm_error, 32);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_EFT_RID_CONFIG_STATUS_LEN);
#else
    mavlink_eft_rid_config_status_t packet;
    packet.operator_latitude = operator_latitude;
    packet.operator_longitude = operator_longitude;
    packet.operator_altitude_geo = operator_altitude_geo;
    packet.status_flags = status_flags;
    packet.arm_status_age_ms = arm_status_age_ms;
    packet.system_age_ms = system_age_ms;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.seq = seq;
    packet.did_enable = did_enable;
    packet.did_mavport = did_mavport;
    packet.did_options = did_options;
    packet.did_can_driver = did_can_driver;
    packet.ua_type = ua_type;
    packet.id_type = id_type;
    packet.op_id_type = op_id_type;
    packet.desc_type = desc_type;
    packet.arm_status = arm_status;
    mav_array_assign_char(packet.uas_id, uas_id, 20);
    mav_array_assign_char(packet.operator_id, operator_id, 20);
    mav_array_assign_char(packet.self_desc, self_desc, 23);
    mav_array_assign_char(packet.arm_error, arm_error, 32);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_EFT_RID_CONFIG_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_EFT_RID_CONFIG_STATUS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_EFT_RID_CONFIG_STATUS_MIN_LEN, MAVLINK_MSG_ID_EFT_RID_CONFIG_STATUS_LEN, MAVLINK_MSG_ID_EFT_RID_CONFIG_STATUS_CRC);
}

/**
 * @brief Pack a eft_rid_config_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system  Reply target system ID.
 * @param target_component  Reply target component ID.
 * @param seq  Echo of request sequence number.
 * @param did_enable  DID_ENABLE (0=disabled, 1=enabled).
 * @param did_mavport  DID_MAVPORT serial port index (-1=disabled).
 * @param did_options  DID_OPTIONS bitmask.
 * @param did_can_driver  DID_CANDRIVER (0=disabled).
 * @param ua_type  UA type.
 * @param id_type  UAS ID type.
 * @param uas_id  UAS ID string.
 * @param op_id_type  Operator ID type.
 * @param operator_id  Operator ID string.
 * @param desc_type  Self ID description type.
 * @param self_desc  Self ID description text.
 * @param operator_latitude [degE7] Operator latitude.
 * @param operator_longitude [degE7] Operator longitude.
 * @param operator_altitude_geo [m] Operator geodetic altitude.
 * @param arm_status  ARM_STATUS from RID transmitter.
 * @param arm_error  Arm failure reason (truncated).
 * @param arm_status_age_ms [ms] Milliseconds since last ARM_STATUS (65535 if never received).
 * @param system_age_ms [ms] Milliseconds since last SYSTEM/SYSTEM_UPDATE (65535 if never received).
 * @param status_flags  Remote ID status bitmask.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_eft_rid_config_status_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint8_t target_system, uint8_t target_component, uint8_t seq, uint8_t did_enable, int8_t did_mavport, uint8_t did_options, uint8_t did_can_driver, uint8_t ua_type, uint8_t id_type, const char *uas_id, uint8_t op_id_type, const char *operator_id, uint8_t desc_type, const char *self_desc, int32_t operator_latitude, int32_t operator_longitude, float operator_altitude_geo, uint8_t arm_status, const char *arm_error, uint16_t arm_status_age_ms, uint16_t system_age_ms, uint32_t status_flags)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_EFT_RID_CONFIG_STATUS_LEN];
    _mav_put_int32_t(buf, 0, operator_latitude);
    _mav_put_int32_t(buf, 4, operator_longitude);
    _mav_put_float(buf, 8, operator_altitude_geo);
    _mav_put_uint32_t(buf, 12, status_flags);
    _mav_put_uint16_t(buf, 16, arm_status_age_ms);
    _mav_put_uint16_t(buf, 18, system_age_ms);
    _mav_put_uint8_t(buf, 20, target_system);
    _mav_put_uint8_t(buf, 21, target_component);
    _mav_put_uint8_t(buf, 22, seq);
    _mav_put_uint8_t(buf, 23, did_enable);
    _mav_put_int8_t(buf, 24, did_mavport);
    _mav_put_uint8_t(buf, 25, did_options);
    _mav_put_uint8_t(buf, 26, did_can_driver);
    _mav_put_uint8_t(buf, 27, ua_type);
    _mav_put_uint8_t(buf, 28, id_type);
    _mav_put_uint8_t(buf, 49, op_id_type);
    _mav_put_uint8_t(buf, 70, desc_type);
    _mav_put_uint8_t(buf, 94, arm_status);
    _mav_put_char_array(buf, 29, uas_id, 20);
    _mav_put_char_array(buf, 50, operator_id, 20);
    _mav_put_char_array(buf, 71, self_desc, 23);
    _mav_put_char_array(buf, 95, arm_error, 32);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_EFT_RID_CONFIG_STATUS_LEN);
#else
    mavlink_eft_rid_config_status_t packet;
    packet.operator_latitude = operator_latitude;
    packet.operator_longitude = operator_longitude;
    packet.operator_altitude_geo = operator_altitude_geo;
    packet.status_flags = status_flags;
    packet.arm_status_age_ms = arm_status_age_ms;
    packet.system_age_ms = system_age_ms;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.seq = seq;
    packet.did_enable = did_enable;
    packet.did_mavport = did_mavport;
    packet.did_options = did_options;
    packet.did_can_driver = did_can_driver;
    packet.ua_type = ua_type;
    packet.id_type = id_type;
    packet.op_id_type = op_id_type;
    packet.desc_type = desc_type;
    packet.arm_status = arm_status;
    mav_array_memcpy(packet.uas_id, uas_id, sizeof(char)*20);
    mav_array_memcpy(packet.operator_id, operator_id, sizeof(char)*20);
    mav_array_memcpy(packet.self_desc, self_desc, sizeof(char)*23);
    mav_array_memcpy(packet.arm_error, arm_error, sizeof(char)*32);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_EFT_RID_CONFIG_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_EFT_RID_CONFIG_STATUS;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_EFT_RID_CONFIG_STATUS_MIN_LEN, MAVLINK_MSG_ID_EFT_RID_CONFIG_STATUS_LEN, MAVLINK_MSG_ID_EFT_RID_CONFIG_STATUS_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_EFT_RID_CONFIG_STATUS_MIN_LEN, MAVLINK_MSG_ID_EFT_RID_CONFIG_STATUS_LEN);
#endif
}

/**
 * @brief Pack a eft_rid_config_status message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system  Reply target system ID.
 * @param target_component  Reply target component ID.
 * @param seq  Echo of request sequence number.
 * @param did_enable  DID_ENABLE (0=disabled, 1=enabled).
 * @param did_mavport  DID_MAVPORT serial port index (-1=disabled).
 * @param did_options  DID_OPTIONS bitmask.
 * @param did_can_driver  DID_CANDRIVER (0=disabled).
 * @param ua_type  UA type.
 * @param id_type  UAS ID type.
 * @param uas_id  UAS ID string.
 * @param op_id_type  Operator ID type.
 * @param operator_id  Operator ID string.
 * @param desc_type  Self ID description type.
 * @param self_desc  Self ID description text.
 * @param operator_latitude [degE7] Operator latitude.
 * @param operator_longitude [degE7] Operator longitude.
 * @param operator_altitude_geo [m] Operator geodetic altitude.
 * @param arm_status  ARM_STATUS from RID transmitter.
 * @param arm_error  Arm failure reason (truncated).
 * @param arm_status_age_ms [ms] Milliseconds since last ARM_STATUS (65535 if never received).
 * @param system_age_ms [ms] Milliseconds since last SYSTEM/SYSTEM_UPDATE (65535 if never received).
 * @param status_flags  Remote ID status bitmask.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_eft_rid_config_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t target_system,uint8_t target_component,uint8_t seq,uint8_t did_enable,int8_t did_mavport,uint8_t did_options,uint8_t did_can_driver,uint8_t ua_type,uint8_t id_type,const char *uas_id,uint8_t op_id_type,const char *operator_id,uint8_t desc_type,const char *self_desc,int32_t operator_latitude,int32_t operator_longitude,float operator_altitude_geo,uint8_t arm_status,const char *arm_error,uint16_t arm_status_age_ms,uint16_t system_age_ms,uint32_t status_flags)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_EFT_RID_CONFIG_STATUS_LEN];
    _mav_put_int32_t(buf, 0, operator_latitude);
    _mav_put_int32_t(buf, 4, operator_longitude);
    _mav_put_float(buf, 8, operator_altitude_geo);
    _mav_put_uint32_t(buf, 12, status_flags);
    _mav_put_uint16_t(buf, 16, arm_status_age_ms);
    _mav_put_uint16_t(buf, 18, system_age_ms);
    _mav_put_uint8_t(buf, 20, target_system);
    _mav_put_uint8_t(buf, 21, target_component);
    _mav_put_uint8_t(buf, 22, seq);
    _mav_put_uint8_t(buf, 23, did_enable);
    _mav_put_int8_t(buf, 24, did_mavport);
    _mav_put_uint8_t(buf, 25, did_options);
    _mav_put_uint8_t(buf, 26, did_can_driver);
    _mav_put_uint8_t(buf, 27, ua_type);
    _mav_put_uint8_t(buf, 28, id_type);
    _mav_put_uint8_t(buf, 49, op_id_type);
    _mav_put_uint8_t(buf, 70, desc_type);
    _mav_put_uint8_t(buf, 94, arm_status);
    _mav_put_char_array(buf, 29, uas_id, 20);
    _mav_put_char_array(buf, 50, operator_id, 20);
    _mav_put_char_array(buf, 71, self_desc, 23);
    _mav_put_char_array(buf, 95, arm_error, 32);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_EFT_RID_CONFIG_STATUS_LEN);
#else
    mavlink_eft_rid_config_status_t packet;
    packet.operator_latitude = operator_latitude;
    packet.operator_longitude = operator_longitude;
    packet.operator_altitude_geo = operator_altitude_geo;
    packet.status_flags = status_flags;
    packet.arm_status_age_ms = arm_status_age_ms;
    packet.system_age_ms = system_age_ms;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.seq = seq;
    packet.did_enable = did_enable;
    packet.did_mavport = did_mavport;
    packet.did_options = did_options;
    packet.did_can_driver = did_can_driver;
    packet.ua_type = ua_type;
    packet.id_type = id_type;
    packet.op_id_type = op_id_type;
    packet.desc_type = desc_type;
    packet.arm_status = arm_status;
    mav_array_assign_char(packet.uas_id, uas_id, 20);
    mav_array_assign_char(packet.operator_id, operator_id, 20);
    mav_array_assign_char(packet.self_desc, self_desc, 23);
    mav_array_assign_char(packet.arm_error, arm_error, 32);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_EFT_RID_CONFIG_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_EFT_RID_CONFIG_STATUS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_EFT_RID_CONFIG_STATUS_MIN_LEN, MAVLINK_MSG_ID_EFT_RID_CONFIG_STATUS_LEN, MAVLINK_MSG_ID_EFT_RID_CONFIG_STATUS_CRC);
}

/**
 * @brief Encode a eft_rid_config_status struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param eft_rid_config_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_eft_rid_config_status_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_eft_rid_config_status_t* eft_rid_config_status)
{
    return mavlink_msg_eft_rid_config_status_pack(system_id, component_id, msg, eft_rid_config_status->target_system, eft_rid_config_status->target_component, eft_rid_config_status->seq, eft_rid_config_status->did_enable, eft_rid_config_status->did_mavport, eft_rid_config_status->did_options, eft_rid_config_status->did_can_driver, eft_rid_config_status->ua_type, eft_rid_config_status->id_type, eft_rid_config_status->uas_id, eft_rid_config_status->op_id_type, eft_rid_config_status->operator_id, eft_rid_config_status->desc_type, eft_rid_config_status->self_desc, eft_rid_config_status->operator_latitude, eft_rid_config_status->operator_longitude, eft_rid_config_status->operator_altitude_geo, eft_rid_config_status->arm_status, eft_rid_config_status->arm_error, eft_rid_config_status->arm_status_age_ms, eft_rid_config_status->system_age_ms, eft_rid_config_status->status_flags);
}

/**
 * @brief Encode a eft_rid_config_status struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param eft_rid_config_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_eft_rid_config_status_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_eft_rid_config_status_t* eft_rid_config_status)
{
    return mavlink_msg_eft_rid_config_status_pack_chan(system_id, component_id, chan, msg, eft_rid_config_status->target_system, eft_rid_config_status->target_component, eft_rid_config_status->seq, eft_rid_config_status->did_enable, eft_rid_config_status->did_mavport, eft_rid_config_status->did_options, eft_rid_config_status->did_can_driver, eft_rid_config_status->ua_type, eft_rid_config_status->id_type, eft_rid_config_status->uas_id, eft_rid_config_status->op_id_type, eft_rid_config_status->operator_id, eft_rid_config_status->desc_type, eft_rid_config_status->self_desc, eft_rid_config_status->operator_latitude, eft_rid_config_status->operator_longitude, eft_rid_config_status->operator_altitude_geo, eft_rid_config_status->arm_status, eft_rid_config_status->arm_error, eft_rid_config_status->arm_status_age_ms, eft_rid_config_status->system_age_ms, eft_rid_config_status->status_flags);
}

/**
 * @brief Encode a eft_rid_config_status struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param eft_rid_config_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_eft_rid_config_status_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_eft_rid_config_status_t* eft_rid_config_status)
{
    return mavlink_msg_eft_rid_config_status_pack_status(system_id, component_id, _status, msg,  eft_rid_config_status->target_system, eft_rid_config_status->target_component, eft_rid_config_status->seq, eft_rid_config_status->did_enable, eft_rid_config_status->did_mavport, eft_rid_config_status->did_options, eft_rid_config_status->did_can_driver, eft_rid_config_status->ua_type, eft_rid_config_status->id_type, eft_rid_config_status->uas_id, eft_rid_config_status->op_id_type, eft_rid_config_status->operator_id, eft_rid_config_status->desc_type, eft_rid_config_status->self_desc, eft_rid_config_status->operator_latitude, eft_rid_config_status->operator_longitude, eft_rid_config_status->operator_altitude_geo, eft_rid_config_status->arm_status, eft_rid_config_status->arm_error, eft_rid_config_status->arm_status_age_ms, eft_rid_config_status->system_age_ms, eft_rid_config_status->status_flags);
}

/**
 * @brief Send a eft_rid_config_status message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system  Reply target system ID.
 * @param target_component  Reply target component ID.
 * @param seq  Echo of request sequence number.
 * @param did_enable  DID_ENABLE (0=disabled, 1=enabled).
 * @param did_mavport  DID_MAVPORT serial port index (-1=disabled).
 * @param did_options  DID_OPTIONS bitmask.
 * @param did_can_driver  DID_CANDRIVER (0=disabled).
 * @param ua_type  UA type.
 * @param id_type  UAS ID type.
 * @param uas_id  UAS ID string.
 * @param op_id_type  Operator ID type.
 * @param operator_id  Operator ID string.
 * @param desc_type  Self ID description type.
 * @param self_desc  Self ID description text.
 * @param operator_latitude [degE7] Operator latitude.
 * @param operator_longitude [degE7] Operator longitude.
 * @param operator_altitude_geo [m] Operator geodetic altitude.
 * @param arm_status  ARM_STATUS from RID transmitter.
 * @param arm_error  Arm failure reason (truncated).
 * @param arm_status_age_ms [ms] Milliseconds since last ARM_STATUS (65535 if never received).
 * @param system_age_ms [ms] Milliseconds since last SYSTEM/SYSTEM_UPDATE (65535 if never received).
 * @param status_flags  Remote ID status bitmask.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_eft_rid_config_status_send(mavlink_channel_t chan, uint8_t target_system, uint8_t target_component, uint8_t seq, uint8_t did_enable, int8_t did_mavport, uint8_t did_options, uint8_t did_can_driver, uint8_t ua_type, uint8_t id_type, const char *uas_id, uint8_t op_id_type, const char *operator_id, uint8_t desc_type, const char *self_desc, int32_t operator_latitude, int32_t operator_longitude, float operator_altitude_geo, uint8_t arm_status, const char *arm_error, uint16_t arm_status_age_ms, uint16_t system_age_ms, uint32_t status_flags)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_EFT_RID_CONFIG_STATUS_LEN];
    _mav_put_int32_t(buf, 0, operator_latitude);
    _mav_put_int32_t(buf, 4, operator_longitude);
    _mav_put_float(buf, 8, operator_altitude_geo);
    _mav_put_uint32_t(buf, 12, status_flags);
    _mav_put_uint16_t(buf, 16, arm_status_age_ms);
    _mav_put_uint16_t(buf, 18, system_age_ms);
    _mav_put_uint8_t(buf, 20, target_system);
    _mav_put_uint8_t(buf, 21, target_component);
    _mav_put_uint8_t(buf, 22, seq);
    _mav_put_uint8_t(buf, 23, did_enable);
    _mav_put_int8_t(buf, 24, did_mavport);
    _mav_put_uint8_t(buf, 25, did_options);
    _mav_put_uint8_t(buf, 26, did_can_driver);
    _mav_put_uint8_t(buf, 27, ua_type);
    _mav_put_uint8_t(buf, 28, id_type);
    _mav_put_uint8_t(buf, 49, op_id_type);
    _mav_put_uint8_t(buf, 70, desc_type);
    _mav_put_uint8_t(buf, 94, arm_status);
    _mav_put_char_array(buf, 29, uas_id, 20);
    _mav_put_char_array(buf, 50, operator_id, 20);
    _mav_put_char_array(buf, 71, self_desc, 23);
    _mav_put_char_array(buf, 95, arm_error, 32);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_EFT_RID_CONFIG_STATUS, buf, MAVLINK_MSG_ID_EFT_RID_CONFIG_STATUS_MIN_LEN, MAVLINK_MSG_ID_EFT_RID_CONFIG_STATUS_LEN, MAVLINK_MSG_ID_EFT_RID_CONFIG_STATUS_CRC);
#else
    mavlink_eft_rid_config_status_t packet;
    packet.operator_latitude = operator_latitude;
    packet.operator_longitude = operator_longitude;
    packet.operator_altitude_geo = operator_altitude_geo;
    packet.status_flags = status_flags;
    packet.arm_status_age_ms = arm_status_age_ms;
    packet.system_age_ms = system_age_ms;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.seq = seq;
    packet.did_enable = did_enable;
    packet.did_mavport = did_mavport;
    packet.did_options = did_options;
    packet.did_can_driver = did_can_driver;
    packet.ua_type = ua_type;
    packet.id_type = id_type;
    packet.op_id_type = op_id_type;
    packet.desc_type = desc_type;
    packet.arm_status = arm_status;
    mav_array_assign_char(packet.uas_id, uas_id, 20);
    mav_array_assign_char(packet.operator_id, operator_id, 20);
    mav_array_assign_char(packet.self_desc, self_desc, 23);
    mav_array_assign_char(packet.arm_error, arm_error, 32);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_EFT_RID_CONFIG_STATUS, (const char *)&packet, MAVLINK_MSG_ID_EFT_RID_CONFIG_STATUS_MIN_LEN, MAVLINK_MSG_ID_EFT_RID_CONFIG_STATUS_LEN, MAVLINK_MSG_ID_EFT_RID_CONFIG_STATUS_CRC);
#endif
}

/**
 * @brief Send a eft_rid_config_status message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_eft_rid_config_status_send_struct(mavlink_channel_t chan, const mavlink_eft_rid_config_status_t* eft_rid_config_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_eft_rid_config_status_send(chan, eft_rid_config_status->target_system, eft_rid_config_status->target_component, eft_rid_config_status->seq, eft_rid_config_status->did_enable, eft_rid_config_status->did_mavport, eft_rid_config_status->did_options, eft_rid_config_status->did_can_driver, eft_rid_config_status->ua_type, eft_rid_config_status->id_type, eft_rid_config_status->uas_id, eft_rid_config_status->op_id_type, eft_rid_config_status->operator_id, eft_rid_config_status->desc_type, eft_rid_config_status->self_desc, eft_rid_config_status->operator_latitude, eft_rid_config_status->operator_longitude, eft_rid_config_status->operator_altitude_geo, eft_rid_config_status->arm_status, eft_rid_config_status->arm_error, eft_rid_config_status->arm_status_age_ms, eft_rid_config_status->system_age_ms, eft_rid_config_status->status_flags);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_EFT_RID_CONFIG_STATUS, (const char *)eft_rid_config_status, MAVLINK_MSG_ID_EFT_RID_CONFIG_STATUS_MIN_LEN, MAVLINK_MSG_ID_EFT_RID_CONFIG_STATUS_LEN, MAVLINK_MSG_ID_EFT_RID_CONFIG_STATUS_CRC);
#endif
}

#if MAVLINK_MSG_ID_EFT_RID_CONFIG_STATUS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by reusing
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_eft_rid_config_status_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t target_system, uint8_t target_component, uint8_t seq, uint8_t did_enable, int8_t did_mavport, uint8_t did_options, uint8_t did_can_driver, uint8_t ua_type, uint8_t id_type, const char *uas_id, uint8_t op_id_type, const char *operator_id, uint8_t desc_type, const char *self_desc, int32_t operator_latitude, int32_t operator_longitude, float operator_altitude_geo, uint8_t arm_status, const char *arm_error, uint16_t arm_status_age_ms, uint16_t system_age_ms, uint32_t status_flags)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_int32_t(buf, 0, operator_latitude);
    _mav_put_int32_t(buf, 4, operator_longitude);
    _mav_put_float(buf, 8, operator_altitude_geo);
    _mav_put_uint32_t(buf, 12, status_flags);
    _mav_put_uint16_t(buf, 16, arm_status_age_ms);
    _mav_put_uint16_t(buf, 18, system_age_ms);
    _mav_put_uint8_t(buf, 20, target_system);
    _mav_put_uint8_t(buf, 21, target_component);
    _mav_put_uint8_t(buf, 22, seq);
    _mav_put_uint8_t(buf, 23, did_enable);
    _mav_put_int8_t(buf, 24, did_mavport);
    _mav_put_uint8_t(buf, 25, did_options);
    _mav_put_uint8_t(buf, 26, did_can_driver);
    _mav_put_uint8_t(buf, 27, ua_type);
    _mav_put_uint8_t(buf, 28, id_type);
    _mav_put_uint8_t(buf, 49, op_id_type);
    _mav_put_uint8_t(buf, 70, desc_type);
    _mav_put_uint8_t(buf, 94, arm_status);
    _mav_put_char_array(buf, 29, uas_id, 20);
    _mav_put_char_array(buf, 50, operator_id, 20);
    _mav_put_char_array(buf, 71, self_desc, 23);
    _mav_put_char_array(buf, 95, arm_error, 32);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_EFT_RID_CONFIG_STATUS, buf, MAVLINK_MSG_ID_EFT_RID_CONFIG_STATUS_MIN_LEN, MAVLINK_MSG_ID_EFT_RID_CONFIG_STATUS_LEN, MAVLINK_MSG_ID_EFT_RID_CONFIG_STATUS_CRC);
#else
    mavlink_eft_rid_config_status_t *packet = (mavlink_eft_rid_config_status_t *)msgbuf;
    packet->operator_latitude = operator_latitude;
    packet->operator_longitude = operator_longitude;
    packet->operator_altitude_geo = operator_altitude_geo;
    packet->status_flags = status_flags;
    packet->arm_status_age_ms = arm_status_age_ms;
    packet->system_age_ms = system_age_ms;
    packet->target_system = target_system;
    packet->target_component = target_component;
    packet->seq = seq;
    packet->did_enable = did_enable;
    packet->did_mavport = did_mavport;
    packet->did_options = did_options;
    packet->did_can_driver = did_can_driver;
    packet->ua_type = ua_type;
    packet->id_type = id_type;
    packet->op_id_type = op_id_type;
    packet->desc_type = desc_type;
    packet->arm_status = arm_status;
    mav_array_assign_char(packet->uas_id, uas_id, 20);
    mav_array_assign_char(packet->operator_id, operator_id, 20);
    mav_array_assign_char(packet->self_desc, self_desc, 23);
    mav_array_assign_char(packet->arm_error, arm_error, 32);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_EFT_RID_CONFIG_STATUS, (const char *)packet, MAVLINK_MSG_ID_EFT_RID_CONFIG_STATUS_MIN_LEN, MAVLINK_MSG_ID_EFT_RID_CONFIG_STATUS_LEN, MAVLINK_MSG_ID_EFT_RID_CONFIG_STATUS_CRC);
#endif
}
#endif

#endif

// MESSAGE EFT_RID_CONFIG_STATUS UNPACKING


/**
 * @brief Get field target_system from eft_rid_config_status message
 *
 * @return  Reply target system ID.
 */
static inline uint8_t mavlink_msg_eft_rid_config_status_get_target_system(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  20);
}

/**
 * @brief Get field target_component from eft_rid_config_status message
 *
 * @return  Reply target component ID.
 */
static inline uint8_t mavlink_msg_eft_rid_config_status_get_target_component(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  21);
}

/**
 * @brief Get field seq from eft_rid_config_status message
 *
 * @return  Echo of request sequence number.
 */
static inline uint8_t mavlink_msg_eft_rid_config_status_get_seq(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  22);
}

/**
 * @brief Get field did_enable from eft_rid_config_status message
 *
 * @return  DID_ENABLE (0=disabled, 1=enabled).
 */
static inline uint8_t mavlink_msg_eft_rid_config_status_get_did_enable(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  23);
}

/**
 * @brief Get field did_mavport from eft_rid_config_status message
 *
 * @return  DID_MAVPORT serial port index (-1=disabled).
 */
static inline int8_t mavlink_msg_eft_rid_config_status_get_did_mavport(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int8_t(msg,  24);
}

/**
 * @brief Get field did_options from eft_rid_config_status message
 *
 * @return  DID_OPTIONS bitmask.
 */
static inline uint8_t mavlink_msg_eft_rid_config_status_get_did_options(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  25);
}

/**
 * @brief Get field did_can_driver from eft_rid_config_status message
 *
 * @return  DID_CANDRIVER (0=disabled).
 */
static inline uint8_t mavlink_msg_eft_rid_config_status_get_did_can_driver(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  26);
}

/**
 * @brief Get field ua_type from eft_rid_config_status message
 *
 * @return  UA type.
 */
static inline uint8_t mavlink_msg_eft_rid_config_status_get_ua_type(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  27);
}

/**
 * @brief Get field id_type from eft_rid_config_status message
 *
 * @return  UAS ID type.
 */
static inline uint8_t mavlink_msg_eft_rid_config_status_get_id_type(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  28);
}

/**
 * @brief Get field uas_id from eft_rid_config_status message
 *
 * @return  UAS ID string.
 */
static inline uint16_t mavlink_msg_eft_rid_config_status_get_uas_id(const mavlink_message_t* msg, char *uas_id)
{
    return _MAV_RETURN_char_array(msg, uas_id, 20,  29);
}

/**
 * @brief Get field op_id_type from eft_rid_config_status message
 *
 * @return  Operator ID type.
 */
static inline uint8_t mavlink_msg_eft_rid_config_status_get_op_id_type(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  49);
}

/**
 * @brief Get field operator_id from eft_rid_config_status message
 *
 * @return  Operator ID string.
 */
static inline uint16_t mavlink_msg_eft_rid_config_status_get_operator_id(const mavlink_message_t* msg, char *operator_id)
{
    return _MAV_RETURN_char_array(msg, operator_id, 20,  50);
}

/**
 * @brief Get field desc_type from eft_rid_config_status message
 *
 * @return  Self ID description type.
 */
static inline uint8_t mavlink_msg_eft_rid_config_status_get_desc_type(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  70);
}

/**
 * @brief Get field self_desc from eft_rid_config_status message
 *
 * @return  Self ID description text.
 */
static inline uint16_t mavlink_msg_eft_rid_config_status_get_self_desc(const mavlink_message_t* msg, char *self_desc)
{
    return _MAV_RETURN_char_array(msg, self_desc, 23,  71);
}

/**
 * @brief Get field operator_latitude from eft_rid_config_status message
 *
 * @return [degE7] Operator latitude.
 */
static inline int32_t mavlink_msg_eft_rid_config_status_get_operator_latitude(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  0);
}

/**
 * @brief Get field operator_longitude from eft_rid_config_status message
 *
 * @return [degE7] Operator longitude.
 */
static inline int32_t mavlink_msg_eft_rid_config_status_get_operator_longitude(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  4);
}

/**
 * @brief Get field operator_altitude_geo from eft_rid_config_status message
 *
 * @return [m] Operator geodetic altitude.
 */
static inline float mavlink_msg_eft_rid_config_status_get_operator_altitude_geo(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field arm_status from eft_rid_config_status message
 *
 * @return  ARM_STATUS from RID transmitter.
 */
static inline uint8_t mavlink_msg_eft_rid_config_status_get_arm_status(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  94);
}

/**
 * @brief Get field arm_error from eft_rid_config_status message
 *
 * @return  Arm failure reason (truncated).
 */
static inline uint16_t mavlink_msg_eft_rid_config_status_get_arm_error(const mavlink_message_t* msg, char *arm_error)
{
    return _MAV_RETURN_char_array(msg, arm_error, 32,  95);
}

/**
 * @brief Get field arm_status_age_ms from eft_rid_config_status message
 *
 * @return [ms] Milliseconds since last ARM_STATUS (65535 if never received).
 */
static inline uint16_t mavlink_msg_eft_rid_config_status_get_arm_status_age_ms(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  16);
}

/**
 * @brief Get field system_age_ms from eft_rid_config_status message
 *
 * @return [ms] Milliseconds since last SYSTEM/SYSTEM_UPDATE (65535 if never received).
 */
static inline uint16_t mavlink_msg_eft_rid_config_status_get_system_age_ms(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  18);
}

/**
 * @brief Get field status_flags from eft_rid_config_status message
 *
 * @return  Remote ID status bitmask.
 */
static inline uint32_t mavlink_msg_eft_rid_config_status_get_status_flags(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  12);
}

/**
 * @brief Decode a eft_rid_config_status message into a struct
 *
 * @param msg The message to decode
 * @param eft_rid_config_status C-struct to decode the message contents into
 */
static inline void mavlink_msg_eft_rid_config_status_decode(const mavlink_message_t* msg, mavlink_eft_rid_config_status_t* eft_rid_config_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    eft_rid_config_status->operator_latitude = mavlink_msg_eft_rid_config_status_get_operator_latitude(msg);
    eft_rid_config_status->operator_longitude = mavlink_msg_eft_rid_config_status_get_operator_longitude(msg);
    eft_rid_config_status->operator_altitude_geo = mavlink_msg_eft_rid_config_status_get_operator_altitude_geo(msg);
    eft_rid_config_status->status_flags = mavlink_msg_eft_rid_config_status_get_status_flags(msg);
    eft_rid_config_status->arm_status_age_ms = mavlink_msg_eft_rid_config_status_get_arm_status_age_ms(msg);
    eft_rid_config_status->system_age_ms = mavlink_msg_eft_rid_config_status_get_system_age_ms(msg);
    eft_rid_config_status->target_system = mavlink_msg_eft_rid_config_status_get_target_system(msg);
    eft_rid_config_status->target_component = mavlink_msg_eft_rid_config_status_get_target_component(msg);
    eft_rid_config_status->seq = mavlink_msg_eft_rid_config_status_get_seq(msg);
    eft_rid_config_status->did_enable = mavlink_msg_eft_rid_config_status_get_did_enable(msg);
    eft_rid_config_status->did_mavport = mavlink_msg_eft_rid_config_status_get_did_mavport(msg);
    eft_rid_config_status->did_options = mavlink_msg_eft_rid_config_status_get_did_options(msg);
    eft_rid_config_status->did_can_driver = mavlink_msg_eft_rid_config_status_get_did_can_driver(msg);
    eft_rid_config_status->ua_type = mavlink_msg_eft_rid_config_status_get_ua_type(msg);
    eft_rid_config_status->id_type = mavlink_msg_eft_rid_config_status_get_id_type(msg);
    mavlink_msg_eft_rid_config_status_get_uas_id(msg, eft_rid_config_status->uas_id);
    eft_rid_config_status->op_id_type = mavlink_msg_eft_rid_config_status_get_op_id_type(msg);
    mavlink_msg_eft_rid_config_status_get_operator_id(msg, eft_rid_config_status->operator_id);
    eft_rid_config_status->desc_type = mavlink_msg_eft_rid_config_status_get_desc_type(msg);
    mavlink_msg_eft_rid_config_status_get_self_desc(msg, eft_rid_config_status->self_desc);
    eft_rid_config_status->arm_status = mavlink_msg_eft_rid_config_status_get_arm_status(msg);
    mavlink_msg_eft_rid_config_status_get_arm_error(msg, eft_rid_config_status->arm_error);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_EFT_RID_CONFIG_STATUS_LEN? msg->len : MAVLINK_MSG_ID_EFT_RID_CONFIG_STATUS_LEN;
        memset(eft_rid_config_status, 0, MAVLINK_MSG_ID_EFT_RID_CONFIG_STATUS_LEN);
    memcpy(eft_rid_config_status, _MAV_PAYLOAD(msg), len);
#endif
}
