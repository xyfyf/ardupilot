/** @file
 *  @brief MAVLink comm protocol generated from eft.xml
 *  @see http://mavlink.org
 */
#pragma once
#ifndef MAVLINK_EFT_H
#define MAVLINK_EFT_H

#ifndef MAVLINK_H
    #error Wrong include order: MAVLINK_EFT.H MUST NOT BE DIRECTLY USED. Include mavlink.h from the same directory instead or set ALL AND EVERY defines from MAVLINK.H manually accordingly, including the #define MAVLINK_H call.
#endif

#define MAVLINK_EFT_XML_HASH 6301540273240915295

#ifdef __cplusplus
extern "C" {
#endif

// MESSAGE LENGTHS AND CRCS

#ifndef MAVLINK_MESSAGE_LENGTHS
#define MAVLINK_MESSAGE_LENGTHS {}
#endif

#ifndef MAVLINK_MESSAGE_CRCS
#define MAVLINK_MESSAGE_CRCS {{501, 137, 48, 48, 0, 0, 0}, {502, 152, 42, 42, 0, 0, 0}, {503, 161, 42, 42, 0, 0, 0}, {504, 79, 6, 6, 0, 0, 0}, {505, 203, 14, 14, 0, 0, 0}, {506, 132, 26, 26, 0, 0, 0}, {507, 83, 1, 1, 0, 0, 0}, {508, 65, 36, 36, 0, 0, 0}, {509, 197, 4, 4, 0, 0, 0}, {510, 29, 12, 12, 0, 0, 0}, {511, 113, 12, 12, 0, 0, 0}, {512, 167, 15, 15, 0, 0, 0}, {513, 240, 36, 36, 0, 0, 0}, {514, 62, 6, 6, 0, 0, 0}, {515, 66, 41, 41, 0, 0, 0}, {516, 253, 4, 4, 0, 0, 0}, {517, 209, 4, 4, 3, 0, 1}, {518, 197, 127, 127, 3, 20, 21}}
#endif

#include "../protocol.h"

#define MAVLINK_ENABLED_EFT

// ENUM DEFINITIONS



// MAVLINK VERSION

#ifndef MAVLINK_VERSION
#define MAVLINK_VERSION 2
#endif

#if (MAVLINK_VERSION == 0)
#undef MAVLINK_VERSION
#define MAVLINK_VERSION 2
#endif

// MESSAGE DEFINITIONS
#include "./mavlink_msg_device_status_array.h"
#include "./mavlink_msg_device_info1_array.h"
#include "./mavlink_msg_device_info2_array.h"
#include "./mavlink_msg_single_radar_data.h"
#include "./mavlink_msg_weight_calibration.h"
#include "./mavlink_msg_weigh_data_eft.h"
#include "./mavlink_msg_pump_calibration_cmd.h"
#include "./mavlink_msg_pump_calibration_results.h"
#include "./mavlink_msg_spray_system_params.h"
#include "./mavlink_msg_battery_data.h"
#include "./mavlink_msg_spreader_control.h"
#include "./mavlink_msg_spreader_status.h"
#include "./mavlink_msg_spreader_calibration_results.h"
#include "./mavlink_msg_multi_radar_data.h"
#include "./mavlink_msg_fmu_pmu_uart_message.h"
#include "./mavlink_msg_mav_framing_override_cmd.h"
#include "./mavlink_msg_eft_rid_config_request.h"
#include "./mavlink_msg_eft_rid_config_status.h"

// base include



#if MAVLINK_EFT_XML_HASH == MAVLINK_PRIMARY_XML_HASH
# define MAVLINK_MESSAGE_INFO {MAVLINK_MESSAGE_INFO_DEVICE_STATUS_ARRAY, MAVLINK_MESSAGE_INFO_DEVICE_INFO1_ARRAY, MAVLINK_MESSAGE_INFO_DEVICE_INFO2_ARRAY, MAVLINK_MESSAGE_INFO_SINGLE_RADAR_DATA, MAVLINK_MESSAGE_INFO_WEIGHT_CALIBRATION, MAVLINK_MESSAGE_INFO_WEIGH_DATA_EFT, MAVLINK_MESSAGE_INFO_PUMP_CALIBRATION_CMD, MAVLINK_MESSAGE_INFO_PUMP_CALIBRATION_RESULTS, MAVLINK_MESSAGE_INFO_SPRAY_SYSTEM_PARAMS, MAVLINK_MESSAGE_INFO_BATTERY_DATA, MAVLINK_MESSAGE_INFO_SPREADER_CONTROL, MAVLINK_MESSAGE_INFO_SPREADER_STATUS, MAVLINK_MESSAGE_INFO_SPREADER_CALIBRATION_RESULTS, MAVLINK_MESSAGE_INFO_MULTI_RADAR_DATA, MAVLINK_MESSAGE_INFO_FMU_PMU_UART_MESSAGE, MAVLINK_MESSAGE_INFO_MAV_FRAMING_OVERRIDE_CMD, MAVLINK_MESSAGE_INFO_EFT_RID_CONFIG_REQUEST, MAVLINK_MESSAGE_INFO_EFT_RID_CONFIG_STATUS}
# define MAVLINK_MESSAGE_NAMES {{ "BATTERY_DATA", 510 }, { "DEVICE_INFO1_ARRAY", 502 }, { "DEVICE_INFO2_ARRAY", 503 }, { "DEVICE_STATUS_ARRAY", 501 }, { "EFT_RID_CONFIG_REQUEST", 517 }, { "EFT_RID_CONFIG_STATUS", 518 }, { "FMU_PMU_UART_MESSAGE", 515 }, { "MAV_FRAMING_OVERRIDE_CMD", 516 }, { "MULTI_RADAR_DATA", 514 }, { "PUMP_CALIBRATION_CMD", 507 }, { "PUMP_CALIBRATION_RESULTS", 508 }, { "SINGLE_RADAR_DATA", 504 }, { "SPRAY_SYSTEM_PARAMS", 509 }, { "SPREADER_CALIBRATION_RESULTS", 513 }, { "SPREADER_CONTROL", 511 }, { "SPREADER_STATUS", 512 }, { "WEIGHT_CALIBRATION", 505 }, { "WEIGH_DATA_EFT", 506 }}
# if MAVLINK_COMMAND_24BIT
#  include "../mavlink_get_info.h"
# endif
#endif

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // MAVLINK_EFT_H
