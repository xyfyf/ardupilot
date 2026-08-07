/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
 *
 * original code by:
 *  BlueMark Innovations BV, Roel Schiphorst
 *  Contributors: Tom Pittenger, Josh Henderson, Andrew Tridgell
 *  Parts of this code are based on/copied from the Open Drone ID project https://github.com/opendroneid/opendroneid-core-c
 *
 * The code has been tested with the BlueMark DroneBeacon MAVLink transponder running this command in the ArduPlane folder:
 * sim_vehicle.py --console --map -A --serial1=uart:/dev/ttyUSB1:9600
 * (and a DroneBeacon MAVLink transponder connected to ttyUSB1)
 *
 * See https://github.com/ArduPilot/ArduRemoteID for an open implementation of a transmitter module on serial
 * and DroneCAN
 */

#include "AP_OpenDroneID_config.h"

#if AP_OPENDRONEID_ENABLED

#include "AP_OpenDroneID.h"
#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Parachute/AP_Parachute.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include <AP_DroneCAN/AP_DroneCAN.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL &hal;

const AP_Param::GroupInfo AP_OpenDroneID::var_info[] = {

    // @Param: ENABLE
    // @DisplayName: Enable ODID subsystem
    // @Description: Enable ODID subsystem
    // @Values: 0:Disabled,1:Enabled
    AP_GROUPINFO_FLAGS("ENABLE", 1, AP_OpenDroneID, _enable, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: MAVPORT
    // @DisplayName: MAVLink serial port
    // @Description: Serial port number to send OpenDroneID MAVLink messages to. Can be -1 if using DroneCAN.
    // @Values: -1:Disabled,0:Serial0,1:Serial1,2:Serial2,3:Serial3,4:Serial4,5:Serial5,6:Serial6
    AP_GROUPINFO("MAVPORT", 2, AP_OpenDroneID, _mav_port, -1),

    // @Param: CANDRIVER
    // @DisplayName: DroneCAN driver number
    // @Description: DroneCAN driver index, 0 to disable DroneCAN
    // @Values: 0:Disabled,1:Driver1,2:Driver2
    AP_GROUPINFO("CANDRIVER", 3, AP_OpenDroneID, _can_driver, 0),
    
    // @Param: OPTIONS
    // @DisplayName: OpenDroneID options
    // @Description: Options for OpenDroneID subsystem
    // @Bitmask: 0:EnforceArming, 1:AllowNonGPSPosition, 2:LockUASIDOnFirstBasicIDRx
    AP_GROUPINFO("OPTIONS", 4, AP_OpenDroneID, _options, 0),

    // @Param: BARO_ACC
    // @DisplayName: Barometer vertical accuraacy
    // @Description: Barometer Vertical Accuracy when installed in the vehicle. Note this is dependent upon installation conditions and thus disabled by default
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("BARO_ACC", 5, AP_OpenDroneID, _baro_accuracy, -1.0),

    // @Param: OP_ID_0
    // @DisplayName: OperatorID bytes 0-3
    // @Description: Packed big-endian bytes 0-3 of the UOM operator_id string
    // @User: Advanced
    AP_GROUPINFO("OP_ID_0",  6, AP_OpenDroneID, _op_id_0, 0),

    // @Param: OP_ID_1
    // @DisplayName: OperatorID bytes 4-7
    AP_GROUPINFO("OP_ID_1",  7, AP_OpenDroneID, _op_id_1, 0),

    // @Param: OP_ID_2
    // @DisplayName: OperatorID bytes 8-11
    AP_GROUPINFO("OP_ID_2",  8, AP_OpenDroneID, _op_id_2, 0),

    // @Param: OP_ID_3
    // @DisplayName: OperatorID bytes 12-15
    AP_GROUPINFO("OP_ID_3",  9, AP_OpenDroneID, _op_id_3, 0),

    // @Param: OP_ID_4
    // @DisplayName: OperatorID bytes 16-19
    AP_GROUPINFO("OP_ID_4", 10, AP_OpenDroneID, _op_id_4, 0),

    // @Param: OP_TYPE
    // @DisplayName: OperatorID type
    // @Description: MAV_ODID_OPERATOR_ID_TYPE value (0=Undefined, 1=CAA registration)
    AP_GROUPINFO("OP_TYPE", 11, AP_OpenDroneID, _op_id_type, 0),

    AP_GROUPEND
};

#if defined(OPENDRONEID_UA_TYPE)
// ensure the type is within the allowed range
#if OPENDRONEID_UA_TYPE < 0 || OPENDRONEID_UA_TYPE > 15
#error "OPENDRONEID_UA_TYPE must be between 0 and 15"
#endif
#endif

// constructor
AP_OpenDroneID::AP_OpenDroneID()
{
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    if (_singleton != nullptr) {
        AP_HAL::panic("OpenDroneID must be singleton");
    }
#endif
    _singleton = this;
    AP_Param::setup_object_defaults(this, var_info);
    id_len = 0;
    id_str[0] = '\0';
    id_type[0] = '\0';
    ua_type[0] = '\0';
}

void AP_OpenDroneID::init()
{
    if (_enable == 0) {
        return;
    }

    load_UAS_ID_from_persistent_memory();
    load_operator_id_from_params();
    _chan = mavlink_channel_t(gcs().get_channel_from_port_number(_mav_port));
    _initialised = true;
}

// ---------- OperatorID pack/unpack helpers ----------

static inline int32_t pack4(const uint8_t *src)
{
    return (int32_t(src[0]) << 24) | (int32_t(src[1]) << 16) |
           (int32_t(src[2]) <<  8) |  int32_t(src[3]);
}

static inline void unpack4(int32_t v, uint8_t *dst)
{
    dst[0] = uint8_t((uint32_t(v) >> 24) & 0xFF);
    dst[1] = uint8_t((uint32_t(v) >> 16) & 0xFF);
    dst[2] = uint8_t((uint32_t(v) >>  8) & 0xFF);
    dst[3] = uint8_t( uint32_t(v)        & 0xFF);
}

void AP_OpenDroneID::load_operator_id_from_params()
{
    const int32_t v0 = _op_id_0.get();
    const int32_t v1 = _op_id_1.get();
    const int32_t v2 = _op_id_2.get();
    const int32_t v3 = _op_id_3.get();
    const int32_t v4 = _op_id_4.get();
    if (v0 == 0 && v1 == 0 && v2 == 0 && v3 == 0 && v4 == 0) {
        return;
    }
    uint8_t buf[ODID_ID_SIZE] = {};
    unpack4(v0, buf +  0);
    unpack4(v1, buf +  4);
    unpack4(v2, buf +  8);
    unpack4(v3, buf + 12);
    unpack4(v4, buf + 16);
    WITH_SEMAPHORE(_sem);
    pkt_operator_id.target_system    = mavlink_system.sysid;
    pkt_operator_id.target_component = MAV_COMP_ID_AUTOPILOT1;
    pkt_operator_id.operator_id_type = (uint8_t)_op_id_type.get();
    memcpy(pkt_operator_id.operator_id, buf, ODID_ID_SIZE);
}

void AP_OpenDroneID::save_operator_id_to_params()
{
    uint8_t buf[ODID_ID_SIZE] = {};
    memcpy(buf, pkt_operator_id.operator_id, ODID_ID_SIZE);
    _op_id_0.set_and_save(pack4(buf +  0));
    _op_id_1.set_and_save(pack4(buf +  4));
    _op_id_2.set_and_save(pack4(buf +  8));
    _op_id_3.set_and_save(pack4(buf + 12));
    _op_id_4.set_and_save(pack4(buf + 16));
    _op_id_type.set_and_save((int8_t)pkt_operator_id.operator_id_type);
}

void AP_OpenDroneID::set_operator_id_from_script(const char *op_id, uint8_t type)
{
    if (op_id == nullptr || op_id[0] == '\0') {
        return;
    }
    WITH_SEMAPHORE(_sem);
    pkt_operator_id.target_system    = mavlink_system.sysid;
    pkt_operator_id.target_component = MAV_COMP_ID_AUTOPILOT1;
    pkt_operator_id.operator_id_type = type;
    memset(pkt_operator_id.operator_id, 0, sizeof(pkt_operator_id.operator_id));
    strncpy(pkt_operator_id.operator_id, op_id, sizeof(pkt_operator_id.operator_id) - 1);
    save_operator_id_to_params();
}

bool AP_OpenDroneID::transmitter_healthy(uint32_t max_age_ms) const
{
    if (last_arm_status_ms == 0) {
        return false;
    }
    return (AP_HAL::millis() - last_arm_status_ms) <= max_age_ms;
}

bool AP_OpenDroneID::rid_heartbeat_enabled() const
{
    // RIDHB_ENABLE is created by rid_heartbeat_check.lua (AP_Float scripting param).
    enum ap_var_type ptype;
    const AP_Param *vp = AP_Param::find("RIDHB_ENABLE", &ptype);
    if (vp == nullptr || ptype != AP_PARAM_FLOAT) {
        return true;
    }
    return ((const AP_Float *)vp)->get() >= 1.0f;
}

// ---------- end OperatorID helpers ----------

void AP_OpenDroneID::load_UAS_ID_from_persistent_memory()
{
    // Prefer an ID already supplied at boot (e.g. Frame SN from FactorySN).
    if (id_len > 0) {
        return;
    }
    id_len = sizeof(id_str);
    size_t id_type_len = sizeof(id_type);
    size_t ua_type_len = sizeof(ua_type);
    if (hal.util->get_persistent_param_by_name("DID_UAS_ID", id_str, id_len) &&
        hal.util->get_persistent_param_by_name("DID_UAS_ID_TYPE", id_type, id_type_len) &&
        hal.util->get_persistent_param_by_name("DID_UA_TYPE", ua_type, ua_type_len)) {
        if (id_len && id_type_len && ua_type_len) {
            _options.set_and_save(_options.get() & ~LockUASIDOnFirstBasicIDRx);
            _options.notify();
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "OpenDroneID: Locked UAS_ID: %s", id_str);
        }
    } else {
        id_len = 0;
    }
}

void AP_OpenDroneID::set_uas_id(const char *uas_id, uint8_t id_type_v, uint8_t ua_type_v)
{
    if (uas_id == nullptr || uas_id[0] == '\0') {
        return;
    }

    // ODID BasicID.uas_id is fixed 20 bytes; keep a trailing NUL in id_str.
    strncpy(id_str, uas_id, ODID_ID_SIZE);
    id_str[ODID_ID_SIZE] = '\0';
    id_len = strlen(id_str);

    snprintf(id_type, sizeof(id_type), "%u", (unsigned)id_type_v);
    snprintf(ua_type, sizeof(ua_type), "%u", (unsigned)ua_type_v);

    WITH_SEMAPHORE(_sem);
    pkt_basic_id.target_system = gcs().sysid_this_mav();
    pkt_basic_id.target_component = MAV_COMP_ID_ODID_TXRX_1;
    pkt_basic_id.id_type = id_type_v;
    pkt_basic_id.ua_type = ua_type_v;
    memset(pkt_basic_id.uas_id, 0, sizeof(pkt_basic_id.uas_id));
    memcpy(pkt_basic_id.uas_id, id_str, id_len);
}

void AP_OpenDroneID::set_basic_id() {
    if (pkt_basic_id.id_type != MAV_ODID_ID_TYPE_NONE) {
        return;
    }
    if (id_len > 0) {
        // prepare basic id pkt
        uint8_t val = gcs().sysid_this_mav();
        pkt_basic_id.target_system = val;
        pkt_basic_id.target_component = MAV_COMP_ID_ODID_TXRX_1;
        pkt_basic_id.id_type = atoi(id_type);
        pkt_basic_id.ua_type = atoi(ua_type);
        char buffer[21];
        snprintf(buffer, sizeof(buffer), "%s", id_str);
        memcpy(pkt_basic_id.uas_id, buffer, sizeof(pkt_basic_id.uas_id));
    }
}

void AP_OpenDroneID::get_persistent_params(ExpandingString &str) const
{
    if ((pkt_basic_id.id_type == MAV_ODID_ID_TYPE_SERIAL_NUMBER)
        && (_options & LockUASIDOnFirstBasicIDRx)
        && id_len == 0) {
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "OpenDroneID: ID is locked as %s", pkt_basic_id.uas_id);
        str.printf("DID_UAS_ID=%s\nDID_UAS_ID_TYPE=%u\nDID_UA_TYPE=%u\n", pkt_basic_id.uas_id, pkt_basic_id.id_type, pkt_basic_id.ua_type);
    }
}

// Perform the pre-arm checks and prevent arming if they are not satisifed
// Except in the case of an in-flight reboot
bool AP_OpenDroneID::pre_arm_check_nolock(char* failmsg, uint8_t failmsg_len) const
{
    // RIDHB_ENABLE=0: ignore RID ARM_STATUS errors, do not block arming
    if (!rid_heartbeat_enabled()) {
        return true;
    }

    // 106/107 (no-fly): block arm even when DID_OPTIONS EnforceArming is off
    if (_enable != 0 &&
        (has_error_code_nolock("106") || has_error_code_nolock("107"))) {
        if (arm_status.error[0] != '\0') {
            strncpy(failmsg, arm_status.error, failmsg_len);
        } else {
            strncpy(failmsg, "RID error 106/107", failmsg_len);
        }
        return false;
    }

    if (!option_enabled(Options::EnforceArming)) {
        return true;
    }

    if(_enable == 0) {
        strncpy(failmsg, "DID_ENABLE must be 1", failmsg_len);
        return false;
    }

    if (pkt_basic_id.id_type == MAV_ODID_ID_TYPE_NONE) {
        strncpy(failmsg, "UA_TYPE required in BasicID", failmsg_len);
        return false;
    }

    if (pkt_system.operator_latitude == 0 && pkt_system.operator_longitude == 0) {
        strncpy(failmsg, "operator location must be set", failmsg_len);
        return false;
    }

    const uint32_t max_age_ms = 3000;
    const uint32_t now_ms = AP_HAL::millis();

    if (last_arm_status_ms == 0 || now_ms - last_arm_status_ms > max_age_ms) {
        strncpy(failmsg, "ARM_STATUS not available", failmsg_len);
        return false;
    }

    if (last_system_ms == 0 ||
        (now_ms - last_system_ms > max_age_ms &&
         (now_ms - last_system_update_ms > max_age_ms))) {
        strncpy(failmsg, "SYSTEM not available", failmsg_len);
        return false;
    }

    if (arm_status.status != MAV_ODID_ARM_STATUS_GOOD_TO_ARM) {
        strncpy(failmsg, arm_status.error, failmsg_len);
        return false;
    }

    return true;
}

bool AP_OpenDroneID::pre_arm_check(char* failmsg, uint8_t failmsg_len)
{
    WITH_SEMAPHORE(_sem);
    return pre_arm_check_nolock(failmsg, failmsg_len);
}

void AP_OpenDroneID::update()
{
    if (_enable == 0) {
        return;
    }

    if ((pkt_basic_id.id_type == MAV_ODID_ID_TYPE_SERIAL_NUMBER)
        && (_options & LockUASIDOnFirstBasicIDRx)
        && id_len == 0
        && !bootloader_flashed) {
        hal.util->flash_bootloader();
        // reset the basic id on next set_basic_id call
        pkt_basic_id.id_type = MAV_ODID_ID_TYPE_NONE;
        bootloader_flashed = true;
    }

    set_basic_id();

    const bool armed = hal.util->get_soft_armed();
    if (armed && !_was_armed) {
        // use arm location as takeoff location
        AP::ahrs().get_location(_takeoff_location);
    }
    _was_armed = armed;

    send_dynamic_out();
    send_static_out();
#if HAL_ENABLE_DRONECAN_DRIVERS
    uint8_t can_num_drivers = AP::can().get_num_drivers();
    for (uint8_t i = 0; i < can_num_drivers; i++) {
        AP_DroneCAN *dronecan = AP_DroneCAN::get_dronecan(i);
        if (dronecan == nullptr) {
            continue;
        }
        if (dronecan->get_driver_index()+1 != _can_driver) {
            continue;
        }
        // send messages
        dronecan_send(dronecan);
    }
#endif
}

// local payload space check which treats invalid channel as having space
// needed to populate the message structures for the DroneCAN backend
#define ODID_HAVE_PAYLOAD_SPACE(id) (_chan == MAV_CHAN_INVALID || HAVE_PAYLOAD_SPACE(_chan, id))

void AP_OpenDroneID::send_dynamic_out()
{
    const uint32_t now = AP_HAL::millis();
    if (now - _last_send_location_ms >= _mavlink_dynamic_period_ms &&
        ODID_HAVE_PAYLOAD_SPACE(OPEN_DRONE_ID_LOCATION)) {
        _last_send_location_ms = now;
        send_location_message();
    }

    // operator location needs to be sent at the same rate as location for FAA compliance
    if (now - _last_send_system_update_ms >= _mavlink_dynamic_period_ms &&
        ODID_HAVE_PAYLOAD_SPACE(OPEN_DRONE_ID_SYSTEM_UPDATE)) {
        _last_send_system_update_ms = now;
        send_system_update_message();
    }
}

void AP_OpenDroneID::send_static_out()
{
    const uint32_t now_ms = AP_HAL::millis();

    // we need to notify user if we lost the transmitter
    if (now_ms - last_arm_status_ms > 5000) {
        if (last_lost_tx_ms == 0 || now_ms - last_lost_tx_ms > ODID_LOST_WARN_REPEAT_MS) {
            last_lost_tx_ms = now_ms;
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "ODID: lost transmitter");
        }
    } else if (last_lost_tx_ms != 0) {
        // we're OK again
        last_lost_tx_ms = 0;
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ODID: transmitter OK");
    }

    // we need to notify user if we lost system msg with operator location
    if (now_ms - last_system_ms > 5000 &&
        (last_lost_operator_msg_ms == 0 || now_ms - last_lost_operator_msg_ms > ODID_LOST_WARN_REPEAT_MS)) {
        last_lost_operator_msg_ms = now_ms;
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "ODID: lost operator location");
    }
    
    const uint32_t msg_spacing_ms = _mavlink_static_period_ms / 4;
    if (now_ms - last_msg_send_ms >= msg_spacing_ms) {
        // allow update of channel during setup, this makes it easy to debug with a GCS
        _chan = mavlink_channel_t(gcs().get_channel_from_port_number(_mav_port));
        bool sent_ok = false;
        switch (next_msg_to_send) {
        case NEXT_MSG_BASIC_ID:
            if (ODID_HAVE_PAYLOAD_SPACE(OPEN_DRONE_ID_BASIC_ID)) {
                send_basic_id_message();
                sent_ok = true;
            }
            break;
        case NEXT_MSG_SYSTEM:
            if (ODID_HAVE_PAYLOAD_SPACE(OPEN_DRONE_ID_SYSTEM)) {
                send_system_message();
                sent_ok = true;
            }
            break;
        case NEXT_MSG_SELF_ID:
            if (ODID_HAVE_PAYLOAD_SPACE(OPEN_DRONE_ID_SELF_ID)) {
                send_self_id_message();
                sent_ok = true;
            }
            break;
        case NEXT_MSG_OPERATOR_ID:
            if (ODID_HAVE_PAYLOAD_SPACE(OPEN_DRONE_ID_OPERATOR_ID)) {
                send_operator_id_message();
                sent_ok = true;
            }
            break;
        case NEXT_MSG_ENUM_END:
            break;
        }
        if (sent_ok) {
            last_msg_send_ms = now_ms;
            next_msg_to_send = next_msg((uint8_t(next_msg_to_send) + 1) % uint8_t(NEXT_MSG_ENUM_END));
        }
    }
}

// The send_location_message
// all open_drone_id send functions use data stored in the open drone id class.
// This location send function is an exception. It uses live location data from the ArduPilot system.
void AP_OpenDroneID::send_location_message()
{
    auto &ahrs = AP::ahrs();
    const auto &barometer = AP::baro();
    const auto &gps = AP::gps();

    const AP_GPS::GPS_Status gps_status = gps.status();
    const bool got_bad_gps_fix = (gps_status < AP_GPS::GPS_Status::GPS_OK_FIX_3D);
    const bool armed = hal.util->get_soft_armed();

    Location current_location;
    if (!ahrs.get_location(current_location)) {
        return;
    }
    uint8_t uav_status = hal.util->get_soft_armed()? MAV_ODID_STATUS_AIRBORNE : MAV_ODID_STATUS_GROUND;
#if HAL_PARACHUTE_ENABLED
    // set emergency status if chute is released
    const auto *parachute = AP::parachute();
    if (parachute != nullptr && parachute->released()) {
        uav_status = MAV_ODID_STATUS_EMERGENCY;
    }
#endif
    if (AP::vehicle()->is_crashed()) {
        // if in crashed state also declare an emergency
        uav_status = MAV_ODID_STATUS_EMERGENCY;
    }

    // if we are armed with no GPS fix and we haven't specifically
    // allowed for non-GPS operation then declare an emergency
    if (got_bad_gps_fix && armed && !option_enabled(Options::AllowNonGPSPosition)) {
        uav_status = MAV_ODID_STATUS_EMERGENCY;
    }

    // if we are disarmed and falling at over 3m/s then declare an
    // emergency. This covers cases such as deliberate crash with
    // advanced failsafe and an unintended reboot or in-flight disarm
    if (!got_bad_gps_fix && !armed && gps.velocity().z > 3.0) {
        uav_status = MAV_ODID_STATUS_EMERGENCY;
    }

    // if we have watchdogged while armed then declare an emergency
    if (hal.util->was_watchdog_armed()) {
        uav_status = MAV_ODID_STATUS_EMERGENCY;
    }

    float direction = ODID_INV_DIR;
    if (!got_bad_gps_fix) {
        direction = wrap_360(degrees(ahrs.groundspeed_vector().angle())); // heading (degrees)
    }

    const float speed_horizontal = create_speed_horizontal(ahrs.groundspeed());

    Vector3f velNED;
    UNUSED_RESULT(ahrs.get_velocity_NED(velNED));
    const float climb_rate = create_speed_vertical(-velNED.z); //make sure climb_rate is within Remote ID limit

    int32_t latitude = 0;
    int32_t longitude = 0;
    if (current_location.check_latlng()) { //set location if they are valid
        latitude = current_location.lat;
        longitude = current_location.lng;
    }

    // altitude referenced against 1013.2mb
    const float base_press_mbar = 1013.2;
    const float altitude_barometric = create_altitude(barometer.get_altitude_difference(base_press_mbar*100, barometer.get_pressure()));

    float altitude_geodetic = -1000;
    int32_t alt_amsl_cm;
    float undulation;
    if (current_location.get_alt_cm(Location::AltFrame::ABSOLUTE, alt_amsl_cm)) {
        altitude_geodetic = alt_amsl_cm * 0.01;
    }
    if (gps.get_undulation(undulation)) {
        altitude_geodetic -= undulation;
    }

    // Compute the current height above the takeoff location
    float height_above_takeoff = 0;     // height above takeoff (meters)
    if (hal.util->get_soft_armed()) {
        int32_t curr_alt_asml_cm;
        int32_t takeoff_alt_asml_cm;
        if (current_location.get_alt_cm(Location::AltFrame::ABSOLUTE, curr_alt_asml_cm) &&
            _takeoff_location.get_alt_cm(Location::AltFrame::ABSOLUTE, takeoff_alt_asml_cm)) {
            height_above_takeoff = (curr_alt_asml_cm - takeoff_alt_asml_cm) * 0.01;
        }
    }

    // Accuracy

    // If we have GPS 3D lock we presume that the accuracies of the system will track the GPS's reported accuracy
    MAV_ODID_HOR_ACC horizontal_accuracy_mav = MAV_ODID_HOR_ACC_UNKNOWN;
    MAV_ODID_VER_ACC vertical_accuracy_mav = MAV_ODID_VER_ACC_UNKNOWN;
    MAV_ODID_SPEED_ACC speed_accuracy_mav = MAV_ODID_SPEED_ACC_UNKNOWN;
    MAV_ODID_TIME_ACC timestamp_accuracy_mav = MAV_ODID_TIME_ACC_UNKNOWN;

    float horizontal_accuracy;
    if (gps.horizontal_accuracy(horizontal_accuracy)) {
        horizontal_accuracy_mav = create_enum_horizontal_accuracy(horizontal_accuracy);
    }

    float vertical_accuracy;
    if (gps.vertical_accuracy(vertical_accuracy)) {
        vertical_accuracy_mav = create_enum_vertical_accuracy(vertical_accuracy);
    }

    float speed_accuracy;
    if (gps.speed_accuracy(speed_accuracy)) {
        speed_accuracy_mav = create_enum_speed_accuracy(speed_accuracy);
    }

    // if we have ever had GPS lock then we will have better than 1s
    // accuracy, as we use system timer to propogate time
    timestamp_accuracy_mav =  create_enum_timestamp_accuracy(1.0);

    // Barometer altitude accuraacy will be highly dependent on the airframe and installation of the barometer in use
    // thus ArduPilot cannot reasonably fill this in.
    // Instead allow a manufacturer to use a parameter to fill this in
    uint8_t barometer_accuracy = MAV_ODID_VER_ACC_UNKNOWN; //ahrs class does not provide accuracy readings
    if (!is_equal(_baro_accuracy.get(), -1.0f)) {
        barometer_accuracy = create_enum_vertical_accuracy(_baro_accuracy);
    }

    // Timestamp here is the number of seconds after into the current hour referenced to UTC time (up to one hour)

    // FIX we need to only set this if w have a GPS lock is 2D good enough for that?
    float timestamp = ODID_INV_TIMESTAMP;
    if (!got_bad_gps_fix) {
        uint32_t time_week_ms = gps.time_week_ms();
        timestamp = float(time_week_ms % (3600 * 1000)) * 0.001;
        timestamp = create_location_timestamp(timestamp);   //make sure timestamp is within Remote ID limit
    }


    {
        WITH_SEMAPHORE(_sem);
        // take semaphore so CAN gets a consistent packet
        pkt_location = mavlink_open_drone_id_location_t{
        latitude : latitude,
        longitude : longitude,
        altitude_barometric : altitude_barometric,
        altitude_geodetic : altitude_geodetic,
        height : height_above_takeoff,
        timestamp : timestamp,
        direction : uint16_t(direction * 100.0), // Heading (centi-degrees)
        speed_horizontal : uint16_t(speed_horizontal * 100.0), // Ground speed (cm/s)
        speed_vertical : int16_t(climb_rate * 100.0), // Climb rate (cm/s)
        target_system : 0,
        target_component : 0,
        id_or_mac : {},
        status : uint8_t(uav_status),
        height_reference : MAV_ODID_HEIGHT_REF_OVER_TAKEOFF,           // height reference enum: Above takeoff location or above ground
        horizontal_accuracy : uint8_t(horizontal_accuracy_mav),
        vertical_accuracy : uint8_t(vertical_accuracy_mav),
        barometer_accuracy : barometer_accuracy,
        speed_accuracy : uint8_t(speed_accuracy_mav),
        timestamp_accuracy : uint8_t(timestamp_accuracy_mav)
        };
        need_send_location = dronecan_send_all;
    }

    if (_chan != MAV_CHAN_INVALID) {
        mavlink_msg_open_drone_id_location_send_struct(_chan, &pkt_location);
    }
}

void AP_OpenDroneID::send_basic_id_message()
{
    // note that packet is filled in by the GCS
    need_send_basic_id |= dronecan_send_all;
    if (_chan != MAV_CHAN_INVALID) {
        mavlink_msg_open_drone_id_basic_id_send_struct(_chan, &pkt_basic_id);
    }
}

void AP_OpenDroneID::send_system_message()
{
    // note that packet is filled in by the GCS
    need_send_system |= dronecan_send_all;
    if (_chan != MAV_CHAN_INVALID) {
        mavlink_msg_open_drone_id_system_send_struct(_chan, &pkt_system);
    }
}

void AP_OpenDroneID::send_self_id_message()
{
    need_send_self_id |= dronecan_send_all;
    if (_chan != MAV_CHAN_INVALID) {
        mavlink_msg_open_drone_id_self_id_send_struct(_chan, &pkt_self_id);
    }
}

void AP_OpenDroneID::send_system_update_message()
{
    need_send_system |= dronecan_send_all;
    // note that packet is filled in by the GCS
    if (_chan != MAV_CHAN_INVALID) {
        const auto pkt_system_update = mavlink_open_drone_id_system_update_t {
        operator_latitude : pkt_system.operator_latitude,
        operator_longitude : pkt_system.operator_longitude,
        operator_altitude_geo : pkt_system.operator_altitude_geo,
        timestamp : pkt_system.timestamp,
        target_system : pkt_system.target_system,
        target_component : pkt_system.target_component,
        };
        mavlink_msg_open_drone_id_system_update_send_struct(_chan, &pkt_system_update);
    }
}

void AP_OpenDroneID::send_operator_id_message()
{
    need_send_operator_id |= dronecan_send_all;
    // note that packet is filled in by the GCS
    if (_chan != MAV_CHAN_INVALID) {
        mavlink_msg_open_drone_id_operator_id_send_struct(_chan, &pkt_operator_id);
    }
}

/*
* This converts a horizontal accuracy float value to the corresponding enum
*
* @param Accuracy The horizontal accuracy in meters
* @return Enum value representing the accuracy
*/
MAV_ODID_HOR_ACC AP_OpenDroneID::create_enum_horizontal_accuracy(float accuracy) const
{
    // Out of bounds return UNKNOWN flag
    if (accuracy < 0.0 || accuracy >= 18520.0) {
        return MAV_ODID_HOR_ACC_UNKNOWN;
    }

    static const struct {
        float accuracy;                 // Accuracy bound in meters
        MAV_ODID_HOR_ACC mavoutput;     // mavlink enum output
    } horiz_accuracy_table[] = {
        { 1.0,    MAV_ODID_HOR_ACC_1_METER},
        { 3.0,    MAV_ODID_HOR_ACC_3_METER},
        {10.0,    MAV_ODID_HOR_ACC_10_METER},
        {30.0,    MAV_ODID_HOR_ACC_30_METER},
        {92.6,    MAV_ODID_HOR_ACC_0_05NM},
        {185.2,   MAV_ODID_HOR_ACC_0_1NM},
        {555.6,   MAV_ODID_HOR_ACC_0_3NM},
        {926.0,   MAV_ODID_HOR_ACC_0_5NM},
        {1852.0,  MAV_ODID_HOR_ACC_1NM},
        {3704.0,  MAV_ODID_HOR_ACC_2NM},
        {7408.0,  MAV_ODID_HOR_ACC_4NM},
        {18520.0, MAV_ODID_HOR_ACC_10NM},
    };

    for (auto elem : horiz_accuracy_table) {
        if (accuracy < elem.accuracy) {
            return elem.mavoutput;
        }
    }

    // Should not reach this
    return MAV_ODID_HOR_ACC_UNKNOWN;
}

/**
* This converts a vertical accuracy float value to the corresponding enum
*
* @param Accuracy The vertical accuracy in meters
* @return Enum value representing the accuracy
*/
MAV_ODID_VER_ACC AP_OpenDroneID::create_enum_vertical_accuracy(float accuracy) const
{
    // Out of bounds return UNKNOWN flag
    if (accuracy < 0.0 || accuracy >= 150.0) {
        return MAV_ODID_VER_ACC_UNKNOWN;
    }

    static const struct {
        float accuracy;                 // Accuracy bound in meters
        MAV_ODID_VER_ACC mavoutput;     // mavlink enum output
    } vertical_accuracy_table[] = {
        { 1.0,  MAV_ODID_VER_ACC_1_METER},
        { 3.0,  MAV_ODID_VER_ACC_3_METER},
        {10.0,  MAV_ODID_VER_ACC_10_METER},
        {25.0,  MAV_ODID_VER_ACC_25_METER},
        {45.0,  MAV_ODID_VER_ACC_45_METER},
        {150.0, MAV_ODID_VER_ACC_150_METER},
    };

    for (auto elem : vertical_accuracy_table) {
        if (accuracy < elem.accuracy) {
            return elem.mavoutput;
        }
    }

    // Should not reach this
    return MAV_ODID_VER_ACC_UNKNOWN;
}

/**
* This converts a speed accuracy float value to the corresponding enum
*
* @param Accuracy The speed accuracy in m/s
* @return Enum value representing the accuracy
*/
MAV_ODID_SPEED_ACC AP_OpenDroneID::create_enum_speed_accuracy(float accuracy) const
{
    // Out of bounds return UNKNOWN flag
    if (accuracy < 0.0 || accuracy >= 10.0) {
        return MAV_ODID_SPEED_ACC_UNKNOWN;
    }

    if (accuracy < 0.3) {
        return MAV_ODID_SPEED_ACC_0_3_METERS_PER_SECOND;
    } else if (accuracy < 1.0) {
        return MAV_ODID_SPEED_ACC_1_METERS_PER_SECOND;
    } else if (accuracy < 3.0) {
        return MAV_ODID_SPEED_ACC_3_METERS_PER_SECOND;
    } else if (accuracy < 10.0) {
        return MAV_ODID_SPEED_ACC_10_METERS_PER_SECOND;
    }

    // Should not reach this
    return MAV_ODID_SPEED_ACC_UNKNOWN;
}

/**
* This converts a timestamp accuracy float value to the corresponding enum
*
* @param Accuracy The timestamp accuracy in seconds
* @return Enum value representing the accuracy
*/
MAV_ODID_TIME_ACC AP_OpenDroneID::create_enum_timestamp_accuracy(float accuracy) const
{
    // Out of bounds return UNKNOWN flag
    if (accuracy < 0.0 || accuracy >= 1.5) {
        return MAV_ODID_TIME_ACC_UNKNOWN;
    }

    static const MAV_ODID_TIME_ACC mavoutput [15] = {
        MAV_ODID_TIME_ACC_0_1_SECOND,
        MAV_ODID_TIME_ACC_0_2_SECOND,
        MAV_ODID_TIME_ACC_0_3_SECOND,
        MAV_ODID_TIME_ACC_0_4_SECOND,
        MAV_ODID_TIME_ACC_0_5_SECOND,
        MAV_ODID_TIME_ACC_0_6_SECOND,
        MAV_ODID_TIME_ACC_0_7_SECOND,
        MAV_ODID_TIME_ACC_0_8_SECOND,
        MAV_ODID_TIME_ACC_0_9_SECOND,
        MAV_ODID_TIME_ACC_1_0_SECOND,
        MAV_ODID_TIME_ACC_1_1_SECOND,
        MAV_ODID_TIME_ACC_1_2_SECOND,
        MAV_ODID_TIME_ACC_1_3_SECOND,
        MAV_ODID_TIME_ACC_1_4_SECOND,
        MAV_ODID_TIME_ACC_1_5_SECOND,
    };

    for (int8_t i = 1; i <= 15; i++) {
        if (accuracy <= 0.1 * i) {
            return mavoutput[i-1];
        }
    }

    // Should not reach this
    return MAV_ODID_TIME_ACC_UNKNOWN;
}

// make sure value is within limits of remote ID standard
uint16_t AP_OpenDroneID::create_speed_horizontal(uint16_t speed) const
{
    if (speed > ODID_MAX_SPEED_H) { // constraint function can't be used, because out of range value is invalid
        speed = ODID_INV_SPEED_H;
    }

    return speed;
}

// make sure value is within limits of remote ID standard
int16_t AP_OpenDroneID::create_speed_vertical(int16_t speed) const
{
    if (speed > ODID_MAX_SPEED_V) { // constraint function can't be used, because out of range value is invalid
        speed = ODID_INV_SPEED_V;
    } else if (speed < ODID_MIN_SPEED_V) {
        speed = ODID_INV_SPEED_V;
    }

    return speed;
}

// make sure value is within limits of remote ID standard
float AP_OpenDroneID::create_altitude(float altitude) const
{
    if (altitude > ODID_MAX_ALT) { // constraint function can't be used, because out of range value is invalid
        altitude = ODID_INV_ALT;
    } else if (altitude < ODID_MIN_ALT) {
        altitude = ODID_INV_ALT;
    }

    return altitude;
}

// make sure value is within limits of remote ID standard
float AP_OpenDroneID::create_location_timestamp(float timestamp) const
{
    if (timestamp > ODID_MAX_TIMESTAMP) { // constraint function can't be used, because out of range value is invalid
        timestamp = ODID_INV_TIMESTAMP;
    } else if (timestamp < 0) {
        timestamp = ODID_INV_TIMESTAMP;
    }

    return timestamp;
}

static bool odid_field_non_empty(const char *str, size_t len)
{
    for (size_t i = 0; i < len; i++) {
        if (str[i] == '\0') {
            return i > 0;
        }
    }
    return true;
}

uint32_t AP_OpenDroneID::compute_rid_status_flags(uint32_t now_ms) const
{
    uint32_t flags = 0;
    const uint32_t max_age_ms = 3000;

    if (_enable.get() != 0) {
        flags |= (1U << 0);
    }
    if (pkt_basic_id.id_type != MAV_ODID_ID_TYPE_NONE) {
        flags |= (1U << 1);
    }
    if (odid_field_non_empty(pkt_operator_id.operator_id, sizeof(pkt_operator_id.operator_id))) {
        flags |= (1U << 2);
    }
    if (pkt_system.operator_latitude != 0 || pkt_system.operator_longitude != 0) {
        flags |= (1U << 3);
    }
    if (odid_field_non_empty(pkt_self_id.description, sizeof(pkt_self_id.description))) {
        flags |= (1U << 4);
    }
    if (last_arm_status_ms != 0 && now_ms - last_arm_status_ms < max_age_ms) {
        flags |= (1U << 5);
    }
    const uint32_t system_latest_ms = MAX(last_system_ms, last_system_update_ms);
    if (system_latest_ms != 0 && now_ms - system_latest_ms < max_age_ms) {
        flags |= (1U << 6);
    }
    if (last_arm_status_ms != 0 && now_ms - last_arm_status_ms < 5000) {
        flags |= (1U << 7);
    }
    if (arm_status.status == MAV_ODID_ARM_STATUS_GOOD_TO_ARM) {
        flags |= (1U << 8);
    }
    char failmsg[50] {};
    if (pre_arm_check_nolock(failmsg, sizeof(failmsg))) {
        flags |= (1U << 9);
    }
    if (option_enabled(Options::LockUASIDOnFirstBasicIDRx) && id_len > 0) {
        flags |= (1U << 10);
    }

    return flags;
}

void AP_OpenDroneID::clear_rid_config_data()
{
    // Restore DID parameters to EFT defaults (persist). DID_OPTIONS left unchanged.
    _enable.set_and_save(1);
    _mav_port.set_and_save(2);
    _can_driver.set_and_save(0);
    _enable.notify();
    _mav_port.notify();
    _can_driver.notify();

    WITH_SEMAPHORE(_sem);

    // Clear runtime OpenDroneID payloads seen by EFT_RID_CONFIG_STATUS.
    memset(&pkt_basic_id, 0, sizeof(pkt_basic_id));
    memset(&pkt_operator_id, 0, sizeof(pkt_operator_id));
    _op_id_0.set_and_save(0);
    _op_id_1.set_and_save(0);
    _op_id_2.set_and_save(0);
    _op_id_3.set_and_save(0);
    _op_id_4.set_and_save(0);
    _op_id_type.set_and_save(0);
    memset(&pkt_self_id, 0, sizeof(pkt_self_id));
    memset(&pkt_system, 0, sizeof(pkt_system));
    memset(&pkt_location, 0, sizeof(pkt_location));
    memset(&arm_status, 0, sizeof(arm_status));

    last_arm_status_ms = 0;
    last_system_ms = 0;
    last_system_update_ms = 0;
    last_lost_tx_ms = 0;
    last_lost_operator_msg_ms = 0;

    // Allow a new BASIC_ID to be accepted (runtime lock / cached persistent ID).
    id_len = 0;
    id_str[0] = '\0';
    id_type[0] = '\0';
    ua_type[0] = '\0';
    bootloader_flashed = false;
    _have_height_above_takeoff = false;

    _chan = mavlink_channel_t(gcs().get_channel_from_port_number(_mav_port));
    _initialised = true;
}

void AP_OpenDroneID::handle_rid_config_request(mavlink_channel_t chan, const mavlink_message_t &msg)
{
    mavlink_eft_rid_config_request_t request {};
    mavlink_msg_eft_rid_config_request_decode(&msg, &request);

    if (request.target_system != 0 && request.target_system != gcs().sysid_this_mav()) {
        return;
    }

    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - last_rid_config_reply_ms < 200) {
        return;
    }
    last_rid_config_reply_ms = now_ms;

    // type: 0=query, 1=clear RID then reply, 2=FactorySN clear (Copter) then reply
    if (request.type == 1) {
        clear_rid_config_data();
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ODID: RID config cleared");
    }
    // type=2 does not touch RID data; FactorySN erase is done in GCS_MAVLINK_Copter.

    if (!HAVE_PAYLOAD_SPACE(chan, EFT_RID_CONFIG_STATUS)) {
        return;
    }

    mavlink_eft_rid_config_status_t reply {};
    reply.target_system = msg.sysid;
    reply.target_component = msg.compid;
    reply.seq = request.seq;
    reply.did_enable = uint8_t(_enable.get());
    reply.did_mavport = int8_t(_mav_port.get());
    reply.did_options = uint8_t(_options.get());
    reply.did_can_driver = uint8_t(_can_driver.get());
    reply.arm_status_age_ms = 0xFFFF;
    reply.system_age_ms = 0xFFFF;

    WITH_SEMAPHORE(_sem);

    reply.ua_type = pkt_basic_id.ua_type;
    reply.id_type = pkt_basic_id.id_type;
    memcpy(reply.uas_id, pkt_basic_id.uas_id, sizeof(reply.uas_id));
    reply.op_id_type = pkt_operator_id.operator_id_type;
    memcpy(reply.operator_id, pkt_operator_id.operator_id, sizeof(reply.operator_id));
    reply.desc_type = pkt_self_id.description_type;
    memcpy(reply.self_desc, pkt_self_id.description, sizeof(reply.self_desc));
    reply.operator_latitude = pkt_system.operator_latitude;
    reply.operator_longitude = pkt_system.operator_longitude;
    reply.operator_altitude_geo = pkt_system.operator_altitude_geo;
    reply.arm_status = arm_status.status;
    strncpy(reply.arm_error, arm_status.error, sizeof(reply.arm_error) - 1);
    if (!rid_heartbeat_enabled()) {
        // RIDHB_ENABLE=0: report as no-error to GCS
        reply.arm_status = MAV_ODID_ARM_STATUS_GOOD_TO_ARM;
        memset(reply.arm_error, 0, sizeof(reply.arm_error));
    }
    reply.arm_error[sizeof(reply.arm_error) - 1] = '\0';

    if (last_arm_status_ms != 0) {
        reply.arm_status_age_ms = uint16_t(MIN(now_ms - last_arm_status_ms, 0xFFFFU));
    }

    const uint32_t system_latest_ms = MAX(last_system_ms, last_system_update_ms);
    if (system_latest_ms != 0) {
        reply.system_age_ms = uint16_t(MIN(now_ms - system_latest_ms, 0xFFFFU));
    }

    reply.status_flags = compute_rid_status_flags(now_ms);

    mavlink_msg_eft_rid_config_status_send_struct(chan, &reply);
}

// handle a message from the GCS
void AP_OpenDroneID::handle_msg(mavlink_channel_t chan, const mavlink_message_t &msg)
{
    if (!_initialised) {
        return;
    }
    WITH_SEMAPHORE(_sem);

    switch (msg.msgid) {
    // only accept ARM_STATUS from the transmitter
    case MAVLINK_MSG_ID_OPEN_DRONE_ID_ARM_STATUS: {
        if (chan == _chan) {
            mavlink_msg_open_drone_id_arm_status_decode(&msg, &arm_status);
            last_arm_status_ms = AP_HAL::millis();
            // RIDHB_ENABLE=0: still notify GCS with 12918, but as no-error
            if (!rid_heartbeat_enabled()) {
                mavlink_open_drone_id_arm_status_t clean {};
                clean.status = MAV_ODID_ARM_STATUS_GOOD_TO_ARM;
                gcs().send_to_active_channels(MAVLINK_MSG_ID_OPEN_DRONE_ID_ARM_STATUS,
                                              (const char *)&clean);
            }
        }
        break;
    }
    // accept other messages from the GCS
    case MAVLINK_MSG_ID_OPEN_DRONE_ID_OPERATOR_ID:
        mavlink_msg_open_drone_id_operator_id_decode(&msg, &pkt_operator_id);
        save_operator_id_to_params();
        break;
    case MAVLINK_MSG_ID_OPEN_DRONE_ID_SELF_ID:
        mavlink_msg_open_drone_id_self_id_decode(&msg, &pkt_self_id);
        break;
    case MAVLINK_MSG_ID_OPEN_DRONE_ID_BASIC_ID:
        if (id_len == 0) {
            mavlink_msg_open_drone_id_basic_id_decode(&msg, &pkt_basic_id);
        }
        break;
    case MAVLINK_MSG_ID_OPEN_DRONE_ID_SYSTEM:
        mavlink_msg_open_drone_id_system_decode(&msg, &pkt_system);
        last_system_ms = AP_HAL::millis();
        break;
    case MAVLINK_MSG_ID_OPEN_DRONE_ID_SYSTEM_UPDATE: {
        mavlink_open_drone_id_system_update_t pkt_system_update;
        mavlink_msg_open_drone_id_system_update_decode(&msg, &pkt_system_update);
        pkt_system.operator_latitude = pkt_system_update.operator_latitude;
        pkt_system.operator_longitude = pkt_system_update.operator_longitude;
        pkt_system.operator_altitude_geo = pkt_system_update.operator_altitude_geo;
        pkt_system.timestamp = pkt_system_update.timestamp;
        last_system_update_ms = AP_HAL::millis();
        if (last_system_ms != 0) {
            // we can only mark system as updated if we have the other
            // information already
            last_system_ms = last_system_update_ms;
        }
        break;
    }
    }
}

// True if arm_status.error contains numeric token `code` (e.g. "106"),
// not as part of a longer number (so "1106" does not match "106").
bool AP_OpenDroneID::has_error_code_nolock(const char* code) const
{
    if (code == nullptr || code[0] == '\0') {
        return false;
    }
    if (arm_status.status == MAV_ODID_ARM_STATUS_GOOD_TO_ARM) {
        return false;
    }

    const size_t n = strlen(code);
    for (const char *p = arm_status.error; (p = strstr(p, code)) != nullptr; p++) {
        const bool left_ok = (p == arm_status.error) || !isdigit((unsigned char)p[-1]);
        const bool right_ok = !isdigit((unsigned char)p[n]);
        if (left_ok && right_ok) {
            return true;
        }
    }
    return false;
}

bool AP_OpenDroneID::has_error_code(const char* code)
{
    WITH_SEMAPHORE(_sem);
    return has_error_code_nolock(code);
}

// singleton instance
AP_OpenDroneID *AP_OpenDroneID::_singleton;

namespace AP
{

AP_OpenDroneID &opendroneid()
{
    return *AP_OpenDroneID::get_singleton();
}

}
#endif //AP_OPENDRONEID_ENABLED
