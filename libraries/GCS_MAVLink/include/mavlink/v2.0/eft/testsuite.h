/** @file
 *    @brief MAVLink comm protocol testsuite generated from eft.xml
 *    @see https://mavlink.io/en/
 */
#pragma once
#ifndef EFT_TESTSUITE_H
#define EFT_TESTSUITE_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MAVLINK_TEST_ALL
#define MAVLINK_TEST_ALL

static void mavlink_test_eft(uint8_t, uint8_t, mavlink_message_t *last_msg);

static void mavlink_test_all(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{

    mavlink_test_eft(system_id, component_id, last_msg);
}
#endif




static void mavlink_test_device_status_array(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_DEVICE_STATUS_ARRAY >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_device_status_array_t packet_in = {
        17235,17339,17443,17547,17651,17755,17859,17963,18067,18171,18275,18379,77,144,211,22,89,156,223,34,101,168,235,46,113,180,247,58,125,192,3,70,137,204,15,82
    };
    mavlink_device_status_array_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.baud_rate_0 = packet_in.baud_rate_0;
        packet1.speed_0 = packet_in.speed_0;
        packet1.baud_rate_1 = packet_in.baud_rate_1;
        packet1.speed_1 = packet_in.speed_1;
        packet1.baud_rate_2 = packet_in.baud_rate_2;
        packet1.speed_2 = packet_in.speed_2;
        packet1.baud_rate_3 = packet_in.baud_rate_3;
        packet1.speed_3 = packet_in.speed_3;
        packet1.baud_rate_4 = packet_in.baud_rate_4;
        packet1.speed_4 = packet_in.speed_4;
        packet1.baud_rate_5 = packet_in.baud_rate_5;
        packet1.speed_5 = packet_in.speed_5;
        packet1.fault_status_0 = packet_in.fault_status_0;
        packet1.control_mode_0 = packet_in.control_mode_0;
        packet1.reserved_0 = packet_in.reserved_0;
        packet1.life_signal_0 = packet_in.life_signal_0;
        packet1.fault_status_1 = packet_in.fault_status_1;
        packet1.control_mode_1 = packet_in.control_mode_1;
        packet1.reserved_1 = packet_in.reserved_1;
        packet1.life_signal_1 = packet_in.life_signal_1;
        packet1.fault_status_2 = packet_in.fault_status_2;
        packet1.control_mode_2 = packet_in.control_mode_2;
        packet1.reserved_2 = packet_in.reserved_2;
        packet1.life_signal_2 = packet_in.life_signal_2;
        packet1.fault_status_3 = packet_in.fault_status_3;
        packet1.control_mode_3 = packet_in.control_mode_3;
        packet1.reserved_3 = packet_in.reserved_3;
        packet1.life_signal_3 = packet_in.life_signal_3;
        packet1.fault_status_4 = packet_in.fault_status_4;
        packet1.control_mode_4 = packet_in.control_mode_4;
        packet1.reserved_4 = packet_in.reserved_4;
        packet1.life_signal_4 = packet_in.life_signal_4;
        packet1.fault_status_5 = packet_in.fault_status_5;
        packet1.control_mode_5 = packet_in.control_mode_5;
        packet1.reserved_5 = packet_in.reserved_5;
        packet1.life_signal_5 = packet_in.life_signal_5;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_DEVICE_STATUS_ARRAY_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_DEVICE_STATUS_ARRAY_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_device_status_array_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_device_status_array_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_device_status_array_pack(system_id, component_id, &msg , packet1.fault_status_0 , packet1.control_mode_0 , packet1.baud_rate_0 , packet1.speed_0 , packet1.reserved_0 , packet1.life_signal_0 , packet1.fault_status_1 , packet1.control_mode_1 , packet1.baud_rate_1 , packet1.speed_1 , packet1.reserved_1 , packet1.life_signal_1 , packet1.fault_status_2 , packet1.control_mode_2 , packet1.baud_rate_2 , packet1.speed_2 , packet1.reserved_2 , packet1.life_signal_2 , packet1.fault_status_3 , packet1.control_mode_3 , packet1.baud_rate_3 , packet1.speed_3 , packet1.reserved_3 , packet1.life_signal_3 , packet1.fault_status_4 , packet1.control_mode_4 , packet1.baud_rate_4 , packet1.speed_4 , packet1.reserved_4 , packet1.life_signal_4 , packet1.fault_status_5 , packet1.control_mode_5 , packet1.baud_rate_5 , packet1.speed_5 , packet1.reserved_5 , packet1.life_signal_5 );
    mavlink_msg_device_status_array_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_device_status_array_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.fault_status_0 , packet1.control_mode_0 , packet1.baud_rate_0 , packet1.speed_0 , packet1.reserved_0 , packet1.life_signal_0 , packet1.fault_status_1 , packet1.control_mode_1 , packet1.baud_rate_1 , packet1.speed_1 , packet1.reserved_1 , packet1.life_signal_1 , packet1.fault_status_2 , packet1.control_mode_2 , packet1.baud_rate_2 , packet1.speed_2 , packet1.reserved_2 , packet1.life_signal_2 , packet1.fault_status_3 , packet1.control_mode_3 , packet1.baud_rate_3 , packet1.speed_3 , packet1.reserved_3 , packet1.life_signal_3 , packet1.fault_status_4 , packet1.control_mode_4 , packet1.baud_rate_4 , packet1.speed_4 , packet1.reserved_4 , packet1.life_signal_4 , packet1.fault_status_5 , packet1.control_mode_5 , packet1.baud_rate_5 , packet1.speed_5 , packet1.reserved_5 , packet1.life_signal_5 );
    mavlink_msg_device_status_array_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_device_status_array_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_device_status_array_send(MAVLINK_COMM_1 , packet1.fault_status_0 , packet1.control_mode_0 , packet1.baud_rate_0 , packet1.speed_0 , packet1.reserved_0 , packet1.life_signal_0 , packet1.fault_status_1 , packet1.control_mode_1 , packet1.baud_rate_1 , packet1.speed_1 , packet1.reserved_1 , packet1.life_signal_1 , packet1.fault_status_2 , packet1.control_mode_2 , packet1.baud_rate_2 , packet1.speed_2 , packet1.reserved_2 , packet1.life_signal_2 , packet1.fault_status_3 , packet1.control_mode_3 , packet1.baud_rate_3 , packet1.speed_3 , packet1.reserved_3 , packet1.life_signal_3 , packet1.fault_status_4 , packet1.control_mode_4 , packet1.baud_rate_4 , packet1.speed_4 , packet1.reserved_4 , packet1.life_signal_4 , packet1.fault_status_5 , packet1.control_mode_5 , packet1.baud_rate_5 , packet1.speed_5 , packet1.reserved_5 , packet1.life_signal_5 );
    mavlink_msg_device_status_array_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("DEVICE_STATUS_ARRAY") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_DEVICE_STATUS_ARRAY) != NULL);
#endif
}

static void mavlink_test_device_info1_array(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_DEVICE_INFO1_ARRAY >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_device_info1_array_t packet_in = {
        17235,17339,17443,17547,17651,17755,41,108,175,242,53,120,187,254,65,132,199,10,77,144,211,22,89,156,223,34,101,168,235,46,113,180,247,58,125,192
    };
    mavlink_device_info1_array_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.hours_0 = packet_in.hours_0;
        packet1.pwm_lost_count_0 = packet_in.pwm_lost_count_0;
        packet1.hours_1 = packet_in.hours_1;
        packet1.pwm_lost_count_1 = packet_in.pwm_lost_count_1;
        packet1.hours_2 = packet_in.hours_2;
        packet1.pwm_lost_count_2 = packet_in.pwm_lost_count_2;
        packet1.type_0 = packet_in.type_0;
        packet1.year_0 = packet_in.year_0;
        packet1.month_0 = packet_in.month_0;
        packet1.day_0 = packet_in.day_0;
        packet1.number_0 = packet_in.number_0;
        packet1.hw_major_0 = packet_in.hw_major_0;
        packet1.hw_minor_0 = packet_in.hw_minor_0;
        packet1.sw_major_0 = packet_in.sw_major_0;
        packet1.sw_minor_0 = packet_in.sw_minor_0;
        packet1.minutes_0 = packet_in.minutes_0;
        packet1.type_1 = packet_in.type_1;
        packet1.year_1 = packet_in.year_1;
        packet1.month_1 = packet_in.month_1;
        packet1.day_1 = packet_in.day_1;
        packet1.number_1 = packet_in.number_1;
        packet1.hw_major_1 = packet_in.hw_major_1;
        packet1.hw_minor_1 = packet_in.hw_minor_1;
        packet1.sw_major_1 = packet_in.sw_major_1;
        packet1.sw_minor_1 = packet_in.sw_minor_1;
        packet1.minutes_1 = packet_in.minutes_1;
        packet1.type_2 = packet_in.type_2;
        packet1.year_2 = packet_in.year_2;
        packet1.month_2 = packet_in.month_2;
        packet1.day_2 = packet_in.day_2;
        packet1.number_2 = packet_in.number_2;
        packet1.hw_major_2 = packet_in.hw_major_2;
        packet1.hw_minor_2 = packet_in.hw_minor_2;
        packet1.sw_major_2 = packet_in.sw_major_2;
        packet1.sw_minor_2 = packet_in.sw_minor_2;
        packet1.minutes_2 = packet_in.minutes_2;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_DEVICE_INFO1_ARRAY_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_DEVICE_INFO1_ARRAY_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_device_info1_array_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_device_info1_array_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_device_info1_array_pack(system_id, component_id, &msg , packet1.type_0 , packet1.year_0 , packet1.month_0 , packet1.day_0 , packet1.number_0 , packet1.hw_major_0 , packet1.hw_minor_0 , packet1.sw_major_0 , packet1.sw_minor_0 , packet1.minutes_0 , packet1.hours_0 , packet1.pwm_lost_count_0 , packet1.type_1 , packet1.year_1 , packet1.month_1 , packet1.day_1 , packet1.number_1 , packet1.hw_major_1 , packet1.hw_minor_1 , packet1.sw_major_1 , packet1.sw_minor_1 , packet1.minutes_1 , packet1.hours_1 , packet1.pwm_lost_count_1 , packet1.type_2 , packet1.year_2 , packet1.month_2 , packet1.day_2 , packet1.number_2 , packet1.hw_major_2 , packet1.hw_minor_2 , packet1.sw_major_2 , packet1.sw_minor_2 , packet1.minutes_2 , packet1.hours_2 , packet1.pwm_lost_count_2 );
    mavlink_msg_device_info1_array_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_device_info1_array_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.type_0 , packet1.year_0 , packet1.month_0 , packet1.day_0 , packet1.number_0 , packet1.hw_major_0 , packet1.hw_minor_0 , packet1.sw_major_0 , packet1.sw_minor_0 , packet1.minutes_0 , packet1.hours_0 , packet1.pwm_lost_count_0 , packet1.type_1 , packet1.year_1 , packet1.month_1 , packet1.day_1 , packet1.number_1 , packet1.hw_major_1 , packet1.hw_minor_1 , packet1.sw_major_1 , packet1.sw_minor_1 , packet1.minutes_1 , packet1.hours_1 , packet1.pwm_lost_count_1 , packet1.type_2 , packet1.year_2 , packet1.month_2 , packet1.day_2 , packet1.number_2 , packet1.hw_major_2 , packet1.hw_minor_2 , packet1.sw_major_2 , packet1.sw_minor_2 , packet1.minutes_2 , packet1.hours_2 , packet1.pwm_lost_count_2 );
    mavlink_msg_device_info1_array_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_device_info1_array_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_device_info1_array_send(MAVLINK_COMM_1 , packet1.type_0 , packet1.year_0 , packet1.month_0 , packet1.day_0 , packet1.number_0 , packet1.hw_major_0 , packet1.hw_minor_0 , packet1.sw_major_0 , packet1.sw_minor_0 , packet1.minutes_0 , packet1.hours_0 , packet1.pwm_lost_count_0 , packet1.type_1 , packet1.year_1 , packet1.month_1 , packet1.day_1 , packet1.number_1 , packet1.hw_major_1 , packet1.hw_minor_1 , packet1.sw_major_1 , packet1.sw_minor_1 , packet1.minutes_1 , packet1.hours_1 , packet1.pwm_lost_count_1 , packet1.type_2 , packet1.year_2 , packet1.month_2 , packet1.day_2 , packet1.number_2 , packet1.hw_major_2 , packet1.hw_minor_2 , packet1.sw_major_2 , packet1.sw_minor_2 , packet1.minutes_2 , packet1.hours_2 , packet1.pwm_lost_count_2 );
    mavlink_msg_device_info1_array_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("DEVICE_INFO1_ARRAY") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_DEVICE_INFO1_ARRAY) != NULL);
#endif
}

static void mavlink_test_device_info2_array(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_DEVICE_INFO2_ARRAY >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_device_info2_array_t packet_in = {
        17235,17339,17443,17547,17651,17755,41,108,175,242,53,120,187,254,65,132,199,10,77,144,211,22,89,156,223,34,101,168,235,46,113,180,247,58,125,192
    };
    mavlink_device_info2_array_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.hours_3 = packet_in.hours_3;
        packet1.pwm_lost_count_3 = packet_in.pwm_lost_count_3;
        packet1.hours_4 = packet_in.hours_4;
        packet1.pwm_lost_count_4 = packet_in.pwm_lost_count_4;
        packet1.hours_5 = packet_in.hours_5;
        packet1.pwm_lost_count_5 = packet_in.pwm_lost_count_5;
        packet1.type_3 = packet_in.type_3;
        packet1.year_3 = packet_in.year_3;
        packet1.month_3 = packet_in.month_3;
        packet1.day_3 = packet_in.day_3;
        packet1.number_3 = packet_in.number_3;
        packet1.hw_major_3 = packet_in.hw_major_3;
        packet1.hw_minor_3 = packet_in.hw_minor_3;
        packet1.sw_major_3 = packet_in.sw_major_3;
        packet1.sw_minor_3 = packet_in.sw_minor_3;
        packet1.minutes_3 = packet_in.minutes_3;
        packet1.type_4 = packet_in.type_4;
        packet1.year_4 = packet_in.year_4;
        packet1.month_4 = packet_in.month_4;
        packet1.day_4 = packet_in.day_4;
        packet1.number_4 = packet_in.number_4;
        packet1.hw_major_4 = packet_in.hw_major_4;
        packet1.hw_minor_4 = packet_in.hw_minor_4;
        packet1.sw_major_4 = packet_in.sw_major_4;
        packet1.sw_minor_4 = packet_in.sw_minor_4;
        packet1.minutes_4 = packet_in.minutes_4;
        packet1.type_5 = packet_in.type_5;
        packet1.year_5 = packet_in.year_5;
        packet1.month_5 = packet_in.month_5;
        packet1.day_5 = packet_in.day_5;
        packet1.number_5 = packet_in.number_5;
        packet1.hw_major_5 = packet_in.hw_major_5;
        packet1.hw_minor_5 = packet_in.hw_minor_5;
        packet1.sw_major_5 = packet_in.sw_major_5;
        packet1.sw_minor_5 = packet_in.sw_minor_5;
        packet1.minutes_5 = packet_in.minutes_5;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_DEVICE_INFO2_ARRAY_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_DEVICE_INFO2_ARRAY_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_device_info2_array_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_device_info2_array_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_device_info2_array_pack(system_id, component_id, &msg , packet1.type_3 , packet1.year_3 , packet1.month_3 , packet1.day_3 , packet1.number_3 , packet1.hw_major_3 , packet1.hw_minor_3 , packet1.sw_major_3 , packet1.sw_minor_3 , packet1.minutes_3 , packet1.hours_3 , packet1.pwm_lost_count_3 , packet1.type_4 , packet1.year_4 , packet1.month_4 , packet1.day_4 , packet1.number_4 , packet1.hw_major_4 , packet1.hw_minor_4 , packet1.sw_major_4 , packet1.sw_minor_4 , packet1.minutes_4 , packet1.hours_4 , packet1.pwm_lost_count_4 , packet1.type_5 , packet1.year_5 , packet1.month_5 , packet1.day_5 , packet1.number_5 , packet1.hw_major_5 , packet1.hw_minor_5 , packet1.sw_major_5 , packet1.sw_minor_5 , packet1.minutes_5 , packet1.hours_5 , packet1.pwm_lost_count_5 );
    mavlink_msg_device_info2_array_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_device_info2_array_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.type_3 , packet1.year_3 , packet1.month_3 , packet1.day_3 , packet1.number_3 , packet1.hw_major_3 , packet1.hw_minor_3 , packet1.sw_major_3 , packet1.sw_minor_3 , packet1.minutes_3 , packet1.hours_3 , packet1.pwm_lost_count_3 , packet1.type_4 , packet1.year_4 , packet1.month_4 , packet1.day_4 , packet1.number_4 , packet1.hw_major_4 , packet1.hw_minor_4 , packet1.sw_major_4 , packet1.sw_minor_4 , packet1.minutes_4 , packet1.hours_4 , packet1.pwm_lost_count_4 , packet1.type_5 , packet1.year_5 , packet1.month_5 , packet1.day_5 , packet1.number_5 , packet1.hw_major_5 , packet1.hw_minor_5 , packet1.sw_major_5 , packet1.sw_minor_5 , packet1.minutes_5 , packet1.hours_5 , packet1.pwm_lost_count_5 );
    mavlink_msg_device_info2_array_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_device_info2_array_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_device_info2_array_send(MAVLINK_COMM_1 , packet1.type_3 , packet1.year_3 , packet1.month_3 , packet1.day_3 , packet1.number_3 , packet1.hw_major_3 , packet1.hw_minor_3 , packet1.sw_major_3 , packet1.sw_minor_3 , packet1.minutes_3 , packet1.hours_3 , packet1.pwm_lost_count_3 , packet1.type_4 , packet1.year_4 , packet1.month_4 , packet1.day_4 , packet1.number_4 , packet1.hw_major_4 , packet1.hw_minor_4 , packet1.sw_major_4 , packet1.sw_minor_4 , packet1.minutes_4 , packet1.hours_4 , packet1.pwm_lost_count_4 , packet1.type_5 , packet1.year_5 , packet1.month_5 , packet1.day_5 , packet1.number_5 , packet1.hw_major_5 , packet1.hw_minor_5 , packet1.sw_major_5 , packet1.sw_minor_5 , packet1.minutes_5 , packet1.hours_5 , packet1.pwm_lost_count_5 );
    mavlink_msg_device_info2_array_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("DEVICE_INFO2_ARRAY") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_DEVICE_INFO2_ARRAY) != NULL);
#endif
}

static void mavlink_test_single_radar_data(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_SINGLE_RADAR_DATA >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_single_radar_data_t packet_in = {
        17235,17339,17443
    };
    mavlink_single_radar_data_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.ground_distance = packet_in.ground_distance;
        packet1.forward_distance = packet_in.forward_distance;
        packet1.backward_distance = packet_in.backward_distance;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_SINGLE_RADAR_DATA_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_SINGLE_RADAR_DATA_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_single_radar_data_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_single_radar_data_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_single_radar_data_pack(system_id, component_id, &msg , packet1.ground_distance , packet1.forward_distance , packet1.backward_distance );
    mavlink_msg_single_radar_data_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_single_radar_data_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.ground_distance , packet1.forward_distance , packet1.backward_distance );
    mavlink_msg_single_radar_data_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_single_radar_data_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_single_radar_data_send(MAVLINK_COMM_1 , packet1.ground_distance , packet1.forward_distance , packet1.backward_distance );
    mavlink_msg_single_radar_data_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("SINGLE_RADAR_DATA") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_SINGLE_RADAR_DATA) != NULL);
#endif
}

static void mavlink_test_weight_calibration(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_WEIGHT_CALIBRATION >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_weight_calibration_t packet_in = {
        17235,{ 17339, 17340, 17341 },29,96,163,230,41,108
    };
    mavlink_weight_calibration_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.calibration_weight = packet_in.calibration_weight;
        packet1.led_control = packet_in.led_control;
        packet1.right_led_brightness = packet_in.right_led_brightness;
        packet1.left_led_brightness = packet_in.left_led_brightness;
        packet1.tare_calibration = packet_in.tare_calibration;
        packet1.weight_calibration = packet_in.weight_calibration;
        packet1.k_calibration = packet_in.k_calibration;
        
        mav_array_memcpy(packet1.k_values, packet_in.k_values, sizeof(uint16_t)*3);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_WEIGHT_CALIBRATION_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_WEIGHT_CALIBRATION_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_weight_calibration_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_weight_calibration_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_weight_calibration_pack(system_id, component_id, &msg , packet1.led_control , packet1.right_led_brightness , packet1.left_led_brightness , packet1.tare_calibration , packet1.weight_calibration , packet1.calibration_weight , packet1.k_calibration , packet1.k_values );
    mavlink_msg_weight_calibration_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_weight_calibration_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.led_control , packet1.right_led_brightness , packet1.left_led_brightness , packet1.tare_calibration , packet1.weight_calibration , packet1.calibration_weight , packet1.k_calibration , packet1.k_values );
    mavlink_msg_weight_calibration_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_weight_calibration_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_weight_calibration_send(MAVLINK_COMM_1 , packet1.led_control , packet1.right_led_brightness , packet1.left_led_brightness , packet1.tare_calibration , packet1.weight_calibration , packet1.calibration_weight , packet1.k_calibration , packet1.k_values );
    mavlink_msg_weight_calibration_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("WEIGHT_CALIBRATION") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_WEIGHT_CALIBRATION) != NULL);
#endif
}

static void mavlink_test_weigh_data_eft(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_WEIGH_DATA_EFT >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_weigh_data_eft_t packet_in = {
        17235,17339,17443,17547,17651,163,230,41,108,175,242,53,120,187,254,65,132,199,10,77,144
    };
    mavlink_weigh_data_eft_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.weight = packet_in.weight;
        packet1.hours = packet_in.hours;
        packet1.sensor1_k = packet_in.sensor1_k;
        packet1.sensor2_k = packet_in.sensor2_k;
        packet1.sensor3_k = packet_in.sensor3_k;
        packet1.liquid_level = packet_in.liquid_level;
        packet1.sensor_status = packet_in.sensor_status;
        packet1.right_led_temp = packet_in.right_led_temp;
        packet1.left_led_temp = packet_in.left_led_temp;
        packet1.led_status = packet_in.led_status;
        packet1.battery_temp = packet_in.battery_temp;
        packet1.device_type = packet_in.device_type;
        packet1.year = packet_in.year;
        packet1.month = packet_in.month;
        packet1.day = packet_in.day;
        packet1.number = packet_in.number;
        packet1.hw_major = packet_in.hw_major;
        packet1.hw_minor = packet_in.hw_minor;
        packet1.sw_major = packet_in.sw_major;
        packet1.sw_minor = packet_in.sw_minor;
        packet1.minutes = packet_in.minutes;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_WEIGH_DATA_EFT_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_WEIGH_DATA_EFT_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_weigh_data_eft_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_weigh_data_eft_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_weigh_data_eft_pack(system_id, component_id, &msg , packet1.liquid_level , packet1.sensor_status , packet1.weight , packet1.right_led_temp , packet1.left_led_temp , packet1.led_status , packet1.battery_temp , packet1.device_type , packet1.year , packet1.month , packet1.day , packet1.number , packet1.hw_major , packet1.hw_minor , packet1.sw_major , packet1.sw_minor , packet1.minutes , packet1.hours , packet1.sensor1_k , packet1.sensor2_k , packet1.sensor3_k );
    mavlink_msg_weigh_data_eft_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_weigh_data_eft_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.liquid_level , packet1.sensor_status , packet1.weight , packet1.right_led_temp , packet1.left_led_temp , packet1.led_status , packet1.battery_temp , packet1.device_type , packet1.year , packet1.month , packet1.day , packet1.number , packet1.hw_major , packet1.hw_minor , packet1.sw_major , packet1.sw_minor , packet1.minutes , packet1.hours , packet1.sensor1_k , packet1.sensor2_k , packet1.sensor3_k );
    mavlink_msg_weigh_data_eft_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_weigh_data_eft_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_weigh_data_eft_send(MAVLINK_COMM_1 , packet1.liquid_level , packet1.sensor_status , packet1.weight , packet1.right_led_temp , packet1.left_led_temp , packet1.led_status , packet1.battery_temp , packet1.device_type , packet1.year , packet1.month , packet1.day , packet1.number , packet1.hw_major , packet1.hw_minor , packet1.sw_major , packet1.sw_minor , packet1.minutes , packet1.hours , packet1.sensor1_k , packet1.sensor2_k , packet1.sensor3_k );
    mavlink_msg_weigh_data_eft_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("WEIGH_DATA_EFT") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_WEIGH_DATA_EFT) != NULL);
#endif
}

static void mavlink_test_pump_calibration_cmd(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_PUMP_CALIBRATION_CMD >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_pump_calibration_cmd_t packet_in = {
        5
    };
    mavlink_pump_calibration_cmd_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.calibration_cmd = packet_in.calibration_cmd;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_PUMP_CALIBRATION_CMD_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_PUMP_CALIBRATION_CMD_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_pump_calibration_cmd_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_pump_calibration_cmd_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_pump_calibration_cmd_pack(system_id, component_id, &msg , packet1.calibration_cmd );
    mavlink_msg_pump_calibration_cmd_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_pump_calibration_cmd_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.calibration_cmd );
    mavlink_msg_pump_calibration_cmd_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_pump_calibration_cmd_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_pump_calibration_cmd_send(MAVLINK_COMM_1 , packet1.calibration_cmd );
    mavlink_msg_pump_calibration_cmd_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("PUMP_CALIBRATION_CMD") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_PUMP_CALIBRATION_CMD) != NULL);
#endif
}

static void mavlink_test_pump_calibration_results(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_PUMP_CALIBRATION_RESULTS >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_pump_calibration_results_t packet_in = {
        { 17235, 17236, 17237, 17238, 17239, 17240 },{ 17859, 17860, 17861, 17862, 17863, 17864 },{ 18483, 18484, 18485, 18486, 18487, 18488 }
    };
    mavlink_pump_calibration_results_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        
        mav_array_memcpy(packet1.pwm, packet_in.pwm, sizeof(uint16_t)*6);
        mav_array_memcpy(packet1.speed, packet_in.speed, sizeof(uint16_t)*6);
        mav_array_memcpy(packet1.flow, packet_in.flow, sizeof(uint16_t)*6);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_PUMP_CALIBRATION_RESULTS_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_PUMP_CALIBRATION_RESULTS_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_pump_calibration_results_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_pump_calibration_results_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_pump_calibration_results_pack(system_id, component_id, &msg , packet1.pwm , packet1.speed , packet1.flow );
    mavlink_msg_pump_calibration_results_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_pump_calibration_results_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.pwm , packet1.speed , packet1.flow );
    mavlink_msg_pump_calibration_results_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_pump_calibration_results_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_pump_calibration_results_send(MAVLINK_COMM_1 , packet1.pwm , packet1.speed , packet1.flow );
    mavlink_msg_pump_calibration_results_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("PUMP_CALIBRATION_RESULTS") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_PUMP_CALIBRATION_RESULTS) != NULL);
#endif
}

static void mavlink_test_spray_system_params(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_SPRAY_SYSTEM_PARAMS >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_spray_system_params_t packet_in = {
        17235,17339
    };
    mavlink_spray_system_params_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.spray_rate = packet_in.spray_rate;
        packet1.spray_width = packet_in.spray_width;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_SPRAY_SYSTEM_PARAMS_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_SPRAY_SYSTEM_PARAMS_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_spray_system_params_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_spray_system_params_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_spray_system_params_pack(system_id, component_id, &msg , packet1.spray_rate , packet1.spray_width );
    mavlink_msg_spray_system_params_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_spray_system_params_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.spray_rate , packet1.spray_width );
    mavlink_msg_spray_system_params_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_spray_system_params_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_spray_system_params_send(MAVLINK_COMM_1 , packet1.spray_rate , packet1.spray_width );
    mavlink_msg_spray_system_params_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("SPRAY_SYSTEM_PARAMS") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_SPRAY_SYSTEM_PARAMS) != NULL);
#endif
}

static void mavlink_test_battery_data(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_BATTERY_DATA >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_battery_data_t packet_in = {
        963497464,17443,17547,17651,17755
    };
    mavlink_battery_data_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.current = packet_in.current;
        packet1.voltage = packet_in.voltage;
        packet1.cell_temp = packet_in.cell_temp;
        packet1.mosfet_temp = packet_in.mosfet_temp;
        packet1.capacity_percent = packet_in.capacity_percent;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_BATTERY_DATA_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_BATTERY_DATA_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_battery_data_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_battery_data_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_battery_data_pack(system_id, component_id, &msg , packet1.voltage , packet1.current , packet1.cell_temp , packet1.mosfet_temp , packet1.capacity_percent );
    mavlink_msg_battery_data_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_battery_data_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.voltage , packet1.current , packet1.cell_temp , packet1.mosfet_temp , packet1.capacity_percent );
    mavlink_msg_battery_data_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_battery_data_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_battery_data_send(MAVLINK_COMM_1 , packet1.voltage , packet1.current , packet1.cell_temp , packet1.mosfet_temp , packet1.capacity_percent );
    mavlink_msg_battery_data_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("BATTERY_DATA") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_BATTERY_DATA) != NULL);
#endif
}

static void mavlink_test_spreader_control(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_SPREADER_CONTROL >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_spreader_control_t packet_in = {
        17235,17339,17,84,151,218,{ 29, 30, 31 },230
    };
    mavlink_spreader_control_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.spreader_motor_pwm = packet_in.spreader_motor_pwm;
        packet1.spreader_valve_pwm = packet_in.spreader_valve_pwm;
        packet1.motor_control_cmd = packet_in.motor_control_cmd;
        packet1.signal_source_cmd = packet_in.signal_source_cmd;
        packet1.spreader_signal_source = packet_in.spreader_signal_source;
        packet1.alarm_config_cmd = packet_in.alarm_config_cmd;
        packet1.spreader_factory_reset = packet_in.spreader_factory_reset;
        
        mav_array_memcpy(packet1.spreader_alarm_config, packet_in.spreader_alarm_config, sizeof(uint8_t)*3);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_SPREADER_CONTROL_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_SPREADER_CONTROL_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_spreader_control_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_spreader_control_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_spreader_control_pack(system_id, component_id, &msg , packet1.motor_control_cmd , packet1.spreader_motor_pwm , packet1.spreader_valve_pwm , packet1.signal_source_cmd , packet1.spreader_signal_source , packet1.alarm_config_cmd , packet1.spreader_alarm_config , packet1.spreader_factory_reset );
    mavlink_msg_spreader_control_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_spreader_control_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.motor_control_cmd , packet1.spreader_motor_pwm , packet1.spreader_valve_pwm , packet1.signal_source_cmd , packet1.spreader_signal_source , packet1.alarm_config_cmd , packet1.spreader_alarm_config , packet1.spreader_factory_reset );
    mavlink_msg_spreader_control_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_spreader_control_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_spreader_control_send(MAVLINK_COMM_1 , packet1.motor_control_cmd , packet1.spreader_motor_pwm , packet1.spreader_valve_pwm , packet1.signal_source_cmd , packet1.spreader_signal_source , packet1.alarm_config_cmd , packet1.spreader_alarm_config , packet1.spreader_factory_reset );
    mavlink_msg_spreader_control_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("SPREADER_CONTROL") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_SPREADER_CONTROL) != NULL);
#endif
}

static void mavlink_test_spreader_status(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_SPREADER_STATUS >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_spreader_status_t packet_in = {
        17235,17339,17443,151,218,29,96,163,230,41,108,175
    };
    mavlink_spreader_status_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.spreader_can_baudrate = packet_in.spreader_can_baudrate;
        packet1.spreader_sequence = packet_in.spreader_sequence;
        packet1.spreader_firmware_version = packet_in.spreader_firmware_version;
        packet1.spreader_servo_angle = packet_in.spreader_servo_angle;
        packet1.spreader_sensor_status = packet_in.spreader_sensor_status;
        packet1.spreader_can_enable = packet_in.spreader_can_enable;
        packet1.spreader_speed = packet_in.spreader_speed;
        packet1.spreader_function_status = packet_in.spreader_function_status;
        packet1.spreader_life_signal = packet_in.spreader_life_signal;
        packet1.spreader_year = packet_in.spreader_year;
        packet1.spreader_month = packet_in.spreader_month;
        packet1.spreader_day = packet_in.spreader_day;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_SPREADER_STATUS_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_SPREADER_STATUS_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_spreader_status_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_spreader_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_spreader_status_pack(system_id, component_id, &msg , packet1.spreader_servo_angle , packet1.spreader_sensor_status , packet1.spreader_can_enable , packet1.spreader_can_baudrate , packet1.spreader_speed , packet1.spreader_function_status , packet1.spreader_life_signal , packet1.spreader_year , packet1.spreader_month , packet1.spreader_day , packet1.spreader_sequence , packet1.spreader_firmware_version );
    mavlink_msg_spreader_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_spreader_status_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.spreader_servo_angle , packet1.spreader_sensor_status , packet1.spreader_can_enable , packet1.spreader_can_baudrate , packet1.spreader_speed , packet1.spreader_function_status , packet1.spreader_life_signal , packet1.spreader_year , packet1.spreader_month , packet1.spreader_day , packet1.spreader_sequence , packet1.spreader_firmware_version );
    mavlink_msg_spreader_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_spreader_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_spreader_status_send(MAVLINK_COMM_1 , packet1.spreader_servo_angle , packet1.spreader_sensor_status , packet1.spreader_can_enable , packet1.spreader_can_baudrate , packet1.spreader_speed , packet1.spreader_function_status , packet1.spreader_life_signal , packet1.spreader_year , packet1.spreader_month , packet1.spreader_day , packet1.spreader_sequence , packet1.spreader_firmware_version );
    mavlink_msg_spreader_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("SPREADER_STATUS") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_SPREADER_STATUS) != NULL);
#endif
}

static void mavlink_test_spreader_calibration_results(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_SPREADER_CALIBRATION_RESULTS >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_spreader_calibration_results_t packet_in = {
        { 17235, 17236, 17237, 17238, 17239, 17240 },{ 17859, 17860, 17861, 17862, 17863, 17864 },{ 18483, 18484, 18485, 18486, 18487, 18488 }
    };
    mavlink_spreader_calibration_results_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        
        mav_array_memcpy(packet1.pwm, packet_in.pwm, sizeof(uint16_t)*6);
        mav_array_memcpy(packet1.angle, packet_in.angle, sizeof(uint16_t)*6);
        mav_array_memcpy(packet1.flow, packet_in.flow, sizeof(uint16_t)*6);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_SPREADER_CALIBRATION_RESULTS_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_SPREADER_CALIBRATION_RESULTS_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_spreader_calibration_results_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_spreader_calibration_results_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_spreader_calibration_results_pack(system_id, component_id, &msg , packet1.pwm , packet1.angle , packet1.flow );
    mavlink_msg_spreader_calibration_results_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_spreader_calibration_results_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.pwm , packet1.angle , packet1.flow );
    mavlink_msg_spreader_calibration_results_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_spreader_calibration_results_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_spreader_calibration_results_send(MAVLINK_COMM_1 , packet1.pwm , packet1.angle , packet1.flow );
    mavlink_msg_spreader_calibration_results_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("SPREADER_CALIBRATION_RESULTS") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_SPREADER_CALIBRATION_RESULTS) != NULL);
#endif
}

static void mavlink_test_multi_radar_data(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_MULTI_RADAR_DATA >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_multi_radar_data_t packet_in = {
        17235,17339,17443
    };
    mavlink_multi_radar_data_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.ground_distance = packet_in.ground_distance;
        packet1.forward_distance = packet_in.forward_distance;
        packet1.backward_distance = packet_in.backward_distance;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_MULTI_RADAR_DATA_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_MULTI_RADAR_DATA_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_multi_radar_data_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_multi_radar_data_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_multi_radar_data_pack(system_id, component_id, &msg , packet1.ground_distance , packet1.forward_distance , packet1.backward_distance );
    mavlink_msg_multi_radar_data_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_multi_radar_data_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.ground_distance , packet1.forward_distance , packet1.backward_distance );
    mavlink_msg_multi_radar_data_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_multi_radar_data_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_multi_radar_data_send(MAVLINK_COMM_1 , packet1.ground_distance , packet1.forward_distance , packet1.backward_distance );
    mavlink_msg_multi_radar_data_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("MULTI_RADAR_DATA") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_MULTI_RADAR_DATA) != NULL);
#endif
}

static void mavlink_test_fmu_pmu_uart_message(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_FMU_PMU_UART_MESSAGE >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_fmu_pmu_uart_message_t packet_in = {
        963497464,17443,17547,17651,17755,17859,{ 17963, 17964, 17965 },18275,18379,77,144,211,22,89,156,223,34,101,168,235,46,{ 113, 114, 115 },58,125
    };
    mavlink_fmu_pmu_uart_message_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.nozzle_control = packet_in.nozzle_control;
        packet1.pump_control = packet_in.pump_control;
        packet1.horizontal_speed = packet_in.horizontal_speed;
        packet1.spray_rate = packet_in.spray_rate;
        packet1.spray_width = packet_in.spray_width;
        packet1.calibration_weight = packet_in.calibration_weight;
        packet1.spreader_motor_pwm = packet_in.spreader_motor_pwm;
        packet1.spreader_valve_pwm = packet_in.spreader_valve_pwm;
        packet1.control_mode = packet_in.control_mode;
        packet1.pump_calibration_cmd = packet_in.pump_calibration_cmd;
        packet1.led_control_cmd = packet_in.led_control_cmd;
        packet1.led_brightness_right = packet_in.led_brightness_right;
        packet1.led_brightness_left = packet_in.led_brightness_left;
        packet1.tare_calibration_cmd = packet_in.tare_calibration_cmd;
        packet1.weight_calibration_cmd = packet_in.weight_calibration_cmd;
        packet1.k_value_calibration_cmd = packet_in.k_value_calibration_cmd;
        packet1.spreader_control_cmd = packet_in.spreader_control_cmd;
        packet1.signal_source_cmd = packet_in.signal_source_cmd;
        packet1.signal_source = packet_in.signal_source;
        packet1.alarm_config_cmd = packet_in.alarm_config_cmd;
        packet1.factory_reset_cmd = packet_in.factory_reset_cmd;
        packet1.spray_spreader_mode = packet_in.spray_spreader_mode;
        
        mav_array_memcpy(packet1.k_values, packet_in.k_values, sizeof(uint16_t)*3);
        mav_array_memcpy(packet1.alarm_config, packet_in.alarm_config, sizeof(uint8_t)*3);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_FMU_PMU_UART_MESSAGE_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_FMU_PMU_UART_MESSAGE_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_fmu_pmu_uart_message_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_fmu_pmu_uart_message_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_fmu_pmu_uart_message_pack(system_id, component_id, &msg , packet1.pump_control , packet1.nozzle_control , packet1.control_mode , packet1.horizontal_speed , packet1.spray_rate , packet1.spray_width , packet1.pump_calibration_cmd , packet1.led_control_cmd , packet1.led_brightness_right , packet1.led_brightness_left , packet1.tare_calibration_cmd , packet1.weight_calibration_cmd , packet1.calibration_weight , packet1.k_value_calibration_cmd , packet1.k_values , packet1.spreader_control_cmd , packet1.spreader_motor_pwm , packet1.spreader_valve_pwm , packet1.signal_source_cmd , packet1.signal_source , packet1.alarm_config_cmd , packet1.alarm_config , packet1.factory_reset_cmd , packet1.spray_spreader_mode );
    mavlink_msg_fmu_pmu_uart_message_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_fmu_pmu_uart_message_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.pump_control , packet1.nozzle_control , packet1.control_mode , packet1.horizontal_speed , packet1.spray_rate , packet1.spray_width , packet1.pump_calibration_cmd , packet1.led_control_cmd , packet1.led_brightness_right , packet1.led_brightness_left , packet1.tare_calibration_cmd , packet1.weight_calibration_cmd , packet1.calibration_weight , packet1.k_value_calibration_cmd , packet1.k_values , packet1.spreader_control_cmd , packet1.spreader_motor_pwm , packet1.spreader_valve_pwm , packet1.signal_source_cmd , packet1.signal_source , packet1.alarm_config_cmd , packet1.alarm_config , packet1.factory_reset_cmd , packet1.spray_spreader_mode );
    mavlink_msg_fmu_pmu_uart_message_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_fmu_pmu_uart_message_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_fmu_pmu_uart_message_send(MAVLINK_COMM_1 , packet1.pump_control , packet1.nozzle_control , packet1.control_mode , packet1.horizontal_speed , packet1.spray_rate , packet1.spray_width , packet1.pump_calibration_cmd , packet1.led_control_cmd , packet1.led_brightness_right , packet1.led_brightness_left , packet1.tare_calibration_cmd , packet1.weight_calibration_cmd , packet1.calibration_weight , packet1.k_value_calibration_cmd , packet1.k_values , packet1.spreader_control_cmd , packet1.spreader_motor_pwm , packet1.spreader_valve_pwm , packet1.signal_source_cmd , packet1.signal_source , packet1.alarm_config_cmd , packet1.alarm_config , packet1.factory_reset_cmd , packet1.spray_spreader_mode );
    mavlink_msg_fmu_pmu_uart_message_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("FMU_PMU_UART_MESSAGE") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_FMU_PMU_UART_MESSAGE) != NULL);
#endif
}

static void mavlink_test_mav_framing_override_cmd(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_MAV_FRAMING_OVERRIDE_CMD >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_mav_framing_override_cmd_t packet_in = {
        17235,139,206
    };
    mavlink_mav_framing_override_cmd_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.crc = packet_in.crc;
        packet1.cmd = packet_in.cmd;
        packet1.magic = packet_in.magic;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_MAV_FRAMING_OVERRIDE_CMD_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_MAV_FRAMING_OVERRIDE_CMD_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mav_framing_override_cmd_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_mav_framing_override_cmd_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mav_framing_override_cmd_pack(system_id, component_id, &msg , packet1.cmd , packet1.magic , packet1.crc );
    mavlink_msg_mav_framing_override_cmd_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mav_framing_override_cmd_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.cmd , packet1.magic , packet1.crc );
    mavlink_msg_mav_framing_override_cmd_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_mav_framing_override_cmd_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mav_framing_override_cmd_send(MAVLINK_COMM_1 , packet1.cmd , packet1.magic , packet1.crc );
    mavlink_msg_mav_framing_override_cmd_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("MAV_FRAMING_OVERRIDE_CMD") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_MAV_FRAMING_OVERRIDE_CMD) != NULL);
#endif
}

static void mavlink_test_eft_rid_config_request(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_EFT_RID_CONFIG_REQUEST >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_eft_rid_config_request_t packet_in = {
        5,72,139,206
    };
    mavlink_eft_rid_config_request_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.target_system = packet_in.target_system;
        packet1.target_component = packet_in.target_component;
        packet1.seq = packet_in.seq;
        packet1.type = packet_in.type;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_EFT_RID_CONFIG_REQUEST_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_EFT_RID_CONFIG_REQUEST_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_eft_rid_config_request_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_eft_rid_config_request_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_eft_rid_config_request_pack(system_id, component_id, &msg , packet1.target_system , packet1.target_component , packet1.seq , packet1.type );
    mavlink_msg_eft_rid_config_request_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_eft_rid_config_request_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target_system , packet1.target_component , packet1.seq , packet1.type );
    mavlink_msg_eft_rid_config_request_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_eft_rid_config_request_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_eft_rid_config_request_send(MAVLINK_COMM_1 , packet1.target_system , packet1.target_component , packet1.seq , packet1.type );
    mavlink_msg_eft_rid_config_request_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("EFT_RID_CONFIG_REQUEST") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_EFT_RID_CONFIG_REQUEST) != NULL);
#endif
}

static void mavlink_test_eft_rid_config_status(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_EFT_RID_CONFIG_STATUS >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_eft_rid_config_status_t packet_in = {
        963497464,963497672,73.0,963498088,18067,18171,65,132,199,10,77,144,211,22,89,"DEFGHIJKLMNOPQRSTUV",216,"YZABCDEFGHIJKLMNOPQ",87,"TUVWXYZABCDEFGHIJKLMNO",159,"RSTUVWXYZABCDEFGHIJKLMNOPQRSTUV"
    };
    mavlink_eft_rid_config_status_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.operator_latitude = packet_in.operator_latitude;
        packet1.operator_longitude = packet_in.operator_longitude;
        packet1.operator_altitude_geo = packet_in.operator_altitude_geo;
        packet1.status_flags = packet_in.status_flags;
        packet1.arm_status_age_ms = packet_in.arm_status_age_ms;
        packet1.system_age_ms = packet_in.system_age_ms;
        packet1.target_system = packet_in.target_system;
        packet1.target_component = packet_in.target_component;
        packet1.seq = packet_in.seq;
        packet1.did_enable = packet_in.did_enable;
        packet1.did_mavport = packet_in.did_mavport;
        packet1.did_options = packet_in.did_options;
        packet1.did_can_driver = packet_in.did_can_driver;
        packet1.ua_type = packet_in.ua_type;
        packet1.id_type = packet_in.id_type;
        packet1.op_id_type = packet_in.op_id_type;
        packet1.desc_type = packet_in.desc_type;
        packet1.arm_status = packet_in.arm_status;
        
        mav_array_memcpy(packet1.uas_id, packet_in.uas_id, sizeof(char)*20);
        mav_array_memcpy(packet1.operator_id, packet_in.operator_id, sizeof(char)*20);
        mav_array_memcpy(packet1.self_desc, packet_in.self_desc, sizeof(char)*23);
        mav_array_memcpy(packet1.arm_error, packet_in.arm_error, sizeof(char)*32);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_EFT_RID_CONFIG_STATUS_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_EFT_RID_CONFIG_STATUS_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_eft_rid_config_status_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_eft_rid_config_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_eft_rid_config_status_pack(system_id, component_id, &msg , packet1.target_system , packet1.target_component , packet1.seq , packet1.did_enable , packet1.did_mavport , packet1.did_options , packet1.did_can_driver , packet1.ua_type , packet1.id_type , packet1.uas_id , packet1.op_id_type , packet1.operator_id , packet1.desc_type , packet1.self_desc , packet1.operator_latitude , packet1.operator_longitude , packet1.operator_altitude_geo , packet1.arm_status , packet1.arm_error , packet1.arm_status_age_ms , packet1.system_age_ms , packet1.status_flags );
    mavlink_msg_eft_rid_config_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_eft_rid_config_status_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target_system , packet1.target_component , packet1.seq , packet1.did_enable , packet1.did_mavport , packet1.did_options , packet1.did_can_driver , packet1.ua_type , packet1.id_type , packet1.uas_id , packet1.op_id_type , packet1.operator_id , packet1.desc_type , packet1.self_desc , packet1.operator_latitude , packet1.operator_longitude , packet1.operator_altitude_geo , packet1.arm_status , packet1.arm_error , packet1.arm_status_age_ms , packet1.system_age_ms , packet1.status_flags );
    mavlink_msg_eft_rid_config_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_eft_rid_config_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_eft_rid_config_status_send(MAVLINK_COMM_1 , packet1.target_system , packet1.target_component , packet1.seq , packet1.did_enable , packet1.did_mavport , packet1.did_options , packet1.did_can_driver , packet1.ua_type , packet1.id_type , packet1.uas_id , packet1.op_id_type , packet1.operator_id , packet1.desc_type , packet1.self_desc , packet1.operator_latitude , packet1.operator_longitude , packet1.operator_altitude_geo , packet1.arm_status , packet1.arm_error , packet1.arm_status_age_ms , packet1.system_age_ms , packet1.status_flags );
    mavlink_msg_eft_rid_config_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("EFT_RID_CONFIG_STATUS") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_EFT_RID_CONFIG_STATUS) != NULL);
#endif
}

static void mavlink_test_uom_arm_status(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_UOM_ARM_STATUS >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_uom_arm_status_t packet_in = {
        17235
    };
    mavlink_uom_arm_status_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.status_code = packet_in.status_code;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_UOM_ARM_STATUS_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_UOM_ARM_STATUS_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_uom_arm_status_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_uom_arm_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_uom_arm_status_pack(system_id, component_id, &msg , packet1.status_code );
    mavlink_msg_uom_arm_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_uom_arm_status_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.status_code );
    mavlink_msg_uom_arm_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_uom_arm_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_uom_arm_status_send(MAVLINK_COMM_1 , packet1.status_code );
    mavlink_msg_uom_arm_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("UOM_ARM_STATUS") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_UOM_ARM_STATUS) != NULL);
#endif
}

static void mavlink_test_uom_fc_status(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_UOM_FC_STATUS >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_uom_fc_status_t packet_in = {
        17235,17339,17,84
    };
    mavlink_uom_fc_status_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.status_code = packet_in.status_code;
        packet1.status_age_s = packet_in.status_age_s;
        packet1.allow_arm = packet_in.allow_arm;
        packet1.is_armed = packet_in.is_armed;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_UOM_FC_STATUS_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_UOM_FC_STATUS_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_uom_fc_status_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_uom_fc_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_uom_fc_status_pack(system_id, component_id, &msg , packet1.status_code , packet1.status_age_s , packet1.allow_arm , packet1.is_armed );
    mavlink_msg_uom_fc_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_uom_fc_status_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.status_code , packet1.status_age_s , packet1.allow_arm , packet1.is_armed );
    mavlink_msg_uom_fc_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_uom_fc_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_uom_fc_status_send(MAVLINK_COMM_1 , packet1.status_code , packet1.status_age_s , packet1.allow_arm , packet1.is_armed );
    mavlink_msg_uom_fc_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("UOM_FC_STATUS") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_UOM_FC_STATUS) != NULL);
#endif
}

static void mavlink_test_uom_operator_id(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_UOM_OPERATOR_ID >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_uom_operator_id_t packet_in = {
        "ABCDEFGHIJKLMNOPQRS",65
    };
    mavlink_uom_operator_id_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.operator_id_type = packet_in.operator_id_type;
        
        mav_array_memcpy(packet1.operator_id, packet_in.operator_id, sizeof(char)*20);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_UOM_OPERATOR_ID_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_UOM_OPERATOR_ID_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_uom_operator_id_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_uom_operator_id_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_uom_operator_id_pack(system_id, component_id, &msg , packet1.operator_id , packet1.operator_id_type );
    mavlink_msg_uom_operator_id_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_uom_operator_id_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.operator_id , packet1.operator_id_type );
    mavlink_msg_uom_operator_id_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_uom_operator_id_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_uom_operator_id_send(MAVLINK_COMM_1 , packet1.operator_id , packet1.operator_id_type );
    mavlink_msg_uom_operator_id_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("UOM_OPERATOR_ID") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_UOM_OPERATOR_ID) != NULL);
#endif
}

static void mavlink_test_eft(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
    mavlink_test_device_status_array(system_id, component_id, last_msg);
    mavlink_test_device_info1_array(system_id, component_id, last_msg);
    mavlink_test_device_info2_array(system_id, component_id, last_msg);
    mavlink_test_single_radar_data(system_id, component_id, last_msg);
    mavlink_test_weight_calibration(system_id, component_id, last_msg);
    mavlink_test_weigh_data_eft(system_id, component_id, last_msg);
    mavlink_test_pump_calibration_cmd(system_id, component_id, last_msg);
    mavlink_test_pump_calibration_results(system_id, component_id, last_msg);
    mavlink_test_spray_system_params(system_id, component_id, last_msg);
    mavlink_test_battery_data(system_id, component_id, last_msg);
    mavlink_test_spreader_control(system_id, component_id, last_msg);
    mavlink_test_spreader_status(system_id, component_id, last_msg);
    mavlink_test_spreader_calibration_results(system_id, component_id, last_msg);
    mavlink_test_multi_radar_data(system_id, component_id, last_msg);
    mavlink_test_fmu_pmu_uart_message(system_id, component_id, last_msg);
    mavlink_test_mav_framing_override_cmd(system_id, component_id, last_msg);
    mavlink_test_eft_rid_config_request(system_id, component_id, last_msg);
    mavlink_test_eft_rid_config_status(system_id, component_id, last_msg);
    mavlink_test_uom_arm_status(system_id, component_id, last_msg);
    mavlink_test_uom_fc_status(system_id, component_id, last_msg);
    mavlink_test_uom_operator_id(system_id, component_id, last_msg);
}

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // EFT_TESTSUITE_H
