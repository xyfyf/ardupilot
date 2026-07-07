/// @file	GCS_MAVLink.h
/// @brief	One size fits all header for MAVLink integration.
#pragma once

#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_Networking/AP_Networking_Config.h>

// we have separate helpers disabled to make it possible
// to select MAVLink 1.0 in the arduino GUI build
#define MAVLINK_SEPARATE_HELPERS
#define MAVLINK_NO_CONVERSION_HELPERS

#define MAVLINK_SEND_UART_BYTES(chan, buf, len) comm_send_buffer(chan, buf, len)

#define MAVLINK_START_UART_SEND(chan, size) comm_send_lock(chan, size)
#define MAVLINK_END_UART_SEND(chan, size) comm_send_unlock(chan)

#if HAL_PROGRAM_SIZE_LIMIT_KB > 1024
// allow 8 telemetry ports, allowing for extra networking or CAN ports
#define MAVLINK_COMM_NUM_BUFFERS 8
#else
// allow five telemetry ports
#define MAVLINK_COMM_NUM_BUFFERS 5
#endif

#define MAVLINK_GET_CHANNEL_BUFFER 1
#define MAVLINK_GET_CHANNEL_STATUS 1

/*
  The MAVLink protocol code generator does its own alignment, so
  alignment cast warnings can be ignored
 */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wcast-align"

#if defined(__GNUC__) && __GNUC__ >= 9
#pragma GCC diagnostic ignored "-Waddress-of-packed-member"
#endif

#include "include/mavlink/v2.0/all/version.h"

#define MAVLINK_MAX_PAYLOAD_LEN 255

#include "include/mavlink/v2.0/mavlink_types.h"

/// MAVLink streams used for each telemetry port
extern AP_HAL::UARTDriver	*mavlink_comm_port[MAVLINK_COMM_NUM_BUFFERS];
extern bool gcs_alternative_active[MAVLINK_COMM_NUM_BUFFERS];

/// MAVLink system definition
extern mavlink_system_t mavlink_system;

/// ArduPilot custom: runtime override of the outgoing MAVLink2 frame format.
/// Set via the MAV_FRAMING_OVERRIDE_CMD (msgid 516) message. See mavlink_helpers.h.
/// mav_tx_magic_override: 0 = default 0xFD start byte, otherwise the byte to use on all channels (e.g. 0xEF).
/// mav_tx_magic_override_chan[]: per-channel default magic when mav_tx_magic_override is zero.
/// mav_tx_crc_override_enable: 0 = normal computed CRC, non-zero = force the CRC field.
/// mav_tx_crc_override_value: 16-bit CRC value used when the override is enabled.
extern uint8_t mav_tx_magic_override;
extern uint8_t mav_tx_magic_override_chan[MAVLINK_COMM_NUM_BUFFERS];
extern uint8_t mav_tx_crc_override_enable;
extern uint16_t mav_tx_crc_override_value;
/// Debug counter: number of outgoing frames that actually had the override applied.
/// If this stays 0 after enabling the override, the patched helper is not compiled in.
extern volatile uint32_t mav_tx_override_hits;

void mavlink_set_channel_magic_override(uint8_t chan, uint8_t magic);

/// Sanity check MAVLink channel
///
/// @param chan		Channel to send to
static inline bool valid_channel(mavlink_channel_t chan)
{
    return static_cast<int>(chan) < MAVLINK_COMM_NUM_BUFFERS;
}

mavlink_message_t* mavlink_get_channel_buffer(uint8_t chan);
mavlink_status_t* mavlink_get_channel_status(uint8_t chan);

void comm_send_buffer(mavlink_channel_t chan, const uint8_t *buf, uint8_t len);

/// Check for available transmit space on the nominated MAVLink channel
///
/// @param chan		Channel to check
/// @returns		Number of bytes available
uint16_t comm_get_txspace(mavlink_channel_t chan);

#define MAVLINK_USE_CONVENIENCE_FUNCTIONS

#pragma GCC diagnostic push
// mavlink relies on strncpy() supporting deliberate truncation
#if !defined(__clang__)  // avoid -Wunknown-warning-option
#pragma GCC diagnostic ignored "-Wstringop-truncation"
#endif  // clang
#include "include/mavlink/v2.0/all/mavlink.h"
#pragma GCC diagnostic pop

// lock and unlock a channel, for multi-threaded mavlink send
void comm_send_lock(mavlink_channel_t chan, uint16_t size);
void comm_send_unlock(mavlink_channel_t chan);
HAL_Semaphore &comm_chan_lock(mavlink_channel_t chan);

#pragma GCC diagnostic pop
