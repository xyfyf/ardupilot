#pragma once

#include "GCS_config.h"

#if AP_MAVLINK_LINK_CRYPTO_ENABLED

#include <stdint.h>
#include "include/mavlink/v2.0/mavlink_types.h"

/*
  Link-layer confidentiality for the raw MAVLink byte stream on a
  serial port (e.g. telemetry radio, USB). This wraps whole MAVLink2
  frames (header+payload+CRC+signature, exactly as handed to
  comm_send_buffer()) in an encrypted envelope:

    [MAGIC 1B][LEN 2B LE][NONCE 12B][CIPHERTEXT LEN bytes]

  Cipher: ChaCha20 (monocypher, IETF variant) as an unauthenticated
  stream cipher. This provides confidentiality against passive
  eavesdropping only; MAVLink's own CRC/signature (if enabled) still
  provide integrity/authenticity, unchanged, on the decrypted bytes.

  Key: derived once per boot from the vendor master secret + the
  autopilot's SN_FC serial number (same approach as
  AP_Logger_EFT_Crypto), so no extra key provisioning step is needed.

  Nonce: 7 random bytes chosen once at boot ("prefix"), 1 byte channel
  number, and a 4 byte per-channel monotonic frame counter. This keeps
  every (key, nonce) pair unique without needing any handshake.
*/
class GCS_MAVLink_Crypto {
public:
    static constexpr uint8_t MAGIC = 0xA5;
    static constexpr uint8_t NONCE_LEN = 12;
    static constexpr uint8_t ENVELOPE_OVERHEAD = 1 + 2 + NONCE_LEN; // magic+len+nonce
    static constexpr uint16_t MAX_FRAME_LEN = MAVLINK_MAX_PACKET_LEN;

    // fills key[32] with the derived link key (cached after first call)
    static void get_key(uint8_t key[32]);

    // fills prefix[7] with this boot's random nonce prefix (cached after first call)
    static void get_nonce_prefix(uint8_t prefix[7]);
};

// sender side: turns one plaintext MAVLink frame into one encrypted envelope
class GCS_MAVLink_Crypto_TX {
public:
    // encrypts buf[0..len) into out, returns envelope length or 0 on failure
    // (out must be at least len + GCS_MAVLink_Crypto::ENVELOPE_OVERHEAD bytes)
    uint16_t encrypt(uint8_t chan, const uint8_t *buf, uint16_t len,
                      uint8_t *out, uint16_t out_max);

private:
    uint32_t counter = 0;
};

// receiver side: reassembles envelopes from a raw byte stream and decrypts them
class GCS_MAVLink_Crypto_RX {
public:
    // feed one raw byte from the wire. Returns true once a full frame has
    // been decrypted; the plaintext is placed in plain_buf (size
    // GCS_MAVLink_Crypto::MAX_FRAME_LEN) with length in plain_len.
    bool feed(uint8_t b, uint8_t *plain_buf, uint16_t &plain_len);

private:
    enum class State : uint8_t {
        WAIT_MAGIC,
        LEN,
        NONCE,
        CIPHERTEXT,
    } state = State::WAIT_MAGIC;

    uint16_t len;
    uint8_t len_bytes[2];
    uint8_t len_pos = 0;

    uint8_t nonce[GCS_MAVLink_Crypto::NONCE_LEN];
    uint8_t nonce_pos = 0;

    uint8_t cipher[GCS_MAVLink_Crypto::MAX_FRAME_LEN];
    uint16_t cipher_pos = 0;

    void reset() {
        state = State::WAIT_MAGIC;
    }
};

#endif  // AP_MAVLINK_LINK_CRYPTO_ENABLED
