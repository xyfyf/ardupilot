#include "GCS_MAVLink_Crypto.h"

#if AP_MAVLINK_LINK_CRYPTO_ENABLED

#include <AP_CheckFirmware/monocypher.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>

#include <stdio.h>
#include <string.h>

extern const AP_HAL::HAL &hal;

/*
  32-byte vendor master secret, same idea as AP_LOGGER_EFT_MASTER_KEY:
  replace before production, keep in sync with the ground-side decrypt
  tool. Kept separate from the log encryption master key so a leak of
  one does not compromise the other.
*/
#ifndef AP_MAVLINK_LINK_CRYPTO_MASTER_KEY
#define AP_MAVLINK_LINK_CRYPTO_MASTER_KEY                                      \
    0xEF, 0x54, 0x4c, 0x49, 0x4e, 0x4b, 0x4b, 0x45, 0x59, 0x21, 0x10, 0x11,    \
    0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d,    \
    0x1e, 0x1f, 0x20, 0x21, 0x22, 0x23, 0x24, 0x25
#endif

static const uint8_t master_key[32] = {AP_MAVLINK_LINK_CRYPTO_MASTER_KEY};
static const char *KDF_SALT = "EFT-LINK-v1";

// same SN_FC1..SN_FC7 reconstruction as AP_Logger_EFT_Crypto::read_fc_sn()
static void read_fc_sn(char *dest, const size_t dest_size)
{
    if (dest == nullptr || dest_size == 0) {
        return;
    }
    size_t pos = 0;
    bool terminated = false;
    for (uint8_t i = 1; i <= 7 && !terminated && pos + 1 < dest_size; i++) {
        char pname[8];
        snprintf(pname, sizeof(pname), "SN_FC%u", (unsigned)i);
        enum ap_var_type ptype;
        AP_Param *vp = AP_Param::find(pname, &ptype);
        if (vp == nullptr || ptype != AP_PARAM_INT32) {
            break;
        }
        const uint32_t v = (uint32_t)((AP_Int32 *)vp)->get();
        const uint8_t bytes[3] = {
            (uint8_t)((v >> 16) & 0xFF),
            (uint8_t)((v >> 8) & 0xFF),
            (uint8_t)(v & 0xFF),
        };
        for (uint8_t b = 0; b < 3 && pos + 1 < dest_size; b++) {
            const uint8_t c = bytes[b];
            if (c == 0) {
                terminated = true;
                break;
            }
            dest[pos++] = (char)c;
        }
    }
    if (pos == 0) {
        for (const char *f = "NOSN"; *f != '\0' && pos + 1 < dest_size; f++) {
            dest[pos++] = *f;
        }
    }
    dest[pos] = '\0';
}

void GCS_MAVLink_Crypto::get_key(uint8_t key[32])
{
    static uint8_t cached_key[32];
    static bool have_key;
    if (!have_key) {
        char fc_sn[22];
        read_fc_sn(fc_sn, sizeof(fc_sn));

        crypto_blake2b_ctx ctx;
        crypto_blake2b_general_init(&ctx, 32, nullptr, 0);
        crypto_blake2b_update(&ctx, master_key, sizeof(master_key));
        crypto_blake2b_update(&ctx, (const uint8_t *)KDF_SALT, strlen(KDF_SALT));
        crypto_blake2b_update(&ctx, (const uint8_t *)fc_sn, strlen(fc_sn));
        crypto_blake2b_final(&ctx, cached_key);

        have_key = true;
    }
    memcpy(key, cached_key, 32);
}

void GCS_MAVLink_Crypto::get_nonce_prefix(uint8_t prefix[7])
{
    static uint8_t cached_prefix[7];
    static bool have_prefix;
    if (!have_prefix) {
        if (!hal.util->get_random_vals(cached_prefix, sizeof(cached_prefix))) {
            const uint64_t t = AP_HAL::micros64();
            memcpy(cached_prefix, &t, sizeof(cached_prefix));
        }
        have_prefix = true;
    }
    memcpy(prefix, cached_prefix, 7);
}

uint16_t GCS_MAVLink_Crypto_TX::encrypt(uint8_t chan, const uint8_t *buf, uint16_t len,
                                          uint8_t *out, uint16_t out_max)
{
    if (buf == nullptr || out == nullptr ||
        len == 0 || len > GCS_MAVLink_Crypto::MAX_FRAME_LEN ||
        out_max < (uint16_t)(len + GCS_MAVLink_Crypto::ENVELOPE_OVERHEAD)) {
        return 0;
    }

    uint8_t key[32];
    GCS_MAVLink_Crypto::get_key(key);

    uint8_t nonce[GCS_MAVLink_Crypto::NONCE_LEN];
    GCS_MAVLink_Crypto::get_nonce_prefix(nonce);
    nonce[7] = chan;
    nonce[8]  = (uint8_t)(counter >> 24);
    nonce[9]  = (uint8_t)(counter >> 16);
    nonce[10] = (uint8_t)(counter >> 8);
    nonce[11] = (uint8_t)(counter);
    counter++;

    out[0] = GCS_MAVLink_Crypto::MAGIC;
    out[1] = (uint8_t)(len & 0xFF);
    out[2] = (uint8_t)(len >> 8);
    memcpy(out + 3, nonce, sizeof(nonce));

    crypto_ietf_chacha20(out + 3 + sizeof(nonce), buf, len, key, nonce);
    crypto_wipe(key, sizeof(key));

    return (uint16_t)(3 + sizeof(nonce) + len);
}

bool GCS_MAVLink_Crypto_RX::feed(uint8_t b, uint8_t *plain_buf, uint16_t &plain_len)
{
    switch (state) {
    case State::WAIT_MAGIC:
        if (b == GCS_MAVLink_Crypto::MAGIC) {
            len_pos = 0;
            state = State::LEN;
        }
        return false;

    case State::LEN:
        len_bytes[len_pos++] = b;
        if (len_pos == sizeof(len_bytes)) {
            len = (uint16_t)len_bytes[0] | ((uint16_t)len_bytes[1] << 8);
            if (len == 0 || len > GCS_MAVLink_Crypto::MAX_FRAME_LEN) {
                // corrupt/garbage length; resync on the next magic byte
                reset();
                return false;
            }
            nonce_pos = 0;
            state = State::NONCE;
        }
        return false;

    case State::NONCE:
        nonce[nonce_pos++] = b;
        if (nonce_pos == sizeof(nonce)) {
            cipher_pos = 0;
            state = State::CIPHERTEXT;
        }
        return false;

    case State::CIPHERTEXT:
        cipher[cipher_pos++] = b;
        if (cipher_pos < len) {
            return false;
        }
        {
            // full envelope received; decrypt
            reset();
            uint8_t key[32];
            GCS_MAVLink_Crypto::get_key(key);
            crypto_ietf_chacha20(plain_buf, cipher, len, key, nonce);
            crypto_wipe(key, sizeof(key));
            plain_len = len;
        }
        return true;
    }

    reset();
    return false;
}

#endif  // AP_MAVLINK_LINK_CRYPTO_ENABLED
