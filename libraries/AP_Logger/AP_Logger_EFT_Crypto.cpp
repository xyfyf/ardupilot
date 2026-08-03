#include "AP_Logger_EFT_Crypto.h"

#if AP_LOGGER_EFT_ENCRYPT_ENABLED

#include <AP_CheckFirmware/monocypher.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_Param/AP_Param.h>

#include <stdio.h>
#include <string.h>

extern const AP_HAL::HAL &hal;

/*
  32-byte vendor master secret. Replace before production; distribute to
  GCS separately (EFT_LOG_MASTER_KEY env; see Tools/eft_log/decrypt_eft.py).
*/
#ifndef AP_LOGGER_EFT_MASTER_KEY
#define AP_LOGGER_EFT_MASTER_KEY                                               \
    0xEF, 0x54, 0x46, 0x4c, 0x4f, 0x47, 0x4d, 0x41, 0x53, 0x54, 0x45, 0x52,    \
    0x4b, 0x45, 0x59, 0x21, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,    \
    0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f
#endif

static const uint8_t master_key[32] = {AP_LOGGER_EFT_MASTER_KEY};

static const uint8_t EFTL_MAGIC[4] = {'E', 'F', 'T', 'L'};

bool AP_Logger_EFT_Crypto::is_eftl_header(const uint8_t *data, const size_t len)
{
    if (data == nullptr || len < 4) {
        return false;
    }
    return memcmp(data, EFTL_MAGIC, 4) == 0;
}

void AP_Logger_EFT_Crypto::read_fc_sn(char *dest, const size_t dest_size)
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

void AP_Logger_EFT_Crypto::derive_key(uint8_t key[32], const char *fc_sn)
{
    crypto_blake2b_ctx ctx;
    crypto_blake2b_general_init(&ctx, 32, nullptr, 0);
    crypto_blake2b_update(&ctx, master_key, sizeof(master_key));
    crypto_blake2b_update(&ctx, (const uint8_t *)KDF_SALT, strlen(KDF_SALT));
    if (fc_sn != nullptr && fc_sn[0] != '\0') {
        crypto_blake2b_update(&ctx, (const uint8_t *)fc_sn, strlen(fc_sn));
    }
    crypto_blake2b_final(&ctx, key);
}

bool AP_Logger_EFT_Crypto::begin_write()
{
    char fc_sn[22];
    read_fc_sn(fc_sn, sizeof(fc_sn));
    derive_key(_key, fc_sn);

    if (!hal.util->get_random_vals(_nonce, sizeof(_nonce))) {
        const uint64_t t = AP_HAL::micros64();
        memcpy(_nonce, &t, MIN(sizeof(_nonce), sizeof(t)));
        const uint32_t ms = AP_HAL::millis();
        memcpy(_nonce + 8, &ms, MIN(sizeof(_nonce) - 8, sizeof(ms)));
    }

    _ready = true;
    return true;
}

bool AP_Logger_EFT_Crypto::begin_read(const char *fc_sn, const uint8_t nonce[12])
{
    if (fc_sn == nullptr || fc_sn[0] == '\0' || nonce == nullptr) {
        return false;
    }
    derive_key(_key, fc_sn);
    memcpy(_nonce, nonce, sizeof(_nonce));
    _ready = true;
    return true;
}

void AP_Logger_EFT_Crypto::format_header(uint8_t buf[HEADER_SIZE]) const
{
    memcpy(buf, EFTL_MAGIC, 4);
    buf[4] = VERSION;
    memcpy(buf + 5, _nonce, sizeof(_nonce));
}

void AP_Logger_EFT_Crypto::crypt_buffer(uint8_t *data, const size_t len, const uint32_t plain_offset) const
{
    if (data == nullptr || len == 0 || !_ready) {
        return;
    }

    size_t pos = 0;
    while (pos < len) {
        const uint32_t abs_off = plain_offset + (uint32_t)pos;
        const uint32_t block_ctr = abs_off / 64U;
        const uint32_t skip = abs_off % 64U;
        const size_t chunk = MIN(len - pos, (size_t)(64U - skip));

        uint8_t keystream[64];
        crypto_ietf_chacha20_ctr(keystream, nullptr, 64, _key, _nonce, block_ctr);

        for (size_t i = 0; i < chunk; i++) {
            data[pos + i] ^= keystream[skip + i];
        }
        pos += chunk;
    }
}

#endif // AP_LOGGER_EFT_ENCRYPT_ENABLED
