#pragma once

#include "AP_Logger_config.h"

#if AP_LOGGER_EFT_ENCRYPT_ENABLED

#include <stdint.h>
#include <stddef.h>

/*
  EFT flight log encryption (ChaCha20-CTR IETF / monocypher).

  File layout: Tools/eft_log/eft-log-decrypt-地面站开发说明.md (local, not in git)
*/
class AP_Logger_EFT_Crypto {
public:
    static constexpr uint8_t HEADER_SIZE = 17;
    static constexpr uint8_t VERSION = 1;

    static bool is_eftl_header(const uint8_t *data, size_t len);

    // Prepare key + nonce for a new log (reads SN_FC params).
    bool begin_write();

    // Write 17-byte EFTL header into buf (HEADER_SIZE bytes).
    void format_header(uint8_t buf[HEADER_SIZE]) const;

    // XOR keystream at plain-text byte offset (encrypt and decrypt are identical).
    void crypt_buffer(uint8_t *data, size_t len, uint32_t plain_offset) const;

    // Load key + nonce from SN_FC and file header for read/decrypt.
    bool begin_read(const char *fc_sn, const uint8_t nonce[12]);

private:
    static constexpr const char *KDF_SALT = "EFT-LOG-v1";

    uint8_t _key[32];
    uint8_t _nonce[12];
    bool _ready;

    static void read_fc_sn(char *dest, size_t dest_size);
    static void derive_key(uint8_t key[32], const char *fc_sn);
};

#endif // AP_LOGGER_EFT_ENCRYPT_ENABLED
