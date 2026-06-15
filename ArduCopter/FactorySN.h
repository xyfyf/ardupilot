#pragma once

#include <AP_Param/AP_Param.h>
#include <stdint.h>
#include <stddef.h>

/*
  Factory serial numbers stored as regular AP_Param values.

  We expose four "ID groups" matching the nameplate fields:
    product_model -> SN_PROD1..7
    factory_sn    -> SN_FACT1..7
    frame_sn      -> SN_FRM1..7
    fc_sn         -> SN_FC1..7

  Each group is 7 x AP_Int32. Every AP_Int32 packs 3 ASCII bytes:
    value = (byte0 << 16) | (byte1 << 8) | byte2
  This keeps the value under 2^24 so it round-trips exactly through the
  MAVLink float-based parameter protocol. 7 x 3 = 21 chars capacity per
  SN (user spec: at most 20 chars).

  Strict per-chunk write-once semantics:
    Each SN_xxxN parameter becomes immutable the moment its current value
    is non-zero. PARAM_SET to a non-zero chunk with a different value is
    denied by the GCS_MAVLINK_Copter PARAM_SET intercept, regardless of
    whether we have rebooted since the first write. Writes that supply
    the SAME value (no-op refreshes) are allowed so periodic GCS sync
    does not produce spurious "locked" warnings.

    Consequence: each chunk MUST be written correctly on the very first
    try. A typo cannot be repaired short of erasing EEPROM (re-flashing).
*/
class FactorySN {
public:
    FactorySN();

    static const struct AP_Param::GroupInfo var_info[];

    static constexpr uint8_t NUM_CHUNKS = 7;
    static constexpr uint8_t BYTES_PER_CHUNK = 3;
    static constexpr uint8_t MAX_CHARS = NUM_CHUNKS * BYTES_PER_CHUNK; // 21

    // True if `name` is one of our SN_xxxN params AND its current value is
    // non-zero. Locking is per-chunk and based on the live AP_Int32 value
    // (NOT a boot snapshot), so it engages instantly after the first
    // successful write.
    bool is_param_locked(const char *name) const;

    // Send each configured SN to all GCS connections via STATUSTEXT.
    void send_banner() const;

private:
    enum class Group : uint8_t {
        PRODUCT_MODEL = 0,
        FACTORY_SN    = 1,
        FRAME_SN      = 2,
        FC_SN         = 3,
        NUM_GROUPS    = 4,
        NONE          = 0xFF,
    };

    AP_Int32 _prod [NUM_CHUNKS];
    AP_Int32 _fact [NUM_CHUNKS];
    AP_Int32 _frame[NUM_CHUNKS];
    AP_Int32 _fc   [NUM_CHUNKS];

    static Group group_for_param(const char *name);
    const AP_Int32 *chunks_for_group(Group g) const;
    static const char *label_for_group(Group g);
    static void decode_to_string(const AP_Int32 *chunks, char *dest, size_t dest_size);

    // Parse the trailing digit ('1'..'7') of a SN_xxxN name to a chunk index 0..6.
    // Returns -1 if the name does not end with a valid digit.
    static int8_t chunk_index_of(const char *name);
};
