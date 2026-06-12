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

  Write-once semantics:
    On boot we snapshot each group's "any non-zero" state. A group whose
    snapshot is set is considered locked: MAVLink/GCS PARAM_SET on any
    chunk in that group is denied. A factory operator can therefore write
    all 7 chunks of a group in one session (lock activates at next boot).
*/
class FactorySN {
public:
    FactorySN();

    static const struct AP_Param::GroupInfo var_info[];

    static constexpr uint8_t NUM_CHUNKS = 7;
    static constexpr uint8_t BYTES_PER_CHUNK = 3;
    static constexpr uint8_t MAX_CHARS = NUM_CHUNKS * BYTES_PER_CHUNK; // 21

    // Sample current chunk values and lock groups whose stored value is non-zero.
    // Call once during boot, after AP_Param::load_all().
    void snapshot_lock_state();

    // True if `name` is one of our SN_* params AND its group is locked.
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

    bool _locked[(uint8_t)Group::NUM_GROUPS];

    static Group group_for_param(const char *name);
    const AP_Int32 *chunks_for_group(Group g) const;
    static const char *label_for_group(Group g);
    static void decode_to_string(const AP_Int32 *chunks, char *dest, size_t dest_size);
};
