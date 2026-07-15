#include "FactorySN.h"

#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h>

#include <string.h>

#define SN_GROUPINFO(_name, _idx, _field)                                                        \
    AP_GROUPINFO(_name "1", (_idx) + 0, FactorySN, _field[0], 0),                               \
    AP_GROUPINFO(_name "2", (_idx) + 1, FactorySN, _field[1], 0),                               \
    AP_GROUPINFO(_name "3", (_idx) + 2, FactorySN, _field[2], 0),                               \
    AP_GROUPINFO(_name "4", (_idx) + 3, FactorySN, _field[3], 0),                               \
    AP_GROUPINFO(_name "5", (_idx) + 4, FactorySN, _field[4], 0),                               \
    AP_GROUPINFO(_name "6", (_idx) + 5, FactorySN, _field[5], 0),                               \
    AP_GROUPINFO(_name "7", (_idx) + 6, FactorySN, _field[6], 0)

const AP_Param::GroupInfo FactorySN::var_info[] = {
    // @Param: PROD1
    // @DisplayName: Product model SN chunk 1
    // @Description: ASCII chunk 1 of product_model. Packed (byte0<<16)|(byte1<<8)|byte2. Write-once via MAVLink.
    // @User: Advanced
    // @ReadOnly: True

    // @Param: PROD2
    // @DisplayName: Product model SN chunk 2
    // @Description: ASCII chunk 2 of product_model.
    // @User: Advanced

    // @Param: PROD3
    // @DisplayName: Product model SN chunk 3
    // @Description: ASCII chunk 3 of product_model.
    // @User: Advanced

    // @Param: PROD4
    // @DisplayName: Product model SN chunk 4
    // @Description: ASCII chunk 4 of product_model.
    // @User: Advanced

    // @Param: PROD5
    // @DisplayName: Product model SN chunk 5
    // @Description: ASCII chunk 5 of product_model.
    // @User: Advanced

    // @Param: PROD6
    // @DisplayName: Product model SN chunk 6
    // @Description: ASCII chunk 6 of product_model.
    // @User: Advanced

    // @Param: PROD7
    // @DisplayName: Product model SN chunk 7
    // @Description: ASCII chunk 7 of product_model.
    // @User: Advanced
    SN_GROUPINFO("PROD",  1, _prod),

    // @Param: FACT1
    // @DisplayName: Factory SN chunk 1
    // @Description: ASCII chunk 1 of factory_sn. Write-once via MAVLink.
    // @User: Advanced

    // @Param: FACT2
    // @DisplayName: Factory SN chunk 2
    // @User: Advanced

    // @Param: FACT3
    // @DisplayName: Factory SN chunk 3
    // @User: Advanced

    // @Param: FACT4
    // @DisplayName: Factory SN chunk 4
    // @User: Advanced

    // @Param: FACT5
    // @DisplayName: Factory SN chunk 5
    // @User: Advanced

    // @Param: FACT6
    // @DisplayName: Factory SN chunk 6
    // @User: Advanced

    // @Param: FACT7
    // @DisplayName: Factory SN chunk 7
    // @User: Advanced
    SN_GROUPINFO("FACT",  8, _fact),

    // @Param: FRM1
    // @DisplayName: Frame SN chunk 1
    // @Description: ASCII chunk 1 of frame_sn. Write-once via MAVLink.
    // @User: Advanced

    // @Param: FRM2
    // @DisplayName: Frame SN chunk 2
    // @User: Advanced

    // @Param: FRM3
    // @DisplayName: Frame SN chunk 3
    // @User: Advanced

    // @Param: FRM4
    // @DisplayName: Frame SN chunk 4
    // @User: Advanced

    // @Param: FRM5
    // @DisplayName: Frame SN chunk 5
    // @User: Advanced

    // @Param: FRM6
    // @DisplayName: Frame SN chunk 6
    // @User: Advanced

    // @Param: FRM7
    // @DisplayName: Frame SN chunk 7
    // @User: Advanced
    SN_GROUPINFO("FRM",  15, _frame),

    // @Param: FC1
    // @DisplayName: Flight controller SN chunk 1
    // @Description: ASCII chunk 1 of fc_sn. Write-once via MAVLink.
    // @User: Advanced

    // @Param: FC2
    // @DisplayName: Flight controller SN chunk 2
    // @User: Advanced

    // @Param: FC3
    // @DisplayName: Flight controller SN chunk 3
    // @User: Advanced

    // @Param: FC4
    // @DisplayName: Flight controller SN chunk 4
    // @User: Advanced

    // @Param: FC5
    // @DisplayName: Flight controller SN chunk 5
    // @User: Advanced

    // @Param: FC6
    // @DisplayName: Flight controller SN chunk 6
    // @User: Advanced

    // @Param: FC7
    // @DisplayName: Flight controller SN chunk 7
    // @User: Advanced
    SN_GROUPINFO("FC",   22, _fc),

    AP_GROUPEND
};

FactorySN::FactorySN()
{
    AP_Param::setup_object_defaults(this, var_info);
}

FactorySN::Group FactorySN::group_for_param(const char *name)
{
    // AP_Param treats parameter names case-insensitively, so we must too.
    if (name == nullptr) {
        return Group::NONE;
    }
    if (strncasecmp(name, "SN_PROD", 7) == 0) {
        return Group::PRODUCT_MODEL;
    }
    if (strncasecmp(name, "SN_FACT", 7) == 0) {
        return Group::FACTORY_SN;
    }
    if (strncasecmp(name, "SN_FRM", 6) == 0) {
        return Group::FRAME_SN;
    }
    if (strncasecmp(name, "SN_FC", 5) == 0) {
        return Group::FC_SN;
    }
    return Group::NONE;
}

const AP_Int32 *FactorySN::chunks_for_group(Group g) const
{
    switch (g) {
    case Group::PRODUCT_MODEL: return _prod;
    case Group::FACTORY_SN:    return _fact;
    case Group::FRAME_SN:      return _frame;
    case Group::FC_SN:         return _fc;
    default:                   return nullptr;
    }
}

const char *FactorySN::label_for_group(Group g)
{
    switch (g) {
    case Group::PRODUCT_MODEL: return "ProductModel";
    case Group::FACTORY_SN:    return "FactorySN";
    case Group::FRAME_SN:      return "FrameSN";
    case Group::FC_SN:         return "FC_SN";
    default:                   return "";
    }
}

int8_t FactorySN::chunk_index_of(const char *name)
{
    if (name == nullptr) {
        return -1;
    }
    const size_t n = strlen(name);
    if (n == 0) {
        return -1;
    }
    const char last = name[n - 1];
    if (last < '1' || last > (char)('0' + NUM_CHUNKS)) {
        return -1;
    }
    return (int8_t)(last - '1');
}

bool FactorySN::is_param_locked(const char *name) const
{
    // Per-chunk strict lock: as soon as a SN_xxxN parameter holds a non-zero
    // value, further PARAM_SET attempts that try to change it are denied.
    const Group g = group_for_param(name);
    if (g == Group::NONE) {
        return false;
    }
    const int8_t idx = chunk_index_of(name);
    if (idx < 0 || idx >= (int8_t)NUM_CHUNKS) {
        return false;
    }
    const AP_Int32 *chunks = chunks_for_group(g);
    if (chunks == nullptr) {
        return false;
    }
    return chunks[idx].get() != 0;
}

void FactorySN::clear_all()
{
    AP_Int32 *groups[4] = { _prod, _fact, _frame, _fc };
    for (uint8_t g = 0; g < 4; g++) {
        for (uint8_t i = 0; i < NUM_CHUNKS; i++) {
            if (groups[g][i].get() != 0) {
                groups[g][i].set_and_save(0);
            }
        }
    }
}

void FactorySN::decode_to_string(const AP_Int32 *chunks, char *dest, size_t dest_size)
{
    if (dest == nullptr || dest_size == 0) {
        return;
    }
    size_t pos = 0;
    for (uint8_t i = 0; i < NUM_CHUNKS && pos + 1 < dest_size; i++) {
        const uint32_t v = (uint32_t)chunks[i].get();
        const uint8_t bytes[BYTES_PER_CHUNK] = {
            (uint8_t)((v >> 16) & 0xFF),
            (uint8_t)((v >>  8) & 0xFF),
            (uint8_t)( v        & 0xFF),
        };
        for (uint8_t b = 0; b < BYTES_PER_CHUNK && pos + 1 < dest_size; b++) {
            const uint8_t c = bytes[b];
            if (c == 0) {
                // Zero byte terminates the string.
                dest[pos] = '\0';
                return;
            }
            // Replace non-printable bytes with '?' so the banner stays GCS-safe.
            dest[pos++] = (c >= 0x20 && c <= 0x7E) ? (char)c : '?';
        }
    }
    dest[pos] = '\0';
}

void FactorySN::send_banner() const
{
#if HAL_GCS_ENABLED
    char buf[MAX_CHARS + 1];
    for (uint8_t g = 0; g < (uint8_t)Group::NUM_GROUPS; g++) {
        const AP_Int32 *chunks = chunks_for_group((Group)g);
        if (chunks == nullptr) {
            continue;
        }
        decode_to_string(chunks, buf, sizeof(buf));
        if (buf[0] == '\0') {
            gcs().send_text(MAV_SEVERITY_INFO, "%s: <unset>", label_for_group((Group)g));
        } else {
            gcs().send_text(MAV_SEVERITY_INFO, "%s: %s", label_for_group((Group)g), buf);
        }
    }
#endif
}
