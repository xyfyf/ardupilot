-- MAVLink message 520: UOM_FC_STATUS
-- FC → GCS: flight controller UOM activation status report (sent periodically)
-- Wire layout (sorted by field size, largest first):
--   bytes 0-1 : status_code   uint16_t  (UOM_ARM_STATUS_CODE; 0=unknown)
--   bytes 2-3 : status_age_s  uint16_t  (seconds since last GCS update; 0xFFFF=never)
--   byte  4   : allow_arm     uint8_t   (0=prohibited, 1=allowed)
--   byte  5   : is_armed      uint8_t   (0=disarmed, 1=armed)
local UOM_FC_STATUS = {}
UOM_FC_STATUS.id = 520
UOM_FC_STATUS.fields = {
    { "status_code",  "<I2" },
    { "status_age_s", "<I2" },
    { "allow_arm",    "<B"  },
    { "is_armed",     "<B"  },
}
return UOM_FC_STATUS
