-- MAVLink message 519: UOM_ARM_STATUS
-- GCS → FC: UOM 平台激活与解锁权限状态
-- Wire layout (sorted by field size, largest first):
--   bytes 0-1 : status_code      uint16_t  (UOM_ARM_STATUS_CODE enum)
--   byte  2   : allow_arm        uint8_t   (0=禁止解锁, 1=允许解锁)
--   byte  3   : target_system    uint8_t
--   byte  4   : target_component uint8_t
local UOM_ARM_STATUS = {}
UOM_ARM_STATUS.id = 519
UOM_ARM_STATUS.fields = {
    { "status_code",      "<I2" },
    { "allow_arm",        "<B"  },
    { "target_system",    "<B"  },
    { "target_component", "<B"  },
}
return UOM_ARM_STATUS
