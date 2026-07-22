-- MAVLink message 519: UOM_ARM_STATUS
-- GCS → FC: UOM 平台激活状态码（是否允许解锁由飞控根据 status_code 推导）
-- Wire layout:
--   bytes 0-1 : status_code  uint16_t  (UOM_ARM_STATUS_CODE enum)
local UOM_ARM_STATUS = {}
UOM_ARM_STATUS.id = 519
UOM_ARM_STATUS.fields = {
    { "status_code", "<I2" },
}
return UOM_ARM_STATUS
