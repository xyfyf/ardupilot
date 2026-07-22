-- MAVLink message 521: UOM_OPERATOR_ID
-- GCS → FC: UOM operator ID delivered during the activation process.
-- Wire layout (both fields are 1-byte elements; declaration order preserved):
--   bytes  0-19 : operator_id      char[20]  ASCII string, null-padded
--   byte   20   : operator_id_type uint8_t   MAV_ODID_OPERATOR_ID_TYPE
local UOM_OPERATOR_ID = {}
UOM_OPERATOR_ID.id = 521
UOM_OPERATOR_ID.fields = {
    { "operator_id",      "<c20" },
    { "operator_id_type", "<B"   },
}
return UOM_OPERATOR_ID
