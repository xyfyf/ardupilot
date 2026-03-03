local BATTERY_DATA = {}
BATTERY_DATA.id = 510
BATTERY_DATA.fields = {
             { "current", "<I4" },
             { "voltage", "<I2" },
             { "cell_temp", "<I2" },
             { "mosfet_temp", "<I2" },
             { "capacity_percent", "<I2" },
             }
return BATTERY_DATA
