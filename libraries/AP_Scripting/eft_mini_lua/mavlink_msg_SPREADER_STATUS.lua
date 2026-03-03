local SPREADER_STATUS = {}
SPREADER_STATUS.id = 512
SPREADER_STATUS.fields = {
             { "spreader_can_baudrate", "<I2" },
             { "spreader_sequence", "<I2" },
             { "spreader_firmware_version", "<I2" },
             { "spreader_servo_angle", "<B" },
             { "spreader_sensor_status", "<B" },
             { "spreader_can_enable", "<B" },
             { "spreader_speed", "<B" },
             { "spreader_function_status", "<B" },
             { "spreader_life_signal", "<B" },
             { "spreader_year", "<B" },
             { "spreader_month", "<B" },
             { "spreader_day", "<B" },
             }
return SPREADER_STATUS
