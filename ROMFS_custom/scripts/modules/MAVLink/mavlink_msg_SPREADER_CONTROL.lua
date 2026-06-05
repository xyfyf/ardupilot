local SPREADER_CONTROL = {}
SPREADER_CONTROL.id = 511
SPREADER_CONTROL.fields = {
             { "spreader_motor_pwm", "<I2" },
             { "spreader_valve_pwm", "<I2" },
             { "motor_control_cmd", "<B" },
             { "signal_source_cmd", "<B" },
             { "spreader_signal_source", "<B" },
             { "alarm_config_cmd", "<B" },
             { "spreader_alarm_config", "<B", 3 },
             { "spreader_factory_reset", "<B" },
             }
return SPREADER_CONTROL
