local FMU_PMU_UART_MESSAGE = {}
FMU_PMU_UART_MESSAGE.id = 515
FMU_PMU_UART_MESSAGE.fields = {
             { "pump_control", "<I2" },
             { "nozzle_control", "<I4" },
             { "control_mode", "<B" },
             { "horizontal_speed", "<I2" },
             { "spray_rate", "<I2" },
             { "spray_width", "<I2" },
             { "pump_calibration_cmd", "<B" },
             { "led_control_cmd", "<B" },
             { "led_brightness_right", "<B" },
             { "led_brightness_left", "<B" },
             { "tare_calibration_cmd", "<B" },
             { "weight_calibration_cmd", "<B" },
             { "calibration_weight", "<I2" },
             { "k_value_calibration_cmd", "<B" },
             { "k_values", "<I2", 3 },
             { "spreader_control_cmd", "<B" },
             { "spreader_motor_pwm", "<I2" },
             { "spreader_valve_pwm", "<I2" },
             { "signal_source_cmd", "<B" },
             { "signal_source", "<B" },
             { "alarm_config_cmd", "<B" },
             { "alarm_config", "<B", 3 },
             { "factory_reset_cmd", "<B" },
             { "spray_spreader_mode", "<B" },
             }
return FMU_PMU_UART_MESSAGE
