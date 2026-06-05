local WEIGHT_CALIBRATION = {}
WEIGHT_CALIBRATION.id = 505
WEIGHT_CALIBRATION.fields = {
             { "calibration_weight", "<I2" },
             { "k_values", "<I2", 3 },
             { "led_control", "<B" },
             { "right_led_brightness", "<B" },
             { "left_led_brightness", "<B" },
             { "tare_calibration", "<B" },
             { "weight_calibration", "<B" },
             { "k_calibration", "<B" },
             }
return WEIGHT_CALIBRATION
