#pragma once

#include <AP_HAL/AP_HAL.h>

// STM32H743 has 2MB flash. Page 15 is the last 128KB sector, starting at 0x081E0000.
// It is unused because EFT_CAAC hwdef uses Page 14 for HAL_STORAGE.
#define CUSTOM_SN_FLASH_PAGE 15
#define CUSTOM_SN_FLASH_ADDR 0x081E0000
#define CUSTOM_SN_MAGIC 0xA1B2C3D4

struct CustomSNData {
    char product_sn[20];
    char fc_sn[20];
    char frame_sn[20];
    char machine_sn[20];
    uint32_t magic;
    uint8_t reserved[12]; // Pad to 96 bytes (multiple of 32 for STM32H7 flash write alignment)
};

class CustomSN {
public:
    // Initialize from flash at boot
    static void init();
    
    // Write new SN data to flash
    static bool write_to_flash(const CustomSNData& new_data);
    
    // Get the RAM copy of the SN data
    static const CustomSNData& get_data() { return _ram_data; }

private:
    static CustomSNData _ram_data;
};
