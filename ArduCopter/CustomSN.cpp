#include "CustomSN.h"
#include <string.h>

extern const AP_HAL::HAL& hal;

CustomSNData CustomSN::_ram_data;

void CustomSN::init() {
    const CustomSNData* flash_data = (const CustomSNData*)CUSTOM_SN_FLASH_ADDR;
    
    // Check if the flash area has been initialized with our magic number
    if (flash_data->magic == CUSTOM_SN_MAGIC) {
        memcpy(&_ram_data, flash_data, sizeof(CustomSNData));
    } else {
        memset(&_ram_data, 0, sizeof(CustomSNData));
    }
}

bool CustomSN::write_to_flash(const CustomSNData& new_data) {
    if (!hal.flash) {
        return false;
    }

    CustomSNData write_data = new_data;
    write_data.magic = CUSTOM_SN_MAGIC;

    // STM32H7 requires unlocking before erase/write
    hal.flash->keep_unlocked(true);
    
    // Erase the entire Page 15 (128KB)
    bool ret = hal.flash->erasepage(CUSTOM_SN_FLASH_PAGE);
    if (ret) {
        // Write the 96-byte struct to the start of Page 15
        ret = hal.flash->write(CUSTOM_SN_FLASH_ADDR, &write_data, sizeof(CustomSNData));
    }
    
    hal.flash->keep_unlocked(false);

    // Update RAM copy if flash write succeeded
    if (ret) {
        memcpy(&_ram_data, &write_data, sizeof(CustomSNData));
    }

    return ret;
}
