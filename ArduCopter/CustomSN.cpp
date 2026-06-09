#include "CustomSN.h"
#include <string.h>

extern const AP_HAL::HAL& hal;

CustomSNData CustomSN::_ram_data;

void CustomSN::init()
{
    // The flash address is directly memory-mapped, so we can read it like a normal pointer.
    const CustomSNData* flash_data = reinterpret_cast<const CustomSNData*>(CUSTOM_SN_FLASH_ADDR);

    if (flash_data->magic == CUSTOM_SN_MAGIC) {
        memcpy(&_ram_data, flash_data, sizeof(CustomSNData));
    } else {
        memset(&_ram_data, 0, sizeof(CustomSNData));
    }
}

bool CustomSN::write_to_flash(const CustomSNData& new_data)
{
    if (hal.flash == nullptr) {
        return false;
    }

    CustomSNData write_data = new_data;
    write_data.magic = CUSTOM_SN_MAGIC;
    // Clear reserved padding to keep the sector deterministic.
    memset(write_data.reserved, 0, sizeof(write_data.reserved));

    hal.flash->keep_unlocked(true);

    bool ok = hal.flash->erasepage(CUSTOM_SN_FLASH_PAGE);
    if (ok) {
        ok = hal.flash->write(CUSTOM_SN_FLASH_ADDR, &write_data, sizeof(write_data));
    }

    hal.flash->keep_unlocked(false);

    if (ok) {
        memcpy(&_ram_data, &write_data, sizeof(write_data));
    }
    return ok;
}
