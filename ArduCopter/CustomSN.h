#pragma once

#include <AP_HAL/AP_HAL.h>

/*
  Custom SN storage in dedicated flash sector.

  STM32H743 has 2MB flash, organized as 16 x 128KB sectors (Page 0~15).
  - Page 0       : Bootloader (FLASH_RESERVE_START_KB = 128)
  - Page 1 ~ 13  : Firmware (limited to 1664KB by linker)
  - Page 14      : HAL_STORAGE (parameters, waypoints, etc.)
  - Page 15      : FREE - we use this for custom SN data.

  Page 15 starts at 0x081E0000 and is 128KB.
  Our payload is only 128 bytes, the rest of the sector is unused/erased.
*/
#define CUSTOM_SN_FLASH_PAGE  15
#define CUSTOM_SN_FLASH_ADDR  0x081E0000
#define CUSTOM_SN_MAGIC       0xA1B2C3D4

// Length of each SN string buffer (including null terminator).
// EFT nameplate "产品型号" can be up to 22 chars (e.g. EFT0-X610-PMES026F0001).
#define CUSTOM_SN_FIELD_LEN   24

/*
  On-flash data layout for the custom serial numbers.
  Total size = 128 bytes (multiple of 32 for STM32H7 flash write alignment).

  Fields are named after the EFT nameplate labels:
    product_model  : 产品型号  (e.g. EFT0-X610-PMES026F0001)
    factory_sn     : 出厂编号  (e.g. EFT0X610202505060001)
    frame_sn       : 机身编号  (e.g. X610F202505060001)
    fc_sn          : 飞控SN    (e.g. X1001202505060001)
*/
struct CustomSNData {
    char     product_model[CUSTOM_SN_FIELD_LEN];   // 产品型号
    char     factory_sn   [CUSTOM_SN_FIELD_LEN];   // 出厂编号
    char     frame_sn     [CUSTOM_SN_FIELD_LEN];   // 机身编号
    char     fc_sn        [CUSTOM_SN_FIELD_LEN];   // 飞控SN
    uint32_t magic;
    uint8_t  reserved[28];                         // pad to 128 bytes
};

static_assert(sizeof(CustomSNData) == 128, "CustomSNData must be 128 bytes");
static_assert(sizeof(CustomSNData) % 32 == 0, "CustomSNData must be aligned to 32 bytes for STM32H7 flash write");

class CustomSN {
public:
    // Read SN data from flash into RAM at boot
    static void init();

    // Erase Page 15 and write new SN data. Magic is filled automatically.
    // WARNING: erase blocks the CPU for ~1s. Do NOT call during init_ardupilot().
    static bool write_to_flash(const CustomSNData& new_data);

    // Return true if the data in RAM was successfully loaded from flash.
    static bool is_valid() { return _ram_data.magic == CUSTOM_SN_MAGIC; }

    // Get the RAM copy of the SN data (zeroed if flash has not been programmed yet)
    static const CustomSNData& get_data() { return _ram_data; }

private:
    static CustomSNData _ram_data;
};
