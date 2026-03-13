/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#pragma once

#include "AP_Compass_config.h"

#if AP_COMPASS_UBLOX_GPS_ENABLED

#include "AP_Compass.h"
#include "AP_Compass_Backend.h"

// 通过 u-blox GPS 串口自定义 UBX 扩展消息 (CLASS=0xF2, ID=0x01)
// 接收 IST8310 磁力计数据的罗盘后端
class AP_Compass_UBLOX_GPS : public AP_Compass_Backend
{
public:
    using AP_Compass_Backend::AP_Compass_Backend;

    static AP_Compass_Backend *probe(uint8_t gps_instance);

    void read(void) override;

    // GPS 驱动解析到 VENDOR-MAG 帧后调用，field 单位 milliGauss（线程安全）
    static void handle_mag(uint8_t gps_instance, const Vector3f &field);

private:
    static AP_Compass_UBLOX_GPS *_instances[2];
};

#endif  // AP_COMPASS_UBLOX_GPS_ENABLED
