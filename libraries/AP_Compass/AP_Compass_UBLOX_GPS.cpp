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

#include <AP_HAL/AP_HAL.h>
#include "AP_Compass_UBLOX_GPS.h"

#if AP_COMPASS_UBLOX_GPS_ENABLED

AP_Compass_UBLOX_GPS *AP_Compass_UBLOX_GPS::_instances[2];

AP_Compass_Backend *AP_Compass_UBLOX_GPS::probe(uint8_t gps_instance)
{
    if (gps_instance >= ARRAY_SIZE(_instances)) {
        return nullptr;
    }

    auto devid = AP_HAL::Device::make_bus_id(
        AP_HAL::Device::BUS_TYPE_SERIAL, gps_instance, 0, DEVTYPE_IST8310);

    auto *ret = NEW_NOTHROW AP_Compass_UBLOX_GPS();
    if (ret == nullptr) {
        return nullptr;
    }
    if (!ret->register_compass(devid)) {
        delete ret;
        return nullptr;
    }
    ret->set_external(true);
    _instances[gps_instance] = ret;
    return ret;
}

void AP_Compass_UBLOX_GPS::handle_mag(uint8_t gps_instance, const Vector3f &field)
{
    if (gps_instance >= ARRAY_SIZE(_instances) || _instances[gps_instance] == nullptr) {
        return;
    }
    Vector3f f = field;
    _instances[gps_instance]->accumulate_sample(f);
}

void AP_Compass_UBLOX_GPS::read(void)
{
    drain_accumulated_samples();
}

#endif // AP_COMPASS_UBLOX_GPS_ENABLED
