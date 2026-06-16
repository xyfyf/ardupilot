/*
 * AP_Compass_UBLOX_GPS.h
 *
 * EFT定制：通过 u-blox GPS 串口接收厂商自定义 VENDOR-MAG UBX 帧
 * 将 QMC5883L 三轴磁力计数据注入 ArduPilot 罗盘子系统（EKF 融合）
 *
 * 数据流：
 *   GNSS 模块(QMC5883L) → UART(UBX VENDOR-MAG) → AP_GPS_UBLOX 解析
 *   → AP_Compass_UBLOX_GPS::handle_mag() → accumulate_sample() → EKF
 *
 * 编译开关：在 hwdef.dat 中设置 define AP_COMPASS_UBLOX_GPS_ENABLED 1
 */
#pragma once

#include <AP_Compass/AP_Compass_config.h>

#if AP_COMPASS_UBLOX_GPS_ENABLED

#include "AP_Compass_Backend.h"
#include <AP_HAL/AP_HAL.h>

class AP_Compass_UBLOX_GPS : public AP_Compass_Backend
{
public:
    /*
     * 工厂函数，由 AP_Compass::_detect_backends() 调用
     * instance 参数用于构造唯一的 bus_id（区分多个同类传感器）
     */
    static AP_Compass_Backend *probe(uint8_t instance);

    // ArduPilot Backend 轮询接口：将累积的磁场样本发布至前端
    void read() override;

    /*
     * 静态回调接口，由 AP_GPS_UBLOX::_handle_vendor_mag() 调用
     * 将解析后的磁场矢量（单位：mGauss）推送至后端缓冲区
     * @param field  三轴磁场强度（mGauss），传感器体坐标系
     */
    static void handle_mag(const Vector3f &field);


private:
    AP_Compass_UBLOX_GPS() {}

    // 注册时分配到的罗盘实例编号（旧版 API 的 accumulate_sample/drain 需要显式传入）
    uint8_t _instance = 0;

    // 全局单例指针，供 handle_mag() 静态方法访问
    static AP_Compass_UBLOX_GPS *_singleton;
};

#endif  // AP_COMPASS_UBLOX_GPS_ENABLED
