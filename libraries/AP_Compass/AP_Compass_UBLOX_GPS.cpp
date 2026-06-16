/*
 * AP_Compass_UBLOX_GPS.cpp
 *
 * EFT定制：通过 u-blox GPS 串口接收厂商自定义 VENDOR-MAG UBX 帧的磁力计后端
 *
 * 坐标系说明：
 *   QMC5883L 输出为模组本体坐标系，如安装方向与飞控 IMU 不同，
 *   需在 COMPASS_ORIENT 参数中配置安装旋转补偿。
 *
 * 单位：
 *   QMC5883L 原始值（int16）× 0.3 mGauss/LSB = mGauss（由 AP_GPS_UBLOX 完成换算）
 *   ArduPilot accumulate_sample() 接受单位为 mGauss
 *
 * Bus ID 编码：BUS_TYPE_SERIAL | port=0 | address=DEVTYPE_QMC5883L | extra=instance
 *   用于在参数树中唯一标识该传感器实例（COMPASS_DEV_ID）
 */
#include "AP_Compass_UBLOX_GPS.h"

#if AP_COMPASS_UBLOX_GPS_ENABLED

#include <AP_HAL/AP_HAL.h>
#include "AP_Compass.h"
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL &hal;

// 静态单例指针初始化
AP_Compass_UBLOX_GPS *AP_Compass_UBLOX_GPS::_singleton = nullptr;

/*
 * 工厂函数：创建后端实例并向 AP_Compass 注册
 * 采用与 AP_Compass_ExternalAHRS 相同的 bus_id 编码方式
 */
AP_Compass_Backend *AP_Compass_UBLOX_GPS::probe(uint8_t instance)
{
    // 构造唯一 bus_id：串口类型 + 端口0 + 器件类型IST8310 + 实例编号
    // 使用 IST8310 作为固定 DEVTYPE，地面站显示 "SERIAL IST8310"
    const uint32_t devid = AP_HAL::Device::make_bus_id(
        AP_HAL::Device::BUS_TYPE_SERIAL,
        0,                   // bus/port（固定为0，串口由 GPS 驱动管理）
        instance,            // address（借用实例编号或0）
        DEVTYPE_IST8310       // devtype（器件类型，存入 COMPASS_DEV_ID）
    );

    auto *ret = NEW_NOTHROW AP_Compass_UBLOX_GPS();
    if (ret == nullptr) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "UBLOX_MAG: probe failed, alloc error");
        return nullptr;
    }

    // 向 AP_Compass 前端注册，分配 instance 槽位和 COMPASS_DEV_ID 参数
    // 注意：此版本 register_compass 需要两个参数（兼容旧版本 API）
    uint8_t instance_out = 0;
    if (!ret->register_compass(devid, instance_out)) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "UBLOX_MAG: register_compass failed");
        delete ret;
        return nullptr;
    }

    // 存储分配到的实例编号，供 accumulate_sample/drain_accumulated_samples 使用
    ret->_instance = instance_out;

    // 关键：旧版 register_compass 不自动调 set_dev_id()，
    // 必须手动设置 detected_dev_id，否则 _get_state_id(Priority) 无法匹配到
    // 这个 state，导致 configured() 里的 registered 检查永远为 false → "Compass X not found"
    ret->set_dev_id(instance_out, devid);

    // 标记为外置传感器（SERIAL 罗盘在飞控外部）
    // 旧版 set_external 需要显式传入 instance 和 external 两个参数
    ret->set_external(instance_out, true);

    // 注意：若上一行 set_external 编译失败（API不存在），请删除它
    // 并在地面站手动将 COMPASS_EXTERN2 参数设为 1

    // 保存单例指针，供 handle_mag() 静态回调使用
    _singleton = ret;

    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "UBLOX_MAG: probe OK, instance=%d devid=%lu",
                  (int)instance_out, (unsigned long)devid);

    return ret;
}

/*
 * Backend 轮询接口（由 AP_Compass::read() 调用，约 10Hz）
 * 将 accumulate_sample() 积累的磁场样本均值发布至前端
 */
void AP_Compass_UBLOX_GPS::read()
{
    // 旧版本 drain_accumulated_samples 需要显式传入 instance 编号
    drain_accumulated_samples(_instance);
}

/*
 * 静态回调：由 AP_GPS_UBLOX::_handle_vendor_mag() 在 GPS 串口解析上下文中调用
 * 将磁场矢量（mGauss）推入内部环形缓冲区，等待 read() 发布
 *
 * accumulate_sample() 在内部会对多次采样取均值，降低高频噪声；
 * max_samples 默认为 10，对于 25Hz 磁力计 + 10Hz read() 频率是合适的。
 */
void AP_Compass_UBLOX_GPS::handle_mag(const Vector3f &field)
{
    if (_singleton == nullptr) {
        return;
    }

    Vector3f f = field;
    // 旧版本 accumulate_sample 需要显式传入 instance 编号
    _singleton->accumulate_sample(f, _singleton->_instance);
}


#endif  // AP_COMPASS_UBLOX_GPS_ENABLED
