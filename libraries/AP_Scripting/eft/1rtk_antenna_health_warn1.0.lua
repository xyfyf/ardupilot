--[[
  脚本名称: 1rtk_antenna_health_warn.lua
  适用机型: EFT_CAAC 飞控
              GPS2 = UM982 双天线 RTK on SERIAL7 (instance 1)

  功能:
    专门检测 UM982 双天线 RTK 的两根天线健康状态, 出现异常时给地面站
    告警, 提示飞行员维修. 本脚本只做"提示告警", 不修改任何参数, 与
    1gps1_gps2_yaw_primary_switch.lua 配合使用 (后者负责参数切换).

  天线故障判定 (RTK 模块本身仍在通信的前提下):
    1. MAIN_FAIL  (主天线异常 / 无定位数据):
         RTK 模块串口仍在响应, 但 gps:status(1) < 3D_FIX.
         典型现象: 主天线被拔掉 / 接触不良 / 主天线线缆断, 模块
         仍能输出 NMEA, 但 GGA 中 fix quality 为 0, 飞控解析后定位
         状态降为 NO_FIX. 此时 RTK 既无定位也无航向.
    2. YAW_FAIL   (副天线异常 / 无航向数据):
         RTK 模块串口仍在响应, gps:status(1) >= 3D_FIX (主天线 OK),
         但 gps:gps_yaw_deg(1) 返回 nil (UM982 解算不出 moving baseline
         heading). 典型现象: 副天线被拔掉 / 接触不良 / 副天线线缆断.
         此时定位还能用, 但飞控失去 GPS 双天线航向, 必须依赖磁罗盘.
    3. NORMAL     (双天线均健康):
         status >= 3D_FIX 且 gps_yaw_deg() 返回非 nil.

  RTK 模块"在线"的判定 (重要 - 与 primary_switch 脚本解耦):
    - 仅通过 gps:num_sensors() >= 2 以及 gps:last_message_time_ms(1) 距离
      当前不超过 MODULE_TIMEOUT_MS 判断模块串口是否还在通信.
    - 本脚本 **不读也不写** GPS2_TYPE 参数, 完全把 GPS2_TYPE 的管理交给
      1gps1_gps2_yaw_primary_switch.lua, 避免两个脚本对同一参数产生时序
      冲突 (primary_switch 在 RTK 离线时会把 GPS2_TYPE 写 0, 在脚本启动
      时又会拉回 25 重新探测).
    - 如果模块整体离线 (例如串口断 / 模块掉电), 表现为 num_sensors() 减少
      或 last_message_time_ms 长时间不更新, 本脚本一律视为 H_UNKNOWN 不
      告警, 由 1gps1_gps2_yaw_primary_switch.lua 统一播报 "GPS: GPS1 only",
      避免重复刷屏.

  防抖动 / 告警频率控制:
    - 状态变化需连续 CONFIRM_COUNT 次 (秒) 满足条件才确认, 避免瞬时
      抖动导致频繁告警.
    - 故障状态下, 每隔 REPEAT_WARN_MS 重复一次告警, 确保飞行员注意到.
    - 故障恢复时播报一次 "OK" 信息.
    - 飞行中 (armed) 与地面 (disarmed) 都生效, 但飞行中只播报, 不会
      做任何参数修改.
--]]

---@diagnostic disable: need-check-nil, cast-local-type, assign-type-mismatch, param-type-mismatch

local RUN_INTERVAL_MS    = 1000   -- 1Hz 检查
local STARTUP_DELAY_MS   = 8000   -- 启动延迟 8 秒, 等 UM982 上电稳定
local CONFIRM_COUNT      = 5      -- 连续 5 次 (5 秒) 确认状态变化
local REPEAT_WARN_MS     = 15000  -- 故障状态重复告警间隔 15 秒
local MODULE_TIMEOUT_MS  = 3000   -- 超过 3 秒没收到 GPS2 消息视为模块离线
local MIN_FIX_STATUS     = 3      -- 3=3D Fix, 主天线最低门槛

local GPS2_INSTANCE      = 1      -- UM982 RTK 实例号

-- 告警严重级别 (MAVLink SEVERITY)
local SEV_INFO     = 6     -- INFO
local SEV_WARN     = 4     -- WARNING
local SEV_CRITICAL = 2     -- CRITICAL (飞行员必须立即关注)

-- 健康状态
local H_UNKNOWN    = 0   -- 启动初期 / 模块离线, 不处理
local H_OK         = 1   -- 双天线均健康
local H_MAIN_FAIL  = 2   -- 主天线故障 (无定位)
local H_YAW_FAIL   = 3   -- 副天线故障 (无航向)

local current_state    = H_UNKNOWN
local pending_state    = H_UNKNOWN
local pending_count    = 0
local last_warn_ms     = millis()   -- uint32_t_ud, 与后续 millis() 比较类型一致

-- 安全读取 GPS 状态: 当 instance 超过 num_sensors() 时返回 0, 避免崩溃
local function safe_gps_status(instance)
    local n = gps:num_sensors() or 0
    if instance >= n then
        return 0
    end
    return gps:status(instance) or 0
end

-- 安全读取 last_message_time_ms: 实例不存在时返回 nil
local function safe_last_msg_ms(instance)
    local n = gps:num_sensors() or 0
    if instance >= n then
        return nil
    end
    return gps:last_message_time_ms(instance)
end

-- 判断 UM982 模块本身是否还在通信
-- 注意: 不读 GPS2_TYPE, 把该参数的管理完全交给 primary_switch 脚本,
-- 避免两个脚本对同一参数的时序冲突 (探测期 / 写 0 期间的瞬时态).
-- 当 primary_switch 把 GPS2_TYPE 写为 0, AP_GPS 不会创建 GPS2 驱动,
-- num_sensors() 会减少到 1, 自然触发下方第一道闸门返回 false.
local function rtk_module_online()
    -- 模块未被探测出来 (num_sensors 仍只有 1), 视为整体离线
    if (gps:num_sensors() or 0) <= GPS2_INSTANCE then
        return false
    end

    -- 最近收到过消息才算在线 (now 与 last 均为 uint32_t_ud, 重载了减法/比较)
    local last = safe_last_msg_ms(GPS2_INSTANCE)
    if last == nil then return false end
    local now = millis()
    return (now - last) < MODULE_TIMEOUT_MS
end

-- 读取当前应处于的健康状态
local function read_state()
    if not rtk_module_online() then
        return H_UNKNOWN
    end

    local fix = safe_gps_status(GPS2_INSTANCE)
    if fix < MIN_FIX_STATUS then
        -- 主天线异常: 模块在线但拿不到 3D Fix
        return H_MAIN_FAIL
    end

    -- 定位 OK, 再看 yaw 是否可用
    local yaw_deg, _yaw_acc, _yaw_t = gps:gps_yaw_deg(GPS2_INSTANCE)
    if yaw_deg == nil then
        -- 副天线异常: 主天线给出定位, 但 moving baseline 算不出 heading
        return H_YAW_FAIL
    end

    return H_OK
end

local function state_name(s)
    if s == H_OK         then return "正常"     end
    if s == H_MAIN_FAIL  then return "主天线故障" end
    if s == H_YAW_FAIL   then return "副天线故障" end
    return "未知"
end

-- 根据状态向地面站播报告警 (中文; gcs:send_text 单条最大 50 字节,
-- UTF-8 中文 3 字节/字, 控制在 16 字以内)
-- 同一状态首次与重复使用完全相同的文字, 避免地面站消息列表里出现不同文案
local function send_warn(state)
    if state == H_OK then
        gcs:send_text(SEV_INFO, "RTK双天线已恢复正常")
    elseif state == H_MAIN_FAIL then
        gcs:send_text(SEV_CRITICAL, "RTK主天线故障 无定位 请检查")
    elseif state == H_YAW_FAIL then
        gcs:send_text(SEV_WARN, "RTK副天线故障 无航向 请检查")
    end
end

function update()
    local s = read_state()

    -- 模块整体离线 (H_UNKNOWN) 时不更新故障状态, 也不清空 current_state,
    -- 这样下次模块重新上线后, 还能正确播报 "恢复" 或 "故障变化".
    if s == H_UNKNOWN then
        pending_state = H_UNKNOWN
        pending_count = 0
        return update, RUN_INTERVAL_MS
    end

    -- 当前已经处于该状态: 仅在故障态下做周期性重复告警
    if s == current_state then
        pending_state = H_UNKNOWN
        pending_count = 0
        if s == H_MAIN_FAIL or s == H_YAW_FAIL then
            local now = millis()
            if (now - last_warn_ms) >= REPEAT_WARN_MS then
                send_warn(s)
                last_warn_ms = now
            end
        end
        return update, RUN_INTERVAL_MS
    end

    -- 状态变化中: 防抖动累积
    if s ~= pending_state then
        pending_state = s
        pending_count = 1
    else
        pending_count = pending_count + 1
    end

    if pending_count >= CONFIRM_COUNT then
        -- 确认状态切换: 只发一条告警, 避免与下方"迁移日志"重复刷屏
        current_state = s
        pending_state = H_UNKNOWN
        pending_count = 0
        last_warn_ms  = millis()
        send_warn(s)
    end

    return update, RUN_INTERVAL_MS
end

return update, STARTUP_DELAY_MS
