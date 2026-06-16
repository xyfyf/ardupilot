--[[
  脚本名称: 1gps1_gps2_yaw_primary_switch.lua
  适用机型: EFT_CAAC 飞控
              GPS1 = ublox GPS                       (instance 0)
              GPS2 = UM982 双天线 RTK on SERIAL7     (instance 1)

  功能 (3 种工作状态):
    1. GPS1 (ublox) + GPS2 (RTK) 都在线, GPS2 状态 >= 3D Fix:
         GPS1_TYPE      = 1   (Auto, 启用 ublox)
         GPS2_TYPE      = 25  (UM982 / Unicore moving baseline, 启用 RTK)
         EK3_SRC1_YAW   = 2   (使用 GPS 双天线 yaw)
         GPS_PRIMARY    = 1   (优先 GPS2)
         COMPASS_USE    = 1   (启用外置罗盘 1, ublox 外接的)
    2. 只识别到 GPS1 (GPS2 不在线 / 状态 < 3D Fix), 即 RTK 断联:
         GPS1_TYPE      = 1
         GPS2_TYPE      = 0   (禁用 GPS2, 消除 "GPS 2: not healthy" 告警)
         EK3_SRC1_YAW   = 1   (回退使用磁罗盘 yaw)
         GPS_PRIMARY    = 0   (优先 GPS1)
         COMPASS_USE    = 1   (启用外置罗盘 1)
    3. 只识别到 GPS2 RTK (ublox GPS1 被拔掉, gps:status(0)==0):
         GPS1_TYPE      = 0   (禁用 GPS1, 让飞控不再提示 "GPS 1: Bad fix" /
                               "GPS 1: not healthy")
         GPS2_TYPE      = 25  (UM982 / Unicore moving baseline, 启用 RTK)
         EK3_SRC1_YAW   = 2   (使用 GPS 双天线 yaw)
         GPS_PRIMARY    = 1   (使用 GPS2; 保证 GPS1 不是 primary,
                               否则会触发 "GPS 1: primary but TYPE 0")
         COMPASS_USE    = 0   (ublox 拔掉后外置罗盘 1 也没了, 关掉它消除
                               "Compass not healthy". COMPASS_USE2=1 保留
                               飞控板内置罗盘.)

  启动探测 (probe):
    - 脚本启动时若发现 GPS1_TYPE == 0 (上次飞行结束时被脚本写成 0),
      强制改回 1 (Auto), 让本次开机有机会重新检测到 ublox.
      因为 AP_GPS::update_instance() 在 drivers[0]==nullptr 时会调用
      detect_instance() 重新探测, 所以运行时把 GPS1_TYPE 从 0 改成 1
      也能让 ublox 被检测出来.
    - 同理, 若发现 GPS2_TYPE == 0 (上次 RTK 断联时被脚本写成 0),
      强制改回 25 (UM982), 让本次开机有机会重新检测到 RTK.
    - 经过 STARTUP_DELAY_MS + CONFIRM_COUNT 秒后仍然 gps:status(0)==0,
      且 GPS2 RTK 正常, 才会把 GPS1_TYPE 写回 0, 永久消除 GPS1 告警.
      反之 RTK 长时间未上线, 才会把 GPS2_TYPE 写回 0.

  安全策略:
    - EK3_SRC1_YAW / GPS_PRIMARY 飞行中切换可能引发 EKF yaw 重对齐,
      所以本脚本仅在 "未解锁 (Disarmed)" 时执行真正的参数切换.
      解锁后只做监控和告警, 不再改参数.
    - 防抖动: 连续 N 次 (N=5 秒) 满足条件才切换, 避免 RTK 状态抖动反复改写参数.
    - 仅在状态真正发生变化时写 flash, 不会反复磨损.
--]]

---@diagnostic disable: need-check-nil, cast-local-type, assign-type-mismatch, param-type-mismatch

local RUN_INTERVAL_MS = 1000   -- 检查频率 (1 秒)
local STARTUP_DELAY_MS = 5000  -- 启动后延迟 5 秒再开始判定 (等 GPS / UM982 上电稳定)
local CONFIRM_COUNT   = 5      -- 连续 5 次 (5 秒) 满足条件才切换, 防抖动

local GPS1_INSTANCE   = 0      -- ublox GPS1 (Lua 实例号 0)
local GPS2_INSTANCE   = 1      -- UM982 RTK GPS2 (Lua 实例号 1)
local MIN_GPS_STATUS  = 3      -- 3=3D Fix, 4=DGPS, 5=RTK Float, 6=RTK Fixed

-- EK3_SRC1_YAW 选项 (见 AP_NavEKF_Source.h SourceYaw)
local YAW_COMPASS = 1
local YAW_GPS     = 2

-- GPS_PRIMARY 选项: 0=GPS1, 1=GPS2
local PRIMARY_GPS1 = 0
local PRIMARY_GPS2 = 1

-- GPS_TYPE 选项 (见 AP_GPS::GPS_Type)
local TYPE_NONE  = 0     -- 关闭该 GPS 实例
local TYPE_AUTO  = 1     -- ublox 自动检测
local TYPE_UM982 = 25    -- UM982 / Unicore moving baseline NMEA (GPS2 RTK)

-- COMPASS_USE 选项: 0=关闭, 1=启用. 这里只动 COMPASS_USE (外置罗盘 1, ublox 自带的),
-- COMPASS_USE2 保留为 1 (飞控板内置罗盘) 避免 "Compass not healthy".
local COMPASS_OFF = 0
local COMPASS_ON  = 1

local STATE_UNKNOWN     = 0
local STATE_BOTH_OK     = 1   -- GPS1 在线 + GPS2 状态 >= 3D Fix
local STATE_GPS1_ONLY   = 2   -- GPS1 在线, GPS2 不在线 / 状态不足
local STATE_GPS2_ONLY   = 3   -- GPS1 被拔掉 (status==0), GPS2 RTK 正常

local SEV_INFO = 6
local SEV_WARN = 4

local current_state = STATE_UNKNOWN
local pending_state = STATE_UNKNOWN
local pending_count = 0
local last_warn_ms = 0
local WARN_REPEAT_MS = 10000  -- 飞行中告警最少间隔 10 秒

-- 应用一组参数, 仅在值不同时写 flash
local function set_param_if_diff(name, value)
    local cur = param:get(name)
    if cur == nil or cur ~= value then
        if param:set_and_save(name, value) then
            return true
        else
            gcs:send_text(SEV_WARN, string.format("%s set fail", name))
        end
    end
    return false
end

-- 启动探测: 如果 GPS1_TYPE 当前为 0 (上次脚本判定 RTK_only 时写入), 拉回 1 (Auto)
-- 让本次开机重新检测 ublox. 即便 ublox 仍未连接, 后面 STATE_GPS2_ONLY 判定通过后
-- 还会再写回 0.
local function probe_gps1_on_startup()
    local cur = param:get("GPS1_TYPE")
    if cur ~= nil and cur == TYPE_NONE then
        param:set_and_save("GPS1_TYPE", TYPE_AUTO)
    end
end

-- 启动探测: 如果 GPS2_TYPE 当前为 0 (上次脚本判定 RTK 断联时写入), 拉回 25 (UM982)
-- 让本次开机重新检测 RTK. 如果 RTK 仍未连接, 后面 STATE_GPS1_ONLY 判定通过后
-- 还会再写回 0.
local function probe_gps2_on_startup()
    local cur = param:get("GPS2_TYPE")
    if cur ~= nil and cur == TYPE_NONE then
        param:set_and_save("GPS2_TYPE", TYPE_UM982)
    end
end

-- 安全读取 GPS 状态: 当 instance 超过 num_sensors() 时直接返回 0,
-- 避免抛出 "bad argument #1 to 'status' (out of range)" 让脚本崩溃.
-- num_sensors() 只统计已经成功探测到驱动的 GPS, 所以启动早期 GPS2 还没被
-- 探测出来时, num_sensors() 可能只有 1, 这时直接调用 gps:status(1) 会报错.
local function safe_gps_status(instance)
    local n = gps:num_sensors() or 0
    if instance >= n then
        return 0    -- 视为 NO_GPS (还没探测到)
    end
    return gps:status(instance) or 0
end

-- 读取当前应处于的状态
local function read_state()
    local g1_status = safe_gps_status(GPS1_INSTANCE)
    local g2_status = safe_gps_status(GPS2_INSTANCE)
    local g1_present = (g1_status >= 1)               -- GPS1 硬件在线 (>=NO_FIX)
    local g2_ok      = (g2_status >= MIN_GPS_STATUS)  -- GPS2 状态足够好

    if g1_present and g2_ok then
        return STATE_BOTH_OK
    elseif g1_present then
        return STATE_GPS1_ONLY
    elseif g2_ok then
        return STATE_GPS2_ONLY
    end
    return STATE_UNKNOWN
end

local function apply_state(state)
    if state == STATE_BOTH_OK then
        -- 双 GPS 都在线: GPS2 必须先有 TYPE=25 才能正常出 yaw, 这里一并维护.
        local a = set_param_if_diff("GPS1_TYPE",    TYPE_AUTO)
        local e = set_param_if_diff("GPS2_TYPE",    TYPE_UM982)
        local b = set_param_if_diff("EK3_SRC1_YAW", YAW_GPS)
        local c = set_param_if_diff("GPS_PRIMARY",  PRIMARY_GPS2)
        local d = set_param_if_diff("COMPASS_USE",  COMPASS_ON)
        if a or b or c or d or e then
            gcs:send_text(SEV_INFO, "GPS: dual RTK")
        end
    elseif state == STATE_GPS1_ONLY then
        -- RTK 断联: 把 GPS2_TYPE 写 0 关闭该实例, 消除 "GPS 2: not healthy" /
        -- "GPS 2: Bad fix" 告警. EK3_SRC1_YAW 回退到磁罗盘.
        local a = set_param_if_diff("GPS1_TYPE",    TYPE_AUTO)
        local e = set_param_if_diff("GPS2_TYPE",    TYPE_NONE)
        local b = set_param_if_diff("EK3_SRC1_YAW", YAW_COMPASS)
        local c = set_param_if_diff("GPS_PRIMARY",  PRIMARY_GPS1)
        local d = set_param_if_diff("COMPASS_USE",  COMPASS_ON)
        if a or b or c or d or e then
            gcs:send_text(SEV_WARN, "GPS: GPS1 only")
        end
    elseif state == STATE_GPS2_ONLY then
        -- 注意写入顺序: 先把 GPS_PRIMARY 切到 1 (GPS2), 并确保 GPS2_TYPE=25,
        -- 再把 GPS1_TYPE 设为 0, 避免 "GPS 1: primary but TYPE 0" 中间态告警.
        -- COMPASS_USE 也跟着关掉, 因为 ublox 拔掉后外置罗盘 1 同样不存在.
        local e = set_param_if_diff("GPS2_TYPE",    TYPE_UM982)
        local a = set_param_if_diff("EK3_SRC1_YAW", YAW_GPS)
        local b = set_param_if_diff("GPS_PRIMARY",  PRIMARY_GPS2)
        local c = set_param_if_diff("GPS1_TYPE",    TYPE_NONE)
        local d = set_param_if_diff("COMPASS_USE",  COMPASS_OFF)
        if a or b or c or d or e then
            gcs:send_text(SEV_WARN, "GPS: RTK only")
        end
    end
end

local function read_state_name(s)
    if s == STATE_BOTH_OK    then return "BOTH_OK"        end
    if s == STATE_GPS1_ONLY  then return "GPS1_ONLY"      end
    if s == STATE_GPS2_ONLY  then return "GPS2_ONLY(RTK)" end
    return "UNKNOWN"
end

function update()
    local s = read_state()

    -- 飞行中: 不改参数, 仅在状态变化时告警 (避免 EKF yaw 重对齐风险)
    if arming:is_armed() then
        if current_state ~= STATE_UNKNOWN and s ~= STATE_UNKNOWN and s ~= current_state then
            local now = millis()
            if (now - last_warn_ms) > WARN_REPEAT_MS then
                gcs:send_text(SEV_WARN, string.format(
                    "GPS change %s->%s (armed)",
                    read_state_name(current_state), read_state_name(s)))
                last_warn_ms = now
            end
        end
        -- 飞行中清空 pending, 防止解锁后立即切换 (希望落地后再次确认)
        pending_state = STATE_UNKNOWN
        pending_count = 0
        return update, RUN_INTERVAL_MS
    end

    -- 状态未知 (双 GPS 都没有有效数据) 时不切换参数, 避免错误关掉 GPS1
    if s == STATE_UNKNOWN then
        pending_state = STATE_UNKNOWN
        pending_count = 0
        return update, RUN_INTERVAL_MS
    end

    -- 未解锁: 当前已是目标状态, 重置 pending
    if s == current_state then
        pending_state = STATE_UNKNOWN
        pending_count = 0
        return update, RUN_INTERVAL_MS
    end

    -- 未解锁: 防抖动累积
    if s ~= pending_state then
        pending_state = s
        pending_count = 1
    else
        pending_count = pending_count + 1
    end

    if pending_count >= CONFIRM_COUNT then
        apply_state(s)
        current_state = s
        pending_state = STATE_UNKNOWN
        pending_count = 0
    end

    return update, RUN_INTERVAL_MS
end

probe_gps1_on_startup()
probe_gps2_on_startup()
return update, STARTUP_DELAY_MS
