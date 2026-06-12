--[[
  脚本名称: 1gps1_gps2_yaw_primary_switch.lua
  适用机型: EFT_CAAC 飞控
              GPS1 = ublox GPS     (instance 0)
              GPS2 = UM982 双天线 RTK on SERIAL7 (instance 1)

  功能:
    1. 检测到 GPS2 (UM982) 在线且定位状态 >= 3D Fix:
         EK3_SRC1_YAW = 2  (使用 GPS 双天线 yaw)
         GPS_PRIMARY  = 1  (优先使用 GPS2)
    2. 只识别到 GPS1 (GPS2 不在线 / 状态 < 3D Fix):
         EK3_SRC1_YAW = 1  (回退使用磁罗盘 yaw)
         GPS_PRIMARY  = 0  (优先使用 GPS1)

  安全策略:
    - EK3_SRC1_YAW 是 EKF 主 yaw 源, 飞行中切换会触发 yaw 重对齐, 有姿态扰动风险.
      所以本脚本仅在 "未解锁 (Disarmed)" 时执行真正的参数切换.
      解锁后只做监控和告警, 不再改参数.
    - 加入连续 N 次满足条件才切换的防抖动逻辑, 避免 RTK 状态抖动反复改写参数.
    - 使用 param:set_and_save 一次性持久化到 flash, 切换后即便断电下次仍生效.
      只在状态真正发生变化时才写 flash, 不会反复磨损.
--]]

---@diagnostic disable: need-check-nil, cast-local-type, assign-type-mismatch, param-type-mismatch

local RUN_INTERVAL_MS = 1000   -- 检查频率 (1 秒)
local STARTUP_DELAY_MS = 5000  -- 启动后延迟 5 秒再开始判定 (等 GPS / UM982 上电稳定)
local CONFIRM_COUNT   = 5      -- 连续 5 次 (5 秒) 满足条件才切换, 防抖动

local GPS2_INSTANCE   = 1      -- GPS2 在 Lua 中的实例号 (0=GPS1, 1=GPS2)
local MIN_GPS_STATUS  = 3      -- 3=3D Fix, 4=DGPS, 5=RTK Float, 6=RTK Fixed

-- EK3_SRC1_YAW 选项 (见 AP_NavEKF_Source.h SourceYaw)
local YAW_COMPASS = 1
local YAW_GPS     = 2

-- GPS_PRIMARY 选项: 0=GPS1, 1=GPS2
local PRIMARY_GPS1 = 0
local PRIMARY_GPS2 = 1

local STATE_UNKNOWN   = 0
local STATE_GPS2_OK   = 1   -- GPS2 在线且 >= 3D Fix
local STATE_GPS1_ONLY = 2   -- GPS2 不在线 / 状态不足

local SEV_INFO = 6
local SEV_WARN = 4

local current_state = STATE_UNKNOWN
local pending_state = STATE_UNKNOWN
local pending_count = 0
local last_warn_ms = 0
local WARN_REPEAT_MS = 10000  -- 飞行中告警最少间隔 10 秒

-- 读取当前应处于的状态
local function read_state()
    local g2_status = gps:status(GPS2_INSTANCE) or 0
    if g2_status >= MIN_GPS_STATUS then
        return STATE_GPS2_OK
    end
    return STATE_GPS1_ONLY
end

-- 应用一组参数, 仅在值不同时写 flash
local function set_param_if_diff(name, value)
    local cur = param:get(name)
    if cur == nil or cur ~= value then
        if param:set_and_save(name, value) then
            return true
        else
            gcs:send_text(SEV_WARN, string.format("Failed to set %s=%d", name, value))
        end
    end
    return false
end

local function apply_state(state)
    if state == STATE_GPS2_OK then
        local a = set_param_if_diff("EK3_SRC1_YAW", YAW_GPS)
        local b = set_param_if_diff("GPS_PRIMARY",  PRIMARY_GPS2)
        if a or b then
            gcs:send_text(SEV_INFO,
                "GPS2(UM982) RTK OK: EK3_SRC1_YAW=2(GPS), GPS_PRIMARY=1(GPS2)")
        end
    elseif state == STATE_GPS1_ONLY then
        local a = set_param_if_diff("EK3_SRC1_YAW", YAW_COMPASS)
        local b = set_param_if_diff("GPS_PRIMARY",  PRIMARY_GPS1)
        if a or b then
            gcs:send_text(SEV_WARN,
                "GPS2 not detected: EK3_SRC1_YAW=1(Compass), GPS_PRIMARY=0(GPS1)")
        end
    end
end

local function read_state_name(s)
    if s == STATE_GPS2_OK   then return "GPS2_OK"   end
    if s == STATE_GPS1_ONLY then return "GPS1_ONLY" end
    return "UNKNOWN"
end

function update()
    local s = read_state()

    -- 飞行中: 不改参数, 仅在状态变化时告警 (避免 EKF yaw 重对齐风险)
    if arming:is_armed() then
        if current_state ~= STATE_UNKNOWN and s ~= current_state then
            local now = millis()
            if (now - last_warn_ms) > WARN_REPEAT_MS then
                gcs:send_text(SEV_WARN, string.format(
                    "Armed: GPS source state changed %s -> %s (params kept)",
                    read_state_name(current_state), read_state_name(s)))
                last_warn_ms = now
            end
        end
        -- 飞行中清空 pending, 防止解锁后立即切换 (希望落地后再次确认)
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

gcs:send_text(SEV_INFO, "1gps1_gps2_yaw_primary_switch loaded")
return update, STARTUP_DELAY_MS
