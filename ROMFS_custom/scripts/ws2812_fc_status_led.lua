--[[
  WS2812 串联LED 状态灯 v7.0

  灯语优先级 (高→低):
  A  指南针校准进行中:
       水平/侧面校准 → 蓝灯常亮
       头朝下(nosedown)校准 → 绿灯常亮
  B  校准完成后 → 红绿慢闪 (提示重启飞机)
  C  校准失败   → 红灯常亮 (提示重新校准)
  1  自检: 红绿橙循环闪烁
  2  故障: 红灯常亮
  3  RC丢失: 橙灯快闪
  4  低电压: 一级/二级红灯闪烁
  D  解锁前-GPS1无数据 → 红灯常亮 (不可起飞)
  E  解锁前-RTK航向与磁罗盘偏差>45° → 红灯常亮 (提示校准磁罗盘)
  5  指南针异常: 红橙交替闪烁
  6  RTK模式: 绿灯双闪
  7  GPS模式: 绿灯慢闪
  8  姿态模式: 橙灯慢闪

  v7.0 变更:
  - 新增指南针校准过程灯语: 水平蓝灯常亮 / 头朝下绿灯常亮 / 成功红绿慢闪 / 失败红灯常亮
  - 新增解锁前 GPS1 无数据检测 → 红灯常亮
  - 新增解锁前 RTK航向 vs 磁罗盘航向偏差 > 45° 检测 → 红灯常亮 + GCS提示
  - 保留 v6.0 全部原有逻辑
--]]

---@diagnostic disable: need-check-nil

-- ========================= 可调参数 =========================
local LED_SERVO_FUNCTION = 94  -- 对应通道 SERVOx_FUNCTION = 94 (Scripting1)
local NUM_LEDS = 8             -- 灯带的灯珠数量 (根据实际情况修改)
local BRIGHTNESS = 255         -- 全局亮度 (1~255)
local UPDATE_MS = 50           -- 刷新率 (50ms = 20Hz)
local ARM_HOLD_MS = 3000       -- 解锁成功后常亮保持时间 (毫秒)
local HDG_MISMATCH_DEG = 45.0  -- RTK/磁罗盘航向偏差告警阈值 (度)
local WARN_INTERVAL_MS = 5000  -- GCS告警重复间隔 (毫秒)
local NOSEDOWN_PITCH_DEG = -60 -- 头朝下判定阈值 (度)

-- ========================= ArduCopter 模式号 =========================
local MODE_STABILIZE   = 0
local MODE_ACRO        = 1
local MODE_ALT_HOLD    = 2
local MODE_AUTO        = 3
local MODE_GUIDED      = 4
local MODE_LOITER      = 5
local MODE_RTL         = 6
local MODE_CIRCLE      = 7
local MODE_LAND        = 9
local MODE_DRIFT       = 11
local MODE_SPORT       = 13
local MODE_POSHOLD     = 16
local MODE_BRAKE       = 17
local MODE_GUIDED_NOGPS= 20
local MODE_SMART_RTL   = 21
local MODE_FLOWHOLD    = 22
local MODE_FOLLOW      = 23
local MODE_ZIGZAG      = 24
local MODE_AUTO_RTL    = 27

-- ========================= 指南针校准状态枚举 =========================
local CAL_NOT_STARTED  = 0
local CAL_RUNNING_1    = 1
local CAL_RUNNING_2    = 2
local CAL_SUCCESS      = 3
local CAL_FAILED       = 4
local CAL_BAD_ORIENT   = 5
local CAL_BAD_RADIUS   = 6

-- ========================= 灯语模式定义 (Pattern) =========================
-- 格式: { {R, G, B, 持续时间ms}, ... }

-- 1. 自检: 红绿橙循环闪烁
local P_INIT = { {255, 0, 0, 300}, {0, 255, 0, 300}, {255, 200, 0, 300} }

-- 2. 系统故障: 红灯常亮
local P_FAULT = { {255, 0, 0, 1000} }

-- 3. RC信号丢失: 橙灯快闪
local P_RC_LOSS = { {255, 200, 0, 150}, {0, 0, 0, 150} }

-- 4. 低电压告警: 一级/二级红灯闪烁
local P_BATT_LVL1 = { {255, 0, 0, 600}, {0, 0, 0, 600} }
local P_BATT_LVL2 = { {255, 0, 0, 150}, {0, 0, 0, 150} }

-- 5. 指南针异常: 红橙交替闪烁
local P_COMPASS_ERR = { {255, 0, 0, 400}, {255, 200, 0, 400} }

-- 6. RTK模式: 绿灯双闪 (三短一长)
local P_RTK = { {0, 255, 0, 150}, {0, 0, 0, 150}, {0, 255, 0, 150}, {0, 0, 0, 800} }

-- 7. GPS模式: 绿灯慢闪
local P_GPS = { {0, 255, 0, 500}, {0, 0, 0, 500} }

-- 8. 姿态模式: 橙灯慢闪
local P_ATTI = { {255, 200, 0, 800}, {0, 0, 0, 800} }

-- A. 指南针校准中-水平/侧面: 蓝灯常亮
local P_CAL_LEVEL    = { {0, 0, 255, 1000} }

-- B. 指南针校准中-头朝下: 绿灯常亮
local P_CAL_NOSEDOWN = { {0, 255, 0, 1000} }

-- C. 校准成功: 红绿慢闪 (提示用户重启飞机)
local P_CAL_SUCCESS  = { {255, 0, 0, 800}, {0, 255, 0, 800} }

-- D. 校准失败: 红灯常亮 (提示重新校准)
local P_CAL_FAIL     = { {255, 0, 0, 1000} }

-- E. 解锁前 GPS1 无数据: 红灯常亮
local P_NO_GPS       = { {255, 0, 0, 1000} }

-- F. 解锁前 RTK/磁罗盘航向偏差 > 45°: 红灯常亮
local P_HDG_ERR      = { {255, 0, 0, 1000} }


-- ========================= 内部逻辑变量 =========================
local led_chan         = nil
local init_ok          = false
local current_pattern  = nil
local pattern_start_ms = 0

-- 解锁闪光: RTK/GPS/姿态模式下解锁成功后常亮 3 秒再恢复闪烁
local prev_armed       = false
local arm_hold_until_ms = 0
local arm_hold_rgb     = nil       -- {R, G, B}

-- 指南针校准状态跟踪
local cal_result          = nil    -- nil / "success" / "fail" (重启清零)
local cal_prev_running    = false  -- 上一周期是否有校准进行

-- GCS 告警节流计时器
local last_gps_warn_ms  = 0
local last_hdg_warn_ms  = 0


-- ========================= 工具函数 =========================

-- 是否为飞行/定点模式 (用于解锁闪光)
local function is_mode_fly_pattern(pat)
    return pat == P_RTK or pat == P_GPS or pat == P_ATTI
end

-- 取对应常亮时的 RGB (解锁闪光用)
local function solid_rgb_for_pattern(pat)
    if pat == P_RTK or pat == P_GPS then
        return 0, 255, 0
    elseif pat == P_ATTI then
        return 255, 200, 0
    end
    return nil
end

-- 驱动 WS2812
local function strip_set_send(chan, r, g, b)
    local s = BRIGHTNESS / 255.0
    r = math.floor(r * s + 0.5)
    g = math.floor(g * s + 0.5)
    b = math.floor(b * s + 0.5)
    serialLED:set_RGB(chan, -1, r, g, b)
    serialLED:send(chan)
end

local function main_battery_voltage()
    if battery:num_instances() < 1 then return nil end
    return battery:voltage(0)
end

local function param_volt(name)
    local v = param:get(name)
    if v == nil or v <= 0.5 then return nil end
    return v
end

-- 安全读取 GPS 状态 (instance 超出 num_sensors 时返回 0)
local function safe_gps_status(instance)
    local n = gps:num_sensors() or 0
    if instance >= n then return 0 end
    return gps:status(instance) or 0
end

-- 判断某个罗盘 slot 是否"启用且在线"
local function compass_slot_active(use_name, id_name)
    local use = param:get(use_name)
    local id  = param:get(id_name)
    local enabled = (use == nil) or (use >= 0.5)
    local online  = (id ~= nil) and (id > 0)
    return enabled and online
end

-- 判断某个罗盘 slot 校准偏移是否全为 0 (从未做过校准)
local function compass_slot_uncalibrated(ofs_prefix)
    local x = param:get(ofs_prefix .. "_X")
    local y = param:get(ofs_prefix .. "_Y")
    local z = param:get(ofs_prefix .. "_Z")
    if x == nil or y == nil or z == nil then return false end
    return x == 0 and y == 0 and z == 0
end

-- 统计启用并在线的罗盘数量
local function active_compass_count()
    local n = 0
    if compass_slot_active("COMPASS_USE",  "COMPASS_DEV_ID")  then n = n + 1 end
    if compass_slot_active("COMPASS_USE2", "COMPASS_DEV_ID2") then n = n + 1 end
    if compass_slot_active("COMPASS_USE3", "COMPASS_DEV_ID3") then n = n + 1 end
    return n
end

-- 检查是否有启用的罗盘未做过校准
local function any_active_compass_uncalibrated()
    if compass_slot_active("COMPASS_USE",  "COMPASS_DEV_ID")  and compass_slot_uncalibrated("COMPASS_OFS")  then return true end
    if compass_slot_active("COMPASS_USE2", "COMPASS_DEV_ID2") and compass_slot_uncalibrated("COMPASS_OFS2") then return true end
    if compass_slot_active("COMPASS_USE3", "COMPASS_DEV_ID3") and compass_slot_uncalibrated("COMPASS_OFS3") then return true end
    return false
end

-- ========================= 指南针校准状态机 =========================
-- 更新 cal_result / cal_prev_running, 返回 any_running
local function update_cal_state()
    local any_running  = false
    local any_success  = false
    local any_fail     = false

    for i = 0, 2 do
        -- 用 pcall 保护: 部分固件版本可能未导出 cal_status
        local ok, st = pcall(function() return compass:cal_status(i) end)
        if not ok then st = nil end
        if st ~= nil then
            if st == CAL_RUNNING_1 or st == CAL_RUNNING_2 then
                any_running = true
            elseif st == CAL_SUCCESS then
                any_success = true
            elseif st == CAL_FAILED or st == CAL_BAD_ORIENT or st == CAL_BAD_RADIUS then
                any_fail = true
            end
        end
    end

    if any_running then
        -- 新一轮校准开始时重置上次结果
        if cal_result ~= nil then cal_result = nil end
        cal_prev_running = true
    else
        if cal_prev_running then
            -- 刚结束校准 - 记录结果
            if any_success then
                cal_result = "success"
                gcs:send_text(6, "指南针校准成功, 请重启飞机")
            elseif any_fail then
                cal_result = "fail"
                gcs:send_text(3, "指南针校准失败, 请重新校准")
            end
            cal_prev_running = false
        end
    end

    return any_running
end

-- ========================= RTK/磁罗盘航向偏差检测 =========================
-- 返回偏差角度 (0~180) 或 nil (无法检测)
local function get_rtk_compass_diff()
    -- 获取 RTK 航向 (GPS2, UM982 双天线)
    local rtk_yaw, _, _ = gps:gps_yaw_deg(1)
    if rtk_yaw == nil then return nil end
    rtk_yaw = rtk_yaw % 360

    -- 方案A: 通过原始磁场向量计算指南针航向
    -- compass:get_field(instance) 返回 Vector3f (ArduPilot 4.3+)
    for i = 0, 2 do
        if compass:healthy(i) then
            local ok, field = pcall(function() return compass:get_field(i) end)
            if ok and field ~= nil then
                local comp_hdg = math.deg(math.atan(field:y(), field:x())) % 360
                local diff = math.abs(rtk_yaw - comp_hdg)
                if diff > 180 then diff = 360 - diff end
                return diff
            end
        end
    end

    -- 方案B: 当 EKF 使用磁罗盘航向时 (EK3_SRC1_YAW=1), AHRS 航向 ≈ 磁罗盘航向
    local ek3_yaw_src = param:get("EK3_SRC1_YAW")
    if ek3_yaw_src ~= nil and math.floor(ek3_yaw_src + 0.5) == 1 then
        local ahrs_yaw_rad = ahrs:get_yaw()
        if ahrs_yaw_rad ~= nil then
            local ahrs_hdg = math.deg(ahrs_yaw_rad) % 360
            local diff = math.abs(rtk_yaw - ahrs_hdg)
            if diff > 180 then diff = 360 - diff end
            return diff
        end
    end

    return nil
end


-- ========================= 灯语选择 (优先级由高到低) =========================
local function pick_pattern()

    -- ── A/B/C: 指南针校准状态 (每次调用都更新状态机) ──────────────────────────
    local cal_running = update_cal_state()

    if cal_running then
        -- 通过 AHRS pitch 区分水平 vs 头朝下
        local pitch_rad = ahrs:get_pitch()
        local pitch_deg = pitch_rad and math.deg(pitch_rad) or 0
        if pitch_deg < NOSEDOWN_PITCH_DEG then
            return P_CAL_NOSEDOWN  -- 头朝下: 绿灯常亮
        else
            return P_CAL_LEVEL     -- 水平/侧面: 蓝灯常亮
        end
    end

    if cal_result == "success" then
        return P_CAL_SUCCESS  -- 校准成功: 红绿慢闪, 提示重启
    end
    if cal_result == "fail" then
        return P_CAL_FAIL     -- 校准失败: 红灯常亮
    end

    -- ── 1. 自检 (AHRS 未初始化) ───────────────────────────────────────────────
    if not ahrs:initialised() then
        return P_INIT
    end

    -- ── 2. 系统故障 (EKF failsafe 或 AHRS 不健康) ─────────────────────────────
    if vehicle:has_ekf_failsafed() or not ahrs:healthy() then
        return P_FAULT
    end

    -- ── 3. RC 信号丢失 ─────────────────────────────────────────────────────────
    if not rc:has_valid_input() then
        return P_RC_LOSS
    end

    -- ── 4. 电压告警 ────────────────────────────────────────────────────────────
    local vbat = main_battery_voltage()
    local crt  = param_volt("BATT_CRT_VOLT")
    local low  = param_volt("BATT_LOW_VOLT")
    if battery:has_failsafed() or (vbat and crt and vbat <= crt) then
        return P_BATT_LVL2
    elseif vbat and low and vbat <= low then
        return P_BATT_LVL1
    end

    -- ── D. 解锁前: GPS 无数据 → 红灯常亮 ─────────────────────────────────────
    -- 仅当 GPS1 和 GPS2/RTK 都没有可用定位时才报警 (GPS2_ONLY 场景下 GPS1=0 属正常)
    if not arming:is_armed() then
        local g1_fix = safe_gps_status(0)
        local g2_fix = safe_gps_status(1)
        local rtk_yaw_pre, _, _ = gps:gps_yaw_deg(1)
        local g2_ok = (g2_fix >= 3) and (rtk_yaw_pre ~= nil)
        if g1_fix < 1 and not g2_ok then
            local now = millis()
            if (now - last_gps_warn_ms) >= WARN_INTERVAL_MS then
                gcs:send_text(3, "GPS无数据,无法解锁")
                last_gps_warn_ms = now
            end
            return P_NO_GPS
        end
    end

    -- ── E. 解锁前: RTK 航向与磁罗盘偏差 > 45° → 红灯常亮 ────────────────────
    if not arming:is_armed() then
        local hdg_diff = get_rtk_compass_diff()
        if hdg_diff ~= nil and hdg_diff > HDG_MISMATCH_DEG then
            local now = millis()
            if (now - last_hdg_warn_ms) >= WARN_INTERVAL_MS then
                gcs:send_text(3, string.format(
                    "RTK/罗盘偏差%.0f度,请校准磁罗盘", hdg_diff))
                last_hdg_warn_ms = now
            end
            return P_HDG_ERR
        end
    end

    -- ── 5. 指南针异常 (v5.1): 未安装/未校准 ───────────────────────────────────
    --    仅检查"安装/校准问题": 无可用罗盘 或 已启用罗盘未做校准
    if active_compass_count() == 0 or any_active_compass_uncalibrated() then
        return P_COMPASS_ERR
    end

    -- ── 6/7/8. 根据飞行模式和 GPS 状态选择 ────────────────────────────────────
    local mode  = vehicle:get_mode()
    local gps_i = gps:primary_sensor()
    local fix   = gps:status(gps_i)

    local pos_modes = {
        [MODE_LOITER]=true, [MODE_POSHOLD]=true, [MODE_GUIDED]=true,
        [MODE_RTL]=true, [MODE_CIRCLE]=true, [MODE_SMART_RTL]=true,
        [MODE_FOLLOW]=true, [MODE_BRAKE]=true, [MODE_AUTO]=true,
        [MODE_AUTO_RTL]=true
    }
    local atti_modes = {
        [MODE_STABILIZE]=true, [MODE_ACRO]=true, [MODE_ALT_HOLD]=true,
        [MODE_SPORT]=true, [MODE_DRIFT]=true, [MODE_GUIDED_NOGPS]=true,
    }

    -- RTK 定点: GPS 模式 且 达到 RTK 精度
    if pos_modes[mode] and
       (fix == gps.GPS_OK_FIX_3D_RTK_FIXED or fix == gps.GPS_OK_FIX_3D_RTK_FLOAT) then
        return P_RTK
    end
    -- GPS 定点 (含 RTK 无数据时回落到 GPS 模式)
    if pos_modes[mode] then
        return P_GPS
    end
    -- 姿态模式
    if atti_modes[mode] then
        return P_ATTI
    end
    return P_ATTI
end


-- ========================= 灯光驱动主循环 =========================
local function run_pattern_engine(now)
    local target_pattern = pick_pattern()

    -- 解锁闪光: 解锁瞬间在飞行模式下常亮 3 秒
    local armed = arming:is_armed()
    if armed and not prev_armed and is_mode_fly_pattern(target_pattern) then
        local r, g, b = solid_rgb_for_pattern(target_pattern)
        if r ~= nil then
            arm_hold_rgb      = { r, g, b }
            arm_hold_until_ms = now + ARM_HOLD_MS
        end
    end
    prev_armed = armed

    if arm_hold_rgb and now < arm_hold_until_ms then
        if is_mode_fly_pattern(target_pattern) then
            strip_set_send(led_chan, arm_hold_rgb[1], arm_hold_rgb[2], arm_hold_rgb[3])
            return
        end
        arm_hold_rgb      = nil
        arm_hold_until_ms = 0
    elseif arm_hold_rgb and now >= arm_hold_until_ms then
        arm_hold_rgb = nil
    end

    -- 状态切换时重置计时器
    if target_pattern ~= current_pattern then
        current_pattern  = target_pattern
        pattern_start_ms = now
    end

    if current_pattern == nil then
        strip_set_send(led_chan, 0, 0, 0)
        return
    end

    -- 计算当前 Pattern 的总周期长度
    local total_duration = 0
    for i = 1, #current_pattern do
        total_duration = total_duration + current_pattern[i][4]
    end
    if total_duration == 0 then return end

    -- 取模, 计算当前处于哪个时间片
    local t   = (now - pattern_start_ms) % total_duration
    local acc = 0
    for i = 1, #current_pattern do
        acc = acc + current_pattern[i][4]
        if t < acc then
            strip_set_send(led_chan, current_pattern[i][1],
                                     current_pattern[i][2],
                                     current_pattern[i][3])
            return
        end
    end
end


-- ========================= 主调度函数 =========================
function update()
    if not init_ok then
        local ch0 = SRV_Channels:find_channel(LED_SERVO_FUNCTION)
        if ch0 == nil then
            gcs:send_text(6, "LED: Waiting SERVOx_FUNCTION=" .. tostring(LED_SERVO_FUNCTION))
            return update, 5000
        end
        led_chan = ch0 + 1
        if not serialLED:set_num_neopixel(led_chan, NUM_LEDS) then
            return update, 5000
        end
        init_ok = true
        gcs:send_text(6, "LED: WS2812 INIT OK")
    end

    run_pattern_engine(millis())
    return update, UPDATE_MS
end

return update()
