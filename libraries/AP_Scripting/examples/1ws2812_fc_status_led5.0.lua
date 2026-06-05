--[[
  WS2812 / NeoPixel 外接灯条 — 定制版混合灯语 v5.0 (基于 A3 & 开源标准)
  
  依据要求复刻：
  - 自检: 红绿黄连续闪烁
  - 故障: 红灯常亮
  - RC丢失: 黄灯快闪
  - 低电量: 1级红灯慢闪，2级红灯快闪
  - 罗盘异常: 红黄交替闪烁
  - RTK模式: 绿灯双闪
  - GPS模式: 绿灯慢闪
  - 姿态模式: 黄灯慢闪
  - 上述三种模式下解锁成功: 对应颜色常亮 3 秒后恢复闪烁

  v5.0 改动:
  - 罗盘异常的判定条件与地面站 "Bad Compass Health" 保持一致 (compass:healthy(0))
    移除了原 v4.0 中过于敏感的 EKF 磁航向新息方差 (mag_var > 0.75) 判定，避免误报
--]]

---@diagnostic disable: need-check-nil

-- ========================= 可调配置 =========================
local LED_SERVO_FUNCTION = 94  -- 对应地面站 SERVOx_FUNCTION = 94 (Scripting1)
local NUM_LEDS = 8             -- 你的灯珠数量 (请根据实际情况修改)
local BRIGHTNESS = 90          -- 全局亮度 (1～255，建议不要太高以免刺眼或过载)
local UPDATE_MS = 50           -- 刷新率（50ms = 20Hz）
local ARM_HOLD_MS = 3000       -- 解锁成功后常亮保持时长（毫秒）

-- ========================= ArduCopter 模式号 =========================
local MODE_STABILIZE = 0
local MODE_ACRO = 1
local MODE_ALT_HOLD = 2
local MODE_AUTO = 3
local MODE_GUIDED = 4
local MODE_LOITER = 5
local MODE_RTL = 6
local MODE_CIRCLE = 7
local MODE_LAND = 9
local MODE_DRIFT = 11
local MODE_SPORT = 13
local MODE_POSHOLD = 16
local MODE_BRAKE = 17
local MODE_GUIDED_NOGPS = 20
local MODE_SMART_RTL = 21
local MODE_FLOWHOLD = 22
local MODE_FOLLOW = 23
local MODE_ZIGZAG = 24
local MODE_AUTO_RTL = 27

-- ========================= 灯语序列定义 (Pattern) =========================
-- 格式: { {R, G, B, 持续时间ms}, {R, G, B, 持续时间ms}, ... }

-- 1. 自检: 红绿黄连续闪烁
local P_INIT = { {255, 0, 0, 300}, {0, 255, 0, 300}, {255, 200, 0, 300} }

-- 2. 故障报警: 红灯常亮
local P_FAULT = { {255, 0, 0, 1000} }

-- 3. 遥控器丢失: 黄灯快闪
local P_RC_LOSS = { {255, 200, 0, 150}, {0, 0, 0, 150} }

-- 4. 低电压报警: 一级红灯慢闪 / 二级红灯快闪
local P_BATT_LVL1 = { {255, 0, 0, 600}, {0, 0, 0, 600} }
local P_BATT_LVL2 = { {255, 0, 0, 150}, {0, 0, 0, 150} }

-- 5. 指南针异常: 红黄交替闪烁
local P_COMPASS_ERR = { {255, 0, 0, 400}, {255, 200, 0, 400} }

-- 6. RTK模式: 绿灯双闪（测试用）
local P_RTK = { {0, 255, 0, 150}, {0, 0, 0, 150}, {0, 255, 0, 150}, {0, 0, 0, 800} }

-- 7. GPS模式: 绿灯慢闪（测试用）
local P_GPS = { {0, 255, 0, 500}, {0, 0, 0, 500} }

-- 8. 姿态模式: 黄灯慢闪
local P_ATTI = { {255, 200, 0, 800}, {0, 0, 0, 800} }


-- ========================= 内部逻辑引擎 =========================
local led_chan = nil
local init_ok = false
local current_pattern = nil
local pattern_start_ms = 0

-- 解锁常亮：仅在 RTK / GPS / 姿态 三种灯语下，检测上锁→解锁边沿
local prev_armed = false
local arm_hold_until_ms = 0
local arm_hold_rgb = nil       -- {R, G, B}，解锁瞬间锁定，3 秒内不变

-- 是否为三种“飞行模式”灯语（非故障/低压等更高优先级）
local function is_mode_fly_pattern(pat)
    return pat == P_RTK or pat == P_GPS or pat == P_ATTI
end

-- 取该灯语常亮时的 RGB（与闪烁亮段颜色一致）
local function solid_rgb_for_pattern(pat)
    if pat == P_RTK or pat == P_GPS then
        return 0, 255, 0
    elseif pat == P_ATTI then
        return 255, 200, 0
    end
    return nil
end

-- 物理输出到 WS2812
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

-- 状态机：按优先级选择当前应当显示的 Pattern
local function pick_pattern()
    -- 1. 自检 (Booting / AHRS未就绪)
    if not ahrs:initialised() then
        return P_INIT
    end

    -- 2. 故障报警 (EKF failsafe 或 姿态完全不健康)
    if vehicle:has_ekf_failsafed() or not ahrs:healthy() then
        return P_FAULT
    end

    -- 3. 遥控器丢失
    if not rc:has_valid_input() then
        return P_RC_LOSS
    end

    -- 4. 低电压报警
    local vbat = main_battery_voltage()
    local crt = param_volt("BATT_CRT_VOLT")
    local low = param_volt("BATT_LOW_VOLT")
    
    if battery:has_failsafed() or (vbat and crt and vbat <= crt) then
        return P_BATT_LVL2  -- 临界电压快闪
    elseif vbat and low and vbat <= low then
        return P_BATT_LVL1  -- 一级低压慢闪
    end

    -- 5. 指南针异常: 判定条件与地面站 "Bad Compass Health" 保持一致
    -- 地面站源码 (GCS_Common.cpp): control_sensors_health 只依赖 compass.healthy()
    -- compass.healthy() 内部实现: (time - last_update_ms < 500)
    -- 这里只用主罗盘 healthy 标志，避免脚本因 EKF 磁航向新息抖动误报
    if not compass:healthy(0) then
        return P_COMPASS_ERR
    end

    -- 获取当前飞行模式与 GPS 状态
    local mode = vehicle:get_mode()
    local gps_i = gps:primary_sensor()
    local fix = gps:status(gps_i)

    -- 定义需要卫星的模式 (GPS 模式组)
    local pos_modes = {
        [MODE_LOITER] = true, [MODE_POSHOLD] = true, [MODE_GUIDED] = true,
        [MODE_RTL] = true, [MODE_CIRCLE] = true, [MODE_SMART_RTL] = true,
        [MODE_FOLLOW] = true, [MODE_BRAKE] = true, [MODE_AUTO] = true, [MODE_AUTO_RTL] = true
    }

    -- 定义纯姿态模式组
    local atti_modes = {
        [MODE_STABILIZE] = true, [MODE_ACRO] = true, [MODE_ALT_HOLD] = true,
        [MODE_SPORT] = true, [MODE_DRIFT] = true, [MODE_GUIDED_NOGPS] = true,
    }

    -- 6. RTK模式 (处在 GPS 模式下，且达到 RTK 精度)
    if pos_modes[mode] and (fix == gps.GPS_OK_FIX_3D_RTK_FIXED or fix == gps.GPS_OK_FIX_3D_RTK_FLOAT) then
        return P_RTK
    end

    -- 7. GPS模式 (处在 GPS 模式下，常规 3D 修复)
    if pos_modes[mode] then
        return P_GPS
    end

    -- 8. 姿态模式
    if atti_modes[mode] then
        return P_ATTI
    end

    -- 默认兜底显示 (如果都不满足，则默认显示姿态模式灯语)
    return P_ATTI
end

-- 根据时间轴解析 Pattern 并输出颜色
local function run_pattern_engine(now)
    local target_pattern = pick_pattern()

    -- 解锁边沿：由未解锁→已解锁，且当前为 RTK/GPS/姿态 灯语时，启动 3 秒常亮
    local armed = arming:is_armed()
    if armed and not prev_armed and is_mode_fly_pattern(target_pattern) then
        local r, g, b = solid_rgb_for_pattern(target_pattern)
        if r ~= nil then
            arm_hold_rgb = { r, g, b }
            arm_hold_until_ms = now + ARM_HOLD_MS
        end
    end
    prev_armed = armed

    -- 常亮窗口内：仍显示解锁时颜色；若出现更高优先级灯语则让位
    if arm_hold_rgb and now < arm_hold_until_ms then
        if is_mode_fly_pattern(target_pattern) then
            strip_set_send(led_chan, arm_hold_rgb[1], arm_hold_rgb[2], arm_hold_rgb[3])
            return
        end
        -- 故障/RC丢失等打断常亮，清除保持状态
        arm_hold_rgb = nil
        arm_hold_until_ms = 0
    elseif arm_hold_rgb and now >= arm_hold_until_ms then
        arm_hold_rgb = nil
    end

    -- 如果状态发生变化，重置序列时间轴，确保灯语从头开始闪烁
    if target_pattern ~= current_pattern then
        current_pattern = target_pattern
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

    -- 取余，计算当前落在哪个时间片
    local t = (now - pattern_start_ms) % total_duration
    local acc = 0

    -- 匹配时间片并输出指定 RGB
    for i = 1, #current_pattern do
        acc = acc + current_pattern[i][4]
        if t < acc then
            strip_set_send(led_chan, current_pattern[i][1], current_pattern[i][2], current_pattern[i][3])
            return
        end
    end
end

-- 主更新函数
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

    -- 运行灯语引擎
    run_pattern_engine(millis())

    return update, UPDATE_MS
end

return update()
