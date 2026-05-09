--[[
  WS2812 / NeoPixel 外接灯条 — 定制版混合灯语 (基于 A3 & 开源标准)
  
  依据要求复刻：
  - 自检: 红绿黄连续闪烁
  - 故障: 红灯常亮
  - RC丢失: 黄灯快闪
  - 低电量: 1级红灯慢闪，2级红灯快闪
  - 罗盘异常: 红黄交替闪烁
  - RTK模式: 蓝灯闪烁
  - GPS模式: 绿灯双闪
  - 姿态模式: 黄灯慢闪
--]]

---@diagnostic disable: need-check-nil

-- ========================= 可调配置 =========================
local LED_SERVO_FUNCTION = 94  -- 对应地面站 SERVOx_FUNCTION = 94 (Scripting1)
local NUM_LEDS = 8             -- 你的灯珠数量 (请根据实际情况修改)
local BRIGHTNESS = 90          -- 全局亮度 (1～255，建议不要太高以免刺眼或过载)
local UPDATE_MS = 50           -- 刷新率（50ms = 20Hz）

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

-- 6. RTK模式: 蓝灯闪烁
local P_RTK = { {0, 0, 255, 500}, {0, 0, 0, 500} }

-- 7. GPS模式: 绿灯双闪
local P_GPS = { {0, 255, 0, 150}, {0, 0, 0, 150}, {0, 255, 0, 150}, {0, 0, 0, 800} }

-- 8. 姿态模式: 黄灯慢闪
local P_ATTI = { {255, 200, 0, 800}, {0, 0, 0, 800} }


-- ========================= 内部逻辑引擎 =========================
local led_chan = nil
local init_ok = false
local current_pattern = nil
local pattern_start_ms = 0

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

    -- 5. 指南针异常 (调用 compass:healthy(0) 检查主罗盘状态)
    local _, _, _, mag_var, _ = ahrs:get_variances()
    local mag_bad = (mag_var ~= nil and mag_var:length() > 0.75)
    if not compass:healthy(0) or mag_bad then
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