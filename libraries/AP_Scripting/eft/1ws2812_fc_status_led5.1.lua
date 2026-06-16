--[[
  WS2812 外接灯条 — 灯语 v5.1
  
  依据要求复刻：
  - 自检: 红绿黄连续闪烁
  - 故障: 红灯常亮
  - RC丢失: 黄灯快闪
  - 低电量: 1级红灯慢闪，2级红灯快闪
  - 罗盘异常: 红黄交替闪烁 (仅当"一个罗盘都没有"或"罗盘未校准"时触发)
  - RTK模式: 绿灯双闪
  - GPS模式: 绿灯慢闪
  - 姿态模式: 黄灯慢闪
  - 上述三种模式下解锁成功: 对应颜色常亮 3 秒后恢复闪烁

  v5.0 改动:
  - 罗盘异常的判定条件与地面站 "Bad Compass Health" 保持一致 (compass:healthy(0))
    移除了原 v4.0 中过于敏感的 EKF 磁航向新息方差 (mag_var > 0.75) 判定，避免误报
  v5.1 改动:
  - 罗盘异常判定改为只关心"配置/校准"层面:
      * 一个罗盘都没有 (没有任何 slot 同时满足 COMPASS_USE*=1 且 COMPASS_DEV_ID*>0)
      * 或 任意启用的罗盘未校准 (COMPASS_OFS*_X/Y/Z 全为 0)
    其它情况 (例如运行中 healthy 抖动、单罗盘) 不再报警，避免飞行中频繁误报
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

-- 判断某个罗盘 slot 是否"启用且在线"
--   prefix_use: COMPASS_USE / COMPASS_USE2 / COMPASS_USE3
--   prefix_id : COMPASS_DEV_ID / COMPASS_DEV_ID2 / COMPASS_DEV_ID3
local function compass_slot_active(use_name, id_name)
    local use = param:get(use_name)
    local id  = param:get(id_name)
    -- USE 参数缺失视为默认启用 (=1)
    local enabled = (use == nil) or (use >= 0.5)
    local online  = (id ~= nil) and (id > 0)
    return enabled and online
end

-- 判断某个罗盘 slot 的校准偏移是否仍为 0 (即未做过校准)
local function compass_slot_uncalibrated(ofs_prefix)
    local x = param:get(ofs_prefix .. "_X")
    local y = param:get(ofs_prefix .. "_Y")
    local z = param:get(ofs_prefix .. "_Z")
    if x == nil or y == nil or z == nil then
        -- 参数不存在时认为"未校准"较保守，但实际 ofs 参数总是存在的
        return false
    end
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

-- 任意启用的罗盘未校准 → 视为整体未校准
local function any_active_compass_uncalibrated()
    if compass_slot_active("COMPASS_USE",  "COMPASS_DEV_ID")  and compass_slot_uncalibrated("COMPASS_OFS")  then return true end
    if compass_slot_active("COMPASS_USE2", "COMPASS_DEV_ID2") and compass_slot_uncalibrated("COMPASS_OFS2") then return true end
    if compass_slot_active("COMPASS_USE3", "COMPASS_DEV_ID3") and compass_slot_uncalibrated("COMPASS_OFS3") then return true end
    return false
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

    -- 5. 指南针异常 (v5.1):
    --    仅在以下两种"配置/校准层面"的问题下触发，避免运行中 healthy 抖动误报
    --      a) 一个罗盘都没有 (没有任何 slot 启用并在线)
    --      b) 任一启用的罗盘从未校准 (COMPASS_OFS*_X/Y/Z 全为 0)
    if active_compass_count() == 0 or any_active_compass_uncalibrated() then
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
