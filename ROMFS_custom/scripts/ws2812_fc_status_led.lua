--[[
  WS2812 串联LED 状态灯 v7.4

  灯语优先级 (高→低):
  A  指南针校准进行中:
       水平/侧面校准 → 蓝灯常亮
       头朝下(nosedown)校准 → 绿灯常亮
  B  校准完成后 → 红绿慢闪 (提示重启飞机)
  C  校准失败   → 红灯常亮 (提示重新校准)
  1  自检: 红绿黄循环闪烁
  2  故障: 红灯常亮
  3  RC丢失: 黄灯快闪
  4  低电压: 一级/二级红灯闪烁
  D  解锁前-GPS1无数据 → 红灯常亮 (不可起飞；LED_GPSARM=0 可关)
  E  解锁前-RTK航向与磁罗盘偏差>45° → 红灯常亮 (提示校准磁罗盘)
  5  指南针异常: 红黄交替闪烁
  6  RTK模式: 绿灯双闪
  7  GPS模式: 绿灯慢闪
  8  姿态模式: 黄灯慢闪

  v7.0 变更:
  - 新增指南针校准过程灯语: 水平蓝灯常亮 / 头朝下绿灯常亮 / 成功红绿慢闪 / 失败红灯常亮
  - 新增解锁前 GPS1 无数据检测 → 红灯常亮
  - 新增解锁前 RTK航向 vs 磁罗盘航向偏差 > 45° 检测 → 红灯常亮 + GCS提示
  - 保留 v6.0 全部原有逻辑
  v7.1 变更:
  - 上述两个解锁前条件 (GPS1 无数据 / RTK 航向偏差>45°) 接入 prearm 授权,
    通过 arming:set_aux_auth_failed 真正禁止解锁 (不再只是红灯提示)。
    GPS1 无数据时即使 RTK 正常也无法解锁; 解锁后该授权不再生效。
  v7.2 变更 (修复校准灯语从未生效):
  - 此前用的 compass:cal_status / compass:get_field 在本固件 Lua 绑定中不存在,
    调用全被 pcall 吞掉, 导致校准蓝/绿/红绿灯语从来不会显示。
  - 固件新增 compass:get_cal_status / compass:get_cal_completion_pct 绑定后,
    本脚本改用真实校准状态; 并修正状态枚举 (与 MAG_CAL_STATUS 一致):
      2=采样(STEP1) 3=拟合(STEP2) 4=成功 5=失败 6=朝向错 7=半径错。
  - 水平/头朝下用固件完成进度区分 (与 CompassCalibrator 两阶段精确对应):
      <50% 水平→蓝灯常亮; >=50% 头朝下→绿灯常亮 (不再用 pitch 角度猜测)。
  v7.3 变更 (修复 RTK 预热期间误报指南针异常):
  - gps1_gps2_yaw_primary_switch.lua 在 RTK_WARMUP / COMP_DLY 期间会故意 COMPASS_USE=0,
    旧逻辑 active_compass_count()==0 一律判为指南针异常 → 红黄交替闪 (误报)。
  - 现识别 "EK3_SRC1_YAW=2 且 COMPASS_USE=0" 为 RTK/GPS 航向下有意关罗盘,
    不再触发指南针异常灯语, 改按飞行模式/GPS 状态显示正常灯语。
  v7.4 变更:
  - 新增 LED_GPSARM：1=GPS1无数据禁止解锁(默认)，0=关闭该检查(可无GPS1解锁)；
    RTK/罗盘航向偏差>45° 仍照常拦解锁。地面站搜索 LED_ 即可。
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

-- LED_GPSARM: 1=开启GPS1解锁检查(默认)，0=关闭(GPS1没接也能解锁)
-- 参数表 key=99；占用: 91=UOM, 96=LNDS, 97=RIDHB, 98=NFZ, 200=GPSYS
local LED_PARAM_KEY = 99
local led_gpsarm_param = nil
do
    local ok_t = param:add_table(LED_PARAM_KEY, "LED_", 1)
    local ok_p = ok_t and param:add_param(LED_PARAM_KEY, 1, "GPSARM", 1)
    if ok_p then
        led_gpsarm_param = Parameter("LED_GPSARM")
    else
        gcs:send_text(4, "LED: LED_GPSARM param init failed, GPS1 arm check stays on")
    end
end

local function gps1_arm_check_enabled()
    if led_gpsarm_param == nil then
        return true
    end
    local v = led_gpsarm_param:get()
    return (v == nil) or (v >= 1)
end
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
-- 必须与 CompassCalibrator::Status (= MAVLink MAG_CAL_STATUS) 完全一致:
local CAL_NOT_STARTED   = 0
local CAL_WAITING       = 1
local CAL_RUNNING_STEP1 = 2   -- 采样阶段 (本固件: 内含"水平"+"头朝下"两个子阶段)
local CAL_RUNNING_STEP2 = 3   -- 内部椭球拟合阶段 (耗时短, 无需用户动作)
local CAL_SUCCESS       = 4
local CAL_FAILED        = 5
local CAL_BAD_ORIENT    = 6
local CAL_BAD_RADIUS    = 7

-- 本固件 (CompassCalibrator.cpp) 把"水平旋转"和"头朝下旋转"都放在 STEP1 里,
-- 用内部 _phase 区分, 而 _phase 不外露。完成进度 completion_pct:
--   0  ~ 50%  → 水平阶段 (phase 0)
--   50 ~ 95%  → 头朝下阶段 (phase 1)
-- 因此用进度百分比来区分蓝灯(水平)/绿灯(头朝下), 与固件阶段精确对应。
local CAL_PHASE_SPLIT_PCT = 50.0

-- ========================= 灯语模式定义 (Pattern) =========================
-- 格式: { {R, G, B, 持续时间ms}, ... }

-- 1. 自检: 红绿黄循环闪烁
local P_INIT = { {255, 0, 0, 300}, {0, 255, 0, 300}, {255, 255, 0, 300} }

-- 2. 系统故障: 红灯常亮
local P_FAULT = { {255, 0, 0, 1000} }

-- 3. RC信号丢失: 黄灯快闪
local P_RC_LOSS = { {255, 255, 0, 150}, {0, 0, 0, 150} }

-- 4. 低电压告警: 一级/二级红灯闪烁
local P_BATT_LVL1 = { {255, 0, 0, 600}, {0, 0, 0, 600} }
local P_BATT_LVL2 = { {255, 0, 0, 150}, {0, 0, 0, 150} }

-- 5. 指南针异常: 红黄交替闪烁
local P_COMPASS_ERR = { {255, 0, 0, 400}, {255, 255, 0, 400} }

-- 6. RTK模式: 绿灯双闪 (三短一长)
local P_RTK = { {0, 255, 0, 150}, {0, 0, 0, 150}, {0, 255, 0, 150}, {0, 0, 0, 800} }

-- 7. GPS模式: 绿灯慢闪
local P_GPS = { {0, 255, 0, 500}, {0, 0, 0, 500} }

-- 8. 姿态模式: 黄灯慢闪
local P_ATTI = { {255, 255, 0, 800}, {0, 0, 0, 800} }

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

-- 解锁授权 (prearm 真正拦截): GPS1 无数据 / RTK 航向偏差 > 45° 时禁止解锁
local arm_auth_id = nil          -- 懒加载: 在 update 中反复尝试获取, 直到成功
local last_auth_warn_ms = 0


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
        return 255, 255, 0
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

-- COMPASS_USE=0 是否为 RTK/GPS 航向模式下的有意关闭 (非故障)
-- gps1_gps2_yaw_primary_switch.lua 在 RTK_WARMUP 及 GPSYS_COMP_DLY 期间会关 COMPASS_USE,
-- 此时不应触发 "指南针异常" 红黄交替灯语.
local function compass_intentionally_disabled()
    local use = param:get("COMPASS_USE")
    if use ~= nil and use >= 0.5 then
        return false
    end
    local ek3_yaw_src = param:get("EK3_SRC1_YAW")
    return ek3_yaw_src ~= nil and math.floor(ek3_yaw_src + 0.5) == 2
end

-- 检查是否有启用的罗盘未做过校准
local function any_active_compass_uncalibrated()
    if compass_slot_active("COMPASS_USE",  "COMPASS_DEV_ID")  and compass_slot_uncalibrated("COMPASS_OFS")  then return true end
    if compass_slot_active("COMPASS_USE2", "COMPASS_DEV_ID2") and compass_slot_uncalibrated("COMPASS_OFS2") then return true end
    if compass_slot_active("COMPASS_USE3", "COMPASS_DEV_ID3") and compass_slot_uncalibrated("COMPASS_OFS3") then return true end
    return false
end

-- ========================= 指南针校准状态机 =========================
-- 更新 cal_result / cal_prev_running, 返回 (any_running, running_pct)
--   any_running : 是否有罗盘正处于采样阶段 (STEP1, 需要用户旋转)
--   running_pct : 正在采样罗盘的 completion_pct (用于区分水平/头朝下)
local function update_cal_state()
    local any_running  = false
    local running_pct  = 0
    local any_success  = false
    local any_fail     = false

    for i = 0, 2 do
        local st = compass:get_cal_status(i)
        if st ~= nil then
            -- 采样阶段 (STEP1): 需要用户旋转, 用进度区分水平/头朝下
            -- STEP2 (内部拟合) 也算"进行中", 但很快, 归入采样末段处理
            if st == CAL_RUNNING_STEP1 or st == CAL_RUNNING_STEP2 then
                any_running = true
                local pct = compass:get_cal_completion_pct(i)
                if pct ~= nil and pct > running_pct then
                    running_pct = pct
                end
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

    return any_running, running_pct
end

-- ========================= RTK/磁罗盘航向偏差检测 =========================
-- 返回偏差角度 (0~180) 或 nil (无法检测)
local function get_rtk_compass_diff()
    -- 获取 RTK 航向 (GPS2, UM982 双天线)
    local rtk_yaw, _, _ = gps:gps_yaw_deg(1)
    if rtk_yaw == nil then return nil end
    rtk_yaw = rtk_yaw % 360

    -- 当 EKF 使用磁罗盘航向时 (EK3_SRC1_YAW=1), AHRS 航向 ≈ 磁罗盘航向,
    -- 与 RTK 航向比较即可反映两者偏差。
    -- (注: 本固件 Lua compass 绑定无 get_field, 故不直接取原始磁场航向)
    local ek3_yaw_src = param:get("EK3_SRC1_YAW")
    if ek3_yaw_src ~= nil and math.floor(ek3_yaw_src + 0.5) == 1 then
        local ahrs_yaw_rad = ahrs:get_yaw_rad()
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
    local cal_running, cal_pct = update_cal_state()

    if cal_running then
        -- 用固件上报的完成进度区分水平/头朝下 (与 CompassCalibrator 阶段精确对应):
        --   < 50%  → 水平旋转阶段 → 蓝灯常亮
        --   >= 50% → 头朝下旋转阶段 → 绿灯常亮
        if cal_pct >= CAL_PHASE_SPLIT_PCT then
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

    -- ── 4. 电压告警（不锁存：只看当前电压，电压回升后灯语恢复）────────────────
    local vbat = main_battery_voltage()
    local crt  = param_volt("BATT_CRT_VOLT")
    local low  = param_volt("BATT_LOW_VOLT")
    if vbat and crt and vbat <= crt then
        return P_BATT_LVL2
    elseif vbat and low and vbat <= low then
        return P_BATT_LVL1
    end

    -- ── D. 解锁前: GPS1 无数据 → 红灯常亮 ────────────────────────────────────
    -- GPS1 (ublox) 必须在线, 即使 RTK 正常也要求 GPS1 有数据才允许解锁
    -- LED_GPSARM=0 时跳过此检查（灯语与拦截同步关闭）
    if gps1_arm_check_enabled() and not arming:is_armed() then
        if safe_gps_status(0) < 1 then
            local now = millis()
            if (now - last_gps_warn_ms) >= WARN_INTERVAL_MS then
                gcs:send_text(3, "GPS1无数据,无法解锁")
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
    --    排除 RTK 模式下 gps 脚本有意关闭 COMPASS_USE 的情况 (v7.3)
    if not compass_intentionally_disabled() then
        if active_compass_count() == 0 or any_active_compass_uncalibrated() then
            return P_COMPASS_ERR
        end
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


-- ========================= 解锁授权 (prearm 真正拦截) =========================
-- 真正禁止解锁的条件 (与红灯灯语一致):
--   1. GPS1 (ublox) 无数据 → 即使 RTK 正常也禁止解锁 (LED_GPSARM=0 可关)
--   2. RTK 航向与磁罗盘偏差 > 45° → 禁止解锁, 提示校准磁罗盘
local function update_arming_auth()
    -- 懒加载授权 ID: 脚本加载瞬间 arming 可能未就绪, 这里反复重试直到拿到。
    -- (旧版在脚本加载时只取一次, 若那一刻失败则永久 nil, 导致拦截彻底失效)
    if arm_auth_id == nil then
        arm_auth_id = arming:get_aux_auth_id()
        if arm_auth_id == nil then
            -- 仍拿不到 (授权槽被占满): 周期性告警, 不再静默
            local now = millis()
            if (now - last_auth_warn_ms) >= 10000 then
                gcs:send_text(3, "LED: no arm auth slot, cannot block arming")
                last_auth_warn_ms = now
            end
            return
        end
    end

    -- 已解锁后无需再判 (aux auth 仅作用于 prearm)
    if arming:is_armed() then
        arming:set_aux_auth_passed(arm_auth_id)
        return
    end

    -- 条件 1: GPS1 无数据 (LED_GPSARM=0 时跳过)
    if gps1_arm_check_enabled() and safe_gps_status(0) < 1 then
        arming:set_aux_auth_failed(arm_auth_id, "GPS1 no data, fix GPS1")
        return
    end

    -- 条件 2: RTK 航向与磁罗盘偏差 > 45°
    local hdg_diff = get_rtk_compass_diff()
    if hdg_diff ~= nil and hdg_diff > HDG_MISMATCH_DEG then
        arming:set_aux_auth_failed(arm_auth_id,
            string.format("RTK/compass %.0fdeg, recal compass", hdg_diff))
        return
    end

    arming:set_aux_auth_passed(arm_auth_id)
end


-- ========================= 主调度函数 =========================
function update()
    -- 解锁授权判断必须最先执行, 且与 LED 硬件初始化完全解耦:
    -- 即使 SERVOx_FUNCTION 未配置 / 灯带初始化失败, GPS 缺失也要能真正禁止解锁。
    update_arming_auth()

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
    end

    run_pattern_engine(millis())
    return update, UPDATE_MS
end

return update()