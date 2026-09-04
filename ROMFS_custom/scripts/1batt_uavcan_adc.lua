--[[
  UAVCAN / ADC 电池脚本（单电 + 双电合一）

  按飞控默认机型参数自动分流：
    GPS2_MB_OFS_Y ≈ -0.7  → 双电串联 12S（E616 等）batt_two
    GPS2_MB_OFS_Y ≈ -0.32 → 单电 6S（X6100 等）batt_one

  使用：
    1) 只放本脚本到 APM/scripts/
    2) SCR_ENABLE = 1
    3) 双电时电池端 battery.id 设为 1 / 2
    4) 若改了 MONITOR 类型，按提示重启一次

  单电实例：BATT=脚本，BATT2=UAVCAN，BATT3=ADC
  双电实例：BATT=脚本，BATT2/3=UAVCAN，BATT4=ADC
--]]

---@diagnostic disable: param-type-mismatch

local OUT_IDX = 0
local INTERNAL_ONLY = 256

local MODE_NONE = 0
local MODE_UAVCAN = 1
local MODE_ADC = 2
local last_mode = MODE_NONE

-- 单电时是否启用 UAVCAN（false=纯 ADC）
local USE_UAVCAN_ONE = true

-- EFT_CAAC 板级 ADC 分压标定（与 defaults.parm BATT_VOLT_MULT 一致）
-- 勿沿用新建模拟监视器的 hwdef 旧默认 21，否则 30V 会显示成约 20V
local ADC_VOLT_MULT = 31
local ADC_VOLT_PIN = 10

local CFG_ONE_OFS_Y = -0.32
local CFG_TWO_OFS_Y = -0.7
local OFS_EPS = 0.02

-- 失效保护电压阈值：按实测母线电压推算串数，不再按机型分支硬编码。
--
-- 起因：X6100F 的 GPS2_MB_OFS_Y 为 -0.32，被判为「单电 6S」分支，于是每次
-- 启动都被写入 LOW=22.2 / CRT=21.6；而该机实际跑 12S（日志实测母线
-- 45.64–50.35 V，MOT_BAT_VOLT_MIN 42 / MAX 50.4）。阈值永远够不着，
-- 等于没有电池失效保护；手工改回去也会被本脚本在下次启动覆盖。
--
-- GPS2_MB_OFS_Y 是 GPS 动基线的真实 Y 偏移，不能为了改电池分支去动它。
-- 因此改为按电压判串数：6S（21–25.2 V）与 12S（42–50.4 V）区间不重叠，
-- 取 33 V 为门限即可无歧义区分。
--
-- 对分类正确的机型，本改动不改变结果：
--   6S  → 6 × 3.70 = 22.2，6 × 3.60 = 21.6   与原「单电」分支同值
--   12S → 12 × 3.70 = 44.4，12 × 3.60 = 43.2 与原「双电」分支同值
local CELL_LOW_V = 3.70   -- 带载。标准 LiPo；半固态/Li-ion 需按实际放电曲线重取
local CELL_CRT_V = 3.60   -- 带载。高于 MOT_BAT_VOLT_MIN 的每片值（42/12 = 3.50）
local S_DETECT_V = 33.0   -- 6S 与 12S 之间的判别门限
local V_MIN_VALID = 16.0  -- 低于此值认为电压源未就绪，不做判定
local volt_thresholds_done = false

local is_two = false
-- one: BATT2=UAVCAN, BATT3=ADC
-- two: BATT2/BATT3=UAVCAN, BATT4=ADC
local UAVCAN_A = 1
local UAVCAN_B = 2
local ADC_IDX = 2
local adc_mult_param = "BATT3_VOLT_MULT"
local adc_pin_param = "BATT3_VOLT_PIN"

local function set_param(name, value)
    local cur = param:get(name)
    if cur == nil then
        if param:set_and_save(name, value) then
            return true
        end
        return false
    end
    if math.abs(cur - value) < 1e-4 then
        return false
    end
    param:set_and_save(name, value)
    return true
end

-- 每次循环兜底：BATT3/4 模拟后端参数要 MONITOR=3 且重启后才出现
local function ensure_adc_scale()
    set_param(adc_mult_param, ADC_VOLT_MULT)
    set_param(adc_pin_param, ADC_VOLT_PIN)
end

local function near(a, b)
    return math.abs((a or 0) - b) <= OFS_EPS
end

-- 按实测母线电压设定 LOW/CRT 阈值，只设一次。
-- 在 update 循环里调用而不是 apply_params_*()：启动那一刻电池后端未必就绪，
-- 拿不到有效电压；等到第一次读到可信电压再写，避免按 0 V 误判成 6S。
local function apply_volt_thresholds(v)
    if volt_thresholds_done then
        return
    end
    if v == nil or v < V_MIN_VALID then
        return
    end
    local cells = (v > S_DETECT_V) and 12 or 6
    local low = cells * CELL_LOW_V
    local crt = cells * CELL_CRT_V
    set_param("BATT_LOW_VOLT", low)
    set_param("BATT_CRT_VOLT", crt)
    volt_thresholds_done = true
    gcs:send_text(6, string.format("BATT: %dS by %.1fV, LOW=%.1f CRT=%.1f", cells, v, low, crt))
end

local function detect_mode()
    local ofs_y = param:get("GPS2_MB_OFS_Y")
    if near(ofs_y, CFG_TWO_OFS_Y) then
        is_two = true
        ADC_IDX = 3
        adc_mult_param = "BATT4_VOLT_MULT"
        adc_pin_param = "BATT4_VOLT_PIN"
        gcs:send_text(6, string.format("BATT: two (GPS2_MB_OFS_Y=%.3f)", ofs_y or 0))
        return true
    end
    if near(ofs_y, CFG_ONE_OFS_Y) then
        is_two = false
        ADC_IDX = 2
        adc_mult_param = "BATT3_VOLT_MULT"
        adc_pin_param = "BATT3_VOLT_PIN"
        gcs:send_text(6, string.format("BATT: one (GPS2_MB_OFS_Y=%.3f)", ofs_y or 0))
        return true
    end
    gcs:send_text(2, string.format("BATT: unknown GPS2_MB_OFS_Y=%.3f, script stop", ofs_y or 0))
    return false
end

local function apply_params_one()
    local capacity = param:get("BATT_CAPACITY") or 22000
    local changed = false

    if USE_UAVCAN_ONE then
        changed = set_param("CAN_P1_DRIVER", 1) or changed
        changed = set_param("CAN_D1_PROTOCOL", 1) or changed
    end

    changed = set_param("BATT_MONITOR", 29) or changed
    changed = set_param("BATT_CAPACITY", capacity) or changed
    -- LOW/CRT 阈值改由 apply_volt_thresholds() 按实测母线电压设定，
    -- 不再按机型分支硬编码 —— 原值 22.2/21.6 在跑 12S 的 X6100F 上永远触发不了
    changed = set_param("BATT_LOW_TIMER", 10) or changed
    changed = set_param("BATT_ARM_VOLT", 0) or changed

    if USE_UAVCAN_ONE then
        changed = set_param("BATT2_MONITOR", 8) or changed
        changed = set_param("BATT2_SERIAL_NUM", 1) or changed
        changed = set_param("BATT2_CAPACITY", capacity) or changed
        changed = set_param("BATT2_LOW_VOLT", 0) or changed
        changed = set_param("BATT2_CRT_VOLT", 0) or changed
        changed = set_param("BATT2_OPTIONS", INTERNAL_ONLY) or changed
    else
        changed = set_param("BATT2_MONITOR", 0) or changed
    end

    changed = set_param("BATT3_MONITOR", 3) or changed
    changed = set_param("BATT3_LOW_VOLT", 0) or changed
    changed = set_param("BATT3_CRT_VOLT", 0) or changed
    changed = set_param("BATT3_OPTIONS", INTERNAL_ONLY) or changed
    changed = set_param("BATT4_MONITOR", 0) or changed
    ensure_adc_scale()

    if changed then
        gcs:send_text(4, "BATT one: params saved, reboot if MONITOR changed")
    else
        gcs:send_text(6, "BATT one: params ok")
    end
end

local function apply_params_two()
    local capacity = param:get("BATT_CAPACITY") or 22000
    local changed = false

    changed = set_param("CAN_P1_DRIVER", 1) or changed
    changed = set_param("CAN_D1_PROTOCOL", 1) or changed

    changed = set_param("BATT_MONITOR", 29) or changed
    changed = set_param("BATT_CAPACITY", capacity) or changed
    -- LOW/CRT 阈值改由 apply_volt_thresholds() 按实测母线电压设定；
    -- 12S 下算得 44.4/43.2，与此处原硬编码同值
    changed = set_param("BATT_LOW_TIMER", 10) or changed
    changed = set_param("BATT_ARM_VOLT", 0) or changed

    changed = set_param("BATT2_MONITOR", 8) or changed
    changed = set_param("BATT2_SERIAL_NUM", 1) or changed
    changed = set_param("BATT2_CAPACITY", capacity) or changed
    changed = set_param("BATT2_LOW_VOLT", 0) or changed
    changed = set_param("BATT2_CRT_VOLT", 0) or changed
    changed = set_param("BATT2_OPTIONS", INTERNAL_ONLY) or changed

    changed = set_param("BATT3_MONITOR", 8) or changed
    changed = set_param("BATT3_SERIAL_NUM", 2) or changed
    changed = set_param("BATT3_CAPACITY", capacity) or changed
    changed = set_param("BATT3_LOW_VOLT", 0) or changed
    changed = set_param("BATT3_CRT_VOLT", 0) or changed
    changed = set_param("BATT3_OPTIONS", INTERNAL_ONLY) or changed

    changed = set_param("BATT4_MONITOR", 3) or changed
    changed = set_param("BATT4_LOW_VOLT", 0) or changed
    changed = set_param("BATT4_CRT_VOLT", 0) or changed
    changed = set_param("BATT4_OPTIONS", INTERNAL_ONLY) or changed
    ensure_adc_scale()

    if changed then
        gcs:send_text(4, "BATT two: params saved, reboot if MONITOR changed")
    else
        gcs:send_text(6, "BATT two: params ok")
    end
end

local function update_one()
    ensure_adc_scale()
    local state = BattMonitorScript_State()
    local can_ok = USE_UAVCAN_ONE and battery:healthy(UAVCAN_A)
    local adc_ok = battery:healthy(ADC_IDX)

    if can_ok then
        local v = battery:voltage(UAVCAN_A) or 0
        state:healthy(true)
        state:voltage(v)
        apply_volt_thresholds(v)

        local i = battery:current_amps(UAVCAN_A)
        if i then
            state:current_amps(i)
        end
        local s = battery:capacity_remaining_pct(UAVCAN_A)
        if s then
            state:capacity_remaining_pct(s)
        end
        local t = battery:get_temperature(UAVCAN_A)
        if t then
            state:temperature(t)
        end

        if last_mode ~= MODE_UAVCAN then
            gcs:send_text(6, "BATT: UAVCAN")
            last_mode = MODE_UAVCAN
        end
    elseif adc_ok then
        local v = battery:voltage(ADC_IDX) or 0
        state:healthy(true)
        state:voltage(v)
        apply_volt_thresholds(v)
        local i = battery:current_amps(ADC_IDX)
        if i then
            state:current_amps(i)
        end
        if last_mode ~= MODE_ADC then
            gcs:send_text(4, "BATT: ADC")
            last_mode = MODE_ADC
        end
    else
        state:healthy(false)
        state:voltage(0)
        if last_mode ~= MODE_NONE then
            gcs:send_text(2, "BATT: no source")
            last_mode = MODE_NONE
        end
    end

    battery:handle_scripting(OUT_IDX, state)
    return update_one, 200
end

local function update_two()
    ensure_adc_scale()
    local state = BattMonitorScript_State()
    local a_ok = battery:healthy(UAVCAN_A)
    local b_ok = battery:healthy(UAVCAN_B)
    local adc_ok = battery:healthy(ADC_IDX)

    if a_ok and b_ok then
        local va = battery:voltage(UAVCAN_A) or 0
        local vb = battery:voltage(UAVCAN_B) or 0
        state:healthy(true)
        state:voltage(va + vb)
        apply_volt_thresholds(va + vb)

        local ia = battery:current_amps(UAVCAN_A)
        local ib = battery:current_amps(UAVCAN_B)
        if ia then
            state:current_amps(ia)
        elseif ib then
            state:current_amps(ib)
        end

        local sa = battery:capacity_remaining_pct(UAVCAN_A)
        local sb = battery:capacity_remaining_pct(UAVCAN_B)
        if sa and sb then
            state:capacity_remaining_pct(math.min(sa, sb))
        elseif sa then
            state:capacity_remaining_pct(sa)
        elseif sb then
            state:capacity_remaining_pct(sb)
        end

        local ta = battery:get_temperature(UAVCAN_A)
        local tb = battery:get_temperature(UAVCAN_B)
        if ta and tb then
            state:temperature(math.max(ta, tb))
        elseif ta then
            state:temperature(ta)
        elseif tb then
            state:temperature(tb)
        end

        if last_mode ~= MODE_UAVCAN then
            gcs:send_text(6, "BATT: UAVCAN series")
            last_mode = MODE_UAVCAN
        end
    elseif adc_ok then
        local v = battery:voltage(ADC_IDX) or 0
        state:healthy(true)
        state:voltage(v)
        apply_volt_thresholds(v)
        if last_mode ~= MODE_ADC then
            gcs:send_text(4, "BATT: fallback ADC")
            last_mode = MODE_ADC
        end
    else
        state:healthy(false)
        state:voltage(0)
        if last_mode ~= MODE_NONE then
            gcs:send_text(2, "BATT: no source")
            last_mode = MODE_NONE
        end
    end

    battery:handle_scripting(OUT_IDX, state)
    return update_two, 200
end

if not detect_mode() then
    return
end

if is_two then
    apply_params_two()
    return update_two, 1000
end

apply_params_one()
return update_one, 1000
