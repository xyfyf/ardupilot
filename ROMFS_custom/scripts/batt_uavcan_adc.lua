--[[
  1batt_uavcan_adc.lua — UAVCAN / ADC 电池融合脚本（单电 + 双电合一）

  【功能概述】
  将 CAN 智能电池与飞控板载 ADC 分压采样合并为一路 BATT 主输出，供 ArduPilot
  低电压告警、解锁检查、剩余电量显示等使用。地面站只看到 BATT，无需关心底层
  有几路 CAN 或 ADC 后端。

  【机型识别 — SN_PROD】
  读取产线烧录的 SN_PROD1..7，解码为 ASCII 产品型号字符串（与 dynamic_model_params.lua
  相同），按型号关键字分流：

    型号含 610  → 单电 6S（X6100 等）
    型号含 616  → 双电串联 12S（E616 等）
    SN 为空     → 每 5s 重试，不写参、不进入更新循环
    其他未知型号 → 每 5s 继续等待，不误判为 E616

  【电池实例映射】
  单电（X6100）：
    BATT  = 脚本输出（MONITOR=29）
    BATT2 = UAVCAN 智能电池，battery.id=1
    BATT3 = ADC 分压备用（MONITOR=3，pin=10，MULT=31）
    BATT4 = 关闭

  双电串联（E616）：
    BATT  = 脚本输出（MONITOR=29）
    BATT2 = 自动绑定第一块 UAVCAN 电池（不限定 battery.id / CAN node ID）
    BATT3 = 自动绑定第二块 UAVCAN 电池（不限定 battery.id / CAN node ID）
    BATT4 = ADC 分压备用（MONITOR=3，pin=10，MULT=31）

  BATT2/3/4 设 INTERNAL_ONLY(256)，不向地面站 MAVLink 上报 BATTERY_STATUS，
  但 PreArm 仍会检查其 healthy；未接 CAN 智能电池时 BATT2 须关闭(MONITOR=0)。

  【运行时数据源优先级】
  单电：UAVCAN(BATT2) 优先 → ADC(BATT3) 回退 → 均无则 unhealthy
  双电：两路 UAVCAN 均在线 → 电压相加(串联)、电流取 A 或 B、SOC 取较小值、
        温度取较大值 → 任一路 CAN 失效则 ADC(BATT4) 回退 → 均无则 unhealthy

  【自动写入的参数】
  识别机型后一次性 set_and_save：
    CAN_P1_DRIVER=1, CAN_D1_PROTOCOL=1（启用 DroneCAN）
    BATT_MONITOR=29（脚本电池）
    BATT_LOW_VOLT / BATT_CRT_VOLT：单电 22.2/21.6V，双电 44.4/43.2V
    BATT2/3/4 的 MONITOR、SERIAL_NUM、OPTIONS 等
  若 MONITOR 类型发生变化，GCS 会提示 reboot；改完后须重启飞控一次。

  【ADC 分压标定】
  EFT_CAAC 板级：VOLT_MULT=31，VOLT_PIN=10（与 defaults.parm 一致）。
  每次循环兜底写入，防止 MONITOR=3 新建实例沿用 hwdef 旧默认 21 导致读数偏低。

  【可调开关】
  BATTS_ENABLE = 1  → 启用本脚本（地面站搜 BATTS_ENABLE）；0=完全退出，不改 BATT 参数
  USE_UAVCAN_ONE = true  → 单电启用 UAVCAN；false 则单电纯 ADC，BATT2 关闭

  【GCS 提示】
  识别成功：BATT: one X6100 / two E616 (型号字符串)
  运行切换：BATT: UAVCAN / ADC / UAVCAN series / fallback ADC / no source

  【部署】
    1) 脚本放入 APM/scripts/（或 ROMFS 内置；仅 EFT_CAAC 固件编进 ROMFS）
    2) SCR_ENABLE = 1
    3) 产线须先写入 SN_PROD；与 dynamic_model_params.lua 可并行运行
    4) 双电无需固定 battery.id 或 CAN node ID，后端按收到的不同节点自动绑定
    5) 首次改 MONITOR 类型后重启飞控
--]]

---@diagnostic disable: param-type-mismatch

local PARAM_TABLE_KEY = 95
local PARAM_TABLE_PREFIX = "BATTS_"
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 1), "BATTS: add_table fail")
assert(param:add_param(PARAM_TABLE_KEY, 1, "ENABLE", 1), "BATTS: add ENABLE fail")
local batts_enable = Parameter("BATTS_ENABLE")

local function script_enabled()
    return (batts_enable:get() or 0) >= 1
end

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

local is_two = false
-- one: BATT2=UAVCAN, BATT3=ADC
-- two: BATT2/BATT3=UAVCAN, BATT4=ADC
local UAVCAN_A = 1
local UAVCAN_B = 2
local ADC_IDX = 2
local adc_mult_param = "BATT3_VOLT_MULT"
local adc_pin_param = "BATT3_VOLT_PIN"
local wait_reboot_announced = false
local uavcan_off_announced = false
local uavcan_missing_ms = nil
local UAVCAN_OFF_DELAY_MS = 15000
local boot_state = "detect"  -- detect | apply | wait_backends | run

local function required_instances()
    -- Dual: script + 2x UAVCAN (ADC fallback is optional, may be instance 3)
    if is_two then
        return 3
    end
    -- Single: script + UAVCAN or ADC
    if USE_UAVCAN_ONE then
        return 2
    end
    return 2
end

local function battery_instance_ok(idx)
    return idx < battery:num_instances()
end

local function battery_healthy(idx)
    if not battery_instance_ok(idx) then
        return false
    end
    return battery:healthy(idx)
end

local function battery_voltage(idx)
    if not battery_instance_ok(idx) then
        return nil
    end
    return battery:voltage(idx)
end

local function battery_current_amps(idx)
    if not battery_instance_ok(idx) then
        return nil
    end
    return battery:current_amps(idx)
end

local function battery_capacity_remaining_pct(idx)
    if not battery_instance_ok(idx) then
        return nil
    end
    return battery:capacity_remaining_pct(idx)
end

local function battery_get_temperature(idx)
    if not battery_instance_ok(idx) then
        return nil
    end
    return battery:get_temperature(idx)
end

local function backends_ready()
    return battery:num_instances() >= required_instances()
end

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

local function get_product_model()
    local name = ""
    for i = 1, 7 do
        local p = param:get("SN_PROD" .. tostring(i))
        if not p or p == 0 then
            break
        end
        local p_int = math.floor(p)
        local b1 = (p_int >> 16) & 0xFF
        local b2 = (p_int >> 8) & 0xFF
        local b3 = p_int & 0xFF

        if b1 == 0 then break end
        name = name .. string.char(b1)
        if b2 == 0 then break end
        name = name .. string.char(b2)
        if b3 == 0 then break end
        name = name .. string.char(b3)
    end
    return name
end

local function is_x6100_model(model_str)
    if string.len(model_str) == 0 then
        return false
    end
    local substring = string.sub(model_str, 1, 8)
    return string.find(substring, "610") ~= nil
end

local function is_e616_model(model_str)
    if string.len(model_str) == 0 then
        return false
    end
    local substring = string.sub(model_str, 1, 8)
    return string.find(substring, "616") ~= nil
end

local function detect_mode()
    local model_str = get_product_model()
    if string.len(model_str) == 0 then
        return nil
    end

    if is_x6100_model(model_str) then
        is_two = false
        ADC_IDX = 2
        adc_mult_param = "BATT3_VOLT_MULT"
        adc_pin_param = "BATT3_VOLT_PIN"
        gcs:send_text(6, string.format("BATT: one X6100 (%s)", model_str))
        return true
    end

    if is_e616_model(model_str) then
        is_two = true
        ADC_IDX = 3
        adc_mult_param = "BATT4_VOLT_MULT"
        adc_pin_param = "BATT4_VOLT_PIN"
        gcs:send_text(6, string.format("BATT: two E616 (%s)", model_str))
        return true
    end

    return nil
end

local function apply_params_one(quiet)
    local capacity = param:get("BATT_CAPACITY") or 22000
    local changed = false

    if USE_UAVCAN_ONE then
        changed = set_param("CAN_P1_DRIVER", 1) or changed
        changed = set_param("CAN_D1_PROTOCOL", 1) or changed
    end

    changed = set_param("BATT_MONITOR", 29) or changed
    changed = set_param("BATT_CAPACITY", capacity) or changed
    changed = set_param("BATT_LOW_VOLT", 22.2) or changed
    changed = set_param("BATT_CRT_VOLT", 21.6) or changed
    changed = set_param("BATT_LOW_TIMER", 10) or changed
    changed = set_param("BATT_ARM_VOLT", 0) or changed

    if USE_UAVCAN_ONE then
        -- 已配置过脚本且 BATT2=0：说明无 CAN 电池，勿再打开 BATT2
        local batt_mon = param:get("BATT_MONITOR") or 0
        local batt2_mon = param:get("BATT2_MONITOR") or 0
        if math.abs(batt_mon - 29) < 1e-4 and batt2_mon == 0 then
            changed = set_param("BATT2_MONITOR", 0) or changed
        else
            changed = set_param("BATT2_MONITOR", 8) or changed
            changed = set_param("BATT2_SERIAL_NUM", 1) or changed
            changed = set_param("BATT2_CAPACITY", capacity) or changed
            changed = set_param("BATT2_LOW_VOLT", 0) or changed
            changed = set_param("BATT2_CRT_VOLT", 0) or changed
            changed = set_param("BATT2_OPTIONS", INTERNAL_ONLY) or changed
        end
    else
        changed = set_param("BATT2_MONITOR", 0) or changed
    end

    changed = set_param("BATT3_MONITOR", 3) or changed
    changed = set_param("BATT3_LOW_VOLT", 0) or changed
    changed = set_param("BATT3_CRT_VOLT", 0) or changed
    changed = set_param("BATT3_OPTIONS", INTERNAL_ONLY) or changed
    changed = set_param("BATT4_MONITOR", 0) or changed
    ensure_adc_scale()

    if not quiet then
        if changed then
            gcs:send_text(4, "BATT one: params saved, reboot if MONITOR changed")
        else
            gcs:send_text(6, "BATT one: params ok")
        end
    end
    return changed
end

local function apply_params_two(quiet)
    local capacity = param:get("BATT_CAPACITY") or 22000
    local changed = false

    changed = set_param("CAN_P1_DRIVER", 1) or changed
    changed = set_param("CAN_D1_PROTOCOL", 1) or changed

    changed = set_param("BATT_MONITOR", 29) or changed
    changed = set_param("BATT_CAPACITY", capacity) or changed
    changed = set_param("BATT_LOW_VOLT", 44.4) or changed
    changed = set_param("BATT_CRT_VOLT", 43.2) or changed
    changed = set_param("BATT_LOW_TIMER", 10) or changed
    changed = set_param("BATT_ARM_VOLT", 0) or changed

    changed = set_param("BATT2_MONITOR", 8) or changed
    -- -1 accepts any battery_id; DroneCAN binds this backend to the first free CAN node
    changed = set_param("BATT2_SERIAL_NUM", -1) or changed
    changed = set_param("BATT2_CAPACITY", capacity) or changed
    changed = set_param("BATT2_LOW_VOLT", 0) or changed
    changed = set_param("BATT2_CRT_VOLT", 0) or changed
    changed = set_param("BATT2_OPTIONS", INTERNAL_ONLY) or changed

    changed = set_param("BATT3_MONITOR", 8) or changed
    -- the next distinct CAN node is assigned to the second free backend
    changed = set_param("BATT3_SERIAL_NUM", -1) or changed
    changed = set_param("BATT3_CAPACITY", capacity) or changed
    changed = set_param("BATT3_LOW_VOLT", 0) or changed
    changed = set_param("BATT3_CRT_VOLT", 0) or changed
    changed = set_param("BATT3_OPTIONS", INTERNAL_ONLY) or changed

    changed = set_param("BATT4_MONITOR", 3) or changed
    changed = set_param("BATT4_LOW_VOLT", 0) or changed
    changed = set_param("BATT4_CRT_VOLT", 0) or changed
    changed = set_param("BATT4_OPTIONS", INTERNAL_ONLY) or changed
    ensure_adc_scale()

    if not quiet then
        if changed then
            gcs:send_text(4, "BATT two: params saved, reboot if MONITOR changed")
        else
            gcs:send_text(6, "BATT two: params ok")
        end
    end
    return changed
end

local function maybe_disable_uavcan_one()
    if not USE_UAVCAN_ONE then
        return
    end
    local batt2_mon = param:get("BATT2_MONITOR") or 0
    if batt2_mon == 0 then
        return
    end
    if battery_healthy(UAVCAN_A) then
        uavcan_missing_ms = nil
        return
    end
    if not battery_healthy(ADC_IDX) then
        return
    end
    local now = millis()
    if not uavcan_missing_ms then
        uavcan_missing_ms = now
        return
    end
    if now - uavcan_missing_ms < UAVCAN_OFF_DELAY_MS then
        return
    end
    if set_param("BATT2_MONITOR", 0) then
        if not uavcan_off_announced then
            gcs:send_text(4, "BATT: no UAVCAN, BATT2 off, reboot")
            uavcan_off_announced = true
        end
    end
end

local function update_one()
    if not script_enabled() then
        return idle, 5000
    end

    ensure_adc_scale()
    maybe_disable_uavcan_one()
    local state = BattMonitorScript_State()
    local can_ok = USE_UAVCAN_ONE and battery_healthy(UAVCAN_A)
    local adc_ok = battery_healthy(ADC_IDX)

    if can_ok then
        state:healthy(true)
        state:voltage(battery_voltage(UAVCAN_A) or 0)

        local i = battery_current_amps(UAVCAN_A)
        if i then
            state:current_amps(i)
        end
        local s = battery_capacity_remaining_pct(UAVCAN_A)
        if s then
            state:capacity_remaining_pct(s)
        end
        local t = battery_get_temperature(UAVCAN_A)
        if t then
            state:temperature(t)
        end

        if last_mode ~= MODE_UAVCAN then
            gcs:send_text(6, "BATT: UAVCAN")
            last_mode = MODE_UAVCAN
        end
    elseif adc_ok then
        state:healthy(true)
        state:voltage(battery_voltage(ADC_IDX) or 0)
        local i = battery_current_amps(ADC_IDX)
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
    if not script_enabled() then
        return idle, 5000
    end

    ensure_adc_scale()
    local state = BattMonitorScript_State()
    local a_ok = battery_healthy(UAVCAN_A)
    local b_ok = battery_healthy(UAVCAN_B)
    local adc_ok = battery_healthy(ADC_IDX)

    if a_ok and b_ok then
        local va = battery_voltage(UAVCAN_A) or 0
        local vb = battery_voltage(UAVCAN_B) or 0
        state:healthy(true)
        state:voltage(va + vb)

        local ia = battery_current_amps(UAVCAN_A)
        local ib = battery_current_amps(UAVCAN_B)
        if ia then
            state:current_amps(ia)
        elseif ib then
            state:current_amps(ib)
        end

        local sa = battery_capacity_remaining_pct(UAVCAN_A)
        local sb = battery_capacity_remaining_pct(UAVCAN_B)
        if sa and sb then
            state:capacity_remaining_pct(math.min(sa, sb))
        elseif sa then
            state:capacity_remaining_pct(sa)
        elseif sb then
            state:capacity_remaining_pct(sb)
        end

        local ta = battery_get_temperature(UAVCAN_A)
        local tb = battery_get_temperature(UAVCAN_B)
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
        state:healthy(true)
        state:voltage(battery_voltage(ADC_IDX) or 0)
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

local function boot()
    if not script_enabled() then
        return idle, 5000
    end

    if boot_state == "detect" then
        local ok = detect_mode()
        if ok == nil then
            return boot, 5000
        end
        boot_state = "apply"
    end

    if boot_state == "apply" then
        local changed
        if is_two then
            changed = apply_params_two(false)
        else
            changed = apply_params_one(false)
        end
        if changed then
            gcs:send_text(4, "BATT: reboot required for MONITOR setup")
            boot_state = "wait_backends"
            wait_reboot_announced = true
            return boot, 5000
        end
        boot_state = "wait_backends"
    end

    if boot_state == "wait_backends" then
        if not backends_ready() then
            if not wait_reboot_announced then
                gcs:send_text(4, string.format("BATT: waiting backends %d/%d (reboot if MONITOR changed)",
                    battery:num_instances(), required_instances()))
                wait_reboot_announced = true
            end
            return boot, 5000
        end
        boot_state = "run"
    end

    wait_reboot_announced = false
    if is_two then
        return update_two, 200
    end
    return update_one, 200
end

local function idle()
    if script_enabled() then
        wait_reboot_announced = false
        uavcan_off_announced = false
        uavcan_missing_ms = nil
        last_mode = MODE_NONE
        boot_state = "detect"
        return boot, 5000
    end
    boot_state = "detect"
    return idle, 5000
end

return idle, 5000
