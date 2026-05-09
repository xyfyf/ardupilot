--[[
  1crsf_radar_battwarn.lua — CRSF/ELRS：前向雷达距离 + 电池低电量告警
  前向雷达/测距 → FrSky S.Port Passthrough → CRSF（SERIAL_PROTOCOL=23）遥测链路
    
  RC_OPTIONS：建议勾选 Suppress CRSF mode/rate message for ELRS systems

  测距 DWORD（32 bit，按无符号整数解析；飞控打包为 uint32，与线序一致为小端 4 字节）
      bit [15:0]   distance_cm — 距离，单位 厘米；无效时为 0
      bit [19:16]  avoid_margin — AVOID_MARGIN 参数值 (0~15m)，即 Byte2 后四个 bit
      bit [23:20]  avoid_dist_max — AVOID_DIST_MAX 参数值 (0~15m)，即 Byte2 前四个 bit
      bit [31:28]  low_batt    — 1：飞控已判定低电量（Byte4 最高位）
--]]

local LOOP_MS = 15

local USE_PROXIMITY_FALLBACK = false
local PRX_SKIP_ANGLE_FILTER = true

local SPORT_DATA_FRAME = 0x10
local SENSOR_ID = 0x73
local APP_ID_FORWARD_RANGE = 0x5010

local USE_RANGEFINDER_BY_INSTANCE = true
local RANGEFINDER_INSTANCE = 0
local ORIENT_FORWARD = 0
local RFND_STATUS_GOOD = 4
local PRX_FORWARD_HALF_DEG = 60

local USE_NAMED_FLOAT_FALLBACK = true
local NAMED_FLOAT_INTERVAL_MS = 400
local NAMED_FLOAT_DIST_M = "FwdDst_m"
local NAMED_FLOAT_VALID = "FwdDstOk"
local NAMED_FLOAT_LOWBAT = "FwdLowBat"

local frsky_push_state = nil
local last_named_float_ms = 0

-- 部署校验标签已更新，方便地面站确认
local SCRIPT_DEPLOY_TAG = "v20260427-batt-bit24"

local FRSKY_PUSH_RETRY_MS = 500
local frsky_probe_last_ms = 0

local BATT_INSTANCE = 0

local function maybe_init_frsky_push_capability()
    if frsky_push_state == true then
        return
    end
    local now_ms = millis()
    if frsky_probe_last_ms ~= 0 and (now_ms - frsky_probe_last_ms < FRSKY_PUSH_RETRY_MS) then
        return
    end
    frsky_probe_last_ms = now_ms
    local ok, pushed = pcall(function()
        return frsky_sport:sport_telemetry_push(SENSOR_ID, SPORT_DATA_FRAME, APP_ID_FORWARD_RANGE, 0)
    end)
    if ok and pushed then
        frsky_push_state = true
        gcs:send_text(6, "CRSF battwarn push OK")
    end
end

local function batt_low_volt_param_name()
    if BATT_INSTANCE == 0 then
        return "BATT_LOW_VOLT"
    else
        return string.format("BATT%d_LOW_VOLT", BATT_INSTANCE + 1)
    end
end

local function batt_crt_volt_param_name()
    if BATT_INSTANCE == 0 then
        return "BATT_CRT_VOLT"
    else
        return string.format("BATT%d_CRT_VOLT", BATT_INSTANCE + 1)
    end
end

local function is_battery_low_for_telem()
    if battery == nil then
        return false
    end
    if battery:has_failsafed() then
        return true
    end
    local v = battery:voltage(BATT_INSTANCE)
    if v == nil or v <= 0.5 then
        return false
    end
    local thr = param:get(batt_low_volt_param_name())
    if thr ~= nil and thr > 0 and v < thr then
        return true
    end
    local crt = param:get(batt_crt_volt_param_name())
    if crt ~= nil and crt > 0 and v < crt then
        return true
    end
    return false
end

local p_avoid_dist_max = Parameter("AVOID_DIST_MAX")
local p_avoid_margin = Parameter("AVOID_MARGIN")

local function pack_forward_telem(valid, source_prx, distance_m, angle_deg, low_batt)
    local dword = 0
    if valid and distance_m ~= nil and distance_m >= 0 then
        local cm = math.floor(distance_m * 100.0 + 0.5)
        if cm > 0xFFFF then
            cm = 0xFFFF
        end
        dword = dword | (cm & 0xFFFF)
    end
    
    -- 获取避障参数并限制在 0~15 范围内（4 bit）
    local dist_max = p_avoid_dist_max:get() or 0
    local margin = p_avoid_margin:get() or 0
    dist_max = math.floor(math.max(0, math.min(15, dist_max)))
    margin = math.floor(math.max(0, math.min(15, margin)))
    
    -- Byte 2 (bit 16~23): 前四个bit (bit 20~23) 放 AVOID_DIST_MAX，后四个bit (bit 16~19) 放 AVOID_MARGIN
    dword = dword | ((margin & 0xF) << 16)
    dword = dword | ((dist_max & 0xF) << 20)
    
    -- Byte 3 (bit 24~31): low_batt
    if low_batt then
        dword = dword | (1 << 28)
    end
    
    return dword & 0xFFFFFFFF
end

local function read_rangefinder_by_instance()
    local n = rangefinder:num_sensors()
    if RANGEFINDER_INSTANCE < 0 or RANGEFINDER_INSTANCE >= n then
        return false, nil
    end
    local backend = rangefinder:get_backend(RANGEFINDER_INSTANCE)
    if backend == nil then
        return false, nil
    end
    if backend:status() ~= RFND_STATUS_GOOD then
        return false, nil
    end
    local d = backend:distance()
    if d == nil then
        return false, nil
    end
    return true, d
end

local function read_rangefinder_by_orientation()
    if not rangefinder:has_orientation(ORIENT_FORWARD) then
        return false, nil
    end
    if rangefinder:status_orient(ORIENT_FORWARD) ~= RFND_STATUS_GOOD then
        return false, nil
    end
    if not rangefinder:has_data_orient(ORIENT_FORWARD) then
        return false, nil
    end
    local d = rangefinder:distance_orient(ORIENT_FORWARD)
    if d == nil then
        return false, nil
    end
    return true, d
end

local function read_forward_rangefinder()
    if USE_RANGEFINDER_BY_INSTANCE then
        return read_rangefinder_by_instance()
    end
    return read_rangefinder_by_orientation()
end

local function read_forward_proximity_fallback()
    if proximity == nil or proximity:num_sensors() == 0 then
        return false, nil, nil, false
    end
    local ang, dist = proximity:get_closest_object()
    if ang == nil or dist == nil then
        return false, nil, nil, false
    end
    if not PRX_SKIP_ANGLE_FILTER then
        local a = ang
        if a > 180.0 then
            a = a - 360.0
        end
        if math.abs(a) > PRX_FORWARD_HALF_DEG then
            return false, nil, ang, false
        end
    end
    return true, dist, ang, true
end

local function push_telemetry(dword, valid, dist_m, low_batt)
    if frsky_push_state == true then
        pcall(function()
            frsky_sport:sport_telemetry_push(SENSOR_ID, SPORT_DATA_FRAME, APP_ID_FORWARD_RANGE, dword)
        end)
        return
    end
    if not USE_NAMED_FLOAT_FALLBACK then
        return
    end
    local now_ms = millis()
    if now_ms - last_named_float_ms < NAMED_FLOAT_INTERVAL_MS then
        return
    end
    last_named_float_ms = now_ms
    pcall(function()
        gcs:send_named_float(NAMED_FLOAT_VALID, (valid and dist_m ~= nil) and 1.0 or 0.0)
        gcs:send_named_float(NAMED_FLOAT_DIST_M, (valid and dist_m ~= nil) and dist_m or 0.0)
        gcs:send_named_float(NAMED_FLOAT_LOWBAT, low_batt and 1.0 or 0.0)
    end)
end

function update()
    maybe_init_frsky_push_capability()

    local valid, dist_m = read_forward_rangefinder()
    local used_prx = false
    local angle_deg = 0.0

    if not valid and USE_PROXIMITY_FALLBACK then
        local pv, pd, pang, prx = read_forward_proximity_fallback()
        valid = pv
        dist_m = pd
        used_prx = prx
        if pang ~= nil then
            angle_deg = pang
        end
    end

    local low_batt = is_battery_low_for_telem()
    local dword = pack_forward_telem(valid, used_prx, dist_m, angle_deg, low_batt)
    push_telemetry(dword, valid, dist_m, low_batt)

    return update, LOOP_MS
end

return update()