--[[
  脚本名称: gps1_gps2_yaw_primary_switch.lua  v6.15
  适用场景: EFT_CAAC 机控; 仅 SN_PROD 含 616/610 (E616/X6100) 时启用, 否则直接退出
              GPS1 = ublox GPS                       (instance 0)
              GPS2 = UM982 双天线 RTK on SERIAL7     (instance 1)

  功能 (4 种工作状态):
    1. GPS1 (ublox) + GPS2 (RTK) 双在线, GPS2 状态 >= 3D Fix 且双天线航向有效:
         GPS1_TYPE      = 1   (Auto, 识别 ublox)
         GPS2_TYPE      = 25  (UM982 / Unicore moving baseline, 启用 RTK)
         EK3_SRC1_YAW   = 2   (使用 GPS 双天线 yaw)
         GPS_PRIMARY    = 1   (优先 GPS2)
         GPS_AUTO_SWITCH= 0   (主 GPS 固定听 GPS_PRIMARY)
         COMPASS_USE2   = 1   (延迟 GPSYS_COMP_DLY 秒后启用; 注册顺序反, 外置在 USE2)
    2. 冷启动等 RTK 航向 (未解锁, UM982 在线但双天线航向尚未解算, 宽限 GPSYS_RTK_WAIT 秒):
         GPS1_TYPE      = 1
         GPS2_TYPE      = 25  (保持 UM982 驱动不断电, 让其慢慢收敛)
         EK3_SRC1_YAW   = 2   (坚持等 GPS 双天线 yaw, 不切磁罗盘)
         GPS_PRIMARY    = 1   (主 GPS 仍指 GPS2)
         GPS_AUTO_SWITCH= 0
         COMPASS_USE2   = 0   (禁用罗盘, 避免 EKF 提前用磁罗盘初始化航向)
    3. RTK 宽限超时 / 天线丢失 / 模块离线 / 飞行中 RTK 故障:
         GPS1_TYPE      = 1
         GPS2_TYPE      = 25  (始终保持 UM982 驱动, 不写 0)
         EK3_SRC1_YAW   = 1   (使用外置磁罗盘 yaw)
         GPS_PRIMARY    = 0   (只用 GPS1 定位)
         GPS_AUTO_SWITCH= 0
         COMPASS_USE2   = 1   (立即启用外置罗盘)
    4. 仅识别 GPS2 RTK (ublox GPS1 掉线, gps:status(0)==0), RTK 完好:
         GPS1_TYPE      = 0   (关闭 GPS1, 并周期性 reprobe 等待热插回)
         GPS2_TYPE      = 25  (UM982 保持在线)
         EK3_SRC1_YAW   = 2   (使用 GPS 双天线 yaw)
         GPS_PRIMARY    = 1   (使用 GPS2)
         GPS_AUTO_SWITCH= 0
         COMPASS_USE2   = 1
         解锁拦截: GPS1 故障但 RTK 正常时禁止解锁, 须恢复 GPS1 后才能飞

  v6.15 变更 (罗盘注册顺序反了):
  - 外置罗盘开关改管 COMPASS_USE2; COMPASS_USE / USE3 保持关闭.

  v6.14 变更 (修复断电插回 RTK 首次上电无法解锁):
  - boot_rtk_guard: 脚本加载瞬间若 GPS2_TYPE=25, 立即写 EK3_SRC1_YAW=2 / GPS_PRIMARY=1 /
    COMPASS_USE2=0, 抢在 EKF 磁罗盘对准前生效, 不等待 5s 启动延迟.
  - RTK_WARMUP 宽限改为看 GPS2_TYPE=25 配置, 不再要求 rtk_module_online(), 避免 UM982
    探测期被误判为 GPS1_ONLY.
  - GPS1_ONLY 降级参数改用 param:set 运行期写入, 不写 flash, 防止热拔 RTK 污染下次冷启动.

  v6.13 变更 (基于 v6.3 + UM982 冷启动优化):
  - 新增 RTK_WARMUP: 冷启动 UM982 在线但航向未就绪时, 保持 GPS2_TYPE=25 不断电,
    坚持 EK3_SRC1_YAW=2 + COMPASS_USE2=0, 宽限 GPSYS_RTK_WAIT 秒等收敛.
  - GPS1_ONLY 时 GPS2_TYPE 始终写 25, 不因模块离线/reprobe 写 0.
  - 曾出过 RTK 航向后丢失 (主/副天线掉): 立即切 GPS1+磁罗盘, 不等 WARMUP/防抖.
  - GPS_AUTO_SWITCH=0, 防止 ublox DGPS 抢走主 GPS 导致 EKF 航向漂移.
  - 罗盘由 manage_compass() 统一管理 COMPASS_USE2: WARMUP 关, BOTH_OK 延迟开, 故障立即开.

  安全约束:
  - 未解锁状态才修改 EK3_SRC1_YAW / GPS_PRIMARY (解锁后 RTK 故障例外)
  - 已解锁但 RTK 恢复: 保持 GPS1_ONLY 模式, 落地后重新确认
  - 防抖: 未解锁 5 次 (5 秒) 确认后才切换; 已解锁 3 次确认后切换
  - 状态不变时不写 flash, 不反复磨损
--]]

---@diagnostic disable: need-check-nil, cast-local-type, assign-type-mismatch, param-type-mismatch

-- 仅 E616 / X6100（SN_PROD 含 616 或 610）启用; SN_PROD 为空则直接退出
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

local function is_target_model(model_str)
    if string.len(model_str) == 0 then
        return false
    end
    local substring = string.sub(model_str, 1, 8)
    return string.find(substring, "616") ~= nil
        or string.find(substring, "610") ~= nil
end

if not is_target_model(get_product_model()) then
    return
end

-- 脚本参数 (地面站 Full Parameter List 搜索 GPSYS_)
--   GPSYS_ENABLE   : 0=禁用脚本, 1=启用(默认)
--   GPSYS_RTK_WAIT : 冷启动等 RTK 双天线航向宽限秒数 (默认 30, UM982 冷启动约需 30~90s)
--   GPSYS_COMP_DLY : RTK 航向就绪后延迟启用罗盘秒数 (默认 30)
local PARAM_TABLE_KEY    = 200
local PARAM_TABLE_PREFIX = "GPSYS_"
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 3), "GPSYS: add_table fail")
assert(param:add_param(PARAM_TABLE_KEY, 1, "ENABLE",   1),  "GPSYS: add ENABLE fail")
assert(param:add_param(PARAM_TABLE_KEY, 2, "RTK_WAIT", 30), "GPSYS: add RTK_WAIT fail")
assert(param:add_param(PARAM_TABLE_KEY, 3, "COMP_DLY", 30), "GPSYS: add COMP_DLY fail")
local gpsys_enable    = Parameter("GPSYS_ENABLE")
local gpsys_rtk_wait  = Parameter("GPSYS_RTK_WAIT")
local gpsys_comp_dly  = Parameter("GPSYS_COMP_DLY")

local function script_enabled()
    return gpsys_enable:get() >= 1
end

local function rtk_wait_ms()
    local s = gpsys_rtk_wait:get() or 30
    if s < 0 then s = 0 end
    return s * 1000
end

local RUN_INTERVAL_MS  = 1000
local STARTUP_DELAY_MS = 1000   -- boot_rtk_guard 已抢占航向源; 此处仅等 GPS 驱动首轮探测
local CONFIRM_COUNT    = 5

local GPS1_INSTANCE    = 0
local GPS2_INSTANCE    = 1
local MIN_GPS_STATUS   = 3

local YAW_COMPASS = 1
local YAW_GPS     = 2

local PRIMARY_GPS1 = 0
local PRIMARY_GPS2 = 1

local AUTOSW_USE_PRIMARY = 0

local TYPE_NONE  = 0
local TYPE_AUTO  = 1
local TYPE_UM982 = 25

local COMPASS_OFF = 0
local COMPASS_ON  = 1

local STATE_UNKNOWN    = 0
local STATE_BOTH_OK    = 1
local STATE_GPS1_ONLY  = 2
local STATE_GPS2_ONLY  = 3
local STATE_RTK_WARMUP = 4

local rtk_warmup_start_ms = 0
local ever_had_rtk_yaw    = false

local SEV_INFO = 6
local SEV_WARN = 4

local current_state   = STATE_UNKNOWN
local pending_state   = STATE_UNKNOWN
local pending_count   = 0
local last_warn_ms    = 0
local WARN_REPEAT_MS  = 10000

local arm_auth_id        = nil
local last_auth_warn_ms  = 0

local armed_rtk_fail_count     = 0
local ARMED_RTK_FAIL_THRESHOLD = 3

local REPROBE_INTERVAL_MS = 15000
local REPROBE1_WINDOW_MS  = 5000

local reprobe1_active    = false
local reprobe1_start_ms  = 0
local last_reprobe1_ms   = 0

local compass_open_at_ms = 0

local MODULE_TIMEOUT_MS = 3000

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

-- 运行期写入, 不落盘 (GPS1_ONLY 降级用, 避免热拔 RTK 把磁罗盘航向写进 flash)
local function set_param_runtime(name, value)
    local cur = param:get(name)
    if cur == nil or cur ~= value then
        if param:set(name, value) then
            return true
        else
            gcs:send_text(SEV_WARN, string.format("%s set fail", name))
        end
    end
    return false
end

local function rtk_configured()
    local g2_type = param:get("GPS2_TYPE")
    return g2_type ~= nil and g2_type == TYPE_UM982
end

-- 上电第一时间抢占航向源: 闪存里可能残留上次热拔 RTK 时的 GPS1_ONLY 参数
local function boot_rtk_guard()
    if not rtk_configured() then
        return
    end
    set_param_runtime("GPS1_TYPE",       TYPE_AUTO)
    set_param_runtime("GPS2_TYPE",       TYPE_UM982)
    set_param_runtime("EK3_SRC1_YAW",    YAW_GPS)
    set_param_runtime("GPS_PRIMARY",     PRIMARY_GPS2)
    set_param_runtime("GPS_AUTO_SWITCH", AUTOSW_USE_PRIMARY)
    set_param_runtime("COMPASS_USE",     COMPASS_OFF)
    set_param_runtime("COMPASS_USE2",    COMPASS_OFF)
    set_param_runtime("COMPASS_USE3",    COMPASS_OFF)
    current_state = STATE_RTK_WARMUP
    if rtk_warmup_start_ms == 0 then
        rtk_warmup_start_ms = millis():tofloat()
    end
end

local function probe_gps1_on_startup()
    local cur = param:get("GPS1_TYPE")
    if cur ~= nil and cur == TYPE_NONE then
        param:set_and_save("GPS1_TYPE", TYPE_AUTO)
    end
end

local function probe_gps2_on_startup()
    local cur = param:get("GPS2_TYPE")
    if cur ~= nil and cur == TYPE_NONE then
        param:set_and_save("GPS2_TYPE", TYPE_UM982)
    end
end

local function safe_gps_status(instance)
    local n = gps:num_sensors() or 0
    if instance >= n then
        return 0
    end
    return gps:status(instance) or 0
end

local function safe_gps_yaw_deg(instance)
    local n = gps:num_sensors() or 0
    if instance >= n then
        return nil, nil, nil
    end
    return gps:gps_yaw_deg(instance)
end

-- UM982 模块是否还在串口通信 (区分 "在线收敛中" vs "真的拔掉")
local function rtk_module_online()
    if (gps:num_sensors() or 0) <= GPS2_INSTANCE then
        return false
    end
    local last = gps:last_message_time_ms(GPS2_INSTANCE)
    if last == nil then
        return false
    end
    return (millis() - last) < MODULE_TIMEOUT_MS
end

local function read_state()
    local g1_status  = safe_gps_status(GPS1_INSTANCE)
    local g2_status  = safe_gps_status(GPS2_INSTANCE)
    local g1_present = (g1_status >= 1)
    local g2_ok      = (g2_status >= MIN_GPS_STATUS)

    local rtk_yaw, _, _ = safe_gps_yaw_deg(GPS2_INSTANCE)
    local g2_yaw_ok = (rtk_yaw ~= nil)

    if g2_ok and g2_yaw_ok then
        ever_had_rtk_yaw = true
        rtk_warmup_start_ms = 0
        if g1_present then
            return STATE_BOTH_OK
        end
        return STATE_GPS2_ONLY
    end

    -- 冷启动: 本会话从未出过 RTK 航向, UM982 已配置或已通信 → 宽限内等收敛
    -- 不能只看 rtk_module_online: 探测期尚无串口数据, 否则会误判 GPS1_ONLY
    if (not arming:is_armed()) and (not ever_had_rtk_yaw) and (rtk_configured() or rtk_module_online()) then
        if rtk_wait_ms() > 0 then
            if rtk_warmup_start_ms == 0 then
                rtk_warmup_start_ms = millis():tofloat()
            end
            if (millis():tofloat() - rtk_warmup_start_ms) < rtk_wait_ms() then
                return STATE_RTK_WARMUP
            end
        end
    elseif ever_had_rtk_yaw or not rtk_configured() then
        rtk_warmup_start_ms = 0
    end

    if g1_present then
        return STATE_GPS1_ONLY
    end
    return STATE_UNKNOWN
end

local function apply_state(state)
    if state == STATE_RTK_WARMUP then
        set_param_if_diff("GPS1_TYPE",       TYPE_AUTO)
        set_param_if_diff("GPS2_TYPE",       TYPE_UM982)
        set_param_if_diff("EK3_SRC1_YAW",    YAW_GPS)
        set_param_if_diff("GPS_PRIMARY",     PRIMARY_GPS2)
        set_param_if_diff("GPS_AUTO_SWITCH", AUTOSW_USE_PRIMARY)
    elseif state == STATE_BOTH_OK then
        local a = set_param_if_diff("GPS1_TYPE",       TYPE_AUTO)
        local e = set_param_if_diff("GPS2_TYPE",       TYPE_UM982)
        local b = set_param_if_diff("EK3_SRC1_YAW",    YAW_GPS)
        local c = set_param_if_diff("GPS_PRIMARY",     PRIMARY_GPS2)
        local f = set_param_if_diff("GPS_AUTO_SWITCH", AUTOSW_USE_PRIMARY)
        if a or b or c or e or f then
            gcs:send_text(SEV_INFO, "GPS: dual RTK")
        end
    elseif state == STATE_GPS1_ONLY then
        -- 降级态只改运行期参数, 不写 flash, 防止下次插回 RTK 冷启动仍读到 YAW=1
        set_param_runtime("GPS1_TYPE",       TYPE_AUTO)
        set_param_runtime("GPS2_TYPE",       TYPE_UM982)
        set_param_runtime("EK3_SRC1_YAW",    YAW_COMPASS)
        set_param_runtime("GPS_PRIMARY",     PRIMARY_GPS1)
        set_param_runtime("GPS_AUTO_SWITCH", AUTOSW_USE_PRIMARY)
    elseif state == STATE_GPS2_ONLY then
        local e = set_param_if_diff("GPS2_TYPE",       TYPE_UM982)
        local a = set_param_if_diff("EK3_SRC1_YAW",    YAW_GPS)
        local b = set_param_if_diff("GPS_PRIMARY",     PRIMARY_GPS2)
        local g = set_param_if_diff("GPS_AUTO_SWITCH", AUTOSW_USE_PRIMARY)
        local c = set_param_if_diff("GPS1_TYPE",       TYPE_NONE)
        if a or b or c or e or g then
            gcs:send_text(SEV_WARN, "GPS: RTK only, arm blocked")
        end
    end
end

local function manage_compass()
    -- 注册顺序反了: 外置罗盘在 COMPASS_USE2, 内置在 COMPASS_USE
    set_param_if_diff("COMPASS_USE", COMPASS_OFF)
    set_param_if_diff("COMPASS_USE3", COMPASS_OFF)

    local rtk_ok = (current_state == STATE_BOTH_OK or current_state == STATE_GPS2_ONLY)

    if current_state == STATE_RTK_WARMUP then
        set_param_if_diff("COMPASS_USE2", COMPASS_OFF)
        compass_open_at_ms = 0
    elseif rtk_ok then
        if compass_open_at_ms == 0 then
            local dly = (gpsys_comp_dly:get() or 30)
            if dly < 0 then dly = 0 end
            compass_open_at_ms = millis():tofloat() + dly * 1000
        end
        if millis():tofloat() >= compass_open_at_ms then
            set_param_if_diff("COMPASS_USE2", COMPASS_ON)
        else
            set_param_if_diff("COMPASS_USE2", COMPASS_OFF)
        end
    else
        compass_open_at_ms = 0
        set_param_if_diff("COMPASS_USE2", COMPASS_ON)
    end
end

local function gps1_fault_rtk_ok()
    local g1_status = safe_gps_status(GPS1_INSTANCE)
    local g2_status = safe_gps_status(GPS2_INSTANCE)
    local rtk_yaw, _, _ = safe_gps_yaw_deg(GPS2_INSTANCE)
    return (g1_status < 1)
        and (g2_status >= MIN_GPS_STATUS)
        and (rtk_yaw ~= nil)
end

local function update_arming_auth()
    if arm_auth_id == nil then
        arm_auth_id = arming:get_aux_auth_id()
        if arm_auth_id == nil then
            local now = millis()
            if (now - last_auth_warn_ms) >= 10000 then
                gcs:send_text(SEV_WARN, "GPSYS: no arm auth slot")
                last_auth_warn_ms = now
            end
            return
        end
    end

    if arming:is_armed() then
        arming:set_aux_auth_passed(arm_auth_id)
        return
    end

    if gps1_fault_rtk_ok() then
        arming:set_aux_auth_failed(arm_auth_id, "GPS1 fault, RTK OK, fix GPS1")
        return
    end

    arming:set_aux_auth_passed(arm_auth_id)
end

local function read_state_name(s)
    if s == STATE_BOTH_OK     then return "BOTH_OK"        end
    if s == STATE_GPS1_ONLY   then return "GPS1_ONLY"      end
    if s == STATE_GPS2_ONLY   then return "GPS2_ONLY(RTK)" end
    if s == STATE_RTK_WARMUP  then return "RTK_WARMUP"     end
    return "UNKNOWN"
end

function update()
    if not script_enabled() then
        return update, RUN_INTERVAL_MS
    end

    update_arming_auth()

    local s = read_state()

    if arming:is_armed() then
        if current_state == STATE_BOTH_OK then
            if s == STATE_GPS1_ONLY or s == STATE_UNKNOWN then
                armed_rtk_fail_count = armed_rtk_fail_count + 1
                if armed_rtk_fail_count >= ARMED_RTK_FAIL_THRESHOLD then
                    apply_state(STATE_GPS1_ONLY)
                    current_state = STATE_GPS1_ONLY
                    armed_rtk_fail_count = 0
                    gcs:send_text(SEV_WARN, "RTK lost: GPS1+compass")
                end
            else
                armed_rtk_fail_count = 0
            end
        end

        if current_state ~= STATE_UNKNOWN and s ~= STATE_UNKNOWN and s ~= current_state then
            local now = millis()
            if (now - last_warn_ms) > WARN_REPEAT_MS then
                gcs:send_text(SEV_WARN, string.format(
                    "GPS %s->%s (armed)",
                    read_state_name(current_state), read_state_name(s)))
                last_warn_ms = now
            end
        end

        pending_state = STATE_UNKNOWN
        pending_count = 0
        manage_compass()
        return update, RUN_INTERVAL_MS
    end

    armed_rtk_fail_count = 0

    -- GPS1 (ublox) 热插再探测: 仅在 GPS2_ONLY 时进行
    if current_state == STATE_GPS2_ONLY then
        local g1_type = param:get("GPS1_TYPE")
        local now = millis()
        if not reprobe1_active then
            if g1_type ~= nil and g1_type == TYPE_NONE
               and (now - last_reprobe1_ms) >= REPROBE_INTERVAL_MS then
                param:set_and_save("GPS1_TYPE", TYPE_AUTO)
                reprobe1_active   = true
                reprobe1_start_ms = now
                gcs:send_text(SEV_INFO, "GPS1 reprobe...")
            end
        else
            if safe_gps_status(GPS1_INSTANCE) >= 1 then
                reprobe1_active  = false
                last_reprobe1_ms = now
            elseif (now - reprobe1_start_ms) >= REPROBE1_WINDOW_MS then
                param:set_and_save("GPS1_TYPE", TYPE_NONE)
                reprobe1_active  = false
                last_reprobe1_ms = now
            end
        end
    else
        reprobe1_active = false
    end

    if s == STATE_UNKNOWN then
        pending_state = STATE_UNKNOWN
        pending_count = 0
        manage_compass()
        return update, RUN_INTERVAL_MS
    end

    -- 进入 RTK_WARMUP 立即生效, 尽早禁磁罗盘等 UM982 冷启动收敛
    if s == STATE_RTK_WARMUP and current_state ~= STATE_RTK_WARMUP then
        apply_state(STATE_RTK_WARMUP)
        current_state = STATE_RTK_WARMUP
        pending_state = STATE_UNKNOWN
        pending_count = 0
        manage_compass()
        return update, RUN_INTERVAL_MS
    end

    -- 宽限超时: 立即切 GPS1+磁罗盘
    if s == STATE_GPS1_ONLY and current_state == STATE_RTK_WARMUP then
        apply_state(STATE_GPS1_ONLY)
        current_state = STATE_GPS1_ONLY
        pending_state = STATE_UNKNOWN
        pending_count = 0
        manage_compass()
        return update, RUN_INTERVAL_MS
    end

    -- 曾出过 RTK 航向后丢失: 立即切 GPS1+磁罗盘
    if s == STATE_GPS1_ONLY and ever_had_rtk_yaw and current_state ~= STATE_GPS1_ONLY then
        apply_state(STATE_GPS1_ONLY)
        current_state = STATE_GPS1_ONLY
        pending_state = STATE_UNKNOWN
        pending_count = 0
        manage_compass()
        return update, RUN_INTERVAL_MS
    end

    if s == current_state then
        pending_state = STATE_UNKNOWN
        pending_count = 0
        manage_compass()
        return update, RUN_INTERVAL_MS
    end

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

    manage_compass()
    return update, RUN_INTERVAL_MS
end

if script_enabled() then
    probe_gps1_on_startup()
    probe_gps2_on_startup()
    boot_rtk_guard()
end
return update, STARTUP_DELAY_MS
