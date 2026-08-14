--[[
  CAN 雷达自动检测 + Scripting1 飞行中避障切换 一体脚本 v6.5
  =====================================================================
  合并自：
    - 1auto_detect_radar5.1.lua  （DroneCAN 雷达识别 + 参数自适应 + ARMING_CHECK 管理）
    - 0radar_avoid_rc7_toggle3.0.lua（RC 飞行中切换 AVOID_ENABLE）

  v6.5 仅 Loiter 模式下避障
  -------------------------
    扫描 RC1~RC16，任一 RCx_OPTION=300 且 PWM>1500，且当前为 Loiter 模式
    → AVOID_ENABLE=7；否则 AVOID_ENABLE=1。非 Loiter 时开关打开也不避障。
  -------------------------
  v6.4 任意 Scripting1 通道 PWM>1500 即开避障（已废弃）
  --------------------------------------------
    扫描 RC1~RC16，任一 RCx_OPTION=300 且 PWM>1500 → AVOID_ENABLE=7；
    否则 AVOID_ENABLE=1。不限飞行模式，不限具体通道号（RC7/RC8 等均可）。
  -----------------------------------------
  根因（v6.0）：
    判定 radar_ok 必须同时满足 NodeStatus + RNGFND Good。
    实测总线 ~7s 即有 Measurement，但 Lua 订阅 NodeStatus 可能漏收。
    超时走 apply_radar_not_ok() 把 RNGFND1_TYPE/PRX1_TYPE 写成 0 并持久化；
    下次上电脚本再把 TYPE 改回 24，驱动才真正就绪 → 表现为第二次上电才正常。

  v6.2 策略（修复拔掉雷达后一直报 PreArm: PRX1: No Data 的问题）：
    1. 以 rangefinder Good 为主判定（与 CAN 分析仪看到的 Measurement 一致）
    2. NodeStatus 仅作辅助，用于尽早 ensure TYPE=24
    3. 拔掉雷达时将 PRX1_TYPE 写成 0 以屏蔽一直报错（RNGFND1_TYPE 保持 24，避免下次必须二次上电）
    4. BOOT_DELAY 从「检测开始时刻」计时，不再用飞控上电绝对时间

  整体逻辑
  ========
  Phase 1 - 开机检测（解锁前，最长 BOOT_DELAY_MS）
    轮询 rangefinder:status_orient == Good，持续 CONFIRM_DELAY_MS → radar_ok
    辅助监听 NodeStatus（CAN1+CAN2），见到节点则尽早写入 TYPE=24

    radar_ok:
      RNGFND1_TYPE=24, PRX1_TYPE=4, AVOID_ENABLE=1, ARMING_CHECK bit15=0

    超时仍未 Good，但曾见节点或曾 Good:
      保持 TYPE=24/PRX=4, AVOID_ENABLE=1

    全程无任何雷达迹象（无节点、无 Good）:
      保持 RNGFND1_TYPE=24, 修改 PRX1_TYPE=0, AVOID_ENABLE=1（屏蔽报错）

  Phase 2 - 飞行中 Scripting1 切换（radar_ok 后启用，仅 Loiter）
    Loiter 且任一 RCx_OPTION=300 通道 PWM > 1500 → AVOID_ENABLE=7
    非 Loiter，或全部 Scripting1 通道 PWM ≤ 1500 → AVOID_ENABLE=1

  依赖
  ----
  CAN_Px_DRIVER=1, CAN_Dx_PROTOCOL=1, SCR_ENABLE=1
  RNGFND1_ADDR 必须与雷达 sensor_id 匹配（默认 0）
--]]

------------------------------------------------------------------
-- 可调参数
------------------------------------------------------------------
local RADAR_NODE_ID      = 120     -- 【必改】雷达的 DroneCAN Node ID
local RANGEFINDER_ORIENT = 0       -- 雷达朝向：0=前, 25=下, 4=后, 2=右, 6=左
local BOOT_DELAY_MS      = 15000   -- 自检测开始起最长等待（7s 有数据 + 3s 确认足够）
local CONFIRM_DELAY_MS   = 2000    -- Good 持续多久才认定稳定
local INIT_DELAY_MS      = 1000    -- 脚本启动后首次检测延迟

local SCRIPTING1_FN      = 300     -- RC 切换功能：SCRIPTING_1（RCx_OPTION=300）
local AVOID_PWM_ON       = 1500    -- 任一 Scripting1 通道 PWM 超过此值即开避障
local MODE_LOITER        = 5       -- ArduCopter Loiter 模式号

------------------------------------------------------------------
-- 常量
------------------------------------------------------------------
local TARGET_RNGFND_TYPE = 24      -- DroneCAN rangefinder
local TARGET_PRX_TYPE    = 4       -- RangeFinder bridge
local AVOID_ON           = 7       -- Fence + Proximity + Beacon
local AVOID_OFF          = 1       -- 仅 Fence

local ARMING_CHECK_ALL_BIT  = 1 << 0
local ARMING_CHECK_RNGFND   = 1 << 15
local ARMING_CHECK_EXPANDED = 0xFFFFE  -- bits 1..19，不含 bit0

local RNGFND_STATUS_GOOD = 4       -- enum RangeFinder::Status::Good

-- DroneCAN NodeStatus（辅助，非主判定）
local NODESTATUS_ID        = 341
local NODESTATUS_SIGNATURE = uint64_t(0x0F0868D0, 0xC1A7C6F1)

------------------------------------------------------------------
-- 参数对象
------------------------------------------------------------------
local rngfnd_param       = Parameter("RNGFND1_TYPE")
local prx_param          = Parameter("PRX1_TYPE")
local avoid_param        = Parameter("AVOID_ENABLE")
local arming_check_param = Parameter("ARMING_CHECK")

local rc_option_params = {}
for ch = 1, 16 do
    rc_option_params[ch] = Parameter(string.format("RC%d_OPTION", ch))
end

------------------------------------------------------------------
-- DroneCAN NodeStatus 订阅（CAN1=driver0, CAN2=driver1）
------------------------------------------------------------------
local nodestatus_handles = {}
for driver_idx = 0, 1 do
    local h = DroneCAN_Handle(driver_idx, NODESTATUS_SIGNATURE, NODESTATUS_ID)
    if h then
        h:subscribe()
        nodestatus_handles[#nodestatus_handles + 1] = h
    end
end

if #nodestatus_handles == 0 then
    gcs:send_text(3, "Radar: no CAN, abort")
    return
end

------------------------------------------------------------------
-- 状态
------------------------------------------------------------------
local phase             = 1          -- 1=detect, 2=rc7 toggle, 0=disabled
local radar_seen        = false      -- 收到了 RADAR_NODE_ID 的 NodeStatus
local had_rngfnd_good   = false      -- 检测窗口内曾出现 Good
local first_good_ms     = nil        -- 当前连续 Good 段起始时间
local detect_start_ms   = nil        -- Phase 1 起始时刻
local last_avoid_on     = nil        -- Phase 2 上次 AVOID 状态
local rngfnd_enabled    = false      -- 已确认/写入 TYPE=24

------------------------------------------------------------------
-- 工具：写参数（不变就跳过），返回 ok, changed
------------------------------------------------------------------
local function set_param_persist(param, name, target)
    local cur = param:get()
    if cur == nil then
        gcs:send_text(4, "Radar: get param err")
        return false, false
    end
    if math.floor(cur) == target then
        return true, false
    end
    if not param:set_and_save(target) then
        gcs:send_text(4, "Radar: set param err")
        return false, false
    end
    return true, true
end

local function ensure_rngfnd_driver_enabled()
    if rngfnd_enabled then
        return
    end
    local cur = rngfnd_param:get()
    if cur ~= nil and math.floor(cur) == TARGET_RNGFND_TYPE then
        rngfnd_enabled = true
        return
    end
    local ok, changed = set_param_persist(rngfnd_param, "RNGFND1_TYPE", TARGET_RNGFND_TYPE)
    if ok then
        rngfnd_enabled = true
        if changed then
            gcs:send_text(6, "Radar: TYPE restored")
        end
    end
end

-- 修复 v6.0 误写的 TYPE=0，避免必须二次上电
do
    local cur = rngfnd_param:get()
    if cur ~= nil and math.floor(cur) == TARGET_RNGFND_TYPE then
        rngfnd_enabled = true
    elseif cur ~= nil and math.floor(cur) == 0 then
        if set_param_persist(rngfnd_param, "RNGFND1_TYPE", TARGET_RNGFND_TYPE) then
            set_param_persist(prx_param, "PRX1_TYPE", TARGET_PRX_TYPE)
            rngfnd_enabled = true
            gcs:send_text(6, "Radar: fixed TYPE=0")
        end
    end
end

-- ARMING_CHECK bit15 管理：enable=true 设 1，false 清 0
local function set_arming_rngfnd_check(enable)
    local cur = arming_check_param:get()
    if cur == nil then
        gcs:send_text(4, "Radar: get ARM err")
        return false
    end
    cur = math.floor(cur)
    local original = cur

    local target
    if enable then
        target = cur | ARMING_CHECK_RNGFND
    else
        if (cur & ARMING_CHECK_ALL_BIT) ~= 0 then
            cur = (cur & (~ARMING_CHECK_ALL_BIT)) | ARMING_CHECK_EXPANDED
        end
        target = cur & (~ARMING_CHECK_RNGFND)
    end

    if target == original then
        return true
    end
    if not arming_check_param:set_and_save(target) then
        gcs:send_text(4, "Radar: set ARM err")
        return false
    end
    gcs:send_text(6, string.format("Radar: ARM %d->%d", original, target))
    return true
end

------------------------------------------------------------------
-- Phase 1：开机检测 + 决策
------------------------------------------------------------------
local function apply_radar_ok()
    local _, c1 = set_param_persist(rngfnd_param, "RNGFND1_TYPE", TARGET_RNGFND_TYPE)
    local _, c2 = set_param_persist(prx_param,    "PRX1_TYPE",    TARGET_PRX_TYPE)
    set_param_persist(avoid_param,  "AVOID_ENABLE", AVOID_OFF)
    set_arming_rngfnd_check(false)

    if c1 or c2 then
        gcs:send_text(4, "Radar: OK, reboot for avoid")
        if not arming:is_armed() then
            vehicle:reboot(false)
        end
    else
        gcs:send_text(6, "Radar: OK")
    end
end

-- 曾见雷达迹象但未稳定 Good：保持驱动，只关避障
local function apply_radar_degraded()
    local _, c1 = set_param_persist(rngfnd_param, "RNGFND1_TYPE", TARGET_RNGFND_TYPE)
    local _, c2 = set_param_persist(prx_param,    "PRX1_TYPE",    0)
    set_param_persist(avoid_param, "AVOID_ENABLE", AVOID_OFF)
    set_arming_rngfnd_check(false)

    if c1 or c2 then
        gcs:send_text(4, "Radar: unstable, reboot")
        if not arming:is_armed() then
            vehicle:reboot(false)
        end
    else
        gcs:send_text(4, "Radar: unstable, AVOID=1")
    end
end

-- 全程无节点且无 Good：只关避障，若不需报错则需改 TYPE=0
local function apply_radar_absent()
    set_param_persist(rngfnd_param, "RNGFND1_TYPE", TARGET_RNGFND_TYPE)
    local _, c2 = set_param_persist(prx_param,    "PRX1_TYPE",    0)
    set_param_persist(avoid_param,  "AVOID_ENABLE", AVOID_OFF)
    set_arming_rngfnd_check(false)
    if c2 then
        gcs:send_text(4, "Radar: absent, reboot to clear err")
        if not arming:is_armed() then
            vehicle:reboot(false)
        end
    else
        gcs:send_text(4, "Radar: absent, AVOID=1")
    end
end

local function finish_detect(radar_ok)
    if radar_ok then
        apply_radar_ok()
        phase = 2
        return
    end

    if radar_seen or had_rngfnd_good then
        apply_radar_degraded()
    else
        apply_radar_absent()
    end
    phase = 0
end

local function detect_step()
    if detect_start_ms == nil then
        detect_start_ms = millis():tofloat()
    end

    local now = millis():tofloat()

    local rngfnd_status = rangefinder and rangefinder:status_orient(RANGEFINDER_ORIENT) or 0
    local rngfnd_good   = rngfnd_status == RNGFND_STATUS_GOOD

    if arming:is_armed() then
        if rngfnd_good or had_rngfnd_good then
            finish_detect(true)
        elseif rngfnd_param:get() == TARGET_RNGFND_TYPE and
               prx_param:get()    == TARGET_PRX_TYPE then
            phase = 2
        else
            phase = 0
        end
        return
    end

    for _, handle in ipairs(nodestatus_handles) do
        local msg, source_node = handle:check_message()
        while msg do
            if source_node == RADAR_NODE_ID then
                radar_seen = true
                ensure_rngfnd_driver_enabled()
            end
            msg, source_node = handle:check_message()
        end
    end

    if rngfnd_good then
        had_rngfnd_good = true
        ensure_rngfnd_driver_enabled()
        if not first_good_ms then
            first_good_ms = now
        end
    else
        first_good_ms = nil
    end

    local elapsed = now - detect_start_ms
    local confirm = rngfnd_good and first_good_ms
                    and (now - first_good_ms >= CONFIRM_DELAY_MS)
    local timeout = elapsed >= BOOT_DELAY_MS

    if confirm then
        finish_detect(true)
    elseif timeout then
        finish_detect(rngfnd_good)
    end
end

------------------------------------------------------------------
-- Phase 2：Scripting1(300) 切换 AVOID_ENABLE
------------------------------------------------------------------
-- 任一 RCx_OPTION=300 的通道 PWM > AVOID_PWM_ON
local function is_radar_rc_on()
    if not rc:has_valid_input() then
        return false
    end
    for ch = 1, 16 do
        local opt = rc_option_params[ch]:get()
        if opt ~= nil and math.floor(opt) == SCRIPTING1_FN then
            local pwm = rc:get_pwm(ch)
            if pwm and pwm > AVOID_PWM_ON then
                return true
            end
        end
    end
    return false
end

local function apply_avoid(want_on)
    local v = want_on and AVOID_ON or AVOID_OFF
    local cur = avoid_param:get()
    if cur ~= nil and math.floor(cur) == v then
        return true
    end
    return avoid_param:set(v)
end

local function rc_toggle_step()
    -- 仅 Loiter + RC 开关打开时才启用近距避障
    local want_on = is_radar_rc_on() and vehicle:get_mode() == MODE_LOITER

    if want_on ~= last_avoid_on then
        if apply_avoid(want_on) then
            last_avoid_on = want_on
            if want_on then
                gcs:send_text(6, "Avoid ON")
            else
                gcs:send_text(6, "Avoid OFF")
            end
        else
            gcs:send_text(3, "Avoid set fail")
        end
    end
end

------------------------------------------------------------------
-- 主循环
------------------------------------------------------------------
function update()
    if phase == 1 then
        detect_step()
    elseif phase == 2 then
        rc_toggle_step()
    end
    return update, 100
end

return update, INIT_DELAY_MS
