--[[
  CAN 雷达自动检测 + RC7 飞行中避障切换 一体脚本 v6.0
  =====================================================================
  合并自：
    - 1auto_detect_radar5.1.lua  （DroneCAN 雷达识别 + 参数自适应 + ARMING_CHECK 管理）
    - 0radar_avoid_rc7_toggle3.0.lua（RC7 飞行中切换 AVOID_ENABLE）

  为什么 v6.0 把 PRX1_TYPE 从 14 改为 4
  -----------------------------------------
  实测发现雷达固件只发布 DroneCAN RangeFinder 报文
  （uavcan.equipment.range_sensor.Measurement），并未发布 Proximity 报文
  （ardupilot.equipment.proximity_sensor.Proximity）。
  因此 PRX1_TYPE=14（DroneCAN proximity）永远 NotConnected。
  改用 PRX1_TYPE=4（RangeFinder bridge），从 RNGFND1 取距离桥接到 PRX1，
  得到单方向避障（取决于 RNGFND1_ORIENT）。

  整体逻辑
  ========
  Phase 1 - 开机检测（解锁前 ≤ BOOT_DELAY_MS）
    监听 DroneCAN NodeStatus（CAN1+CAN2）+ 轮询 rangefinder:status_orient
    双重验证雷达可用：
      A. 收到 RADAR_NODE_ID 的 NodeStatus
      B. rangefinder:status_orient(RANGEFINDER_ORIENT) == Good
         持续 CONFIRM_DELAY_MS 毫秒
    达成 → radar_ok = true，提前结束等待；否则 BOOT_DELAY_MS 超时按 false 处理。

    radar_ok = true:
      RNGFND1_TYPE       = 24  (DroneCAN)
      PRX1_TYPE          = 4   (RangeFinder bridge)
      AVOID_ENABLE       = 7   (Fence + Proximity + Beacon)
      ARMING_CHECK bit15 = 0   (恒清零，避免雷达瞬时异常卡解锁)
      → 任一项有变化时提示 "REBOOT to enable avoidance"

    radar_ok = false:
      RNGFND1_TYPE       = 0
      PRX1_TYPE          = 0
      AVOID_ENABLE       = 1   (仅 Fence，跳过近距避障)
      ARMING_CHECK bit15 = 0   (恒清零)
      → 不阻塞解锁

  备注：bit15 在两种状态下都清零，因为用户希望 PRX/RNGFND 的
  解锁前检查永不阻塞解锁，避障可用与否完全由 AVOID_ENABLE 与
  实际数据驱动。

  Phase 2 - 飞行中 RC7 切换（仅 radar_ok 时启用）
    每 100 ms 检测一次 RC 通道：
      RC7 PWM > 1500 且非 AltHold(mode=2) 且 RC 有效  → AVOID_ENABLE = 7
      其他情况                                          → AVOID_ENABLE = 1
    radar_ok = false 时 Phase 2 不启用，AVOID_ENABLE 保持 1。

  配置项
  --------
  RADAR_NODE_ID      : DroneCAN 监视器里雷达的 Node ID
  RANGEFINDER_ORIENT : 雷达朝向（0=前, 25=下, 详见 ROTATION_* 枚举）
  BOOT_DELAY_MS      : 无雷达时的最长等待（毫秒）
  CONFIRM_DELAY_MS   : 看到雷达后再观察稳定的时长（毫秒）
  RC_CH              : 切换避障的 RC 通道（默认 7）
  PWM_THRESHOLD      : 切换阈值（默认 1500）

  依赖
  ----
  CAN_Px_DRIVER=1, CAN_Dx_PROTOCOL=1, SCR_ENABLE=1
  RNGFND1_ADDR 必须与雷达 DroneCAN Rangefinder 的 sensor_id 匹配
--]]

------------------------------------------------------------------
-- 可调参数
------------------------------------------------------------------
local RADAR_NODE_ID      = 120     -- 【必改】雷达的 DroneCAN Node ID
local RANGEFINDER_ORIENT = 0       -- 雷达朝向：0=前, 25=下, 4=后, 2=右, 6=左
local BOOT_DELAY_MS      = 12000   -- 无雷达时最长等待
local CONFIRM_DELAY_MS   = 3000    -- Good 持续多久才认定稳定

local RC_CH              = 7       -- RC 切换通道
local PWM_THRESHOLD      = 1500    -- > 阈值视为打开避障

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

-- DroneCAN NodeStatus
local NODESTATUS_ID        = 341
local NODESTATUS_SIGNATURE = uint64_t(0x0F0868D0, 0xC1A7C6F1)

local MODE_ALTHOLD = 2

------------------------------------------------------------------
-- 参数对象
------------------------------------------------------------------
local rngfnd_param       = Parameter("RNGFND1_TYPE")
local prx_param          = Parameter("PRX1_TYPE")
local avoid_param        = Parameter("AVOID_ENABLE")
local arming_check_param = Parameter("ARMING_CHECK")

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
    gcs:send_text(3, "Radar6.0: no DroneCAN handle, abort")
    return
end

gcs:send_text(6, string.format("Radar6.0: started, node=%d, CAN drivers=%d",
    RADAR_NODE_ID, #nodestatus_handles))

------------------------------------------------------------------
-- 状态
------------------------------------------------------------------
local phase           = 1          -- 1=detect, 2=rc7 toggle, 0=disabled
local radar_seen      = false      -- 收到了 RADAR_NODE_ID 的 NodeStatus
local first_good_ms   = nil        -- rangefinder Good 起始时间
local last_avoid_on   = nil        -- Phase 2 上次 AVOID 状态

------------------------------------------------------------------
-- 工具：写参数（不变就跳过），返回 ok, changed
------------------------------------------------------------------
local function set_param_persist(param, name, target)
    local cur = param:get()
    if cur == nil then
        gcs:send_text(4, "Radar6.0: get " .. name .. " err")
        return false, false
    end
    if math.floor(cur) == target then
        return true, false
    end
    if not param:set_and_save(target) then
        gcs:send_text(4, "Radar6.0: set " .. name .. " err")
        return false, false
    end
    return true, true
end

-- ARMING_CHECK bit15 管理：enable=true 设 1，false 清 0
local function set_arming_rngfnd_check(enable)
    local cur = arming_check_param:get()
    if cur == nil then
        gcs:send_text(4, "Radar6.0: get ARMING_CHECK err")
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
        gcs:send_text(4, "Radar6.0: set ARMING_CHECK err")
        return false
    end
    gcs:send_text(6, string.format("Radar6.0: ARMING_CHECK %d -> %d", original, target))
    return true
end

------------------------------------------------------------------
-- Phase 1：开机检测 + 决策
------------------------------------------------------------------
local function apply_radar_ok()
    local _, c1 = set_param_persist(rngfnd_param, "RNGFND1_TYPE", TARGET_RNGFND_TYPE)
    local _, c2 = set_param_persist(prx_param,    "PRX1_TYPE",    TARGET_PRX_TYPE)
    local _, c3 = set_param_persist(avoid_param,  "AVOID_ENABLE", AVOID_ON)
    -- bit15 始终清零：哪怕雷达 OK 也不让 PRX/RNGFND 的解锁前检查阻塞解锁
    set_arming_rngfnd_check(false)

    if c1 or c2 then
        gcs:send_text(4,
            "Radar6.0: RNGFND1_TYPE=24, PRX1_TYPE=4 saved, REBOOT to enable avoidance")
    elseif c3 then
        gcs:send_text(6, "Radar6.0: radar OK, AVOID_ENABLE=7 set")
    else
        gcs:send_text(6, "Radar6.0: radar OK, all params already correct")
    end
end

local function apply_radar_not_ok(reason_text)
    local _, c1 = set_param_persist(rngfnd_param, "RNGFND1_TYPE", 0)
    local _, c2 = set_param_persist(prx_param,    "PRX1_TYPE",    0)
    set_param_persist(avoid_param, "AVOID_ENABLE", AVOID_OFF)
    set_arming_rngfnd_check(false)

    if c1 or c2 then
        gcs:send_text(4, "Radar6.0: " .. reason_text ..
                         ", RNGFND/PRX cleared, arming unlocked")
    else
        gcs:send_text(6, "Radar6.0: " .. reason_text ..
                         ", RNGFND/PRX already 0")
    end
end

local function detect_step()
    if arming:is_armed() then
        -- 解锁前没完成检测就直接解锁了：保守按当前参数继续
        if rngfnd_param:get() == TARGET_RNGFND_TYPE and
           prx_param:get()    == TARGET_PRX_TYPE then
            phase = 2
        else
            phase = 0
        end
        return
    end

    local now = millis()

    for _, handle in ipairs(nodestatus_handles) do
        local msg, source_node = handle:check_message()
        while msg do
            if source_node == RADAR_NODE_ID then
                radar_seen = true
            end
            msg, source_node = handle:check_message()
        end
    end

    local rngfnd_status = rangefinder and rangefinder:status_orient(RANGEFINDER_ORIENT) or 0
    local rngfnd_good   = rngfnd_status == RNGFND_STATUS_GOOD
    if rngfnd_good then
        if not first_good_ms then first_good_ms = now end
    else
        first_good_ms = nil
    end

    local confirm = radar_seen and rngfnd_good and first_good_ms
                    and (now - first_good_ms > CONFIRM_DELAY_MS)
    local timeout = now > BOOT_DELAY_MS

    if confirm or timeout then
        local radar_ok = radar_seen and rngfnd_good
        if radar_ok then
            apply_radar_ok()
            phase = 2    -- 启用 RC7 切换
        else
            local reason
            if radar_seen and not rngfnd_good then
                reason = string.format("Node %d online but RNGFND status=%d (check RNGFND1_ADDR vs sensor_id)",
                    RADAR_NODE_ID, rngfnd_status)
            elseif not radar_seen then
                reason = string.format("Node %d NodeStatus not seen, radar offline?", RADAR_NODE_ID)
            else
                reason = "radar not OK"
            end
            apply_radar_not_ok(reason)
            phase = 0    -- 禁用 RC7 切换
        end
    end
end

------------------------------------------------------------------
-- Phase 2：RC7 切换 AVOID_ENABLE
------------------------------------------------------------------
local function apply_avoid(want_on)
    local v = want_on and AVOID_ON or AVOID_OFF
    if avoid_param:get() == v then
        return true
    end
    return avoid_param:set(v)
end

local function rc_toggle_step()
    local want_on
    local reason

    if not rc:has_valid_input() then
        want_on = false
        reason = "no RC"
    else
        local pwm = rc:get_pwm(RC_CH)
        if pwm == nil then
            want_on = true
            reason = "RC7 n/a"
        else
            want_on = pwm > PWM_THRESHOLD
            reason = string.format("ch7=%u", pwm)
        end
    end

    if want_on then
        local mode = vehicle:get_mode()
        if mode == MODE_ALTHOLD then
            want_on = false
            reason = "AltHold no avoid"
        end
    end

    if want_on ~= last_avoid_on then
        if apply_avoid(want_on) then
            last_avoid_on = want_on
            if want_on then
                gcs:send_text(6, string.format("Avoid ON: %s", reason))
            else
                gcs:send_text(6, string.format("Avoid OFF: %s", reason))
            end
        else
            gcs:send_text(3, "AVOID_ENABLE set failed")
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
    -- phase == 0：雷达不 OK，AVOID_ENABLE 已固定为 1，不再做任何事
    return update, 100
end

return update, 3000
