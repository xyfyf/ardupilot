--[[
  UAVCAN/DroneCAN 雷达自动检测与参数适配脚本 v5.0
  =====================================================================
  与 v4.0 的核心区别：
  1) 无雷达时把 PRX1_TYPE 写回 0（不再"保留 14"）。
     原因：v4.0 保留 EEPROM 里的 14，导致下次开机 num_instances=1，
     而雷达未就绪时 prearm_healthy() 返回 false，即使修改 C++ 让
     proximity_checks 受 ARMING_CHECK bit15 控制，EEPROM 里的 14 也会
     让飞控在下下次重启（雷达又断了）时卡住。v5.0 双向写，彻底解决。
  2) 配合已修改的 AP_Arming.cpp（proximity_checks 受 ARMING_CHECK bit15
     控制），无雷达 + bit15=0 时解锁不阻塞、不报 CRITICAL 错误。
  3) 改进 ARMING_CHECK bit15 的管理：有雷达 → 置 1；无雷达 → 清 0。
     这样就算 C++ 没修改，飞控也能通过 ARMING_CHECK 旁路。

  行为时序（含早退出优化）
  --------
  开机后脚本以 100 ms 周期监听 DroneCAN NodeStatus。
  以下任一条件成立即做决策：
    (A) 看到雷达 + 持续观察 CONFIRM_DELAY_MS（默认 3 s）确认稳定
        → 提前完成（接好雷达时 3~5 s 即出结果）
    (B) 到 BOOT_DELAY_MS（默认 12 s）仍未看到 → 判定无雷达
  决策动作：
    radar_seen=true  → PRX1_TYPE 持久化为 14；ARMING_CHECK bit15 → 1
                       若原值不是 14，打印 "REBOOT to enable avoidance"
    radar_seen=false → PRX1_TYPE 持久化为 0；ARMING_CHECK bit15 → 0
                       "No radar: PRX1_TYPE=0, arming unlocked"
  解锁后立即冻结脚本（check_done=true），飞行中绝不再动参数。

  配置项
  --------
  RADAR_NODE_ID    : 雷达在 DroneCAN 监视器里显示的 Node ID（必改）
  BOOT_DELAY_MS    : 无雷达时的最长等待，毫秒，默认 12000（旧版 30000）
  CONFIRM_DELAY_MS : 看到雷达后再观察确认稳定的时长，毫秒，默认 3000

  依赖
  ----
  CAN_Px_DRIVER=1, CAN_Dx_PROTOCOL=1, SCR_ENABLE=1
  脚本会同时监听 DroneCAN driver 0 和 1（CAN1/CAN2），雷达接哪路 CAN 都能识别
--]]

local RADAR_NODE_ID    = 120   -- 【必改】DroneCAN 监视器里雷达的 Node ID
local BOOT_DELAY_MS    = 12000 -- 无雷达时的最长等待（毫秒）。雷达上线快可改 8000
local CONFIRM_DELAY_MS = 3000  -- 看到雷达后再观察这么久确认稳定（避免上电瞬间的噪声）

local TARGET_PRX_TYPE = 14   -- DroneCAN proximity

local prx_param          = Parameter("PRX1_TYPE")
local arming_check_param = Parameter("ARMING_CHECK")

local ARMING_CHECK_ALL_BIT  = 1 << 0
local ARMING_CHECK_RNGFND   = 1 << 15
local ARMING_CHECK_EXPANDED = 0xFFFFE  -- bits 1..19，不含 bit0

-- DroneCAN NodeStatus 订阅（同时监听 CAN1=driver0 和 CAN2=driver1）
local NODESTATUS_ID        = 341
local NODESTATUS_SIGNATURE = uint64_t(0x0F0868D0, 0xC1A7C6F1)

local nodestatus_handles = {}
for driver_idx = 0, 1 do
    local h = DroneCAN_Handle(driver_idx, NODESTATUS_SIGNATURE, NODESTATUS_ID)
    if h then
        h:subscribe()
        nodestatus_handles[#nodestatus_handles + 1] = h
    end
end

if #nodestatus_handles == 0 then
    gcs:send_text(3, "Radar5.0: no DroneCAN handle, abort")
    return
end

gcs:send_text(6, string.format("Radar5.0: started, node=%d, CAN drivers=%d",
    RADAR_NODE_ID, #nodestatus_handles))

local radar_seen        = false
local first_seen_ms     = nil   -- 第一次看到雷达 NodeStatus 的时间戳
local check_done        = false

-- 把 PRX1_TYPE 持久化；返回 ok, changed
local function set_prx_type(v)
    local cur = prx_param:get()
    if cur == nil then
        gcs:send_text(4, "Radar5.0: get PRX1_TYPE err")
        return false, false
    end
    if math.floor(cur) == v then
        return true, false
    end
    if not prx_param:set_and_save(v) then
        gcs:send_text(4, "Radar5.0: set PRX1_TYPE err")
        return false, false
    end
    return true, true
end

-- 管理 ARMING_CHECK bit15（Rangefinder/Proximity 解锁前检查）
-- enable=true  → 置 1（雷达在线，恢复检查）
-- enable=false → 清 0（无雷达，跳过检查，避免无雷达无法解锁）
local function set_arming_rngfnd_check(enable)
    local cur = arming_check_param:get()
    if cur == nil then
        gcs:send_text(4, "Radar5.0: get ARMING_CHECK err")
        return false
    end
    cur = math.floor(cur)
    local original = cur

    local target
    if enable then
        target = cur | ARMING_CHECK_RNGFND
    else
        -- 若 ALL(bit0)=1，先展开为显式 bits 1..19，再清 bit15
        if (cur & ARMING_CHECK_ALL_BIT) ~= 0 then
            cur = (cur & (~ARMING_CHECK_ALL_BIT)) | ARMING_CHECK_EXPANDED
        end
        target = cur & (~ARMING_CHECK_RNGFND)
    end

    if target == original then
        return true
    end
    if not arming_check_param:set_and_save(target) then
        gcs:send_text(4, "Radar5.0: set ARMING_CHECK err")
        return false
    end
    gcs:send_text(6, string.format("ARMING_CHECK: %d -> %d", original, target))
    return true
end

function update()
    if check_done then return end

    -- 解锁后立即停止，不再修改任何参数
    if arming:is_armed() then
        check_done = true
        return
    end

    -- 轮询所有 DroneCAN 总线上的 NodeStatus
    local now = millis()
    for _, handle in ipairs(nodestatus_handles) do
        local msg, source_node = handle:check_message()
        while msg do
            if source_node == RADAR_NODE_ID then
                if not radar_seen then
                    first_seen_ms = now
                end
                radar_seen = true
            end
            msg, source_node = handle:check_message()
        end
    end

    -- 决策时机：
    -- (A) 看到雷达且观察 CONFIRM_DELAY_MS 后稳定 → 提前完成
    -- (B) 到 BOOT_DELAY_MS 仍没看到雷达 → 判定无雷达
    local early_confirm = radar_seen and first_seen_ms and (now - first_seen_ms > CONFIRM_DELAY_MS)
    local timeout       = now > BOOT_DELAY_MS

    if early_confirm or timeout then
        if radar_seen then
            -- ---- 检测到雷达 ----
            local ok, changed = set_prx_type(TARGET_PRX_TYPE)
            if ok and changed then
                gcs:send_text(4, "Radar5.0: PRX1_TYPE=14 saved, REBOOT to enable avoidance")
            elseif ok then
                gcs:send_text(6, "Radar5.0: PRX1_TYPE=14 OK, avoidance live")
            end
            set_arming_rngfnd_check(true)
            gcs:send_text(6, "Radar5.0: ARMING_CHECK bit15=1 (radar online)")
        else
            -- ---- 未检测到雷达 ----
            -- v5.0 关键改动：写回 0，不再保留 14，避免下次开机 proximity_checks 阻止解锁
            local ok, changed = set_prx_type(0)
            if ok and changed then
                gcs:send_text(4, "Radar5.0: No radar, PRX1_TYPE=0 saved, arming unlocked")
            else
                gcs:send_text(6, "Radar5.0: No radar seen, PRX1_TYPE already 0")
            end
            set_arming_rngfnd_check(false)
            gcs:send_text(6, "Radar5.0: ARMING_CHECK bit15=0 (no radar)")
        end

        check_done = true
        return
    end

    return update, 100
end

return update, 3000
