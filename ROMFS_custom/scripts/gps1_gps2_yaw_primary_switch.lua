--[[
  脚本名称: gps1_gps2_yaw_primary_switch.lua  v6.2
  适用场景: EFT_CAAC 机控
              GPS1 = ublox GPS                       (instance 0)
              GPS2 = UM982 双天线 RTK on SERIAL7     (instance 1)

  功能 (3 种工作状态):
    1. GPS1 (ublox) + GPS2 (RTK) 双在线, GPS2 状态 >= 3D Fix 且双天线航向有效:
         GPS1_TYPE      = 1   (Auto, 识别 ublox)
         GPS2_TYPE      = 25  (UM982 / Unicore moving baseline, 启用 RTK)
         EK3_SRC1_YAW   = 2   (使用 GPS 双天线 yaw)
         GPS_PRIMARY    = 1   (优先 GPS2)
         COMPASS_USE    = 1   (启用外置罗盘, ublox 内磁)
    2. 仅识别 GPS1 (GPS2 离线 / 状态 < 3D Fix / 双天线航向丢失), 即 RTK 故障:
         GPS1_TYPE      = 1
         GPS2_TYPE      = 0   (关闭 GPS2)
         EK3_SRC1_YAW   = 1   (使用外置磁罗盘 yaw)
         GPS_PRIMARY    = 0   (优先 GPS1)
         COMPASS_USE    = 1   (启用外置罗盘)
    3. 仅识别 GPS2 RTK (ublox GPS1 掉线, gps:status(0)==0):
         GPS1_TYPE      = 0   (关闭 GPS1)
         GPS2_TYPE      = 25  (UM982 / Unicore moving baseline, 启用 RTK)
         EK3_SRC1_YAW   = 2   (使用 GPS 双天线 yaw)
         GPS_PRIMARY    = 1   (使用 GPS2)
         COMPASS_USE    = 0   (ublox 内磁关闭)

  v6.0 变更:
  - read_state() 新增 RTK 双天线航向检测 (gps:gps_yaw_deg):
      任一天线丢失 → yaw=nil → 降级为 STATE_GPS1_ONLY (切外置罗盘)
  - 解锁后 (armed) RTK 故障自动切换:
      若当前为 STATE_BOTH_OK, RTK 持续 3 秒故障 → apply_state(GPS1_ONLY)
      切换后直到落地/重启才恢复 RTK 模式, 防止空中频繁切换

  v6.1 变更:
  - 修复 RTK 拔出后再插回无法被识别 (GPS2_TYPE 不会改回 25) 的问题:
      GPS1_ONLY 状态下 GPS2_TYPE 被写成 0, AP_GPS 不再创建 GPS2 驱动,
      热插回 RTK 时 status(1) 恒为 0, 永远回不到 BOTH_OK. 现新增运行期
      再探测: 未解锁且处于 GPS1_ONLY 时每 15 秒把 GPS2_TYPE 临时设回 25
      开 8 秒探测窗口, 检测到 UM982 则切回 BOTH_OK, 否则关回 0 抑制告警.

  v6.2 变更:
  - 对称修复 GPS1 (ublox) 拔出后再插回无法识别的问题:
      GPS2_ONLY 状态下 GPS1_TYPE 被写成 0, AP_GPS 不再创建 GPS1 驱动,
      热插回 ublox 时 status(0) 恒为 0, 永远回不到 BOTH_OK。新增 GPS1 运行期
      再探测: 未解锁且处于 GPS2_ONLY 时每 15 秒把 GPS1_TYPE 临时设回 1(Auto)
      开 5 秒探测窗口, 检测到 ublox 则切回 BOTH_OK, 否则关回 0。

  安全约束:
  - 未解锁状态才修改 EK3_SRC1_YAW / GPS_PRIMARY, 避免 EKF yaw 参考突变
    (v6.0 例外: 解锁后 RTK 故障强制切换, 此时 RTK 已丢失, 切换为磁罗盘更安全)
  - 已解锁但 RTK 恢复: 保持 GPS1_ONLY 模式, 落地后重新确认
  - 防抖: 未解锁 5 次 (5 秒) 确认后才切换; 已解锁 3 次确认后切换
  - 状态不变时不写 flash, 不反复磨损
--]]

---@diagnostic disable: need-check-nil, cast-local-type, assign-type-mismatch, param-type-mismatch

local RUN_INTERVAL_MS  = 1000   -- 运行频率 (1 秒)
local STARTUP_DELAY_MS = 5000   -- 启动延迟 5 秒再开始判断 (让 GPS / UM982 上电稳定)
local CONFIRM_COUNT    = 5      -- 需要 5 次 (5 秒) 连续确认后切换, 防抖

local GPS1_INSTANCE    = 0      -- ublox GPS1 (Lua 实例号 0)
local GPS2_INSTANCE    = 1      -- UM982 RTK GPS2 (Lua 实例号 1)
local MIN_GPS_STATUS   = 3      -- 3=3D Fix, 4=DGPS, 5=RTK Float, 6=RTK Fixed

-- EK3_SRC1_YAW 选项 (见 AP_NavEKF_Source.h SourceYaw)
local YAW_COMPASS = 1
local YAW_GPS     = 2

-- GPS_PRIMARY 选项: 0=GPS1, 1=GPS2
local PRIMARY_GPS1 = 0
local PRIMARY_GPS2 = 1

-- GPS_TYPE 选项
local TYPE_NONE  = 0    -- 关闭该 GPS 实例
local TYPE_AUTO  = 1    -- ublox 自动识别
local TYPE_UM982 = 25   -- UM982 / Unicore moving baseline NMEA

-- COMPASS_USE 选项: 0=关闭, 1=启用
local COMPASS_OFF = 0
local COMPASS_ON  = 1

local STATE_UNKNOWN   = 0
local STATE_BOTH_OK   = 1   -- GPS1 在线 + GPS2 状态 >= 3D Fix 且双天线航向有效
local STATE_GPS1_ONLY = 2   -- GPS1 在线, GPS2 离线 / 状态不足 / 航向丢失
local STATE_GPS2_ONLY = 3   -- GPS1 掉线 (status==0), GPS2 RTK 在线且航向有效

local SEV_INFO = 6
local SEV_WARN = 4

local current_state   = STATE_UNKNOWN
local pending_state   = STATE_UNKNOWN
local pending_count   = 0
local last_warn_ms    = 0
local WARN_REPEAT_MS  = 10000  -- 已解锁有告警间隔再隔 10 秒

-- 解锁后 RTK 故障防抖计数
local armed_rtk_fail_count     = 0
local ARMED_RTK_FAIL_THRESHOLD = 3   -- 连续 3 秒检测到故障才切换

-- ── GPS2 (UM982) 运行期再探测 ─────────────────────────────────────────────
-- 问题: GPS1_ONLY 状态下 GPS2_TYPE 被写成 0, AP_GPS 不再创建 GPS2 驱动,
--       此时热插回 RTK 也无法被识别 (status(1) 恒为 0, 永远回不到 BOTH_OK).
-- 方案: 未解锁且处于 GPS1_ONLY 时, 周期性把 GPS2_TYPE 临时设回 25 开一个
--       探测窗口. 窗口内检测到 UM982 → 正常切回 BOTH_OK; 窗口超时仍无数据
--       → 设回 0, 继续抑制 "GPS 2: not healthy" 告警.
local reprobe_active     = false
local reprobe_start_ms   = 0
local last_reprobe_ms    = 0
local REPROBE_INTERVAL_MS = 15000   -- 每 15 秒发起一次再探测
local REPROBE_WINDOW_MS   = 8000    -- 探测窗口 8 秒 (UM982 上电稳定需要时间)

-- ── GPS1 (ublox) 运行期再探测 ─────────────────────────────────────────────
-- 对称问题: GPS2_ONLY 状态下 GPS1_TYPE 被写成 0, AP_GPS 不再创建 GPS1 驱动,
--           热插回 ublox 也无法被识别 (status(0) 恒为 0, 永远回不到 BOTH_OK).
-- 方案: 未解锁且处于 GPS2_ONLY 时, 周期性把 GPS1_TYPE 临时设回 1(Auto) 探测,
--       检测到 ublox → 切回 BOTH_OK; 窗口超时仍无数据 → 设回 0.
local reprobe1_active    = false
local reprobe1_start_ms  = 0
local last_reprobe1_ms   = 0
local REPROBE1_WINDOW_MS  = 5000    -- ublox 探测窗口 5 秒 (ublox 上电较快)


-- 应用一组参数, 仅当值不同时才写 flash
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

-- 启动探测: 若 GPS1_TYPE 当前为 0 (上次切换写入), 恢复 1 (Auto) 以便重新探测 ublox
local function probe_gps1_on_startup()
    local cur = param:get("GPS1_TYPE")
    if cur ~= nil and cur == TYPE_NONE then
        param:set_and_save("GPS1_TYPE", TYPE_AUTO)
    end
end

-- 启动探测: 若 GPS2_TYPE 当前为 0 (上次 RTK 故障时写入), 恢复 25 (UM982) 重新探测
local function probe_gps2_on_startup()
    local cur = param:get("GPS2_TYPE")
    if cur ~= nil and cur == TYPE_NONE then
        param:set_and_save("GPS2_TYPE", TYPE_UM982)
    end
end

-- 安全读取 GPS 状态 (instance 超出 num_sensors 时返回 0)
local function safe_gps_status(instance)
    local n = gps:num_sensors() or 0
    if instance >= n then
        return 0
    end
    return gps:status(instance) or 0
end

-- 读取当前应处于的状态
-- v6.0: 增加双天线航向有效性检查
--   RTK 有 3D Fix 但 yaw=nil (任一天线丢失) → 降级为 GPS1_ONLY
local function read_state()
    local g1_status  = safe_gps_status(GPS1_INSTANCE)
    local g2_status  = safe_gps_status(GPS2_INSTANCE)
    local g1_present = (g1_status >= 1)               -- GPS1 硬件在线 (>=NO_FIX)
    local g2_ok      = (g2_status >= MIN_GPS_STATUS)  -- GPS2 状态足够

    -- 检查 RTK 双天线航向是否有效 (任一天线丢失则 nil)
    local rtk_yaw, _, _ = gps:gps_yaw_deg(GPS2_INSTANCE)
    local g2_yaw_ok = (rtk_yaw ~= nil)

    if g1_present and g2_ok and g2_yaw_ok then
        return STATE_BOTH_OK
    elseif g1_present then
        -- GPS2 离线 / 状态不足 / 双天线航向丢失 → 仅 GPS1
        return STATE_GPS1_ONLY
    elseif g2_ok and g2_yaw_ok then
        -- GPS1 掉线, GPS2 RTK 双天线均正常
        return STATE_GPS2_ONLY
    end
    return STATE_UNKNOWN
end

local function apply_state(state)
    if state == STATE_BOTH_OK then
        -- 双 GPS 均在线: GPS2 提供位置 + 双天线 yaw, 外置罗盘保持启用
        local a = set_param_if_diff("GPS1_TYPE",    TYPE_AUTO)
        local e = set_param_if_diff("GPS2_TYPE",    TYPE_UM982)
        local b = set_param_if_diff("EK3_SRC1_YAW", YAW_GPS)
        local c = set_param_if_diff("GPS_PRIMARY",  PRIMARY_GPS2)
        local d = set_param_if_diff("COMPASS_USE",  COMPASS_ON)
        if a or b or c or d or e then
            gcs:send_text(SEV_INFO, "GPS: dual RTK")
        end
    elseif state == STATE_GPS1_ONLY then
        -- RTK 故障 (含任一天线丢失): 切回 GPS1 定点 + 外置磁罗盘定向
        local a = set_param_if_diff("GPS1_TYPE",    TYPE_AUTO)
        local e = set_param_if_diff("GPS2_TYPE",    TYPE_NONE)
        local b = set_param_if_diff("EK3_SRC1_YAW", YAW_COMPASS)
        local c = set_param_if_diff("GPS_PRIMARY",  PRIMARY_GPS1)
        local d = set_param_if_diff("COMPASS_USE",  COMPASS_ON)
        if a or b or c or d or e then
            gcs:send_text(SEV_WARN, "GPS: GPS1 only")
        end
    elseif state == STATE_GPS2_ONLY then
        -- 注意写入顺序: 先设 GPS_PRIMARY=1 (GPS2), 再关 GPS1_TYPE=0,
        -- 避免 "GPS 1: primary but TYPE 0" 中间态告警
        local e = set_param_if_diff("GPS2_TYPE",    TYPE_UM982)
        local a = set_param_if_diff("EK3_SRC1_YAW", YAW_GPS)
        local b = set_param_if_diff("GPS_PRIMARY",  PRIMARY_GPS2)
        local c = set_param_if_diff("GPS1_TYPE",    TYPE_NONE)
        local d = set_param_if_diff("COMPASS_USE",  COMPASS_OFF)
        if a or b or c or d or e then
            gcs:send_text(SEV_WARN, "GPS: RTK only")
        end
    end
end

local function read_state_name(s)
    if s == STATE_BOTH_OK    then return "BOTH_OK"        end
    if s == STATE_GPS1_ONLY  then return "GPS1_ONLY"      end
    if s == STATE_GPS2_ONLY  then return "GPS2_ONLY(RTK)" end
    return "UNKNOWN"
end

function update()
    local s = read_state()

    -- ── 已解锁 ────────────────────────────────────────────────────────────────
    if arming:is_armed() then

        -- v6.0: RTK 解锁后故障自动切换
        -- 当前为 RTK 双天线模式, 且检测到 RTK 降级 (位置故障 / 任一天线丢失)
        if current_state == STATE_BOTH_OK then
            if s == STATE_GPS1_ONLY or s == STATE_UNKNOWN then
                armed_rtk_fail_count = armed_rtk_fail_count + 1
                if armed_rtk_fail_count >= ARMED_RTK_FAIL_THRESHOLD then
                    -- 连续确认后切换: GPS1 定点 + 外置磁罗盘定向
                    apply_state(STATE_GPS1_ONLY)
                    current_state = STATE_GPS1_ONLY
                    armed_rtk_fail_count = 0
                    gcs:send_text(SEV_WARN, "RTK lost: GPS1+compass")
                end
            else
                -- RTK 仍正常 (或短暂恢复), 重置计数
                armed_rtk_fail_count = 0
            end
        end

        -- 非 RTK 故障切换时的状态变化告警 (仅提示, 不改参数)
        if current_state ~= STATE_UNKNOWN and s ~= STATE_UNKNOWN and s ~= current_state then
            local now = millis()
            if (now - last_warn_ms) > WARN_REPEAT_MS then
                gcs:send_text(SEV_WARN, string.format(
                    "GPS %s->%s (armed)",
                    read_state_name(current_state), read_state_name(s)))
                last_warn_ms = now
            end
        end

        -- 解锁期间不通过 pending 机制修改其他参数, 落地后重新确认
        pending_state = STATE_UNKNOWN
        pending_count = 0
        return update, RUN_INTERVAL_MS
    end

    -- ── 未解锁 ────────────────────────────────────────────────────────────────

    -- 重置解锁后故障计数 (每次落地归零)
    armed_rtk_fail_count = 0

    -- GPS2 (UM982) 运行期再探测: 仅在 GPS1_ONLY (GPS2_TYPE 已被写成 0) 时进行
    if current_state == STATE_GPS1_ONLY then
        local g2_type = param:get("GPS2_TYPE")
        local now = millis()
        if not reprobe_active then
            -- 仅当 GPS2_TYPE 当前确为 0 时才需要再探测
            if g2_type ~= nil and g2_type == TYPE_NONE
               and (now - last_reprobe_ms) >= REPROBE_INTERVAL_MS then
                param:set_and_save("GPS2_TYPE", TYPE_UM982)
                reprobe_active   = true
                reprobe_start_ms = now
                gcs:send_text(SEV_INFO, "GPS2 reprobe...")
            end
        else
            -- 探测窗口进行中: 检查 UM982 是否已被识别
            local g2_status = safe_gps_status(GPS2_INSTANCE)
            local rtk_yaw, _, _ = gps:gps_yaw_deg(GPS2_INSTANCE)
            if g2_status >= MIN_GPS_STATUS and rtk_yaw ~= nil then
                -- 探测成功: 结束探测, 交给下方 pending 机制切回 BOTH_OK
                reprobe_active = false
                last_reprobe_ms = now
            elseif (now - reprobe_start_ms) >= REPROBE_WINDOW_MS then
                -- 探测窗口超时仍无数据: 关回 GPS2_TYPE=0, 抑制告警
                param:set_and_save("GPS2_TYPE", TYPE_NONE)
                reprobe_active  = false
                last_reprobe_ms = now
            end
        end
    else
        -- 非 GPS1_ONLY 状态 (GPS2_TYPE 已是 25), 无需再探测
        reprobe_active = false
    end

    -- GPS1 (ublox) 运行期再探测: 仅在 GPS2_ONLY (GPS1_TYPE 已被写成 0) 时进行
    if current_state == STATE_GPS2_ONLY then
        local g1_type = param:get("GPS1_TYPE")
        local now = millis()
        if not reprobe1_active then
            -- 仅当 GPS1_TYPE 当前确为 0 时才需要再探测
            if g1_type ~= nil and g1_type == TYPE_NONE
               and (now - last_reprobe1_ms) >= REPROBE_INTERVAL_MS then
                param:set_and_save("GPS1_TYPE", TYPE_AUTO)
                reprobe1_active   = true
                reprobe1_start_ms = now
                gcs:send_text(SEV_INFO, "GPS1 reprobe...")
            end
        else
            -- 探测窗口进行中: 检查 ublox 是否已被识别
            if safe_gps_status(GPS1_INSTANCE) >= 1 then
                -- 探测成功: 结束探测, 交给下方 pending 机制切回 BOTH_OK
                reprobe1_active  = false
                last_reprobe1_ms = now
            elseif (now - reprobe1_start_ms) >= REPROBE1_WINDOW_MS then
                -- 探测窗口超时仍无数据: 关回 GPS1_TYPE=0, 抑制 GPS1 告警
                param:set_and_save("GPS1_TYPE", TYPE_NONE)
                reprobe1_active  = false
                last_reprobe1_ms = now
            end
        end
    else
        reprobe1_active = false
    end

    -- 双 GPS 均未出现有效数据时不切换
    if s == STATE_UNKNOWN then
        pending_state = STATE_UNKNOWN
        pending_count = 0
        return update, RUN_INTERVAL_MS
    end

    -- 当前已是目标状态: 清空 pending
    if s == current_state then
        pending_state = STATE_UNKNOWN
        pending_count = 0
        return update, RUN_INTERVAL_MS
    end

    -- 状态变化: 累计计数防抖
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

    return update, RUN_INTERVAL_MS
end

probe_gps1_on_startup()
probe_gps2_on_startup()
return update, STARTUP_DELAY_MS
