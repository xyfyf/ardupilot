--[[
  脚本名称: gps1_gps2_yaw_primary_switch.lua  v6.12
  适用场景: EFT_CAAC 机控
              GPS1 = ublox GPS                       (instance 0)
              GPS2 = UM982 双天线 RTK on SERIAL7     (instance 1)

  功能 (4 种工作状态):
    1. GPS1 (ublox) + GPS2 (RTK) 双在线, GPS2 状态 >= 3D Fix 且双天线航向有效:
         GPS1_TYPE      = 1   (Auto, 识别 ublox)
         GPS2_TYPE      = 25  (UM982 / Unicore moving baseline, 启用 RTK)
         EK3_SRC1_YAW   = 2   (使用 GPS 双天线 yaw)
         GPS_PRIMARY    = 1   (优先 GPS2)
         GPS_AUTO_SWITCH= 0   (NONE, 主 GPS 固定 GPS2, 不按 fix 等级被 ublox 抢走)
         COMPASS_USE/2/3= 0   (禁用罗盘, 强制 EKF 等待并使用 RTK 航向)
    2. 冷启动等 RTK 航向 (未解锁, UM982 在线但双天线航向尚未解算, 宽限 GPSYS_RTK_WAIT 秒):
         GPS1_TYPE      = 1
         GPS2_TYPE      = 25  (保持 UM982 在线收敛)
         EK3_SRC1_YAW   = 2   (坚持等 GPS 双天线 yaw, 不切磁罗盘)
         GPS_PRIMARY    = 1   (主 GPS 仍指 GPS2)
         GPS_AUTO_SWITCH= 0
         COMPASS_USE/2/3= 0   (禁用罗盘, 避免 EKF 提前用磁罗盘初始化航向)
    3. RTK 宽限超时 / 天线丢失 / 模块离线 / 飞行中 RTK 故障:
         GPS1_TYPE      = 1
         GPS2_TYPE      = 25  (始终保持 UM982 驱动, 不写 0)
         EK3_SRC1_YAW   = 1   (使用外置磁罗盘 yaw)
         GPS_PRIMARY    = 0   (只用 GPS1 定位)
         GPS_AUTO_SWITCH= 0   (NONE, 主 GPS 固定 GPS1)
         COMPASS_USE/2/3= 1   (启用外置罗盘, 定点模式可用 GPS1+磁罗盘解锁)
    4. 仅识别 GPS2 RTK (ublox GPS1 掉线, gps:status(0)==0), RTK 完好:
         GPS1_TYPE      = 0   (关闭 GPS1, 并周期性 reprobe 等待热插回)
         GPS2_TYPE      = 25  (UM982 保持在线)
         EK3_SRC1_YAW   = 2   (使用 GPS 双天线 yaw)
         GPS_PRIMARY    = 1   (使用 GPS2)
         GPS_AUTO_SWITCH= 0   (NONE, 主 GPS 固定 GPS2)
         COMPASS_USE/2/3= 1   (启用外置罗盘)
         解锁拦截: GPS1 故障但 RTK 正常时禁止解锁, 须恢复 GPS1 后才能飞

  v6.12 变更 (RTK 故障不切 GPS2_TYPE, 天线丢失立即降级):
  - GPS1_ONLY 时 GPS2_TYPE 始终写 25, 不再因模块离线/拔掉/reprobe 写 0.
  - 曾成功解出过 RTK 航向后航向丢失 (主/副天线掉): 立即切 GPS1+磁罗盘, 不再等 RTK_WARMUP.
  - 仅冷启动从未出过航向时, 未解锁宽限内仍走 RTK_WARMUP 等收敛.
  - 删除 GPS2 reprobe 写 0 逻辑 (保持驱动不断电, 热插回由状态机自然恢复).

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

  v6.3 变更:
  - 修复 GPS/RTK 均未接时 gps:gps_yaw_deg(1) 实例越界导致 Lua 报错及 PreArm 阻塞:
      GPS2_TYPE=0 或未探测到 GPS2 时 num_sensors<2, 须先检查实例再读 yaw.

  v6.4 变更 (根治 "RTK 插着却一直未定位", 不再依赖长延时):
  - 核心: 新增 rtk_module_online() (用 gps:last_message_time_ms 判断模块串口是否还在通信)。
    只要 UM982 还在出数据, 就绝不把 GPS2_TYPE 写 0 销毁驱动, 让它全程不断电慢慢收敛:
      · 切到 GPS1_ONLY 时, 模块在线 → 保持 GPS2_TYPE=25 (只切磁罗盘 yaw + GPS1 主), 等收敛后自动回 BOTH_OK;
      · 仅当模块真正离线 (拔掉/串口无数据) → 才写 GPS2_TYPE=0, 并交给 reprobe 热插探测。
  - reprobe 成功判据改为 "模块是否在通信" 而非 "拿到定位+航向":
    检测到模块开始出数据即结束探测并保持 25, 让其在线收敛, 彻底消除 0↔25 反复横跳的死循环。
  - 因不再需要靠长延时等收敛, STARTUP_DELAY_MS 改回 5 秒, 脚本早点开始工作。

  v6.5 变更 (根治 "RTK 航向稳定但 EK3 航向匀速漂"):
  - 现象: UM982(GPS2) 双天线航向稳定, 但 EK3 输出航向以 ~10°/秒 匀速转, EKF3 core unhealthy。
  - 根因: GPS_AUTO_SWITCH 默认 = 1(USE_BEST) 按 fix 等级选主 GPS。ublox(GPS1) 常处于
    DGPS(4), UM982 单模块 type25 只到 3D Fix(3), 于是飞控把主 GPS 选成 ublox,
    GPS_PRIMARY=1 被无视。EKF 只读主 GPS 的航向, ublox 无双天线航向 -> 航向只剩陀螺积分而漂。
  - 修复: 脚本自己当总开关, 三种状态都把 GPS_AUTO_SWITCH 写 0(NONE), 主 GPS 完全听 GPS_PRIMARY:
      · BOTH_OK / GPS2_ONLY -> 主 GPS 固定 UM982(GPS2), 航向进 EKF;
      · GPS1_ONLY (RTK 不可用) -> 主 GPS 固定 ublox(GPS1)。

  v6.11 变更 (恢复冷启动等 RTK 航向, 默认宽限 30 秒):
  - 未解锁且 UM982 在线但无航向: 先进入 RTK_WARMUP, 坚持 EK3_SRC1_YAW=2 + COMPASS_USE=0,
    等 GPSYS_RTK_WAIT 秒 (默认 30); 航向到位无缝切 BOTH_OK, 超时再切 GPS1+磁罗盘.
  - GPS1_ONLY 仍保持 v6.10: 模块在线不关 GPS2_TYPE=25, 不发 GCS 告警.

  v6.10 变更 (恢复 v6.4, 撤销 v6.9 关 GPS2 逻辑):
  - RTK 宽限超时后切 GPS1_ONLY: GPS_PRIMARY=0 + EK3_SRC1_YAW=1 + COMPASS_USE=1,
    UM982 仍在通信时保持 GPS2_TYPE=25, 不关驱动.

  v6.8 变更:
  - 需求三: GPS1 故障但 RTK 完好时禁止解锁 (aux auth prearm 拦截).
    仍保持 GPS2_ONLY 参数态 + GPS1 reprobe, 更换 GPS1 后自动识别恢复.

  v6.7 变更:
  - 修复 RTK 冷启动时 EKF3 提前融合磁罗盘航向导致 "Need Position Estimate" 报警:
      现象: RTK 刚上电时航向未就绪, EKF3 会退而融合磁罗盘航向, 导致后续即使 RTK 航向
            就绪也不再切换, 出现航向偏差报警.
      修复: 仅在 STATE_RTK_WARMUP (等待 RTK 航向解算) 期间禁用 COMPASS_USE/2/3,
            迫使 EKF3 在此窗口内无法用磁罗盘初始化航向, 只能等 RTK GPS yaw 就绪.
            RTK 航向就绪后 (STATE_BOTH_OK / STATE_GPS2_ONLY) 立即重新启用罗盘:
            此时 EKF3 航向已由 GPS yaw 锁定, 罗盘只作辅助, 不会反客为主;
            且飞行中 RTK 突然故障时 (切 STATE_GPS1_ONLY) 罗盘可立即接管航向, 保证安全.

  v6.6 变更:
  - 新增参数 GPSYS_ENABLE (地面站搜索 GPSYS_):
      0 = 禁用脚本, 不修改任何 GPS/EKF 参数, 完全由飞手手动配置;
      1 = 启用脚本 (默认), 按 GPS/RTK 状态自动切换.
  - 新增 "起飞前优先等 RTK 航向" (根治 "位置估计反复丢失"):
      现象: UM982 冷启动解算双天线航向需要时间, 期间脚本旧逻辑会先切磁罗盘航向,
            等 RTK 航向到了又切回 GPS 航向 -> EKF 航向源来回切/重对齐 -> 位置估计反复丢失。
      修复: 新增 STATE_RTK_WARMUP。起飞前 (未解锁) 只要 RTK 模块在线 (已接),
            就坚持 EK3_SRC1_YAW=2 一直等 RTK 双天线航向, 期间绝不切磁罗盘;
            RTK 航向到位后无缝转 BOTH_OK (参数一致, 不再改写)。
            只有 "没接 RTK (模块离线)" 或 "等够 GPSYS_RTK_WAIT 秒仍无航向 (副天线故障)"
            才退回磁罗盘航向 (GPS1_ONLY)。
      新增参数 GPSYS_RTK_WAIT (秒, 默认 90): 起飞前等 RTK 航向的最长宽限时间。

  安全约束:
  - 未解锁状态才修改 EK3_SRC1_YAW / GPS_PRIMARY, 避免 EKF yaw 参考突变
    (v6.0 例外: 解锁后 RTK 故障强制切换, 此时 RTK 已丢失, 切换为磁罗盘更安全)
  - 已解锁但 RTK 恢复: 保持 GPS1_ONLY 模式, 落地后重新确认
  - 防抖: 未解锁 5 次 (5 秒) 确认后才切换; 已解锁 3 次确认后切换
  - 状态不变时不写 flash, 不反复磨损
--]]

---@diagnostic disable: need-check-nil, cast-local-type, assign-type-mismatch, param-type-mismatch

-- 脚本参数 (地面站 Full Parameter List 搜索 GPSYS_)
--   GPSYS_ENABLE   : 0=禁用脚本(不改任何参数), 1=启用(默认)
--   GPSYS_RTK_WAIT : 冷启动等 RTK 双天线航向的宽限秒数 (默认 30).
--                    模块在线但航向未就绪时坚持 EK3_SRC1_YAW=2 不切磁罗盘;
--                    等够仍无航向才退回 GPS1+磁罗盘. 设为 0 表示不等待.
--   GPSYS_COMP_DLY : RTK 航向就绪后延迟多久再启用 COMPASS_USE (秒, 默认 30).
--                    等待期间 COMPASS_USE 保持 0, 让 EKF3 充分稳定在 RTK 航向后
--                    再融合罗盘, 避免刚切入时罗盘瞬时偏差干扰 EKF3.
--                    设为 0 表示 RTK 航向就绪后立即启用罗盘.
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

-- 冷启动等 RTK 航向的宽限时间 (ms), 由 GPSYS_RTK_WAIT (秒) 换算
local function rtk_wait_ms()
    local s = gpsys_rtk_wait:get() or 30
    if s < 0 then s = 0 end
    return s * 1000
end

local RUN_INTERVAL_MS  = 1000   -- 运行频率 (1 秒)
local STARTUP_DELAY_MS = 5000   -- 启动延迟 5 秒, 仅等 num_sensors 填充; 不再靠长延时等 RTK 收敛
                                -- (RTK 慢慢收敛交给 rtk_module_online() 逻辑: 模块在通信就保持 GPS2_TYPE=25 不断电等)
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

-- GPS_AUTO_SWITCH 选项 (见 AP_GPS.h GPSAutoSwitch)
--   0=NONE 始终用 GPS_PRIMARY 指定的那路 (不按 fix 等级自动切)
--   1=USE_BEST (默认) 按 fix 等级选最好的一路 -> 会让 ublox 的 DGPS(4) 压过 UM982 的 3D(3)
-- 本脚本自己当总开关, 用 GPS_PRIMARY 决定主 GPS, 故强制 NONE, 否则 GPS_PRIMARY=1 会被 USE_BEST 覆盖,
-- 导致 EKF 实际主 GPS 是没有双天线航向的 ublox -> EK3 航向只剩陀螺积分而漂.
local AUTOSW_USE_PRIMARY = 0
local AUTOSW_USE_BEST    = 1

-- GPS_TYPE 选项
local TYPE_NONE  = 0    -- 关闭该 GPS 实例
local TYPE_AUTO  = 1    -- ublox 自动识别
local TYPE_UM982 = 25   -- UM982 / Unicore moving baseline NMEA

-- COMPASS_USE 选项: 0=关闭, 1=启用
local COMPASS_OFF = 0
local COMPASS_ON  = 1

local STATE_UNKNOWN   = 0
local STATE_BOTH_OK   = 1   -- GPS1 在线 + GPS2 状态 >= 3D Fix 且双天线航向有效
local STATE_GPS1_ONLY = 2   -- GPS1 在线, RTK 无航向/离线 → GPS1 定位 + 磁罗盘 yaw
local STATE_GPS2_ONLY = 3   -- GPS1 掉线 (status==0), GPS2 RTK 在线且航向有效
local STATE_RTK_WARMUP = 4  -- 未解锁, RTK 在线但航向未就绪: 宽限内等 GPS yaw

-- RTK 航向宽限计时: 模块在线但航向未就绪的起始时刻 (0=未在等)
local rtk_warmup_start_ms = 0
-- 本会话是否曾解出过 RTK 双天线航向 (用于区分冷启动等待 vs 天线故障立即降级)
local ever_had_rtk_yaw = false

local SEV_INFO = 6
local SEV_WARN = 4

local current_state   = STATE_UNKNOWN
local pending_state   = STATE_UNKNOWN
local pending_count   = 0
local last_warn_ms    = 0
local WARN_REPEAT_MS  = 10000  -- 已解锁有告警间隔再隔 10 秒

-- 解锁授权: GPS1 故障但 RTK 完好时禁止解锁 (需求三)
local arm_auth_id        = nil
local last_auth_warn_ms  = 0

-- 解锁后 RTK 故障防抖计数
local armed_rtk_fail_count     = 0
local ARMED_RTK_FAIL_THRESHOLD = 3   -- 连续 3 秒检测到故障才切换

-- ── GPS2 (UM982) 运行期再探测 ─────────────────────────────────────────────
-- v6.12: GPS1_ONLY 不再写 GPS2_TYPE=0, 故无需周期性关断/再探测.
-- 保留启动时 probe_gps2_on_startup() 把 flash 里残留的 0 恢复为 25.

local REPROBE_INTERVAL_MS = 15000   -- GPS1 热插再探测间隔

-- ── GPS1 (ublox) 运行期再探测 ─────────────────────────────────────────────
-- 对称问题: GPS2_ONLY 状态下 GPS1_TYPE 被写成 0, AP_GPS 不再创建 GPS1 驱动,
--           热插回 ublox 也无法被识别 (status(0) 恒为 0, 永远回不到 BOTH_OK).
-- 方案: 未解锁且处于 GPS2_ONLY 时, 周期性把 GPS1_TYPE 临时设回 1(Auto) 探测,
--       检测到 ublox → 切回 BOTH_OK; 窗口超时仍无数据 → 设回 0.
local reprobe1_active    = false
local reprobe1_start_ms  = 0
local last_reprobe1_ms   = 0
local REPROBE1_WINDOW_MS  = 30000   -- ublox 探测窗口 15 秒 (ublox 上电较快，但也给宽裕点)


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

-- 安全读取 GPS 双天线航向 (instance 超出 num_sensors 时返回 nil, 避免 Lua 报错)
local function safe_gps_yaw_deg(instance)
    local n = gps:num_sensors() or 0
    if instance >= n then
        return nil, nil, nil
    end
    return gps:gps_yaw_deg(instance)
end

-- 判断 UM982 (GPS2) 模块本身是否还在串口通信
-- v6.4: 关键 - 区分 "RTK 模块在线但尚未定位/航向" 与 "RTK 真的拔掉离线"。
--   只要模块串口还在出数据 (num_sensors>=2 且最近收到过消息), 就认为它在线,
--   此时即使还没定位/航向, 也绝不能把 GPS2_TYPE 写 0 销毁驱动 (会让 UM982
--   冷启动重来, 永远搜不到星). 只有真正离线才允许写 0 + reprobe 热插探测.
local MODULE_TIMEOUT_MS = 3000   -- 超过 3 秒没收到 GPS2 消息视为模块离线
local function rtk_module_online()
    if (gps:num_sensors() or 0) <= GPS2_INSTANCE then
        return false
    end
    local last = gps:last_message_time_ms(GPS2_INSTANCE)
    if last == nil then
        return false
    end
    local now = millis()
    return (now - last) < MODULE_TIMEOUT_MS
end

-- 读取当前应处于的状态
-- RTK 有 3D Fix 且双天线航向有效 → BOTH_OK / GPS2_ONLY
-- 未解锁 + 冷启动从未出过航向 + RTK 在线 → RTK_WARMUP (宽限 GPSYS_RTK_WAIT 秒)
-- 曾出过航向后丢失 / 宽限超时 / 模块离线 / 已解锁 RTK 故障 → GPS1_ONLY
local function read_state()
    local g1_status  = safe_gps_status(GPS1_INSTANCE)
    local g2_status  = safe_gps_status(GPS2_INSTANCE)
    local g1_present = (g1_status >= 1)               -- GPS1 硬件在线 (>=NO_FIX)
    local g2_ok      = (g2_status >= MIN_GPS_STATUS)  -- GPS2 状态足够

    -- 检查 RTK 双天线航向是否有效 (任一天线丢失则 nil)
    local rtk_yaw, _, _ = safe_gps_yaw_deg(GPS2_INSTANCE)
    local g2_yaw_ok = (rtk_yaw ~= nil)

    -- RTK 双天线航向已就绪 (fix 足够 + 航向有效)
    if g2_ok and g2_yaw_ok then
        ever_had_rtk_yaw = true
        rtk_warmup_start_ms = 0
        if g1_present then
            return STATE_BOTH_OK
        end
        return STATE_GPS2_ONLY         -- GPS1 掉线, 仅 RTK
    end

    -- 航向未就绪: 仅冷启动 (本会话从未出过 RTK 航向) 且未解锁 → 宽限内等 RTK
    if (not arming:is_armed()) and rtk_module_online() and (not ever_had_rtk_yaw) then
        if rtk_wait_ms() > 0 then
            if rtk_warmup_start_ms == 0 then
                rtk_warmup_start_ms = millis():tofloat()
            end
            if (millis():tofloat() - rtk_warmup_start_ms) < rtk_wait_ms() then
                return STATE_RTK_WARMUP
            end
        end
        -- 宽限超时仍无航向 → 落到下方磁罗盘分支
    else
        rtk_warmup_start_ms = 0
    end

    -- 天线丢失 / 宽限超时 / 模块离线 / 飞行中 RTK 故障 → GPS1 定位 + 磁罗盘
    if g1_present then
        return STATE_GPS1_ONLY
    end
    return STATE_UNKNOWN
end

local function apply_state(state)
    if state == STATE_RTK_WARMUP then
        -- 冷启动等 RTK 航向: 参数与 RTK 模式一致, 航向源固定 GPS, 罗盘由 manage_compass 保持关闭
        set_param_if_diff("GPS1_TYPE",       TYPE_AUTO)
        set_param_if_diff("GPS2_TYPE",       TYPE_UM982)
        set_param_if_diff("EK3_SRC1_YAW",    YAW_GPS)
        set_param_if_diff("GPS_PRIMARY",     PRIMARY_GPS2)
        set_param_if_diff("GPS_AUTO_SWITCH", AUTOSW_USE_PRIMARY)
    elseif state == STATE_BOTH_OK then
        -- 双 GPS 均在线: GPS2 提供位置 + 双天线 yaw.
        -- COMPASS_USE 由 manage_compass() 管理 (延迟 GPSYS_COMP_DLY 秒后启用).
        local a = set_param_if_diff("GPS1_TYPE",       TYPE_AUTO)
        local e = set_param_if_diff("GPS2_TYPE",       TYPE_UM982)
        local b = set_param_if_diff("EK3_SRC1_YAW",    YAW_GPS)
        local c = set_param_if_diff("GPS_PRIMARY",     PRIMARY_GPS2)
        -- 强制用 GPS_PRIMARY=GPS2, 不让 USE_BEST 因 ublox fix 等级更高而抢主
        local f = set_param_if_diff("GPS_AUTO_SWITCH", AUTOSW_USE_PRIMARY)
        if a or b or c or e or f then
            gcs:send_text(SEV_INFO, "GPS: dual RTK")
        end
    elseif state == STATE_GPS1_ONLY then
        -- RTK 故障: GPS1 定位 + 外置磁罗盘; GPS2_TYPE 始终保持 25 不关断
        set_param_if_diff("GPS1_TYPE",       TYPE_AUTO)
        set_param_if_diff("GPS2_TYPE",       TYPE_UM982)
        set_param_if_diff("EK3_SRC1_YAW",    YAW_COMPASS)
        set_param_if_diff("GPS_PRIMARY",     PRIMARY_GPS1)
        set_param_if_diff("GPS_AUTO_SWITCH", AUTOSW_USE_PRIMARY)
        -- 不发 GCS 告警: 无航向降级为 GPS1+磁罗盘属正常收敛过程
    elseif state == STATE_GPS2_ONLY then
        -- 注意写入顺序: 先设 GPS_PRIMARY=1 (GPS2), 再关 GPS1_TYPE=0,
        -- 避免 "GPS 1: primary but TYPE 0" 中间态告警
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

-- 罗盘延迟开启计时: 记录 "可以把 COMPASS_USE 改为 1" 的最早时刻 (0=未计时)
local compass_open_at_ms = 0

-- 每帧统一管理 COMPASS_USE / COMPASS_USE2 / COMPASS_USE3.
-- COMPASS_USE2/3 始终禁用.
-- COMPASS_USE 逻辑:
--   RTK_WARMUP : 关罗盘 (禁止 EKF 提前用磁罗盘初始化航向)
--   BOTH_OK / GPS2_ONLY : RTK 航向就绪, 延迟 GPSYS_COMP_DLY 秒后再开罗盘
--   GPS1_ONLY / 其他 : RTK 无航向, 立即 COMPASS_USE=1 (磁罗盘为航向源)
local function manage_compass()
    set_param_if_diff("COMPASS_USE2", COMPASS_OFF)
    set_param_if_diff("COMPASS_USE3", COMPASS_OFF)

    local rtk_ok = (current_state == STATE_BOTH_OK or current_state == STATE_GPS2_ONLY)

    if current_state == STATE_RTK_WARMUP then
        set_param_if_diff("COMPASS_USE", COMPASS_OFF)
        compass_open_at_ms = 0
    elseif rtk_ok then
        -- RTK 航向已就绪: 第一次进入时启动延迟计时
        if compass_open_at_ms == 0 then
            local dly = (gpsys_comp_dly:get() or 30)
            if dly < 0 then dly = 0 end
            compass_open_at_ms = millis():tofloat() + dly * 1000
        end
        if millis():tofloat() >= compass_open_at_ms then
            set_param_if_diff("COMPASS_USE", COMPASS_ON)
        else
            set_param_if_diff("COMPASS_USE", COMPASS_OFF)
        end
    else
        -- RTK 故障 / 未知: 立即开罗盘, 重置计时 (下次 RTK 就绪重新计)
        compass_open_at_ms = 0
        set_param_if_diff("COMPASS_USE", COMPASS_ON)
    end
end

-- GPS1 掉线且 RTK 定位+航向均正常 (需求三拦截条件)
local function gps1_fault_rtk_ok()
    local g1_status = safe_gps_status(GPS1_INSTANCE)
    local g2_status = safe_gps_status(GPS2_INSTANCE)
    local rtk_yaw, _, _ = safe_gps_yaw_deg(GPS2_INSTANCE)
    return (g1_status < 1)
        and (g2_status >= MIN_GPS_STATUS)
        and (rtk_yaw ~= nil)
end

-- prearm 真正拦截: GPS1 故障但 RTK 完好时不允许解锁
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
        manage_compass()
        return update, RUN_INTERVAL_MS
    end

    -- ── 未解锁 ────────────────────────────────────────────────────────────────

    -- 重置解锁后故障计数 (每次落地归零)
    armed_rtk_fail_count = 0

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
        manage_compass()
        return update, RUN_INTERVAL_MS
    end

    -- 进入 RTK_WARMUP 立即生效 (不等 5 秒防抖), 尽早禁磁罗盘等 RTK 航向
    if s == STATE_RTK_WARMUP and current_state ~= STATE_RTK_WARMUP then
        apply_state(STATE_RTK_WARMUP)
        current_state = STATE_RTK_WARMUP
        pending_state = STATE_UNKNOWN
        pending_count = 0
        manage_compass()
        return update, RUN_INTERVAL_MS
    end

    -- 宽限超时后退回 GPS1+磁罗盘, 立即切换 (已等够 GPSYS_RTK_WAIT)
    if s == STATE_GPS1_ONLY and current_state == STATE_RTK_WARMUP then
        apply_state(STATE_GPS1_ONLY)
        current_state = STATE_GPS1_ONLY
        pending_state = STATE_UNKNOWN
        pending_count = 0
        manage_compass()
        return update, RUN_INTERVAL_MS
    end

    -- 曾出过 RTK 航向后丢失 (天线掉/RTK 拔): 立即切 GPS1+磁罗盘, 不等防抖
    if s == STATE_GPS1_ONLY and ever_had_rtk_yaw and current_state ~= STATE_GPS1_ONLY then
        apply_state(STATE_GPS1_ONLY)
        current_state = STATE_GPS1_ONLY
        pending_state = STATE_UNKNOWN
        pending_count = 0
        manage_compass()
        return update, RUN_INTERVAL_MS
    end

    -- 当前已是目标状态: 清空 pending
    if s == current_state then
        pending_state = STATE_UNKNOWN
        pending_count = 0
        manage_compass()
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

    manage_compass()
    return update, RUN_INTERVAL_MS
end

if script_enabled() then
    probe_gps1_on_startup()
    probe_gps2_on_startup()
end
return update, STARTUP_DELAY_MS
