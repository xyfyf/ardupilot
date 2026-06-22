--[[
    按「相对 Home 高度」动态限制下降相关参数（多旋翼 Copter）

    思路：
    - 相对起飞点（Home）高度 < 门槛：将末段降落速度、航点最大下降速度、手控最大下降速度
      统一限制为「缓降」速度（默认 0.5 m/s），避免近地仍按较大参数下沉。
    - 高于门槛：恢复脚本启动时记录的「基准」参数值（不在 EEPROM 中反复写入）。

    依赖：SCR_ENABLE=1；需开机运行或在地面站加载。

    注意：
    1) 高度优先用「相对 Home 的 NED 位置」的垂直分量（与航线/RTL 一致），与仅用 EKF D+原点高差相比，
       在水平远离起飞点后仍与当前位置估计一致；不可用时回退到 get_relative_position_D_home。
       仍非测距真离地；地形起伏大时请谨慎。
    2) 使用 param:set（内存），关机前若需在地面站保留数值，请手动保存参数或重新写入基准。
    3) 新固件多为 m/s（LAND_SPD_MS、WP_SPD_DN）；旧固件为 cm/s（LAND_SPEED、WPNAV_SPEED_DN 等），
       脚本会按是否存在参数名自动选用。
    4) 近地锁存：相对 Home 低于 LNDS_REL_M（默认 0.25 m）时锁存并恢复基准参数；锁存后不再因高度读数弹回而重进缓降区，
       避免着陆检测计时（LDET.Count）被反复清零、自动上锁延迟。

    LDET 对照分析（测试飞行设 LNDS_DBG=1）：
    - 固件写 LDET：Flags（各检测条件位）、Count（land_detector_count，满足条件时递增，失败则归零）
    - 脚本写 LNDX：Alt/Slw/Ltc/Fly/Spl/Pdn/ZChg/TLms，与 LDET 同一时间轴对照
    - Mission Planner → 数据闪存：同图绘制 LDET.Count 与 LNDX.Slw、LNDX.Ltc
    - 若 Count 接近 loop_rate 后归零，且 LNDX.Slw 由 0 变 1，多为高度弹回重进缓降区（5.0 锁存应阻止）

    版本：
      5.0 近地锁存 LNDS_REL_M + LNDX 日志对照 LDET；修复高度弹回重进缓降区
      4.0 近地释放 0.1 m 写死（合并 2.0 逻辑）
      2.0 近地释放，修复 AltHold 接地后自动上锁延迟
      1.0 按相对 Home 高度切换缓降参数
--]]

local SCRIPT_VERSION = "5.0"
local MAV_SEVERITY = { INFO = 6, NOTICE = 5, WARNING = 4 }

-- 脚本专用参数表（避免与其它脚本冲突可调 KEY）
local PARAM_TABLE_KEY = 96
local PARAM_PREFIX = "LNDS_"

local function bind_add_param(name, idx, default_value)
    assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), "add_param " .. name)
    return Parameter(PARAM_PREFIX .. name)
end

assert(param:add_table(PARAM_TABLE_KEY, PARAM_PREFIX, 8), "add_table LNDS")

--[[
  @Param: LNDS_ENABLE
  @DisplayName: Land speed by rel-alt enable
  @Description: 1=按相对 Home 高度切换下降参数；0=不运行逻辑并在下一周期尝试恢复基准
  @Values: 0:Disabled,1:Enabled
  @User: Standard
--]]
local P_ENABLE = bind_add_param("ENABLE", 1, 1)

--[[
  @Param: LNDS_ALT_M
  @DisplayName: Height threshold above home (m)
  @Description: 相对 Home 高度低于此值（米）时启用缓降速度
  @Range: 0.5 50
  @Units: m
  @User: Standard
--]]
local P_ALT_M = bind_add_param("ALT_M", 2, 2.0)

--[[
  @Param: LNDS_SLOW_MS
  @DisplayName: Slow descent limit (m/s)
  @Description: 低于 LNDS_ALT_M 时写入的下降速度上限（米/秒）
  @Range: 0.2 2
  @Units: m/s
  @User: Standard
--]]
local P_SLOW_MS = bind_add_param("SLOW_MS", 3, 0.5)

--[[
  @Param: LNDS_HYST_M
  @DisplayName: Hysteresis (m)
  @Description: 防止在门槛附近反复切换；从缓降区回到正常区需高于 (LNDS_ALT_M + 本值)
  @Range: 0 5
  @Units: m
  @User: Advanced
--]]
local P_HYST_M = bind_add_param("HYST_M", 4, 0.2)

--[[
  @Param: LNDS_RATE_MS
  @DisplayName: Update interval (ms)
  @Description: 主循环周期，默认 100 ms；建议 100~250
  @Range: 50 2000
  @Units: ms
  @User: Advanced
--]]
local P_RATE_MS = bind_add_param("RATE_MS", 5, 100)

--[[
  @Param: LNDS_ONLY_ARMED
  @DisplayName: Only when armed
  @Description: 1=仅解锁后调整；未解锁时恢复基准，避免在地面长期保持「缓降」参数
  @Values: 0:Always adjust,1:Only when armed
  @User: Standard
--]]
local P_ONLY_ARMED = bind_add_param("ONLY_ARMED", 6, 1)

--[[
  @Param: LNDS_DBG
  @DisplayName: LDET debug log
  @Description: 0=关闭；1=近地阶段写 LNDX 数据闪存日志（与 LDET 对照）；2=另发 GCS 文本（锁存/缓降切换）
  @Values: 0:Off,1:LNDX log,2:LNDX+GCS
  @User: Advanced
--]]
local P_DBG = bind_add_param("DBG", 7, 0)

--[[
  @Param: LNDS_REL_M
  @DisplayName: Near-ground latch height (m)
  @Description: 相对 Home 低于此高度（米）时锁存并恢复基准参数；锁存后本架次不再重进缓降区。须小于 LNDS_ALT_M
  @Range: 0.05 5
  @Units: m
  @User: Standard
--]]
local P_REL_M = bind_add_param("REL_M", 8, 0.25)

-- LNDX 日志：近地监控窗口 = alt < (LNDS_ALT_M + 此余量)
local LOG_ALT_MARGIN_M = 0.5
local LOG_INTERVAL_MS = 200

-- 内部：是否已从飞控读取过基准
local baseline_captured = false
-- 当前是否处于「缓降区」参数集
local in_slow_zone = false
-- 近地已锁存：不再因高度读数弹回而重进缓降区
local near_ground_latched = false
local latch_begin_ms = 0
-- 本次解锁以来缓降区切换次数（LNDX.ZChg）
local zone_change_count = 0
local last_log_ms = 0
local was_armed = false

-- 基准缓存：新参 m/s 与旧参 cm/s 二选一存在
local base = {
    use_new_land = false,
    land_spd_ms = 0.0,
    land_spd_cms = 0,
    use_new_land_high = false,
    land_spd_high_ms = 0.0,
    land_spd_high_cms = 0,
    use_new_wp = false,
    wp_dn_ms = 0.0,
    wp_dn_cms = 0,
    pilot_dn_cms = 0,
}

local function ms_to_cms(v)
    return math.floor(v * 100.0 + 0.5)
end

local function reset_flight_state()
    near_ground_latched = false
    latch_begin_ms = 0
    in_slow_zone = false
    zone_change_count = 0
    last_log_ms = 0
end

local function capture_baseline()
    if baseline_captured then
        return
    end

    if param:get("LAND_SPD_MS") ~= nil then
        base.use_new_land = true
        base.land_spd_ms = param:get("LAND_SPD_MS")
    elseif param:get("LAND_SPEED") ~= nil then
        base.use_new_land = false
        base.land_spd_cms = param:get("LAND_SPEED")
    else
        base.use_new_land = true
        base.land_spd_ms = 0.5
    end

    if param:get("LAND_SPD_HIGH_MS") ~= nil then
        base.use_new_land_high = true
        base.land_spd_high_ms = param:get("LAND_SPD_HIGH_MS")
    elseif param:get("LAND_SPEED_HIGH") ~= nil then
        base.use_new_land_high = false
        base.land_spd_high_cms = param:get("LAND_SPEED_HIGH")
    else
        base.use_new_land_high = true
        base.land_spd_high_ms = 0.0
    end

    if param:get("WP_SPD_DN") ~= nil then
        base.use_new_wp = true
        base.wp_dn_ms = param:get("WP_SPD_DN")
    elseif param:get("WPNAV_SPEED_DN") ~= nil then
        base.use_new_wp = false
        base.wp_dn_cms = param:get("WPNAV_SPEED_DN")
    else
        base.use_new_wp = true
        base.wp_dn_ms = 1.0
    end

    if param:get("PILOT_SPEED_DN") ~= nil then
        base.pilot_dn_cms = param:get("PILOT_SPEED_DN")
    else
        base.pilot_dn_cms = 0
    end

    baseline_captured = true
    gcs:send_text(MAV_SEVERITY.INFO, "LNDS: Base params saved")
end

local function apply_slow(slow_ms)
    local cms = ms_to_cms(slow_ms)
    if base.use_new_land then
        param:set("LAND_SPD_MS", slow_ms)
        param:set("LAND_SPD_HIGH_MS", slow_ms)
    else
        param:set("LAND_SPEED", cms)
        param:set("LAND_SPEED_HIGH", cms)
    end
    if base.use_new_wp then
        param:set("WP_SPD_DN", slow_ms)
    else
        param:set("WPNAV_SPEED_DN", cms)
    end
    param:set("PILOT_SPEED_DN", cms)
end

local function apply_baseline()
    if not baseline_captured then
        return
    end
    if base.use_new_land then
        param:set("LAND_SPD_MS", base.land_spd_ms)
    else
        param:set("LAND_SPEED", base.land_spd_cms)
    end
    if base.use_new_land_high then
        param:set("LAND_SPD_HIGH_MS", base.land_spd_high_ms)
    else
        param:set("LAND_SPEED_HIGH", base.land_spd_high_cms)
    end
    if base.use_new_wp then
        param:set("WP_SPD_DN", base.wp_dn_ms)
    else
        param:set("WPNAV_SPEED_DN", base.wp_dn_cms)
    end
    param:set("PILOT_SPEED_DN", base.pilot_dn_cms)
end

-- 近地锁存高度：限制在 0.05 m ~ (LNDS_ALT_M - 0.05 m)
local function get_release_alt_m(thr_m)
    local rel = P_REL_M:get()
    rel = math.max(0.05, rel)
    local max_rel = math.max(0.05, thr_m - 0.05)
    return math.min(rel, max_rel)
end

local function rel_alt_m_above_home()
    local ned = ahrs:get_relative_position_NED_home()
    if ned ~= nil then
        return -ned:z()
    end
    local pos_d = ahrs:get_relative_position_D_home()
    if pos_d == nil then
        return nil
    end
    return -pos_d
end

--[[
  写 LNDX 到数据闪存，与固件 LDET 同一时间轴对照。
  Slw/Ltc 与 LDET.Count 归零事件对齐，可判断是否为缓降参数导致检测计时被重置。
--]]
local function maybe_log_ldet_debug(altm, dbg_level)
    if dbg_level < 1 then
        return
    end

    local thr = P_ALT_M:get()
    if not arming:is_armed() then
        return
    end
    if not (in_slow_zone or near_ground_latched or altm < (thr + LOG_ALT_MARGIN_M)) then
        return
    end

    local now = millis()
    if (now - last_log_ms) < LOG_INTERVAL_MS then
        return
    end
    last_log_ms = now

    local likely_flying = vehicle:get_likely_flying()
    local spool = motors:get_spool_state()
    local pdn = param:get("PILOT_SPEED_DN") or 0
    local tlatch_ms = 0
    if near_ground_latched and latch_begin_ms > 0 then
        tlatch_ms = now - latch_begin_ms
    end

    -- Alt,Slw,Ltc,Fly,Spl,Pdn,ZChg,TLms
    logger:write(
        "LNDX",
        "Alt,Slw,Ltc,Fly,Spl,Pdn,ZChg,TLms",
        "fBBBBHHI",
        "m-------",
        "--------",
        altm,
        in_slow_zone and 1 or 0,
        near_ground_latched and 1 or 0,
        likely_flying and 1 or 0,
        spool,
        pdn,
        zone_change_count,
        tlatch_ms
    )

    if dbg_level >= 2 and near_ground_latched and tlatch_ms > 0 and (tlatch_ms % 1000) < LOG_INTERVAL_MS then
        gcs:send_text(
            MAV_SEVERITY.INFO,
            string.format("LNDX latch %.2fs alt=%.2f Slw=%d Fly=%d Spl=%d",
                tlatch_ms * 0.001, altm, in_slow_zone and 1 or 0, likely_flying and 1 or 0, spool)
        )
    end
end

local function set_slow_zone(want_slow, slow_ms, dbg_level)
    if want_slow and not in_slow_zone then
        apply_slow(slow_ms)
        in_slow_zone = true
        zone_change_count = zone_change_count + 1
        if dbg_level >= 2 then
            gcs:send_text(MAV_SEVERITY.NOTICE, string.format("LNDS: enter slow zone (#%d)", zone_change_count))
        end
    elseif (not want_slow) and in_slow_zone then
        apply_baseline()
        in_slow_zone = false
        zone_change_count = zone_change_count + 1
        if dbg_level >= 2 then
            gcs:send_text(MAV_SEVERITY.NOTICE, string.format("LNDS: leave slow zone (#%d)", zone_change_count))
        end
    end
end

function update()
    local next_ms = math.floor(P_RATE_MS:get() + 0.5)
    next_ms = math.max(50, math.min(2000, next_ms))
    local dbg_level = math.floor(P_DBG:get() + 0.5)

    local armed = arming:is_armed()
    if was_armed and not armed then
        if in_slow_zone and baseline_captured then
            apply_baseline()
        end
        reset_flight_state()
    end
    was_armed = armed

    if P_ENABLE:get() < 0.5 then
        if in_slow_zone and baseline_captured then
            apply_baseline()
            in_slow_zone = false
        end
        return update, next_ms
    end

    if not ahrs:home_is_set() then
        return update, next_ms
    end

    if P_ONLY_ARMED:get() > 0.5 and not armed then
        if in_slow_zone and baseline_captured then
            apply_baseline()
            in_slow_zone = false
        end
        return update, next_ms
    end

    capture_baseline()

    local altm = rel_alt_m_above_home()
    if altm == nil then
        return update, next_ms
    end

    local thr = P_ALT_M:get()
    local h = P_HYST_M:get()
    local slow = P_SLOW_MS:get()
    if slow < 0.1 then
        slow = 0.3
    end

    local release_m = get_release_alt_m(thr)

    -- 近地锁存：一旦低于 LNDS_REL_M，本架次不再重进缓降区
    if (not near_ground_latched) and altm < release_m then
        near_ground_latched = true
        latch_begin_ms = millis()
        if dbg_level >= 2 then
            gcs:send_text(MAV_SEVERITY.NOTICE,
                string.format("LNDS: latch at %.2fm (REL_M=%.2f)", altm, release_m))
        end
    end

    local should_slow = false
    if not near_ground_latched then
        should_slow = in_slow_zone
        if not in_slow_zone then
            if altm < thr then
                should_slow = true
            end
        else
            if altm > (thr + h) then
                should_slow = false
            else
                should_slow = true
            end
        end
    end

    set_slow_zone(should_slow, slow, dbg_level)
    maybe_log_ldet_debug(altm, dbg_level)

    return update, next_ms
end

gcs:send_text(MAV_SEVERITY.NOTICE, string.format("LNDS v%s loaded", SCRIPT_VERSION))
return update()
