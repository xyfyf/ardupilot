--[[
    按「相对 Home 高度」动态限制下降相关参数（多旋翼 Copter，全模式）

    思路：
    - 相对 Home 低于 LNDS_ALT_M（默认 4 m）：将 LAND_* / WP*下降 / PILOT_SPEED_DN
      统一限制为缓降（默认 0.5 m/s = 50 cm/s），覆盖 Loiter、AltHold、Auto、RTL、Land 等。
    - 高于门槛（含迟滞）：恢复解锁时记录的基准（如 200 cm/s）。
    - 低于 LNDS_REL_M（默认 0.15 m）：近地锁存，恢复基准且本架次不再重进缓降区，避免 LDET 计时被反复清零、上锁延迟。

    依赖：SCR_ENABLE=1。

    注意：
    1) 高度为相对 Home，非测距真离地；地形起伏大时请谨慎。
    2) param:set 仅改内存，不写入 EEPROM。
    3) 6.0 在缓降/锁存期间每周期重刷参数，避免部分模式下仅边沿写入不生效。
    4) 解锁记录基准后会校验：PILOT_SPEED_DN / LAND_SPEED / WPNAV_SPEED_DN 须大于 LNDS_SLOW_MS，
       否则 GCS 告警且缓降无效（例如基准 50 与缓降 50 相同）。

    版本：
      6.3 修复地面解锁(相对高≈0<REL_M)被立刻近地锁存导致整架次跳过缓降：
          新增「本架次须先爬升超过 LNDS_ALT_M 才允许近地锁存」门控
      6.2 校验改为「只要有一项基准>缓降目标就启用」，单项相等（如 LAND_SPEED=50=缓降50）不再误关整套；
          修复 DBG=2 时 LNDX latch 文本因 uint32_t 传入 %.2f 触发 string.format 报错
      6.1 解锁时校验基准下降速度 > 缓降目标，配置错误 GCS 告警
      6.0 全模式周期刷新下降限速；默认 4 m 缓降 / 0.15 m 锁存恢复基准
      5.0 近地锁存 LNDS_REL_M + LNDX 日志对照 LDET
--]]

local SCRIPT_VERSION = "6.3"
local MAV_SEVERITY = { INFO = 6, NOTICE = 5, WARNING = 4 }

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
local P_ENABLE = bind_add_param("ENABLE", 1, 0)

--[[
  @Param: LNDS_ALT_M
  @DisplayName: Height threshold above home (m)
  @Description: 相对 Home 高度低于此值（米）时启用缓降速度
  @Range: 0.5 50
  @Units: m
  @User: Standard
--]]
local P_ALT_M = bind_add_param("ALT_M", 2, 4.0)

--[[
  @Param: LNDS_SLOW_MS
  @DisplayName: Slow descent limit (m/s)
  @Description: 低于 LNDS_ALT_M 时写入的下降速度上限（米/秒）；0.5=50cm/s
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
  @Description: 主循环周期，默认 100 ms；建议 50~250
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
  @Description: 0=关闭；1=近地阶段写 LNDX 数据闪存日志；2=另发 GCS 文本（锁存/缓降切换）
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
local P_REL_M = bind_add_param("REL_M", 8, 0.15)

local LOG_ALT_MARGIN_M = 0.5
local LOG_INTERVAL_MS = 200

local baseline_captured = false
local in_slow_zone = false
local near_ground_latched = false
-- 本架次是否曾爬升超过 LNDS_ALT_M 阈值；未起飞前禁止近地锁存，避免地面解锁被误判
local has_climbed_above_thr = false
local latch_begin_ms = 0
local zone_change_count = 0
local last_log_ms = 0
local was_armed = false
-- 本架次是否已因「基准<=缓降」发过告警
local baseline_margin_warned = false
-- 基准是否足以产生可见缓降（解锁校验通过）
local baseline_ok_for_slow = true

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
    has_climbed_above_thr = false
    latch_begin_ms = 0
    in_slow_zone = false
    zone_change_count = 0
    last_log_ms = 0
    baseline_captured = false
    baseline_margin_warned = false
    baseline_ok_for_slow = true
end

-- 返回本架次基准的三项下降上限（cm/s）；PILOT_SPEED_DN=0 时与固件一致回退 PILOT_SPEED_UP
local function baseline_descent_cms()
    local pilot_cms = base.pilot_dn_cms
    if pilot_cms == 0 then
        local up = param:get("PILOT_SPEED_UP")
        if up ~= nil then
            pilot_cms = math.floor(math.abs(up) + 0.5)
        end
    end

    local land_cms = 0
    if base.use_new_land then
        land_cms = ms_to_cms(base.land_spd_ms)
    else
        land_cms = math.floor(math.abs(base.land_spd_cms) + 0.5)
    end

    local wp_cms = 0
    if base.use_new_wp then
        wp_cms = ms_to_cms(base.wp_dn_ms)
    else
        wp_cms = math.floor(math.abs(base.wp_dn_cms) + 0.5)
    end

    return pilot_cms, land_cms, wp_cms
end

--[[
  缓降要有体感，基准（解锁时 EEPROM/内存中的值）必须大于 LNDS_SLOW_MS。
  例：PILOT_SPEED_DN=200、LNDS_SLOW_MS=0.5(50cm/s) 才有效；两者皆为 50 则脚本写入无变化。
  @return boolean 是否通过
--]]
local function validate_baseline_vs_slow(slow_ms)
    local slow_cms = ms_to_cms(slow_ms)
    local pilot_cms, land_cms, wp_cms = baseline_descent_cms()

    -- 只要有一项基准 > 缓降目标，缓降就有可见效果（如 PILOT_SPEED_DN=200），即启用。
    -- 仅当三项全部 <= 缓降目标、写入毫无变化时才禁用，避免因单项（如 LAND_SPEED=50）误关整套。
    local any_effective = (pilot_cms > slow_cms) or (land_cms > slow_cms) or (wp_cms > slow_cms)

    if any_effective then
        return true
    end

    gcs:send_text(
        MAV_SEVERITY.WARNING,
        string.format(
            "LNDS: all base<=%dcm/s slow off; raise PILOT_SPEED_DN", slow_cms
        )
    )
    return false
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

    local slow_ms = P_SLOW_MS:get()
    if slow_ms < 0.1 then
        slow_ms = 0.3
    end
    baseline_ok_for_slow = validate_baseline_vs_slow(slow_ms)

    local slow_cms = ms_to_cms(slow_ms)
    if not baseline_ok_for_slow then
        baseline_margin_warned = true
        gcs:send_text(
            MAV_SEVERITY.WARNING,
            string.format("LNDS: slow zone disabled until base>%dcm/s", slow_cms)
        )
    end
end

-- 写入缓降集：覆盖 LAND / 航线下降 / 手控下降，全模式生效
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

local function dbg_level_from_param()
    return math.max(0, math.min(2, math.floor(P_DBG:get() + 0.5)))
end

local function maybe_log_ldet_debug(altm, dbg_level)
    if dbg_level < 1 then
        return
    end
    if not arming:is_armed() then
        return
    end

    local thr = P_ALT_M:get()
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
        -- tlatch_ms 为 uint32_t，必须先转 float 再喂给 %.2f，否则 string.format 报 number expected
        local tlatch_s = tlatch_ms:tofloat() * 0.001
        gcs:send_text(
            MAV_SEVERITY.INFO,
            string.format("LNDX latch %.2fs alt=%.2f Slw=%d Fly=%d Spl=%d",
                tlatch_s, altm, in_slow_zone and 1 or 0, likely_flying and 1 or 0, spool)
        )
    end
end

--[[
  6.0：缓降/锁存期间每周期重刷参数（不仅边沿），确保 Loiter 等模式持续读到 PILOT_SPEED_DN 等。
--]]
local function maintain_speed_limits(want_slow, slow_ms, dbg_level)
    if near_ground_latched then
        apply_baseline()
        if in_slow_zone then
            in_slow_zone = false
            zone_change_count = zone_change_count + 1
            if dbg_level >= 2 then
                gcs:send_text(MAV_SEVERITY.NOTICE,
                    string.format("LNDS: latch release baseline (#%d)", zone_change_count))
            end
        end
        return
    end

    if want_slow then
        if not baseline_ok_for_slow then
            apply_baseline()
            if in_slow_zone then
                in_slow_zone = false
            end
            if not baseline_margin_warned then
                baseline_margin_warned = true
                gcs:send_text(MAV_SEVERITY.WARNING,
                    "LNDS: skip slow zone (baseline<=slow)")
            end
            return
        end
        apply_slow(slow_ms)
        if not in_slow_zone then
            in_slow_zone = true
            zone_change_count = zone_change_count + 1
            if dbg_level >= 2 then
                gcs:send_text(MAV_SEVERITY.NOTICE,
                    string.format("LNDS: enter slow zone (#%d)", zone_change_count))
            end
        end
    else
        apply_baseline()
        if in_slow_zone then
            in_slow_zone = false
            zone_change_count = zone_change_count + 1
            if dbg_level >= 2 then
                gcs:send_text(MAV_SEVERITY.NOTICE,
                    string.format("LNDS: leave slow zone (#%d)", zone_change_count))
            end
        end
    end
end

function update()
    local next_ms = math.floor(P_RATE_MS:get() + 0.5)
    next_ms = math.max(50, math.min(2000, next_ms))
    local dbg_level = dbg_level_from_param()

    local armed = arming:is_armed()
    if was_armed and not armed then
        if baseline_captured then
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

    -- 本架次必须先爬升超过阈值，近地锁存才有意义；否则地面解锁(alt≈0)会被立刻误锁，全程跳过缓降
    if (not has_climbed_above_thr) and altm > thr then
        has_climbed_above_thr = true
    end

    if has_climbed_above_thr and (not near_ground_latched) and altm < release_m then
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
        elseif altm > (thr + h) then
            should_slow = false
        else
            should_slow = true
        end
    end

    maintain_speed_limits(should_slow, slow, dbg_level)
    maybe_log_ldet_debug(altm, dbg_level)

    return update, next_ms
end

return update()
