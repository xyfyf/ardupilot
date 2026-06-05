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
    4) 近地释放：相对 Home 低于 0.1 m 时强制恢复基准参数（脚本内写死，不可调）。否则 AltHold 接地后
       PILOT_SPEED_DN 仍为缓降值，着陆检测与自动上锁会延迟。

    版本：
      4.0 近地释放 0.1 m 写死（合并 2.0 逻辑）
      2.0 近地释放，修复 AltHold 接地后自动上锁延迟
      1.0 按相对 Home 高度切换缓降参数
--]]

local SCRIPT_VERSION = "4.0"
local MAV_SEVERITY = { INFO = 6, NOTICE = 5, WARNING = 4 }

-- 脚本专用参数表（避免与其它脚本冲突可调 KEY）
local PARAM_TABLE_KEY = 96
local PARAM_PREFIX = "LNDS_"

local function bind_add_param(name, idx, default_value)
    assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), "add_param " .. name)
    return Parameter(PARAM_PREFIX .. name)
end

assert(param:add_table(PARAM_TABLE_KEY, PARAM_PREFIX, 6), "add_table LNDS")

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
  @Description: 主循环周期，建议 200~500
  @Range: 50 2000
  @Units: ms
  @User: Advanced
--]]
local P_RATE_MS = bind_add_param("RATE_MS", 5, 250)

--[[
  @Param: LNDS_ONLY_ARMED
  @DisplayName: Only when armed
  @Description: 1=仅解锁后调整；未解锁时恢复基准，避免在地面长期保持「缓降」参数
  @Values: 0:Always adjust,1:Only when armed
  @User: Standard
--]]
local P_ONLY_ARMED = bind_add_param("ONLY_ARMED", 6, 1)

-- 近地释放高度 (m)，写死：低于此高度恢复基准参数，便于着陆检测与自动上锁
local RELEASE_ALT_M = 0.1

-- 内部：是否已从飞控读取过基准
local baseline_captured = false
-- 当前是否处于「缓降区」参数集
local in_slow_zone = false

-- 基准缓存：新参 m/s 与旧参 cm/s 二选一存在
local base = {
    use_new_land = false,  -- LAND_SPD_MS
    land_spd_ms = 0.0,     -- 新
    land_spd_cms = 0,      -- 旧
    use_new_land_high = false,
    land_spd_high_ms = 0.0,
    land_spd_high_cms = 0,
    use_new_wp = false,
    wp_dn_ms = 0.0,
    wp_dn_cms = 0,
    pilot_dn_cms = 0,      -- PILOT_SPEED_DN 始终为 cm/s
}

-- 将 m/s 转为旧参 cm/s
local function ms_to_cms(v)
    return math.floor(v * 100.0 + 0.5)
end

-- 从飞控读取并锁定基准（仅一次），失败则用合理默认
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

-- 把下降限制写成「缓降」
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
    -- 手控最大下降：cm/s
    param:set("PILOT_SPEED_DN", cms)
end

-- 恢复启动时缓存的基准
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

--[[
  相对 Home 的「高度」(米, 上为正)。

  优先使用 get_relative_position_NED_home()：由当前 get_location 与 Home 做 NED 差分，
  与任务/RTL 使用的「相对 Home 高度」一致；避免仅用 EKF origin 的 posD 与 (origin.alt-home.alt)
  组合时，在水平离开起飞点一段距离后垂直分量与全局位置不一致，导致门槛判断偏高、缓降不生效。

  NED 的 z 为「向下为正」，故相对 Home 的高度 = -z。
  若 NED_home 不可用（例如尚无有效位置），再回退到 get_relative_position_D_home。
--]]
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

function update()
    local next_ms = math.floor(P_RATE_MS:get() + 0.5)
    next_ms = math.max(50, math.min(2000, next_ms))

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

    -- 需要时可选择在地面不篡改参数
    if P_ONLY_ARMED:get() > 0.5 and (not arming:is_armed()) then
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

    -- 带滞回：从低速区切回全速，要高于 thr+h
    local should_slow = in_slow_zone
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

    -- 近地释放：低于 RELEASE_ALT_M(0.1m) 时恢复 PILOT_SPEED_DN 等基准
    if altm < RELEASE_ALT_M then
        should_slow = false
    end

    if should_slow and not in_slow_zone then
        apply_slow(slow)
        in_slow_zone = true
    elseif (not should_slow) and in_slow_zone then
        apply_baseline()
        in_slow_zone = false
    end

    return update, next_ms
end

gcs:send_text(MAV_SEVERITY.NOTICE, string.format("LNDS v%s loaded", SCRIPT_VERSION))
return update()
