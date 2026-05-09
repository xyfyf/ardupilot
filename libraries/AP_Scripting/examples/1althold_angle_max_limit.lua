--[[
    AltHold 下限制「最大水平飞行速度」+ 可选倾角上限（多旋翼 Copter）

    背景：
    - GPS 模式（如 Loiter）有 LOIT_SPEED 等参数，可直接约束水平地速。
    - AltHold 只有摇杆→倾角，没有水平速度环，固件本身不能单独「按 m/s 限死」。

    本脚本两种手段（可同时开）：
    1) 地速硬约束（推荐）：用 AHAM_SPD_MAX>0 启用。根据 ahrs 水平速度，超速时通过 RC override
       把横滚/俯仰杆量向该通道 TRIM 衰减，使地速无法长期维持超过设定值（顺风下仍会瞬时略超，随后被拉回）。
    2) 倾角上限（可选）：用 AHAM_USE_ANG=1 时，在 AltHold 内临时写小 ANGLE_MAX（厘度），与全机基准取 min；
       用于限制全杆机动能力，不能等价于地速上限。

    依赖：SCR_ENABLE=1；Copter 模式 AltHold=2。建议检查 RC1/RC2 与 RCMAP 一致。

    重要（RC 覆盖）：
    - 限速分支使用 rc:get_channel(1/2):set_override。未超速时脚本不发送 override，依赖 RC 通道超时恢复真机杆量。
    - 若恢复手感偏慢，请在地面站适当减小 RC_OVERRIDE_TIME（勿设为 0，否则可能禁用覆盖逻辑），典型 0.5～1 s。
    - 无 GPS/速度估计差时，地速来自 EKF，限速可能不准。

    参数写入：ANGLE_MAX 仍用 param:set（内存）；断电后 EEPROM 不变，除非你手动保存。
--]]

local MAV_SEVERITY = { INFO = 6, NOTICE = 5, WARNING = 4 }

local MODE_ALTHOLD = 2

local PARAM_ANGLE_MAX = "ANGLE_MAX"

local PARAM_TABLE_KEY = 97
local PARAM_PREFIX = "AHAM_"

local function bind_add_param(name, idx, default_value)
    assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), "add_param " .. name)
    return Parameter(PARAM_PREFIX .. name)
end

-- 7 个脚本参数：ENABLE, USE_ANG, ANG, SPD_MAX, SPD_GAIN, RATE_MS, ARMONLY
assert(param:add_table(PARAM_TABLE_KEY, PARAM_PREFIX, 7), "add_table AHAM")

--[[
  @Param: AHAM_ENABLE
  @DisplayName: Master enable
  @Description: 1=启用脚本（见 USE_ANG / SPD_MAX）；0=关闭并恢复 ANGLE_MAX 基准
  @Values: 0:Disabled,1:Enabled
  @User: Standard
--]]
local P_ENABLE = bind_add_param("ENABLE", 1, 0)

--[[
  @Param: AHAM_USE_ANG
  @DisplayName: Use ANGLE_MAX reduction
  @Description: 1=在 AltHold 内按 AHAM_ANG 临时降低 ANGLE_MAX；0=不改倾角参数，仅地速策略生效
  @Values: 0:No,1:Yes
  @User: Standard
--]]
local P_USE_ANG = bind_add_param("USE_ANG", 2, 1)

--[[
  @Param: AHAM_ANG
  @DisplayName: Max lean angle cap in AltHold (deg)
  @Description: USE_ANG=1 时，写入 ANGLE_MAX（与基准取 min，且夹在合理范围）。飞控内 ANGLE_MAX 多为厘度，脚本内部按度再换算。
  @Range: 10 80
  @Units: deg
  @User: Standard
--]]
local P_ANG = bind_add_param("ANG", 3, 25.0)

--[[
  @Param: AHAM_SPD_MAX
  @DisplayName: Max horizontal groundspeed in AltHold (m/s)
  @Description: >0 时启用水平地速限制（NED 水平速度模长）。0=不启用 RC 限速。
  @Range: 0 30
  @Units: m/s
  @User: Standard
--]]
local P_SPD_MAX = bind_add_param("SPD_MAX", 4, 0)

--[[
  @Param: AHAM_SPD_GAIN
  @DisplayName: Speed limit gain
  @Description: 超速 (m/s) 时每周期衰减影子杆量的强度：factor=min(1, gain*excess)。越大越“硬”，过大易抖。
  @Range: 0.05 2
  @User: Advanced
--]]
local P_SPD_GAIN = bind_add_param("SPD_GAIN", 5, 0.35)

--[[
  @Param: AHAM_RATE_MS
  @DisplayName: Update interval (ms)
  @Description: 主循环周期。地速限制建议 40～80ms；纯倾角可放宽到 250ms。
  @Range: 20 2000
  @Units: ms
  @User: Advanced
--]]
local P_RATE_MS = bind_add_param("RATE_MS", 6, 50)

--[[
  @Param: AHAM_ARMONLY
  @DisplayName: Only when armed
  @Description: 1=仅解锁后生效；未解锁时恢复 ANGLE 基准且不注入 RC override
  @Values: 0:Always when mode matches,1:Only when armed
  @User: Standard
--]]
local P_ARMONLY = bind_add_param("ARMONLY", 7, 1)

--- 基准 ANGLE_MAX（度），用于恢复
local baseline_deg = nil

--- 倾角限制是否已写入（需恢复）
local angle_limiting_active = false

--- 地速限制：影子杆量（PWM），在「未超速」时与接收链路同步
local shadow_roll_pwm = nil
local shadow_pitch_pwm = nil

--- 读取 RC 通道 trim（用于衰减终点，兼容非 1500 中位）
local function rc_trim_pwm(ch)
    local v = param:get("RC" .. tostring(ch) .. "_TRIM")
    if v == nil then
        return 1500
    end
    return v
end

local function capture_baseline_once()
    if baseline_deg ~= nil then
        return
    end
    local v = param:get(PARAM_ANGLE_MAX)
    if v ~= nil then
        baseline_deg = v / 100.0
        gcs:send_text(MAV_SEVERITY.INFO, string.format("AHAM: Base ANGLE_MAX=%.1f", baseline_deg))
    else
        baseline_deg = 30.0
        gcs:send_text(MAV_SEVERITY.WARNING, "AHAM: ANGLE_MAX read err, default 30")
    end
end

local function clamp_angle_deg(x)
    if x < 10.0 then
        return 10.0
    end
    if x > 80.0 then
        return 80.0
    end
    return x
end

local function desired_limited_deg()
    local want = clamp_angle_deg(P_ANG:get())
    return math.min(want, baseline_deg)
end

local function restore_angle_baseline()
    if baseline_deg == nil or not angle_limiting_active then
        return
    end
    param:set(PARAM_ANGLE_MAX, math.floor(baseline_deg * 100 + 0.5))
    angle_limiting_active = false
end

--- 水平地速 (m/s)，nil 若速度不可用
local function horizontal_ground_speed_ms()
    local vel = ahrs:get_velocity_NED()
    if vel == nil then
        return nil
    end
    local vx = vel:x()
    local vy = vel:y()
    return math.sqrt(vx * vx + vy * vy)
end

--- 地速 RC 限速：超速则把影子杆往 trim 拉；未超速则影子跟随当前输入（未覆盖时有效）
local function apply_speed_governor(vmax_ms, gain, trim_r, trim_p)
    local gspd = horizontal_ground_speed_ms()
    local rc_r = rc:get_pwm(1)
    local rc_p = rc:get_pwm(2)
    if rc_r == nil or rc_p == nil then
        return
    end

    -- 初始化影子
    if shadow_roll_pwm == nil then
        shadow_roll_pwm = rc_r
    end
    if shadow_pitch_pwm == nil then
        shadow_pitch_pwm = rc_p
    end

    if gspd == nil then
        shadow_roll_pwm = rc_r
        shadow_pitch_pwm = rc_p
        return
    end

    -- 未超速：同步影子（此时 rc 应为真接收或已超时恢复）
    if gspd <= vmax_ms then
        shadow_roll_pwm = rc_r
        shadow_pitch_pwm = rc_p
        return
    end

    -- 超速：沿影子向 trim 衰减
    local excess = gspd - vmax_ms
    local factor = math.min(1.0, gain * excess)
    shadow_roll_pwm = trim_r + (1.0 - factor) * (shadow_roll_pwm - trim_r)
    shadow_pitch_pwm = trim_p + (1.0 - factor) * (shadow_pitch_pwm - trim_p)

    local ch1 = rc:get_channel(1)
    local ch2 = rc:get_channel(2)
    if ch1 ~= nil then
        ch1:set_override(math.floor(shadow_roll_pwm + 0.5))
    end
    if ch2 ~= nil then
        ch2:set_override(math.floor(shadow_pitch_pwm + 0.5))
    end
end

function update()
    if P_ENABLE:get() ~= 1 then
        restore_angle_baseline()
        shadow_roll_pwm = nil
        shadow_pitch_pwm = nil
        return update, math.floor(P_RATE_MS:get() + 0.5)
    end

    capture_baseline_once()

    local in_althold = (vehicle:get_mode() == MODE_ALTHOLD)
    local armed_ok = (P_ARMONLY:get() ~= 1) or arming:is_armed()
    local active_scene = in_althold and armed_ok

    local trim_r = rc_trim_pwm(1)
    local trim_p = rc_trim_pwm(2)

    if not active_scene then
        restore_angle_baseline()
        shadow_roll_pwm = nil
        shadow_pitch_pwm = nil
        return update, math.floor(P_RATE_MS:get() + 0.5)
    end

    -- 可选：倾角上限（写 ANGLE_MAX 厘度）
    if P_USE_ANG:get() == 1 then
        local ang = desired_limited_deg()
        param:set(PARAM_ANGLE_MAX, math.floor(ang * 100 + 0.5))
        angle_limiting_active = true
    else
        restore_angle_baseline()
    end

    -- 可选：水平地速上限（RC override）
    local vmax = P_SPD_MAX:get()
    if vmax > 0 then
        apply_speed_governor(vmax, P_SPD_GAIN:get(), trim_r, trim_p)
    else
        shadow_roll_pwm = nil
        shadow_pitch_pwm = nil
    end

    return update, math.floor(P_RATE_MS:get() + 0.5)
end

return update, 500
