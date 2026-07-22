-- 1stick_inner_arm_outer_disarm5.1.lua
-- 美国手 Mode 2，摇杆通道：1横滚 2俯仰 3油门 4偏航
--
-- 关于遥控器 PWM（例如左/下=1000，右/上=2000）：
--   脚本用的是飞控校准后的「归一化值」，不是直接用微秒。
--   · 横滚/俯仰/偏航：norm_input() 以 RCx_TRIM 为中心，一般中位 1500 → 0，
--     左/下（较小 PWM）→ 负，右/上（较大 PWM）→ 正（若某通道在飞控里反转则相反）。
--   · 油门：多轴上 RC3_TRIM 常在最低位，若仍用 norm_input()，怠速往往接近 0 而不是 -1，
--     会导致「油门低」永远判不到。因此油门改用 norm_input_ignore_trim()：按 RC3_MIN/MAX
--     线性映射，下(1000)→约 -1，上(2000)→约 +1，与物理行程一致。
--
-- 【解锁】内八或外八持续满 GESTURE_HOLD_MS → 宽限 ARM_GRACE_MS 内仍尝试 arm()
-- 【上锁】
--        · 空中 (likely_flying)：纯内八或纯外八持续 AIR_GESTURE_DISARM_MS → 强制上锁
--        · 地面且从未离地：纯内八或纯外八持续 GESTURE_HOLD_MS → 上锁
--        · 解锁后须先松杆离开内八/外八，再次进入才累计上锁（避免解锁姿势误触上锁）
--        · 起飞后已再次触地（落稳）：内八/外八不上锁，避免着陆误触；仍可用收油门或组合键
--        · 油门最低 + 三轴中立：连续 THROTTLE_DISARM_HOLD_MS（未在飞时）
--        · 参数 SCR_USER1 位掩码：勾选 RC5～RC12 高电平组合，持续 COMBO_HOLD_MS 强制上锁
--
-- 安全：空中强制上锁需固件/参数允许空中 DISARM（见机型文档）；组合键仅在本脚本内解析。
-- 原生摇杆解锁：脚本启动时尝试 ARMING_RUDDER=0。
-- UOM 解锁鉴权：收到 GCS 下发的 UOM_ARM_STATUS (msg 519) 后判断是否允许解锁，
--   1107=允许；1101/1102/1103/1104/1106=禁止并在地面站显示原因。

local SCRIPT_NAME = "InOutArm5"

-- ========== UOM 解锁鉴权 ==========
local mavlink_msgs = require("MAVLink/mavlink_msgs")

local UOM_MAVMSG_ID       = 519
local UOM_FC_STATUS_ID    = 520
local UOM_OPERATOR_ID_MSG = 521
local UOM_MSG_MAP         = {
    [UOM_MAVMSG_ID]       = "UOM_ARM_STATUS",
    [UOM_FC_STATUS_ID]    = "UOM_FC_STATUS",
    [UOM_OPERATOR_ID_MSG] = "UOM_OPERATOR_ID",
}
-- 重复告警的最小间隔（毫秒），避免刷屏
local UOM_WARN_INTERVAL_MS = 5000
-- FC→GCS 上报间隔（毫秒）
local UOM_REPORT_MS        = 5000

-- 各状态码对应的地面站提示文字
local UOM_STATUS_MSG = {
    [1101] = "UOM: 无人机不存在于系统中，请联系管理员",
    [1102] = "UOM: 实名登记已注销，请到UOM官网进行实名登记",
    [1103] = "UOM: 未实名登记，请到UOM官网进行实名登记",
    [1104] = "UOM: 实名登记验证失败，请稍后重新请求获取状态",
    [1106] = "UOM: 设备未激活，请先激活",
    [1107] = "UOM: 设备已激活",
}

local uom_status_code  = 0      -- 当前状态码（持久化恢复 or 最后收到）
local uom_allow_arm    = false  -- 当前是否允许解锁
local uom_last_msg_ms  = nil    -- 最近一次收到 GCS 消息的时间戳（nil=本次启动后未收到）
local uom_auth_id      = nil    -- PreArm 鉴权 ID
local uom_last_warn_ms = 0      -- 上次向地面站发告警的时间戳
local uom_report_ms    = 0      -- 上次向地面站发 UOM_FC_STATUS 的时间戳
local uom_param        = nil    -- 持久化参数对象（UOM_STATUS）
local uom_param_enable = nil    -- 持久化参数对象（UOM_ENABLE）
local uom_enabled      = true   -- 功能开关（由参数 UOM_ENABLE 控制）

-- ---------- 参数持久化初始化 ----------
-- key=91, prefix="UOM_"
--   UOM_ENABLE : 1=启用 UOM 解锁鉴权（默认），0=禁用（任何状态码都不拦截解锁）
--   UOM_STATUS : 最后一次有效状态码，断电不丢失，开机即生效
local UOM_PARAM_KEY = 91
do
    local ok_t  = param:add_table(UOM_PARAM_KEY, "UOM_", 2)
    local ok_en = ok_t and param:add_param(UOM_PARAM_KEY, 1, "ENABLE", 1)
    local ok_st = ok_t and param:add_param(UOM_PARAM_KEY, 2, "STATUS", 0)
    if ok_en then
        uom_param_enable = Parameter("UOM_ENABLE")
        local v = uom_param_enable:get()
        uom_enabled = (v == nil) or (math.floor(v + 0.5) ~= 0)
    else
        gcs:send_text(4, SCRIPT_NAME .. ": UOM_ENABLE param init failed")
    end
    if ok_st then
        uom_param = Parameter("UOM_STATUS")
        local saved = uom_param:get()
        if saved ~= nil then
            uom_status_code = math.floor(saved + 0.5)
            uom_allow_arm   = (uom_status_code == 1107)
        end
    else
        gcs:send_text(4, SCRIPT_NAME .. ": UOM_STATUS param init failed, status not persistent")
    end
end

-- 初始化 MAVLink 接收：队列深度 5，注册 3 种消息 ID
mavlink:init(5, 3)
mavlink:register_rx_msgid(UOM_MAVMSG_ID)
mavlink:register_rx_msgid(UOM_OPERATOR_ID_MSG)

-- 向飞控申请一个独立的 PreArm 鉴权槽
uom_auth_id = arming:get_aux_auth_id()
if uom_auth_id == nil then
    gcs:send_text(3, SCRIPT_NAME .. ": UOM auth_id 获取失败，UOM 鉴权不可用")
end

-- 向 GCS 广播当前 UOM_FC_STATUS（channel 0 和 1）
local function uom_send_report(now)
    local age_s = 0xFFFF
    if uom_last_msg_ms ~= nil then
        local diff_ms = tonumber(now - uom_last_msg_ms) or 0
        age_s = math.min(0xFFFF, math.floor(diff_ms / 1000))
    end
    local msgid, payload = mavlink_msgs.encode("UOM_FC_STATUS", {
        status_code  = uom_status_code,
        status_age_s = age_s,
        allow_arm    = uom_allow_arm and 1 or 0,
        is_armed     = arming:is_armed() and 1 or 0,
    })
    mavlink:send_chan(0, msgid, payload)
    mavlink:send_chan(1, msgid, payload)
    uom_report_ms = now
end

-- 每次 update() 调用：接收 GCS→FC 消息、更新 PreArm 鉴权、定期上报 FC→GCS
local function uom_update(now)
    -- UOM_ENABLE=0 时直接放行，不拦截解锁
    if not uom_enabled then
        if uom_auth_id ~= nil then
            arming:set_aux_auth_passed(uom_auth_id)
        end
        return
    end

    local currently_armed = arming:is_armed()

    -- 排空接收队列，处理 UOM_ARM_STATUS (519) 和 UOM_OPERATOR_ID (521)
    local msg, _ = mavlink:receive_chan()
    while msg ~= nil do
        local parsed = mavlink_msgs.decode(msg, UOM_MSG_MAP)
        if parsed ~= nil then
            if parsed.msgid == UOM_MAVMSG_ID then
                -- ---- 519: 激活状态码 ----
                local new_code  = parsed.status_code
                local new_allow = (new_code == 1107)
                uom_last_msg_ms = now

                if currently_armed and not new_allow then
                    -- 飞行中收到"禁止解锁"状态 → 忽略，等降落后再生效
                    gcs:send_text(4, "UOM: armed, status " .. new_code .. " deferred until landing")
                else
                    -- 未解锁，或新状态仍为允许 → 正常更新
                    if new_code ~= uom_status_code then
                        uom_status_code = new_code
                        if uom_param ~= nil then
                            uom_param:set_and_save(uom_status_code)
                        end
                    end
                    uom_allow_arm = new_allow
                end

            elseif parsed.msgid == UOM_OPERATOR_ID_MSG then
                -- ---- 521: operate_id 持久化 + 写入 RID ----
                local op_id   = parsed.operator_id or ""
                local op_type = parsed.operator_id_type or 0
                -- 去掉尾部 null 字节（char[20] 可能有填充 \0）
                op_id = op_id:match("^([^%z]*)") or ""
                if #op_id > 0 then
                    if opendroneid ~= nil then
                        opendroneid:set_operator_id_from_script(op_id, op_type)
                        gcs:send_text(6, "UOM: operator_id set: " .. op_id)
                    else
                        gcs:send_text(4, "UOM: opendroneid not available, operator_id not set")
                    end
                end
            end
        end
        msg, _ = mavlink:receive_chan()
    end

    -- 定期向 GCS 上报 FC 侧状态（每 5s）
    if (now - uom_report_ms) >= UOM_REPORT_MS then
        uom_send_report(now)
    end

    if uom_auth_id == nil then return end

    -- 解锁判断：以当前 uom_allow_arm（来自持久化或最新 GCS 消息）为准，无超时逻辑
    if uom_allow_arm then
        arming:set_aux_auth_passed(uom_auth_id)
    else
        local fail_msg
        if uom_status_code == 0 then
            fail_msg = "UOM: 等待激活状态，请检查网络连接"
        else
            fail_msg = UOM_STATUS_MSG[uom_status_code]
                       or ("UOM: 未知状态 " .. tostring(uom_status_code) .. "，禁止解锁")
        end
        arming:set_aux_auth_failed(uom_auth_id, fail_msg)
        if (now - uom_last_warn_ms) >= UOM_WARN_INTERVAL_MS then
            gcs:send_text(3, fail_msg)
            uom_last_warn_ms = now
        end
    end
end
-- ========== END UOM 解锁鉴权 ==========

-- 俯仰通道是否在飞控/遥控器中相对「未反转」时符号相反。
-- false：内八/外八要求俯仰在负端满偏（经典：拉杆对应 norm 为负）。
-- true：俯仰已反向时，同一物理拉杆动作对应 norm 为正端，改判 pitch > STICK_THRESHOLD。
local PITCH_REVERSED_FOR_GESTURE = true

-- 内外八解锁、地面摇杆上锁共用保持时间（毫秒）
local GESTURE_HOLD_MS = 2000

-- 空中纯内八或纯外八强制上锁保持时间（毫秒）
local AIR_GESTURE_DISARM_MS = 5000

-- 摇杆归一化阈值（与官方 stick_gesture_arm 一致）
local STICK_THRESHOLD = 0.90

-- 内八/外八满 2s 后，在此宽限内仍尝试 arm()（毫秒）
local ARM_GRACE_MS = 250

-- 油门路径上锁时，横滚/俯仰/偏航允许的中立带
local STICK_NEUTRAL = 0.22

-- 油门路径上锁是否要求未在飞（true：仅地面/未离地）
local THROTTLE_DISARM_ONLY_WHEN_NOT_FLYING = true

-- 「油门最低 + 三轴中立」须连续满足此毫秒数才上锁
local THROTTLE_DISARM_HOLD_MS = 2000

-- 成功 arm/disarm 后的冷却（毫秒）
local COOLDOWN_MS = 800

-- ---------- 组合键（参数 SCR_USER1 位掩码，勾选 RC5～RC12 需为高）----------
-- 地面站设置 SCR_USER1：整数，bit0 对应 RC5 参与检测，bit1 对应 RC6 … bit7 对应 RC12。
-- 为 0 时关闭组合键上锁。PWM ≥ COMBO_PWM_HI_US 视为该路「按下/高」。
local SCR_COMBO_MASK_PARAM = "SCR_USER1"
local COMBO_PWM_HI_US = 1700
local COMBO_HOLD_MS = 400

-- ---------- 状态 ----------
local last_action_ms = 0

-- 未解锁：内八 / 外八累计解锁
local inner_arm_start_ms = nil
local outer_arm_start_ms = nil
local arm_grace_until_ms = 0

-- 已解锁：内八 / 外八累计上锁
local inner_disarm_start_ms = nil
local outer_disarm_start_ms = nil

-- 油门路径上锁累计
local throttle_disarm_hold_start_ms = nil

-- 组合键累计
local combo_disarm_hold_start_ms = nil

-- 是否曾经离地（用于「起飞后触地禁止内外八上锁」）
local has_ever_been_flying = false

-- 解锁后是否已松杆离开内八/外八（为 true 时才允许内外八累计上锁）
local stick_gesture_rearm_for_disarm = false
local was_armed = false

--- 读取 SCR_USER1 组合掩码（失败则 0）
local function read_combo_mask()
    local p = Parameter(SCR_COMBO_MASK_PARAM)
    if not p then
        return 0
    end
    local v = p:get()
    if v == nil then
        return 0
    end
    return math.floor(v + 0.5) % 256
end

--- 掩码要求的所有选中辅助通道是否均为高电平
local function combo_channels_high(mask)
    if mask == 0 then
        return false
    end
    for bit = 0, 7 do
        local bit_flag = (1 << bit) & 0xFF
        if (mask & bit_flag) ~= 0 then
            local chan = 5 + bit
            -- get_pwm 返回该路 PWM 微秒；未映射时可能为 nil
            local pwm = rc:get_pwm(chan)
            if pwm == nil or pwm < COMBO_PWM_HI_US then
                return false
            end
        end
    end
    return true
end

--- 读取四通道归一化输入（约 -1 ~ 1）
local function get_stick_input()
    local c1 = rc:get_channel(1)
    local c2 = rc:get_channel(2)
    local c3 = rc:get_channel(3)
    local c4 = rc:get_channel(4)
    if not c1 or not c2 or not c3 or not c4 then
        return nil, nil, nil, nil
    end
    return c1:norm_input(), c2:norm_input(), c3:norm_input_ignore_trim(), c4:norm_input()
end

local function cooldown_ok(now)
    return (now - last_action_ms) >= COOLDOWN_MS
end

local arming_rudder_param = Parameter("ARMING_RUDDER")
if arming_rudder_param then
    local rv = arming_rudder_param:get()
    if rv ~= nil and rv ~= 0 then
        arming_rudder_param:set_and_save(0)
        gcs:send_text(6, SCRIPT_NAME .. ": ARMING_RUDDER=0")
    end
end

function update()
    local roll, pitch, throttle, yaw = get_stick_input()
    if not roll then
        return update, 100
    end

    local now = millis()

    -- UOM 解锁鉴权：接收 GCS 下发的激活状态，更新 PreArm 结果
    uom_update(now)
    local is_throttle_low = throttle < -STICK_THRESHOLD
    -- 内八/外八的「两杆下角」需俯仰满偏（拉杆）一端；俯仰反转后 norm 符号对调，由 PITCH_REVERSED_FOR_GESTURE 选择正端或负端
    local is_pitch_for_cross = (PITCH_REVERSED_FOR_GESTURE and (pitch > STICK_THRESHOLD))
        or ((not PITCH_REVERSED_FOR_GESTURE) and (pitch < -STICK_THRESHOLD))


    -- 内八：油门低 + 偏航右 + 俯仰满偏（拉杆一侧）+ 横滚左
    local is_inner = is_throttle_low
        and (yaw > STICK_THRESHOLD)
        and is_pitch_for_cross
        and (roll < -STICK_THRESHOLD)

    -- 外八：油门低 + 偏航左 + 俯仰满偏（拉杆一侧）+ 横滚右
    local is_outer = is_throttle_low
        and (yaw < -STICK_THRESHOLD)
        and is_pitch_for_cross
        and (roll > STICK_THRESHOLD)

    local sticks_neutral = (math.abs(roll) < STICK_NEUTRAL)
        and (math.abs(pitch) < STICK_NEUTRAL)
        and (math.abs(yaw) < STICK_NEUTRAL)

    local armed = arming:is_armed()
    local likely_flying = vehicle:get_likely_flying()

    if armed and (not was_armed) then
        stick_gesture_rearm_for_disarm = false
    end
    was_armed = armed

    if likely_flying then
        has_ever_been_flying = true
    end

    -- 起飞后再次触地：禁止靠内八/外八上锁（防着陆误触）
    local block_stick_gesture_disarm = has_ever_been_flying and (not likely_flying)

    -- 空中与地面摇杆上锁时间不同
    local gesture_disarm_ms = likely_flying and AIR_GESTURE_DISARM_MS or GESTURE_HOLD_MS

    local combo_mask = read_combo_mask()

    -- ========== 宽限期补解锁（内/外八已满 2s 后松杆仍可解锁）==========
    if (not armed) and (now < arm_grace_until_ms) then
        if arming:arm() then
            gcs:send_text(6, SCRIPT_NAME .. ": Armed")
            last_action_ms = now
            stick_gesture_rearm_for_disarm = false
        end
    end

    if not armed then
        inner_disarm_start_ms = nil
        outer_disarm_start_ms = nil
        throttle_disarm_hold_start_ms = nil
        combo_disarm_hold_start_ms = nil

        -- 未解锁：内八或外八累计
        if is_inner and (not is_outer) and (now >= arm_grace_until_ms) then
            outer_arm_start_ms = nil
            if inner_arm_start_ms == nil then
                inner_arm_start_ms = now
            elseif (now - inner_arm_start_ms) >= GESTURE_HOLD_MS then
                arm_grace_until_ms = now + ARM_GRACE_MS
                inner_arm_start_ms = nil
            end
        elseif is_outer and (not is_inner) and (now >= arm_grace_until_ms) then
            inner_arm_start_ms = nil
            if outer_arm_start_ms == nil then
                outer_arm_start_ms = now
            elseif (now - outer_arm_start_ms) >= GESTURE_HOLD_MS then
                arm_grace_until_ms = now + ARM_GRACE_MS
                outer_arm_start_ms = nil
            end
        else
            if now >= arm_grace_until_ms then
                inner_arm_start_ms = nil
                outer_arm_start_ms = nil
            end
        end
    else
        inner_arm_start_ms = nil
        outer_arm_start_ms = nil
        arm_grace_until_ms = 0

        local disarm_ok = cooldown_ok(now)

        -- ---------- 内八 / 外八上锁（着陆后曾飞过则关闭；须先松杆再重新进入）----------
        if not block_stick_gesture_disarm and disarm_ok then
            if (not is_inner) and (not is_outer) then
                stick_gesture_rearm_for_disarm = true
            end

            if stick_gesture_rearm_for_disarm then
                if is_inner and (not is_outer) then
                    outer_disarm_start_ms = nil
                    if inner_disarm_start_ms == nil then
                        inner_disarm_start_ms = now
                    elseif (now - inner_disarm_start_ms) >= gesture_disarm_ms then
                        if arming:disarm() then
                            local tag = likely_flying and "Air" or "Gnd"
                            gcs:send_text(6, SCRIPT_NAME .. ": Disarmed (" .. tag .. ")")
                            last_action_ms = now
                        end
                        inner_disarm_start_ms = nil
                    end
                elseif is_outer and (not is_inner) then
                    inner_disarm_start_ms = nil
                    if outer_disarm_start_ms == nil then
                        outer_disarm_start_ms = now
                    elseif (now - outer_disarm_start_ms) >= gesture_disarm_ms then
                        if arming:disarm() then
                            local tag = likely_flying and "Air" or "Gnd"
                            gcs:send_text(6, SCRIPT_NAME .. ": Disarmed (" .. tag .. ")")
                            last_action_ms = now
                        end
                        outer_disarm_start_ms = nil
                    end
                else
                    inner_disarm_start_ms = nil
                    outer_disarm_start_ms = nil
                end
            else
                inner_disarm_start_ms = nil
                outer_disarm_start_ms = nil
            end
        else
            inner_disarm_start_ms = nil
            outer_disarm_start_ms = nil
        end

        -- ---------- 收油门 + 中立 ----------
        local ground_ok = (not THROTTLE_DISARM_ONLY_WHEN_NOT_FLYING) or (not likely_flying)
        local throttle_pose_ok = is_throttle_low and sticks_neutral and ground_ok

        if throttle_pose_ok and disarm_ok then
            if throttle_disarm_hold_start_ms == nil then
                throttle_disarm_hold_start_ms = now
            elseif (now - throttle_disarm_hold_start_ms) >= THROTTLE_DISARM_HOLD_MS then
                if arming:disarm() then
                    gcs:send_text(6, SCRIPT_NAME .. ": Idle disarmed")
                    last_action_ms = now
                end
                throttle_disarm_hold_start_ms = nil
            end
        else
            throttle_disarm_hold_start_ms = nil
        end

        -- ---------- 组合键 SCR_USER1 位掩码 ----------
        if combo_mask ~= 0 and combo_channels_high(combo_mask) and disarm_ok then
            if combo_disarm_hold_start_ms == nil then
                combo_disarm_hold_start_ms = now
            elseif (now - combo_disarm_hold_start_ms) >= COMBO_HOLD_MS then
                if arming:disarm() then
                    gcs:send_text(4, SCRIPT_NAME .. ": Combo disarmed")
                    last_action_ms = now
                end
                combo_disarm_hold_start_ms = nil
            end
        else
            combo_disarm_hold_start_ms = nil
        end
    end

    return update, 50
end

return update()

