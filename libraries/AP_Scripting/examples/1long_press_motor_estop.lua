-- 长按 RC 通道指定时间后触发电机紧急停桨（与 RCx_OPTION=MOTOR_ESTOP 高档位等效）
--
-- 重要参数设置（地面站）：
--   1) 将 RC9_OPTION 设为 0（Disabled）
--      若仍设为 31（Motor EStop），飞控会在拨杆置高时立即停桨，本脚本的 5 秒长按逻辑不会生效。
--   2) SCR_ENABLE = 1，SCR_HEAP_SIZE 足够，并将本脚本设为开机运行脚本（或按需加载）。
--
-- 行为说明：
--   - 通道 9 PWM 持续处于“高位”（默认 >1800us）满 HOLD_MS 毫秒后，调用 MOTOR_ESTOP 高档，电机停转。
--   - 拨杆回到“低位”（默认 <1400us）时，调用 MOTOR_ESTOP 低档，解除停桨（与固件内 RC 选项行为一致）。
--   - 中间位（1400~1800）不解除已触发的停桨，但会打断长按计时（需重新从高位累计满 5 秒）。

-- RC_Channel::AUX_FUNC::MOTOR_ESTOP，与参数列表中 “Motor EStop” 选项号一致
local AUX_FUNC_MOTOR_ESTOP = 31
-- AuxSwitchPos：与 docs.lua 中 rc:run_aux_function 的 ch_flag 一致，0=低，2=高
local AUX_SWITCH_LOW = 0
local AUX_SWITCH_HIGH = 2

-- 监听的 RC 通道号（ArduPilot Lua 中为 1 起始，与 RCx 一致）
local RC_CHAN = 11
-- 长按生效时间（毫秒）
local HOLD_MS = 5000
-- 判定为“高位”的 PWM 下限（三档开关高位典型约 1900us，可按遥控器实际微调）
local PWM_HIGH_US = 1800
-- 判定为“低位”的 PWM 上限
local PWM_LOW_US = 1400

-- 主循环周期（毫秒），50Hz 量级即可兼顾响应与 CPU
local LOOP_MS = 50

-- 进入高位后用于累计长按的起始时刻（毫秒）；nil 表示未在累计
local hold_start_ms = nil
-- 本次高位是否已经触发过停桨，避免重复调用 run_aux_function
local fired_this_hold = false

-- MAV_SEVERITY，用于 gcs:send_text（与 MAVLink 严重等级一致）
local MAV_SEVERITY = {
    EMERGENCY = 0,
    ALERT = 1,
    CRITICAL = 2,
    ERROR = 3,
    WARNING = 4,
    NOTICE = 5,
    INFO = 6,
    DEBUG = 7,
}

---@return function
---@return integer
function update()
    local pwm = rc:get_pwm(RC_CHAN)
    local now = millis()

    -- 无有效 RC 输入时不改状态
    if pwm == nil then
        return update, LOOP_MS
    end

    if pwm >= PWM_HIGH_US then
        -- 刚开始进入高位：记录起点（不在此处清零 fired_this_hold，避免已停桨后从中位回高位时误判为未触发）
        if hold_start_ms == nil then
            hold_start_ms = now
        end

        local held_ms = now - hold_start_ms
        -- 已满长按时间且尚未触发：执行紧急停桨
        if (held_ms >= HOLD_MS) and (not fired_this_hold) then
            -- 以 SCRIPTING 源调用与 RC 开关相同的 MOTOR_ESTOP 逻辑
            if rc:run_aux_function(AUX_FUNC_MOTOR_ESTOP, AUX_SWITCH_HIGH) then
                fired_this_hold = true
                gcs:send_text(
                    MAV_SEVERITY.CRITICAL,
                    string.format("CH%u hold %.1fs: Motor EStop", RC_CHAN, HOLD_MS * 0.001)
                )
            else
                gcs:send_text(MAV_SEVERITY.ERROR, "EStop HIGH fail")
            end
        end
    elseif pwm <= PWM_LOW_US then
        -- 低位：清除长按计时；若此前已停桨则解除
        hold_start_ms = nil
        if fired_this_hold then
            if rc:run_aux_function(AUX_FUNC_MOTOR_ESTOP, AUX_SWITCH_LOW) then
                gcs:send_text(MAV_SEVERITY.NOTICE, string.format("CH%u low: EStop released", RC_CHAN))
            end
            fired_this_hold = false
        end
    else
        -- 中间档：打断长按累计，但不自动解除已生效的 EStop（与固件 MOTOR_ESTOP 中间位行为一致）
        hold_start_ms = nil
    end

    return update, LOOP_MS
end

return update()
