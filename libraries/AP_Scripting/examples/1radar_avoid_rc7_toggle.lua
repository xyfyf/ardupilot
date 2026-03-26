--[[
  CAN 雷达避障开关：RC7 PWM + 失控保护
  - 无有效遥控信号（rc:has_valid_input() 为 false，含未连接、失控等）：强制打开避障
  - 有遥控时：PWM > 1500 打开避障；<= 1500 关闭避障

  依赖：参数文件中已配置 DroneCAN 测距与 PRX（如 RNGFND1_TYPE=24、PRX1_TYPE=4 等），本脚本仅切换 AVOID_ENABLE。

  使用前请设置：
    SCR_ENABLE = 1
    建议 SCR_HEAP_SIZE 按脚本内存需求调整（默认通常足够）

  版本：
    1.0 初始版本
    1.1增加失控打开避障功能
--]]

local PWM_THRESHOLD = 1500
local RC_CH = 7

-- 与 1号caac-eft-fmu参数-rtk-cangnss-elrs-3dr-allok-配置雷达-匹配gps优先级3.0.param 中“开”一致
local AVOID_ON = 7
local AVOID_OFF = 0

-- 使用 Parameter 对象避免每次按字符串查参（见 docs.lua / param_get_set_test.lua）
local AVOID_ENABLE = Parameter("AVOID_ENABLE")

local last_state = nil -- true = 避障开, false = 关, nil = 未初始化

--- 将当前期望的避障状态应用到飞控（仅在与上次不同时写入，减少开销）
---@param want_on boolean
---@return boolean
local function apply_avoid(want_on)
    local v = want_on and AVOID_ON or AVOID_OFF
    if AVOID_ENABLE:get() == v then
        return true
    end
    return AVOID_ENABLE:set(v)
end

function update()
    -- 无有效 RC：默认开避障（与失控/断链路时仍希望雷达参与一致）
    local want_on
    local reason

    if not rc:has_valid_input() then
        want_on = true
        reason = "无RC信号/失控，强制开避障"
    else
        local pwm = rc:get_pwm(RC_CH)
        if pwm == nil then
            -- 有 RC 但读不到该通道：保守保持开启
            want_on = true
            reason = "RC7不可用，保持开避障"
        else
            want_on = pwm > PWM_THRESHOLD
            reason = string.format("RC7=%u %s %u", pwm, want_on and ">" or "<=", PWM_THRESHOLD)
        end
    end

    if want_on ~= last_state then
        if apply_avoid(want_on) then
            last_state = want_on
            if want_on then
                gcs:send_text(6, string.format("避障已开 (%s, AVOID_ENABLE=%d)", reason, AVOID_ON))
            else
                gcs:send_text(6, string.format("避障已关 (%s, AVOID_ENABLE=%d)", reason, AVOID_OFF))
            end
        else
            gcs:send_text(3, "AVOID_ENABLE 设置失败")
        end
    end

    return update, 100
end

gcs:send_text(6, "1radar_avoid_rc7_toggle: 已加载 (无RC强制开避障; 有RC时RC7>1500开)")

return update()
