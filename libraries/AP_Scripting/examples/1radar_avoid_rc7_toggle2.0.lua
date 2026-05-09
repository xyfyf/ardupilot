--[[
  CAN 雷达避障开关：RC7 PWM + 失控保护
  - 无有效遥控信号（rc:has_valid_input() 为 false，含未连接、失控等）：强制打开避障
  - 有遥控时：PWM > 1500 打开避障；<= 1500 关闭避障

  依赖：参数文件中已配置 DroneCAN 测距与 PRX（如 RNGFND1_TYPE=24、PRX1_TYPE=4 等），本脚本仅切换 AVOID_ENABLE。
  AVOID_ENABLE 位掩码：关通道时写 1（仅围栏），开通道时写 7（围栏+近距+Beacon）。若未使用围栏，bit0 无实际效果但近距/雷达已关闭。

  使用前请设置：
    SCR_ENABLE = 1
    建议 SCR_HEAP_SIZE 按脚本内存需求调整（默认通常足够）

  版本：
    1.0 初始版本
    1.1增加失控打开避障功能
    1.2 关闭时 AVOID_ENABLE=1（仅围栏），打开时=7
--]]

local PWM_THRESHOLD = 1500
local RC_CH = 7

-- 开：近距+围栏+Beacon 全开；关：仅保留围栏位（测距/近距不参与避障，地面站等仍可按需显示其它数据）
local AVOID_ON = 7
local AVOID_OFF = 1

-- 使用 Parameter 对象避免每次按字符串查参（见 docs.lua / param_get_set_test.lua）
local AVOID_ENABLE = Parameter("AVOID_ENABLE")

local last_state = nil -- true = 全开(7), false = 仅围栏(1), nil = 未初始化

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
        want_on = false
        reason = "no RC"
    else
        local pwm = rc:get_pwm(RC_CH)
        if pwm == nil then
            -- 有 RC 但读不到该通道：保守保持开启
            want_on = true
            reason = "RC7 n/a"
        else
            want_on = pwm > PWM_THRESHOLD
            reason = string.format("ch7=%u", pwm)
        end
    end

    -- 模式保护逻辑：仅在依赖GPS的模式下开启避障（屏蔽姿态、定高模式）
    -- 0: Stabilize, 1: Acro, 2: AltHold, 3: Auto, 4: Guided, 5: Loiter, 6: RTL, 7: Circle
    -- 9: Land, 11: Drift, 13: Sport, 14: Flip, 15: AutoTune, 16: PosHold, 17: Brake, 18: Throw, 19: Avoid_ADSB
    -- 20: Guided_NoGPS, 21: Smart_RTL, 22: FlowHold, 23: Follow, 24: ZigZag, 25: SystemID, 26: Autorotate, 27: AutoRTL
    if want_on then
        local mode = vehicle:get_mode()
        -- 如果是 Stabilize(0), Acro(1), AltHold(2), Drift(11), Sport(13) 等非GPS模式，强制关闭避障
        -- 为了让遥控器能绝对控制，并支持在地面/定高模式下测试雷达，此处注释掉强制关闭逻辑
        -- if mode == 0 or mode == 1 or mode == 2 or mode == 11 or mode == 13 or mode == 20 then
        --     want_on = false
        --     reason = string.format("mode %d no-GPS", mode)
        -- end
    end

    if want_on ~= last_state then
        if apply_avoid(want_on) then
            last_state = want_on
            if want_on then
                gcs:send_text(6, string.format("Avoid ON: %s", reason))
            else
                gcs:send_text(6, string.format("Avoid OFF: %s", reason))
            end
        else
            gcs:send_text(3, "AVOID_ENABLE set failed")
        end
    end

    return update, 100
end

-- 启动横幅：短英文，减少地面站消息长度
gcs:send_text(6, "Radar avoid loaded")

return update()
