-- fence_althold_to_loiter.lua
--
-- 用途：飞机在 AltHold（定高）模式下飞行，一旦触发电子围栏（围栏 breach），
--       自动把飞行模式切换到 Loiter（GPS 悬停定位），防止飞出围栏。
--
-- 行为说明：
--   1. 只在已解锁(armed) 且 当前模式为 AltHold 时监控围栏；
--   2. 检测到 fence:get_breaches() ~= 0 时，调用 vehicle:set_mode(LOITER)；
--   3. 切换成功后保持 Loiter，不再反复切（飞手可自行再切回任何模式）；
--   4. 一旦飞手手动切到非 AltHold/Loiter 的模式，脚本认为飞手已接管，复位状态；
--   5. 切到 Loiter 失败（例如没有 GPS / 卫星不足）会通过 GCS 弹出提示。
--
-- 适用：ArduCopter（多旋翼），需要启用脚本（SCR_ENABLE=1），并打开围栏（FENCE_ENABLE=1）。
--
-- 把本文件放到 SD 卡 /APM/scripts/ 目录下，重启或重新加载脚本即可生效。

-- ====== 可配置参数 ======
local LOOP_MS = 200    -- 检查周期 ms

-- ArduCopter 飞行模式号
local MODE_ALTHOLD = 2
local MODE_LOITER  = 5

-- ====== 内部状态 ======
local switched = false    -- 是否本次已经由脚本切到 Loiter 了

local function describe_breach(breaches)
    local names = {}
    if (breaches & 1) ~= 0 then table.insert(names, "MaxAlt") end
    if (breaches & 2) ~= 0 then table.insert(names, "Circle") end
    if (breaches & 4) ~= 0 then table.insert(names, "Polygon") end
    if (breaches & 8) ~= 0 then table.insert(names, "MinAlt") end
    if #names == 0 then return "None" end
    return table.concat(names, "+")
end

function update()
    if not arming:is_armed() then
        switched = false
        return update, LOOP_MS
    end

    local mode = vehicle:get_mode()

    if mode == MODE_ALTHOLD then
        local breaches = fence:get_breaches()
        if breaches ~= 0 and not switched then
            if vehicle:set_mode(MODE_LOITER) then
                switched = true
                gcs:send_text(2, string.format(
                    "Fence breach (%s): AltHold -> Loiter",
                    describe_breach(breaches)))
            else
                gcs:send_text(2, "Fence breach: switch to Loiter FAILED (no GPS?)")
            end
        end

    elseif mode == MODE_LOITER then
        -- 维持脚本接管状态，等待飞手自己处理；不做任何事
    else
        -- 飞手切到其它模式 -> 视为飞手已接管，复位
        switched = false
    end

    return update, LOOP_MS
end

return update, 1000
