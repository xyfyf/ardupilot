-- fence_althold_to_loiter.lua
--
-- 用途：飞机在 AltHold（定高）模式下飞行，靠近或飞出电子围栏时
--       自动切换到 Loiter（GPS 悬停定位），防止飞出围栏。
--
-- 行为说明：
--   1. 只在已解锁(armed) 且 当前模式为 AltHold 时监控围栏；
--   2. 检测"进入缓冲带"采用直接计算距离的方式（圆形围栏：直接算飞机离
--      Home 的水平距离；多边形/其他：fallback 到 fence:get_breaches()）；
--      避免依赖 fence:get_margin_breaches()——该接口对圆形围栏在实际飞行
--      中存在不能可靠触发的问题，导致总是飞出去才切 Loiter；
--      同时按当前朝围栏方向的速度预测刹车距离，速度越快越提前切换，
--      保证切到 Loiter 后刹得住、不冲出围栏（与定点模式避障原理一致）；
--   3. 切换成功后保持 Loiter，不再反复切（飞手可自行再切回任何模式）；
--   4. 一旦飞手手动切到非 AltHold/Loiter 的模式，脚本认为飞手已接管，复位状态；
--   5. 切到 Loiter 失败（例如没有 GPS / 卫星不足）会通过 GCS 弹出提示。
--
-- 适用：ArduCopter（多旋翼），需要启用脚本（SCR_ENABLE=1），并打开围栏（FENCE_ENABLE=1）。

-- ====== 可配置参数 ======
local LOOP_MS = 200    -- 检查周期 ms

-- ArduCopter 飞行模式号
local MODE_ALTHOLD = 2
local MODE_LOITER  = 5

-- ====== 内部状态 ======
local switched = false    -- 是否本次已经由脚本切到 Loiter 了

-- 估算切到 Loiter 后沿外飞方向还要冲出去多远（刹车距离，米）
-- v_out: 朝围栏外的径向速度 m/s（<=0 时无需刹车）
local function braking_distance(v_out)
    if v_out <= 0 then
        return 0
    end
    -- Loiter 刹车加速度 cm/s/s，默认 250；刹车启动延迟 s，默认 1
    local brk_accel = param:get("LOIT_BRK_ACCEL") or 250
    local brk_delay = param:get("LOIT_BRK_DELAY") or 1.0
    local accel = math.max(brk_accel, 50) * 0.01   -- 转 m/s^2，防止除 0
    -- 延迟期间匀速 + 之后匀减速；再加脚本检测周期的裕量
    return v_out * (brk_delay + LOOP_MS * 0.001) + (v_out * v_out) / (2 * accel)
end

-- 判断飞机是否已进入圆形围栏缓冲带（距边界 < FENCE_MARGIN + 刹车距离）或已越界
-- 返回 true 表示需要切 Loiter，同时返回原因字符串
local function circle_fence_triggered()
    -- 直接算水平距离，不依赖 get_margin_breaches()
    local pos = ahrs:get_relative_position_NED_home()
    if pos == nil then
        return false, ""
    end
    local dist = math.sqrt(pos:x() * pos:x() + pos:y() * pos:y())

    local fence_radius = param:get("FENCE_RADIUS")
    local fence_margin = param:get("FENCE_MARGIN")
    if fence_radius == nil or fence_margin == nil then
        return false, ""
    end

    -- 计算朝围栏外的径向速度（水平速度在"Home->飞机"方向上的投影）
    local v_out = 0
    local vel = ahrs:get_velocity_NED()
    if vel ~= nil and dist > 0.1 then
        v_out = (vel:x() * pos:x() + vel:y() * pos:y()) / dist
    end

    if dist >= fence_radius then
        return true, string.format("breach(%.1fm out)", dist - fence_radius)
    elseif dist + braking_distance(v_out) >= fence_radius - fence_margin then
        return true, string.format("margin(%.1fm left, %.1fm/s out)", fence_radius - dist, v_out)
    end
    return false, ""
end

function update()
    if not arming:is_armed() then
        switched = false
        return update, LOOP_MS
    end

    local mode = vehicle:get_mode()

    if mode == MODE_ALTHOLD then
        local triggered, reason = circle_fence_triggered()

        -- 多边形/其他围栏兜底：真越界时也切
        if not triggered and fence:get_breaches() ~= 0 then
            triggered = true
            reason = "breach(other)"
        end

        if triggered and not switched then
            if vehicle:set_mode(MODE_LOITER) then
                switched = true
                gcs:send_text(2, string.format("Fence %s: AltHold -> Loiter", reason))
            else
                gcs:send_text(2, string.format("Fence %s: switch to Loiter FAILED (no GPS?)", reason))
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
