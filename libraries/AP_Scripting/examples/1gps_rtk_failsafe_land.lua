-- 脚本名称: 1gps_rtk_failsafe_land.lua
-- 功能描述: 
-- 1. 当RTK(GPS1)出现离谱的异常值（位置突变、精度下降）时，强制切换到普通GPS(GPS2)。
-- 2. 当普通GPS(GPS2)也出现离谱的异常值时，强制切换到 AltHold (定高) 模式防止乱飞。
-- 3. 当所有的GPS/RTK信号都彻底失效（低于3D Fix）持续一定时间后，强制飞机原地降落 (LAND)。

local RUN_INTERVAL_MS = 500      -- 脚本运行频率 (500毫秒)
local FAIL_TIMEOUT_MS = 3000     -- 彻底失效的超时时间 (3秒)

local COPTER_MODE_ALTHOLD = 2    -- 多旋翼的 ALTHOLD 模式编号
local COPTER_MODE_LAND = 9       -- 多旋翼的 LAND 模式编号

-- 异常值判定阈值 (可根据实际情况调整)
local MAX_HDOP = 200             -- HDOP > 2.0 认为异常 (200 = 2.0)
local MAX_HACC = 5.0             -- 水平精度 > 5.0米 认为异常
local MAX_JUMP_SPEED = 15.0      -- 位置突变速度 > 15 m/s 认为异常 (GPS Glitch)

local fail_start_time = 0
local last_loc = {}
local last_time = {}
local is_forced_gps2 = false

-- 检查指定的GPS是否出现“离谱的异常值”
function is_gps_outrageous(instance)
    local status = gps:status(instance)
    -- 如果状态低于 3D Fix (3)，直接认为不可用/异常
    if not status or status < 3 then
        return true
    end
    
    -- 检查 HDOP
    local hdop = gps:get_hdop(instance)
    if hdop and hdop > MAX_HDOP then
        return true
    end
    
    -- 检查水平精度
    local hacc = gps:horizontal_accuracy(instance)
    if hacc and hacc > MAX_HACC then
        return true
    end
    
    -- 检查位置突变 (GPS Glitch)
    local loc = gps:location(instance)
    local now = millis()
    if loc then
        if last_loc[instance] and last_time[instance] then
            local dist = last_loc[instance]:get_distance(loc)
            local dt = (now - last_time[instance]) / 1000.0
            if dt > 0 and dt < 5.0 then
                local speed = dist / dt
                if speed > MAX_JUMP_SPEED then
                    return true
                end
            end
        end
        last_loc[instance] = loc
        last_time[instance] = now
    end
    
    return false
end

function update()
    -- 如果飞机没有解锁，则不需要执行检测，重置状态
    if not arming:is_armed() then
        fail_start_time = 0
        if is_forced_gps2 then
            param:set("GPS_AUTO_SWITCH", 1)
            is_forced_gps2 = false
        end
        return update, RUN_INTERVAL_MS
    end

    local rtk_bad = is_gps_outrageous(0)
    local gps_bad = is_gps_outrageous(1)
    
    local gps1_status = gps:status(0) or 0
    local gps2_status = gps:status(1) or 0

    -- 1. 彻底失效逻辑：两个GPS都低于 3D_FIX
    if gps1_status < 3 and gps2_status < 3 then
        local now = millis()
        if fail_start_time == 0 then
            fail_start_time = now
        elseif (now - fail_start_time) > FAIL_TIMEOUT_MS then
            local current_mode = vehicle:get_mode()
            if current_mode ~= COPTER_MODE_LAND then
                gcs:send_text(2, "CRITICAL: All GPS Lost! Forcing LAND mode.")
                vehicle:set_mode(COPTER_MODE_LAND)
            end
        end
        return update, RUN_INTERVAL_MS
    else
        fail_start_time = 0
    end

    local current_mode = vehicle:get_mode()

    -- 2. 如果两个GPS都有离谱的异常值，切到 AltHold
    if rtk_bad and gps_bad then
        -- 如果当前是依赖GPS的模式（简单起见，只要不是AltHold和Land就切）
        if current_mode ~= COPTER_MODE_ALTHOLD and current_mode ~= COPTER_MODE_LAND then
            gcs:send_text(2, "GPS1 & GPS2 Abnormal! Forcing ALTHOLD.")
            vehicle:set_mode(COPTER_MODE_ALTHOLD)
        end
        return update, RUN_INTERVAL_MS
    end

    -- 3. 如果仅RTK异常，切换到普通GPS (GPS2)
    if rtk_bad and not gps_bad then
        if not is_forced_gps2 then
            param:set("GPS_AUTO_SWITCH", 0)
            param:set("GPS_PRIMARY", 1)
            is_forced_gps2 = true
            gcs:send_text(3, "RTK Abnormal! Switched to GPS2.")
        end
    end

    -- 4. 如果RTK恢复正常，切回RTK (恢复自动切换逻辑)
    if not rtk_bad and is_forced_gps2 then
        param:set("GPS_AUTO_SWITCH", 1)
        param:set("GPS_PRIMARY", 0)
        is_forced_gps2 = false
        gcs:send_text(4, "RTK Recovered! Switched back to RTK.")
    end

    return update, RUN_INTERVAL_MS
end

gcs:send_text(6, "GPS/RTK Failsafe & Switch script loaded.")
return update()
