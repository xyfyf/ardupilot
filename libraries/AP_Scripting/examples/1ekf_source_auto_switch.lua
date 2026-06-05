-- 自动切换 EKF3 数据源脚本 (飞行中 GPS/罗盘 异常时自动降级保命)
-- 脚本会自动配置 EK3_SRC2 的相关参数，定高使用气压计

local RUN_INTERVAL_MS = 1000 -- 每 1000 毫秒 (1秒) 检查一次

local MIN_GPS_STATUS = 5 -- 最低可接受的 GPS 状态 (5=RTK Float, 6=RTK Fixed。如果要求必须固定解，请改为 6)

local SOURCE_PRIMARY = 0   -- 对应 EK3_SRC1 (正常 GPS 模式)
local SOURCE_SECONDARY = 1 -- 对应 EK3_SRC2 (无 GPS 降级模式)

local COPTER_MODE_ALTHOLD = 2 -- Copter 的 AltHold 模式编号

-- 核心逻辑配置
local FORCE_GPS_BEFORE_ARMING = true -- true: 未解锁时强制使用主源(GPS)，有问题就不让解锁
local SWITCH_IN_FLIGHT = true        -- true: 飞行中GPS异常时允许切换备用源保命

-- 初始化 EK3_SRC2 参数 (定高用气压计，无GPS水平位置)
local function init_src2_params()
    local params_to_set = {
        {"EK3_SRC2_POSXY", 0}, -- 0: None
        {"EK3_SRC2_VELXY", 0}, -- 0: None
        {"EK3_SRC2_POSZ",  1}, -- 1: Baro (气压计)
        {"EK3_SRC2_VELZ",  0}, -- 0: None
    }
    
    for i = 1, #params_to_set do
        local p_name = params_to_set[i][1]
        local p_val = params_to_set[i][2]
        local current_val = param:get(p_name)
        if current_val ~= p_val then
            param:set(p_name, p_val)
        end
    end
    gcs:send_text(6, "EKF SRC2 params initialized for Baro AltHold")
end

init_src2_params()

function update()
    local is_armed = arming:is_armed()
    local current_source = ahrs:get_posvelyaw_source_set()

    -- 1. 未解锁时的逻辑：如果要求起飞前必须有GPS，则强制切回主源，让飞控的原生预检去拦截解锁
    if not is_armed and FORCE_GPS_BEFORE_ARMING then
        if current_source ~= SOURCE_PRIMARY then
            ahrs:set_posvelyaw_source_set(SOURCE_PRIMARY)
            gcs:send_text(6, "Disarmed: Forced EKF Source 1 (GPS required to arm)")
        end
        return update, RUN_INTERVAL_MS
    end

    -- 2. 飞行中不允许切换的保护 (如果设为 false 则直接跳过)
    if is_armed and not SWITCH_IN_FLIGHT then
        return update, RUN_INTERVAL_MS
    end

    -- 3. 检查 GPS 状态
    local primary_gps = gps:primary_sensor()
    local gps_good = false
    if primary_gps ~= nil then
        local gps_status = gps:status(primary_gps)
        if (gps_status >= MIN_GPS_STATUS) and gps:is_healthy(primary_gps) then
            gps_good = true
        end
    end

    -- 4. 检查罗盘状态，并动态设置 EK3_SRC2_YAW
    local compass_ok = compass:healthy(0)
    local target_yaw_src = 0 -- 默认不用罗盘 (0: None)
    if compass_ok then
        target_yaw_src = 1   -- 罗盘健康时使用 (1: Compass)
    end
    
    local current_yaw_src = param:get("EK3_SRC2_YAW")
    if current_yaw_src ~= target_yaw_src then
        param:set("EK3_SRC2_YAW", target_yaw_src)
        if target_yaw_src == 0 then
            gcs:send_text(4, "Compass bad, EK3_SRC2_YAW set to None")
        else
            gcs:send_text(6, "Compass OK, EK3_SRC2_YAW set to Compass")
        end
    end

    -- 5. 切换 EKF 源及飞行模式保命
    if gps_good then
        -- GPS 良好，切回主源 (SRC1)
        if current_source ~= SOURCE_PRIMARY then
            ahrs:set_posvelyaw_source_set(SOURCE_PRIMARY)
            gcs:send_text(6, "GPS Good: Switched to EKF Source 1")
        end
    else
        -- GPS 异常，切到备用源 (SRC2)
        if current_source ~= SOURCE_SECONDARY then
            ahrs:set_posvelyaw_source_set(SOURCE_SECONDARY)
            gcs:send_text(4, "GPS Bad: Switched to EKF Source 2 (Baro)")
            
            -- 如果在空中切到了无GPS的备用源，强制将飞行模式切换为 AltHold (定高增稳)
            -- 防止原本在 Loiter/Auto 等模式下因为失去位置源而导致姿态乱飞或坠机
            if is_armed then
                local current_mode = vehicle:get_mode()
                if current_mode ~= COPTER_MODE_ALTHOLD then
                    if vehicle:set_mode(COPTER_MODE_ALTHOLD) then
                        gcs:send_text(4, "Emergency: Switched to AltHold mode")
                    else
                        gcs:send_text(3, "Emergency: Failed to switch to AltHold!")
                    end
                end
            end
        end
    end
    
    return update, RUN_INTERVAL_MS
end

gcs:send_text(6, "EKF Source Auto-Switch Script Loaded")
return update()