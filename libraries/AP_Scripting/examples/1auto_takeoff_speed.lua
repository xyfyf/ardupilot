--[[
  针对 CAAC 大动力无人机的 Auto 模式起飞限速脚本
  功能：在 0~1 米高度内限制上升速度，超过 1 米后恢复正常速度
  注意：此脚本仅在内存中修改参数，不会保存到飞控 Flash，避免磨损。
]]

local ALT_THRESHOLD = 1.0  -- 高度阈值，单位：米
local SLOW_SPEED = 20      -- 1米以下限制速度为 30 cm/s (0.3 m/s)，起飞极慢
local NORMAL_SPEED = 100   -- 1米以上恢复正常速度 100 cm/s (1.0 m/s)，对应你参数文件中的值
local PARAM_NAME = "WPNAV_SPEED_UP" -- 对应你参数文件中的参数名

local is_slow_mode = false -- 标记当前是否处于慢速起飞模式

-- 主循环函数
function update()
    -- 获取当前是否解锁
    local armed = arming:is_armed()
    
    -- 获取当前飞行模式 (Copter 中 Auto 模式编号为 3)
    local mode = vehicle:get_mode()
    
    -- 如果未解锁，或者不是 Auto 模式，确保速度恢复正常状态
    if not armed or mode ~= 3 then
        if is_slow_mode then
            param:set(PARAM_NAME, NORMAL_SPEED)
            is_slow_mode = false
        end
        return update, 200 -- 200ms 循环检查
    end

    -- 获取相对 Home 点的高度 (NED坐标系，D为Down，所以高度是负的D)
    local pos_d = ahrs:get_relative_position_D_home()
    if not pos_d then
        return update, 100
    end
    
    -- 计算当前相对高度 (单位：米)
    local alt = -pos_d

    -- 根据高度动态调整上升速度
    if alt < ALT_THRESHOLD then
        -- 在 1 米以下，且尚未进入慢速模式时，降低速度
        if not is_slow_mode then
            -- param:set 仅在内存中修改，不写入 Flash
            param:set(PARAM_NAME, SLOW_SPEED)
            is_slow_mode = true
            gcs:send_text(6, string.format("Takeoff: Slow speed (%d cm/s)", SLOW_SPEED))
        end
    else
        -- 超过 1 米，且当前处于慢速模式时，恢复正常速度
        if is_slow_mode then
            param:set(PARAM_NAME, NORMAL_SPEED)
            is_slow_mode = false
            gcs:send_text(6, string.format("Takeoff: Normal speed (%d cm/s)", NORMAL_SPEED))
        end
    end

    return update, 100 -- 100ms (10Hz) 循环运行，保证响应及时
end

-- 脚本初始化：获取飞控中当前的正常速度作为基准，防止覆盖你后续在地面站修改的配置
local current_speed = param:get(PARAM_NAME)
if current_speed and current_speed > SLOW_SPEED then
    NORMAL_SPEED = current_speed
end

-- 向地面站发送脚本加载成功的提示
gcs:send_text(6, "Auto Takeoff loaded")

return update, 1000 -- 延迟 1 秒后开始第一次执行