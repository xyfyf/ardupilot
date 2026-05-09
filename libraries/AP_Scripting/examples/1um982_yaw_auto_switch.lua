-- 1um982_yaw_auto_switch.lua
-- 功能：当安装了UM982并且抓到了GPS航向就把EK3_SRC1_YAW设为2(GPS)，否则设为1(Compass)
-- 约束：只在解锁前更改参数，解锁后不更改。

local UPDATE_RATE_MS = 1000

local YAW_COMPASS = 1
local YAW_GPS = 2

local VOTE_MAX = 3
local yaw_vote = 0

local MAV_SEVERITY = {
    EMERGENCY = 0, ALERT = 1, CRITICAL = 2, ERROR = 3,
    WARNING = 4, NOTICE = 5, INFO = 6, DEBUG = 7
}

-- 记录当前脚本认为的偏航源，避免频繁读取和打印
local current_yaw_src = -1

local function update()
    -- 如果已经解锁，则不更改参数，直接返回
    if arming:is_armed() then
        return update, UPDATE_RATE_MS
    end

    local actual_yaw_src = param:get("EK3_SRC1_YAW")
    if actual_yaw_src == nil then
        return update, UPDATE_RATE_MS
    end

    -- 初始化 current_yaw_src
    if current_yaw_src == -1 then
        current_yaw_src = actual_yaw_src
    end

    local has_gps_yaw = false
    local num_gps = gps:num_sensors()
    
    -- 遍历所有 GPS 实例，检查是否抓到了 GPS 航向
    -- 只要有任何一个 GPS 实例提供了有效的 yaw，就认为抓到了 GPS 航向（UM982 双天线已就绪）
    for i = 0, num_gps - 1 do
        local yaw_valid, yaw_deg, yaw_acc, yaw_ms = gps:gps_yaw_deg(i)
        if yaw_valid then
            has_gps_yaw = true
            break
        end
    end

    -- 投票机制防抖 (大约需要 VOTE_MAX 秒才能切换，避免信号临界点频繁跳动)
    if has_gps_yaw then
        yaw_vote = math.min(yaw_vote + 1, VOTE_MAX)
    else
        yaw_vote = math.max(yaw_vote - 1, -VOTE_MAX)
    end

    local target_yaw_src = current_yaw_src
    if yaw_vote >= VOTE_MAX then
        target_yaw_src = YAW_GPS
    elseif yaw_vote <= -VOTE_MAX then
        target_yaw_src = YAW_COMPASS
    end

    -- 如果需要更改参数
    if target_yaw_src ~= current_yaw_src then
        -- 使用 param:set 而非 set_and_save，避免频繁切换磨损闪存
        -- 如果需要掉电保存，请在地面站手动写入一次参数
        if param:set("EK3_SRC1_YAW", target_yaw_src) then
            current_yaw_src = target_yaw_src
            if target_yaw_src == YAW_GPS then
                gcs:send_text(MAV_SEVERITY.INFO, "UM982 Yaw: EK3_SRC1_YAW set to 2 (GPS)")
            else
                gcs:send_text(MAV_SEVERITY.INFO, "UM982 Yaw: EK3_SRC1_YAW set to 1 (Compass)")
            end
        else
            gcs:send_text(MAV_SEVERITY.ERROR, "UM982 Yaw: Failed to set EK3_SRC1_YAW")
        end
    end

    return update, UPDATE_RATE_MS
end

gcs:send_text(MAV_SEVERITY.INFO, "1um982_yaw_auto_switch.lua loaded")
return update, UPDATE_RATE_MS
