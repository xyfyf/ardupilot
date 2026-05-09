--[[
  TR24DCA100 24GHz FMCW CAN 雷达避障驱动脚本
  --
  功能:
    - 从 CAN1 接口（非 UAVCAN）读取雷达原始 CAN 帧
    - 解析距离数据并推送给飞控 Proximity (PRX) 避障库
    - 通道 7 PWM > 1500 时开启避障；PWM < 1500 时关闭避障

  CAN 帧格式 (来自官方协议文档表 3-2):
    文档字节编号含 CAN ID 头 4 字节, data[N] = 文档 Byte(N+4)
    data[0] = Byte4:  0xA5      (帧头1)
    data[1] = Byte5:  0x5A      (帧头2)
    data[2] = Byte6:  0x04      (帧长)
    data[3] = Byte7:  距离低字节 (cm, little-endian)
    data[4] = Byte8:  距离高字节 (cm, little-endian), 距离 = Byte8*256 + Byte7
    data[5] = Byte9:  信号强度 (0~255)
    data[6] = Byte10: 状态值 (0x00 = 正常)
    data[7] = Byte11: 校验和 = (Byte7+Byte8+Byte9+Byte10) & 0xFF

  雷达规格 (来自官方手册):
    测距范围: 1m ~ 25m
    更新频率: 50Hz
    CAN波特率: 1Mbps
    CANID: 0x0301=前向, 0x0302=后向
    安装: 线头朝下, 建议向上倾斜 15° 安装

  飞控参数配置:
    SCR_ENABLE      = 1
    CAN_P1_DRIVER   = 1
    CAN_D1_PROTOCOL = 10       (Scripting, 非 DroneCAN)
    CAN_P1_BITRATE  = 1000000  (1Mbps, 雷达固定波特率)
    PRX1_TYPE       = 15       (Scripting Proximity 后端)
--]]

---@diagnostic disable: undefined-field
---@diagnostic disable: undefined-global

-- ===================== 用户配置 =====================
local CAN_ID_RADAR    = 0x0301   -- 雷达 CANID (0x0301=前向, 0x0302=后向)
local CAN_BUF_LEN     = 10       -- CAN 接收缓冲区帧数
local PRX_MIN_M       = 1.0      -- 雷达最小探测距离: 1m (来自官方手册)
local PRX_MAX_M       = 25.0     -- 雷达最大探测距离: 25m (来自官方手册)
local RC_CHANNEL      = 7        -- 避障开关通道
local RC_THRESHOLD    = 1500     -- PWM 阈值 (>此值开启避障)
local RADAR_YAW_DEG   = 0.0      -- 雷达安装偏航角 (正前方=0, 右=90, 左=-90)
local RADAR_PITCH_DEG = 0.0      -- 雷达安装俯仰角 (水平=0)
local UPDATE_RATE_MS  = 20       -- 脚本循环周期 (ms), 与雷达 50Hz 更新率匹配
local PARAM_NUM_LUA_PRX_BACKEND = 15  -- Lua Proximity 后端类型编号
-- ====================================================

-- 初始化 CAN 驱动 (CAN1 协议设置为 Scripting=10)
local can_driver = CAN:get_device(CAN_BUF_LEN)
if not can_driver then
    can_driver = CAN:get_device2(CAN_BUF_LEN)
end
if not can_driver then
    gcs:send_text(0, "TR24: No Scripting CAN")
    return
end

-- 添加 CAN 接收过滤器, 只接收 ID=0x0301 的帧
-- mask=0x7FF 匹配所有 11 位标准帧 ID
can_driver:add_filter(0x7FF, CAN_ID_RADAR)
gcs:send_text(6, "TR24: CAN init OK")

-- 状态变量
local lua_prx_backend       = nil    -- Proximity 后端, 首次 update 时初始化
local prx_initialized       = false  -- 后端是否已初始化
local prx_wait_logged       = false  -- "等待后端"消息是否已打印过 (防洪水)
local min_max_set           = false  -- set_distance_min_max 是否已调用
local avoidance_enabled     = false  -- 当前避障是否开启
local last_avoidance_state  = nil    -- 上次避障状态 (用于检测切换)
local last_dist_m           = -1     -- 最近一次有效距离 (米)
local last_log_ms           = 0      -- 上次距离打印的时间戳 (ms)
local LOG_INTERVAL_MS       = 5000   -- 距离打印间隔: 5s

-- 在 update 首次运行时查找 Proximity 后端 (惰性初始化)
-- 这样可以避免飞控启动时 PRX 子系统未就绪的问题
local function try_init_prx()
    if prx_initialized then
        return true
    end
    local sensor_count = proximity:num_sensors()
    for j = 0, sensor_count - 1 do
        local device = proximity:get_backend(j)
        if device and (device:type() == PARAM_NUM_LUA_PRX_BACKEND) then
            lua_prx_backend = device
            prx_initialized = true
            gcs:send_text(6, "TR24: PRX init OK")
            return true
        end
    end
    return false
end

-- 主更新函数 (无返回值, 由 protected_wrapper 调度)
local function update()
    -- 第一步: 不管 PRX 是否就绪, 都先清空 CAN 缓冲, 防止积压旧数据
    -- PRX 就绪后会立即处理最新数据, 而不是先吐出一堆历史旧帧
    local frames = {}
    local count = 0
    while count < 10 do
        local frame = can_driver:read_frame()
        if not frame then break end
        frames[count + 1] = frame
        count = count + 1
    end

    -- 第二步: 确保 Proximity 后端已准备好 (惰性初始化)
    if not try_init_prx() then
        -- 只打印一次"等待"提示, 防止每 20ms 刷屏 GCS 消息窗口
        if not prx_wait_logged then
            gcs:send_text(3, "TR24: Waiting PRX")
            prx_wait_logged = true
        end
        return
    end

    -- 第三步: 设置距离范围 (只需调用一次, docs 说明: Only need to do it once)
    if not min_max_set and lua_prx_backend then
        lua_prx_backend:set_distance_min_max(PRX_MIN_M, PRX_MAX_M)
        min_max_set = true
    end

    -- 第四步: 读取遥控器通道 7 判断避障开关状态
    local pwm = rc:get_pwm(RC_CHANNEL)
    if pwm then
        avoidance_enabled = (pwm > RC_THRESHOLD)
    end

    -- 状态切换时发送 GCS 通知
    if avoidance_enabled ~= last_avoidance_state then
        if avoidance_enabled then
            gcs:send_text(6, string.format("TR24: Avoid ON (CH%d)", RC_CHANNEL))
        elseif last_avoidance_state ~= nil then
            gcs:send_text(6, string.format("TR24: Avoid OFF (CH%d)", RC_CHANNEL))
        end
        last_avoidance_state = avoidance_enabled
    end

    -- 第五步: 处理已读取的 CAN 帧
    -- 注意: 即使避障被关闭, 上面也已经将帧读出清空了缓冲区,
    -- 这里只在避障开启时才解析和推送距离数据。
    if not avoidance_enabled then
        return
    end

    for i = 1, count do
        local frame = frames[i]

        -- 跳过错误帧和远程帧
        if not frame:isErrorFrame() and not frame:isRemoteTransmissionRequest() then
            -- 校验帧头: data[0]=0xA5, data[1]=0x5A (文档 Byte4, Byte5)
            if frame:data(0) == 0xA5 and frame:data(1) == 0x5A then

                -- 校验状态值: data[6] (文档 Byte10) 必须为 0x00 才表示数据有效
                if frame:data(6) == 0x00 then

                    -- 校验和验证: (Byte7+Byte8+Byte9+Byte10) & 0xFF = data[7]
                    local chk = (frame:data(3) + frame:data(4) + frame:data(5) + frame:data(6)) & 0xFF
                    if chk == frame:data(7) then

                        -- 解析距离: Byte8*256 + Byte7 = data[4]*256 + data[3], 单位 cm
                        local dist_cm = frame:data(3) | (frame:data(4) << 8)
                        local dist_m  = dist_cm / 100.0

                        -- 仅推送在官方量程 (1m~25m) 内的有效距离数据
                        if dist_m >= PRX_MIN_M and dist_m <= PRX_MAX_M and lua_prx_backend then
                            -- update_boundary=true: 每次直接更新避障边界 (适合单波束雷达)
                            lua_prx_backend:handle_script_distance_msg(dist_m, RADAR_YAW_DEG, RADAR_PITCH_DEG, true)
                            last_dist_m = dist_m
                        end

                    end -- checksum ok
                end -- status ok
            end -- header ok
        end
    end

    -- 每 5s 向 GCS 输出一次当前距离 (便于调试, 无论避障是否开启都输出)
    ---@diagnostic disable-next-line: cast-local-type
    local now_ms = millis()
    if (now_ms - last_log_ms) >= LOG_INTERVAL_MS then
        last_log_ms = now_ms
        local state_str = avoidance_enabled and "ON" or "OFF"
        if last_dist_m >= 0 then
            gcs:send_text(6, string.format("TR24: Dist=%.2fm %s", last_dist_m, state_str))
        else
            gcs:send_text(6, string.format("TR24: Dist=N/A %s", state_str))
        end
    end
end

-- pcall 保护包装器: 捕获 update() 运行时错误, 防止脚本崩溃停止
local function protected_wrapper()
    local ok, err = pcall(update)
    if not ok then
        gcs:send_text(0, "TR24 Error: " .. tostring(err))
        return protected_wrapper, 1000  -- 出错后延迟 1s 再恢复运行
    end
    return protected_wrapper, UPDATE_RATE_MS
end

return protected_wrapper()
