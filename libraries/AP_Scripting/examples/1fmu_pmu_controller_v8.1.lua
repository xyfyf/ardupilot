--[[
   PMU与电池双向MAVLink控制器 - V8.1
   - 作者: CYF
   - 日期: 2025-09-23
   - 描述: 
        V8.1: 新增航线智能喷洒控制和称重数据监控。
        
        【核心新增功能】
        1. 航线智能喷洒控制 - 只在AUTO模式的有效航线段启动水泵，避免药液浪费
        2. 称重数据实时监控 - 10秒间隔显示重量数据，支持字节序自动修正
        3. 喷头灵活控制 - 喷头可独立控制，支持预热/清洗/调试等场景
        
        【地面站控制消息支持】
        1. WEIGHT_CALIBRATION (505) - LED控制、重量校准、K值校准
        2. PUMP_CALIBRATION_CMD (507) - 水泵/播撒器校准控制
        3. SPRAY_SYSTEM_PARAMS (509) - 喷洒参数设置
        4. SPREADER_CONTROL (511) - 播撒器完整控制
        5. WEIGH_DATA_EFT (506) - 称重数据接收与处理
        
        【核心架构与通信】
        1. 双向MAVLink通信系统 - 基于串口的MAVLink 2.0协议，自包含解析状态机
        2. 多通道消息转发代理 - 同时向6个MAVLink通道转发(USB + 5个TELEM端口)
        3. 航线状态感知系统 - 实时监控AUTO模式、任务状态、航点进度
        
        【硬件控制功能】
        4. 智能农业设备控制 - 水泵系统(航线控制)、喷头系统(独立控制)、播撒器控制
        5. 遥控器/地面站双重控制 - RC通道与MAVLink消息智能融合控制
        6. 多重安全保护 - 偏航速率保护、航线状态检查、上升沿检测、PWM范围限制
        
        【校准与配置功能】
        7. 多种校准支持 - 地面站控制水泵校准、播撒器校准、重量传感器校准、LED控制
        8. 参数化配置系统 - 支持地面站动态配置15种控制参数
        
        【MAVLink消息处理】
        9. 完整自定义消息支持(501-515) - 传感器数据、设备状态、校准控制、校准结果、系统参数
        10. 地面站控制消息处理 - 505, 507, 509, 511四种控制消息的完整解析和应用
        11. 精确的CRC校验 - 基于C头文件的准确CRC_EXTRA表，完整MAVLink 2.0校验机制
        
        【电池与传感器管理】
        12. 智能电池监控 - 14S锂电池组监控，电压/电流/温度采集，标准BatteryMonitor接口
        13. 称重数据处理 - 自动字节序修正，实时重量显示，支持大载荷监控
        
        【飞行状态感知】
        14. 多源速度计算 - GPS地速优先，AHRS备用，cm/s精度水平速度反馈
        15. 姿态感知保护 - 偏航角速度监控，可调阈值，滤波算法防误触发
        16. 航线进度监控 - 实时跟踪航点索引，区分起飞/喷洒/返航阶段
        
        【性能优化特性】
        17. 高频数据处理 - 10ms循环(100Hz)，100ms控制发送(10Hz)，高效串口处理
        18. 调试与监控 - 实时状态反馈，版本信息上报，完整错误处理，地面站显示
        
        【航线智能控制逻辑】
        - 起飞阶段: 水泵关闭，喷头可预热
        - 前往首个航点: 水泵关闭，喷头可运转
        - 航线喷洒作业: 水泵+喷头协同工作
        - 返航阶段: 水泵关闭，喷头可清洗
        - 非AUTO模式: 水泵关闭，喷头独立控制
        
        基于V8.0稳定版本开发，增加航线智能感知和称重数据监控功能。
        适用场景: 精准农业自动喷洒、大载荷作业监控、智能航线管理、工业级应用
--]]

---------------------------------------------------------------------
-- 参数配置区域 (与之前版本相同)
---------------------------------------------------------------------
local parameters = {
    SERIAL_PORT_NUM = 0, BAUD_RATE = 115200,
    MAV_SYSTEM_ID = 1, MAV_COMPONENT_ID = 190,
    BATTERY_INSTANCE = 0, CELL_COUNT = 14,
    CALIBRATION_PUMP_CHAN = 8,
    CALIBRATION_SPREADER_CHAN = 9, SPRAY_MODE_SWITCH_CHAN = 10,
    LED_CONTROL_CHAN = 6,  -- LED控制通道
    YAW_RATE_ON_THRESH = 25.0, YAW_RATE_OFF_THRESH = 30.0, YAW_FILTER_ALPHA = 0.4,
    SEND_INTERVAL_MS = 100, LOOP_RATE_MS = 10 -- 加快循环以处理更多串口数据
}

-- 将自定义消息同时转发到这些MAVLink通道（0一般为USB/Type‑C，1一般为TELEM1；按需调整）
local FWD_CHANS = {0, 1, 2, 3 ,4 ,5}

---------------------------------------------------------------------
-- 控制命令常量定义
---------------------------------------------------------------------
local PUMP_CALIBRATION_CMD = 0x11      -- 水泵校准命令
local SPREADER_CALIBRATION_CMD = 0x31  -- 播撒器校准命令
local PUMP_ON_VALUE = 0x0101           -- 水泵开启值 (2字节大端)
local NOZZLE_ON_VALUE = 0x01010101     -- 喷头开启值 (4字节大端)

---------------------------------------------------------------------
-- 地面站控制变量 - 存储从MAVLink消息接收到的控制参数
---------------------------------------------------------------------
local gcs_control = {
    -- WEIGHT_CALIBRATION (505) 消息控制的变量
    led_control_enable = false,          -- LED控制使能
    led_brightness_right = 0,            -- 右LED亮度 0-100
    led_brightness_left = 0,             -- 左LED亮度 0-100
    tare_calibration_trigger = false,    -- 去皮校准触发
    weight_calibration_trigger = false,  -- 重量校准触发
    calibration_weight = 0,              -- 校准重量值(克)
    k_calibration_trigger = false,       -- K值校准触发
    k_values = {0, 0, 0},               -- K值数组
    
    -- PUMP_CALIBRATION_CMD (507) 消息控制的变量
    pump_calibration_cmd = 0x00,         -- 水泵校准命令
    
    -- SPRAY_SYSTEM_PARAMS (509) 消息控制的变量
    spray_rate_from_gcs = 3000,          -- 地面站设置的喷洒速率 (mL/mu)
    spray_width_from_gcs = 600,          -- 地面站设置的喷洒宽度 (cm)
    
    -- SPREADER_CONTROL (511) 消息控制的变量
    spreader_motor_pwm = 1000,           -- 播撒器电机PWM
    spreader_valve_pwm = 1000,           -- 播撒器阀门PWM
    spreader_signal_source = 0x00,       -- 信号源 (0: PWM, 1: CANBUS)
    spreader_alarm_config = {0x01, 0x01, 0x00}, -- 报警配置
    spreader_factory_reset = false,      -- 出厂重置触发
    
    -- 消息接收时间戳 - 用于判断数据新鲜度
    last_505_time = 0,
    last_507_time = 0,
    last_509_time = 0,
    last_511_time = 0,
}

-- 传感器数据存储
local sensor_data = {
    weight = 0,                         -- 重量(克)
    last_weight_update_time = 0,        -- 上次更新时间戳
}

---------------------------------------------------------------------
-- 初始化
---------------------------------------------------------------------

-- 1. 加载MAVLink库和消息定义 (DRY原则重构)
local mavlink_msgs = require("MAVLink/mavlink_msgs")

-- MAVLink消息配置表 (按照eft.xml中的message ID顺序 501-515) fuck一下方便查找
--增加message需要更新xml，重新编译ardupilot固件并烧录，生成lua文件放到scripts/modules/MAVLink/mavlink_msg_xxx.lua,并在下面加上新message!
local mavlink_msg_configs = {
    {501, "DEVICE_STATUS_ARRAY",           "设备状态",         137, true},
    {502, "DEVICE_INFO1_ARRAY",            "设备信息1",        152, true},
    {503, "DEVICE_INFO2_ARRAY",            "设备信息2",        161, true},
    {504, "SINGLE_RADAR_DATA",             "单雷达数据",       79,  true},
    {505, "WEIGHT_CALIBRATION",            "LED控制重量校准",  203, false},
    {506, "WEIGH_DATA_EFT",                "重量数据",         132, false},
    {507, "PUMP_CALIBRATION_CMD",          "泵校准命令",       83,  false},
    {508, "PUMP_CALIBRATION_RESULTS",      "泵校准结果",       65,  true},
    {509, "SPRAY_SYSTEM_PARAMS",           "喷洒参数",         197, false},
    {510, "BATTERY_DATA",                  "电池数据",         29,  false},
    {511, "SPREADER_CONTROL",              "播撒控制",         113, false},
    {512, "SPREADER_STATUS",               "播撒状态",         167, true},
    {513, "SPREADER_CALIBRATION_RESULTS",  "播撒校准结果",     240, true},
    {514, "MULTI_RADAR_DATA",              "多雷达数据",       62,  true},
    {515, "FMU_PMU_UART_MESSAGE",          "FMU-PMU通信",      66,  false},
}

-- 自动加载消息定义并建立查找表
local MAVLINK_DEFS = {}
local ids_to_forward = {}
local CRC_EXTRA_TABLE = {}

for _, config in ipairs(mavlink_msg_configs) do
    local msg_id, msg_name, description, crc_extra, forward = config[1], config[2], config[3], config[4], config[5]
    
    -- 动态加载消息定义
    local msg_def = require("MAVLink/mavlink_msg_" .. msg_name)
    msg_def.name = msg_name
    
    -- 建立查找表
    MAVLINK_DEFS[msg_id] = msg_def
    CRC_EXTRA_TABLE[msg_id] = crc_extra
    
    -- 设置转发标志
    if forward then
        ids_to_forward[msg_id] = true
    end
end

-- 2. 初始化串口
local port = serial:find_serial(parameters.SERIAL_PORT_NUM)
if not port then gcs:send_text(6, "ERR: SERIAL port not found") return end
port:begin(parameters.BAUD_RATE)
port:set_flow_control(0)

-- 3. 状态变量
local last_send_time = 0
local yaw_ok_state = true
local yaw_rate_filt_deg = 0.0
local last_calib_pump_state = false
local last_calib_spreader_state = false
local mavlink_tx_seq = 0
local last_display_time = 0

---------------------------------------------------------------------
-- 核心功能函数
---------------------------------------------------------------------

-- CRC计算 (内置)
local function generate_mavlink_crc(buffer)
    local crc = 0xFFFF
    for i = 1, #buffer do
        local tmp = string.byte(buffer, i, i) ~ (crc & 0xFF)
        tmp = (tmp ~ (tmp << 4)) & 0xFF
        crc = (crc >> 8) ~ (tmp << 8) ~ (tmp << 3) ~ (tmp >> 4)
        crc = crc & 0xFFFF
    end
    return crc
end

-- MAVLink发送逻辑
local function send_mavlink_packet(msg_def, payload)
    local msg_id = msg_def.id
    local payload_encoded

    -- 检查是否为需要手动排序的特殊消息 (FMU_PMU_UART_MESSAGE, ID 515)
    if msg_id == 515 then
        -- 手动按XML规定顺序打包payload，使用大端模式
        payload_encoded = string.pack(">I2", payload.pump_control) ..
                        string.pack(">I4", payload.nozzle_control) ..
                        string.pack(">B", payload.control_mode) ..
                        string.pack(">I2", payload.horizontal_speed) ..
                        string.pack(">I2", payload.spray_rate) ..
                        string.pack(">I2", payload.spray_width) ..
                        string.pack(">B", payload.pump_calibration_cmd) ..
                        string.pack(">B", payload.led_control_cmd) ..
                        string.pack(">B", payload.led_brightness_right) ..
                        string.pack(">B", payload.led_brightness_left) ..
                        string.pack(">B", payload.tare_calibration_cmd) ..
                        string.pack(">B", payload.weight_calibration_cmd) ..
                        string.pack(">I2", payload.calibration_weight) ..
                        string.pack(">B", payload.k_value_calibration_cmd) ..
                        string.pack(">I2I2I2", table.unpack(payload.k_values)) ..
                        string.pack(">B", payload.spreader_control_cmd) ..
                        string.pack(">I2", payload.spreader_motor_pwm) ..
                        string.pack(">I2", payload.spreader_valve_pwm) ..
                        string.pack(">B", payload.signal_source_cmd) ..
                        string.pack(">B", payload.signal_source) ..
                        string.pack(">B", payload.alarm_config_cmd) ..
                        string.pack(">BBB", table.unpack(payload.alarm_config)) ..
                        string.pack(">B", payload.factory_reset_cmd) ..
                        string.pack(">B", payload.spray_spreader_mode)
    else
        -- 对所有其他消息，使用标准的MAVLink库进行编码
        local _, temp_payload = mavlink_msgs.encode(msg_def.name, payload)
        payload_encoded = temp_payload
    end

    local payload_len = #payload_encoded
    local header_fields = { payload_len, 0, 0, mavlink_tx_seq, parameters.MAV_SYSTEM_ID, parameters.MAV_COMPONENT_ID, msg_id & 0xFF, (msg_id >> 8) & 0xFF, (msg_id >> 16) & 0xFF }
    local header_packed = string.pack("<BBBBBBBBB", table.unpack(header_fields))

    -- 正确的两步CRC计算（用于发送）
    -- 1. 计算基础CRC
    local crc_data = header_packed .. payload_encoded
    local checksum = generate_mavlink_crc(crc_data)

    -- 2. 累加crc_extra
    local crc_extra = CRC_EXTRA_TABLE[msg_id] or 0
    local tmp = crc_extra ~ (checksum & 0xFF)
    tmp = (tmp ~ (tmp << 4)) & 0xFF
    checksum = (checksum >> 8) ~ (tmp << 8) ~ (tmp << 3) ~ (tmp >> 4)
    checksum = checksum & 0xFFFF

    local final_packet = string.char(0xFD) .. header_packed .. payload_encoded .. string.pack("<H", checksum)
    for i = 1, #final_packet do port:write(final_packet:byte(i)) end
    mavlink_tx_seq = (mavlink_tx_seq + 1) % 256
end

-- MAVLink字节流解析状态机
local rx_state = {
    step = 0, buffer = {}, header = {}, payload = {},
    len = 0, crc = 0, msgid = 0
}
local MAV_STX = 0xFD
local RX_STEP = { STX=0, LEN=1, HDR=2, PAY=3, CK=4 }

-- 自包含的Payload解码函数
local function decode_mavlink_payload(msgid, payload_str)
    local msg_def = MAVLINK_DEFS[msgid]
    if not msg_def then return nil end

    local decoded_msg = {}
    local offset = 1
    for _, field_info in ipairs(msg_def.fields) do
        local field_name = field_info[1]
        local field_format = field_info[2]
        local array_len = field_info[3]

        if array_len then
            decoded_msg[field_name] = {}
            for i=1, array_len do
                local value
                value, offset = string.unpack(field_format, payload_str, offset)
                table.insert(decoded_msg[field_name], value)
            end
        else
            local value
            value, offset = string.unpack(field_format, payload_str, offset)
            decoded_msg[field_name] = value
        end
    end
    return decoded_msg
end

-- 处理地面站控制消息
local function handle_gcs_control_message(msgid, decoded_msg)
    local current_time = millis():toint()
    
    if msgid == 505 then  -- WEIGHT_CALIBRATION
        gcs_control.led_control_enable = decoded_msg.led_control == 1
        gcs_control.led_brightness_right = decoded_msg.right_led_brightness or 0
        gcs_control.led_brightness_left = decoded_msg.left_led_brightness or 0
        gcs_control.tare_calibration_trigger = decoded_msg.tare_calibration == 1
        gcs_control.weight_calibration_trigger = decoded_msg.weight_calibration == 1
        gcs_control.calibration_weight = decoded_msg.calibration_weight or 0
        gcs_control.k_calibration_trigger = decoded_msg.k_calibration == 1
        if decoded_msg.k_values and type(decoded_msg.k_values) == "table" then
            gcs_control.k_values = {decoded_msg.k_values[1] or 0, decoded_msg.k_values[2] or 0, decoded_msg.k_values[3] or 0}
        end
        gcs_control.last_505_time = current_time
        gcs:send_text(6, string.format("GCS: LED update R:%d L:%d", gcs_control.led_brightness_right, gcs_control.led_brightness_left))
        
    elseif msgid == 507 then  -- PUMP_CALIBRATION_CMD
        gcs_control.pump_calibration_cmd = decoded_msg.calibration_cmd or 0x00
        gcs_control.last_507_time = current_time
        gcs:send_text(6, string.format("GCS: Calib cmd 0x%02X", gcs_control.pump_calibration_cmd))
        
    elseif msgid == 509 then  -- SPRAY_SYSTEM_PARAMS
        gcs_control.spray_rate_from_gcs = decoded_msg.spray_rate or 3000
        gcs_control.spray_width_from_gcs = decoded_msg.spray_width or 600
        gcs_control.last_509_time = current_time
        gcs:send_text(6, string.format("GCS: Spray rate:%d width:%d", gcs_control.spray_rate_from_gcs, gcs_control.spray_width_from_gcs))
        
    elseif msgid == 511 then  -- SPREADER_CONTROL
        if decoded_msg.motor_control_cmd == 0xF1 then
            gcs_control.spreader_motor_pwm = decoded_msg.spreader_motor_pwm or 1000
            gcs_control.spreader_valve_pwm = decoded_msg.spreader_valve_pwm or 1000
        end
        if decoded_msg.signal_source_cmd == 0xF2 then
            gcs_control.spreader_signal_source = decoded_msg.spreader_signal_source or 0x00
        end
        if decoded_msg.alarm_config_cmd == 0xF3 then
            if decoded_msg.spreader_alarm_config and type(decoded_msg.spreader_alarm_config) == "table" then
                gcs_control.spreader_alarm_config = {
                    decoded_msg.spreader_alarm_config[1] or 0x01,
                    decoded_msg.spreader_alarm_config[2] or 0x01,
                    decoded_msg.spreader_alarm_config[3] or 0x00
                }
            end
        end
        if decoded_msg.spreader_factory_reset == 0xF5 then
            gcs_control.spreader_factory_reset = true
        end
        gcs_control.last_511_time = current_time
        gcs:send_text(6, string.format("GCS: Spreader mot:%d val:%d", gcs_control.spreader_motor_pwm, gcs_control.spreader_valve_pwm))
    end
end

-- 成功解析MAVLink消息后的处理函数
local function process_mavlink_payload(msgid, payload_str)
    local msg_def = MAVLINK_DEFS[msgid]
    if not msg_def then return end

    if ids_to_forward[msgid] then
        -- 同时向多个MAVLink通道转发原始payload
        for i=1,#FWD_CHANS do
            pcall(mavlink.send_chan, FWD_CHANS[i], msgid, payload_str)
        end
        return
    end

    -- 只有不转发的消息，才需要解码并进行后续处理
    local decoded_msg = decode_mavlink_payload(msgid, payload_str)
    if not decoded_msg then return end

    -- 根据消息ID分发到不同的处理函数
    if msgid == 510 then  -- BATTERY_DATA消息ID
        local state = BattMonitorScript_State()
        state:healthy(true)
        state:voltage(decoded_msg.voltage * 0.01)
        state:cell_count(parameters.CELL_COUNT)
        state:capacity_remaining_pct(math.floor(decoded_msg.capacity_percent * 0.1))
        state:current_amps(-decoded_msg.current * 0.001)
        state:temperature(decoded_msg.cell_temp * 0.1)
        battery:handle_scripting(parameters.BATTERY_INSTANCE, state)

    elseif msgid == 506 then  -- 处理WEIGH_DATA_EFT称重消息
        -- 1. 手动将原始消息转发到所有GCS通道，保持原有功能
        for i=1,#FWD_CHANS do
            pcall(mavlink.send_chan, FWD_CHANS[i], msgid, payload_str)
        end
        
        -- 2. 解析并存储重量数据
        local raw_weight = decoded_msg.weight or 0
        
        -- (raw_weight & 0xFF)为低字节, ((raw_weight >> 8) & 0xFF)为高字节。
        local corrected_weight = ((raw_weight & 0x00FF) << 8) | ((raw_weight >> 8) & 0xFF)
        
        sensor_data.weight = corrected_weight
        sensor_data.last_weight_update_time = millis():toint()

    elseif msgid == 505 or msgid == 507 or msgid == 509 or msgid == 511 then
        -- 处理地面站控制消息
        handle_gcs_control_message(msgid, decoded_msg)
    end
end

-- MAVLink解析状态机核心
local function parse_mavlink_byte(byte)
    if rx_state.step == RX_STEP.STX then
        if byte == MAV_STX then
            rx_state.step = RX_STEP.LEN
            rx_state.buffer = { byte }
        end
    elseif rx_state.step == RX_STEP.LEN then
        rx_state.len = byte
        table.insert(rx_state.buffer, byte)
        rx_state.header = {}
        rx_state.step = RX_STEP.HDR
    elseif rx_state.step == RX_STEP.HDR then
        table.insert(rx_state.header, byte)
        table.insert(rx_state.buffer, byte)
        if #rx_state.header == 8 then
            rx_state.msgid = rx_state.header[6] + (rx_state.header[7] << 8) + (rx_state.header[8] << 16)
            rx_state.payload = {}
            if rx_state.len > 0 then
                rx_state.step = RX_STEP.PAY
            else
                rx_state.step = RX_STEP.CK
            end
        end
    elseif rx_state.step == RX_STEP.PAY then
        table.insert(rx_state.payload, byte)
        table.insert(rx_state.buffer, byte)
        if #rx_state.payload == rx_state.len then
            rx_state.step = RX_STEP.CK
        end
    elseif rx_state.step == RX_STEP.CK then
        table.insert(rx_state.buffer, byte)
        if #rx_state.buffer == (10 + rx_state.len + 2) then -- 10字节头 + payload + 2字节CRC
            -- 校验CRC
            local msg_def = MAVLINK_DEFS[rx_state.msgid]
            if msg_def then
                -- 1. 计算消息体的基础CRC
                local crc_data_str = string.char(table.unpack(rx_state.buffer, 2, 10 + rx_state.len))
                local calculated_crc = generate_mavlink_crc(crc_data_str)
                
                -- 2. 累加crc_extra，与C代码完全一致
                local crc_extra = CRC_EXTRA_TABLE[rx_state.msgid] or 0
                local tmp = crc_extra ~ (calculated_crc & 0xFF)
                tmp = (tmp ~ (tmp << 4)) & 0xFF
                calculated_crc = (calculated_crc >> 8) ~ (tmp << 8) ~ (tmp << 3) ~ (tmp >> 4)
                calculated_crc = calculated_crc & 0xFFFF

                local received_crc = rx_state.buffer[10 + rx_state.len + 1] + (rx_state.buffer[10 + rx_state.len + 2] << 8)
                
                if calculated_crc == received_crc then
                    process_mavlink_payload(rx_state.msgid, string.char(table.unpack(rx_state.payload)))
                end
            end
            rx_state.step = RX_STEP.STX -- 完成或失败都重置状态机
        end
    end
end

-- 更新偏航速率保护状态
local function update_yaw_rate_protection()
    local gyro_vec = ahrs:get_gyro()
    if not gyro_vec then return end
    
    local yaw_rate_rad = gyro_vec:z() -- 获取Z轴角速度
    local yaw_rate_deg = math.abs(math.deg(yaw_rate_rad))
    
    -- 一阶低通滤波
    yaw_rate_filt_deg = yaw_rate_filt_deg + parameters.YAW_FILTER_ALPHA * (yaw_rate_deg - yaw_rate_filt_deg)
    
    -- 迟滞判断
    if yaw_rate_filt_deg < parameters.YAW_RATE_ON_THRESH then
        yaw_ok_state = true
    elseif yaw_rate_filt_deg > parameters.YAW_RATE_OFF_THRESH then
        yaw_ok_state = false
    end
end

-- 获取水平速度 (cm/s)
local function get_horizontal_speed_cm()
    -- 优先使用GPS速度
    local success, gps_speed = pcall(gps.ground_speed, gps, 0)
    if success and gps_speed and type(gps_speed) == "number" and gps_speed > 0 then
        return math.floor(gps_speed * 100) -- m/s转换为cm/s
    end
    
    -- 备选方案：使用AHRS速度向量
    local vel = ahrs:get_velocity_NED()
    if vel then
        local speed_ms = math.sqrt(vel:x()^2 + vel:y()^2)
        return math.floor(speed_ms * 100)
    end
    
    return 0
end

-- 检查是否在有效航线喷洒状态
local function is_in_valid_mission_spraying()
    -- 1. 检查是否在AUTO模式
    local current_mode = vehicle:get_mode()
    if current_mode ~= 3 then  -- 3 = AUTO模式
        return false
    end
    
    -- 2. 检查任务状态是否为RUNNING
    local mission_state = mission:state()
    if mission_state ~= mission.MISSION_RUNNING then
        return false
    end
    
    -- 3. 获取当前任务航点索引
    local current_nav_index = mission:get_current_nav_index()
    if current_nav_index <= 2 then  -- 索引0是HOME点，索引1通常是起飞点
        return false  -- 前往第一个航点时不喷洒
    end
    
    -- 4. 获取总航点数
    local total_commands = mission:num_commands()
    if current_nav_index >= total_commands - 1 then  -- 最后一个航点通常是返航点
        return false  -- 返回起飞点时不喷洒
    end
    
    -- 5. 检查当前航点是否有位置信息（导航航点）
    local current_nav_id = mission:get_current_nav_id()
    if not mission:cmd_has_location(current_nav_id) then
        return false  -- 如果当前命令没有位置信息，不是导航航点
    end
    
    return true  -- 满足所有条件，允许喷洒
end

-- PMU控制逻辑 - 融合RC和地面站控制
local function send_pump_control_packet()
    -- 更新偏航速率保护状态
    update_yaw_rate_protection()
    
    -- 检查航线喷洒状态
    local mission_spray_allowed = is_in_valid_mission_spraying()
    
    -- 读取遥控器通道 (按新规范: >50开，<50关)
    local rc_calib_pump_pwm = rc:get_pwm(parameters.CALIBRATION_PUMP_CHAN) or 1000
    local rc_calib_spreader_pwm = rc:get_pwm(parameters.CALIBRATION_SPREADER_CHAN) or 1000
    local rc_spray_mode_pwm = rc:get_pwm(parameters.SPRAY_MODE_SWITCH_CHAN) or 1000
    local rc_led_control_pwm = rc:get_pwm(parameters.LED_CONTROL_CHAN) or 1000
    
    -- 将PWM转换为百分比 (1000-2000 -> 0-100)
    local function pwm_to_percent(pwm)
        return math.max(0, math.min(100, (pwm - 1000) / 10))
    end
    
    local calib_pump_percent = pwm_to_percent(rc_calib_pump_pwm)
    local calib_spreader_percent = pwm_to_percent(rc_calib_spreader_pwm)
    local spray_mode_percent = pwm_to_percent(rc_spray_mode_pwm)
    
    -- 控制开关状态 (>50开，<50关)
    local calib_pump_on = calib_pump_percent > 50
    local calib_spreader_on = calib_spreader_percent > 50
    local spray_mode = spray_mode_percent > 50 and 1 or 0 -- 0:喷洒模式, 1:播撒模式
    
    -- 上升沿检测
    local calib_pump_edge = calib_pump_on and not last_calib_pump_state
    local calib_spreader_edge = calib_spreader_on and not last_calib_spreader_state
    last_calib_pump_state = calib_pump_on
    last_calib_spreader_state = calib_spreader_on
    
    -- 1. 水泵控制 (2字节) - 大端编码，只在航线喷洒时启动
    local pump_control = (yaw_ok_state and mission_spray_allowed) and PUMP_ON_VALUE or 0
    
    -- 2. 喷头控制 (4字节) - 大端编码，只在航线喷洒时启动
    local nozzle_control = (mission_spray_allowed) and NOZZLE_ON_VALUE or 0
    
    -- 3. 控制模式 (1字节) - PWM模式
    local control_mode = 0x00
    
    -- 4. 水平速度 (2字节) - 大端自动处理
    local horizontal_speed = get_horizontal_speed_cm()
    
    -- 5. 喷洒参数 - 优先使用地面站设置的值
    local spray_rate = gcs_control.spray_rate_from_gcs    -- 使用地面站设置的喷洒速率
    local spray_width = gcs_control.spray_width_from_gcs  -- 使用地面站设置的喷洒宽度
    
    -- 6. 水泵校准 - 融合RC和地面站控制
    local pump_calibration_cmd = 0x00
    if gcs_control.pump_calibration_cmd ~= 0x00 then
        -- 优先使用地面站命令
        pump_calibration_cmd = gcs_control.pump_calibration_cmd
        gcs_control.pump_calibration_cmd = 0x00  -- 使用后清除
    elseif calib_pump_edge then
        pump_calibration_cmd = PUMP_CALIBRATION_CMD
    elseif calib_spreader_edge then
        pump_calibration_cmd = SPREADER_CALIBRATION_CMD
    end
    
    -- 7. LED控制 - 根据通道6控制LED开关
    -- 通道6大于1500为开启LED，小于1500为关闭LED
    local led_enabled = rc_led_control_pwm > 1500
    local led_control_cmd = 0xE1
    local led_brightness_right = led_enabled and 40 or 0
    local led_brightness_left = led_enabled and 40 or 0
    
    -- 8. 去皮校准 - 使用地面站触发
    local tare_calibration_cmd = gcs_control.tare_calibration_trigger and 0xF6 or 0x00
    if gcs_control.tare_calibration_trigger then
        gcs_control.tare_calibration_trigger = false  -- 触发后清除
    end
    
    -- 9. 重量校准 - 使用地面站设置的值
    local weight_calibration_cmd = gcs_control.weight_calibration_trigger and 0xF7 or 0x00
    local calibration_weight = gcs_control.calibration_weight
    if gcs_control.weight_calibration_trigger then
        gcs_control.weight_calibration_trigger = false  -- 触发后清除
    end
    
    -- 10. K值校准 - 使用地面站设置的值
    local k_value_calibration_cmd = gcs_control.k_calibration_trigger and 0xFC or 0x00
    local k_values = {gcs_control.k_values[1], gcs_control.k_values[2], gcs_control.k_values[3]}
    if gcs_control.k_calibration_trigger then
        gcs_control.k_calibration_trigger = false  -- 触发后清除
    end
    
    -- 11. 撒播器控制 - 使用地面站设置的PWM值
    local spreader_control_cmd = 0xF1  -- 电机控制命令标识
    local spreader_motor_pwm = gcs_control.spreader_motor_pwm
    local spreader_valve_pwm = gcs_control.spreader_valve_pwm
    
    -- 12. 信号源选择 - 使用地面站设置的值
    local signal_source_cmd = 0xF2  -- 信号源命令标识
    local signal_source = gcs_control.spreader_signal_source
    
    -- 13. 报警配置 - 使用地面站设置的值
    local alarm_config_cmd = 0xF3  -- 报警配置命令标识
    local alarm_config = {gcs_control.spreader_alarm_config[1], gcs_control.spreader_alarm_config[2], gcs_control.spreader_alarm_config[3]}
    
    -- 14. 恢复出厂设置 - 使用地面站触发
    local factory_reset_cmd = gcs_control.spreader_factory_reset and 0xF5 or 0x00
    if gcs_control.spreader_factory_reset then
        gcs_control.spreader_factory_reset = false  -- 触发后清除
    end
    
    -- 15. 播撒喷洒模式切换 (1字节)
    local spray_spreader_mode = spray_mode
    
    local payload = {
        nozzle_control = nozzle_control,
        pump_control = pump_control,
        horizontal_speed = horizontal_speed,
        spray_rate = spray_rate,
        spray_width = spray_width,
        calibration_weight = calibration_weight,
        k_values = k_values,
        spreader_motor_pwm = spreader_motor_pwm,
        spreader_valve_pwm = spreader_valve_pwm,
        control_mode = control_mode,
        pump_calibration_cmd = pump_calibration_cmd,
        led_control_cmd = led_control_cmd,
        led_brightness_right = led_brightness_right,
        led_brightness_left = led_brightness_left,
        tare_calibration_cmd = tare_calibration_cmd,
        weight_calibration_cmd = weight_calibration_cmd,
        k_value_calibration_cmd = k_value_calibration_cmd,
        spreader_control_cmd = spreader_control_cmd,
        signal_source_cmd = signal_source_cmd,
        signal_source = signal_source,
        alarm_config_cmd = alarm_config_cmd,
        alarm_config = alarm_config,
        factory_reset_cmd = factory_reset_cmd,
        spray_spreader_mode = spray_spreader_mode
    }
    
    send_mavlink_packet(MAVLINK_DEFS[515], payload)  -- FMU_PMU_UART_MESSAGE
end

---------------------------------------------------------------------
-- 主更新循环
---------------------------------------------------------------------
function update()
    -- 1. 尽可能多地处理PMU发来的数据
    local byte
    while port:available() > 0 do
        byte = port:read()
        if byte then
            parse_mavlink_byte(byte)
        else
            break
        end
    end

    -- 2. 根据定时器发送控制指令给PMU
    local now = millis():toint()
    if now - last_send_time >= parameters.SEND_INTERVAL_MS then
        send_pump_control_packet()
        last_send_time = now
    end
    
    -- 3. 每10秒向地面站发送一次称重数据
    if now - last_display_time >= 10000 then
        if sensor_data.last_weight_update_time > 0 then
            gcs:send_text(6, string.format("WEIGHT: %.2fkg", sensor_data.weight * 0.001))
        end
        last_display_time = now
    end
    
    return update, parameters.LOOP_RATE_MS
end

-- 启动脚本
gcs:send_text(6, "SYS: PMU controller v8.1 started")
return update, 1000
