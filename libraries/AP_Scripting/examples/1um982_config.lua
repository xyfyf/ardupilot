-- UM982 自动配置脚本
-- 功能：
-- 1. 自动检测并配置连接的 UM982 模块到 ArduPilot 推荐状态
-- 2. 配置完成后，自动将串口参数修改为 GPS 模式 (Protocol=5, Baud=230400)
-- 3. 提示用户重启飞控
--
-- 使用方法：
-- 1. 将脚本放入飞控 SD 卡 APM/scripts 目录
-- 2. 设置 SCR_ENABLE = 1 并重启
-- 3. 设置连接 UM982 的串口 SERIALx_PROTOCOL = 28 (Scripting)
-- 4. 设置 SERIALx_BAUD = 115 (推荐初值，脚本会自动适配)
-- 5. 重启飞控，观察消息栏提示
-- 6. 脚本提示“配置完成，参数已更新”后，手动断电重启即可

local uart = serial:find_serial(0)
local step = 0
local last_step_time = 0
local wait_delay = 1000
local target_serial_idx = -1 -- 自动识别的串口号

-- 查找当前脚本使用的串口号 (通过遍历参数 SERIALx_PROTOCOL == 28)
function find_scripting_serial()
    -- 遍历 SERIAL0 到 SERIAL8
    for i = 0, 8 do
        local param_name = "SERIAL" .. i .. "_PROTOCOL"
        local protocol = param:get(param_name)
        if protocol == 28 then
            gcs:send_text(6, "UM982: 自动识别到目标串口: SERIAL" .. i)
            return i
        end
    end
    return -1
end

-- 辅助函数：发送字符串命令
function send_cmd(cmd)
    if not uart then return end
    -- gcs:send_text(6, "UM982: 发送 " .. cmd) -- 调试用，平时可注释
    for i = 1, #cmd do
        uart:write(string.byte(cmd, i))
    end
    uart:write(13) -- \r
    uart:write(10) -- \n
end

function update()
    local now = millis()
    if now - last_step_time < wait_delay then
        return update, 100
    end
    
    if not uart then
        gcs:send_text(0, "UM982: 未找到 Scripting 串口 (请设置 SERIALx_PROTOCOL=28)")
        return update, 5000
    end

    last_step_time = now
    step = step + 1

    -- 状态机流程
    if step == 1 then
        target_serial_idx = find_scripting_serial()
        if target_serial_idx == -1 then
            gcs:send_text(0, "UM982: 错误 - 未找到 Protocol=28 的串口参数")
            return update, 5000
        end
        gcs:send_text(5, "UM982: 脚本启动，准备配置模块...")
        
        -- 尝试用 230400 发送重置（应对之前已经配置过的情况）
        uart:begin(230400)
        send_cmd("FRESET")
        wait_delay = 500
    
    elseif step == 2 then
        -- 尝试用 115200 发送重置（应对出厂默认情况）
        uart:begin(115200)
        send_cmd("FRESET")
        gcs:send_text(5, "UM982: 已发送重置命令，等待重启...")
        wait_delay = 5000 -- 重启需要较长时间
        
    elseif step == 3 then
        -- 重启后，模块应该处于 115200 波特率
        uart:begin(115200)
        send_cmd("GPGGA COM1 0.2")
        wait_delay = 200
        
    elseif step == 4 then
        send_cmd("GPRMC COM1 0.2")
        wait_delay = 200
        
    elseif step == 5 then
        send_cmd("AGRICA COM1 0.2")
        wait_delay = 200
        
    elseif step == 6 then
        send_cmd("GPGSA COM1 0.2")
        wait_delay = 200
        
    elseif step == 7 then
        send_cmd("GPGST COM1 0.2")
        wait_delay = 200
        
    elseif step == 8 then
        send_cmd("UNIHEADINGA COM1 0.2")
        wait_delay = 200
        
    elseif step == 9 then
        gcs:send_text(5, "UM982: 配置波特率为 230400...")
        send_cmd("config com1 230400")
        wait_delay = 1000 -- 等待命令生效
        
    elseif step == 10 then
        -- 切换脚本使用的波特率以匹配模块
        uart:begin(230400)
        -- 发送保存配置
        send_cmd("saveconfig")
        gcs:send_text(5, "UM982: 发送保存配置命令")
        wait_delay = 2000
        
    elseif step == 11 then
        gcs:send_text(2, "UM982: 配置完成！正在修改飞控参数...")
        
        if target_serial_idx ~= -1 then
            -- 修改协议为 GPS (5)
            local proto_param = "SERIAL" .. target_serial_idx .. "_PROTOCOL"
            if param:set_and_save(proto_param, 5) then
                gcs:send_text(6, "UM982: 参数 " .. proto_param .. " 已设为 5 (GPS)")
            else
                gcs:send_text(0, "UM982: 错误 - 无法修改参数 " .. proto_param)
            end
            
            -- 修改波特率为 230 (230400)
            local baud_param = "SERIAL" .. target_serial_idx .. "_BAUD"
            if param:set_and_save(baud_param, 230) then
                 gcs:send_text(6, "UM982: 参数 " .. baud_param .. " 已设为 230 (230400)")
            else
                 gcs:send_text(0, "UM982: 错误 - 无法修改参数 " .. baud_param)
            end
            
            -- 也可以尝试设置 GPS_TYPE = 21 (Unicore)
            if param:get("GPS_TYPE") == 0 then
                 param:set_and_save("GPS_TYPE", 21)
                 gcs:send_text(6, "UM982: 参数 GPS_TYPE 已设为 21 (Unicore)")
            end

            wait_delay = 2000
        else
            gcs:send_text(0, "UM982: 错误 - 无法确定目标串口号")
        end
        
    elseif step == 12 then
        gcs:send_text(0, "===== UM982 配置流程结束 =====")
        gcs:send_text(0, "请立即断电重启飞控以启用 GPS！")
        return -- 结束运行
    end

    return update, 100
end

return update()
