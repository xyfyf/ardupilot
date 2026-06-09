--[[
  UM982 接 UART（如 SERIAL7）时，经 Scripting 串口在 **115200** 下下发和芯文本配置（不修改 COM1 波特率）。

  流程：
  1) 可选 FRESET（RUN_FRESET）
  2) 依次发送 GPGGA/GPRMC/AGRICA/GPGSA/GPGST/UNIHEADINGA、saveconfig（均在 115200 下）

  重要：
  - serial:find_serial(N) 的 N 为全机 PROTOCOL=28 的 Scripting 口从 0 起的序号。

  使用前：
  1) 接 UM982 的 SERIALx_PROTOCOL = 28，SERIALx_BAUD = 115200（须与 UM982 COM1 当前波特率一致）
  2) SCR_ENABLE = 1；完成后改 PROTOCOL=5、BAUD 保持 115200（或与模块一致），GPS_TYPE=24 或 25，保存重启并移走本脚本

  参考：Holybro UM982
  https://docs.holybro.com/gps-and-rtk-system/h-rtk-unicore-um982/factory-setting-and-com-port-allocation
--]]

---@diagnostic disable: need-check-nil

-- Scripting 串口实例序号（从 0 起）
local SCRIPTING_INSTANCE = 0

-- 与 UM982 COM1 及 SERIALx_BAUD 保持一致（本脚本全程使用该波特率）
local BAUD_RATE = 115200

-- 是否先发 FRESET（会清模块侧配置；不需要则改为 false）
local RUN_FRESET = true

local CMD_INTERVAL_MS = 500
local POST_FRESET_MS = 2500

local SEV_INFO = 6
local SEV_WARN = 4

-- 在 115200 下发送的配置行（与需求一致；以 \r\n 结尾由 send_line 追加）
local cmds_body = {
    "GPGGA COM1 0.2",
    "GPRMC COM1 0.2",
    "AGRICA COM1 0.2",
    "GPGSA COM1 0.2",
    "GPGST COM1 0.2",
    "UNIHEADINGA COM1 0.2",
    "saveconfig",
}

-- 组装完整发送列表：可选 FRESET + 上表
local function build_cmds()
    local t = {}
    if RUN_FRESET then
        t[#t + 1] = "FRESET"
    end
    for i = 1, #cmds_body do
        t[#t + 1] = cmds_body[i]
    end
    return t
end

local cmds = build_cmds()

local port = nil
local step = 0
local last_send_ms = 0
local next_gap_ms = CMD_INTERVAL_MS

local function open_scripting_port()
    port = serial:find_serial(SCRIPTING_INSTANCE)
    if port == nil then
        gcs:send_text(SEV_WARN, string.format(
            "UM982: No Scripting inst %d",
            SCRIPTING_INSTANCE))
        return false
    end
    port:begin(BAUD_RATE)
    port:set_flow_control(0)
    gcs:send_text(SEV_INFO, string.format("UM982: Inst %d @%d", SCRIPTING_INSTANCE, BAUD_RATE))
    return true
end

local function send_line(line)
    -- Scripting 串口整段字符串用 writestring，勿用 write(字符串)
    port:writestring(line .. "\r\n")
    gcs:send_text(SEV_INFO, "UM982 >> " .. line)
end

function update()
    if port == nil then
        if not open_scripting_port() then
            return update, 3000
        end
        last_send_ms = millis() - math.max(next_gap_ms, 1)
    end

    local now = millis()
    if now - last_send_ms < next_gap_ms then
        return update, 50
    end

    step = step + 1
    if step > #cmds then
        gcs:send_text(SEV_INFO, "UM982: Config Done")
        return nil
    end

    local line = cmds[step]
    send_line(line)
    last_send_ms = now
    if line == "FRESET" then
        next_gap_ms = POST_FRESET_MS
    else
        next_gap_ms = CMD_INTERVAL_MS
    end

    return update, 50
end

return update()
