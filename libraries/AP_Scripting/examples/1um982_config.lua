--[[
  UM982 生产工装脚本 — 接 UART（如 SERIAL7）经 Scripting 串口下发和芯文本配置。

  灯语（WS2812，SERVOx_FUNCTION=94，独立于 ws2812_fc_status_led.lua，勿同时加载）:
    红灯常亮 — 未识别到 RTK / 多次校验仍失败
    蓝灯常亮 — 正在配置或等待校验
    绿灯常亮 — 配置完成且持续收到 GPGGA + UNIHEADINGA

  波特率：探测只轮询 230400 / 115200；目标 COM1 = 230400（Holybro 出厂推荐）

  使用前：
  1) SERIALx_PROTOCOL = 28，SERIALx_BAUD = 230（230400）
  2) SERVOy_FUNCTION = 94（Scripting1），SCR_ENABLE = 1
  3) 产线完成后改 PROTOCOL=5、GPS_TYPE=24/25，移走本脚本

  参考：Holybro UM982
  https://docs.holybro.com/gps-and-rtk-system/h-rtk-unicore-um982/factory-setting-and-com-port-allocation
--]]

---@diagnostic disable: need-check-nil

-- ========================= Scripting 串口 =========================
local SCRIPTING_INSTANCE = 0
local TARGET_BAUD = 230400
local FRESET_DEFAULT_BAUD = 115200
local BAUD_CANDIDATES = {230400, 115200}
local RUN_FRESET = true
local CMD_INTERVAL_MS = 500
local POST_FRESET_MS = 5000
local POST_BAUD_SWITCH_MS = 800
local POST_SAVE_MS = 5000          -- saveconfig 后模块重启，需更长时间
local VERIFY_TIMEOUT_MS = 45000    -- 重启后搜星 + 双天线航向，室内可能较慢
local VERIFY_RETRY_MAX = 3         -- 校验失败只重试等待，不重新 FRESET

-- ========================= WS2812 状态灯 =========================
local LED_SERVO_FUNCTION = 94
local NUM_LEDS = 8
local BRIGHTNESS = 255
local UPDATE_MS = 50

-- ========================= 探测 =========================
local DETECT_PROBE_MS = 2000
local DETECT_BAUD_TRY_MS = 2500
local LINK_LOST_MS = 5000

local SEV_INFO = 6
local SEV_WARN = 4

local CONFIG_BODY = {
    "GPGGA COM1 0.2",
    "GPRMC COM1 0.2",
    "AGRICA COM1 0.2",
    "GPGSA COM1 0.2",
    "GPGST COM1 0.2",
    "UNIHEADINGA COM1 0.2",
    "config com1 " .. tostring(TARGET_BAUD),
    "saveconfig",
}

local function build_cmds(include_freset)
    local t = {}
    if include_freset and RUN_FRESET then
        t[#t + 1] = "FRESET"
    end
    for i = 1, #CONFIG_BODY do
        t[#t + 1] = CONFIG_BODY[i]
    end
    return t
end

local LED_NO_RTK = 1
local LED_CONFIG = 2
local LED_OK = 3

local PHASE_DETECT = 2
local PHASE_CONFIG = 3
local PHASE_VERIFY = 4
local PHASE_PASS = 5

local port = nil
local phase = PHASE_DETECT
local led_state = LED_NO_RTK
local active_baud = nil

local led_chan = nil
local led_init_ok = false

local cmds = build_cmds(true)
local config_step = 0
local last_send_ms = 0
local next_gap_ms = CMD_INTERVAL_MS
local config_sent = false           -- 本轮是否已完整下发过配置
local freset_sent = false           -- 本会话是否已发过 FRESET
local verify_retry = 0

local rx_buf = ""
local RX_BUF_MAX = 8192

local detect_baud_idx = 1
local detect_last_probe_ms = 0
local detect_last_rx_ms = 0

local verify_deadline_ms = 0
local seen_gpgga = false
local seen_heading = false
local last_gpgga_ms = 0
local last_heading_ms = 0

local function set_led(r, g, b)
    if not led_init_ok then
        return
    end
    local s = BRIGHTNESS / 255.0
    r = math.floor(r * s + 0.5)
    g = math.floor(g * s + 0.5)
    b = math.floor(b * s + 0.5)
    serialLED:set_RGB(led_chan, -1, r, g, b)
    serialLED:send(led_chan)
end

local function apply_led_state()
    if led_state == LED_NO_RTK then
        set_led(255, 0, 0)
    elseif led_state == LED_CONFIG then
        set_led(0, 0, 255)
    else
        set_led(0, 255, 0)
    end
end

local function init_led()
    if led_init_ok then
        return true
    end
    local ch0 = SRV_Channels:find_channel(LED_SERVO_FUNCTION)
    if ch0 == nil then
        return false
    end
    led_chan = ch0 + 1
    if not serialLED:set_num_neopixel(led_chan, NUM_LEDS) then
        return false
    end
    led_init_ok = true
    return true
end

local function available_count()
    local n = port:available()
    if type(n) == "userdata" then
        return n:toint()
    end
    return n or 0
end

local function read_port_all()
    local n = available_count()
    if n <= 0 then
        return ""
    end
    if port.readstring then
        return port:readstring(n)
    end
    local ret = ""
    for _ = 1, n do
        ret = ret .. string.char(port:read())
    end
    return ret
end

local function mark_from_text(text, now)
    if text == nil or text == "" then
        return
    end
    if string.find(text, "$GPGGA", 1, true) or string.find(text, "$GNGGA", 1, true) then
        seen_gpgga = true
        last_gpgga_ms = now
    end
    if string.find(text, "UNIHEADINGA", 1, true) or string.find(text, "#UNIHEADINGA", 1, true) then
        seen_heading = true
        last_heading_ms = now
    end
end

local function process_new_rx(data, now)
    if data == nil or data == "" then
        return
    end
    rx_buf = rx_buf .. data
    if #rx_buf > RX_BUF_MAX then
        rx_buf = string.sub(rx_buf, #rx_buf - RX_BUF_MAX + 1)
    end
    mark_from_text(data, now)
end

local function rx_has_rtk_marker()
    if string.find(rx_buf, "982", 1, true) then
        return true
    end
    if string.find(rx_buf, "UNICORE", 1, true) then
        return true
    end
    if string.find(rx_buf, "VERSION", 1, true) then
        return true
    end
    if string.find(rx_buf, "$GPGGA", 1, true) or string.find(rx_buf, "$GNGGA", 1, true) then
        return true
    end
    if string.find(rx_buf, "#AGRICA", 1, true) then
        return true
    end
    if string.find(rx_buf, "UNIHEADINGA", 1, true) then
        return true
    end
    return false
end

local function verify_ok()
    return seen_gpgga and seen_heading
end

local function pass_link_ok(now)
    return (now - last_gpgga_ms < LINK_LOST_MS) and (now - last_heading_ms < LINK_LOST_MS)
end

local function set_port_baud(baud)
    port:begin(baud)
    port:set_flow_control(0)
    active_baud = baud
end

local function open_scripting_port()
    port = serial:find_serial(SCRIPTING_INSTANCE)
    if port == nil then
        gcs:send_text(SEV_WARN, string.format(
            "UM982工装: 无 Scripting 口 inst %d",
            SCRIPTING_INSTANCE))
        return false
    end
    set_port_baud(BAUD_CANDIDATES[detect_baud_idx])
    gcs:send_text(SEV_INFO, string.format(
        "UM982工装: 口 %d 探测 @%d",
        SCRIPTING_INSTANCE, active_baud))
    return true
end

local function send_line(line)
    port:writestring(line .. "\r\n")
    gcs:send_text(SEV_INFO, "UM982 >> " .. line)
end

local function send_detect_probe(now)
    if now - detect_last_probe_ms < DETECT_PROBE_MS then
        return
    end
    detect_last_probe_ms = now
    port:writestring("log version\r\n")
end

local function switch_detect_baud(now)
    detect_baud_idx = detect_baud_idx + 1
    if detect_baud_idx > #BAUD_CANDIDATES then
        detect_baud_idx = 1
    end
    set_port_baud(BAUD_CANDIDATES[detect_baud_idx])
    rx_buf = ""
    detect_last_probe_ms = 0
    detect_last_rx_ms = now
    gcs:send_text(SEV_WARN, string.format("UM982工装: 尝试 %d", active_baud))
end

local function begin_detect(now)
    phase = PHASE_DETECT
    led_state = LED_NO_RTK
    rx_buf = ""
    detect_baud_idx = 1
    detect_last_probe_ms = 0
    detect_last_rx_ms = now
    if port ~= nil then
        set_port_baud(BAUD_CANDIDATES[detect_baud_idx])
    end
    send_detect_probe(now)
    gcs:send_text(SEV_INFO, "UM982工装: 探测 RTK...")
end

local function begin_config(now, skip_freset)
    phase = PHASE_CONFIG
    led_state = LED_CONFIG
    local use_freset = RUN_FRESET and not skip_freset and not freset_sent
    cmds = build_cmds(use_freset)
    config_step = 0
    last_send_ms = now - math.max(next_gap_ms, 1)
    rx_buf = ""
    seen_gpgga = false
    seen_heading = false
    last_gpgga_ms = 0
    last_heading_ms = 0
    gcs:send_text(SEV_INFO, string.format(
        "UM982工装: 开始配置 @%d%s",
        active_baud or 0, use_freset and " (含FRESET)" or ""))
end

local function begin_verify(now, is_retry)
    phase = PHASE_VERIFY
    led_state = LED_CONFIG
    if not is_retry then
        -- 保留 saveconfig 等待期间已收到的数据
        mark_from_text(rx_buf, now)
    end
    verify_deadline_ms = now + VERIFY_TIMEOUT_MS
    if is_retry then
        gcs:send_text(SEV_WARN, string.format(
            "UM982工装: 校验重试 %d/%d @%d",
            verify_retry, VERIFY_RETRY_MAX, active_baud or 0))
    else
        gcs:send_text(SEV_INFO, string.format("UM982工装: 校验输出 @%d", active_baud or 0))
    end
end

local function begin_pass(now, reason)
    phase = PHASE_PASS
    led_state = LED_OK
    verify_retry = 0
    gcs:send_text(SEV_INFO, reason or "UM982工装: 通过 — GPGGA + UNIHEADINGA 正常")
end

local function fail_hard(reason)
    phase = PHASE_DETECT
    led_state = LED_NO_RTK
    config_sent = false
    verify_retry = 0
    rx_buf = ""
    detect_baud_idx = 1
    detect_last_probe_ms = 0
    detect_last_rx_ms = millis()
    if port ~= nil then
        set_port_baud(BAUD_CANDIDATES[detect_baud_idx])
    end
    gcs:send_text(SEV_WARN, "UM982工装: " .. reason)
end

local function fail_verify(reason, now)
    if config_sent and verify_retry < VERIFY_RETRY_MAX then
        verify_retry = verify_retry + 1
        if active_baud ~= TARGET_BAUD then
            set_port_baud(TARGET_BAUD)
        end
        rx_buf = ""
        begin_verify(now, true)
        return
    end
    fail_hard(reason)
end

local function gap_for_line(line)
    if line == "FRESET" then
        return POST_FRESET_MS
    end
    if string.sub(line, 1, 10) == "config com" then
        return POST_BAUD_SWITCH_MS
    end
    if line == "saveconfig" then
        return POST_SAVE_MS
    end
    return CMD_INTERVAL_MS
end

function update()
    if not init_led() then
        return update, 5000
    end

    local now = millis()

    if port == nil then
        if not open_scripting_port() then
            led_state = LED_NO_RTK
            apply_led_state()
            return update, 3000
        end
        begin_detect(now)
    end

    local new_data = read_port_all()
    if new_data ~= "" then
        detect_last_rx_ms = now
    end
    process_new_rx(new_data, now)

    if phase == PHASE_DETECT then
        send_detect_probe(now)
        if rx_has_rtk_marker() then
            gcs:send_text(SEV_INFO, string.format("UM982工装: RTK 已识别 @%d，重新配置", active_baud))
            begin_config(now, false)
        elseif now - detect_last_rx_ms >= DETECT_BAUD_TRY_MS then
            switch_detect_baud(now)
            send_detect_probe(now)
        end

    elseif phase == PHASE_CONFIG then
        if config_step < #cmds then
            if now - last_send_ms >= next_gap_ms then
                local prev = cmds[config_step]
                if config_step >= 1 and prev == "FRESET" then
                    if active_baud ~= FRESET_DEFAULT_BAUD then
                        set_port_baud(FRESET_DEFAULT_BAUD)
                    end
                    freset_sent = true
                    rx_buf = ""
                    gcs:send_text(SEV_INFO, string.format(
                        "UM982工装: FRESET 后按 %d 发配置", FRESET_DEFAULT_BAUD))
                elseif config_step >= 1 and string.sub(prev, 1, 10) == "config com" then
                    set_port_baud(TARGET_BAUD)
                    gcs:send_text(SEV_INFO, string.format("UM982工装: 切到 %d", TARGET_BAUD))
                end
                config_step = config_step + 1
                local line = cmds[config_step]
                send_line(line)
                last_send_ms = now
                next_gap_ms = gap_for_line(line)
            end
        elseif now - last_send_ms >= next_gap_ms then
            if active_baud ~= TARGET_BAUD then
                set_port_baud(TARGET_BAUD)
            end
            config_sent = true
            verify_retry = 0
            begin_verify(now, false)
        end

    elseif phase == PHASE_VERIFY then
        if verify_ok() then
            begin_pass(now)
        elseif now >= verify_deadline_ms then
            local miss = {}
            if not seen_gpgga then
                miss[#miss + 1] = "GPGGA"
            end
            if not seen_heading then
                miss[#miss + 1] = "UNIHEADINGA"
            end
            fail_verify("校验超时，缺 " .. table.concat(miss, "+"), now)
        end

    elseif phase == PHASE_PASS then
        if not pass_link_ok(now) then
            fail_hard("数据中断，请检查 RTK 连接")
        end
    end

    apply_led_state()
    return update, UPDATE_MS
end

return update()
