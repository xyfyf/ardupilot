--[[
  临时测试灯语（测完请删除本文件）:
    Type-C / 数传当前出厂/板级包头语义：
      MAV_TX_MAGIC==0 或 0xEF(239) → 绿灯常亮（EF）
      MAV_TX_MAGIC==0xFD(253)      → 红灯常亮（FD）
      其它值                       → 黄灯常亮（未知）

  依赖: SERVOx_FUNCTION=94 (Scripting1), SCR_ENABLE=1
  注意: 勿与 ws2812_fc_status_led.lua 同时加载。
--]]

---@diagnostic disable: need-check-nil

local LED_SERVO_FUNCTION = 94
local NUM_LEDS = 8
local BRIGHTNESS = 255
local UPDATE_MS = 200

local MAGIC_FD = 253  -- 0xFD
local MAGIC_EF = 239  -- 0xEF

local led_chan = nil
local init_ok = false
local last_magic = -999
local last_warn_ms = 0

local function strip_set(r, g, b)
    local s = BRIGHTNESS / 255.0
    r = math.floor(r * s + 0.5)
    g = math.floor(g * s + 0.5)
    b = math.floor(b * s + 0.5)
    serialLED:set_RGB(led_chan, -1, r, g, b)
    serialLED:send(led_chan)
end

local function framing_rgb(magic)
    if magic == nil then
        return 255, 255, 0  -- 黄：读不到参数
    end
    if magic == 0 or magic == MAGIC_EF then
        return 0, 255, 0    -- 绿：EF / 板级默认
    end
    if magic == MAGIC_FD then
        return 255, 0, 0    -- 红：FD
    end
    return 0, 255, 0      -- 绿灯常亮（EF）
end

function update()
    if not init_ok then
        local ch0 = SRV_Channels:find_channel(LED_SERVO_FUNCTION)
        if ch0 == nil then
            local now = millis()
            if now - last_warn_ms > 5000 then
                last_warn_ms = now
                gcs:send_text(6, "FrameLED: wait SERVOx_FUNCTION=94")
            end
            return update, 1000
        end
        led_chan = ch0 + 1
        if not serialLED:set_num_neopixel(led_chan, NUM_LEDS) then
            return update, 1000
        end
        init_ok = true
        gcs:send_text(6, "FrameLED TEST: EF=green FD=red")
    end

    local magic = param:get("MAV_TX_MAGIC")
    if magic ~= nil then
        magic = math.floor(magic + 0.5)
    end

    local r, g, b = framing_rgb(magic)
    strip_set(r, g, b)

    if magic ~= last_magic then
        last_magic = magic
        if magic == nil then
            gcs:send_text(6, "FrameLED: MAV_TX_MAGIC missing")
        elseif magic == 0 then
            gcs:send_text(6, "FrameLED: board default (EF) -> green")
        elseif magic == MAGIC_EF then
            gcs:send_text(6, "FrameLED: magic=0xEF -> green")
        elseif magic == MAGIC_FD then
            gcs:send_text(6, "FrameLED: magic=0xFD -> red")
        else
            gcs:send_text(6, string.format("FrameLED: magic=%d -> yellow", magic))
        end
    end

    return update, UPDATE_MS
end

return update()
