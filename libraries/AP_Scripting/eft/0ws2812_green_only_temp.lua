--[[
  WS2812 临时测试脚本 — 只常亮绿灯

  基于 1ws2812_fc_status_led5.1.lua 精简，无灯语/状态机，用于硬件接线与亮度验证。
  验证完成后请改回正式脚本 1ws2812_fc_status_led5.1.lua
--]]

---@diagnostic disable: need-check-nil

local LED_SERVO_FUNCTION = 94  -- SERVOx_FUNCTION = 94 (Scripting1)
local NUM_LEDS = 8
local BRIGHTNESS = 90
local UPDATE_MS = 200

local led_chan = nil
local init_ok = false

local function strip_set_green(chan)
    local s = BRIGHTNESS / 255.0
    local g = math.floor(255 * s + 0.5)
    serialLED:set_RGB(chan, -1, 0, g, 0)
    serialLED:send(chan)
end

function update()
    if not init_ok then
        local ch0 = SRV_Channels:find_channel(LED_SERVO_FUNCTION)
        if ch0 == nil then
            gcs:send_text(6, "LED: Waiting SERVOx_FUNCTION=" .. tostring(LED_SERVO_FUNCTION))
            return update, 5000
        end
        led_chan = ch0 + 1
        if not serialLED:set_num_neopixel(led_chan, NUM_LEDS) then
            return update, 5000
        end
        init_ok = true
        gcs:send_text(6, "LED: WS2812 GREEN ONLY OK")
    end

    strip_set_green(led_chan)
    return update, UPDATE_MS
end

return update()
