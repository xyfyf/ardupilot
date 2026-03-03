-- stick_gesture_arm.lua
-- Implements "V-Shape" (Inner-Eight) stick gesture for Arming/Disarming
-- Typically used in DJI-style controllers:
-- Left Stick: Bottom-Right (Throttle Low, Yaw Right)
-- Right Stick: Bottom-Left (Pitch Low, Roll Left)
--
-- CAUTION: This script allows disarming via sticks. Use with care.

local SCRIPT_NAME = "StickArm"
local ARM_DELAY_MS = 1500 -- Time in ms to hold the gesture
local STICK_THRESHOLD = 0.90 -- Threshold 0.0-1.0 (90% stick deflection)

local last_gesture_time = 0
local is_gesture_active = false

-- Helper to get normalized stick input (-1.0 to 1.0)
local function get_stick_input()
    -- Assuming standard channel mapping: 1=Roll, 2=Pitch, 3=Throttle, 4=Yaw
    local c1 = rc:get_channel(1)
    local c2 = rc:get_channel(2)
    local c3 = rc:get_channel(3)
    local c4 = rc:get_channel(4)
    
    if not c1 or not c2 or not c3 or not c4 then return nil, nil, nil, nil end
    
    return c1:norm_input(), c2:norm_input(), c3:norm_input(), c4:norm_input()
end

function update()
    local roll, pitch, throttle, yaw = get_stick_input()
    
    -- Safety check for RC link
    if not roll then return update, 100 end

    -- Check for "Inner-Eight" / "V-Shape" Gesture (Mode 2)
    -- Left Stick (Thr/Yaw): Bottom-Right -> Throttle Low (-), Yaw Right (+)
    -- Right Stick (Pit/Rol): Bottom-Left -> Pitch Low (-), Roll Left (-)
    
    local is_throttle_low = throttle < -STICK_THRESHOLD
    local is_yaw_right    = yaw      >  STICK_THRESHOLD
    local is_pitch_low    = pitch    < -STICK_THRESHOLD
    local is_roll_left    = roll     < -STICK_THRESHOLD

    local gesture_detected = is_throttle_low and is_yaw_right and is_pitch_low and is_roll_left

    if gesture_detected then
        local now = millis()
        if not is_gesture_active then
            -- Gesture started
            is_gesture_active = true
            last_gesture_time = now
        elseif (now - last_gesture_time) > ARM_DELAY_MS then
            -- Gesture held long enough
            
            if arming:is_armed() then
                -- Try to DISARM
                -- Note: ArduPilot prevents disarming in air by default unless in Stabilize/Acro
                if arming:disarm() then
                    gcs:send_text(6, SCRIPT_NAME .. ": Disarmed!")
                end
            else
                -- Try to ARM
                if arming:arm() then
                    gcs:send_text(6, SCRIPT_NAME .. ": Armed!")
                end
            end
            
            -- Prevent rapid re-triggering by resetting timer into the future
            last_gesture_time = now + 2000 
        end
    else
        is_gesture_active = false
    end

    return update, 50 -- Run at 20Hz
end

gcs:send_text(6, SCRIPT_NAME .. ": Loaded")
return update()

