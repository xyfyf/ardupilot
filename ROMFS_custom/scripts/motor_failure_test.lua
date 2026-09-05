-- This is a script that stops motors in flight, for use testing motor failure handling
-- Requires RCx_OPTION=301 (Scripting2); if none configured, script exits silently.

-- find rc switch with option 301 (Scripting2)
local switch = rc:find_channel_for_option(301)
if not switch then
  return
end

-- add new params MOT_STOP_BITMASK and MOT_STOP_DECL
local PARAM_TABLE_KEY = 75
assert(param:add_table(PARAM_TABLE_KEY, "MOT_", 2), "could not add param table")
assert(param:add_param(PARAM_TABLE_KEY, 1, "STOP_BITMASK", 0), "could not add param")
-- MOT_STOP_DECL: tell the mixer about the injected failure, instead of leaving it
-- to the rpm detector.
--
--   0 (default) - inject only.  The mixer is not told, so the vehicle sees exactly
--       what a real stoppage looks like and MOT_FAIL_RPM has to find it.  This is
--       the mode that exercises the whole detect -> degrade -> reallocate chain,
--       and the only one that says anything about the declared detection capability.
--
--   1 - declare.  On the first switch-high this writes the stopped motor number to
--       MOT_FAIL_IDX, so the mixer degrades on the very next pass with no detection
--       delay at all.  Use it to test the allocator on its own - first real-aircraft
--       flights, or any time a detection failure would be the thing that hurts.
--
-- This is a test aid and nothing more: in a real failure no script writes MOT_FAIL_IDX.
-- Never let a run with MOT_STOP_DECL=1 stand as evidence that detection works.
assert(param:add_param(PARAM_TABLE_KEY, 2, "STOP_DECL", 0), "could not add param")

local stop_motor_bitmask = Parameter()
assert(stop_motor_bitmask:init("MOT_STOP_BITMASK"), "could not find param")
local declare_to_mixer = Parameter()
assert(declare_to_mixer:init("MOT_STOP_DECL"), "could not find param")

local declared = false
local mask_refused = false   -- 多位掩码只告警一次，且此后整架次不再注入
local warned_empty = false

-- read spin min param, we set motors to this PWM to stop them
local pwm_min
if quadplane then
  pwm_min = assert(param:get("Q_M_PWM_MIN"),"Lua: Could not read Q_M_PWM_MIN")
else
  pwm_min = assert(param:get("MOT_PWM_MIN"),"Lua: Could not read MOT_PWM_MIN")
end

local stop_motor_chan
local last_motor_bitmask

-- find any motors enabled, populate channels numbers to stop
local function update_stop_motors(new_bitmask)
  if last_motor_bitmask == new_bitmask then
    return
  end
  stop_motor_chan = {}
  for i = 1, 12 do
    if ((1 << (i-1)) & new_bitmask) ~= 0 then
      -- convert motor number to output function number
      local output_function
      if i <= 8 then
        output_function = i+32
      else
        output_function = i+81-8
      end

      -- get channel number for output function
      local temp_chan = SRV_Channels:find_channel(output_function)
      if temp_chan then
        table.insert(stop_motor_chan, temp_chan)
      end
    end
  end
  last_motor_bitmask = new_bitmask
end

-- Hand the injected failure straight to the mixer.  Runs once, on the first
-- switch-high, and only with MOT_STOP_DECL=1.
--
-- MOT_FAIL_IDX takes one motor, so refuse a bitmask naming several: degrading on a
-- guess about which one is meant would remove a healthy column - the same reason
-- update_failure_detection() warns instead of acting when several ESCs read stopped.
--
-- param:set writes RAM only.  Saving it would leave the value across a reboot, and
-- arming is refused while MOT_FAIL_IDX is non-zero, so a saved value would block the
-- next flight.  Note the degradation itself is deliberately one-way: clearing
-- MOT_FAIL_IDX does not bring the motor back, and the vehicle stays degraded until
-- it is rebooted.  That is what makes the arming refusal the right interlock - do
-- not "helpfully" zero this on disarm, or the next takeoff starts degraded with
-- nothing on screen to say so.
-- Validate the mask **before anything acts on it**, and let one answer gate both
-- the declaration and the injection.
--
-- This used to live inside declare_failure(), which meant a mask naming several
-- motors was refused for declaration and then injected anyway: update() ran its
-- override loop over stop_motor_chan regardless, and that array is filled from
-- every bit in the mask.  With mask=3 the result was two motors genuinely
-- stopped while the mixer knew nothing - a hexacopter down two rotors has no
-- trim solution at all, so that outcome is worse than either stopping one or
-- stopping none.  The only thing on screen was "not declaring", which reads
-- like a safe refusal.
--
-- Returns the motor number when exactly one is named, otherwise nil.
local function validate_mask(bitmask)
  local first, count = nil, 0
  for i = 1, 12 do
    if ((1 << (i-1)) & bitmask) ~= 0 then
      count = count + 1
      first = first or i
    end
  end

  -- Empty mask: keep waiting, do not latch.  MOT_STOP_BITMASK arrives from the
  -- ground station and the switch is thrown by hand, so the two can land in
  -- either order - and the switch getting there first is the ordinary case when
  -- a parameter write is still in flight.  Latching here would consume the one
  -- declaration this flight gets on a mask that names nobody, and the real
  -- injection a moment later would then degrade nothing at all, silently.
  -- Warn on the rising edge only; this runs at 100 Hz.
  if count == 0 then
    if not warned_empty then
      warned_empty = true
      gcs:send_text(4, "MotorFail: switch high, MOT_STOP_BITMASK empty - waiting")
    end
    return nil
  end
  warned_empty = false

  -- Several motors named: refuse the whole injection, and latch.  MOT_FAIL_IDX
  -- takes one motor, so degrading on a guess about which is meant would remove a
  -- healthy column - the same reason update_failure_detection() warns instead of
  -- acting when several ESCs read stopped.  Latching is deliberate: the mask was
  -- set wrong, and quietly acting on a later correction mid-flight is worse than
  -- making the operator land and fix it.
  if count > 1 then
    if not mask_refused then
      mask_refused = true
      gcs:send_text(2, string.format(
        "MotorFail: %d motors in mask - injection BLOCKED, land and fix", count))
    end
    return nil
  end

  return first
end


local function declare_failure(first)
  if declared or declare_to_mixer:get() ~= 1 then
    return
  end

  declared = true

  -- param:set writes RAM only.  Saving it would leave the value across a reboot,
  -- and arming is refused while MOT_FAIL_IDX is non-zero, so a saved value would
  -- block the next flight.  The degradation itself is deliberately one-way:
  -- clearing MOT_FAIL_IDX does not bring the motor back, and the vehicle stays
  -- degraded until rebooted.  That is what makes the arming refusal the right
  -- interlock - do not "helpfully" zero this on disarm, or the next takeoff
  -- starts degraded with nothing on screen to say so.
  if param:set("MOT_FAIL_IDX", first) then
    gcs:send_text(4, string.format("MotorFail: declared motor %d to mixer", first))
  else
    gcs:send_text(2, "MotorFail: could not set MOT_FAIL_IDX")
  end
end

function update()

  local bitmask = stop_motor_bitmask:get()
  update_stop_motors(bitmask)

  if switch:get_aux_switch_pos() == 2 then
    -- One validation gates both halves.  Anything but exactly one motor and
    -- nothing happens at all - no declaration, and no output override either.
    local target = validate_mask(bitmask)
    if target == nil then
      return update, 10
    end
    declare_failure(target)
    for i = 1, #stop_motor_chan do
      -- override for 15ms, called every 10ms
      -- using timeout means if the script dies the timeout will expire and all motors will come back
      -- we cant leave the vehicle in a un-flyable state
      SRV_Channels:set_output_pwm_chan_timeout(stop_motor_chan[i],pwm_min,15)
    end
  end

  return update, 10 -- reschedule at 100hz
end

return update() -- run immediately before starting to reschedule
