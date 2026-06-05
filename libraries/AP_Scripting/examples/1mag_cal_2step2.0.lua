-- 磁罗盘两步式快速校准 (姿态自动触发)
-- 触发：未解锁 + pitch<-60° 保持 5s
-- 流程：机头朝下转一圈 → 水平转一圈 → 静止等保存
-- 成功后冻结在 DONE 状态直到重启，不会被误触发
-- 失败后放平 2s 可再次触发
--
-- 版本：2.5 修复成功后多余提示 + 成功后禁止再触发

local LOOP_INTERVAL_MS = 200

local TRIG_PITCH_RAD    = math.rad(-60)
local TRIG_HOLD_MS      = 5000
local RECOVER_PITCH_RAD = math.rad(-30)
local RECOVER_HOLD_MS   = 2000

local T_STEP1_END_MS = 35000
local T_STEP2_END_MS = 70000
local T_GUIDE_END_MS = 90000
local T_MAX_WAIT_MS  = 180000

local MAV_CMD_DO_START_MAG_CAL  = 42424
local MAV_CMD_DO_CANCEL_MAG_CAL = 42426

local OFS_DELTA_THRESH = 1.0
local CAL_MASK         = 0

local P_CAL_FIT  = Parameter("COMPASS_CAL_FIT")
local P_AUTO_ROT = Parameter("COMPASS_AUTO_ROT")

local mags = {
    {pfx = "COMPASS_OFS_",  use = "COMPASS_USE",  bx=0, by=0, bz=0, x=nil, y=nil, z=nil, enabled=false},
    {pfx = "COMPASS_OFS2_", use = "COMPASS_USE2", bx=0, by=0, bz=0, x=nil, y=nil, z=nil, enabled=false},
    {pfx = "COMPASS_OFS3_", use = "COMPASS_USE3", bx=0, by=0, bz=0, x=nil, y=nil, z=nil, enabled=false},
}
for i = 1, #mags do
    mags[i].x = Parameter(mags[i].pfx .. "X")
    mags[i].y = Parameter(mags[i].pfx .. "Y")
    mags[i].z = Parameter(mags[i].pfx .. "Z")
end

-- 状态
local STATE_IDLE  = 0
local STATE_STEP1 = 1
local STATE_STEP2 = 2
local STATE_HOLD  = 3
local STATE_WAIT  = 4
local STATE_DONE  = 5   -- 失败后可重触发
local STATE_SAVED = 6   -- 成功后永久冻结，直到重启

local state            = STATE_IDLE
local cal_start_ms     = 0
local trig_first_ms    = 0
local recover_first_ms = 0
local reboot_hint_ms   = 0
local last_pbar_ms     = 0
local last_pbar_pct    = -1

local function apply_easy_cal_params()
    P_CAL_FIT:set(32)
    P_AUTO_ROT:set(0)
end

local function snapshot_baselines()
    for i = 1, #mags do
        local m = mags[i]
        m.enabled = (param:get(m.use) or 0) >= 1
        if m.enabled then
            m.bx = m.x:get() or 0
            m.by = m.y:get() or 0
            m.bz = m.z:get() or 0
        end
    end
end

local function start_cal()
    if arming:is_armed() then
        gcs:send_text(3, "磁罗盘:请先上锁")
        return false
    end
    apply_easy_cal_params()
    snapshot_baselines()
    local res = gcs:run_command_int(MAV_CMD_DO_START_MAG_CAL, {
        p1 = CAL_MASK, p2 = 0, p3 = 1, p4 = 0,
        x  = 0, y  = 0, z  = 0,
    })
    if res ~= 0 then
        gcs:send_text(3, string.format("磁罗盘:启动失败(%d)", res))
        return false
    end
    cal_start_ms = millis():toint()
    trig_first_ms = 0
    last_pbar_ms = 0
    last_pbar_pct = -1
    state = STATE_STEP1
    gcs:send_text(5, "第1步:机头朝下慢转一圈")
    return true
end

local function cancel_cal(reason)
    gcs:run_command_int(MAV_CMD_DO_CANCEL_MAG_CAL, { p1 = CAL_MASK })
    state = STATE_IDLE
    cal_start_ms = 0
    trig_first_ms = 0
    recover_first_ms = 0
    last_pbar_ms = 0
    last_pbar_pct = -1
    gcs:send_text(5, string.format("磁罗盘:已取消 %s", reason or ""))
end

local function check_cal_save()
    for i = 1, #mags do
        local m = mags[i]
        if m.enabled then
            local nx = m.x:get()
            local ny = m.y:get()
            local nz = m.z:get()
            if nx ~= nil and ny ~= nil and nz ~= nil then
                if math.abs(nx - m.bx) > OFS_DELTA_THRESH
                   or math.abs(ny - m.by) > OFS_DELTA_THRESH
                   or math.abs(nz - m.bz) > OFS_DELTA_THRESH then
                    -- 成功：发一条带偏置值的消息，进入永久冻结状态
                    gcs:send_text(2, string.format("校准成功 #%d 偏置=%.0f,%.0f,%.0f 请重启", i-1, nx, ny, nz))
                    state = STATE_SAVED
                    cal_start_ms = 0
                    reboot_hint_ms = millis():toint()
                    return true
                end
            end
        end
    end
    return false
end

local function update_phase()
    if state ~= STATE_STEP1 and state ~= STATE_STEP2
       and state ~= STATE_HOLD and state ~= STATE_WAIT then
        return
    end

    if check_cal_save() then
        return
    end

    local now = millis():toint()
    local elapsed = now - cal_start_ms

    -- 文本进度条，每 5s 一次
    if (now - last_pbar_ms) >= 5000 then
        last_pbar_ms = now
        local pct = math.floor((elapsed * 100) / T_GUIDE_END_MS)
        pct = math.max(0, math.min(99, pct))
        if pct ~= last_pbar_pct then
            last_pbar_pct = pct
            local filled = math.floor(pct / 5)
            local bar = string.rep("#", filled) .. string.rep("-", 20 - filled)
            local label = "等待"
            if state == STATE_STEP1 then label = "第1步"
            elseif state == STATE_STEP2 then label = "第2步"
            elseif state == STATE_HOLD  then label = "静止"
            elseif state == STATE_WAIT  then label = "保存中"
            end
            gcs:send_text(6, string.format("[%s]%2d%% %s", bar, pct, label))
        end
    end

    if state == STATE_STEP1 and elapsed >= T_STEP1_END_MS then
        state = STATE_STEP2
        gcs:send_text(5, "第2步:水平慢转一圈")
    elseif state == STATE_STEP2 and elapsed >= T_STEP2_END_MS then
        state = STATE_HOLD
        gcs:send_text(5, "磁罗盘:保持静止 等待保存")
    elseif state == STATE_HOLD and elapsed >= T_GUIDE_END_MS then
        state = STATE_WAIT
        gcs:send_text(6, "磁罗盘:等待飞控保存 请稍候")
    elseif state == STATE_WAIT then
        if elapsed >= T_MAX_WAIT_MS then
            gcs:send_text(3, "磁罗盘失败:偏置未保存")
            gcs:run_command_int(MAV_CMD_DO_CANCEL_MAG_CAL, { p1 = CAL_MASK })
            state = STATE_DONE
            cal_start_ms = 0
            recover_first_ms = 0
            last_pbar_ms = 0
            last_pbar_pct = -1
        end
    end
end

function update()
    update_phase()

    local now = millis():toint()
    local pitch = ahrs:get_pitch_rad()

    -- 解锁中取消校准
    if arming:is_armed() then
        if state ~= STATE_IDLE and state ~= STATE_DONE and state ~= STATE_SAVED then
            cancel_cal("已解锁")
        end
        return update, LOOP_INTERVAL_MS
    end

    -- 成功后永久冻结：每 60s 提醒一次重启
    if state == STATE_SAVED then
        if (now - reboot_hint_ms) >= 60000 then
            reboot_hint_ms = now
            gcs:send_text(6, "请重启飞控 校准已保存")
        end
        return update, LOOP_INTERVAL_MS
    end

    -- 失败后等姿态放平才允许再次触发
    if state == STATE_DONE then
        if pitch > RECOVER_PITCH_RAD then
            if recover_first_ms == 0 then
                recover_first_ms = now
            elseif now - recover_first_ms >= RECOVER_HOLD_MS then
                state = STATE_IDLE
                recover_first_ms = 0
                trig_first_ms = 0
                gcs:send_text(6, "磁罗盘:已就绪 可再次触发")
            end
        else
            recover_first_ms = 0
        end
        return update, LOOP_INTERVAL_MS
    end

    -- IDLE：检测机头朝下 5s 触发
    if state == STATE_IDLE then
        if pitch < TRIG_PITCH_RAD then
            if trig_first_ms == 0 then
                trig_first_ms = now
                gcs:send_text(6, "磁罗盘:机头朝下 保持5秒触发")
            elseif now - trig_first_ms >= TRIG_HOLD_MS then
                trig_first_ms = 0
                if not start_cal() then
                    state = STATE_DONE
                    recover_first_ms = 0
                end
            end
        else
            if trig_first_ms ~= 0 then
                trig_first_ms = 0
            end
        end
        return update, LOOP_INTERVAL_MS
    end

    return update, LOOP_INTERVAL_MS
end

gcs:send_text(6, "磁罗盘两步校准v2.5 已加载")
return update()
