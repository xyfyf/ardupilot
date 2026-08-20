-- 首次开机一次性写入：电台丢失保护仅告警不返航 (FS_THR_ENABLE=0)
-- SCR_USER4：已写入则不再执行；参数全清/重刷固件后会再跑一遍

local RUN_INTERVAL_MS = 1000
local DONE_FLAG_PARAM = "SCR_USER4"
local DONE_FLAG_VALUE = 6166102

local PARAMS = {
    FS_THR_ENABLE = 0,
}

local function is_flag_set()
    local flag = param:get(DONE_FLAG_PARAM)
    return flag and math.abs(flag - DONE_FLAG_VALUE) < 0.0001
end

local function apply_once()
    local changed = 0
    for name, target in pairs(PARAMS) do
        local current = param:get(name)
        if not current or math.abs(current - target) > 0.0001 then
            if param:set_and_save(name, target) then
                changed = changed + 1
            end
        end
    end
    param:set_and_save(DONE_FLAG_PARAM, DONE_FLAG_VALUE)
    gcs:send_text(6, "Radio FS: warn-only saved (" .. tostring(changed) .. " updated)")
end

function update()
    if is_flag_set() then
        return
    end
    apply_once()
end

return update()
