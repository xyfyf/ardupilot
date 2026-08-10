--[[
  rid_heartbeat_check.lua

  功能:
    通过飞控 OpenDroneID 判断 DID_MAVPORT（默认 SERIAL2）上的 RID 是否在线。
    依据：RID 模块周期上报的 OPEN_DRONE_ID_ARM_STATUS（不是普通 HEARTBEAT）。
    超过 HB_TIMEOUT_MS 未收到 → 地面站报警并禁止解锁。

  参数 (地面站搜索 RIDHB_):
    RIDHB_ENABLE : 0=关闭(任意 RID 错误含 106/107/GB46750 等：不转发原始原因、518/12918 报无错误、不拦解锁), 1=开启(默认)

  相关飞控参数:
    DID_ENABLE=1, DID_MAVPORT=2（RID 接在 SERIAL2 / UART4）
--]]

---@diagnostic disable: need-check-nil

local SCRIPT_NAME      = "RID_HB"
local RUN_INTERVAL_MS  = 1000    -- 1 Hz 检查
local STARTUP_DELAY_MS = 10000   -- 等待 RID 模块上电（10s，仅开启时）
local HB_TIMEOUT_MS    = 5000    -- 超过 5s 未收到 ARM_STATUS 视为 RID 丢失
local WARN_INTERVAL_MS = 5000    -- 重复告警最小间隔（ms）

-- 脚本参数
local PARAM_TABLE_KEY    = 97   -- 合法范围 0~200；91=UOM, 96=LNDS, 200=GPSYS
local PARAM_TABLE_PREFIX = "RIDHB_"
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 1), "RIDHB: add_table fail")
assert(param:add_param(PARAM_TABLE_KEY, 1, "ENABLE", 1), "RIDHB: add ENABLE fail")
local ridhb_enable = Parameter("RIDHB_ENABLE")

local rid_auth_id  = nil
local last_warn_ms = 0

local function script_enabled()
    return (ridhb_enable:get() or 0) >= 1
end

local function release_auth()
    if rid_auth_id ~= nil then
        arming:set_aux_auth_passed(rid_auth_id)
    end
end

rid_auth_id = arming:get_aux_auth_id()
if rid_auth_id == nil then
    if script_enabled() then
        gcs:send_text(3, SCRIPT_NAME .. ": 鉴权槽获取失败，无法禁止解锁")
    end
else
    if not script_enabled() then
        release_auth()
    end
end

function update()
    local now = millis()

    if not script_enabled() then
        release_auth()
        return update, RUN_INTERVAL_MS
    end

    -- 与固件 "ODID: lost transmitter" 同源：看 ARM_STATUS 是否新鲜
    local rid_ok = opendroneid:transmitter_healthy(HB_TIMEOUT_MS)

    if rid_auth_id ~= nil then
        if rid_ok then
            arming:set_aux_auth_passed(rid_auth_id)
        else
            local fail_msg = "COM2的RID丢失，禁止解锁"
            arming:set_aux_auth_failed(rid_auth_id, fail_msg)
            if (now - last_warn_ms) >= WARN_INTERVAL_MS then
                gcs:send_text(3, fail_msg)
                last_warn_ms = now
            end
        end
    end

    return update, RUN_INTERVAL_MS
end

if script_enabled() then
    return update, STARTUP_DELAY_MS
end
return update, RUN_INTERVAL_MS
