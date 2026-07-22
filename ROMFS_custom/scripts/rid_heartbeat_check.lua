--[[
  rid_heartbeat_check.lua

  功能:
    监测 SERIAL2 (COM2) 上 RID 模块的 MAVLink 心跳包。
    超过 HB_TIMEOUT_MS 未收到心跳 → 地面站报警并禁止解锁。

  参数 (地面站搜索 RIDHB_):
    RIDHB_ENABLE : 0=关闭检测, 1=开启(默认)

  前提:
    SERIAL2_PROTOCOL = 2 (MAVLink2)，RID 模块按标准 MAVLink 发送 HEARTBEAT。
    SERIAL2 是第 3 个启用 MAVLink 的串口，对应 channel 2。
    若通道号不同，修改下方 RID_CHAN 常量。
--]]

---@diagnostic disable: need-check-nil

local SCRIPT_NAME      = "RID_HB"
local RUN_INTERVAL_MS  = 1000    -- 1 Hz 检查
local STARTUP_DELAY_MS = 10000   -- 等待 RID 模块上电（10s）
local HB_TIMEOUT_MS    = 5000    -- 超过 5s 未收到心跳视为 RID 丢失
local WARN_INTERVAL_MS = 5000    -- 重复告警最小间隔（ms）

local HEARTBEAT_MSGID  = 0       -- MAVLink HEARTBEAT message id
local RID_CHAN         = 2       -- SERIAL2 对应的 MAVLink 接收通道号

-- 脚本参数
local PARAM_TABLE_KEY    = 97   -- 合法范围 0~200；91=UOM, 96=LNDS, 200=GPSYS
local PARAM_TABLE_PREFIX = "RIDHB_"
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 1), "RIDHB: add_table fail")
assert(param:add_param(PARAM_TABLE_KEY, 1, "ENABLE", 1), "RIDHB: add ENABLE fail")
local ridhb_enable = Parameter("RIDHB_ENABLE")

local rid_auth_id  = nil
local last_hb_ms   = nil   -- 最近一次收到 RID 心跳的时间戳（nil=从未收到）
local last_warn_ms = 0

mavlink:init(10, 1)
mavlink:register_rx_msgid(HEARTBEAT_MSGID)

rid_auth_id = arming:get_aux_auth_id()
if rid_auth_id == nil then
    gcs:send_text(3, SCRIPT_NAME .. ": 鉴权槽获取失败，无法禁止解锁")
end

local function script_enabled()
    return (ridhb_enable:get() or 0) >= 1
end

function update()
    local now = millis()

    -- 关闭时放行解锁，并排空队列避免积压
    if not script_enabled() then
        local msg = mavlink:receive_chan()
        while msg ~= nil do
            msg = mavlink:receive_chan()
        end
        if rid_auth_id ~= nil then
            arming:set_aux_auth_passed(rid_auth_id)
        end
        last_hb_ms = nil
        return update, RUN_INTERVAL_MS
    end

    -- 排空队列，只记录来自 RID_CHAN 的心跳时间
    local msg, chan = mavlink:receive_chan()
    while msg ~= nil do
        if chan == RID_CHAN then
            last_hb_ms = now
        end
        msg, chan = mavlink:receive_chan()
    end

    local rid_ok = (last_hb_ms ~= nil) and ((now - last_hb_ms) < HB_TIMEOUT_MS)

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

return update, STARTUP_DELAY_MS
