--[[
  UAVCAN/DroneCAN 雷达自动检测与参数适配脚本（内存优先，默认不重启）

  逻辑：开机后一段时间内监听 DroneCAN NodeStatus，根据雷达节点是否在线，用 param:set
  仅修改内存中的 PRX1_TYPE（不写 Flash），从而尽量避免「改参 + 重启」的笨拙流程。

  为何可以不全重启：
  - PRX 后端在 init() 时按类型创建；DroneCAN 类可在类型已为 DroneCAN 且收到报文时延迟创建驱动
    （见 libraries/AP_Proximity 中 DroneCAN 后端逻辑）。
  - 若上电时 PRX1_TYPE=0（无后端实例），脚本在内存里写成 DroneCAN 类型后，雷达开始发数据即可挂载。
  - valid_instance() 会检查当前类型是否为 None：内存里改成 0 后，该实例对避障而言一般即失效，无需为了
    「关避障」一定重启（个别版本若 OA 仍认为有传感器，再以地面站保存参数/重启为准）。

  何时仍可能需要保存或重启（脚本只在注释里说明，不自动做）：
  - 上电时 PRX1_TYPE 已是非 0 且与目标不符时，固件可能要求重启才能换类型（DroneCAN 后端注释）。
  - 需要断电后仍自动开关雷达：必须把 PRX 写入 EEPROM（地面站保存参数或改用 set_and_save）。

  依赖：CAN_Px_DRIVER=1 (DroneCAN)；SCR_ENABLE=1。解锁后不执行任何修改。
--]]

local TARGET_PRX_TYPE = 14 -- 原生 DroneCAN 避障雷达通常为 14；测距仪转避障等见 Wiki
local RADAR_NODE_ID = 120 -- 【必改】地面站 UAVCAN 里看到的雷达 Node ID

local prx_param = Parameter("PRX1_TYPE")

-- DroneCAN Handle：NodeStatus id 341
local NODESTATUS_ID = 341
local NODESTATUS_SIGNATURE = uint64_t(0x0F0868D0, 0xC1A7C6F1)
local nodestatus_handle = DroneCAN_Handle(0, NODESTATUS_SIGNATURE, NODESTATUS_ID)

    if not nodestatus_handle then
        gcs:send_text(4, "DroneCAN handle err")
        return
    end

nodestatus_handle:subscribe()

local BOOT_DELAY_MS = 30000
local radar_seen = false
local check_done = false

--- 仅写 RAM，不写 EEPROM；断电后恢复为上次保存的 PRX1_TYPE
local function set_prx_type_memory_only(v)
    if not prx_param:set(v) then
        gcs:send_text(4, "param set PRX1_TYPE err")
        return false
    end
    return true
end

function update()
    if check_done then
        return
    end

    local now = millis()

    if arming:is_armed() then
        check_done = true
        return
    end

    local msg, source_node = nodestatus_handle:check_message()
    while msg do
        if source_node == RADAR_NODE_ID then
            radar_seen = true
        end
        msg, source_node = nodestatus_handle:check_message()
    end

    if now > BOOT_DELAY_MS then
        local current_type = prx_param:get()
        if current_type == nil then
            check_done = true
            return
        end

        if radar_seen and current_type == 0 then
            gcs:send_text(6, "Radar detected, PRX1_TYPE=DroneCAN(RAM)")
            set_prx_type_memory_only(TARGET_PRX_TYPE)
        elseif not radar_seen and current_type ~= 0 then
            -- 注释掉关闭雷达的逻辑，避免因雷达启动慢或 Node ID 不匹配导致误关雷达
            gcs:send_text(6, "No radar seen, but keeping PRX1_TYPE")
            -- set_prx_type_memory_only(0)
        else
            gcs:send_text(6, "Radar state matched")
        end

        check_done = true
        return
    end

    return update, 100
end

return update, 3000
