--[[
  UAVCAN/DroneCAN 雷达自动检测与参数适配脚本（内存优先，默认不重启）

  逻辑：开机后一段时间内监听 DroneCAN NodeStatus，根据雷达节点是否在线，用 param:set
  仅修改内存中的 PRX1_TYPE（不写 Flash），从而尽量避免「改参 + 重启」的笨拙流程。
  同时根据检测结果自适应 ARMING_CHECK 的 Rangefinder 位（bit 15）：
  - 检测到雷达 → 置 1，恢复正常测距仪解锁前检查
  - 没检测到雷达 → 置 0，让飞控不接雷达也能正常解锁/起飞，不报错
  注意：若原 ARMING_CHECK 置 1（即 ALL 总开关），关闭 Rangefinder 时会自动把 ALL
  展开为 bits 1..19 的显式位再清 bit 15，避免 ALL 强制覆盖单独清位无效的问题。

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
local arming_check_param = Parameter("ARMING_CHECK")

-- ARMING_CHECK bit 定义（与 AP_Arming.h 的 enum class Check 对应）
local ARMING_CHECK_ALL_BIT  = 1 << 0  -- bit 0：所有检查总开关（置 1 时单独清其他位无效）
local ARMING_CHECK_RNGFND   = 1 << 15 -- bit 15：Rangefinder
-- 当需要把 ALL 展开为显式位时使用：bits 1..19 全开（覆盖常规检查项，含/不含 Rangefinder 由后续运算决定）
local ARMING_CHECK_EXPANDED = 0xFFFFE

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

--- 仅写 RAM 切换 ARMING_CHECK 的 Rangefinder 位
-- enable=true  → 置 1（雷达检测到，恢复测距仪解锁前检查）
-- enable=false → 置 0（无雷达，跳过测距仪检查，避免无雷达起不来）
-- 行为说明（重要）：
-- 1) 若 ALL(bit0)=0（你这种"显式勾选"模式）：只动 bit 15，其他位完全不变
-- 2) 若 ALL(bit0)=1（默认全开模式）且要关 Rangefinder：必须先把 ALL 展开为显式位
--    （bits 1..19 全开），然后再清 bit 15；否则 ALL 会强制启用所有检查
local function set_arming_rangefinder_check(enable)
    local cur = arming_check_param:get()
    if cur == nil then
        gcs:send_text(4, "param get ARMING_CHECK err")
        return false
    end
    cur = math.floor(cur)
    local original = cur

    local target
    if enable then
        target = cur | ARMING_CHECK_RNGFND
    else
        if (cur & ARMING_CHECK_ALL_BIT) ~= 0 then
            cur = (cur & (~ARMING_CHECK_ALL_BIT)) | ARMING_CHECK_EXPANDED
            gcs:send_text(6, string.format("ARMING_CHECK ALL expanded: %d -> %d", original, cur))
        end
        target = cur & (~ARMING_CHECK_RNGFND)
    end

    if target == original then
        gcs:send_text(6, string.format("ARMING_CHECK unchanged: %d", original))
        return true
    end

    if not arming_check_param:set(target) then
        gcs:send_text(4, "param set ARMING_CHECK err")
        return false
    end
    gcs:send_text(6, string.format("ARMING_CHECK: %d -> %d", original, target))
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

        -- 同步 ARMING_CHECK 的 Rangefinder 位：检测到雷达 → 启用，未检测到 → 禁用
        -- 仅写内存，避免污染 EEPROM；下次重启脚本会再次按当前情况设置
        if set_arming_rangefinder_check(radar_seen) then
            if radar_seen then
                gcs:send_text(6, "ARMING_CHECK: Rangefinder enabled")
            else
                gcs:send_text(6, "ARMING_CHECK: Rangefinder disabled (no radar)")
            end
        end

        check_done = true
        return
    end

    return update, 100
end

return update, 3000
