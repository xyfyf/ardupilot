--[[
  UAVCAN/DroneCAN 雷达自动检测与参数适配脚本（方案 A：写 EEPROM 持久化）

  ===== 测试速读（30 秒看完） =====
  1) 用地面站 DroneCAN 监视器查到雷达真实 Node ID，填到下面的 RADAR_NODE_ID。
  2) 首次部署：装脚本 → 雷达插着上电 → 等 30s → 看到 GCS 提示
     "PRX1_TYPE saved=14, REBOOT to enable avoidance" → 重启 1 次（仅此一次）。
  3) 之后行为：雷达插着 = 自动避障；拔了重启 = 能正常解锁起飞、不避障；
     雷达插回 = 立刻恢复避障；全程【不再需要重启】。
  4) 若 30s 后没看到任何 "Radar state matched / ARMING_CHECK saved / REBOOT…" 类
     消息，先检查 RADAR_NODE_ID、PRX1_ADDR、CAN_Px_DRIVER=1、SCR_ENABLE=1。
  =================================

  v4.0 能做什么
  -------------
  目标：让 DroneCAN 避障雷达"插着即用、拔了也能起飞、插回当场恢复"，把
  「改参 → 保存 → 重启」这套笨拙流程压缩到只剩首次部署的一次重启。

  1) 开机 0..BOOT_DELAY_MS（默认 30s）内监听 DroneCAN NodeStatus，根据指定的
     RADAR_NODE_ID 自动判断雷达是否在线。
  2) 检测到雷达：
     - 把 PRX1_TYPE 持久化为 14（DroneCAN 避障）。保证下一次开机
       AP_Proximity::init() 把 num_instances 设到 1，handle_measurement() 懒加载
       driver、update() 循环消费 items 队列，整条避障数据链打通。
     - ARMING_CHECK bit15（Rangefinder/Proximity 解锁前检查）持久化为 1，
       恢复正常解锁前检查。
  3) 没检测到雷达：
     - 【不动】PRX1_TYPE。保留 EEPROM 里之前存好的 14，下一次雷达回来后无需再
       写参数、无需再重启即可生效。
     - ARMING_CHECK bit15 持久化为 0，让没接雷达时也能正常解锁起飞，不报错。
  4) 雷达插拔后【无需重启】：
     - 插着 → init() 已经设好 num_instances=1，handle_measurement() 收到第一帧
       Proximity 报文就懒加载 drivers[0]，避障当场上线。
     - 拔了 → drivers[0]=nullptr，valid_instance()=false，update() 跳过该实例，
       不报错也不避障，可以正常起飞。
     - 重新插回 → 数据一到，懒加载触发，避障立刻恢复。
     唯一需要重启的场景：首次把 PRX1_TYPE 从 0 写成 14 那一次（脚本会打
     "REBOOT to enable avoidance" 提醒）。
  5) EEPROM 友好：所有 set_and_save 都先比较"目标值 vs 当前值"，相同则跳过，
     避免反复擦写。
  6) 处理 ARMING_CHECK 的 ALL(bit0) 总开关陷阱：bit0=1 时单独清 bit15 无效，
     脚本会先把 ALL 展开为 bits 1..19 显式位、再清 bit15。
  7) 解锁后立即冻结脚本（check_done=true），飞行中绝不再动任何参数。

  不替你解决的事（外部依赖，重启再多次也救不回来）：
   - RADAR_NODE_ID 必须与雷达真实 Node ID 一致；填错则永远 radar_seen=false。
   - PRX1_ADDR 必须与雷达上报的 sensor_id 匹配，否则 get_dronecan_backend 不
     创建 driver、数据被丢弃。
   - 避障总开关：AVOID_ENABLE、OA_TYPE、FENCE_TYPE（视需要含 bit4）必须自行
     一次性配好；脚本不管这些。
   - 飞行模式需支持避障（Loiter / AltHold / PosHold / Auto 等）。
   - 雷达启动慢于 BOOT_DELAY_MS 时，会被脚本判成不在线；PRX1_TYPE 不会被误清，
     但 ARMING_CHECK bit15 会被临时 disable（不影响避障实际生效，下次重启如雷达
     30s 内就绪会自动改回）。雷达启动确实很慢的话把 BOOT_DELAY_MS 调大。

  v4.0 与 v3.0 的根本差异
  -----------------------
  v3.0 用 prx_param:set() 仅写 RAM，期望「雷达开始发数据就懒加载 driver」。但是：
    AP_Proximity::init() 只在开机时按【EEPROM 里】的 PRX1_TYPE 决定 num_instances；
    AP_Proximity::update() 的迭代上限就是 num_instances。
    所以即便后续 RAM 里把 PRX1_TYPE 改成 14、雷达也发了数据、driver 也被
    handle_measurement() 懒加载出来，drivers[i]->update() 永远不会被 update() 循环
    调到，items 队列里的雷达点云从来不被消费，OA boundary / database 拿不到数据，
    表现为「DroneCAN 监视器里有数据，但飞控不避障」。

  v4.0 改成写 EEPROM 持久化，规避这个时序坑：
    - PRX1_TYPE 一旦判定为 14 就 set_and_save(14)，让下次开机 init() 拿到正确值，
      num_instances 设到 1，懒加载 + update() 全链路打通。
    - 首次写 EEPROM 时（原值不是 14）必须重启一次才生效，脚本会给出明确提示。
    - 没检测到雷达时【不动 PRX1_TYPE】，保持 EEPROM 里的 14，下次启动雷达回来直接用，
      不再需要重启第二次。
    - ARMING_CHECK bit 15（Rangefinder）按当前是否看到雷达 set_and_save，
      让「拔雷达 → 重启 → 仍能解锁起飞」这个状态跨重启稳定。
    - 所有 set_and_save 都只在「目标值 != 当前值」时才执行，避免反复擦写 EEPROM。

  行为时序
  --------
  开机 0..BOOT_DELAY_MS 内：监听 DroneCAN NodeStatus，看 RADAR_NODE_ID 是否上线。
  到 BOOT_DELAY_MS：
    radar_seen=true  → 把 PRX1_TYPE 持久化为 14；ARMING_CHECK bit15 → 1
    radar_seen=false → 不动 PRX1_TYPE；ARMING_CHECK bit15 → 0
  之后脚本就停了，已解锁也直接停。

  ALL(bit0) 展开逻辑保留：bit0=1 时单独清其他位无效，先把 bit0 展开为 bits 1..19。

  依赖：CAN_Px_DRIVER=1 (DroneCAN)；SCR_ENABLE=1。
--]]

local TARGET_PRX_TYPE = 14
local RADAR_NODE_ID = 120 -- 【必改】地面站 UAVCAN 里看到的雷达 Node ID

local prx_param = Parameter("PRX1_TYPE")
local arming_check_param = Parameter("ARMING_CHECK")

local ARMING_CHECK_ALL_BIT  = 1 << 0
local ARMING_CHECK_RNGFND   = 1 << 15
local ARMING_CHECK_EXPANDED = 0xFFFFE

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

--- 把 PRX1_TYPE 持久化到 EEPROM；返回 ok, changed
-- changed=true 表示真的写了 EEPROM（首次启用，需要重启避障才能生效）
local function ensure_prx_type_persistent(v)
    local cur = prx_param:get()
    if cur == nil then
        gcs:send_text(4, "param get PRX1_TYPE err")
        return false, false
    end
    if math.floor(cur) == v then
        return true, false
    end
    if not prx_param:set_and_save(v) then
        gcs:send_text(4, "param set_and_save PRX1_TYPE err")
        return false, false
    end
    return true, true
end

--- 把 ARMING_CHECK 的 Rangefinder 位（bit15）持久化为 enable
-- enable=true  → 置 1（雷达在线，恢复测距仪/避障解锁前检查）
-- enable=false → 置 0（无雷达，跳过测距仪检查，避免无雷达无法解锁）
-- 若原值 ALL(bit0)=1 且要 disable：先把 ALL 展开为 bits 1..19 再清 bit15
local function ensure_arming_rangefinder_check(enable)
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

    if not arming_check_param:set_and_save(target) then
        gcs:send_text(4, "param set_and_save ARMING_CHECK err")
        return false
    end
    gcs:send_text(6, string.format("ARMING_CHECK saved: %d -> %d", original, target))
    return true
end

function update()
    if check_done then
        return
    end

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

    if millis() > BOOT_DELAY_MS then
        if radar_seen then
            local ok, changed = ensure_prx_type_persistent(TARGET_PRX_TYPE)
            if ok and changed then
                gcs:send_text(4, "PRX1_TYPE saved=14, REBOOT to enable avoidance")
            elseif ok then
                gcs:send_text(6, "PRX1_TYPE already 14, avoidance live")
            end
            if ensure_arming_rangefinder_check(true) then
                gcs:send_text(6, "ARMING_CHECK: Rangefinder enabled (radar seen)")
            end
        else
            -- 不动 PRX1_TYPE：保留 EEPROM 里的设置，下次启动雷达回来直接生效
            gcs:send_text(6, "No radar seen, keep PRX1_TYPE in EEPROM")
            if ensure_arming_rangefinder_check(false) then
                gcs:send_text(6, "ARMING_CHECK: Rangefinder disabled (no radar)")
            end
        end

        check_done = true
        return
    end

    return update, 100
end

return update, 3000
