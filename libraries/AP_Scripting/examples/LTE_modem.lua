-- LTE_modem.lua  v3.0-dev
-- 基于 v2.0；v2.0 修复了 MQTT 解析 Bug、GPS 时间戳、UOM 激活全流程
-- v3.0 TODO：（在此列出下一步改进目标）
--   [ ] 遥测 JSON 增加电池/飞行模式字段（按云端协议扩展）
--   [ ] 断线重连后自动重新激活（MQTT Keep-Alive 优化）
--   [ ] 支持可配置 APN（当前写死 cmnet）
--[[
    ========================================================================
    UOM 云平台 · 五个 ID 说明（对接文档 2.3 变量说明，请以此为准）
    ========================================================================

    【vendor_id】
    - 云平台用来区分厂商的字符串，本方案固定为 "eft"。
    - 不出现在遥测/激活 JSON 里，而是写在 MQTT 主题路径中，例如：
        uav/up/telemetry/eft/{fcu_id}
        uav/up/activation/eft/{fcu_id}
        uav/down/activation/eft/{fcu_id}
    - 脚本内写死（uom.TOPIC_* 中的 "eft" 段）；换厂商需改主题模板并与平台约定一致。

    【fcu_id】
    - 单台飞控在 UOM 平台上的主键；激活请求仅含 fcu_id + timestamp。
    - 脚本实现：fcu_id = 飞控sn码。
    - 同时作为 MQTT Client ID、订阅/发布主题里的设备后缀。

    【uas_id】
    - 国标 Remote ID「航空器 ID」，最多 20 字符 ASCII。
    - Mission Planner 经 MAVLink 下发 OPEN_DRONE_ID_BASIC_ID 后写入 LTE_UAS_W01~10 并掉电保存。

    【operator_id】
    - 国标 Remote ID「运营人 ID」，最多 20 字符。
    - 遥测 JSON 字段 "operator_id"。

    【user_id】
    - 云平台遥测里的用户/声明字段，非 ICAO 强制 ID。
    - 优先：OPEN_DRONE_ID_SELF_ID 的 description（自我声明）；
      否则：参数 LTE_USER_ID 的整数值转字符串；均为空则 "0"。
    相关但非 ID：op_lat / op_lng / op_alt（操控员位置，来自 SYSTEM / SYSTEM_UPDATE）。

    ========================================================================
    driver for LTE modem (合宙 Air780E / Air780 only)

    EFT_CAAC 接线约定（无需改 LTE_SERPORT / LTE_SCRPORT）：
    - 4G 接飞控 SERIAL1（MP 里 Serial Port 1 / UART1），SERIAL1_PROTOCOL = Scripting(28)
    - SCR_SDEV1_PROTO = PPP(48)，NET_ENABLE = 1
    - UOM：仅需 LTE_UOM_IP0~3 + PORT（默认已填云平台地址），LTE_UOM_ENABLE 默认开
    - 禁飞区：PPP CONNECT 后写 APM/lte_ppp_ready.flag=1，供 1noflyzone_checker.lua 拉 HTTP
    - UOM MQTT down code=6：PreArm 禁止解锁；飞入禁飞区 GCS 告警；在飞时强制 RTL
--]]

local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}

local PARAM_TABLE_KEY = 106
local PARAM_TABLE_PREFIX = "LTE_"

-- local MAVLINK2 = 2
local PPP = 48

-- 固定 SERIAL1：Scripting 口列表里第 1 个（索引 0），SCR_SDEV1 PPP 也是索引 0
local LTE_SERPORT_FIXED = 0
local LTE_SCRPORT_FIXED = 0
-- 默认 APN（移动/通用 cmnet）；联通卡可改脚本为 3gnet，电信 ctnet
local LTE_APN_DEFAULT = "cmnet"

-- 与 1noflyzone_checker.lua 共享：PPP 数据链路就绪（'1'）/ 未就绪（'0'）
local LTE_PPP_READY_FILE = "APM/lte_ppp_ready.flag"
local lte_ppp_ready_state = nil
-- CONNECT 后延迟置 flag，避免底层 PPP 尚未协商完就触发禁飞脚本 HTTP
local PPP_NFZ_FLAG_DELAY_MS = 15000
local ppp_connected_ms = nil

-- add a parameter and bind it to a variable
local function bind_add_param(name, idx, default_value)
    assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format('could not add param %s', name))
    return Parameter(PARAM_TABLE_PREFIX .. name)
end

-- Setup Parameters
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 52), 'LTE_modem: could not add param table')

--[[
    // @Param: LTE_ENABLE
    // @DisplayName: LTE Enable
    // @Description: Enable or disable the LTE modem driver
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
--]]
local LTE_ENABLE = bind_add_param('ENABLE',  1, 1)

--[[
    // @Param: LTE_SERPORT
    // @DisplayName: Serial Port
    // @Description: Serial port to use for the LTE modem. This is the index of the SERIALn_ ports that are set to 28 for "scripting"
    // @Range: 0 8
    // @User: Standard
--]]
local LTE_SERPORT = bind_add_param('SERPORT',  2, 0)

--[[
    // @Param: LTE_SCRPORT
    // @DisplayName: Scripting Serial Port
    // @Description: Scripting Serial port to use for the LTE modem. This is the index of the SCR_SDEV ports that are set to 2 for "MAVLink2"
    // @Range: 0 8
    // @User: Standard
--]]
local LTE_SCRPORT = bind_add_param('SCRPORT',  3, 0)

--[[
    // @Param: LTE_SERVER_IP0
    // @DisplayName: Server IP 0
    // @Description: First octet of the server IP address to connect to
    // @Range: 0 255
    // @User: Standard
--]]
local LTE_SERVER_IP0  = bind_add_param('SERVER_IP0',  4, 0)

--[[
    // @Param: LTE_SERVER_IP1
    // @DisplayName: Server IP 1
    // @Description: Second octet of the server IP address to connect to
    // @Range: 0 255
    // @User: Standard
--]]
local LTE_SERVER_IP1  = bind_add_param('SERVER_IP1',  5, 0)

--[[
    // @Param: LTE_SERVER_IP2
    // @DisplayName: Server IP 2
    // @Description: Third octet of the server IP address to connect to
    // @Range: 0 255
    // @User: Standard
--]]
local LTE_SERVER_IP2  = bind_add_param('SERVER_IP2',  6, 0)

--[[
    // @Param: LTE_SERVER_IP3
    // @DisplayName: Server IP 3
    // @Description: Fourth octet of the server IP address to connect to
    // @Range: 0 255
    // @User: Standard
--]]
local LTE_SERVER_IP3  = bind_add_param('SERVER_IP3',  7, 0)

--[[
    // @Param: LTE_SERVER_PORT
    // @DisplayName: Server Port
    // @Description: IPv4 Port of the server to connect to
    // @Range: 1 65525
    // @User: Standard
--]]
local LTE_SERVER_PORT = bind_add_param('SERVER_PORT',  8, 0)

--[[
    // @Param: LTE_BAUD
    // @DisplayName: Serial Baud Rate
    // @Description: Baud rate for the serial port to the LTE modem when connected. Initial power on baudrate is in LTE_IBAUD
    // @Values: 19200:19200,38400:38400,57600:57600,115200:115200,230400:230400,460800:460800,921600:921600,3686400:3686400
    // @User: Standard
--]]
local LTE_BAUD        = bind_add_param('BAUD',  9, 115200)

--[[
    // @Param: LTE_TIMEOUT
    // @DisplayName: Timeout
    // @Description: Timeout in seconds for the LTE connection. If no data is received for this time, the connection will be reset. A value of zero disables the timeout
    // @Range: 0 60
    // @Units: s
    // @User: Standard
--]]
local LTE_TIMEOUT     = bind_add_param('TIMEOUT', 10, 10)

--[[
    // @Param: LTE_PROTOCOL
    // @DisplayName: LTE protocol
    // @Description: The protocol that we will use in communication with the LTE modem. If this is PPP then the LTE_SERVER parameters are not used and instead a PPP connection will be established and you should use the NET_ parameters to enable network ports. If this is MAVLink2 then the LTE_SERVER parameters are used to create a TCP or UDP connection to a single server.
    // @Values: 2:MavLink2,48:PPP
    // @User: Standard
--]]
local LTE_PROTOCOL     = bind_add_param('PROTOCOL', 11, 48)

--[[
    写入 PPP 就绪标志，供禁飞区脚本读取（见 examples/1noflyzone_checker.lua）。
    须放在 LTE_PROTOCOL 定义之后，否则 Lua 会把 LTE_PROTOCOL 当全局 nil。
    ready=true 仅在 LTE_PROTOCOL=PPP(48) 且模组侧 PPP 已 CONNECT 满延迟后置位。
--]]
local function write_lte_ppp_ready_flag(ready)
    if ready and LTE_PROTOCOL:get() ~= PPP then
        return
    end
    if lte_ppp_ready_state == ready then
        return
    end
    local f = io.open(LTE_PPP_READY_FILE, 'w')
    if not f then
        if ready then
            gcs:send_text(MAV_SEVERITY.WARNING,
                'LTE: 无法写入 ' .. LTE_PPP_READY_FILE)
        end
        return
    end
    f:write(ready and '1' or '0')
    f:close()
    lte_ppp_ready_state = ready
    if ready then
        -- PPP 就绪不写 GCS（禁飞脚本读 flag 即可）
    end
end

--[[
    // @Param: LTE_OPTIONS
    // @DisplayName: LTE options
    // @Description: Options to control the LTE modem driver. If VerboseSignalInfoGCS is set then additional NAMED_VALUE_FLOAT values are sent with verbose signal information
    // @Bitmask: 0:LogAllData,1:VerboseSignalInfoGCS,2:DisableMultiplexing,3:DisableSignalQueries,4:UseTCP
    // @User: Standard
--]]
local LTE_OPTIONS     = bind_add_param('OPTIONS', 12, 0)

--[[
    // @Param: LTE_IBAUD
    // @DisplayName: LTE initial baudrate
    // @Description: This is the initial baud rate on power on for the modem. This is set in the modem with the AT+IREX=baud command
    // @Values: 19200:19200,38400:38400,57600:57600,115200:115200,230400:230400,460800:460800,921600:921600,3686400:3686400
    // @User: Standard
--]]
local LTE_IBAUD       = bind_add_param('IBAUD', 13, 115200)

--[[
    // @Param: LTE_MCCMNC
    // @DisplayName: LTE operator selection
    // @Description: This allows selection of network operator
    // @Values: -1:NoChange,0:Default,AU-Telstra:50501,AU-Optus:50502,AU-Vodaphone:50503
    // @User: Standard
--]]
local LTE_MCCMNC      = bind_add_param('MCCMNC', 14, -1)

local supports_routing = networking and networking.add_route -- luacheck: ignore 143

if supports_routing then
    -- add_route() API only on newer firmwares
    --[[
        // @Param: LTE_ROUTE_IP0
        // @DisplayName: custom route IP 0
        // @Description: First octet of the custom route IP address
        // @Range: 0 255
        // @User: Standard
    --]]
    LTE_ROUTE_IP0  = bind_add_param('ROUTE_IP0',  15, 0)

    --[[
        // @Param: LTE_ROUTE_IP1
        // @DisplayName: custom route IP 1
        // @Description: Second octet of the custom route IP address
        // @Range: 0 255
        // @User: Standard
    --]]
    LTE_ROUTE_IP1  = bind_add_param('ROUTE_IP1',  16, 0)

    --[[
        // @Param: LTE_ROUTE_IP2
        // @DisplayName: custom route IP 2
        // @Description: Third octet of the custom route IP address
        // @Range: 0 255
        // @User: Standard
    --]]
    LTE_ROUTE_IP2  = bind_add_param('ROUTE_IP2',  17, 0)

    --[[
        // @Param: LTE_ROUTE_IP3
        // @DisplayName: custom route IP 3
        // @Description: Fourth octet of the custom route IP address
        // @Range: 0 255
        // @User: Standard
    --]]
    LTE_ROUTE_IP3  = bind_add_param('ROUTE_IP3',  18, 0)

    --[[
        // @Param: LTE_ROUTE_MASK
        // @DisplayName: custom route netmask length
        // @Description: number of bits in route netmask. Use 32 for a single IP
        // @Range: 0 32
        // @User: Standard
    --]]
    LTE_ROUTE_MASK  = bind_add_param('ROUTE_MASK',  19, 32)
end

--[[
    // @Param: LTE_TX_RATE
    // @DisplayName: Max transmit rate
    // @Description: Maximum data transmit rate to the modem in bytes/second. Use zero for unlimited
    // @User: Standard
--]]
local LTE_TX_RATE  = bind_add_param('TX_RATE',  20, 0)

--[[
    // @Param: LTE_BAND
    // @DisplayName: LTE band selection
    // @Description: This allows selection of LTE band. A value of -1 means no band setting change is made. A value of 0 sets all bands. Otherwise the specified band is set.
    // @Range: -1 50
    // @User: Standard
--]]
local LTE_BAND      = bind_add_param('BAND', 21, -1)

--[[
    // @Param: LTE_UOM_ENABLE
    // @DisplayName: UOM MQTT reporting enable
    // @Description: Enable UOM telemetry reporting via MQTT over PPP
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
--]]
LTE_UOM_ENABLE = bind_add_param('UOM_ENABLE', 22, 1)

--[[
    // @Param: LTE_UOM_IP0
    // @DisplayName: UOM MQTT server IP octet 0
    // @Range: 0 255
    // @User: Standard
--]]
LTE_UOM_IP0 = bind_add_param('UOM_IP0', 23, 47)

--[[
    // @Param: LTE_UOM_IP1
    // @DisplayName: UOM MQTT server IP octet 1
    // @Range: 0 255
    // @User: Standard
--]]
LTE_UOM_IP1 = bind_add_param('UOM_IP1', 24, 120)

--[[
    // @Param: LTE_UOM_IP2
    // @DisplayName: UOM MQTT server IP octet 2
    // @Range: 0 255
    // @User: Standard
--]]
LTE_UOM_IP2 = bind_add_param('UOM_IP2', 25, 16)

--[[
    // @Param: LTE_UOM_IP3
    // @DisplayName: UOM MQTT server IP octet 3
    // @Range: 0 255
    // @User: Standard
--]]
LTE_UOM_IP3 = bind_add_param('UOM_IP3', 26, 113)

--[[
    // @Param: LTE_UOM_PORT
    // @DisplayName: UOM MQTT server port
    // @Range: 1 65535
    // @User: Standard
--]]
LTE_UOM_PORT = bind_add_param('UOM_PORT', 27, 1883)

--[[
    // @Param: LTE_USER_ID
    // @DisplayName: UOM MQTT user ID
    // @Description: User ID for UOM MQTT JSON (RID self-declaration, 0=unset)
    // @Range: 0 4294967295
    // @User: Standard
--]]
local LTE_USER_ID = bind_add_param('USER_ID', 28, 0)

-- 飞行器/操作员 ID 各 20 字符，拆成 10 个参数（每参数 2 字节 ASCII，float 存储）
local LTE_UAS_W = {}
local LTE_OP_W = {}
for i = 1, 10 do
    LTE_UAS_W[i] = bind_add_param(string.format('UAS_W%02u', i), 28 + i, 0)
    LTE_OP_W[i]  = bind_add_param(string.format('OP_W%02u', i), 38 + i, 0)
end

--[[
    // @Param: LTE_OP_LAT
    // @DisplayName: Operator latitude
    // @Description: Persisted operator latitude (deg) from GCS OpenDroneID
    // @User: Standard
--]]
local LTE_OP_LAT = bind_add_param('OP_LAT', 49, 0)

--[[
    // @Param: LTE_OP_LNG
    // @DisplayName: Operator longitude
    // @Description: Persisted operator longitude (deg) from GCS OpenDroneID
    // @User: Standard
--]]
local LTE_OP_LNG = bind_add_param('OP_LNG', 50, 0)

--[[
    // @Param: LTE_OP_ALT
    // @DisplayName: Operator altitude
    // @Description: Persisted operator geodetic altitude (m) from GCS OpenDroneID
    // @User: Standard
--]]
local LTE_OP_ALT = bind_add_param('OP_ALT', 51, 0)

LTE_OPTIONS_LOGALL  = (1<<0)
LTE_OPTIONS_SIGNALS = (1<<1)
LTE_OPTIONS_NOMUX   = (1<<2)
LTE_OPTIONS_NOSIGQUERY = (1<<3)
LTE_OPTIONS_TCP = (1<<4)

-- ============================================================
-- UOM 无人机运营管理系统 MQTT 上报模块
-- 五个 ID：见本文件最顶部注释表（vendor_id / fcu_id / uas_id / operator_id / user_id）
-- 在 PPP 连通（step=CONNECTED）后自动启动
-- 配置：LTE_UOM_ENABLE=1, LTE_UOM_IP0~3, LTE_UOM_PORT
-- ODID：地面站经任意 MAVLink 口（如 SERIAL6）下发 OpenDroneID 消息后写入 LTE_* 参数持久化
-- 主题（vendor_id=eft 为路径固定段，fcu_id 为后缀）:
--   激活  uav/up/activation/eft/{fcu_id}   QoS1
--   响应  uav/down/activation/eft/{fcu_id} QoS1 SUBSCRIBE
--   遥测  uav/up/telemetry/eft/{fcu_id}    QoS0
-- ============================================================

-- UOM/ODID 模块独立作用域，避免主 chunk local 超过 100 上限
uom_update = (function()

-- MQTT Broker 认证（云平台提供，修改后需重新拷贝脚本到 SD 卡）
local UOM_MQTT_USER = "yifeite"
local UOM_MQTT_PASS = "eftMqtt!qwer"

-- ---- ODID 运行时缓存 ----
local odid = {
    uas_id       = "",
    operator_id  = "",
    self_desc    = "",   -- 自我声明内容（OPEN_DRONE_ID_SELF_ID.description）
    op_lat       = 0.0,
    op_lng       = 0.0,
    op_alt       = 0.0,
}

-- MAVLink OpenDroneID 消息 ID（ArduPilot 所有串口收到的消息均会转发给 Lua）
local MSGID_BASIC_ID       = 12900
local MSGID_SELF_ID        = 12903   -- OPEN_DRONE_ID_SELF_ID（自我声明）
local MSGID_SYSTEM         = 12904
local MSGID_OPERATOR_ID    = 12905
local MSGID_SYSTEM_UPDATE  = 12919

-- 将 2 个 ASCII 字符打包为 float 参数（避免 float32 精度问题，不用 4 字节打包）
local function odid_pack2chars(s, pos)
    local b1, b2 = s:byte(pos, pos + 1)
    if not b1 then return 0 end
    if not b2 then b2 = 0 end
    return b1 + b2 * 256
end

local function odid_unpack2chars(v)
    v = math.floor(v + 0.5)
    if v <= 0 then return "" end
    local b1 = v % 256
    local b2 = math.floor(v / 256) % 256
    if b2 == 0 then return string.char(b1) end
    return string.char(b1, b2)
end

-- 从 LTE_UAS_W / LTE_OP_W 参数组还原字符串（最多 20 字符）
local function odid_words_to_str(words)
    local s = ""
    for i = 1, #words do
        s = s .. odid_unpack2chars(words[i]:get())
    end
    return (s:match("^([^%z]*)") or ""):match("^%s*(.-)%s*$") or ""
end

-- 字符串写入参数组并掉电保存（仅内容变化时写入，减少闪存磨损）
local function odid_str_to_words(s, words)
    s = s or ""
    for i = 1, #words do
        local v = odid_pack2chars(s, (i - 1) * 2 + 1)
        if math.floor(words[i]:get() + 0.5) ~= v then
            words[i]:set_and_save(v)
        end
    end
end

-- 操作员位置是否有效
-- MAVLink 规范：lat/lng 同时为 0 表示未知
-- 用 OR 条件：lat 或 lng 任一接近 0 均视为无效，防止 MP 未设地面 GPS 时
-- 只有 lng 非零（如 53.89°）而 lat=0 的错误坐标被接受
local function odid_op_loc_valid(lat, lng, alt)
    if not lat or not lng then return false end
    if math.abs(lat) < 0.001 or math.abs(lng) < 0.001 then return false end
    if alt and alt <= -999 then return false end
    return true
end

-- 从参数加载 ODID 缓存（脚本启动时调用）
local function odid_load_from_params()
    odid.uas_id = odid_words_to_str(LTE_UAS_W)
    odid.operator_id = odid_words_to_str(LTE_OP_W)
    odid.op_lat = LTE_OP_LAT:get()
    odid.op_lng = LTE_OP_LNG:get()
    odid.op_alt = LTE_OP_ALT:get()
end

local function odid_save_uas_id(s)
    if not s or s == "" then return end
    if odid.uas_id ~= s then
        -- 静默缓存，不写 GCS
    end
    odid.uas_id = s
    odid_str_to_words(s, LTE_UAS_W)
end

local function odid_save_operator_id(s)
    if not s or s == "" then return end
    if odid.operator_id ~= s then
        -- 静默缓存，不写 GCS
    end
    odid.operator_id = s
    odid_str_to_words(s, LTE_OP_W)
end

local function odid_save_op_location(lat, lng, alt)
    if not odid_op_loc_valid(lat, lng, alt) then return end
    if odid.op_lat ~= lat or odid.op_lng ~= lng then
        -- 静默缓存，不写 GCS
    end
    odid.op_lat = lat
    odid.op_lng = lng
    odid.op_alt = alt
    if math.abs(LTE_OP_LAT:get() - lat) > 1e-7 then
        LTE_OP_LAT:set_and_save(lat)
    end
    if math.abs(LTE_OP_LNG:get() - lng) > 1e-7 then
        LTE_OP_LNG:set_and_save(lng)
    end
    if math.abs(LTE_OP_ALT:get() - alt) > 0.01 then
        LTE_OP_ALT:set_and_save(alt)
    end
end

-- 上报用有效值：运行时缓存优先，否则读持久化参数
local function odid_get_uas_id()
    if odid.uas_id ~= "" then return odid.uas_id end
    return odid_words_to_str(LTE_UAS_W)
end

local function odid_get_operator_id()
    if odid.operator_id ~= "" then return odid.operator_id end
    return odid_words_to_str(LTE_OP_W)
end

-- home 位置辅助：三个 getter 共用，避免重复调用 ahrs:get_home()
-- 返回 (lat_deg, lng_deg, alt_m) 或 (nil, nil, nil)
local function odid_home_loc()
    local home = ahrs:get_home()
    if not home then return nil, nil, nil end
    local hlat = home:lat() * 1e-7
    local hlng = home:lng() * 1e-7
    local halt = home:alt() * 0.01  -- cm → m
    if odid_op_loc_valid(hlat, hlng, halt) then
        return hlat, hlng, halt
    end
    return nil, nil, nil
end

local function odid_get_op_lat()
    if odid_op_loc_valid(odid.op_lat, odid.op_lng, odid.op_alt) then
        return odid.op_lat
    end
    local lat = LTE_OP_LAT:get()
    local lng = LTE_OP_LNG:get()
    local alt = LTE_OP_ALT:get()
    if odid_op_loc_valid(lat, lng, alt) then return lat end
    -- 最终回退：使用 home（解锁前 GPS 定位，即起飞位置）
    local hlat = odid_home_loc()
    if hlat then return hlat end
    return 0.0
end

local function odid_get_op_lng()
    if odid_op_loc_valid(odid.op_lat, odid.op_lng, odid.op_alt) then
        return odid.op_lng
    end
    local lat = LTE_OP_LAT:get()
    local lng = LTE_OP_LNG:get()
    local alt = LTE_OP_ALT:get()
    if odid_op_loc_valid(lat, lng, alt) then return lng end
    local _, hlng = odid_home_loc()
    if hlng then return hlng end
    return 0.0
end

local function odid_get_op_alt()
    if odid_op_loc_valid(odid.op_lat, odid.op_lng, odid.op_alt) then
        return odid.op_alt
    end
    local lat = LTE_OP_LAT:get()
    local lng = LTE_OP_LNG:get()
    local alt = LTE_OP_ALT:get()
    if odid_op_loc_valid(lat, lng, alt) then return alt end
    local _, _, halt = odid_home_loc()
    if halt then return halt end
    return 0.0
end

-- 解析 ArduPilot 传入的 mavlink_message_t 二进制结构（非串口线格式）
-- 布局见 modules/MAVLink/mavlink_msgs.lua decode_header()
local function mavlink_c_payload(msg_str)
    if not msg_str or #msg_str < 13 then return nil, nil end
    local magic = msg_str:byte(3)
    if magic ~= 0xFD and magic ~= 0xFE then return nil, nil end
    local payload_len = msg_str:byte(4)
    local id = msg_str:byte(10) | (msg_str:byte(11) << 8) | (msg_str:byte(12) << 16)
    if payload_len <= 0 or #msg_str < 12 + payload_len then return nil, nil end
    return id, msg_str:sub(13, 12 + payload_len)
end

-- 注册 ODID 消息接收（所有 MAVLink 通道，含 SERIAL6 地面站）
local function odid_mavlink_setup()
    mavlink:init(10, 5)  -- 5 种消息类型
    local ok, err = pcall(function()
        mavlink:register_rx_msgid(MSGID_BASIC_ID)
        mavlink:register_rx_msgid(MSGID_SELF_ID)
        mavlink:register_rx_msgid(MSGID_SYSTEM)
        mavlink:register_rx_msgid(MSGID_OPERATOR_ID)
        mavlink:register_rx_msgid(MSGID_SYSTEM_UPDATE)
    end)
    if ok then
        -- 注册成功不提示
    else
        gcs:send_text(MAV_SEVERITY.WARNING, string.format("ODID: reg FAIL: %s", tostring(err)))
    end
end
odid_mavlink_setup()
odid_load_from_params()

-- 轮询 MAVLink 收件箱，更新 ODID 缓存并持久化到 LTE_* 参数
-- 注意：MAVLink v2 会自动裁剪末尾零字节（Trailing Zero Trimming），
--       char[20] 类字段（uas_id / operator_id）若未填满则包比标称长度短，
--       必须用补零再解包，或直接用 sub 读字符串字段，不能用硬性长度下限。
local function odid_poll()
    local n = 0
    while n < uom.ODID_POLL_MAX_MSGS do
        n = n + 1
        local msg_str, _chan = mavlink:receive_chan()
        if not msg_str then break end
        local id, buf = mavlink_c_payload(msg_str)
        if not id or not buf then break end

        if id == MSGID_BASIC_ID then
            -- uas_id = bytes 25~44；末尾零已被 MAVLink v2 裁剪，直接取子串
            -- 字段顺序: target_sys(1) target_comp(1) id_or_mac(20) id_type(1) ua_type(1) uas_id(20)
            if #buf >= 24 then
                local uas_raw = buf:sub(25):match("^([^%z]*)") or ""
                if uas_raw ~= "" then odid_save_uas_id(uas_raw) end
            end

        elseif id == MSGID_SELF_ID then
            -- 自我声明：description = bytes 24~46；末尾零已被 MAVLink v2 裁剪
            -- 字段顺序: target_sys(1) target_comp(1) id_or_mac(20) desc_type(1) description(23)
            if #buf >= 23 then
                local desc = buf:sub(24):match("^([^%z]*)") or ""
                if desc ~= "" and odid.self_desc ~= desc then
                    -- self_desc 静默缓存
                    odid.self_desc = desc
                end
            end

        elseif id == MSGID_OPERATOR_ID then
            -- operator_id = bytes 24~43；末尾零已被 MAVLink v2 裁剪，直接取子串
            -- 字段顺序: target_sys(1) target_comp(1) id_or_mac(20) op_id_type(1) operator_id(20)
            if #buf >= 23 then
                local op_raw = buf:sub(24):match("^([^%z]*)") or ""
                if op_raw ~= "" then odid_save_operator_id(op_raw) end
            end

        elseif id == MSGID_SYSTEM then
            -- MAVLink 线格式按字段大小降序排列（4字节在前，1字节在后）：
            -- op_lat(i4) op_lng(i4) area_ceiling(f) area_floor(f) op_alt(f)
            -- timestamp(I4) area_count(H) area_radius(H)
            -- target_sys(B) target_comp(B) id_or_mac(c20) ...
            -- op_lat 在 bytes 1-4，op_lng 在 bytes 5-8，op_alt 在 bytes 17-20
            if #buf >= 8 then
                local full = buf .. string.rep("\0", math.max(0, 20 - #buf))
                local olat, olng, _, _, oalt = string.unpack("<i4i4fff", full)
                odid_save_op_location(olat * 1e-7, olng * 1e-7, oalt)
            end

        elseif id == MSGID_SYSTEM_UPDATE then
            -- MAVLink 线格式：op_lat(i4) op_lng(i4) op_alt(f) timestamp(I4) target_sys(B) target_comp(B)
            -- op_lat 在 bytes 1-4，op_lng 在 bytes 5-8，op_alt 在 bytes 9-12
            if #buf >= 8 then
                local full = buf .. string.rep("\0", math.max(0, 12 - #buf))
                local olat, olng, oalt = string.unpack("<i4i4f", full)
                odid_save_op_location(olat * 1e-7, olng * 1e-7, oalt)
            end
        end
    end
end

uom = {}
uom.sock                  = nil
uom.state                 = "IDLE"
uom.last_pub_ms           = uint32_t(0)
uom.last_ping_ms          = uint32_t(0)
uom.connect_timeout_ms    = uint32_t(0)
uom.activate_timeout_ms   = uint32_t(0)
uom.TOPIC_TELEMETRY       = "uav/up/telemetry/eft/%s"
uom.TOPIC_ACT_UP          = "uav/up/activation/eft/%s"
uom.TOPIC_ACT_DOWN        = "uav/down/activation/eft/%s"
uom.publish_count         = 0
uom.connect_sent          = false
uom.subscribe_sent        = false
uom.activation_sent       = false
uom.activated             = false
uom.activation_code       = -1
uom.activation_send_after_ms = uint32_t(0)
uom.packet_id             = 1
uom.rx_buf                = ""
uom.retry_after_ms        = uint32_t(0)
uom.PING_MS               = 30000
uom.REPORT_MS             = 1000
uom.ACTIVATE_TIMEOUT_MS   = 30000  -- 等待激活 down 响应（云端接口约 3s，留足蜂窝 RTT 余量）
uom.ACTIVATE_RETRY_MS     = 8000   -- 超时后再次发送 up 激活的最小间隔
uom.SUBACK_TIMEOUT_MS     = 5000   -- 未收到 SUBACK 时仍进入激活阶段（避免卡死）
uom.subscribe_sent_ms     = uint32_t(0)
uom.RX_BUF_MAX            = 2048   -- 接收缓冲上限
uom.MQTT_RECV_MAX_CHUNKS  = 2      -- 单次最多读 2 个 TCP 块（防 exceeded time limit）
uom.MQTT_DISPATCH_MAX_PKTS = 4      -- 单次最多解析 4 个 MQTT 包
uom.TICK_MS               = 50      -- UOM 主逻辑最短周期（step_CONNECTED 为 5ms 时不每拍跑满）
uom.ODID_POLL_MS          = 250     -- ODID MAVLink 轮询周期
uom.ODID_POLL_MAX_MSGS    = 2       -- 每次 ODID 最多处理条数
uom.last_tick_ms          = uint32_t(0)
uom.last_odid_poll_ms     = uint32_t(0)
uom.gps_warn_sent         = false
uom.subscribed_fcu_id     = ""      -- 已订阅的 down 主题后缀，须与当前 fcu_id 一致
uom.mqtt_connected_ms     = uint32_t(0)
uom.FCU_ID_MIN_LEN        = 12      -- 完整 UAS ID 长度不足时不订阅（避免 EFT26 与 EFT2605210001 不一致）
uom.id_wait_warn_sent     = false
uom.auth_id               = arming:get_aux_auth_id()  -- UOM 禁飞区解锁鉴权（与 NFZ 脚本独立）
uom.in_nfz_zone           = false   -- 云平台 MQTT code=6：在禁飞区内/接近禁飞区
local UOM_NFZ_ARming_MSG  = "UOM：禁飞区内禁止解锁"
-- ArduCopter 飞行模式号（强制返航用）
local COPTER_MODE_RTL       = 6
local COPTER_MODE_LAND      = 9
local COPTER_MODE_SMART_RTL = 21
local COPTER_MODE_AUTO_RTL  = 27

-- 是否已在返航/降落类模式（避免重复 set_mode）
local function uom_mode_is_rtl_or_land(mode)
    return mode == COPTER_MODE_RTL
        or mode == COPTER_MODE_LAND
        or mode == COPTER_MODE_SMART_RTL
        or mode == COPTER_MODE_AUTO_RTL
end

-- 与 AP_GPS::istate_time_to_epoch_ms 一致（libraries/AP_GPS/AP_GPS.h）
local AP_MSEC_PER_WEEK    = 604800000
local UNIX_OFFSET_MSEC    = 17000 * 86400 + 52 * 10 * AP_MSEC_PER_WEEK - 18000

-- 激活响应状态码 → 地面站中文说明
local ACTIVATION_MSG = {
    [0] = "激活成功",
    [1] = "服务器异常",
    [2] = "无人机不存在于系统中",
    [3] = "实名登记验证失败",
    [4] = "激活状态上报失败",
    [5] = "设备正在激活中，请稍候",
    [6] = "在禁飞区内/接近禁飞区",
}

-- ---- MQTT 工具函数 ----

local function mqtt_encode_len(n)
    local r = ""
    repeat
        local b = n % 128
        n = math.floor(n / 128)
        if n > 0 then b = b | 0x80 end
        r = r .. string.char(b)
    until n == 0
    return r
end

local function mqtt_str(s)
    local l = #s
    return string.char(math.floor(l/256), l%256) .. s
end

-- 解码 MQTT 剩余长度字段，返回 (length, next_pos)
-- MQTT 可变长度整数：bit7=1 表示后续还有字节，bit7=0 表示最后一字节
local function mqtt_decode_rem_len(data, pos)
    local multiplier = 1
    local value = 0
    local i = pos
    while true do
        if i > #data then return nil, pos end
        local encoded = data:byte(i)
        value = value + (encoded % 128) * multiplier
        multiplier = multiplier * 128
        i = i + 1
        if encoded < 128 then break end  -- bit7=0：最后一个长度字节
    end
    return value, i
end

-- MQTT CONNECT：支持可选用户名/密码（CONNACK rc=4 表示账号密码错误）
local function mqtt_connect_pkt(client_id, username, password)
    local flags = 0x02  -- Clean Session
    local pl = mqtt_str(client_id)
    if username and username ~= "" then
        flags = flags | 0x80  -- Username
        pl = pl .. mqtt_str(username)
        if password and password ~= "" then
            flags = flags | 0x40  -- Password
            pl = pl .. mqtt_str(password)
        end
    end
    local vh = "\0\4MQTT\4" .. string.char(flags) .. "\0\x3C"  -- MQTT 3.1.1, keepalive 60s
    return "\x10" .. mqtt_encode_len(#vh + #pl) .. vh .. pl
end

-- MQTT PUBLISH，qos=0/1
local function mqtt_publish_pkt(topic, payload, qos, packet_id)
    qos = qos or 0
    local flags = qos * 2
    local vh = mqtt_str(topic)
    if qos > 0 then
        vh = vh .. string.char(math.floor(packet_id / 256), packet_id % 256)
    end
    return string.char(0x30 | flags) .. mqtt_encode_len(#vh + #payload) .. vh .. payload
end

-- MQTT SUBSCRIBE
local function mqtt_subscribe_pkt(packet_id, topic, qos)
    qos = qos or 1
    local pl = mqtt_str(topic) .. string.char(qos)
    local vh = string.char(math.floor(packet_id / 256), packet_id % 256)
    return "\x82" .. mqtt_encode_len(#vh + #pl) .. vh .. pl
end

local function mqtt_pingreq_pkt()
    return "\xC0\x00"
end

-- 应答 broker 下发的 QoS1 PUBLISH（必须回 PUBACK，否则可能收不到后续 down 消息）
local function mqtt_puback_pkt(packet_id)
    return string.char(0x40, 0x02, math.floor(packet_id / 256), packet_id % 256)
end

-- 获取 fcu_id（UAS ID，无则用 default_sn）
local function uom_get_fcu_id()
    local id = odid_get_uas_id()
    if id == "" then id = "default_sn" end
    return id
end

-- 绑定返回值可能是 Lua number 或 uint32_t（:toint）；勿对 number 写 v.toint
local function scripting_to_int(v)
    if v == nil then
        return nil
    end
    if type(v) == "number" then
        return v
    end
    if type(v) ~= "number" and v.toint then
        return v:toint()
    end
    return tonumber(v)
end

-- uint64_t → Lua number（time_epoch_usec 不能直接 math.floor）
local function uint64_to_number(v)
    if v == nil then
        return nil
    end
    if type(v) == "number" then
        return v
    end
    if v.split then
        local hi, lo = v:split()
        return scripting_to_int(hi) * 4294967296.0 + scripting_to_int(lo)
    end
    return tonumber(v)
end

-- 时间戳（毫秒）：优先 3D GPS 的 UTC/周时，否则 millis（遥测用，激活须 uom_ts_json）
local function uom_timestamp_ms()
    local inst = uom_gps_inst_3d()
    if inst ~= nil then
        local ms = uom_gps_epoch_ms(inst)
        if ms == nil then
            ms = uom_gps_week_to_unix_ms(inst)
        end
        if ms ~= nil then
            return ms
        end
    end
    return scripting_to_int(millis()) or 0
end

-- 十进制整数转字符串（避免 %.0f 输出 2e+12 导致 JSON 非法）
local function int64_to_dec_str(n)
    n = math.floor(tonumber(n) or 0)
    if n <= 0 then
        return "0"
    end
    local s = ""
    while n > 0 do
        local d = n % 10
        s = string.char(48 + d) .. s
        n = math.floor(n / 10)
    end
    return s
end

-- 返回首个具备 3D 定位的 GPS 实例（日志里常为 GPS 1 → instance 1，勿死用 0）
local function uom_gps_inst_3d()
    local n = gps:num_sensors()
    if n < 1 then
        return nil
    end
    local pri = gps:primary_sensor()
    if pri ~= nil and gps:status(pri) >= gps.GPS_OK_FIX_3D then
        return pri
    end
    for inst = 0, n - 1 do
        if gps:status(inst) >= gps.GPS_OK_FIX_3D then
            return inst
        end
    end
    return nil
end

-- 由 GPS 周 + 周内毫秒得到 Unix 毫秒（与 AP_GPS::istate_time_to_epoch_ms 一致）
local function uom_gps_week_to_unix_ms(inst)
    local w_i = scripting_to_int(gps:time_week(inst))
    local wms_i = scripting_to_int(gps:time_week_ms(inst))
    if w_i == nil or wms_i == nil or w_i <= 0 then
        return nil
    end
    -- 用最近一次 fix 时间把周内毫秒推进到当前时刻
    local fix_i = scripting_to_int(gps:last_fix_time_ms(inst))
    local now_i = scripting_to_int(millis())
    if fix_i and now_i and fix_i > 0 then
        wms_i = wms_i + (now_i - fix_i)
    end
    local ms = UNIX_OFFSET_MSEC + w_i * AP_MSEC_PER_WEEK + wms_i
    if ms > 1000000000000 then
        return ms
    end
    return nil
end

-- 从 time_epoch_usec 得到 Unix 毫秒（uint64 微秒 ÷ 1000）
local function uom_gps_epoch_ms(inst)
    local utc = gps:time_epoch_usec(inst)
    if not utc or not (utc > 0) then
        return nil
    end
    local usec = uint64_to_number(utc)
    if not usec or usec < 1e15 then
        return nil
    end
    local ms = math.floor(usec / 1000.0)
    if ms > 1000000000000 then
        return ms
    end
    return nil
end

-- JSON 用 Unix 毫秒时间戳（须 ≥1e12，拒绝 177 等无效值）
local function uom_ts_json()
    local inst = uom_gps_inst_3d()
    if inst == nil then
        return "0"
    end
    local ms = uom_gps_epoch_ms(inst)
    if ms == nil or ms < 1000000000000 then
        ms = uom_gps_week_to_unix_ms(inst)
    end
    if ms == nil or ms < 1000000000000 then
        return "0"
    end
    return int64_to_dec_str(ms)
end

-- fcu_id 变化后须重新订阅 down 主题
local function uom_fcu_id_changed_resubscribe()
    local fcu_id = uom_get_fcu_id()
    if uom.subscribed_fcu_id == "" or uom.subscribed_fcu_id == fcu_id then
        return false
    end
    if not uom.sock then
        return false
    end
    gcs:send_text(MAV_SEVERITY.WARNING, "UOM: 设备编号变化，重新订阅")
    uom.state = "SUBSCRIBING"
    uom.subscribe_sent = false
    uom.subscribe_sent_ms = uint32_t(0)
    uom.activation_sent = false
    uom.subscribed_fcu_id = ""
    return true
end

-- 遥测发送成功计数：仅在 1、10、100、1000… 条时通知地面站（避免每 10 条刷屏）
local function uom_telemetry_log_milestone(cnt)
    if cnt == 1 then
        return true
    end
    local step = 10
    while step <= cnt do
        if cnt == step then
            return true
        end
        if step > 1000000000 then
            break
        end
        step = step * 10
    end
    return false
end

-- 构造激活请求 JSON
local function uom_build_activation_json()
    local fcu_id = uom_get_fcu_id()
    return string.format('{"fcu_id":"%s","timestamp":%s}', fcu_id, uom_ts_json())
end

-- 简易 JSON 字段提取（激活响应）
local function json_get_number(s, key)
    local v = s:match('"' .. key .. '"%s*:%s*(%-?%d+)')
    return v and tonumber(v) or nil
end

local function json_get_string(s, key)
    return s:match('"' .. key .. '"%s*:%s*"([^"]*)"')
end

-- 向地面站发送激活状态
-- uom_close 为模块级函数（非 local），PPP 断开时也可调用
uom_close = function() end

-- 云平台下行 code=6：禁飞区状态 → PreArm 禁止解锁；飞入时在飞强制 RTL
local function uom_apply_nfz_from_cloud(inside, message)
    local was_inside = uom.in_nfz_zone
    uom.in_nfz_zone = inside

    if inside and not was_inside then
        -- 上电在区内 / 解锁后飞入：边沿触发，避免重复刷屏
        if arming:is_armed() and vehicle:get_likely_flying() then
            local mode = vehicle:get_mode()
            if uom_mode_is_rtl_or_land(mode) then
                gcs:send_text(MAV_SEVERITY.CRITICAL, "已进入禁飞区")
            elseif vehicle:set_mode(COPTER_MODE_RTL) then
                gcs:send_text(MAV_SEVERITY.CRITICAL, "已进入禁飞区，正在强制返航")
            else
                gcs:send_text(MAV_SEVERITY.CRITICAL, "已进入禁飞区，请立刻返航或降落！")
            end
        else
            gcs:send_text(MAV_SEVERITY.CRITICAL, "已进入禁飞区，无人机禁止解锁")
        end
    end

    if uom.auth_id then
        if inside then
            arming:set_aux_auth_failed(uom.auth_id, UOM_NFZ_ARming_MSG)
        elseif was_inside then
            -- 仅在本模块曾判入禁飞区后离开（如后续收到 code=0）时释放
            arming:set_aux_auth_passed(uom.auth_id)
        end
    end
end

local function uom_notify_activation(code, message)
    if code == 0 then
        return  -- 激活成功不写 GCS
    end
    if code == 6 then
        -- 发状态说明；不用「UOM激活[6]」格式，也不转发云平台英文 message
        if not uom.in_nfz_zone then
            gcs:send_text(MAV_SEVERITY.WARNING,
                "UOM: " .. (ACTIVATION_MSG[6] or "在禁飞区内/接近禁飞区"))
        end
        return
    end
    local desc = ACTIVATION_MSG[code] or "未知状态"
    local sev = MAV_SEVERITY.INFO
    if code == 5 then
        sev = MAV_SEVERITY.WARNING
    else
        sev = MAV_SEVERITY.ERROR
    end
    gcs:send_text(sev, string.format("UOM: %s", desc))
end

-- 进入 ACTIVATING（发送 uav/up/activation/... 并等待 down 响应）
local function uom_begin_activating(now, reason)
    if uom.state == "ACTIVATING" or uom.state == "READY" then
        return
    end
    if uom_fcu_id_changed_resubscribe() then
        return
    end
    uom.state = "ACTIVATING"
    uom.activation_sent = false
    uom.activation_send_after_ms = now
    uom.activate_timeout_ms = now + uint32_t(uom.ACTIVATE_TIMEOUT_MS)
end

-- 处理激活响应 JSON
local function uom_handle_activation_response(payload)
    local code = json_get_number(payload, "code")
    if code == nil then return end
    local message = json_get_string(payload, "message") or ""
    uom.activation_code = code
    uom_notify_activation(code, message)

    if code == 0 then
        uom_apply_nfz_from_cloud(false, message)
        uom.activated = true
        uom.state = "READY"
        uom.last_pub_ms = uint32_t(0)
    elseif code == 5 then
        -- 正在激活中，延迟后重试
        uom.activation_sent = false
        uom.activation_send_after_ms = millis() + uint32_t(uom.ACTIVATE_RETRY_MS)
        uom.activate_timeout_ms = uom.activation_send_after_ms + uint32_t(uom.ACTIVATE_TIMEOUT_MS)
    elseif code == 6 then
        -- 禁飞区通知：保持 MQTT/遥测，仅 PreArm 禁止解锁
        uom_apply_nfz_from_cloud(true, message)
        uom.activated = false
        if uom.state == "ACTIVATING" then
            uom.activation_sent = false
            uom.activation_send_after_ms = millis() + uint32_t(uom.ACTIVATE_RETRY_MS)
            uom.activate_timeout_ms = uom.activation_send_after_ms + uint32_t(uom.ACTIVATE_TIMEOUT_MS)
        end
    else
        -- 致命错误：断开重连，冷却后再走激活流程
        uom.activated = false
        uom.retry_after_ms = millis() + uint32_t(30000)
        uom_close()
    end
end

-- 解析 rx_buf 中的 MQTT 包，处理 SUBACK / PUBLISH（单次调用有包数上限）
-- 激活等待期间收到任意 MQTT 下行则延长等待（避免与云端 3~15s 处理撞车）
local function uom_mqtt_dispatch()
    local pkt_count = 0
    while #uom.rx_buf >= 2 and pkt_count < uom.MQTT_DISPATCH_MAX_PKTS do
        pkt_count = pkt_count + 1
        local b0 = uom.rx_buf:byte(1)
        local msg_type = math.floor(b0 / 16)
        local rem_len, hdr_end = mqtt_decode_rem_len(uom.rx_buf, 2)
        if not rem_len then break end
        local total = hdr_end - 1 + rem_len
        if #uom.rx_buf < total then break end  -- 包未收全

        local body = uom.rx_buf:sub(hdr_end, total)
        uom.rx_buf = uom.rx_buf:sub(total + 1)

        if msg_type == 9 then
            -- SUBACK：订阅成功，进入激活阶段
            if uom.state == "SUBSCRIBING" then
                uom_begin_activating(millis(), "SUBACK")
            end

        elseif msg_type == 3 then
            -- PUBLISH：解析 topic + payload
            if #body < 2 then goto continue end
            local tlen = body:byte(1) * 256 + body:byte(2)
            if #body < 2 + tlen then goto continue end
            local topic = body:sub(3, 2 + tlen)
            local pos = 3 + tlen
            local qos = (b0 >> 1) & 0x03
            local pub_pid = nil
            if qos > 0 then
                if #body < pos + 1 then goto continue end
                pub_pid = body:byte(pos) * 256 + body:byte(pos + 1)
                pos = pos + 2
            end
            local payload = body:sub(pos)

            if qos > 0 and pub_pid and uom.sock then
                local ack = mqtt_puback_pkt(pub_pid)
                uom.sock:send(ack, #ack)
            end

            -- 激活响应：云 → 飞控，主题 uav/down/activation/{vendor}/{fcu_id}
            local fcu_id = uom_get_fcu_id()
            local act_down = string.format(uom.TOPIC_ACT_DOWN, fcu_id)
            if topic == act_down or topic:find("activation") then
                if uom.state == "ACTIVATING" and uom.activation_sent then
                    uom.activate_timeout_ms = millis() + uint32_t(uom.ACTIVATE_TIMEOUT_MS)
                end
                if payload == "" or payload == nil then
                    gcs:send_text(MAV_SEVERITY.WARNING, "UOM: 激活响应为空")
                else
                    local code = json_get_number(payload, "code")
                    if code == nil then
                        gcs:send_text(MAV_SEVERITY.WARNING, "UOM: 激活响应无法解析")
                    end
                    uom_handle_activation_response(payload)
                end
            end

        elseif msg_type == 4 or msg_type == 13 then
            -- PUBACK(4) / PINGRESP(13)：已消费，无需处理
        end
        ::continue::
    end
end

-- 非阻塞读取 MQTT 下行数据（限制单次循环工作量，避免 exceeded time limit）
local function uom_mqtt_recv()
    if not uom.sock then return end
    local chunks = 0
    while uom.sock:pollin(0) and chunks < uom.MQTT_RECV_MAX_CHUNKS do
        chunks = chunks + 1
        local chunk = uom.sock:recv(256)
        if chunk == nil then break end
        if #chunk == 0 then
            gcs:send_text(MAV_SEVERITY.WARNING, "UOM: 服务器断开连接")
            uom.retry_after_ms = millis() + uint32_t(5000)
            uom_close()
            return
        end
        uom.rx_buf = uom.rx_buf .. chunk
        if #uom.rx_buf > uom.RX_BUF_MAX then
            uom.rx_buf = uom.rx_buf:sub(#uom.rx_buf - uom.RX_BUF_MAX + 1)
        end
    end
    uom_mqtt_dispatch()
end

-- 分配 MQTT 包 ID（1~65535 循环）
local function uom_next_packet_id()
    local id = uom.packet_id
    uom.packet_id = uom.packet_id + 1
    if uom.packet_id > 65535 then uom.packet_id = 1 end
    return id
end

-- ---- 构造 UOM JSON（飞行遥测 + ODID 信息）----
local function uom_build_json()
    local lat, lng = 0.0, 0.0
    local alt_rel, alt_gps = 0.0, 0.0
    local speed, yaw, pitch, roll = 0.0, 0.0, 0.0, 0.0
    local accuracy = 9999

    local loc = ahrs:get_position()
    if loc then
        lat = loc:lat() * 1e-7
        lng = loc:lng() * 1e-7
    end

    local d = ahrs:get_relative_position_D_home()
    if d then alt_rel = -d end

    local gps_loc = gps:location(0)
    if gps_loc then alt_gps = gps_loc:alt() * 0.01 end

    local vel = ahrs:get_velocity_NED()
    if vel then speed = math.sqrt(vel:x()^2 + vel:y()^2) end

    -- ArduPilot Lua：get_roll/pitch/yaw_rad（无 get_euler_angles）
    roll  = math.deg(ahrs:get_roll_rad())
    pitch = math.deg(ahrs:get_pitch_rad())
    yaw   = math.deg(ahrs:get_yaw_rad())
    if yaw < 0 then yaw = yaw + 360 end

    local hacc = gps:horizontal_accuracy(0)
    if hacc then accuracy = math.floor(hacc * 100) end

    -- sys_status_bit 固定为 0（按需求）
    local uas_id = odid_get_uas_id()
    local operator_id = odid_get_operator_id()
    -- user_id：优先使用 Mission Planner 下发的自我声明内容（SELF_ID.description），
    --          否则回退到 LTE_USER_ID 参数（整数转字符串）
    local user_id = odid.self_desc ~= "" and odid.self_desc
                    or tostring(math.floor(LTE_USER_ID:get()))
    return string.format(
        '{"ts":%s,'  ..
        '"lng":%.7f,"lat":%.7f,'  ..
        '"alt":%.1f,"alt_gps":%.1f,'  ..
        '"speed":%.2f,"yaw":%.1f,"pitch":%.1f,"roll":%.1f,'  ..
        '"accuracy":%d,"sys_status_bit":0,'  ..
        '"user_id":"%s",' ..
        '"uas_id":"%s",'   ..
        '"operator_id":"%s",'  ..
        '"op_lat":%.7f,"op_lng":%.7f,"op_alt":%.1f}',
        uom_ts_json(),
        lng, lat,
        alt_rel, alt_gps,
        speed, yaw, pitch, roll,
        accuracy,
        user_id,
        uas_id,
        operator_id,
        odid_get_op_lat(), odid_get_op_lng(), odid_get_op_alt())
end

uom_close = function()
    if uom.sock then
        uom.sock:close()
        uom.sock = nil
    end
    uom.state = "IDLE"
    uom.connect_sent = false
    uom.subscribe_sent = false
    uom.activation_sent = false
    uom.activated = false
    uom.activation_code = -1
    uom.rx_buf = ""
    uom.last_send_ok_ms = nil
    uom.subscribe_sent_ms = uint32_t(0)
    uom.gps_warn_sent = false
    uom.subscribed_fcu_id = ""
    uom.mqtt_connected_ms = uint32_t(0)
    uom.id_wait_warn_sent = false
    uom.in_nfz_zone = false
    if uom.auth_id then
        arming:set_aux_auth_passed(uom.auth_id)
    end
end

-- MQTT 套接字仍可用（uom_close / 对端断开 / PPP 掉线后必须为 false）
local function uom_sock_alive()
    return uom.sock ~= nil
end

-- ---- UOM 主更新函数，在 step_CONNECTED() 中按 uom.TICK_MS 节流派发 ----
return function()
    if LTE_UOM_ENABLE:get() ~= 1 then return end
    if LTE_PROTOCOL:get() ~= PPP then return end

    local now = millis()
    if (now - uom.last_odid_poll_ms) >= uint32_t(uom.ODID_POLL_MS) then
        uom.last_odid_poll_ms = now
        odid_poll()
    end

    -- CONNECTED 步 5ms 调用一次；MQTT 不必每拍跑满，避免 scripting time limit
    if uom.state ~= "ACTIVATING" and uom.state ~= "READY" and uom.state ~= "SUBSCRIBING" then
        if (now - uom.last_tick_ms) < uint32_t(uom.TICK_MS) then
            return
        end
    elseif (now - uom.last_tick_ms) < uint32_t(20) then
        -- 激活/遥测阶段略快（20ms），仍远低于 5ms
        return
    end
    uom.last_tick_ms = now
    local host = string.format("%d.%d.%d.%d",
        LTE_UOM_IP0:get(), LTE_UOM_IP1:get(),
        LTE_UOM_IP2:get(), LTE_UOM_IP3:get())
    local port = math.floor(LTE_UOM_PORT:get())

    if uom.state == "IDLE" then
        if port <= 0 then return end
        if now < uom.retry_after_ms then return end
        uom.sock = Socket(0) -- 0 for TCP
        if not uom.sock then return end
        uom.sock:set_blocking(false)
        uom.sock:connect(host, port)
        uom.state = "CONNECTING"
        uom.connect_sent = false
        uom.subscribe_sent = false
        uom.activation_sent = false
        uom.activated = false
        uom.rx_buf = ""
        uom.connect_timeout_ms = now + uint32_t(8000)
        return
    end

    if uom.state == "CONNECTING" then
        if not uom_sock_alive() then
            uom_close()
            return
        end
        if now > uom.connect_timeout_ms then
            uom_close()  -- 静默重连，不写 GCS
            return
        end
        if not uom.connect_sent then
            local client_id = uom_get_fcu_id()
            local pkt = mqtt_connect_pkt(client_id, UOM_MQTT_USER, UOM_MQTT_PASS)
            local n = uom.sock:send(pkt, #pkt)
            if not n or n <= 0 then return end
            uom.connect_sent = true
            return
        end
        if not uom.sock:pollin(0) then return end
        local ack = uom.sock:recv(4)
        if ack and #ack >= 4 then
            if ack:byte(1) == 0x20 and ack:byte(4) == 0x00 then
                uom.mqtt_connected_ms = now
                uom.state = "SUBSCRIBING"
                uom.subscribe_sent = false
                uom.last_ping_ms = now
                uom.last_send_ok_ms = now
            else
                local rc = ack:byte(4)
                gcs:send_text(MAV_SEVERITY.ERROR,
                    string.format("UOM: 连接被拒绝(%d)", rc or 0))
                uom.retry_after_ms = now + uint32_t(10000)
                uom_close()
            end
        end
        return
    end

    -- SUBSCRIBING：订阅激活响应主题 uav/down/activation/eft/{fcu_id}
    if uom.state == "SUBSCRIBING" then
        uom_mqtt_recv()
        if not uom_sock_alive() then return end
        if uom_fcu_id_changed_resubscribe() then
            return
        end
        odid_poll()
        if not uom.subscribe_sent then
            local fcu_id = uom_get_fcu_id()
            if #fcu_id < uom.FCU_ID_MIN_LEN then
                return
            end
            local sub_topic = string.format(uom.TOPIC_ACT_DOWN, fcu_id)
            local pid = uom_next_packet_id()
            -- 与云平台 down 发布 QoS0 一致
            local pkt = mqtt_subscribe_pkt(pid, sub_topic, 0)
            local n = uom.sock:send(pkt, #pkt)
            if n and n > 0 then
                uom.subscribe_sent = true
                uom.subscribe_sent_ms = now
                uom.subscribed_fcu_id = fcu_id
            end
        elseif uom.subscribe_sent_ms > 0 and
               (now - uom.subscribe_sent_ms) > uint32_t(uom.SUBACK_TIMEOUT_MS) then
            -- 未收到 SUBACK 时仍进入激活（部分 broker SUBACK 格式异常会导致一直卡住）
            uom_begin_activating(now, "SUBACK timeout")
        end
        if uom_sock_alive() and now - uom.last_ping_ms >= uint32_t(uom.PING_MS) then
            uom.sock:send(mqtt_pingreq_pkt(), 2)
            uom.last_ping_ms = now
        end
        return
    end

    -- ACTIVATING：发送 up 激活，等待 down 响应（云 → 飞控）
    if uom.state == "ACTIVATING" then
        uom_mqtt_recv()
        if not uom_sock_alive() then return end
        if uom.state == "READY" then
            return
        end
        if uom_fcu_id_changed_resubscribe() then
            return
        end
        odid_poll()

        if uom.activation_sent and now > uom.activate_timeout_ms then
            uom.activation_sent = false
            uom.activation_send_after_ms = now + uint32_t(uom.ACTIVATE_RETRY_MS)
        end

        if not uom.activation_sent and now >= uom.activation_send_after_ms then
            local ts_s = uom_ts_json()
            if ts_s == "0" then
                return  -- 无 GPS 时间则等待，不写 GCS
            end
            local fcu_id = uom_get_fcu_id()
            -- 飞控 → 云：uav/up/activation/eft/{fcu_id}（协议 3.2 激活请求）
            local act_topic = string.format(uom.TOPIC_ACT_UP, fcu_id)
            local json = string.format(
                '{"fcu_id":"%s","timestamp":%s}', fcu_id, ts_s)
            local pid = uom_next_packet_id()
            -- QoS0：与文档一致且无需等 broker PUBACK，避免占满接收缓冲
            local pkt = mqtt_publish_pkt(act_topic, json, 0, pid)
            local n = uom.sock:send(pkt, #pkt)
            if n and n > 0 then
                uom.activation_sent = true
                uom.activate_timeout_ms = now + uint32_t(uom.ACTIVATE_TIMEOUT_MS)
                uom.last_send_ok_ms = now
            end
        end

        if uom_sock_alive() and now - uom.last_ping_ms >= uint32_t(uom.PING_MS) then
            uom.sock:send(mqtt_pingreq_pkt(), 2)
            uom.last_ping_ms = now
        end
        return
    end

    -- READY：激活成功后上报遥测
    if uom.state == "READY" then
        uom_mqtt_recv()
        if not uom_sock_alive() then return end

        -- 云平台 code=6 后持续维持解锁禁止（与 1noflyzone_checker 鉴权 ID 独立）
        if uom.in_nfz_zone and uom.auth_id then
            arming:set_aux_auth_failed(uom.auth_id, UOM_NFZ_ARming_MSG)
        end

        if uom.activated and now - uom.last_pub_ms >= uint32_t(uom.REPORT_MS) then
            local json = uom_build_json()
            local sn = uom_get_fcu_id()
            local topic = string.format(uom.TOPIC_TELEMETRY, sn)

            local pkt = mqtt_publish_pkt(topic, json, 0)
            local n = uom.sock:send(pkt, #pkt)
            if n and n > 0 then
                uom.last_pub_ms = now
                uom.last_send_ok_ms = now
                uom.publish_count = uom.publish_count + 1
                if uom_telemetry_log_milestone(uom.publish_count) then
                    gcs:send_text(MAV_SEVERITY.INFO,
                        string.format("UOM#%d SEND SUCCESS",
                            uom.publish_count))
                end
            end
        end

        if uom.last_send_ok_ms ~= nil and
           (now - uom.last_send_ok_ms) > uint32_t(60000) then
            gcs:send_text(MAV_SEVERITY.WARNING, "UOM: 发送超时，重连中")
            uom.retry_after_ms = now + uint32_t(5000)
            uom_close()
            return
        end

        if uom_sock_alive() and now - uom.last_ping_ms >= uint32_t(uom.PING_MS) then
            uom.sock:send(mqtt_pingreq_pkt(), 2)
            uom.last_ping_ms = now
        end
    end
end

end)()

--[[
    合宙 Air780E / Air780 AT 指令表（EFT_CAAC 仅使用此模组）
--]]
local Air780 = {
    banner = 'AirM2M_780E',
    banners = { 'AirM2M_780E', 'Air780E', 'Air780', 'AIR780', '780E' },
    cmux = nil,
    setbaud = 'AT+IPR=%u\r\n',
    cgact = 'AT+CGACT=1,1\r\n',
    pppopen = 'ATD*99#\r',
    cpin = 'AT+CPIN?\r\n',
    reset = 'AT+CFUN=1,1\r\n',
    cipmode = 'AT+CIPMODE=1\r\n',
}

local modem = nil  -- nil=尚未识别；识别成功后指向 Air780

--[[
    return true if an option is enabled
--]]
local function option_enabled(option)
    return (LTE_OPTIONS:get() & option) ~= 0
end

-- ============================================================
-- 启动时检查关键系统参数，不对则通过 GCS 告警提示用户
-- ============================================================
local function check_hw_params()
    -- { 参数名, 期望值, 比较符("eq"/"ge"), 说明 }
    local checks = {
        { "SERIAL1_PROTOCOL", 28,     "eq", "应为28(Scripting)" },
        { "SERIAL1_BAUD",     115,    "eq", "应为115(=115200bps)" },
        { "SCR_SDEV_EN",      1,      "eq", "需开启Scripting虚拟串口" },
        { "NET_ENABLE",       1,      "eq", "需开启网络栈" },
        { "SCR_ENABLE",       1,      "eq", "需开启Lua脚本引擎" },
        { "SCR_HEAP_SIZE",    65536,  "ge", "堆内存建议>=65536(推荐204800)" },
    }
    local warn_count = 0
    for _, c in ipairs(checks) do
        local name, expect, op, hint = c[1], c[2], c[3], c[4]
        local v = param:get(name)
        local bad = false
        if v == nil then
            bad = true
            hint = "参数不存在，请升级固件"
        elseif op == "eq" and math.floor(v + 0.5) ~= expect then
            bad = true
        elseif op == "ge" and v < expect then
            bad = true
        end
        if bad then
            gcs:send_text(MAV_SEVERITY.WARNING,
                string.format("LTE: ⚠ %s=%s %s", name, tostring(v), hint))
            warn_count = warn_count + 1
        end
    end
    -- 仅参数异常时告警，通过时不提示
    if warn_count > 0 then
        gcs:send_text(MAV_SEVERITY.WARNING,
            string.format("LTE: 发现%d项参数异常，请按SOP修正后重启", warn_count))
    end
end

if LTE_ENABLE:get() == 0 then
    -- disabled
    return
end

check_hw_params()
-- 启动先清零，避免沿用上电前 SD 里残留的 '1' 导致禁飞脚本过早 HTTP
write_lte_ppp_ready_flag(false)

local uart = serial:find_serial(LTE_SERPORT_FIXED)
if not uart then
    gcs:send_text(MAV_SEVERITY.ERROR,
        'LTE_modem: SERIAL1 未设为 Scripting(28)，请设 SERIAL1_PROTOCOL=28 后重启')
    return
end

local ser_device = serial:find_simulated_device(PPP, LTE_SCRPORT_FIXED)
if not ser_device then
    gcs:send_text(MAV_SEVERITY.ERROR, 'LTE_modem: could not find SCR_SDEV device')
    return
end

local step = "ATI"

-- GCS 仅上报少数关键节点，避免 step/AT 过程刷屏
local lte_link_gcs_done = false    -- 已提示 4G 已连接

local function lte_gcs_link_up_once()
    if lte_link_gcs_done then
        return
    end
    lte_link_gcs_done = true
    gcs:send_text(MAV_SEVERITY.INFO, "LTE: 4G已连接")
end

local function lte_gcs_link_clear()
    lte_link_gcs_done = false
end

-- ATI 未识别到 Air780 时提示（限频，避免刷屏）
local lte_4g_nf_last_ms = uint32_t(0)
local LTE_4G_NF_GAP_MS = 15000

local function lte_gcs_4g_not_found()
    local now = millis()
    if lte_4g_nf_last_ms > 0 and (now - lte_4g_nf_last_ms):toint() < LTE_4G_NF_GAP_MS then
        return
    end
    lte_4g_nf_last_ms = now
    gcs:send_text(MAV_SEVERITY.WARNING, "没有找到4G模块，请检查")
end

local stats = { bytes_in = 0, bytes_out = 0 }

uart:begin(LTE_IBAUD:get())

--[[
    Open a log file to log the output from the modem
    This is useful for debugging the connection process
--]]
local log_file = io.open('LTE_modem.log', 'w')

--[[
    log data to log_file
--]]
local function log_data(s, marker)
    if s and #s > 0 and log_file then
        log_file:write(marker .. '[' .. s .. ']\n')
        log_file:flush()
    end
end

--[[
    Function to read from the UART and log the output
    This function reads up to 512 bytes at a time and writes it to the log file
    returns the string read or nil
--]]
local function uart_read()
    local s = uart:readstring(512)
    if not s then
        return ""
    end
    log_data(s, '<<<')
    stats.bytes_in = stats.bytes_in + #s
    return s
end

local pending_to_uart = ""

--[[
    write any pending bytes to the uart
--]]
local function uart_write_pending()
    if #pending_to_uart > 0 then
        local n = uart:writestring(pending_to_uart)
        pending_to_uart = pending_to_uart:sub(n+1)
    end
end

--[[
    Function to write to the UART and log the command
--]]
local function uart_write(s)
    pending_to_uart = pending_to_uart .. s
    if option_enabled(LTE_OPTIONS_LOGALL) or step ~= "CONNECTED" then
        log_data(s, '>>>')
    end
    stats.bytes_out = stats.bytes_out + #s
    return #s
end

-- Constants for GSM 07.10 CMUX framing
FLAG = 0xF9
UIH = 0xEF
SABM = 0x2F
--local UA = 0x63
EA = 0x01
CR_SEND = 0x02

DLC_AT = 1
DLC_DATA = 2

-- CMUX buffer state
local cmux = {}
cmux.buffers = {[DLC_AT] = "", [DLC_DATA] = ""} -- DLC1=AT, DLC2=DATA(PPP or TCP)

last_mccmnc = nil
last_band = nil

--[[
    FCS lookup table for polynomial x^8 + x^2 + x^1 + 1 (0x07)
    This is the reverse of the standard CRC-8 table
--]]
local fcs_table = {
    0x00, 0x91, 0xe3, 0x72, 0x07, 0x96, 0xe4, 0x75,
    0x0e, 0x9f, 0xed, 0x7c, 0x09, 0x98, 0xea, 0x7b,
    0x1c, 0x8d, 0xff, 0x6e, 0x1b, 0x8a, 0xf8, 0x69,
    0x12, 0x83, 0xf1, 0x60, 0x15, 0x84, 0xf6, 0x67,
    0x38, 0xa9, 0xdb, 0x4a, 0x3f, 0xae, 0xdc, 0x4d,
    0x36, 0xa7, 0xd5, 0x44, 0x31, 0xa0, 0xd2, 0x43,
    0x24, 0xb5, 0xc7, 0x56, 0x23, 0xb2, 0xc0, 0x51,
    0x2a, 0xbb, 0xc9, 0x58, 0x2d, 0xbc, 0xce, 0x5f,
    0x70, 0xe1, 0x93, 0x02, 0x77, 0xe6, 0x94, 0x05,
    0x7e, 0xef, 0x9d, 0x0c, 0x79, 0xe8, 0x9a, 0x0b,
    0x6c, 0xfd, 0x8f, 0x1e, 0x6b, 0xfa, 0x88, 0x19,
    0x62, 0xf3, 0x81, 0x10, 0x65, 0xf4, 0x86, 0x17,
    0x48, 0xd9, 0xab, 0x3a, 0x4f, 0xde, 0xac, 0x3d,
    0x46, 0xd7, 0xa5, 0x34, 0x41, 0xd0, 0xa2, 0x33,
    0x54, 0xc5, 0xb7, 0x26, 0x53, 0xc2, 0xb0, 0x21,
    0x5a, 0xcb, 0xb9, 0x28, 0x5d, 0xcc, 0xbe, 0x2f,
    0xe0, 0x71, 0x03, 0x92, 0xe7, 0x76, 0x04, 0x95,
    0xee, 0x7f, 0x0d, 0x9c, 0xe9, 0x78, 0x0a, 0x9b,
    0xfc, 0x6d, 0x1f, 0x8e, 0xfb, 0x6a, 0x18, 0x89,
    0xf2, 0x63, 0x11, 0x80, 0xf5, 0x64, 0x16, 0x87,
    0xd8, 0x49, 0x3b, 0xaa, 0xdf, 0x4e, 0x3c, 0xad,
    0xd6, 0x47, 0x35, 0xa4, 0xd1, 0x40, 0x32, 0xa3,
    0xc4, 0x55, 0x27, 0xb6, 0xc3, 0x52, 0x20, 0xb1,
    0xca, 0x5b, 0x29, 0xb8, 0xcd, 0x5c, 0x2e, 0xbf,
    0x90, 0x01, 0x73, 0xe2, 0x97, 0x06, 0x74, 0xe5,
    0x9e, 0x0f, 0x7d, 0xec, 0x99, 0x08, 0x7a, 0xeb,
    0x8c, 0x1d, 0x6f, 0xfe, 0x8b, 0x1a, 0x68, 0xf9,
    0x82, 0x13, 0x61, 0xf0, 0x85, 0x14, 0x66, 0xf7,
    0xa8, 0x39, 0x4b, 0xda, 0xaf, 0x3e, 0x4c, 0xdd,
    0xa6, 0x37, 0x45, 0xd4, 0xa1, 0x30, 0x42, 0xd3,
    0xb4, 0x25, 0x57, 0xc6, 0xb3, 0x22, 0x50, 0xc1,
    0xba, 0x2b, 0x59, 0xc8, 0xbd, 0x2c, 0x5e, 0xcf
}

--[[
    Calculate FCS for a byte array
    data: table of bytes (numbers 0-255) or string
    Returns: FCS value (0-255)
--]]
local function fcs_calc(data)
    local fcs = 0xff  -- Initial value
    
    for i = 1, #data do
        local byte = string.byte(data, i)
        fcs = fcs_table[((fcs ~ byte) & 0xff) + 1] ~ (fcs >> 8)
    end

    return (~fcs) & 0xff
end

-- Construct a CMUX frame for a given DLC, data type and data
function cmux.encode_cmux_frame(dlc, dtype, data)
    local addr = string.char((dlc << 2) | EA | CR_SEND)
    local ctrl = string.char(dtype | 0x10)
    local len = #data
    local len_byte = string.char((len << 1) | EA)
    local header = addr .. ctrl .. len_byte
    local fcs = string.char(fcs_calc(header))
    return string.char(FLAG) .. header .. data .. fcs .. string.char(FLAG)
end

local found_cmux = false

--[[
    return true if we should use CMUX
--]]
local function cmux_enabled()
    if found_cmux then
        return true
    end
    return modem and modem.cmux and not option_enabled(LTE_OPTIONS_NOMUX)
end

--[[
    send an AT command string with possible CMUX framing
--]]
local function AT_send(atcmd)
    local s
    if cmux_enabled() then
        s = cmux.encode_cmux_frame(DLC_AT, UIH, atcmd)
    else
        s = atcmd
    end
    return uart_write(s) == #s
end

--[[
    send an appropriate data reset for the protocol
--]]
local function send_data_reset()
    local profile = modem or Air780
    if profile.reset then
        lte_gcs_link_clear()
        AT_send(profile.reset)
        if not profile.reset_not_baudrate then
            -- a reset changes the baud rate to the initial baud rate
            uart:begin(LTE_IBAUD:get())
        end
        -- and clears cmux state
        found_cmux = false
        return
    end
end

--[[
    Function to handle errors in the response from the modem
    If an error is detected, it resets the modem
    returns true if an error was detected
--]]
local function handle_error(s)
    if s and s:find('\nERROR\r\n') then
        gcs:send_text(MAV_SEVERITY.ERROR, 'LTE_modem: error response from modem')
        send_data_reset()
        step = "ATI"
        return true
    end
    return false
end

-- Send SABM (Set Asynchronous Balanced Mode) for all DLCs
function cmux.send_sabm()
    uart_write(cmux.encode_cmux_frame(0, SABM, ""))
    uart_write(cmux.encode_cmux_frame(1, SABM, ""))
    uart_write(cmux.encode_cmux_frame(2, SABM, ""))
end

--[[
 Parses a single CMUX frame from a byte buffer.
 Returns: DLC number, extracted payload, and remaining buffer (or nils on failure)
--]]
function cmux.parse_cmux_frame(buf)
    local start_idx = buf:find(string.char(FLAG))
    if not start_idx then
        --gcs:send_text(MAV_SEVERITY.INFO, "no start idx")
        log_data("NOSTART:" .. buf, "{XXX}")
        return nil, nil, nil
    end
    if #buf < 6 then
        return nil, nil, nil, "short"
    end
    local len_byte = buf:byte(4)
    if (len_byte & EA) == 0 then
        log_data("mux multibyte", "{XXX}")
        return nil, nil, nil -- we don't handle multi-byte length yet
    end
    local len = len_byte >> 1
    local end_idx = 6 + len
    if buf:byte(end_idx) ~= FLAG then
        log_data("no end idx", "{XXX}")
        return nil, nil, nil, "short"
    end

    local frame = buf:sub(start_idx + 1, end_idx - 1)
    if #frame < 4 then
        log_data("too short", "{XXX}")
        return nil, nil, nil
    end

    local addr = frame:byte(1)
    local ctrl = frame:byte(2)

    --gcs:send_text(MAV_SEVERITY.INFO, string.format("addr=0x%02x ctrl=0x%02x", addr, ctrl))

    if ctrl == SABM then
        return nil, nil, buf:sub(end_idx + 1)
    end

    if (ctrl & 0xef) ~= UIH then
        --gcs:send_text(MAV_SEVERITY.INFO, "not UIH")
        return nil, nil, nil
    end

    if #frame ~= 3 + len + 1 then
        log_data("bad flen", "{XXX}")
        return nil, nil, nil
    end

    local data = frame:sub(4, 3 + len)
    local fcs_field = frame:byte(3 + len + 1)
    local header = frame:sub(1, 3)
    local calc_fcs = fcs_calc(header)
    if calc_fcs ~= fcs_field then
        log_data("FCS mismatch", "{XXX}")
        return nil, nil, nil -- FCS mismatch
    end

    local dlc = (addr >> 2) & 0x3F
    local remainder = buf:sub(end_idx + 1)
    --gcs:send_text(MAV_SEVERITY.INFO, string.format("CMUX got: dlc=%d ldata=%d lrem=%d", dlc, #data, #remainder))
    return dlc, data, remainder
end

-- Feeds raw UART data into CMUX frame parser and routes payloads to DLC buffers
function cmux.feed_uart_in(raw)
    while #raw > 0 do
        local dlc, data, rest, err = cmux.parse_cmux_frame(raw)
        if not dlc or not data or not rest then
            if err == "short" then
                -- gcs:send_text(MAV_SEVERITY.INFO, "short")
                return raw
            end
            -- discard
            return ""
        end
        if cmux.buffers[dlc] then
            cmux.buffers[dlc] = cmux.buffers[dlc] .. data
        end
        raw = rest
    end
    return raw
end

--[[
    send data with possible CMUX framing
--]]
local function data_send(data)
    local s
    if cmux_enabled() then
        s = cmux.encode_cmux_frame(DLC_DATA, UIH, data)
    else
        s = data
    end
    return uart_write(s) == #s
end

--[[
    send data with possible CMUX framing when connected (logging only if data
    logging enabled)
--]]
local function data_send_connected(data)
    local s
    if cmux_enabled() then
        s = cmux.encode_cmux_frame(DLC_DATA, UIH, data)
    else
        s = data
    end
    local n = uart_write(s)
    stats.bytes_out = stats.bytes_out + n
    return n == #s
end

local ati_sequence = 0

-- reset back to ATI step
local function reset_to_ATI()
    ppp_connected_ms = nil
    write_lte_ppp_ready_flag(false)
    send_data_reset()
    step = "ATI"
    modem = nil
    found_cmux = false
end

-- 根据 ATI 回显确认是否为 Air780
local function check_modem_banner(s)
    local keys = Air780.banners or { Air780.banner }
    for i = 1, #keys do
        if keys[i] and s:find(keys[i], 1, true) then
            modem = Air780
            lte_4g_nf_last_ms = uint32_t(0)
            return
        end
    end
end

--[[
    Function to confirm the connection to the modem
    it uses AIT command to get the modem info

    when we enter the ATI step the modem could be in one of several states:

    - in AT command mode
    - in muxed mode
    - in muxed mode at higher baudrate
--]]
function step_ATI()
    -- 上电首轮先发软复位，退出上次 PPP/数据模式以便响应 AT
    if ati_sequence == 0 then
        send_data_reset()
    end
    local s = uart_read()
    if s and modem == nil then
        check_modem_banner(s)
    end
    if modem ~= nil then
        if not cmux_enabled() then
            step = "BAUD"
        else
            step = "CMUX"
        end
        return
    end
    if s and #s >= 4 and s:byte(1) == FLAG and s:byte(-1) == FLAG then
        -- already in mux mode
        found_cmux = true
        -- CMUX 就绪不提示
        log_data("{INCMUX}", '***')
        AT_send('ATI\r')
        return
    end
    -- 未识别到 Air780（无回包 / 有数据但非 Air780 回显）
    if ati_sequence > 0 and ati_sequence % 15 == 0 then
        if not s or #s == 0 or modem == nil then
            lte_gcs_4g_not_found()
        end
    end
    if ati_sequence % 3 == 2 then
        uart_write('+++')
    elseif ati_sequence % 3 == 1 then
        uart_write(cmux.encode_cmux_frame(DLC_AT, UIH, "ATI\r"))
    else
        uart_write('\rATI\r')
    end
    if ati_sequence % 10 == 5 then
        uart:begin(LTE_BAUD:get())
        log_data(string.format("{BAUD=%d}", LTE_BAUD:get()), '***')
    end
    if ati_sequence % 10 == 9 then
        uart:begin(LTE_IBAUD:get())
        log_data(string.format("{BAUD=%d}", LTE_IBAUD:get()), '***')
    end
    -- 长时间无应答：软复位模组（退出 PPP 数据模式）
    if ati_sequence > 0 and ati_sequence % 12 == 0 then
        uart_write('AT+CFUN=1,1\r\n')
        lte_gcs_4g_not_found()
    end
    ati_sequence = ati_sequence + 1
end

local change_baud = nil

--[[
    change baud rate
--]]
function step_BAUD()
    if modem.setbaud and LTE_BAUD:get() ~= LTE_IBAUD:get() then
        change_baud = LTE_BAUD:get()
        AT_send(string.format(modem.setbaud, change_baud))
    end
    step = "CPIN"
end

--[[
    set preferred network using MCC country code and MNC network code
--]]
function set_MCCMNC()
    if not modem.mccmnc then
        return
    end
    local mccmnc = math.floor(LTE_MCCMNC:get())
    if mccmnc > 0 then
        AT_send(string.format(modem.mccmnc, mccmnc))
    elseif mccmnc == 0 then
        AT_send("AT+COPS=0\r\n")
    end
    last_mccmnc = mccmnc
end

--[[
    set preferred LTE band
--]]
function set_BAND()
    if not modem.setband and not modem.setband_mask then
        return
    end
    local band = math.floor(LTE_BAND:get())
    if band > 0 then
       if modem.setband_mask then
          AT_send(string.format(modem.setband_mask, 1<<(band-1)))
       else
          AT_send(string.format(modem.setband, band))
       end
    elseif band == 0 then
        AT_send(modem.setband_all)
    end
    last_band = band
end

--[[
    configuration step
--]]
function step_CONFIG()
    set_BAND()
    set_MCCMNC()
    -- Air780：配置 APN 并打开注册状态上报（与 1LTE/2LTE 一致，否则易卡 CREG）
    AT_send(string.format('AT+CGDCONT=1,"IP","%s"\r\n', LTE_APN_DEFAULT))
    AT_send("AT+CREG=2\r\n")
    AT_send("AT+CGATT=1\r\n")
    if modem.config_extra then
        AT_send(modem.config_extra)
    end
    step = "CREG"
end

--[[
    check for a SIM
--]]
function step_CPIN()
    local s = uart_read()
    if s and s:find("READY") then
        step = "CONFIG"
    end
    AT_send('AT+CPIN?\r\n')
end

--[[
    confirm we are registered on the network
--]]
function step_CREG()
    local s = uart_read()
    if handle_error(s) then
        return
    end
    if s then
        if cmux_enabled() and #s > 4 and not cmux.parse_cmux_frame(s) then
            -- not really in CMUX mode when we should be, try again
            step = "CMUX"
            return
        end
        local reg = s:match('CREG: %d,(%d+)')
        if reg == "1" or reg == "5" then
            -- CREG 就绪不提示
            if LTE_PROTOCOL:get() == PPP then
                if modem.cgact then
                    step = "CGACT"
                else
                    step = "PPPOPEN"
                end
            else
                if modem.cipmode then
                    step = "CIPMODE"
                else
                    step = "CIPOPEN"
                end
            end
            return
        elseif reg then
            local status_map = {
                [0] = "0: not registered, not searching",
                [1] = "1: registered, home network",
                [2] = "2: not registered, searching",
                [3] = "3: registration denied",
                [4] = "4: unknown",
                [5] = "5: registered, roaming",
                [6] = "6: registered for SMS only (home network)",
                [7] = "7: registered for SMS only (roaming)",
                [8] = "8: attached for emergency services only",
                [9] = "9: registered for CSFB not preferred",
            }
            local status = status_map[tonumber(reg)] or (tostring(reg) .. ": unknown status")
            -- CREG 状态不提示
            if reg == "0" then
                AT_send("AT+CFUN=1\r\n")
                AT_send("AT+COPS?\r\n")
            end
        end
    end
    AT_send('AT+CREG?\r\n')
end

local last_data_ms = millis()
local pending_to_modem = ""
local pending_to_fc = ""
local pending_to_parse = ""

--[[
    reset connection buffers
--]]
local function reset_buffers()
    last_data_ms = millis()
    pending_to_modem = ""
    pending_to_fc = ""
    pending_to_parse = ""
    cmux.buffers[DLC_AT] = ""
    cmux.buffers[DLC_DATA] = ""
    while ser_device:available() > 0 do
        ser_device:readstring(512)
    end
end

--[[
    activate network
--]]
function step_CGACT()
    local s = uart_read()
    if handle_error(s) then
        return
    end
    if s and s:find('\r\nOK\r\n') then
        -- CGACT 就绪不提示
        if LTE_PROTOCOL:get() == PPP then
            step = "PPPOPEN"
            last_data_ms = millis()
        else
            step = "CIPMODE"
        end
        return
    end
    data_send(modem.cgact)
    if modem.cfun then
        data_send(modem.cfun)
    end
end

--[[
    set the modem to transparent mode
--]]
function step_CIPMODE()
    local s = uart_read()
    if s:find('AT+CACID=0,0') then
        -- 网络上下文就绪不提示
        step = "NETOPEN"
        return
    end
    if handle_error(s) then
        return
    end
    if s:find('\r\r\nOK\r') then
        -- 透明模式就绪不提示
        step = "NETOPEN"
        return
    end
    data_send(modem.cipmode)
end

--[[
    setup CMUX multiplexing mode
--]]
function step_CMUX()
    local s = uart_read()
    if s then
        if s:find("CME ERROR") then
            AT_send('AT+CFUN=1\r\n')
        elseif #s >= 4 and (cmux.parse_cmux_frame(s) or s:find('CMUX=0\r\r\nOK\r')) then
            -- CMUX 模式就绪不提示
            -- send SABM frames to establish the DLCs
            cmux.send_sabm()
            step = "BAUD"
            return
        end
    end
    AT_send(modem.cmux)
end

--[[
    open the network stack
    needed to be able to open a TCP or UDP connection
--]]
function step_NETOPEN()
    if not modem.netopen then
        step = "CIPOPEN"
        return
    end
    local s = uart_read()
    if s:find("AT+CNACT=0,1") and s:find("ERROR") and modem.netclose then
        data_send(modem.netclose)
        return
    end
    if handle_error(s) then
        return
    end
    if s and (s:find('NETOPEN\r') or s:find('ACTIVE\r')) and s:find('OK\r') then
        -- NETOPEN 就绪不提示
        step = "CIPOPEN"
        return
    end
    data_send(modem.netopen)
end

--[[
    open PPP mode
--]]
function step_PPPOPEN()
    local s = uart_read()
    if s and modem.cgact and s:find("\r\nNO CARRIER\r\n") then
        write_lte_ppp_ready_flag(false)
        send_data_reset()
        step = "ATI"
        return
    end
    if s and s:find("CME ERROR:") then
        write_lte_ppp_ready_flag(false)
        send_data_reset()
        step = "ATI"
        return
    end
    if handle_error(s) then
        return
    end

    if s and s:find('CONNECT') then
        lte_gcs_link_up_once()
        reset_buffers()
        step = "CONNECTED"
        ppp_connected_ms = nil  -- 进入 CONNECTED 后计时，延迟写 NFZ flag
        return
    end
    data_send(modem.pppopen)
end

--[[
    open a TCP or UDP connection to the server
    the server IP and port are defined in the parameters
--]]
function step_CIPOPEN()
    local s = uart_read()
    if handle_error(s) then
        return
    end

    if s then
        if s == "" and modem.cipclose then
            -- possibly need to close an old connection after restarting
            AT_send(modem.cipclose)
        end
        if s:find('+CAOPEN: 0,0') and s:find('OK\r') and modem.caswitch then
            data_send(modem.caswitch)
            return
        end
        if s:find('CONNECT') or (s:find('+CAOPEN: 0,0') and s:find('OK\r')) then
            lte_gcs_link_up_once()
            reset_buffers()
            step = "CONNECTED"
            return
        end
    end
    if LTE_SERVER_PORT:get() <= 0 then
        gcs:send_text(MAV_SEVERITY.ERROR, "Must set LTE_SERVER_PORT")
        return
    end
    local cipopen
    if option_enabled(LTE_OPTIONS_TCP) then
        cipopen = modem.cipopen_tcp
    else
        cipopen = modem.cipopen_udp
    end
    data_send(string.format(cipopen,
                            LTE_SERVER_IP0:get(), LTE_SERVER_IP1:get(), LTE_SERVER_IP2:get(), LTE_SERVER_IP3:get(),
                            LTE_SERVER_PORT:get()))
end

--[[
    check for CSQ reply
--]]
function check_CSQ(s)
    local rssi_raw, ber_raw = s:match("%+CSQ:%s*(%d+),(%d+)")
    if rssi_raw then
        gcs:send_named_float('LTE_RSSI', rssi_raw)
        logger:write("LTE",'RSSI,BER,Bin,Bout','iiII',
                     rssi_raw,
                     ber_raw,
                     stats.bytes_in,
                     stats.bytes_out)
        -- gcs:send_text(MAV_SEVERITY.INFO, string.format("RSSI:%d BER:%d", rssi_raw, ber_raw))
        return true
    end
    return false
end

--[[
    check for CGACT reply
--]]
function check_CGACT(s)
    local ctx, active = s:match("%+CGACT:%s*(%d+),(%d+)")
    if ctx then
        ctx = tonumber(ctx) or 0
        active = tonumber(active) or 0
        -- CGACT 查询结果不提示
        return true
    end
    return false
end

--[[
    check for CPSI reply
--]]
function check_CPSI(s)
    -- example1: +CPSI: LTE,Online,505-02,0xCBE8,36519691,101,EUTRAN-BAND3,1800,5,5,-147,-1143,-764,11
    -- example2: +CPSI: LTE CAT-M1,Online,505-01,0x2036,134523149,238,EUTRAN-BAND28,9410,5,5,-20,-116,-82,6

    if not s:find("+CPSI") then
        return false
    end

    logger:write("LTER","R1,R2",'ZZ', s:sub(1,64), s:sub(65,128))

    local system_mode, operation_mode, mcc_mnc, tac_str, scell_id_str, pcid_str, earfcn_band, ul_freq_str, dl_freq_str, tdd_cfg_str, rsrp_str, rsrq_str, rssi_str, sinr_str =
    s:match("+CPSI:%s*([^,]+),([^,]+),([^,]+),([^,]+),([^,]+),([^,]+),([^,]+),([^,]+),([^,]+),([^,]+),([%-]?%d+),([%-]?%d+),([%-]?%d+),([%-]?%d+)")

    if system_mode then
        -- Convert strings to numbers
        local tac = tonumber(tac_str:match("0x(%w+)"), 16) or tonumber(tac_str) or 0
        local scell_id = tonumber(scell_id_str) or 0
        local pcid = tonumber(pcid_str) or 0
        local ul_freq = tonumber(ul_freq_str) or 0
        local dl_freq = tonumber(dl_freq_str) or 0
        local tdd_cfg = tonumber(tdd_cfg_str) or 0
        local rsrp = tonumber(rsrp_str) or 0
        local rsrq = tonumber(rsrq_str) or 0
        local rssi = tonumber(rssi_str) or 0
        local sinr = tonumber(sinr_str) or 0
        local band = earfcn_band:match("[^%d]+(%d+)") or -1
        logger:write("LTES",'Md,Op,MCC,TAC,CID,PID,BND,F,DF,TDD,RP,RQ,RS,SR','NNNIIINHhhhhhh',
                     system_mode, operation_mode, mcc_mnc, tac, scell_id, pcid, earfcn_band,
                     ul_freq, dl_freq, tdd_cfg, rsrp, rsrq, rssi, sinr)
        if option_enabled(LTE_OPTIONS_SIGNALS) then
            gcs:send_named_float('LTE_RSRP', rsrp)
            gcs:send_named_float('LTE_RSRQ', rsrq)
            gcs:send_named_float('LTE_BAND', band)
            -- shift to remove antenna selection within tower
            gcs:send_named_float('LTE_CID', scell_id>>8)
            local mcc, mnc = mcc_mnc:match("(%d+)-(%d+)")
            if mcc and mnc then
                gcs:send_named_float('LTE_MCCMNC', mcc*100+mnc)
            end
        end
        return true
    end
    return false
end

--[[
    check for QENG reply
--]]
function check_QENG(s)
    -- Example1: +QENG: "servingcell","NOCONN","LTE","FDD",505,02,12AED4A,445,3750,8,3,3,CBE8,-99,-14,-71,53,30
    -- Example2: +QENG: "servingcell","NOCONN","LTE","FDD",505,02,22D3F32,271,9260,28,3,3,CBE8,-109,-15,-78,38,20
    -- +QENG:"servingcell",<state>,"LTE",<is_tdd>,<mcc>,<mnc>,<cellid>,<pcid>,<earfcn>,<freq_band_ind>,<ul_bandwidth>,<dl_bandwidth>,<tac>,<rsrp>,<rsrq>,<rssi>,<sinr>,<srxlev>
    if not s:find("+QENG") then
        return false
    end

    logger:write("LTER","R1,R2",'ZZ', s:sub(1,64), s:sub(65,128))

    local mcc_str, mnc_str, cid_hex, pcid_str, earfcn_str, band_str, tac_hex, rsrp_str, rsrq_str, rssi_str, sinr_str =
        s:match('+QENG:%s+"servingcell","[^"]+","LTE","[^"]+",(%d+),(%d+),([%x]+),(%d+),(%d+),(%d+),%d+,%d+,([%x]+),([%-]?%d+),([%-]?%d+),([%-]?%d+)')

    if mcc_str then
        local tac = tonumber(tac_hex, 16) or 0
        local cid = tonumber(cid_hex, 16) or 0
        local pcid = tonumber(pcid_str) or 0
        local earfcn = tonumber(earfcn_str) or 0
        local rsrp = tonumber(rsrp_str) or 0
        local rsrq = tonumber(rsrq_str) or 0
        local rssi = tonumber(rssi_str) or 0
        local sinr = tonumber(sinr_str) or 0
        local band = tonumber(band_str) or -1
        local mcc = tonumber(mcc_str) or 0
        local mnc = tonumber(mnc_str) or 0

        logger:write("LTES", 'MCC,MNC,TAC,CID,PID,EF,RSRP,RSRQ,RSSI,SINR', 'iiiiiiiiii',
                     mcc, mnc, tac, cid, pcid, earfcn, rsrp, rsrq, rssi, sinr)

        if option_enabled(LTE_OPTIONS_SIGNALS) then
            gcs:send_named_float('LTE_RSRP', rsrp)
            gcs:send_named_float('LTE_RSRQ', rsrq)
            gcs:send_named_float('LTE_BAND', band)
            -- shift to remove antenna selection within tower
            gcs:send_named_float('LTE_CID', cid>>8)
            gcs:send_named_float('LTE_MCCMNC', mcc*100+mnc)
        end
        return true
    end
    return false
end

--[[
    handle AT replies in CMUX mode
--]]
function handle_AT_reply(s)
    check_CSQ(s)
    if check_CPSI(s) then
        return
    end
    if check_QENG(s) then
        return
    end
    if check_CGACT(s) then
        return
    end

    if s:find("PPPD: DISCONNECTED") then
        step = "PPPOPEN"
    end
end

local last_CSQ_ms = millis()
local last_CSQ_reply_ms = uint32_t(0)
local last_parse_ms = uint32_t(0)
local last_route_ms = uint32_t(0)
local last_send_data_ms = uint32_t(0)

--[[
    handle data while connected
--]]
function step_CONNECTED()
    local s = uart:readstring(512)
    stats.bytes_in = stats.bytes_in + #s
    if option_enabled(LTE_OPTIONS_LOGALL) then
        log_data(s, '<<<')
    end
    if s and s:find('\r\nCLOSED\r\n') then
        lte_gcs_link_clear()
        gcs:send_text(MAV_SEVERITY.WARNING, 'LTE: 断开，重连中')
        uom_close()
        ppp_connected_ms = nil
        write_lte_ppp_ready_flag(false)
        step = "CIPOPEN"
        return
    end
    if s and s:find('PPPD: DISCONNECTED\r\n') then
        lte_gcs_link_clear()
        gcs:send_text(MAV_SEVERITY.WARNING, 'LTE: PPP断开，重连中')
        uom_close()
        ppp_connected_ms = nil
        write_lte_ppp_ready_flag(false)
        step = "PPPOPEN"
        return
    end
    local now_ms = millis()
    -- PPP 模式：CONNECT 满 PPP_NFZ_FLAG_DELAY_MS 后再通知禁飞脚本（减轻 PPP 抢跑）
    if LTE_PROTOCOL:get() == PPP then
        if not ppp_connected_ms then
            ppp_connected_ms = now_ms
        elseif (now_ms - ppp_connected_ms):toint() >= PPP_NFZ_FLAG_DELAY_MS then
            write_lte_ppp_ready_flag(true)
        end
    end
    if s and #s > 0 then
        if not cmux_enabled() then
            pending_to_fc = pending_to_fc .. s
            last_data_ms = now_ms
        else
            pending_to_parse = pending_to_parse .. s
            pending_to_parse = cmux.feed_uart_in(pending_to_parse)
            if now_ms - last_parse_ms > 1000 then
                pending_to_parse = ""
            end
            if #cmux.buffers[DLC_AT] > 0 then
                last_parse_ms = now_ms
                --gcs:send_text(MAV_SEVERITY.INFO, string.format("AT reply %d", #cmux.buffers[DLC_AT]))
                handle_AT_reply(cmux.buffers[DLC_AT])
                cmux.buffers[DLC_AT] = ""
            end
            if #cmux.buffers[DLC_DATA] > 0 then
                last_data_ms = now_ms
                -- gcs:send_text(MAV_SEVERITY.INFO, string.format("data input %d", #cmux.buffers[DLC_DATA]))
                last_parse_ms = now_ms
                pending_to_fc = pending_to_fc .. cmux.buffers[DLC_DATA]
                cmux.buffers[DLC_DATA] = ""
            end
        end
    elseif LTE_TIMEOUT:get() > 0 and now_ms - last_data_ms > uint32_t(LTE_TIMEOUT:get() * 1000) then
        gcs:send_text(MAV_SEVERITY.ERROR, 'LTE_modem: timeout')
        reset_to_ATI()
        return
    end
    s = ser_device:readstring(512)
    if s then
        pending_to_modem = pending_to_modem .. s
    end

    --[[
        going via these pending buffers allows for rapid bursts of data and takes advantage
        of the hardware flow control
    --]]
    local buffer_limit = 10240 -- so we don't run out of memory
    if #pending_to_modem > buffer_limit then
        pending_to_modem = ""
    end
    if #pending_to_fc > buffer_limit then
        pending_to_fc = ""
    end

    local quota = 0
    if LTE_TX_RATE:get() > 0 then
        local dt = (now_ms - last_send_data_ms):tofloat()*0.001
        quota = math.floor(dt * LTE_TX_RATE:get())
    end

    local data_sent = 0
    while #pending_to_modem > 0 do
        local n = #pending_to_modem
        if n > 100 then
            n = 100
        end
        if quota > 0 and quota - data_sent < n then
            n = quota - data_sent
        end
        local data = pending_to_modem:sub(1, n)

        data_sent = data_sent + #data
        last_send_data_ms = now_ms

        -- gcs:send_text(MAV_SEVERITY.INFO, string.format("data output %d", n))
        if not data_send_connected(data) then
            break
        end
        pending_to_modem = pending_to_modem:sub(n + 1)

        if quota > 0 and data_sent >= quota then
            break
        end
    end
    if #pending_to_fc > 0 then
        local nwritten = ser_device:writestring(pending_to_fc)
        if nwritten > 0 then
            pending_to_fc = pending_to_fc:sub(nwritten + 1)
        end
    end
    if cmux_enabled() and not option_enabled(LTE_OPTIONS_NOSIGQUERY) then
        -- if we support CMUX then request CSQ signal strength at 1Hz
        if now_ms - last_CSQ_ms > 1000 then
            last_CSQ_ms = now_ms
            AT_send("AT+CSQ\r\n")
            if modem.cpsi then
                AT_send(modem.cpsi)
            end
        end
        if now_ms - last_CSQ_reply_ms > 5000 then
            last_CSQ_reply_ms = now_ms
            gcs:send_named_float('LTE_RSSI', -1)
        end
        if LTE_MCCMNC:get() ~= last_mccmnc and modem.mccmnc then
            set_MCCMNC()
            gcs:send_text(MAV_SEVERITY.INFO, string.format("LTE_modem: set MCCMNC=%d", last_mccmnc))
            step = "CREG"
        end
        if LTE_BAND:get() ~= last_band and (modem.setband or modem.setband_mask) then
            set_BAND()
            if last_band ~= 0 then
                step = "CREG"
            end
            gcs:send_text(MAV_SEVERITY.INFO, string.format("LTE_modem: set BAND=%d", last_band))
        end
    end

    -- UOM MQTT 上报（PPP 连通后启动）
    uom_update()

    -- newer firmware allows for multiple PPP interfaces and custom routing
    if supports_routing and now_ms - last_route_ms > 1000 then
        last_route_ms = now_ms
        local dest = uint32_t(LTE_ROUTE_IP0:get())<<24
        dest = dest | uint32_t(LTE_ROUTE_IP1:get())<<16
        dest = dest | uint32_t(LTE_ROUTE_IP2:get())<<8
        dest = dest | uint32_t(LTE_ROUTE_IP3:get())
        if dest ~= uint32_t(0) then
            networking:add_route(0, 1, dest, math.floor(LTE_ROUTE_MASK:get())) -- luacheck: ignore 143
        end
    end
end

step_count = 0
last_step = nil

function run_step()
    if change_baud then
        uart:begin(change_baud)
        change_baud = nil
    end

    if step == "CONNECTED" then
        -- run the connected step at 200Hz
        step_CONNECTED()
        step_count = 0
        return 5
    end

    -- prevent getting stuck
    if step == last_step and step ~= "ATI" then
        step_count = step_count + 1
        if step_count > 50 then
            reset_to_ATI()
        end
    else
        step_count = 0
    end
    last_step = step

    if step == "ATI" then
        step_ATI()
        return 1100
    end

    if step == "BAUD" then
        step_BAUD()
        return 500
    end

    if step == "CREG" then
        step_CREG()
        return 1000
    end

    if step == "CGACT" then
        step_CGACT()
        return 500
    end
    
    if step == "CIPMODE" then
        step_CIPMODE()
        return 200
    end

    if step == "NETOPEN" then
        step_NETOPEN()
        return 200
    end

    if step == "CONFIG" then
        step_CONFIG()
        return 200
    end
    
    if step == "CMUX" then
        step_CMUX()
        return 200
    end

    if step == "CPIN" then
        step_CPIN()
        return 500
    end
    
    if step == "PPPOPEN" then
        step_PPPOPEN()
        return 200
    end
    
    if step == "CIPOPEN" then
        step_CIPOPEN()
        return 200
    end

    gcs:send_text(MAV_SEVERITY.ERROR, string.format("LTE_modem: bad step %s", step))
    reset_to_ATI()
end

local function update()
    if LTE_ENABLE:get() == 0 then
        return update, 500
    end
    local delay = run_step()
    uart_write_pending()
    return update, delay
end

return update,500
