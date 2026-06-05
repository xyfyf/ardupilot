--[[
    1noflyzone_checker.lua  v3.0 — 方案 A：PPP + TCP HTTP GET 直连后端

    链路：
      Air780E 4G → LTE_modem.lua(PPP) → NET: IP → 本脚本 Socket(TCP:80)

    后端接口（查询无人机禁飞区配置列表）：
      GET http://47.120.16.113/train-api/train/noFlyZone/list
          ?pageNum=1&pageSize=<NFZ_PAGE_SIZE 默认10，勿9999>
          [&updateTimestamp=<上次最大 updateTime 毫秒>]
      内存：与 LTE_modem+UOM 共用 SCR_HEAP_SIZE，建议 307200~409600

    后端要求：
      - HTTP/1.x 明文端口 80（飞控 Lua 无 HTTPS）
      - 响应 JSON：code=200, rows[], 每条含 orbit[{lng,lat},...]
      - 勿对 4G 出口 IP 做严格白名单（IP 随运营商 NAT 变化）
      - 响应建议 <1MB；过大可分页或 gzip（脚本未解压 gzip）

    依赖（须与 LTE_modem.lua 同时部署）：
      - LTE_modem.lua：LTE_PROTOCOL=48(PPP)、NET_ENABLE=1、SCR_SDEV_EN=1
      - 标志文件 APM/lte_ppp_ready.flag：LTE 侧 PPP CONNECT 后为 '1'，断开/复位为 '0'
      - 本脚本与 LTE_modem 分属两个 Lua 任务，勿阻塞式 connect（会拖死 PPP）
--]]

local MAV_SEVERITY = {EMERGENCY=0,ALERT=1,CRITICAL=2,ERROR=3,WARNING=4,NOTICE=5,INFO=6,DEBUG=7}

-- ============================================================
-- 参数
-- ============================================================
local PARAM_TABLE_KEY    = 109
local PARAM_TABLE_PREFIX = "NFZ_"
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 4), 'NFZ: 无法添加参数表')

local function bind_param(name, idx, default)
    assert(param:add_param(PARAM_TABLE_KEY, idx, name, default),
        string.format('NFZ: 无法添加参数 %s', name))
    return Parameter(PARAM_TABLE_PREFIX .. name)
end

--[[
    // @Param: NFZ_ENABLE
    // @DisplayName: 禁飞区检查使能
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
--]]
local NFZ_ENABLE  = bind_param('ENABLE',  1, 1)

--[[
    // @Param: NFZ_REFRESH
    // @DisplayName: 周期刷新间隔（秒）
    // @Range: 60 86400
    // @User: Standard
--]]
local NFZ_REFRESH = bind_param('REFRESH', 2, 3600)

--[[
    // @Param: NFZ_PAGE_SIZE
    // @DisplayName: 禁飞区列表每页条数
    // @Description: HTTP 单次拉取条数。过大(如9999)且多边形顶点很多时会耗尽 SCR_HEAP_SIZE
    // @Range: 1 100
    // @User: Standard
--]]
local NFZ_PAGE_SIZE = bind_param('PAGE_SIZE', 3, 10)

-- ============================================================
-- API（方案 A：直连 47.120.16.113，IP 变更时改 API_IP）
-- ============================================================
local API_HOST = "drone.test.effort-tech.com"
local API_IP   = "47.120.16.113"
local API_PORT = 80
-- 单条禁飞区多边形最多保留顶点数（深圳福田等超大区会抽稀，否则 Lua OOM）
local MAX_ORBIT_PTS = 80
-- 单次 HTTP 响应缓冲上限（字节），与 SCR_HEAP_SIZE 匹配
local NFZ_MAX_RESP_BYTES = 384 * 1024

local CACHE_FILE      = "APM/nofly_zones.json"
-- 路径须与 LTE_modem.lua 中 LTE_PPP_READY_FILE 一致
local LTE_READY_FILE  = "APM/lte_ppp_ready.flag"  -- LTE_modem.lua 写入 '1'/'0'

local PPP_AFTER_LTE_MS  = 10000   -- flag='1' 后再等 NET（LTE 已延迟约15s才写 flag）
local WAIT_LOG_MS       = 60000   -- 等待中每 60s 打一条
local TCP_CONN_MS       = 45000   -- 非阻塞 connect 最长等待

-- ============================================================
-- 运行时状态
-- ============================================================
local nfz = {
    zones          = {},
    zones_by_id    = {},
    zones_ready    = false,
    max_update_ms  = 0,
    last_fetch_sec = 0,

    http_state     = "IDLE",
    sock           = nil,
    resp_buf       = "",
    conn_timeout   = uint32_t(0),
    recv_timeout   = uint32_t(0),
    is_incremental = false,
    recv_eof       = false,

    fail_count     = 0,
    next_retry_sec = 0,
    wait_ppp_logged = false,
    last_wait_log_ms = uint32_t(0),
    lte_ok_ms        = nil,       -- 首次见到 LTE_PPP_READY 的时刻
}

local auth_id = arming:get_aux_auth_id()
if not auth_id then
    gcs:send_text(MAV_SEVERITY.CRITICAL, "NFZ: 无法获取解锁鉴权ID")
end

-- ============================================================
-- JSON 工具
-- ============================================================
local function json_num(s, key)
    local v = s:match('"' .. key .. '"%s*:%s*(%-?[%d%.eE+]+)')
    return v and tonumber(v) or nil
end

local function json_str(s, key)
    return s:match('"' .. key .. '"%s*:%s*"([^"]*)"')
end

-- ============================================================
-- 多边形射线法
-- ============================================================
local function point_in_polygon(lat, lng, polygon)
    local n = #polygon
    if n < 3 then return false end
    local inside = false
    local j = n
    for i = 1, n do
        local xi, yi = polygon[i].lng, polygon[i].lat
        local xj, yj = polygon[j].lng, polygon[j].lat
        if ((yi > lat) ~= (yj > lat)) and
           (lng < (xj - xi) * (lat - yi) / (yj - yi) + xi) then
            inside = not inside
        end
        j = i
    end
    return inside
end

local function check_in_nfz()
    local loc = ahrs:get_location()
    if not loc then return false, nil end
    local lat = loc:lat() * 1e-7
    local lng = loc:lng() * 1e-7
    for _, zone in ipairs(nfz.zones) do
        if zone.is_enable == 1 and zone.eff_status == 1 then
            if point_in_polygon(lat, lng, zone.polygon) then
                return true, zone.name
            end
        end
    end
    return false, nil
end

-- 超大禁飞区抽稀顶点，降低 Lua 内存占用
local function simplify_polygon(polygon)
    local n = #polygon
    if n <= MAX_ORBIT_PTS then
        return polygon
    end
    local out = {}
    local step = n / MAX_ORBIT_PTS
    for i = 1, MAX_ORBIT_PTS do
        local idx = math.floor((i - 1) * step + 1)
        if idx < 1 then idx = 1 end
        if idx > n then idx = n end
        out[#out + 1] = polygon[idx]
    end
    return out
end

-- ============================================================
-- 解析 API JSON（code=200, rows[]）
-- ============================================================
local function merge_api_body(body, is_full)
    if not body or #body < 10 then
        gcs:send_text(MAV_SEVERITY.WARNING, "NFZ: 响应体过短")
        return false
    end

    local api_code = json_num(body, "code")
    if api_code and api_code ~= 200 then
        local msg = json_str(body, "msg") or "未知"
        gcs:send_text(MAV_SEVERITY.WARNING,
            string.format("NFZ: API code=%d %s", api_code, msg))
        return false
    end

    if not body:find('"rows"', 1, true) then
        gcs:send_text(MAV_SEVERITY.WARNING, "NFZ: JSON 缺少 rows")
        return false
    end

    if is_full then
        nfz.zones         = {}
        nfz.zones_by_id   = {}
        nfz.max_update_ms = 0
    end

    local count_new, count_upd = 0, 0
    local pos = 1

    while true do
        local id_pos = body:find('"id":', pos, true)
        if not id_pos then break end

        local next_id   = body:find('"id":', id_pos + 5, true)
        local chunk_end = next_id and (next_id - 1) or #body
        local chunk     = body:sub(id_pos, chunk_end)

        local id = tonumber(chunk:match('"id"%s*:%s*"?(%d+)"?'))
        if id then
            local update_ms  = json_num(chunk, "updateTime") or 0
            local name       = json_str(chunk, "detailAddress") or ("zone_" .. id)
            local is_enable  = json_num(chunk, "isEnable")     or 0
            local eff_status = json_num(chunk, "effectStatus") or 0
            local orbit_str  = chunk:match('"orbit"%s*:%s*(%b[])')

            if update_ms > nfz.max_update_ms then
                nfz.max_update_ms = update_ms
            end

            local polygon = {}
            if orbit_str then
                for pt in orbit_str:gmatch('%b{}') do
                    local plat = json_num(pt, "lat")
                    local plng = json_num(pt, "lng")
                    if plat and plng then
                        polygon[#polygon + 1] = {lat = plat, lng = plng}
                    end
                end
            end

            if #polygon >= 3 then
                if #polygon > MAX_ORBIT_PTS then
                    gcs:send_text(MAV_SEVERITY.INFO,
                        string.format("NFZ: id%d 顶点%d->%d",
                            id, #polygon, MAX_ORBIT_PTS))
                    polygon = simplify_polygon(polygon)
                end
                local zone = {
                    id = id, name = name,
                    is_enable = is_enable, eff_status = eff_status,
                    polygon = polygon,
                }
                if nfz.zones_by_id[id] then
                    nfz.zones[nfz.zones_by_id[id]] = zone
                    count_upd = count_upd + 1
                else
                    nfz.zones[#nfz.zones + 1] = zone
                    nfz.zones_by_id[id] = #nfz.zones
                    count_new = count_new + 1
                end
            end
        end
        pos = id_pos + 5
    end

    if #nfz.zones == 0 then
        gcs:send_text(MAV_SEVERITY.WARNING, "NFZ: 解析后 0 条有效禁飞区")
        return false
    end

    if is_full then
        gcs:send_text(MAV_SEVERITY.INFO,
            string.format("NFZ: 全量 %d 条 max_ts=%d", #nfz.zones, nfz.max_update_ms))
    else
        gcs:send_text(MAV_SEVERITY.INFO,
            string.format("NFZ: 增量 +%d ~%d 总%d",
                count_new, count_upd, #nfz.zones))
    end
    nfz.zones_ready = true
    return true
end

-- ============================================================
-- SD 缓存
-- ============================================================
local function save_cache()
    local f = io.open(CACHE_FILE, 'w')
    if not f then
        gcs:send_text(MAV_SEVERITY.WARNING, "NFZ: SD 写入失败")
        return
    end
    f:write(string.format('{"ts":%d,"z":[', nfz.max_update_ms))
    local first_zone = true
    for _, z in ipairs(nfz.zones) do
        if not first_zone then f:write(',') end
        first_zone = false
        local safe_name = z.name:gsub('\\', '\\\\'):gsub('"', '\\"')
        f:write(string.format('{"i":%d,"n":"%s","e":%d,"s":%d,"p":[',
            z.id, safe_name, z.is_enable, z.eff_status))
        local first_pt = true
        for _, p in ipairs(z.polygon) do
            if not first_pt then f:write(',') end
            first_pt = false
            f:write(string.format('[%.8f,%.8f]', p.lng, p.lat))
        end
        f:write(']}')
    end
    f:write(']}')
    f:close()
    gcs:send_text(MAV_SEVERITY.INFO,
        string.format("NFZ: 缓存 %d 条", #nfz.zones))
end

local function load_cache()
    local f = io.open(CACHE_FILE, 'r')
    if not f then return false end
    local raw = f:read('*a')
    f:close()
    if not raw or #raw < 10 then return false end

    local ts = tonumber(raw:match('"ts"%s*:%s*(%d+)'))
    if not ts then return false end
    nfz.max_update_ms = ts

    nfz.zones       = {}
    nfz.zones_by_id = {}

    for obj in raw:gmatch('%b{}') do
        local id = tonumber(obj:match('"i"%s*:%s*(%d+)'))
        if id and id ~= 0 then
            local name       = obj:match('"n"%s*:%s*"([^"]*)"') or ("zone_" .. id)
            local is_enable  = tonumber(obj:match('"e"%s*:%s*(%d+)')) or 0
            local eff_status = tonumber(obj:match('"s"%s*:%s*(%d+)')) or 0
            local pts_str    = obj:match('"p"%s*:%s*(%b[])')
            local polygon = {}
            if pts_str then
                for pair in pts_str:gmatch('%b[]') do
                    local lng_s, lat_s = pair:match('%[([^,]+),([^%]]+)%]')
                    if lng_s and lat_s then
                        polygon[#polygon + 1] = {
                            lat = tonumber(lat_s),
                            lng = tonumber(lng_s),
                        }
                    end
                end
            end
            if #polygon >= 3 then
                nfz.zones[#nfz.zones + 1] = {
                    id = id, name = name,
                    is_enable = is_enable, eff_status = eff_status,
                    polygon = polygon,
                }
                nfz.zones_by_id[id] = #nfz.zones
            end
        end
    end

    if #nfz.zones == 0 then return false end
    nfz.zones_ready = true
    gcs:send_text(MAV_SEVERITY.INFO,
        string.format("NFZ: SD 缓存 %d 条", #nfz.zones))
    return true
end

-- ============================================================
-- HTTP GET 状态机（TCP Socket，方案 A）
-- ============================================================
local function build_api_path()
    local ps = math.floor(NFZ_PAGE_SIZE:get())
    if ps < 1 then ps = 1 end
    if ps > 100 then ps = 100 end
    local path = string.format(
        "/train-api/train/noFlyZone/list?pageNum=1&pageSize=%d", ps)
    if nfz.max_update_ms > 0 then
        path = path .. string.format("&updateTimestamp=%d", nfz.max_update_ms)
    end
    return path
end

-- HTTP 响应是否已收齐（Content-Length 或 Connection:close 后对端关闭）
local function http_recv_complete()
    if #nfz.resp_buf < 12 then
        return false
    end
    local hdr_end = nfz.resp_buf:find("\r\n\r\n", 1, true)
    if not hdr_end then
        return false
    end
    local cl = tonumber(nfz.resp_buf:match("Content%-Length:%s*(%d+)"))
    if cl then
        local body = nfz.resp_buf:sub(hdr_end + 4)
        return #body >= cl
    end
    return nfz.recv_eof
end

-- 须 LTE_modem PPP CONNECTED（标志文件为 '1'）后再拉，避免 CREG 未稳时抢 PPP
local function lte_modem_connected()
    local f = io.open(LTE_READY_FILE, 'r')
    if not f then
        return false
    end
    local v = f:read(1)
    f:close()
    return v == '1'
end

local function ppp_ready()
    if not lte_modem_connected() then
        nfz.lte_ok_ms = nil
        return false
    end
    if not nfz.lte_ok_ms then
        nfz.lte_ok_ms = millis()
    end
    if (millis() - nfz.lte_ok_ms):toint() < PPP_AFTER_LTE_MS then
        return false
    end
    -- 勿在此处 socket.tcp() 探测：会与 PPP/UOM 抢网络栈导致 PPP reconnecting
    return true
end

local function http_reset(reason)
    if nfz.sock then
        nfz.sock:close()
        nfz.sock = nil
    end
    nfz.http_state = "IDLE"
    nfz.resp_buf   = ""
    nfz.recv_eof   = false
    if reason then
        gcs:send_text(MAV_SEVERITY.WARNING, "NFZ: " .. reason)
        nfz.fail_count = nfz.fail_count + 1
        -- 失败后多等一会，避免与 LTE_modem 抢 Lua、加剧 PPP 重连
        local backoff = math.min(60 * (2 ^ (nfz.fail_count - 1)), 300)
        nfz.next_retry_sec = millis():tofloat() * 0.001 + backoff
        gcs:send_text(MAV_SEVERITY.INFO,
            string.format("NFZ: %d 秒后重试", backoff))
    else
        nfz.fail_count = 0
        nfz.next_retry_sec = 0
    end
end

local function new_tcp_sock()
    if socket and socket.tcp then
        local s = socket.tcp()
        if s then
            return s, "socket.tcp"
        end
    end
    local s = Socket(0)
    if s then
        return s, "Socket(0)"
    end
    return nil, nil
end

local function http_step()
    local now = millis()

    if nfz.http_state == "IDLE" then
        local via
        nfz.sock, via = new_tcp_sock()
        if not nfz.sock then
            gcs:send_text(MAV_SEVERITY.INFO, "NFZ: 无法创建 TCP 套接字")
            return
        end
        nfz.resp_buf       = ""
        nfz.is_incremental = (nfz.max_update_ms > 0)
        gcs:send_text(MAV_SEVERITY.INFO,
            string.format("NFZ: TCP %s:%d %s (%s)",
                API_IP, API_PORT, via or "?",
                nfz.is_incremental and "增量" or "全量"))

        -- 禁止阻塞 connect：会卡住整个 Lua，导致 LTE_modem 无响应 timeout/PPP 重连
        nfz.sock:set_blocking(false)
        nfz.sock:connect(API_IP, API_PORT)
        nfz.http_state   = "CONNECTING"
        nfz.conn_timeout = now + uint32_t(TCP_CONN_MS)
        return
    end

    if nfz.http_state == "CONNECTING" then
        if now > nfz.conn_timeout then
            http_reset("TCP超时(先稳PPP/LTE,勿刷reconnecting)")
            return
        end
        if not nfz.sock:is_connected() and not nfz.sock:pollout(0) then
            return
        end

        local path = build_api_path()
        local req  = string.format(
            "GET %s HTTP/1.1\r\nHost: %s\r\nAccept: application/json\r\nConnection: close\r\n\r\n",
            path, API_HOST)
        local sent = nfz.sock:send(req, #req)
        if not sent or sent <= 0 then
            http_reset("HTTP发送失败")
            return
        end
        nfz.http_state   = "RECEIVING"
        nfz.recv_timeout = now + uint32_t(120000)
        nfz.recv_eof     = false
        gcs:send_text(MAV_SEVERITY.INFO, "NFZ: GET " .. path:sub(1, 80))
        return
    end

    if nfz.http_state == "RECEIVING" then
        if now > nfz.recv_timeout then
            if http_recv_complete() then
                nfz.recv_eof = true
            else
                http_reset("接收超时")
                return
            end
        end

        -- 非阻塞 recv：须先 pollin，否则 nil 会被误判为收完（导致 0 字节）
        if not nfz.recv_eof then
            if nfz.sock.pollin and nfz.sock:pollin(0) then
                local chunk = nfz.sock:recv(4096)
                if chunk and #chunk > 0 then
                    nfz.resp_buf = nfz.resp_buf .. chunk
                    if #nfz.resp_buf > NFZ_MAX_RESP_BYTES then
                        http_reset("响应过大,减小NFZ_PAGE_SIZE")
                        return
                    end
                elseif chunk ~= nil and #chunk == 0 then
                    nfz.recv_eof = true
                end
            end
        end

        if not http_recv_complete() then
            return
        end

        gcs:send_text(MAV_SEVERITY.INFO,
            string.format("NFZ: 收到 %d 字节", #nfz.resp_buf))

        local status_code = nfz.resp_buf:match("HTTP/%S+ (%d+)")
        if status_code and status_code ~= "200" then
            gcs:send_text(MAV_SEVERITY.WARNING, "NFZ: HTTP " .. status_code)
            http_reset(nil)
            return
        end

        local body = nfz.resp_buf:match("\r\n\r\n(.*)")
        nfz.resp_buf = ""  -- 尽早释放大字符串
        if not body or #body < 5 then
            http_reset("响应体为空")
            return
        end

        local ok = merge_api_body(body, not nfz.is_incremental)
        body = nil
        if ok then
            nfz.last_fetch_sec = millis():tofloat() * 0.001
            save_cache()
        end
        http_reset(nil)
    end
end

local function try_fetch()
    if not ppp_ready() then
        local now = millis()
        if not nfz.wait_ppp_logged then
            nfz.wait_ppp_logged = true
            nfz.last_wait_log_ms = now
            if lte_modem_connected() then
                gcs:send_text(MAV_SEVERITY.INFO,
                    string.format("NFZ: LTE已连,等NET约%ds",
                        math.floor(PPP_AFTER_LTE_MS / 1000)))
            else
                gcs:send_text(MAV_SEVERITY.INFO, "NFZ: wait LTE PPP flag=1")
            end
        elseif (now - nfz.last_wait_log_ms):toint() > WAIT_LOG_MS then
            nfz.last_wait_log_ms = now
            if not lte_modem_connected() then
                gcs:send_text(MAV_SEVERITY.INFO, "NFZ: still wait PPP flag")
            else
                local left_ms = PPP_AFTER_LTE_MS - (millis() - nfz.lte_ok_ms):toint()
                if left_ms < 0 then left_ms = 0 end
                gcs:send_text(MAV_SEVERITY.INFO,
                    string.format("NFZ: 仍等待NET(%ds)", math.floor(left_ms / 1000)))
            end
        end
        return
    end
    if nfz.wait_ppp_logged then
        gcs:send_text(MAV_SEVERITY.INFO, "NFZ: PPP就绪,开始HTTP")
    end
    nfz.wait_ppp_logged = false
    http_step()
end

-- ============================================================
-- 主循环
-- ============================================================
local function update()
    if NFZ_ENABLE:get() ~= 1 then
        return update, 5000
    end

    -- PPP 未就绪时降低轮询频率，减轻与 LTE_modem 的 Lua 竞争
    local loop_ms = 2000
    if nfz.http_state == "IDLE" and not ppp_ready() then
        loop_ms = 5000
    end

    if auth_id then
        if not nfz.zones_ready then
            arming:set_aux_auth_passed(auth_id)
        else
            local in_nfz, zone_name = check_in_nfz()
            if in_nfz then
                arming:set_aux_auth_failed(auth_id,
                    string.format("禁飞区内禁止解锁:%s",
                        (zone_name and zone_name:sub(1, 20)) or "未知"))
            else
                arming:set_aux_auth_passed(auth_id)
            end
        end
    end

    local now_sec = millis():tofloat() * 0.001

    if nfz.http_state ~= "IDLE" then
        http_step()
        return update, 2000
    else
        local elapsed    = now_sec - nfz.last_fetch_sec
        local need_fetch = (not nfz.zones_ready) or (elapsed > NFZ_REFRESH:get())

        if need_fetch then
            if now_sec < nfz.next_retry_sec then
                return update, loop_ms
            end
            if not nfz.zones_ready then
                load_cache()
            end
            try_fetch()
        end
    end

    return update, loop_ms
end

-- ============================================================
-- 启动
-- ============================================================
gcs:send_text(MAV_SEVERITY.INFO, "NFZ: v3.0 wait LTE PPP then HTTP")

if NFZ_ENABLE:get() == 1 then
    load_cache()
end

return update, 5000
