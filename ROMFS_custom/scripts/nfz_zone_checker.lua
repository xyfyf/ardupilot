--[[
  nfz_zone_checker.lua

  禁飞区识别：优先用机场等多边形，无多边形时才用圆。
  SD 卡不整表进内存，只缓存飞机附近窗口；支持参数/MAVLink 增删改圆。

  SD 卡 EFT/nfz/ :
    index.csv    id,kind,lat,lng,r     kind:0圆 1多边形（r=包围半径，仅窗口筛选）
    circles.csv  id,lat,lng,r
    polys.csv    id,n,lon,lat,lon,lat,...
    overlay.csv  动态圆（脚本维护）
    deleted.csv  已删 id

  默认包编在固件 @ROMFS/nfz/；开机若 SD 缺文件则写出，已有则不改。

  参数 NFZ_* 增删改动态圆（本机 ROMFS 无 COMMAND_LONG 模块，暂不用 MAVLink 收令）。

  NFZ_ACTION:
    0 = 仅告警
    1 = 靠近边界提前 Loiter 刹车悬停
    2 = 靠近/进入后 RTL 返航（默认）

  NFZ_BASE:
    0 = 只用 overlay
    1 = 分帧加载基座（默认；公司附近机场等也会生效）
--]]

---@diagnostic disable: need-check-nil, undefined-global

-- 打包常量/状态，避免 main chunk local > 100（AP Lua 上限）
local CFG = {
    SCRIPT = "NFZ",
    LOOP_MS = 200,
    WARN_MS = 30000,
    MODE_LOITER = 5,
    MODE_RTL = 6,
    MAX_CIRCLES = 40,
    MAX_POLYS = 32,
    MAX_VERTS = 640,
    MAX_OVERLAY = 32,
    MAX_DELETED = 64,
    MAX_WANT = 48,
    INDEX = "EFT/nfz/index.csv",
    CIRCLES = "EFT/nfz/circles.csv",
    POLYS = "EFT/nfz/polys.csv",
    OVERLAY = "EFT/nfz/overlay.csv",
    DELETED = "EFT/nfz/deleted.csv",
    ROMFS = "@ROMFS/nfz/",
    POLYS_EXPECTED = 191708,
    LINES_PER_SLICE = 30,
    SEED_CHUNK = 2048,
    SEED_CHUNKS = 8,
}

local PAR = {}
assert(param:add_table(98, "NFZ_", 11), "NFZ: add_table fail")
assert(param:add_param(98, 1, "ENABLE", 0), "NFZ: ENABLE")
assert(param:add_param(98, 2, "ARMBLK", 1), "NFZ: ARMBLK")
assert(param:add_param(98, 3, "ACTION", 2), "NFZ: ACTION")
assert(param:add_param(98, 4, "RADIUS", 40), "NFZ: RADIUS")
assert(param:add_param(98, 5, "CMD", 0), "NFZ: CMD")
assert(param:add_param(98, 6, "ID", 0), "NFZ: ID")
assert(param:add_param(98, 7, "LAT", 0), "NFZ: LAT")
assert(param:add_param(98, 8, "LNG", 0), "NFZ: LNG")
assert(param:add_param(98, 9, "RAD", 0), "NFZ: RAD")
assert(param:add_param(98, 10, "MARGIN", 50), "NFZ: MARGIN")
assert(param:add_param(98, 11, "BASE", 1), "NFZ: BASE")
PAR.enable = Parameter("NFZ_ENABLE")
PAR.armblk = Parameter("NFZ_ARMBLK")
PAR.action = Parameter("NFZ_ACTION")
PAR.radius = Parameter("NFZ_RADIUS")
PAR.cmd    = Parameter("NFZ_CMD")
PAR.id     = Parameter("NFZ_ID")
PAR.lat    = Parameter("NFZ_LAT")
PAR.lng    = Parameter("NFZ_LNG")
PAR.rad    = Parameter("NFZ_RAD")
PAR.margin = Parameter("NFZ_MARGIN")
PAR.base   = Parameter("NFZ_BASE")

local D = {
    c_id = {}, c_lat = {}, c_lng = {}, c_r = {}, c_n = 0,
    p_id = {}, p_off = {}, p_nv = {}, p_n = 0,
    vtx_lng = {}, vtx_lat = {}, vtx_n = 0,
    ov_id = {}, ov_lat = {}, ov_lng = {}, ov_r = {}, ov_n = 0,
    del_id = {}, del_n = 0,
    want_id = {}, want_n = 0,
}

local ST = {
    cache_lat = nil,
    cache_lng = nil,
    auth_id = nil,
    last_warn_ms = 0,
    was_inside = false,
    loiter_held = false,
    dynamic_loaded = false,
    arm_blocked = false, -- 区内拦解锁；仅进出切换，避免 PreArm 狂刷
}

local seed = {
    i = 1,
    part = 1,
    src = nil,
    dst = nil,
    done = false,
    list = {
        { sd = CFG.INDEX,   rom = CFG.ROMFS .. "index.csv" },
        { sd = CFG.CIRCLES, rom = CFG.ROMFS .. "circles.csv" },
        {
            sd = CFG.POLYS,
            parts = {
                CFG.ROMFS .. "polys.part1.csv",
                CFG.ROMFS .. "polys.part2.csv",
                CFG.ROMFS .. "polys.part3.csv",
                CFG.ROMFS .. "polys.part4.csv",
            },
            expect = CFG.POLYS_EXPECTED,
        },
        { sd = CFG.OVERLAY, rom = CFG.ROMFS .. "overlay.csv" },
        { sd = CFG.DELETED, rom = CFG.ROMFS .. "deleted.csv" },
    },
}

seed.file_size = function(path)
    local f = io.open(path, "rb")
    if f == nil then
        return nil
    end
    local sz = f:seek("end")
    f:close()
    return sz
end

seed.sd_ready = function(item)
    local sz = seed.file_size(item.sd)
    if sz == nil then
        return false
    end
    if item.expect ~= nil and sz < item.expect then
        remove(item.sd)
        return false
    end
    return true
end

seed.open_next_src = function(item)
    if item.parts ~= nil then
        while seed.part <= #item.parts do
            local src = io.open(item.parts[seed.part], "rb")
            seed.part = seed.part + 1
            if src ~= nil then
                return src
            end
        end
        return nil
    end
    return io.open(item.rom, "rb")
end

seed.close_dst = function()
    if seed.dst ~= nil then
        seed.dst:close()
        seed.dst = nil
    end
end

seed.step = function()
    if seed.done then
        return
    end
    for _ = 1, CFG.SEED_CHUNKS do
        if seed.src == nil then
            if seed.dst ~= nil then
                local item = seed.list[seed.i]
                local src = seed.open_next_src(item)
                if src == nil then
                    seed.close_dst()
                    seed.i = seed.i + 1
                    seed.part = 1
                else
                    seed.src = src
                end
            else
                while seed.i <= #seed.list do
                    local item = seed.list[seed.i]
                    if seed.sd_ready(item) then
                        seed.i = seed.i + 1
                        seed.part = 1
                    else
                        seed.part = 1
                        local src = seed.open_next_src(item)
                        if src == nil then
                            seed.i = seed.i + 1
                            seed.part = 1
                        else
                            local dst = io.open(item.sd, "wb")
                            if dst == nil then
                                src:close()
                                seed.i = seed.i + 1
                                seed.part = 1
                            else
                                seed.src = src
                                seed.dst = dst
                                break
                            end
                        end
                    end
                end
                if seed.src == nil then
                    seed.done = true
                    return
                end
            end
        end

        if seed.src ~= nil then
            local block = seed.src:read(CFG.SEED_CHUNK)
            if block ~= nil and #block > 0 then
                seed.dst:write(block)
            else
                seed.src:close()
                seed.src = nil
                local item = seed.list[seed.i]
                if item.parts ~= nil and seed.part <= #item.parts then
                    -- next part
                else
                    seed.close_dst()
                    seed.i = seed.i + 1
                    seed.part = 1
                end
            end
        end
    end
end

local rf = {
    phase = "idle",
    f = nil,
    clat = 0,
    clng = 0,
    win_m = 0,
    busy = false,
}

local F = {}

--------------------------------------------------------------------
F.enabled = function()
    return (PAR.enable:get() or 0) >= 1
end

F.dist_m = function(lat1, lng1, lat2, lng2)
    local mid = (lat1 + lat2) * 0.5
    local dlat = (lat1 - lat2) * 111320.0
    local dlng = (lng1 - lng2) * 111320.0 * math.cos(mid * 0.017453292519943295)
    return math.sqrt(dlat * dlat + dlng * dlng)
end

F.is_deleted = function(id)
    for i = 1, D.del_n do
        if D.del_id[i] == id then
            return true
        end
    end
    return false
end

F.overlay_index = function(id)
    for i = 1, D.ov_n do
        if D.ov_id[i] == id then
            return i
        end
    end
    return nil
end

F.remove_deleted = function(id)
    for i = 1, D.del_n do
        if D.del_id[i] == id then
            D.del_id[i] = D.del_id[D.del_n]
            D.del_id[D.del_n] = nil
            D.del_n = D.del_n - 1
            return
        end
    end
end

F.add_deleted = function(id)
    if F.is_deleted(id) then
        return true
    end
    if D.del_n >= CFG.MAX_DELETED then
        return false
    end
    D.del_n = D.del_n + 1
    D.del_id[D.del_n] = id
    return true
end

-- 射线法点在多边形内（平面 lon/lat，窗口内足够）
F.point_in_poly = function(lat, lng, off, nv)
    local inside = false
    local j = off + nv - 1
    for i = off, off + nv - 1 do
        local yi = D.vtx_lat[i]
        local xi = D.vtx_lng[i]
        local yj = D.vtx_lat[j]
        local xj = D.vtx_lng[j]
        if ((yi > lat) ~= (yj > lat)) then
            local xint = (xj - xi) * (lat - yi) / ((yj - yi) + 1e-18) + xi
            if lng < xint then
                inside = not inside
            end
        end
        j = i
    end
    return inside
end

--------------------------------------------------------------------
F.write_overlay_file = function()
    local f = io.open(CFG.OVERLAY, "w")
    if f == nil then
        return false
    end
    f:write("id,kind,lat,lng,r\n")
    for i = 1, D.ov_n do
        f:write(string.format("%d,0,%.7f,%.7f,%d\n", D.ov_id[i], D.ov_lat[i], D.ov_lng[i], math.floor(D.ov_r[i] + 0.5)))
    end
    f:close()
    return true
end

F.write_deleted_file = function()
    local f = io.open(CFG.DELETED, "w")
    if f == nil then
        return false
    end
    f:write("id\n")
    for i = 1, D.del_n do
        f:write(string.format("%d\n", D.del_id[i]))
    end
    f:close()
    return true
end

F.load_overlay = function()
    D.ov_n = 0
    local f = io.open(CFG.OVERLAY, "r")
    if f == nil then
        return
    end
    while true do
        local line = f:read("*l")
        if line == nil then
            break
        end
        local id, kind, lat, lng, r = line:match("^(%-?%d+),(%d+),(%-?%d+%.?%d*),(%-?%d+%.?%d*),(%-?%d+%.?%d*)")
        if id ~= nil and D.ov_n < CFG.MAX_OVERLAY then
            -- 动态层目前只加载圆
            if tonumber(kind) == 0 then
                D.ov_n = D.ov_n + 1
                D.ov_id[D.ov_n] = tonumber(id)
                D.ov_lat[D.ov_n] = tonumber(lat)
                D.ov_lng[D.ov_n] = tonumber(lng)
                D.ov_r[D.ov_n] = tonumber(r)
            end
        end
    end
    f:close()
end

F.load_deleted = function()
    D.del_n = 0
    local f = io.open(CFG.DELETED, "r")
    if f == nil then
        return
    end
    while true do
        local line = f:read("*l")
        if line == nil then
            break
        end
        if line ~= "" and line:byte(1) ~= 105 then
            local id = tonumber(line)
            if id ~= nil and D.del_n < CFG.MAX_DELETED then
                D.del_n = D.del_n + 1
                D.del_id[D.del_n] = id
            end
        end
    end
    f:close()
end

F.clear_active = function()
    D.c_n = 0
    D.p_n = 0
    D.vtx_n = 0
end

F.want_has = function(id)
    for i = 1, D.want_n do
        if D.want_id[i] == id then
            return true
        end
    end
    return false
end

F.rf_close = function()
    if rf.f ~= nil then
        rf.f:close()
        rf.f = nil
    end
end

F.rf_finish = function()
    F.rf_close()
    rf.phase = "idle"
    rf.busy = false
    ST.cache_lat, ST.cache_lng = rf.clat, rf.clng
end

F.rf_start = function(clat, clng)
    F.rf_close()
    F.clear_active()
    D.want_n = 0
    rf.clat = clat
    rf.clng = clng
    local win_m = (PAR.radius:get() or 80) * 1000.0
    if win_m < 5000 then
        win_m = 5000
    end
    rf.win_m = win_m
    rf.busy = true

    -- 仅 overlay：不扫全国库
    if (PAR.base:get() or 1) < 1 then
        ST.cache_lat, ST.cache_lng = clat, clng
        rf.busy = false
        rf.phase = "idle"
        return
    end

    rf.f = io.open(CFG.INDEX, "r")
    if rf.f == nil then
        gcs:send_text(3, CFG.SCRIPT .. ": 无 " .. CFG.INDEX .. "（仅 overlay 仍可用）")
        ST.cache_lat, ST.cache_lng = clat, clng
        rf.busy = false
        rf.phase = "idle"
        return
    end
    rf.phase = "index"
end

-- 每帧只读若干行，避免 time limit
F.rf_step = function()
    if not rf.busy then
        return
    end

    local n = 0
    while n < CFG.LINES_PER_SLICE do
        n = n + 1
        if rf.phase == "index" then
            local line = rf.f:read("*l")
            if line == nil then
                F.rf_close()
                rf.f = io.open(CFG.CIRCLES, "r")
                rf.phase = "circles"
            else
                local id, kind, lat, lng, r = line:match("^(%-?%d+),(%d+),(%-?%d+%.?%d*),(%-?%d+%.?%d*),(%-?%d+%.?%d*)")
                if id ~= nil then
                    id = tonumber(id)
                    lat = tonumber(lat)
                    lng = tonumber(lng)
                    r = tonumber(r)
                    if (not F.is_deleted(id)) and F.overlay_index(id) == nil then
                        if F.dist_m(rf.clat, rf.clng, lat, lng) <= (rf.win_m + r) then
                            if D.want_n < CFG.MAX_WANT then
                                D.want_n = D.want_n + 1
                                D.want_id[D.want_n] = id
                            end
                        end
                    end
                end
            end

        elseif rf.phase == "circles" then
            if rf.f == nil then
                rf.f = io.open(CFG.POLYS, "r")
                rf.phase = "polys"
            else
                local line = rf.f:read("*l")
                if line == nil then
                    F.rf_close()
                    rf.f = io.open(CFG.POLYS, "r")
                    rf.phase = "polys"
                else
                    local id, lat, lng, r = line:match("^(%-?%d+),(%-?%d+%.?%d*),(%-?%d+%.?%d*),(%-?%d+%.?%d*)")
                    if id ~= nil then
                        id = tonumber(id)
                        if F.want_has(id) and D.c_n < CFG.MAX_CIRCLES then
                            D.c_n = D.c_n + 1
                            D.c_id[D.c_n] = id
                            D.c_lat[D.c_n] = tonumber(lat)
                            D.c_lng[D.c_n] = tonumber(lng)
                            D.c_r[D.c_n] = tonumber(r)
                        end
                    end
                end
            end

        elseif rf.phase == "polys" then
            if rf.f == nil then
                F.rf_finish()
                return
            end
            local line = rf.f:read("*l")
            if line == nil then
                F.rf_finish()
                return
            end
            -- 先快速看 id，不在 want 则跳过重解析
            local id_s = line:match("^(%-?%d+),")
            if id_s ~= nil then
                local id = tonumber(id_s)
                if F.want_has(id) and D.p_n < CFG.MAX_POLYS then
                    local n = tonumber(line:match("^%-?%d+,(%d+),"))
                    if n ~= nil and n >= 3 and D.vtx_n + n <= CFG.MAX_VERTS then
                        local rest = line:match("^%-?%d+,%d+,(.*)$")
                        if rest ~= nil then
                            local off = D.vtx_n + 1
                            local got = 0
                            for num in rest:gmatch("%-?%d+%.?%d*") do
                                got = got + 1
                                if got > n * 2 then
                                    break
                                end
                                if (got % 2) == 1 then
                                    D.vtx_n = D.vtx_n + 1
                                    D.vtx_lng[D.vtx_n] = tonumber(num)
                                else
                                    D.vtx_lat[D.vtx_n] = tonumber(num)
                                end
                            end
                            if D.vtx_n >= off + n - 1 then
                                D.vtx_n = off + n - 1
                                D.p_n = D.p_n + 1
                                D.p_id[D.p_n] = id
                                D.p_off[D.p_n] = off
                                D.p_nv[D.p_n] = n
                            else
                                D.vtx_n = off - 1
                            end
                        end
                    end
                end
            end
        else
            F.rf_finish()
            return
        end
    end
end

F.maybe_refresh = function(clat, clng)
    if rf.busy then
        return
    end
    local win_m = (PAR.radius:get() or 80) * 1000.0
    local thresh = win_m * 0.35
    if ST.cache_lat == nil or F.dist_m(ST.cache_lat, ST.cache_lng, clat, clng) >= thresh then
        F.rf_start(clat, clng)
    end
end

--------------------------------------------------------------------
-- CRUD（动态层：圆）
--------------------------------------------------------------------
F.upsert_zone = function(id, lat, lng, r)
    id = math.floor(id)
    if id == 0 or r == nil or r <= 0 then
        return false, "bad args"
    end
    F.remove_deleted(id)
    local idx = F.overlay_index(id)
    if idx ~= nil then
        D.ov_lat[idx], D.ov_lng[idx], D.ov_r[idx] = lat, lng, r
    else
        if D.ov_n >= CFG.MAX_OVERLAY then
            return false, "overlay full"
        end
        D.ov_n = D.ov_n + 1
        D.ov_id[D.ov_n], D.ov_lat[D.ov_n], D.ov_lng[D.ov_n], D.ov_r[D.ov_n] = id, lat, lng, r
    end
    F.write_overlay_file()
    F.write_deleted_file()
    ST.cache_lat = nil
    return true, "ok"
end

F.delete_zone = function(id)
    id = math.floor(id)
    if id == 0 then
        return false, "bad id"
    end
    local idx = F.overlay_index(id)
    if idx ~= nil then
        D.ov_id[idx] = D.ov_id[D.ov_n]
        D.ov_lat[idx] = D.ov_lat[D.ov_n]
        D.ov_lng[idx] = D.ov_lng[D.ov_n]
        D.ov_r[idx] = D.ov_r[D.ov_n]
        D.ov_id[D.ov_n], D.ov_lat[D.ov_n], D.ov_lng[D.ov_n], D.ov_r[D.ov_n] = nil, nil, nil, nil
        D.ov_n = D.ov_n - 1
    end
    if not F.add_deleted(id) then
        return false, "deleted full"
    end
    F.write_overlay_file()
    F.write_deleted_file()
    ST.cache_lat = nil
    return true, "ok"
end

F.clear_dynamic = function()
    D.ov_n = 0
    D.del_n = 0
    F.write_overlay_file()
    F.write_deleted_file()
    ST.cache_lat = nil
    return true, "ok"
end

F.handle_cmd = function(op, id, lat, lng, r)
    op = math.floor(op or 0)
    if op == 1 then
        return F.upsert_zone(id, lat, lng, r)
    elseif op == 2 then
        return F.delete_zone(id)
    elseif op == 3 then
        return F.clear_dynamic()
    elseif op == 4 then
        ST.cache_lat = nil
        F.rf_close()
        rf.busy = false
        rf.phase = "idle"
        local loc = ahrs:get_location()
        if loc ~= nil then
            F.rf_start(loc:lat() * 1.0e-7, loc:lng() * 1.0e-7)
        end
        return true, "reloaded"
    elseif op == 5 then
        gcs:send_text(6, string.format("%s: circ=%d poly=%d vtx=%d ov=%d del=%d",
            CFG.SCRIPT, D.c_n, D.p_n, D.vtx_n, D.ov_n, D.del_n))
        return true, "status"
    end
    return false, "bad op"
end

F.poll_param_cmd = function()
    local op = PAR.cmd:get() or 0
    if op < 1 then
        return
    end
    local ok, msg = F.handle_cmd(op, PAR.id:get() or 0, PAR.lat:get() or 0, PAR.lng:get() or 0, PAR.rad:get() or 0)
    PAR.cmd:set(0)
    if ok then
        gcs:send_text(6, CFG.SCRIPT .. ": CMD ok " .. tostring(msg))
    else
        gcs:send_text(3, CFG.SCRIPT .. ": CMD fail " .. tostring(msg))
    end
end

--------------------------------------------------------------------
F.check_inside = function(lat, lng)
    for i = 1, D.ov_n do
        if F.dist_m(lat, lng, D.ov_lat[i], D.ov_lng[i]) <= D.ov_r[i] then
            return D.ov_id[i]
        end
    end
    for i = 1, D.c_n do
        if F.dist_m(lat, lng, D.c_lat[i], D.c_lng[i]) <= D.c_r[i] then
            return D.c_id[i]
        end
    end
    for i = 1, D.p_n do
        if F.point_in_poly(lat, lng, D.p_off[i], D.p_nv[i]) then
            return D.p_id[i]
        end
    end
    return nil
end

-- 水平地速 m/s
F.ground_speed = function()
    local vel = ahrs:get_velocity_NED()
    if vel == nil then
        return 0
    end
    return math.sqrt(vel:x() * vel:x() + vel:y() * vel:y())
end

-- 朝禁飞区接近速度（圆：指向圆心；>0 表示正在靠近）
F.closing_speed_to = function(lat, lng, tlat, tlng)
    local vel = ahrs:get_velocity_NED()
    if vel == nil then
        return 0
    end
    local dN = (tlat - lat) * 111320.0
    local dE = (tlng - lng) * 111320.0 * math.cos(lat * 0.017453292519943295)
    local dist = math.sqrt(dN * dN + dE * dE)
    if dist < 0.1 then
        return 0
    end
    return (vel:x() * dN + vel:y() * dE) / dist
end

-- 沿当前速度方向外推 ahead_m（悬停则原位）
F.ahead_latlng = function(lat, lng, ahead_m)
    local vel = ahrs:get_velocity_NED()
    if vel == nil or ahead_m <= 0 then
        return lat, lng
    end
    local vn, ve = vel:x(), vel:y()
    local spd = math.sqrt(vn * vn + ve * ve)
    if spd < 0.4 then
        return lat, lng
    end
    local scale = ahead_m / spd
    local coslat = math.cos(lat * 0.017453292519943295)
    if coslat < 0.2 then
        coslat = 0.2
    end
    return lat + (vn * scale) / 111320.0, lng + (ve * scale) / (111320.0 * coslat)
end

-- Loiter 刹车距离（与围栏脚本同思路）
F.braking_distance = function(v_close)
    if v_close <= 0 then
        return 0
    end
    local brk_accel = param:get("LOIT_BRK_ACCEL") or 250
    local brk_delay = param:get("LOIT_BRK_DELAY") or 1.0
    local accel = math.max(brk_accel, 50) * 0.01
    return v_close * (brk_delay + CFG.LOOP_MS * 0.001) + (v_close * v_close) / (2 * accel)
end

-- 到圆边界外侧距离（>0 在外，<=0 已进入）
F.circle_clearance = function(lat, lng, clat, clng, r)
    return F.dist_m(lat, lng, clat, clng) - r
end

-- 找最近威胁：已在区内，或沿航迹外推会进入，或距圆边 < 裕量且正在靠近
-- 返回 id, reason ("inside"|"approach")
F.nearest_threat = function(lat, lng)
    local margin = PAR.margin:get() or 20
    if margin < 0 then
        margin = 0
    end
    local gs = F.ground_speed()
    local brk = F.braking_distance(gs)
    local need = margin + brk

    local id_now = F.check_inside(lat, lng)
    if id_now ~= nil then
        return id_now, "inside"
    end

    -- 沿速度方向预判：未进入时就把预测点刹在区外
    local alat, alng = F.ahead_latlng(lat, lng, need)
    if alat ~= lat or alng ~= lng then
        local id_ahead = F.check_inside(alat, alng)
        if id_ahead ~= nil then
            return id_ahead, "approach"
        end
    end

    -- 圆形额外：距边界 clearance，正在靠近则提前刹
    local function consider_circle(id, clat, clng, r)
        local clr = F.circle_clearance(lat, lng, clat, clng, r)
        if clr <= need then
            local v_in = F.closing_speed_to(lat, lng, clat, clng)
            if v_in > 0.4 then
                return id
            end
        end
        return nil
    end

    for i = 1, D.ov_n do
        local id = consider_circle(D.ov_id[i], D.ov_lat[i], D.ov_lng[i], D.ov_r[i])
        if id ~= nil then
            return id, "approach"
        end
    end
    for i = 1, D.c_n do
        local id = consider_circle(D.c_id[i], D.c_lat[i], D.c_lng[i], D.c_r[i])
        if id ~= nil then
            return id, "approach"
        end
    end

    return nil, nil
end

F.try_loiter = function(id, reason)
    if not arming:is_armed() then
        return
    end
    if ST.loiter_held and vehicle:get_mode() == CFG.MODE_LOITER then
        return
    end
    if vehicle:set_mode(CFG.MODE_LOITER) then
        ST.loiter_held = true
    else
        gcs:send_text(3, CFG.SCRIPT .. ": 切 Loiter 失败(需定位)")
    end
end

F.try_rtl = function()
    if not arming:is_armed() then
        return
    end
    if vehicle:get_mode() == CFG.MODE_RTL then
        return
    end
    vehicle:set_mode(CFG.MODE_RTL)
end

--------------------------------------------------------------------
ST.auth_id = arming:get_aux_auth_id()
if ST.auth_id == nil then
    gcs:send_text(3, CFG.SCRIPT .. ": 鉴权槽失败")
end

function update()
    if not seed.done then
        seed.step()
        return update, CFG.LOOP_MS
    end
    if not ST.dynamic_loaded then
        F.load_deleted()
        F.load_overlay()
        ST.dynamic_loaded = true
    end

    F.poll_param_cmd()

    if not F.enabled() then
        if ST.auth_id ~= nil then
            arming:set_aux_auth_passed(ST.auth_id)
            ST.arm_blocked = false
        end
        ST.was_inside = false
        ST.loiter_held = false
        return update, CFG.LOOP_MS
    end

    if not arming:is_armed() then
        ST.loiter_held = false
    elseif ST.loiter_held and vehicle:get_mode() ~= CFG.MODE_LOITER then
        -- 飞手已切走，认为接管
        ST.loiter_held = false
    end

    local loc = ahrs:get_location()
    if loc == nil then
        -- 短暂无定位时保持原鉴权，避免 failed↔passed 抖动导致 PreArm 数秒一刷
        return update, CFG.LOOP_MS
    end

    local lat = loc:lat() * 1.0e-7
    local lng = loc:lng() * 1.0e-7
    F.maybe_refresh(lat, lng)
    F.rf_step()

    local id_inside = F.check_inside(lat, lng)
    local threat_id, reason = F.nearest_threat(lat, lng)
    local now = millis()
    local action = PAR.action:get() or 2

    -- 解锁：仅「当前已在区内」才拦；鉴权只在状态变化时写一次
    local want_block = (id_inside ~= nil) and ((PAR.armblk:get() or 0) >= 1)
    if ST.auth_id ~= nil then
        if want_block and not ST.arm_blocked then
            arming:set_aux_auth_failed(ST.auth_id, "禁飞区内禁止解锁")
            ST.arm_blocked = true
            ST.last_warn_ms = now
        elseif (not want_block) and ST.arm_blocked then
            arming:set_aux_auth_passed(ST.auth_id)
            ST.arm_blocked = false
        elseif not want_block and not ST.arm_blocked then
            arming:set_aux_auth_passed(ST.auth_id)
        end
    end

    if threat_id ~= nil then
        if arming:is_armed() then
            -- 在飞：靠近/进入都提示返航
            if not ST.was_inside or (now - ST.last_warn_ms) >= CFG.WARN_MS then
                gcs:send_text(2, "禁止进入禁飞区，飞机即将返航")
                ST.last_warn_ms = now
            end
            local act = math.floor(action + 0.5)
            if act == 1 then
                F.try_loiter(threat_id, reason)
            elseif act >= 2 then
                F.try_rtl()
            end
        end
        -- 地面区内：只靠 PreArm 鉴权文案（约 30s 一次），不再额外 gcs 刷屏

        ST.was_inside = (reason == "inside") or (reason == "approach" and arming:is_armed())
    else
        ST.was_inside = false
        if ST.loiter_held and vehicle:get_mode() == CFG.MODE_LOITER then
            -- 保持悬停
        else
            ST.loiter_held = false
        end
    end

    return update, CFG.LOOP_MS
end

return update, 2000
