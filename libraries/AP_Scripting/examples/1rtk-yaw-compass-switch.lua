-- RTK 固定解与磁罗盘偏航源自动切换（直接改 EK3_SRC1_YAW）
--
-- 行为说明（与 ArduPilot AP_NavEKF_Source::SourceYaw 枚举一致）：
--   未进入 RTK 固定解（或无有效 GPS 双天线偏航）→ EK3_SRC1_YAW = 1（COMPASS，纯罗盘）
--   进入 RTK 固定解且 gps:gps_yaw_deg 有效            → EK3_SRC1_YAW = 3（GPS_COMPASS_FALLBACK，优先 GPS/RTK 测向，失效时回退罗盘）
--
-- 注意：值为 3 时并非“关闭罗盘”，而是 EKF 定义的“GPS 测向 + 罗盘回退”组合模式。
--
-- ──────────────────────────────────────────────
-- 使用前飞控端建议配置（位置/速度仍用 GPS 等，仅偏航由本脚本切换）
-- ──────────────────────────────────────────────
--   SCR_ENABLE = 1
--   EK3_SRC1_POSXY = 3   (GPS)
--   EK3_SRC1_VELXY = 3   (GPS)
--   EK3_SRC1_VELZ  = 3 或 0（按需求）
--   EK3_SRC1_POSZ  = 1   (Baro)
--   EK3_SRC1_YAW   = 1 或 3（脚本运行后会自动改写；上电可先设 1）
--
-- 双天线 RTK 模块需向飞控提供有效 GPS yaw（与原先 SRC 组切换方案相同）。
-- ──────────────────────────────────────────────

---@diagnostic disable: need-check-nil

-- 脚本主循环周期（毫秒）
local UPDATE_RATE_MS = 500

-- MAV_SEVERITY 等级常量（用于 gcs:send_text）
local MAV_SEVERITY = {
    EMERGENCY = 0, ALERT = 1, CRITICAL = 2, ERROR = 3,
    WARNING = 4, NOTICE = 5, INFO = 6, DEBUG = 7
}

-- EKF3 偏航源参数取值（与 libraries/AP_NavEKF/AP_NavEKF_Source.h 中 SourceYaw 一致）
local YAW_COMPASS = 1                -- COMPASS：仅用磁罗盘
local YAW_GPS_COMPASS_FALLBACK = 3   -- GPS_COMPASS_FALLBACK：GPS/RTK 测向优先，异常时回退罗盘

-- 防抖：连续 VOTE_MAX 个周期后才切换，避免在固定解边界抖动
-- UPDATE_RATE_MS=500、VOTE_MAX=4 → 约 2 s 防抖窗口
local VOTE_MAX = 4

-- 当前已写入飞控的 EK3_SRC1_YAW（-1 表示尚未由本脚本改写）
local current_yaw_src = -1
local rtk_vote = 0
local init_done = false

--[[
    初始化：确认能读取 EK3_SRC1_YAW，并提示当前值
--]]
local function do_init()
    local y = param:get("EK3_SRC1_YAW")
    if y == nil then
        gcs:send_text(MAV_SEVERITY.ERROR,
            "RTK-YAW: read EK3_SRC1_YAW err")
        return false
    end

    gcs:send_text(MAV_SEVERITY.INFO,
        string.format("RTK-YAW: Init EK3_SRC1_YAW=%.0f",
            y))
    return true
end

--[[
    将 EK3_SRC1_YAW 设为 new_yaw（1 或 3）。
    使用 param:set 而非 set_and_save，避免频繁切换磨损闪存；需要掉电保存请手动写入参数文件或地面站保存一次。
--]]
local function set_ek3_src1_yaw(new_yaw)
    if new_yaw == current_yaw_src then
        return
    end

    if not param:set("EK3_SRC1_YAW", new_yaw) then
        gcs:send_text(MAV_SEVERITY.ERROR,
            string.format("RTK-YAW: set EK3_SRC1_YAW=%d err", new_yaw))
        return
    end

    current_yaw_src = new_yaw

    if new_yaw == YAW_GPS_COMPASS_FALLBACK then
        gcs:send_text(MAV_SEVERITY.NOTICE,
            "RTK-YAW: RTK fixed -> YAW=3")
        notify:play_tune("L12EE")
    else
        gcs:send_text(MAV_SEVERITY.NOTICE,
            "RTK-YAW: No RTK -> YAW=1")
        notify:play_tune("L8C")
    end
end

--[[
    主循环：根据 RTK 固定解与 gps_yaw 有效性投票，再切换 EK3_SRC1_YAW
--]]
local function update()
    if not init_done then
        if do_init() then
            init_done = true
        else
            return update, 5000
        end
    end

    local gps_inst = gps:primary_sensor()
    local fix_type = gps:status(gps_inst)

    -- 仅 RTK FIXED（枚举值 6）视为“进入固定解”
    local is_rtk_fixed = (fix_type == gps.GPS_OK_FIX_3D_RTK_FIXED)

    local has_gps_yaw = false
    if is_rtk_fixed then
        local yaw_valid, yaw_deg, yaw_acc, yaw_ms = gps:gps_yaw_deg(gps_inst)
        has_gps_yaw = (yaw_valid == true)
        if yaw_valid and yaw_deg ~= nil then
            gcs:send_named_float("RTK_YAW", yaw_deg)
        end
    end

    -- 投票：满足固定解且偏航有效 → 倾向 3；否则倾向 1
    if is_rtk_fixed and has_gps_yaw then
        rtk_vote = math.min(rtk_vote + 1, VOTE_MAX)
    else
        rtk_vote = math.max(rtk_vote - 1, -VOTE_MAX)
    end

    if rtk_vote >= VOTE_MAX then
        set_ek3_src1_yaw(YAW_GPS_COMPASS_FALLBACK)
    elseif rtk_vote <= -VOTE_MAX then
        set_ek3_src1_yaw(YAW_COMPASS)
    end

    gcs:send_named_float("RTK_FIX", is_rtk_fixed and 1 or 0)
    gcs:send_named_float("EK3YAW", current_yaw_src >= 0 and current_yaw_src or 0)
    gcs:send_named_float("RTK_VOTE", rtk_vote)

    return update, UPDATE_RATE_MS
end

gcs:send_text(MAV_SEVERITY.INFO, "RTK-YAW loaded")
return update, UPDATE_RATE_MS
