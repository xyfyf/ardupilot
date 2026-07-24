-- 烧录后首次开机，根据 SN_PROD 产品型号一次性写入 E616 / X6100 差异参数
-- SCR_USER2：机型差异参数已写入；SCR_USER3：电池电压已写入（各自只写一次）
-- 重刷固件且参数清零后会再执行一次

local RUN_INTERVAL_MS = 1000
local DONE_FLAG_PARAM = "SCR_USER2"
local DONE_FLAG_VALUE = 6166100
local VOLT_DONE_PARAM = "SCR_USER3"
local VOLT_DONE_VALUE = 6166101
local SN_PROD_WAIT_LOOPS = 30  -- 最多等待 30 秒读取 SN_PROD

-- 电池电压：首次开机按机型写入一次，之后不再改动
local params_voltage = {
    BATT_LOW_VOLT = {44.4, 22.2},
    BATT_CRT_VOLT = {43.2, 21.6},
}

-- 其他机型差异参数
local params_diff = {
    GPS2_MB_OFS_Y    = {-0.7, -0.32},
    GPS1_POS_X       = {0.3, 0.11},
    GPS1_POS_Y       = {0.17, 0},
    GPS1_POS_Z       = {-0.1, -0.04},
    GPS2_POS_X       = {0, 0.05},
    GPS2_POS_Y       = {-0.35, -0.15},
    GPS2_POS_Z       = {-0.1, -0.04},
    ATC_ACCEL_Y_MAX  = {8950, 6000},
    ATC_ANG_PIT_P    = {4.2, 4.5},
    ATC_ANG_RLL_P    = {4.2, 4.5},
    ATC_RAT_PIT_I    = {0.12, 0.14},
    ATC_RAT_PIT_D    = {0.015, 0.013},
    ATC_RAT_PIT_IMAX = {0.2, 0.5},
    ATC_RAT_RLL_I    = {0.12, 0.14},
    ATC_RAT_RLL_D    = {0.015, 0.013},
    ATC_RAT_RLL_IMAX = {0.2, 0.5},
    ATC_RAT_YAW_P    = {1, 1.3},
    ATC_RAT_YAW_I    = {0.5, 1},
    PSC_VELXY_P      = {1, 2},
    PSC_VELXY_I      = {0.5, 1},
    PSC_VELXY_IMAX   = {500, 1000}
}

local wait_loops = 0

local function get_product_model()
    local name = ""
    for i = 1, 7 do
        local p = param:get("SN_PROD" .. tostring(i))
        if not p or p == 0 then
            break
        end
        local p_int = math.floor(p)
        local b1 = (p_int >> 16) & 0xFF
        local b2 = (p_int >> 8) & 0xFF
        local b3 = p_int & 0xFF

        if b1 == 0 then break end
        name = name .. string.char(b1)
        if b2 == 0 then break end
        name = name .. string.char(b2)
        if b3 == 0 then break end
        name = name .. string.char(b3)
    end
    return name
end

local function is_x6100_model(model_str)
    if string.len(model_str) == 0 then
        return false
    end
    local substring = string.sub(model_str, 1, 8)
    return string.find(substring, "610") ~= nil
end

local function is_flag_set(flag_param, flag_value)
    local flag = param:get(flag_param)
    return flag and math.abs(flag - flag_value) < 0.0001
end

local function apply_param_table(param_table, is_x6100)
    local changed_count = 0
    for k, v in pairs(param_table) do
        local target_val = is_x6100 and v[2] or v[1]
        local current_val = param:get(k)
        if not current_val or math.abs(current_val - target_val) > 0.0001 then
            if param:set_and_save(k, target_val) then
                changed_count = changed_count + 1
            end
        end
    end
    return changed_count
end

local function apply_model_params(model_str)
    local is_x6100 = is_x6100_model(model_str)
    local changed_count = apply_param_table(params_diff, is_x6100)
    local volt_changed = 0

    if not is_flag_set(VOLT_DONE_PARAM, VOLT_DONE_VALUE) then
        volt_changed = apply_param_table(params_voltage, is_x6100)
        param:set_and_save(VOLT_DONE_PARAM, VOLT_DONE_VALUE)
        changed_count = changed_count + volt_changed
    end

    param:set_and_save(DONE_FLAG_PARAM, DONE_FLAG_VALUE)

    local display_name = model_str
    if display_name == "" then display_name = "None" end

    gcs:send_text(6, "Product Model: " .. display_name)
    gcs:send_text(6, "Dynamic Params: First boot saved " .. (is_x6100 and "X6100" or "E616") .. " (" .. tostring(changed_count) .. " updated)")
end

function update()
    if is_flag_set(DONE_FLAG_PARAM, DONE_FLAG_VALUE) then
        return
    end

    wait_loops = wait_loops + 1
    local current_model_str = get_product_model()

    if string.len(current_model_str) == 0 and wait_loops < SN_PROD_WAIT_LOOPS then
        return update, RUN_INTERVAL_MS
    end

    apply_model_params(current_model_str)
end

return update()
