-- 该脚本用于根据产品型号动态调整E616与X6100的差异参数
-- 会持续监控 SN_PROD (product_model) 参数
-- 若前8位中包含"610"，将强制写入X6100的参数，否则写入E616的参数

local RUN_INTERVAL_MS = 1000

-- 包含E616和X6100有差异的所有参数
-- 格式：[参数名] = {E616默认值, X6100默认值}
local params_diff = {
    BATT_LOW_VOLT    = {44.4, 22.2},
    BATT_CRT_VOLT    = {43.2, 21.6},
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

-- 解析产品型号 (从 SN_PROD1 提取至 SN_PROD7)
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

local last_model_str = nil

function update()
    local current_model_str = get_product_model()
    
    -- 当型号发生改变或者每次飞控重新开机时触发覆盖检查
    if current_model_str ~= last_model_str then
        last_model_str = current_model_str
        
        local is_x6100 = false
        
        -- 判断是否写入了产品型号且包含对应的特征
        if string.len(current_model_str) > 0 then
            -- 只在产品型号的前 8 个字符中查找 "610"
            local substring = string.sub(current_model_str, 1, 8)
            if string.find(substring, "610") then
                is_x6100 = true
            end
        end
        
        local changed_count = 0
        
        -- 强制修改并保存参数值，会覆盖用户的当前设置
        for k, v in pairs(params_diff) do
            local target_val = v[1] -- 默认使用E616
            if is_x6100 then
                target_val = v[2]
            end
            
            -- 为防止每次开机都磨损Flash，先读取当前值，如果不一致再执行覆盖并保存
            local current_val = param:get(k)
            if not current_val or math.abs(current_val - target_val) > 0.0001 then
                if param:set_and_save(k, target_val) then
                    changed_count = changed_count + 1
                end
            end
        end
        
        -- 拼接打印信息
        local display_name = current_model_str
        if display_name == "" then display_name = "None" end
        
        gcs:send_text(6, "Product Model: " .. display_name)
        gcs:send_text(6, "Dynamic Params: Force saved " .. (is_x6100 and "X6100" or "E616") .. " (" .. tostring(changed_count) .. " updated)")
    end
    
    return update, RUN_INTERVAL_MS
end

-- 开始运行
return update()