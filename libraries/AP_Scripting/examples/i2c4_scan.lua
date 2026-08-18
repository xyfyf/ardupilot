-- I2C bus 1 scan (EFT_EDU I2C4)
local BUS = 1
local address = 0
local found = 0

local i2c_bus = i2c:get_device(BUS, 0)
if not i2c_bus then
    gcs:send_text(0, "I2C bus " .. BUS .. " open failed")
    return
end
i2c_bus:set_retries(1)

function update()
    i2c_bus:set_address(address)
    if i2c_bus:read_registers(0) then
        gcs:send_text(0, string.format("I2C bus %u found 0x%02X", BUS, address))
        found = found + 1
    end

    address = address + 1
    if address >= 127 then
        gcs:send_text(0, string.format("I2C bus %u scan done, %u device(s)", BUS, found))
        return  -- stop
    end
    return update, 50
end

return update()