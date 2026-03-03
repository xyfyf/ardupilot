    -- hello.lua: a simple script to send a message to the GCS
    
    function update()
        gcs:send_text(6, "Hello from my first Lua script!")
        return update, 1000  -- call update() again in 1000ms (1 second)
    end
    
    return update() -- run the update function immediately