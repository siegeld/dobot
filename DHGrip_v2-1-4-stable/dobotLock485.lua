local lockerName = "dobot_485_lock"

function Lock485(time, waitTime)
    local result = Lock(lockerName, time, waitTime)
    Wait(10)
    return result
end

function UnLock485()
    UnLock(lockerName)
end
