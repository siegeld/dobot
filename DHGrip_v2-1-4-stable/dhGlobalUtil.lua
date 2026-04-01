local GlobalUtil = {}
local gripperToolBaudDataName = "DH_TOOL_BAUD_Data"
function GlobalUtil.readGripperFailHandle()
    local res = {
        isErr = -1,
        ip = -1,
        port = -1,
        slaveID = -1,
        initStatus = -1,
        posSet = -1,
        force = -1,
        speed = -1,
        gripStatus = -1,
        posRel = -1,
        errStatus = -1
    }
    return res
end

function GlobalUtil.CheckScoket(config)
    local ip = config["ip"]
    local port = config["port"]
    local slaveID = config["slave_id"]
    local err, modbus = ModbusCreate(ip, port, slaveID, true)
    if err ~= 0 then
        EcoLog("error to init modbus ! ", ip, port, slaveID)
        return -1
    end

    config["modbus_id"] = modbus
    return config
end

function GlobalUtil.getGripperData(data)
    --设置波特率
    GlobalUtil.SetTool485()
    ---连接Modbus
    local config = GlobalUtil.CheckScoket(data)
    -- 连接失败
    if config == -1 then
        local res = GlobalUtil.readGripperFailHandle()
        return res
    else
        local tryTime = 3
        local res
        -- 连接成功
        local modbus = config["modbus_id"]
        --失败读取三次
        while tryTime > 0 do
            -- 3. 读取状态寄存器的区域
            local ret = GetHoldRegs(modbus, 0x0200, 3)
            local ret2 = GetHoldRegs(modbus, 0x0100, 5)
            local errBuf = GetHoldRegs(modbus, 0x0205, 1)
            if ret == nil or #ret ~= 3 or ret2 == nil or #ret2 ~= 5 then
                res = GlobalUtil.readGripperFailHandle()
                EcoLog('GlobalUtil:dh read gripper data fail, slaveID:' .. data.slave_id)
            else
                local errStatus
                if type(errBuf) == "table" and #errBuf == 1 then
                    errStatus = errBuf[1]
                end
                res = {
                    isErr = 0,
                    ip = "127.0.0.1",
                    port = 60000,
                    slaveID = config.slave_id,
                    initStatus = ret[1],
                    posSet = ret2[4] / 10.0,
                    force = ret2[2],
                    speed = ret2[5],
                    gripStatus = ret[2],
                    posRel = ret[3] / 10,
                    errStatus = errStatus
                }
                break;
            end
            tryTime = tryTime - 1
        end
        ModbusClose(modbus)
        return res
    end
end

function GlobalUtil.saveGripperToolBaudData(baud, parity, stopBits)
    SetVal(gripperToolBaudDataName, {baud = baud, parity = parity, stopBits=stopBits})
end
function GlobalUtil.SetTool485()
    local toolBaudData = GetVal(gripperToolBaudDataName)
    if toolBaudData == nil then
        toolBaudData = {
            baud = 115200,
            parity = 'N',
            stopBits= 1
        }
    end
    SetTool485(toolBaudData.baud, toolBaudData.parity, toolBaudData.stopBits)
end

return GlobalUtil