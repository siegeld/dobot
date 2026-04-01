require("dobotLock485")
--EcoLog(" DH daemon called......")
local GlobalUtil = require("dhGlobalUtil")
local DHPlugin = {}
local dhInitStatusKeyName = "DH_IS_INIT"
local dhGrippersStatusKeyName = "DH_GRIPPERS_STATUS"
local dhInitConfigs = "ECO_DH_INIT_CONFIGS"
local dhCurrentConfig = "ECO_DH_CURRENT_CONFIG"
local MqttRobot = require("MqttRobot")
function DHPlugin.readGripperFailHandle()
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

function DHPlugin.CheckScoket(config)
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

function DHPlugin.getGripperData(data)
  ---连接Modbus
  local config = DHPlugin.CheckScoket(data)
  -- 连接失败
  if config == -1 then
    local res = DHPlugin.readGripperFailHandle()
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
        res = DHPlugin.readGripperFailHandle()
        EcoLog('守护进程：dh read gripper data fail, slaveID:' .. data.slave_id)
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

function DHPlugin.freshGrippersStatus()
  local initConfigData = GetVal(dhInitConfigs)
  local grippersStatus = {}
  if initConfigData ~= nil then
    --遍历已初始化的夹爪，读取夹爪数据
    for _, value in pairs(initConfigData) do
      local name = "gripper" .. value.slave_id
      grippersStatus[name] = DHPlugin.getGripperData(value)
      Wait(50)
    end
  end
  if GetVal(dhGrippersStatusKeyName) ~= nil and initConfigData ~= nil then
    --如果两次（每次没读到会再读两次，相当于6次）未读到夹爪数据，则从初始化列表中去除
    for key, value in pairs(GetVal(dhGrippersStatusKeyName)) do
      if value.slaveID == -1 and grippersStatus[key] ~= nil and grippersStatus[key].slaveID == -1 then
        if initConfigData[key].modbus_id ~= nil then
          MqttRobot.publish("/DHGrip/ModbusClose", initConfigData[key].modbus_id)
        end
        EcoLog("DH： 6次未获取到夹爪信息，从初始化列表中删除")
        initConfigData[key] = nil
        SetVal(dhInitConfigs, initConfigData)
      end
    end
  end
  --获取设置的ID
  local currentConfigData = GetVal(dhCurrentConfig)
  local currentSlaveID
  if currentConfigData == nil then
    currentSlaveID = 1
  else
    currentSlaveID = currentConfigData.slave_id
  end
  --将当前插件中配置ID的夹爪数据通过MQTT发送到插件页面
  MqttRobot.publish("/DH/status", grippersStatus["gripper" .. currentSlaveID])
  --EcoLog(grippersStatus, initConfigData)
  --将读到夹爪的数据存入数据库中
  SetVal(dhGrippersStatusKeyName, grippersStatus)
end

function handleDHModule()
  while true do
    local initConfigData = GetVal(dhInitConfigs)
    local initConfigDataLength = 0
    --计算初始化table长度
    if initConfigData ~= nil and type(initConfigData) == "table" then
      for _, value in pairs(initConfigData) do
        initConfigDataLength = initConfigDataLength + 1
      end
    end
    -- 长度不为0时才获取夹爪状态
    if initConfigDataLength ~= 0 then
      local lockStatus = Lock485(3000, 200)
      if lockStatus then
        GlobalUtil.SetTool485()
        DHPlugin.freshGrippersStatus()
        UnLock485()
      end
    end
    Wait(1000)
  end
end

local thread = {}
thread[1] = systhread.create(handleDHModule, 1)
thread[1]:wait()
