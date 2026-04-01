require("dobotLock485")
local Plugin = {}
local MqttRobot = require("MqttRobot")
local GlobalUtil = require("dhGlobalUtil")
local dhInitStatusKeyName = "DH_IS_INIT"
local dhGrippersStatusKeyName = "DH_GRIPPERS_STATUS"
local dhDefaultConfig = "ECO_DH_DEFAULT_CONFIG"
local dhToolConfig = "ECO_DH_TOOL_CONFIG"
local dhInitConfigs = 'ECO_DH_INIT_CONFIGS'
local dhCurrentConfig = 'ECO_DH_CURRENT_CONFIG'

function Plugin.DHGripModbusClose(strTopic, strMsg)
	if strTopic == "/DHGrip/ModbusClose" then
		ModbusClose(tonumber(strMsg))
	end
end

function Plugin.runMqtt()
	local topicArray={"/DHGrip/ModbusClose"}
	MqttRobot.subscribe(topicArray,Plugin.DHGripModbusClose)
end

-- 插件的卸载函数,一般可以用于释放数据库变量,释放硬件资源
function Plugin.OnUninstall()
	EcoLog("-------Plugin.OnUninstall---- DH ----")
	SetVal(dhInitStatusKeyName, nil)
	SetVal(dhGrippersStatusKeyName, nil)
	SetVal(dhDefaultConfig, {})
	SetVal(dhToolConfig, {})
	SetVal(dhCurrentConfig, nil)
	local initConfigs = GetVal(dhInitConfigs)
	if initConfigs ~= nil then
		for key, value in pairs(initConfigs) do
			if value.modbus_id ~= nil then
				ModbusClose(value.modbus_id)
				initConfigs[key].modbus_id = nil
			end
		end
	end
	SetVal(dhInitConfigs, initConfigs)
end

-- 插件的安装函数,主要是用于 数据库变量的初始值设置, DH的场景中暂时未用到!
function Plugin.OnInstall()
	EcoLog("-------Plugin.OnInstall---- DH ----")
	systhread.create(Plugin.runMqtt)
	SetVal(dhCurrentConfig, {
		ip = "127.0.0.1",
		port = 60000,
		slave_id = 1
	})
	GlobalUtil.saveGripperToolBaudData(115200, 'N', 1)
end
-- 快捷键创建Modbus用
function Plugin.getModbusConfig()
	local currentConfigData = GetVal(dhCurrentConfig)
	if currentConfigData == nil then return -1 end
	local err, modbus = ModbusCreate("127.0.0.1", 60000, currentConfigData.slave_id, true)
	if err ~= 0 then
		EcoLog("创建Modbus连接失败---getModbusConfig")
		Plugin.delInitDataBySlaveId(currentConfigData.slave_id)
		return -1
	end
	return modbus
end

-- 处理初始化夹爪的数据，将已初始化夹爪存入数据库中
function Plugin.handleInitConfigs(config)
	local name = "gripper"..config.slave_id
	local initConfigData = GetVal(dhInitConfigs)
	if initConfigData == nil then
		initConfigData = {
			[name] = config
		}
	else
		 initConfigData[name] = config
	end
	SetVal(dhInitConfigs, initConfigData)
end

-- 初始化失败时清除数据库中该夹爪
function Plugin.delInitDataBySlaveId(slaveID)
	local name = "gripper"..slaveID
  local initConfigData = GetVal(dhInitConfigs)
	local statusData = GetVal(dhGrippersStatusKeyName)
	if initConfigData == nil or initConfigData[name] == nil then return end
	if initConfigData[name].modbus_id ~= nil then
		ModbusClose(initConfigData[name].modbus_id)
	end
	initConfigData[name] = nil
	SetVal(dhInitConfigs, initConfigData)
	if statusData== nil or statusData[name] == nil then return end
	statusData[name] = nil
	SetVal(dhGrippersStatusKeyName, statusData)
end
-- 通过slaveID在已初始化的table中查找配置
function Plugin.getInitedConnection(slaveID)
	local name = "gripper"..slaveID
  local initConfigData = GetVal(dhInitConfigs)
	if initConfigData == nil then return nil end
  return initConfigData[name]
end
-- modbusId检查
function Plugin.modbusIdErr(slaveID, modbus)
	local initConfigData = GetVal(dhInitConfigs)
	local name = "gripper"..slaveID
	local flag = false
	if initConfigData ~= nil then
		for key, value in pairs(initConfigData) do
			if key ~= name and value.modbus_id == modbus then
				ModbusClose(modbus)
				flag = true
			end
		end
		if flag then
			for key, value in pairs(initConfigData) do
					ModbusClose(modbus)
					initConfigData[key].modbus_id = nil
			end
			SetVal(dhInitConfigs, initConfigData)
		end
	end
	return flag
end
-- 大寰的初始化接口,设计中应该允许夹爪连接到除了末端以外的其他外设上!
function Plugin.DHReInit(ip, port, slaveID)
	-- 判断是否在已初始化夹爪的配置中
	local config = Plugin.getInitedConnection(slaveID)
	-- 已存在并且有modbus_id，则断开连接，删除该条数据
	if nil ~= config and nil ~= config.modbus_id then
		local tempModbus = config.modbus_id
		EcoLog('tempModbus', tempModbus)
		Plugin.delInitDataBySlaveId(slaveID)
		config = {}
	end
	SetVal(dhInitStatusKeyName, false)
	-- 创建连接
	local err, modbus = ModbusCreate(ip, port, slaveID, true)
	if err ~= 0 then
		EcoLog("HTTP error to init modbus ! ", ip, port, slaveID)
		Plugin.delInitDataBySlaveId(slaveID)
		return -1
	end
	SetVal(dhInitStatusKeyName, false)
	Lock485(10000,3000)
	GlobalUtil.SetTool485()
	SetHoldRegs(modbus, 0x0100, 1, {0xA5})
	local waitMax = 80
	local start_time = Systime()
	repeat
		-- 这里 通过读取 0x2000 位置的寄存器来获得夹爪的初始化结果
		local ret = GetHoldRegs(modbus, 0x0200, 1)
		-- 如果五十次读不到，也就是五秒，则退出
		if (ret == nil or #ret ~= 1) and waitMax == 30 then
			Plugin.delInitDataBySlaveId(slaveID)
			EcoLog("HTTP 读取夹爪modbus失败")
			UnLock485()
			ModbusClose(modbus)
			return -1
		end
		if ret and #ret == 1 and ret[1] == 1 then
			break;
		end
		Wait(100)
		waitMax = waitMax - 1
		--循环时间大于10s 自动退出
		if Systime() - start_time > 10000 then
			EcoLog("over time")
			waitMax = 0
			break
		end
	until waitMax == 0 or (ret and #ret == 1 and ret[1] == 1) -- 检查初始化成功,最多 5s 时间,检测内容参考手册
	if waitMax == 0 then -- 初始化超时
		EcoLog("DH init over time !")
		Plugin.delInitDataBySlaveId(slaveID)
		ModbusClose(modbus)
		UnLock485()
		return -1
	end
	--4. 初始化成功,将默认的连接信息保存到数据库中
	local config = {}
	config["ip"] = ip
	config["port"] = port
	config["slave_id"] = slaveID
	config["modbus_id"] = modbus
	SetVal(dhInitStatusKeyName, true)
	UnLock485()
	-- 将该ID夹爪的数据存入数据库中
  Plugin.handleInitConfigs(config)
	return 0
end

-- 末端 485 的设置接口
function Plugin.DhSetTool485(baud, parity, stopBits)
	--这里没有实际设置波特率，知识将波特率存下来了
	GlobalUtil.saveGripperToolBaudData(baud, parity, stopBits)
	return 0
end

-- 大寰的夹爪控制指令
function Plugin.DhMov(position, force, speed, slaveID)
	--在已初始化的table中查找slaveID的配置
	local config = Plugin.getInitedConnection(slaveID)
	-- 没有说明未初始化
  if config == nil then return -1 end
	Lock485(3000, 3000)
	GlobalUtil.SetTool485()
	local modbus = config["modbus_id"]
	local ip = config["ip"]
	local port = config["port"]
	local slaveID = config["slave_id"]
	local connectResult
	if modbus ~= nil then
		connectResult = GetHoldRegs(modbus, 0x0100, 1)
	end
	if modbus == nil or Plugin.modbusIdErr(slaveID, modbus) == true or (modbus ~= nil and (not (connectResult and #connectResult == 1))) then
		if modbus ~= nil then ModbusClose(modbus) end
		local err, tempModbus = ModbusCreate(ip, port, slaveID, true)
		if err ~= 0 then
			EcoLog("error to init modbus ! ", ip, port, slaveID)
			return -1	
		end
		modbus = tempModbus
		config["modbus_id"] = tempModbus
		Plugin.handleInitConfigs(config)
	end
	
	-- 4. 检查力的参数是否存在 如果存在合理值那么就写入
	if force and force ~= -1 then
		if force < 20 then
			force = 20
		end
		if force > 100 then
			force = 100
		end
		SetHoldRegs(modbus, 0x0101, 1, {force}) -- 寄存器位置 从数据手册获得
		EcoLog("force ", force)
	end
	-- 5. 检查速度值是否合理,如果存在合理的值就写入
	if speed and speed ~= -1 then
		if speed < 1 then
			speed = 1
		end
		if speed > 100 then
			speed = 100
		end
		SetHoldRegs(modbus, 0x0104, 1, {speed}) -- 寄存器位置 从数据手册获得
		EcoLog("speed ", speed)
	end
	-- 6. 检查目标位置值是否合理,合理就写入
	if position and position ~= -1 and position >= 0 and position <= 100 then
		position = position * 10 -- 参数范围是百分比  下发到夹爪为千分比
		SetHoldRegs(modbus, 0x0103, 1, {position}) -- 寄存器位置 从数据手册获得
		EcoLog("position ", position)
	end
	EcoLog("done !!!!")
	Plugin.handleInitConfigs(config)
	UnLock485()
	return 0
end

-- 大寰的状态获取指令
function Plugin.DhGetStatus()
	local slaveID = 1
	if GetVal(dhCurrentConfig) ~= nil then
		slaveID = GetVal(dhCurrentConfig).slave_id
	end
	local name = "gripper"..slaveID
	local data = GetVal(dhGrippersStatusKeyName)
	if data == nil then return nil end
	return data[name]
end

-- 设置插件页面当前夹爪id
function Plugin.setDhCurrentConfig(slaveID)
	local data = GetVal(dhGrippersStatusKeyName)
	local name = "gripper"..slaveID
	SetVal(dhCurrentConfig, {
		ip = "127.0.0.1",
		port = 60000,
		slave_id = slaveID
	})
	if data == nil then	return end
	return data[name]
end

-- 获取插件页面当前夹爪id
function Plugin.getDhCurrentConfig(slaveID)
	if GetVal(dhCurrentConfig) == nil then return 1 end
	return GetVal(dhCurrentConfig).slave_id
end

function Plugin.getDhInitDataLength()
	local initConfigData = GetVal(dhInitConfigs)
	local initConfigDataLength = 0
	--计算初始化table长度
	if initConfigData ~= nil and type(initConfigData) == "table" then
		for _, value in pairs(initConfigData) do
			initConfigDataLength = initConfigDataLength + 1
		end
	end
	return initConfigDataLength
end

-- 快捷键
function Plugin.DHGripper_Open()
	EcoLog("----------------------START-----------------------")
	if Plugin.getInitedConnection(GetVal(dhCurrentConfig).slave_id) == nil then
		EcoLog("请先初始化连接")
		return -1
	end
	local force = 100
	local speed = 100
	local position = 100
	-- 1. 从数据库获得之前已经保存好的参数
	local modbus = Plugin.getModbusConfig()
	if modbus == -1 then 
		EcoLog('创建modbus失败')
		EcoLog("---------------------END-----------------------")
		return -1
	end
	Lock485(2000,2000)
	GlobalUtil.SetTool485()
	SetHoldRegs(modbus, 0x0104, 1, {speed})
	SetHoldRegs(modbus, 0x0101, 1, {force})
	position = position * 10 -- 参数范围是百分比  下发到夹爪为千分比
	SetHoldRegs(modbus, 0x0103, 1, {position})
	EcoLog("done !!!!")
	ModbusClose(modbus)
	UnLock485()
	Wait(100)
	EcoLog("---------------------END-----------------------")
	return 0
end
-- 快捷键
function Plugin.DHGripper_Close()
	EcoLog("----------------------START-----------------------")
	if Plugin.getInitedConnection(GetVal(dhCurrentConfig).slave_id) == nil then
		EcoLog("请先初始化连接")
		return -1
	end
	local force = 100
	local speed = 100
	local position = 0
	-- 1. 从数据库获得之前已经保存好的参数
	local modbus = Plugin.getModbusConfig()
	if modbus == -1 then 
		EcoLog('创建modbus失败')
		EcoLog("---------------------END-----------------------")
		return -1
	end
	Lock485(2000,2000)
	GlobalUtil.SetTool485()
	SetHoldRegs(modbus, 0x0104, 1, {speed})
	SetHoldRegs(modbus, 0x0101, 1, {force})
	position = position * 10 -- 参数范围是百分比  下发到夹爪为千分比
	SetHoldRegs(modbus, 0x0103, 1, {position})
	ModbusClose(modbus)
	EcoLog("done !!!!")
	UnLock485()
	Wait(100)
	return 0
end
-- 快捷键
function Plugin.DHGripperInit()
	EcoLog("----------------------START-----------------------")
	--SetTool485(115200, "N", 1)
	Wait(100)
	SetVal(dhInitStatusKeyName, false)
	local currentConfigData = GetVal(dhCurrentConfig)
	local modbus = Plugin.getModbusConfig()
	if modbus == -1 then 
		EcoLog('创建modbus失败')
		EcoLog("---------------------END-----------------------")
		return -1
	end
	Lock485(10000, 3000)
	GlobalUtil.SetTool485()
	SetHoldRegs(modbus, 0x0100, 1, {0xA5})
	local waitMax = 50
	local start_time = Systime()
	repeat
		local ret = GetHoldRegs(modbus, 0x0200, 1)
		-- 如果无法连接modbus那么直接退出初始化
		if ret == nil or #ret ~= 1 then
			EcoLog("读取夹爪modbus失败")
			ModbusClose(modbus)
			UnLock485()
			EcoLog("---------------------END-----------------------")
			return -1
		end
		Wait(100)
		waitMax = waitMax - 1
		if Systime() - start_time > 10000 then
			EcoLog("over time")
			waitMax = 0
			break
		end
		UnLock485()
	until waitMax == 0 or (ret and #ret == 1 and ret[1] == 1)
	if waitMax == 0 then
		EcoLog("DH init over time !")
		ModbusClose(modbus)
		UnLock485()
		EcoLog("---------------------END-----------------------")
		return -1
	end
	local config = {
		ip = currentConfigData.ip,
		port = currentConfigData.port,
		slave_id = currentConfigData.slave_id
	}
	Plugin.handleInitConfigs(config)
	ModbusClose(modbus)
	SetVal(dhInitStatusKeyName, true)
	UnLock485()
	EcoLog("---------------------END-----------------------")
	return 0
end

function Plugin.OnRegistHotKey()
	return {press = {"DHGripper_Open", "DHGripper_Close"}, longPress = {"DHGripperInit"}}
end

return Plugin
