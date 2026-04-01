require("dobotLock485")
local GlobalUtil = require("dhGlobalUtil")
local Plugin = {}
local LanguageTool = require("dhLanguage")
local dhErrStatusKeyName = "DH_ERR_STATUS"
local dhInitStatusKeyName = "DH_IS_INIT"
local dhInitConfigs = 'ECO_DH_INIT_CONFIGS'
local dhGrippersStatusKeyName = "DH_GRIPPERS_STATUS"
-- internal use ,
function Plugin.NewConnection(id, connector)
	if nil == id or type(id) ~= "number" then id = 1 end
	if nil == connector or type(connector) ~= "number" then connector = 1 end
	if connector == 1 then
		GlobalUtil.SetTool485() -- 单末端
	else
		error('ERROR: connector should be 1 or nil')
		return -1
	end
	local initConfigData = GetVal(dhInitConfigs)
	local initConfigDataLength = 0
	--计算初始化table长度
	if initConfigData ~= nil and type(initConfigData) == "table" then
		for _, value in pairs(initConfigData) do
			initConfigDataLength = initConfigDataLength + 1
		end
	end
	if initConfigDataLength == 4 then
		Plugin.PluginError()
	end
	-- 创建新的连接 并且把连接放到连接池里面去
	local name = string.format("%d_%d",connector,  id)
	local ip = "127.0.0.1"
	local port = 60000
	local conn = Plugin[name]
	if Plugin[name] == nil then
		local err, modbus = ModbusCreate(ip, port, id, true)
		if err == -1 then
			Plugin[name] = nil
			Plugin.handleInitConfigs(nil)
			return
		end
		conn = modbus
	end
	local config = {}
	config["ip"] = ip
	config["port"] = port
	config["slave_id"] = id
	Plugin.handleInitConfigs(config)
	Plugin[name] = conn
	return conn
end

function Plugin.handleInitConfigs(config)
	local name = "gripper"..config.slave_id
	local initConfigData = GetVal(dhInitConfigs)
	if initConfigData == nil then
		initConfigData = {
			[name] = config
		}
	else
		if initConfigData[name] ~= nil then
			config.modbus_id = initConfigData[name].modbus_id
		end
    initConfigData[name] = config
	end
	SetVal(dhInitConfigs, initConfigData)
end

function Plugin.getInitedConnection(slaveID)
	local name = "gripper"..slaveID
  local initConfigData = GetVal(dhInitConfigs)
	if initConfigData == nil then return nil end
  return initConfigData[name]
end

-- 初始化失败时清除数据库中该夹爪
function Plugin.initErrorHandle(slaveID)
	local name = "gripper"..slaveID
  local initConfigData = GetVal(dhInitConfigs)
	if initConfigData == nil or initConfigData[name] == nil then return end
	initConfigData[name] = nil
	SetVal(dhInitConfigs, initConfigData)
	local statusData = GetVal(dhGrippersStatusKeyName)
	if statusData== nil or statusData[name] == nil then return end
	statusData[name] = nil
	SetVal(dhGrippersStatusKeyName, statusData)
end
---报错兼容处理，没有EcoScriptError指令则用error
function Plugin.PluginError(msg)
	if EcoScriptError ~= nil then
		EcoScriptError(LanguageTool.getI18nError(msg))
	else
		error(LanguageTool.getI18nError(msg))
	end
end

function Plugin.CheckType(optionalParams)
	if optionalParams ~= nil and type(optionalParams) ~= "table" then
		Plugin.PluginError('optionalParamsError')
		return false
	end
	if optionalParams ~= nil and type(optionalParams) == "table" then
		if optionalParams["id"] ~= nil and type(optionalParams["id"]) ~= "number" then
			Plugin.PluginError('idError')
			return false
		end
		if optionalParams["isBlock"] ~= nil and type(optionalParams["isBlock"]) ~= "boolean" then
			Plugin.PluginError('isBlockError')
			return false
		end
		if optionalParams["connection"] ~= nil and type(optionalParams["connection"]) ~= "number" then
			error('ERROR: Invalid connection：only support number')
			return false
		end
	end
	return true
end

---末端连接485模式检测
function Plugin.PluginCheckToolMode()
	if GetToolMode ~= nil then
			local mode, type = GetToolMode()
			if mode ~= nil and mode == 1 then
					return true
			end
			Plugin.PluginError('not485ToolMode')
	else
			return true
	end
end


function Plugin.RepeatSendHoldRegs(modbus, address, dataLen, sendData)
	local tryTime = 3
	local res
	-- 发送失败时，重试3次
	while tryTime > 0 do
		res = SetHoldRegs(modbus, address, dataLen, sendData)
		dobotTool.EcoLog("SetHoldRegs"..address.."res:"..res)
		if res ~= 0 then
			dobotTool.EcoLog("SetHoldRegs"..address.."error")
			-- 发送失败统一将数据赋值为-1
			res = -1
			tryTime = tryTime - 1
		else
			break;
		end
	end
	return res
end

function Plugin.RepeatGetHoldRegs(modbus, address, dataLen)
	local tryTime = 3
	local res
	-- 读取的数据不对时，重试3次
	while tryTime > 0 do
		res = GetHoldRegs(modbus, address, dataLen)
		if res == nil or type(res) ~= "table" or #res ~= dataLen then
			dobotTool.EcoLog("GetHoldRegs"..address.."error")
			-- 读取失败统一将数据赋值为nil
			res = nil
			tryTime = tryTime - 1
		else
			break;
		end
	end
	return res
end

-- ○ id：设备ID号，默认id为1
-- ○ connector：航插序号，取值1或2；可选参数，不填默认为1
-- ○ 下面的所有指令均含有上述两个参数，定义相同
function Plugin.DHGripInit(optionalParams) --初始化夹爪		
	Plugin.PluginCheckToolMode()
	local id = 1
	local connection = 1
	if Plugin.CheckType(optionalParams) then
		if optionalParams["id"] ~= nil and type(optionalParams["id"]) == "number" then
			id = optionalParams["id"]
		end
		if optionalParams["connection"] ~= nil and type(optionalParams["connection"]) == "number" then
			connection = optionalParams["connection"]
		end
	else
		return
	end
	if id < 1 or id > 255 or id ~= math.floor(id)  then
		Plugin.PluginError('idErrorTip')
	end
	local lockStatus = Lock485(10000, 5000)
	if lockStatus == false then 
		dobotTool.EcoLog("DHGripInit Lock485 failed")
		return -1 
	end
	local modbus = Plugin.NewConnection(id, connector)
	if nil == modbus or modbus == -1 then
		UnLock485()
		Plugin.initErrorHandle(id)
		return -1
	end
	local addr = 0x0100                  -- 初始化写的寄存器地址
	local writeTab = { 0xA5 }            -- 初始化写入的值
	local isInitDone = false             -- 是否初始化成功标志
	local isOverTime = false             -- 是否初始化超时标志
	local timeOutMax_ms = 10000          -- 最大初始化超时时间
	local checkGap = 1000                -- 初始化成功的检查间隔
	local beginTime = Systime()          -- 初始化开始的时间
	local initStatusRegisterAddr = 0x0200 -- 初始化结果的查询寄存器

	SetHoldRegs(modbus, addr, #writeTab, writeTab)
	repeat
		local ret2 = Plugin.RepeatGetHoldRegs(modbus, initStatusRegisterAddr, 1)
		if ret2 == nil then
			Plugin.initErrorHandle(id)
			UnLock485()
			dobotTool.EcoLog("USERAPI DHGripInit read data Failed")
			Plugin.PluginError('readFailed')
		end
		if (ret2 ~= nil and #ret2 == 1 and ret2[1] == 1) then
			isInitDone = true
		end
		if false == isInitDone and Systime() - beginTime > timeOutMax_ms then
			isOverTime = true
		end
		Wait(checkGap)
	until true == isOverTime or true == isInitDone
	UnLock485()
	if isInitDone == true then
		SetVal(dhInitStatusKeyName, 1)
		return 0
	end
	Plugin.PluginError('initOverTimeError')
	Plugin.initErrorHandle(id)
	return -1
end

-- ○ 参数1：夹爪目标宽度设置
-- ○ 参数2：夹爪目标力设置
-- ○ 参数3：夹爪目标速度设置
-- ○ 参数4：可选参数
---- id: 夹爪ID，默认值为1
---- isBlock: 是否阻塞后续程序运行，默认为阻塞
---- connector：航插序号，可选参数，不填默认为1

-- 返回值 : -1 通讯类的异常 1: 到位 2:抓住 3: 掉落
function Plugin.DHGripControl(width, force, speed, optionalParams) --控制夹爪运动
	Plugin.PluginCheckToolMode()
	local id = 1
	local isBlock = true
	local connection = 1
	if Plugin.CheckType(optionalParams) then
		if optionalParams["id"] ~= nil and type(optionalParams["id"]) == "number" then
			id = optionalParams["id"]
		end
		if optionalParams["isBlock"] ~= nil and type(optionalParams["isBlock"]) == "boolean" then
			isBlock = optionalParams["isBlock"]
		end
		if optionalParams["connection"] ~= nil and type(optionalParams["connection"]) == "number" then
			connection = optionalParams["connection"]
		end
	else
		return -1
	end
	if id < 1 or id > 255 or id ~= math.floor(id)  then
		Plugin.PluginError('idErrorTip')
	end
	if width < 0 or width > 100 or width ~= math.floor(width) then
		Plugin.PluginError('positionErrorTip')
	end
	if force < 20 or force > 100 or force ~= math.floor(force) then
		Plugin.PluginError('forceErrorTip')
	end
	if speed < 1 or speed > 100 or speed ~= math.floor(speed) then
		Plugin.PluginError('speedErrorTip')
	end
	local config = Plugin.getInitedConnection(id)
	if config == nil then 
		Plugin.PluginError('gripperNotInit')
	end
	local lockStatus = Lock485(10000, 5000)
	if lockStatus == false then 
		dobotTool.EcoLog("DHGripControl Lock485 failed")
		return -1 
	end
	local conn = Plugin.NewConnection(id, connector)
	EcoLog("DHGripControl ", conn)
	if nil == conn then 
		UnLock485()
		return -1 
	end
	if nil == width then 
		UnLock485()
		return -1 
	end -- 至少一个必选参数
	local forceRegisterAddr = 0x0101
	local speedRegisterAddr = 0x0104
	local widthRegisterAddr = 0x0103
	local ret1 = 0
	local ret2 = 0
	local ret3 = 0
	
	if force and force ~= -1 then
		ret1 = Plugin.RepeatSendHoldRegs(conn, forceRegisterAddr, 1, { force }) -- 寄存器位置 从数据手册获得
	end
	if ret1 ~= -1 and speed and speed ~= -1 then
		ret2 = Plugin.RepeatSendHoldRegs(conn, speedRegisterAddr, 1, { speed }) -- 寄存器位置 从数据手册获得
	end
	if ret1 ~= -1 and ret2 ~= -1 and width and width ~= -1 then
		if width >= 0 and width <= 100 then
			-- width = width * 10 -- 参数范围是百分比  下发到夹爪为千分比
			ret3 = Plugin.RepeatSendHoldRegs(conn, widthRegisterAddr, 1, { width * 10 }) -- 寄存器位置 从数据手册获得
		else
			ret3 = -1
		end
	end
	if ret1 == -1 or ret2 == -1 or ret3 == -1 then
		UnLock485()
		dobotTool.EcoLog("USERAPI DHGripControl send data Failed")
		return -1
	end
	--- 开始处理阻塞与否的问题
	-- 阻塞的退出条件
	-- 1) 到位
	-- 2) 抓到物体
	-- 3) 掉落  返回特殊值
	local ret = -1
	if (nil == isBlock) or (isBlock == true or isBlock == 1) then
		-- 0x0201 grip status    ;  0 : running 1:arrive position 2: gripped 3: drop 		
		local registerAddr = 0x0201
		local MaxConnLost = 50 -- try 50 times 5000 ms
		local timeOutMs = 50000
		local timeOutCheckGap = 50
		local timeBegin = Systime()
		repeat
			local buf = Plugin.RepeatGetHoldRegs(conn, registerAddr, 1)
			if buf == nil then
				UnLock485()
				dobotTool.EcoLog("USERAPI DHGripControl send data Failed")
				return -1
			end
			if buf and #buf == 1 and (buf[1] == 1 or buf[1] == 2 or buf[1] == 3) then
				ret = buf[1]
			end
			if ret ~= 1 and ret ~= 2 then
				Wait(timeOutCheckGap)
			end
			if (Systime() - timeBegin > timeOutMs) then -- 超时
				UnLock485()
				return -1
			end
		until ret == 1 or ret == 2
	else 
		ret = ret1 | ret2 | ret3
	end
	UnLock485()
	return ret
end

-- ○ 返回夹爪的当前位置，单位%
function Plugin.DHGripWidthGet(optionalParams) --获取夹爪的当前宽度
	Plugin.PluginCheckToolMode()
	local id = 1
	local connection = 1
	if Plugin.CheckType(optionalParams) then
		if optionalParams["id"] ~= nil and type(optionalParams["id"]) == "number" then
			id = optionalParams["id"]
		end
		if optionalParams["connection"] ~= nil and type(optionalParams["connection"]) == "number" then
			connection = optionalParams["connection"]
		end
	else
		return
	end
	if id < 1 or id > 255 or id ~= math.floor(id)  then
		Plugin.PluginError('idErrorTip')
	end
	local lockStatus = Lock485(2000, 5000)
	if lockStatus == false then 
		dobotTool.EcoLog("DHGripWidthGet Lock485 failed ")
		return -1 
	end
	local data = GlobalUtil.getGripperData(Plugin.getInitedConnection(id))
	UnLock485()
	if data == nil then
		return -1
	end
	return data.posRel
end

-- ○ 0—向指定位置运动中
-- ○ 1—已到达指定位置，但未检测到物体
-- ○ 2—抓取到物体
-- ○ 3—物体掉落
function Plugin.DHGripMoveStatus(optionalParams) --获取夹爪抓取状态
	Plugin.PluginCheckToolMode()
	local id = 1
	local connection = 1
	if Plugin.CheckType(optionalParams) then
		if optionalParams["id"] ~= nil and type(optionalParams["id"]) == "number" then
			id = optionalParams["id"]
		end
		if optionalParams["connection"] ~= nil and type(optionalParams["connection"]) == "number" then
			connection = optionalParams["connection"]
		end
	else
		return
	end
	if id < 1 or id > 255 or id ~= math.floor(id)  then
		Plugin.PluginError('idErrorTip')
	end
	local lockStatus = Lock485(2000, 5000)
	if lockStatus == false then 
		dobotTool.EcoLog("DHGripMoveStatus Lock485 failed ")
		return -1 
	end
	local data = GlobalUtil.getGripperData(Plugin.getInitedConnection(id))
	UnLock485()
	if data == nil then
		return -1
	end
	return data.gripStatus
end

-- ○ 0—初始化未完成
-- ○ 1—初始化已完成
-- ○ 2—初始化中
-- ○ -1—通讯异常
function Plugin.DHGripInitStatus(optionalParams) --获取夹爪初始化状态
	Plugin.PluginCheckToolMode()
	local id = 1
	local connection = 1
	if Plugin.CheckType(optionalParams) then
		if optionalParams["id"] ~= nil and type(optionalParams["id"]) == "number" then
			id = optionalParams["id"]
		end
		if optionalParams["connection"] ~= nil and type(optionalParams["connection"]) == "number" then
			connection = optionalParams["connection"]
		end
	else
		return
	end
	if id < 1 or id > 255 or id ~= math.floor(id)  then
		Plugin.PluginError('idErrorTip')
	end
	local lockStatus = Lock485(2000, 5000)
	if lockStatus == false then 
		dobotTool.EcoLog("DHGripInitStatus Lock485 failed ")
		return -1 
	end
	local data = GlobalUtil.getGripperData(Plugin.getInitedConnection(id))
	UnLock485()
	if data == nil then
		return -1
	end
	if data.initStatus == 0 then return -1 end
	return data.initStatus
end

-- ○ 返回具体的故障码，通过查找  暂时2023-03-15 看手册 没看到 具体的寄存器
function Plugin.DHGripErrStatus(optionalParams) --获取夹爪故障状态
	Plugin.PluginCheckToolMode()
	local id = 1
	local connection = 1
	if Plugin.CheckType(optionalParams) then
		if optionalParams["id"] ~= nil and type(optionalParams["id"]) == "number" then
			id = optionalParams["id"]
		end
		if optionalParams["connection"] ~= nil and type(optionalParams["connection"]) == "number" then
			connection = optionalParams["connection"]
		end
	else
		return
	end
	if id < 1 or id > 255 or id ~= math.floor(id)  then
		Plugin.PluginError('idErrorTip')
	end
	local lockStatus = Lock485(2000, 5000)
	if lockStatus == false then 
		dobotTool.EcoLog("DHGripErrStatus Lock485 failed ")
		return -1 
	end
	local data = GlobalUtil.getGripperData(Plugin.getInitedConnection(id))
	UnLock485()
	if data == nil then
		return -1
	end
	return data.errStatus
end

-- 对用户进行接口的注册,这个函数会在用户脚本启动的阶段被调用~
function Plugin.OnRegist()
	EcoLog(" --- OnRegist ....  --- ")
	-- 0. 接口导出
	local isErr = ExportFunction("DHGripInit", Plugin.DHGripInit) or
			ExportFunction("DHGripControl", Plugin.DHGripControl) or
			ExportFunction("DHGripWidthGet", Plugin.DHGripWidthGet) or
			ExportFunction("DHGripMoveStatus", Plugin.DHGripMoveStatus) or
			ExportFunction("DHGripInitStatus", Plugin.DHGripInitStatus) or
			ExportFunction("DHGripErrStatus", Plugin.DHGripErrStatus)
	-- 1. 错误的处理		
	if isErr then
		EcoLog(" --- ERR to  register ....  --- ", isErr)
		dobotTool.SetError(0)
	end
end

return Plugin
