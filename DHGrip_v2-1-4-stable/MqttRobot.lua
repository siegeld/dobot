-- Mqtt的封装，以及部分业务的封装
require("libplugin_eco")
local jsonLuaUtils = require("luaJson")

local MqttRobotProtocol={
    ip = "127.0.0.1",
    port = 1883,
    qos = 0,
    retain = false,
    connectId = nil
}
MqttRobotProtocol.__index = MqttRobotProtocol

function MqttRobotProtocol:new()
    return setmetatable({},self)
end

function MqttRobotProtocol:setAddress(ip, port)
    self.ip = ip
    self.port = port
end

function MqttRobotProtocol:setQOS(qos)
    self.qos = qos
end

--[[
功能：连接mqtt
参数：connectId-连接的id字符串
返回值：true成功，false失败
]]--
function MqttRobotProtocol:connect(connectId,strLastWillTopic,strLastWillMsg)
    local function pfnExecute(self, connectId)
        MQTTCreate(connectId, self.ip, self.port, 600)
        MQTTSetLastWill(connectId,strLastWillTopic,strLastWillMsg,self.qos,self.retain)
        local ret = MQTTConnect(connectId)
        EcoLog("MQTTConnect:"..tostring(ret))
        self.connectId = connectId
    end
    local isOk,err = pcall(pfnExecute, self, connectId)
    if not isOk then
        EcoLog("MQTTConnect fail:"..tostring(err))
    end
    if self.connectId then return true
    else return false
    end
end

--[[
功能：断开mqtt
参数：无
返回值：无
]]--
function MqttRobotProtocol:disconnect()
    if self.connectId then
        local isOk,err = pcall(MQTTDisconnect, self.connectId)
        if not isOk then
            EcoLog("MQTTDisconnect fail:"..tostring(err))
        end
        self.connectId = nil
    end
end

--[[
功能：是否已经连接mqtt
参数：无
返回值：true已连接，false未连接
]]--
function MqttRobotProtocol:isConnected()
    if self.connectId then return true
    else return false
    end
end

--[[
功能：发布消息
参数：strTopic-主题
      strMsg-消息
返回值：true成功，false失败
]]--
function MqttRobotProtocol:publish(strTopic, strMsg)
    if not self:isConnected() then return false end
    local isOk,err = pcall(MQTTPublish,self.connectId,strTopic,strMsg,self.qos,self.retain)
    if not isOk then
        self:disconnect() --发布失败了则断开连接
        EcoLog("MQTTPublish fail:"..tostring(err))
        return false
    end
    return true
end

--[[
功能：订阅消息
参数：strTopic-主题
返回值：成功返回“订阅消息”字符串，失败返回nil
说明：此函数非阻塞
]]--
function MqttRobotProtocol:subscribe(strTopic)
    if not self:isConnected() then return nil end
    local isOk,err = pcall(MQTTSubscribe,self.connectId,strTopic,self.qos)
    if not isOk then
        self:disconnect() --订阅失败了则断开连接
        EcoLog("MQTTSubscribe fail:"..tostring(err))
        return nil
    end
    return err
end

--[[
功能：订阅消息
参数：strTopicArray-订阅的主题，可以多个同时订阅，strTopicArray={"topic1","topic2",...}
      callbackName-回调函数名称：该回调函数一定要是全局函数，形如function cbFun(topic,msg) end
返回值：成功则阻塞，发生失败则立刻返回false
说明：此函数可能阻塞
]]--
function MqttRobotProtocol:subscribeWait(strTopicArray,callbackName)
    if not self:isConnected() or type(strTopicArray)~="table" then return false end
    for i=1,#strTopicArray do
        if not self:subscribe(strTopicArray[i]) then
            return false
        end
    end
    local isOk,err = pcall(MQTTWait,self.connectId,callbackName)
    if not isOk then
        self:disconnect()
        EcoLog("MQTTWait fail:"..tostring(err))
        return false
    end
    return err
end

--****************************************************************************************************************************************
--**以上是对mqtt的封装，与业务无关，尽量不要修改******************************************************************************************
--****************************************************************************************************************************************
--****************************************************************************************************************************************
--****************************************************************************************************************************************
local function innerExecutePublish(obj,strTopic, msg)
    if type(msg)=="table" then
        msg = jsonLuaUtils.encode(msg)
    elseif type(msg)=="string" then
    else
        msg = tostring(msg)
    end
    if nil==obj.mqtt then
        obj.mqtt = MqttRobotProtocol:new()
    end
    if not obj.mqtt:isConnected() then
        obj.mqtt:connect(obj.connectId, obj.lastWillTopic, obj.lastWillMsg)
    end
    return obj.mqtt:publish(strTopic,msg)
end

local function innerExecuteSubscribe(obj,strTopicArray, callbackName)
    if nil==obj.mqtt then
        obj.mqtt = MqttRobotProtocol:new()
    end
    if not obj.mqtt:isConnected() then
        obj.mqtt:connect(obj.connectId, obj.lastWillTopic, obj.lastWillMsg)
    end
    return obj.mqtt:subscribeWait(strTopicArray,callbackName)
end

local MqttRobot = {
    mqtt = nil,
    connectId = "DHGrip",
    lastWillTopic = "DHGripPluginCrash",
    lastWillMsg = "DHGrip process has exit or crash"
}

--此函数一定是全局唯一的
function __pfnExecuteHttpDHGripSubscribe(strTopic, strMsg)
    if type(MqttRobot.pfnCBSunscribe)=="function" then
        local _ok,msg = pcall(MqttRobot.pfnCBSunscribe,strTopic, strMsg)
        if not _ok then EcoLog(msg) end
    end
end

--[[
功能：通用mqtt发布接口
参数：strTopic-主题
      msg-消息,可以是table,string,或其他，内部最终都会转为字符串发出。
返回值：true成功，false失败
]]--
function MqttRobot.publish(strTopic, msg)
    local isOk,errmsg = pcall(innerExecutePublish,MqttRobot,strTopic, msg)
    if not isOk then
        EcoLog("publish fail:"..tostring(errmsg))
    end
    return isOk
end


--[[
功能：通用mqtt订阅接口
参数：strTopicArray-主题数组，可以同时订阅多个主题
      pfnCallback-回调函数，形如 function (topic,msg) end
返回值：true成功，false失败
说明：此接口通常是阻塞的，只需要调用一次。
]]--
function MqttRobot.subscribe(strTopicArray, pfnCallback)
    MqttRobot.pfnCBSunscribe = pfnCallback
    local isOk,errmsg = pcall(innerExecuteSubscribe,MqttRobot,strTopicArray, "__pfnExecuteHttpDHGripSubscribe")
    if not isOk then
        EcoLog("subscribe fail:"..tostring(errmsg))
    end
    return isOk
end

return MqttRobot