local LanguageMap = {}
local LanguageTool = {}

function LanguageTool.getI18nError(key)
    if GetLanguage ~= nil then
        local languag = GetLanguage()
        if string.find(languag, "zh") ~= nil then
            return LanguageMap["zh"][key]
        else
            return LanguageMap["en"][key]
        end
    else
        return LanguageMap["en"][key]
    end
end

LanguageMap["zh"] = {
    optionalParamsError = "无效的高级参数，仅支持table类型",
    idError = "无效的id，仅支持数字类型",
    isBlockError = "无效的isBlock，仅支持布尔类型",
    initOverTimeError = '夹爪初始化超时',
    initDataMaxTip = '最多支持同时控制4个夹爪',
    idErrorTip = "id范围为：1-255，仅支持整数",
    not485ToolMode = "当前末端不是485模式，无法获取夹爪数据！",
    gripperNotInit = '夹爪未初始化',
    sendFailed = '初始化失败，数据写入失败',
    readFailed = '初始化失败，数据读取失败',
    positionErrorTip = "宽度范围为: 0-100，仅支持整数",
    forceErrorTip = "力度范围为: 20-100，仅支持整数",
    speedErrorTip = "速度范围为: 1-100，仅支持整数"
}

LanguageMap["en"] = {
    optionalParamsError = "Invalid advanced parameter. Only the table type is supported",
    idError = "Invalid id. Only numeric types are supported",
    isBlockError = "Invalid isBlock, only Boolean types are supported",
    initOverTimeError = 'Gripper initialization timeout',
    initDataMaxTip = 'Supports simultaneous control of up to 4 grippers.',
    idErrorTip = "The ID range is: 1-255, only integers are supported",
    not485ToolMode = "The current end is not 485 mode, can not get the claw data!",
    gripperNotInit = 'Gripper not initialized',
    sendFailed = 'The init process failed, and the write data operation failed',
    readFailed = 'The init process failed, failed to read data',
    positionErrorTip = "Width range: 0-100, only integers are supported",
    forceErrorTip = "Force range: 20-100, only integers are supported",
    speedErrorTip = "Speed ​​range: 1-100, only integers are supported"
}

return LanguageTool
