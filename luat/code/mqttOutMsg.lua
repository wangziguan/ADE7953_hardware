--- 模块功能：MQTT客户端数据发送处理
-- @author wangziguan
-- @module mqtt.mqttOutMsg
-- @license MIT
-- @copyright wangziguan
-- @release 2018.06.04

module(...,package.seeall)

--加载常用的全局函数至本地
local ssub,schar,smatch,sbyte,slen = string.sub,string.char,string.match,string.byte,string.len

--数据发送的消息队列
local msgQuene = {}

function insertMsg(topic,payload,qos,user)
    table.insert(msgQuene,{t=topic,p=payload,q=qos,user=user})
end

--[[
函数名：cellinfo2hexs
功能  ：cellinfo转换成字符串
参数  ：无
返回值：十六进制字符串hexs	
]]
local function cellinfo2hexs()
    local hexs = ""
    local cellinfo = net.getCellInfo()
    --确保得到cellinfo
    if cellinfo == nil or type(cellinfo) ~= "string" then return nil,"nil cell info" end
    --cellinfo模式为lac.ci.rssi;
    local lac, ci, rssi = smatch(cellinfo, "(%d+)%.(%d+)%.(%d+)%;")
    hexs=hexs..string.format("%04u",tonumber(lac))..string.format("%06u",tonumber(ci))
    return hexs
end

--- Online回调函数
local function pubOnlineCb(result)
    log.info("mqttOutMsg.pubOnlineCb",result)
end

--- 上报Online信息
function pubOnline()
    insertMsg("ADE7953/"..misc.getImei().."/out", string.fromHex("0431"..cellinfo2hexs()), 1, {cb=pubOnlineCb})
end

--- 初始化“MQTT客户端数据发送”
-- @return 无
-- @usage mqttOutMsg.init()
function init()
    pubOnline()
end

--- 去初始化“MQTT客户端数据发送”
-- @return 无
-- @usage mqttOutMsg.unInit()
function unInit()
    sys.timerStopAll()
    while #msgQuene>0 do
        local outMsg = table.remove(msgQuene,1)
        if outMsg.user and outMsg.user.cb then outMsg.user.cb(false,outMsg.user.para) end
    end
end

--- MQTT客户端是否有数据等待发送
-- @return 有数据等待发送返回true，否则返回false
-- @usage mqttOutMsg.waitForSend()
function waitForSend()
    return #msgQuene > 0
end

--- MQTT客户端数据发送处理
-- @param mqttClient，MQTT客户端对象
-- @return 处理成功返回true，处理出错返回false
-- @usage mqttOutMsg.proc(mqttClient)
function proc(mqttClient)
    while #msgQuene>0 do
        local outMsg = table.remove(msgQuene,1)
        local result = mqttClient:publish(outMsg.t,outMsg.p,outMsg.q)
        if outMsg.user and outMsg.user.cb then outMsg.user.cb(result,outMsg.user.para) end
        if not result then return end
    end
    return true
end
