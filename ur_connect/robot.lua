------------
--- Represent the connection to a real robot.
-- @module robot

local socket = require('socket')
local util = require('ur_connect/util')
local template = require('ur_connect/template')

local SCRIPT_PORT = 30002 -- The robot executes scripts sent to this port
local PORT = 9031 -- The port the robot will connect to
local NET_TIMEOUT = 1
local MULT_jointstate = 1000000
local SCRIPT_CONTROL = 'ur_connect/urscript/control.urscript'

local Robot = util.class()

--- Construct a new Robot.
function Robot:_init(handle, ip)
  self.handle = handle
  self.ip = ip
  self.port = SCRIPT_PORT
end

function Robot:run_script(script)
  if script == nil then
    error('Script is empty')
  end

  local client = socket.tcp()
  client:settimeout(NET_TIMEOUT)
  client:connect(self.ip, self.port)

  if client == nil then
    error('Failed to open connection')
    return
  end

  -- Robot scripts won't be evaluated unless they end in a carriage return
  client:send(script .. '\r\n')
  client:close()
end

function ip_same_subnet(ip, ips)
  local targetParts = util.string_split(ip, '.')
  local sameSubnetIps = {}

  for i, otherIp in ipairs(ips) do
    local inSameSubnet = true
    local otherParts = util.string_split(otherIp, '.')

    for j = 1, 3 do
      if otherParts[j] ~= targetParts[j] then
        inSameSubnet = false
      end
    end

    if inSameSubnet then
      table.insert(sameSubnetIps, otherIp)
    end
  end

  return sameSubnetIps
end

function Robot:connect()
  local _, info = socket.dns.toip(socket.dns.gethostname())

  local ips = ip_same_subnet(self.ip, info.ip)
  local ip = ips[1]

  local server = assert(socket.bind(ip, PORT))
  server:settimeout(NET_TIMEOUT)

  local controlScript = template.render(util.read_file(SCRIPT_CONTROL), {
    CONTROL_IP = ip,
    CONTROL_PORT = PORT,
  })
  self:run_script(controlScript)

  local client, err = server:accept()

  if not (client == nil and err == 'timeout') then
    client:settimeout(NET_TIMEOUT)
    self.robotSocket = client
    return
  end

  server:close()

  error('Timed out waiting for robot to connect')
end

function Robot:get_joint_angles()
  local joints = sim.getObjectsInTree(self.handle, sim.object_joint_type)

  return {
    base = sim.getJointPosition(joints[1]),
    shoulder = sim.getJointPosition(joints[2]),
    elbow = sim.getJointPosition(joints[3]),
    wrist1 = sim.getJointPosition(joints[4]),
    wrist2 = sim.getJointPosition(joints[5]),
    wrist3 = sim.getJointPosition(joints[6]),
  }
end

-- Takes a hash table with angles in radians for keys
-- base, shoulder, elbow, wrist1, wrist2, and wrist3
-- and tells the robot to servo its joints to match the specified angles.
-- Defaults to the joint angles of the robot in the V-REP scene if none specified.
function Robot:servo_to(jointAngles)
  if jointAngles == nil then
    jointAngles = self:get_joint_angles()
  end

  sim.setThreadIsFree(true)

  local keepalive = 1
  local values = {
    jointAngles.base,
    jointAngles.shoulder,
    jointAngles.elbow,
    jointAngles.wrist1,
    jointAngles.wrist2,
    jointAngles.wrist3,
    keepalive,
  }

  local data = {}
  for i, v in ipairs(values) do
    for j, byte in ipairs(util.int32_to_bytes(v * MULT_jointstate)) do
      table.insert(data, byte)
    end
  end

  self.robotSocket:send(util.bytes_to_string(data))

  -- Receive 6 int32 values and a 1 byte delimiter (carriage return)
  local rxData, err = self.robotSocket:receive(6 * 4 + 1)
  sim.setThreadIsFree(false)

  if rxData == nil then
    print('Warning: receiving from robot timed out')
    return
  end

  local stateData = util.string_to_bytes(rxData)
  --assert(stateData[6 * 4 + 1] == 13)
  if stateData[6 * 4 + 1] ~= 13 then
    return
  end

  local jointsNow = {
    base = bytes_to_int32(stateData[1], stateData[2], stateData[3], stateData[4]) / MULT_jointstate,
    shoulder = bytes_to_int32(stateData[5], stateData[6], stateData[7], stateData[8]) / MULT_jointstate,
    elbow = bytes_to_int32(stateData[9], stateData[10], stateData[11], stateData[12]) / MULT_jointstate,
    wrist1 = bytes_to_int32(stateData[13], stateData[14], stateData[15], stateData[16]) / MULT_jointstate,
    wrist2 = bytes_to_int32(stateData[17], stateData[18], stateData[19], stateData[20]) / MULT_jointstate,
    wrist3 = bytes_to_int32(stateData[21], stateData[22], stateData[23], stateData[24]) / MULT_jointstate,
  }
end

function Robot:disconnect()
  self.robotSocket:close()
end

return Robot
