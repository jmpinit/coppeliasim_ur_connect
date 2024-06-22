------------
--- Represent the connection to a real robot.
-- @module robot

local socket = require('socket')
local util = require('ur_connect/util')
local template = require('ur_connect/template')
local server = require('ur_connect.core')

local SCRIPT_PORT = 30002 -- The robot executes scripts sent to this port
local PORT = 9031 -- The port the robot will connect to
local NET_TIMEOUT = 1
local SCRIPT_PATH = 'urscript/control.urscript'

local Robot = util.class()

function Robot:_init(handle, bindIp, robotIp)
  self.handle = handle
  self.bindIp = bindIp
  self.robotIp = robotIp
  self.port = SCRIPT_PORT
  self.lastKnownJointState = {
    base = 0,
    shoulder = 0,
    elbow = 0,
    wrist1 = 0,
    wrist2 = 0,
    wrist3 = 0,
  }
  self.debugMode = true
  self.connected = false
end

function Robot:host_up()
  local client = socket.tcp()
  client:settimeout(NET_TIMEOUT)
  local success = client:connect(self.robotIp, self.port)

  print('Connecting to ' .. self.robotIp .. ':' .. self.port)

  if success == nil then
    return false
  else
    client:close()
    return true
  end
end

function Robot:run_script(script)
  if script == nil then
    error('Script is empty')
  end

  local client = socket.tcp()
  client:settimeout(NET_TIMEOUT)
  local success = client:connect(self.robotIp, self.port)

  if not success then
    error('Failed to open connection')
    return
  end

  -- Robot scripts won't be evaluated unless they end in a carriage return
  client:send(script .. '\r\n')
  client:close()
end

function Robot:connect()
  if not self:host_up() then
    error('Robot not online at specified address')
  else
    print('Host is up!')
  end

  server.start_server(self.bindIp, PORT)

  print('Control server running at tcp://' .. self.bindIp .. ':' .. PORT)

  local luaDir = sim.getStringParam(sim.stringparam_luadir)
  local resourceDir = luaDir:match('^(.+)/[^/]+$')
  local packageDir = resourceDir .. '/luarocks/share/lua/5.3/ur_connect'
  local scriptPath = packageDir .. '/' .. SCRIPT_PATH

  local controlScript = template.render(util.read_file(scriptPath), {
    CONTROL_IP = self.bindIp,
    CONTROL_PORT = PORT,
  })
  print('Instructing the robot to connect to ' .. self.bindIp .. ':' .. PORT)
  self:run_script(controlScript)

  self.connected = true

  local ghostPath = '/' .. sim.getObjectName(self.handle) .. '_ghost'
  local ok, existingGhostHandle = pcall(function () return sim.getObject(ghostPath) end)

  if not ok then
    self.ghostHandle = util.make_ghost_model(self.handle, true)
  else
    self.ghostHandle = existingGhostHandle
  end
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

function Robot:update_ghost()
  if self.ghostHandle == nil or self.lastKnownJointState == nil then
    return
  end

  local ghostJoints = sim.getObjectsInTree(self.ghostHandle, sim.object_joint_type)

  sim.setJointPosition(ghostJoints[1], self.lastKnownJointState.base)
  sim.setJointPosition(ghostJoints[2], self.lastKnownJointState.shoulder)
  sim.setJointPosition(ghostJoints[3], self.lastKnownJointState.elbow)
  sim.setJointPosition(ghostJoints[4], self.lastKnownJointState.wrist1)
  sim.setJointPosition(ghostJoints[5], self.lastKnownJointState.wrist2)
  sim.setJointPosition(ghostJoints[6], self.lastKnownJointState.wrist3)
end

function Robot:freedrive()
  if not self.connected then
    return
  end

  local pose
  if self.lastKnownJointState == nil then
    local jointAngles = self:get_joint_angles()

    -- The CoppeliaSim UR5 model has the shoulder and wrist1 joints rotated 90
    -- degrees relative to the real robot, so we compensate for that here
    pose = {
      jointAngles.base,
      jointAngles.shoulder - math.pi / 2,
      jointAngles.elbow,
      jointAngles.wrist1 - math.pi / 2,
      jointAngles.wrist2,
      jointAngles.wrist3,
    }
  else
    pose = {
      self.lastKnownJointState.base,
      self.lastKnownJointState.shoulder,
      self.lastKnownJointState.elbow,
      self.lastKnownJointState.wrist1,
      self.lastKnownJointState.wrist2,
      self.lastKnownJointState.wrist3,
    }
  end

  server.update_pose(pose, 255)

  local base, shoulder, elbow, wrist1, wrist2, wrist3 = server.get_pose()

  if base ~= nil then
    -- The CoppeliaSim UR5 model has the shoulder and wrist1 joints rotated 90
    -- degrees relative to the real robot, so we compensate for that here
    self.lastKnownJointState.base = base
    self.lastKnownJointState.shoulder = shoulder + math.pi / 2
    self.lastKnownJointState.elbow = elbow
    self.lastKnownJointState.wrist1 = wrist1 + math.pi / 2
    self.lastKnownJointState.wrist2 = wrist2
    self.lastKnownJointState.wrist3 = wrist3

    self:update_ghost()
  end
end

-- Takes a table with angles in radians for keys
-- base, shoulder, elbow, wrist1, wrist2, and wrist3
-- and tells the robot to servo its joints to match the specified angles.
-- Defaults to the joint angles of the robot in the scene if none specified.
function Robot:servo_to(jointAngles, doMove)
  if not self.connected then
    return
  end

  if jointAngles == nil then
    jointAngles = self:get_joint_angles()
  end

  local pose = {
    -- The CoppeliaSim UR5 model has the shoulder and wrist1 joints rotated 90
    -- degrees relative to the real robot, so we compensate for that here
    jointAngles.base,
    jointAngles.shoulder - math.pi / 2,
    jointAngles.elbow,
    jointAngles.wrist1 - math.pi / 2,
    jointAngles.wrist2,
    jointAngles.wrist3,
  }

  local cmd
  if doMove then
    cmd = 1
  else
    cmd = 2
  end

  server.update_pose(pose, cmd)

  local base, shoulder, elbow, wrist1, wrist2, wrist3 = server.get_pose()

  if base ~= nil then
    -- The CoppeliaSim UR5 model has the shoulder and wrist1 joints rotated 90
    -- degrees relative to the real robot, so we compensate for that here
    self.lastKnownJointState.base = base
    self.lastKnownJointState.shoulder = shoulder + math.pi / 2
    self.lastKnownJointState.elbow = elbow
    self.lastKnownJointState.wrist1 = wrist1 + math.pi / 2
    self.lastKnownJointState.wrist2 = wrist2
    self.lastKnownJointState.wrist3 = wrist3

    self:update_ghost()
  end
end

function Robot:disconnect()
  if not self.connected then
    return
  end

  server.stop_server()

  self.connected = false
end

return Robot
