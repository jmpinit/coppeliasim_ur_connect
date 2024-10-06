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

--- Return true if we have a connection to the real robot.
function Robot:is_connected()
  return self.connected
end

--- Return the joint angles of a UR5 model in the scene.
local get_joints_from_ur5 = function(objHandle)
  local sceneJoints = sim.getObjectsInTree(objHandle, sim.object_joint_type)

  return {
    base = sim.getJointPosition(sceneJoints[1]),
    shoulder = sim.getJointPosition(sceneJoints[2]),
    elbow = sim.getJointPosition(sceneJoints[3]),
    wrist1 = sim.getJointPosition(sceneJoints[4]),
    wrist2 = sim.getJointPosition(sceneJoints[5]),
    wrist3 = sim.getJointPosition(sceneJoints[6]),
  }
end

--- Return the joint angles of the robot model in the scene.
function Robot:get_virtual_joint_angles()
  return get_joints_from_ur5(self.handle)
end

local apply_joints_to_ur5 = function(objHandle, jointState)
  local sceneJoints = sim.getObjectsInTree(objHandle, sim.object_joint_type)

  sim.setJointPosition(sceneJoints[1], jointState.base)
  sim.setJointPosition(sceneJoints[2], jointState.shoulder)
  sim.setJointPosition(sceneJoints[3], jointState.elbow)
  sim.setJointPosition(sceneJoints[4], jointState.wrist1)
  sim.setJointPosition(sceneJoints[5], jointState.wrist2)
  sim.setJointPosition(sceneJoints[6], jointState.wrist3)
end

--- Move the ghost to the last known pose of the real robot.
function Robot:update_ghost()
  if self.ghostHandle == nil or self.lastKnownJointState == nil then
    return
  end

  apply_joints_to_ur5(self.ghostHandle, self.lastKnownJointState)
end

--- Return the joint state from the real robot and store it in the last known joint state.
function Robot:get_real_joint_angles()
  if not self.connected then
    return
  end

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
    return self.lastKnownJointState
  end

  return nil
end

--- Retrieve the joint state from the real robot and use it to update the ghost model.
function Robot:apply_real_pose()
  local angles = self:get_real_joint_angles()

  if angles ~= nil then
    self:update_ghost()
  end
end

function Robot:freedrive()
  if not self.connected then
    return
  end

  local pose
  if self.lastKnownJointState == nil then
    local jointAngles = self:get_virtual_joint_angles()

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

  self:apply_real_pose()
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
    jointAngles = self:get_virtual_joint_angles()
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

  self:apply_real_pose()
end

--- Disconnect from the real robot and stop the control server thread.
function Robot:disconnect()
  if not self.connected then
    return
  end

  server.stop_server()

  self.connected = false
end

return {
  Robot=Robot,
  get_joints_from_ur5=get_joints_from_ur5,
  apply_joints_to_ur5=apply_joints_to_ur5,
}
