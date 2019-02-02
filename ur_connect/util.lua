------------
--- Utility functions.
-- @module util

local bit = require('bit')

--- Create a class to use for OOP patterns.
-- @param parent An optional parent class to extend
function class(parent)
  local classTable = {}
  classTable.__index = classTable

  setmetatable(classTable, {
    __index = parent,
    __call = function (cls, ...)
      -- Create self from the object returned by constructor of the parent
      local self = setmetatable(parent == nil and {} or parent(), cls) -- cls is the new class table
      self:_init(...) -- Call constructor with supplied arguments
      return self
    end,
  })

  return classTable
end

--- Check if a file exists on the filesystem.
function file_exists(filename)
  local file = io.open(filename, 'rb')

  if file then
    file:close()
  end

  return file ~= nil
end

--- Read an entire file given a filename.
-- @param filename The path to the file to read.
-- @return The contents of the file.
function read_file(filename)
  -- Ensure the file exists
  if not file_exists(filename) then
    return nil
  end

  local file = io.open(filename, 'rb')
  return file:read('*a')
end

--- Convert a string to a table of byte values.
function string_to_bytes(s)
  local bytes = {}

  for i=1,string.len(s) do
    table.insert(bytes, string.byte(s, i))
  end

  return bytes
end

--- Convert a table of byte values to a string.
function bytes_to_string(t)
  local bytearr = {}

  for _, v in ipairs(t) do
    table.insert(bytearr, string.char(v))
  end

  return table.concat(bytearr)
end

--- Split a 32 bit int into bytes.
-- @return A table of bytes in big-endian order.
function int32_to_bytes(v)
  if (v < 0) then
    v = math.ceil(v)

    -- Two's complement
    v = 0xffffffff + v
  else
    v = math.floor(v)
  end

  local bytes = {
    bit.band(bit.rshift(v, 24), 0xff),
    bit.band(bit.rshift(v, 16), 0xff),
    bit.band(bit.rshift(v, 8), 0xff),
    bit.band(v, 0xff),
  }

  return bytes
end

--- Check if a value is in the range of a byte.
-- @return True if value is a byte.
function is_byte(v)
  if v == nil or not type(v) == 'number' then
    -- Must be a number to be able to be a byte
    return false
  end

  return not (v < 0 or v > 0xff)
end

--- Combine 4 bytes into a 32 bit int value.
function bytes_to_int32(b3, b2, b1, b0)
  assert(is_byte(b3))
  assert(is_byte(b2))
  assert(is_byte(b1))
  assert(is_byte(b0))

  local val = bit.bor(bit.lshift(b3, 24), bit.lshift(b2, 16), bit.lshift(b1, 8), b0)

  if val < 0 then
    val = val + 1
  end

  return val
end

-- Adapted from https://stackoverflow.com/questions/1426954/split-string-in-lua
function string_split(inputstr, sep)
  if sep == nil then
    sep = "%s"
  end

  local t = {};
  local i = 1

  for str in string.gmatch(inputstr, "([^"..sep.."]+)") do
    t[i] = str
    i = i + 1
  end

  return t
end

function string_trim(s)
  return (s:gsub('^%s*(.-)%s*$', '%1'))
end

function is_valid_ip_address(ipText)
  if ipText == nil then
    return false
  end

  local ipParts = string_split(ipText, '.')
  
  -- Check length
  if #ipParts ~= 4 then
    return false
  end

  -- Check values express bytes
  for i, v in ipairs(ipParts) do
    if not is_byte(tonumber(v)) then
      return false
    end
  end

  return true
end

function value_or_error(valueOrErrorCode, msg)
  if msg == nil then
    msg = 'Unknown error'
  end

  if valueOrErrorCode == -1 then
    error(msg)
  else
    return valueOrErrorCode
  end
end

function set_color(object, r, g, b, a)
  sim.setShapeColor(object, nil, sim.colorcomponent_ambient_diffuse, { r, g, b })
  sim.setShapeColor(object, nil, sim.colorcomponent_emission, { r, g, b })
  sim.setShapeColor(object, nil, sim.colorcomponent_specular, { r, g, b })
  sim.setShapeColor(object, nil, sim.colorcomponent_transparency, { a })
end

function make_ghost_model(modelBaseHandle, filterDummy)
  local ghostModelProperties = sim.modelproperty_not_collidable
    + sim.modelproperty_not_measurable
    + sim.modelproperty_not_renderable
    + sim.modelproperty_not_detectable
    + sim.modelproperty_not_cuttable
    + sim.modelproperty_not_dynamic
    + sim.modelproperty_not_respondable
    + sim.modelproperty_scripts_inactive

  local copiedModel = sim.copyPasteObjects({ modelBaseHandle }, 1)[1]
  value_or_error(sim.setModelProperty(copiedModel, ghostModelProperties))

  local objectsInModel = sim.getObjectsInTree(copiedModel)
  for i, object in ipairs(objectsInModel) do
    value_or_error(sim.setModelProperty(object, ghostModelProperties))

    local modelName = string.sub(sim.getObjectName(object), 0, -3) -- remove "#0"
    local ghostName =  modelName .. '_ghost'
    sim.setObjectName(object, ghostName)

    local objType = sim.getObjectType(object)

    -- Remove any scripts from the original object that might act on the ghost
    local script = sim.getScriptAssociatedWithObject(object)
    if script ~= -1 and script ~= nil then
      sim.removeScript(script)
    end

    if objType == sim.object_shape_type then
      set_color(object, 1, 1, 1, 0.3)
    elseif objType == sim.object_joint_type then
      -- Set the joint mode to passive so that it will maintain a pose it is set to
      sim.setJointMode(object, sim.jointmode_passive, 0)
    elseif objType == sim.object_dummy_type and filterDummy then
      sim.removeObject(object)
    end
  end

  return copiedModel
end

--- Load a string saved to the app session associated with a string key
function session_load_string(key)
  local stringData = sim.readCustomDataBlock(sim.handle_app, key)

  if stringData == nil then
    -- Found no saved data
    return nil
  end

  local stringBytes = sim.unpackUInt8Table(stringData)
  
  local str = ''
  for i, byte in ipairs(stringBytes) do
    str = str .. string.char(byte)
  end

  return str
end

--- Save a string to the app session associated with a string key
function session_save_string(key, str)
  local stringBytes = string_to_bytes(str)
  sim.writeCustomDataBlock(sim.handle_app, key, sim.packUInt8Table(stringBytes))
end

--- @export
return {
  class = class,
  file_exists = file_exists,
  read_file = read_file,
  string_to_bytes = string_to_bytes,
  bytes_to_string = bytes_to_string,
  int32_to_bytes = int32_to_bytes,
  is_byte = is_byte,
  bytes_to_int32 = bytes_to_int32,
  string_split = string_split,
  string_trim = string_trim,
  is_valid_ip_address = is_valid_ip_address,
  make_ghost_model = make_ghost_model,
  session_load_string = session_load_string,
  session_save_string = session_save_string,
}
