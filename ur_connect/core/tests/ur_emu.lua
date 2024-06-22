--- Emulate the control script loaded onto the Universal Robots robot

--[[
Requires the LuaBitOp library:

1. Download [LuaBitOp-1.0.2.zip](http://bitop.luajit.org/download/LuaBitOp-1.0.2.zip) and unzip.
2. Compile with:
  a. `gcc -c -I/opt/homebrew/opt/lua@5.3/include/lua5.3 -fPIC bit.c -o bit.o`
  b. `gcc -shared -fPIC -o bit.so bit.o -L/Applications/coppeliaSim.app/Contents/Frameworks -llua.5.3`
3. Copy the resulting **bit.so** to /Applications/coppeliaSim.app/Contents/Resources/luarocks/lib/lua/5.3
--]]
local bit = require('bit')

local socket = require('socket')
local util = require('ur_connect/util')

local MULT_JOINTSTATE = 1000000
local NET_TIMEOUT = 3
local SCRIPT_PORT = 30002

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

function die(msg)
  error(msg)
end

function receive_script()
  local server = socket.tcp()
  local success, message = server:bind('127.0.0.1', SCRIPT_PORT)

  if not success then
    error(message)
  end

  server:listen()

  while true do
    local client = server:accept()
    print('Client connected')

    local script = client:receive('*a')
    print(script)

    if script ~= nil and #script > 0 then
      client:close()
      server:close()
      return
    end
  end
end

function emulate_robot(ip, port)
  receive_script()

  local client = socket.tcp()
  client:settimeout(NET_TIMEOUT)
  local res, errmsg = client:connect(ip, port)

  if res == nil then
    die('Failed to open connection: ' .. errmsg)
  end

  while true do
    local dataStr = client:receive(7 * 4)

    if dataStr == nil then
      return
    end

    local data = util.string_to_bytes(dataStr)

    local pose = {}
    local msg = ''
    for i = 0, 5 do
      local b3 = data[i * 4 + 1]
      local b2 = data[i * 4 + 2]
      local b1 = data[i * 4 + 3]
      local b0 = data[i * 4 + 4]

      local value = bytes_to_int32(b3, b2, b1, b0)
      table.insert(pose, value / MULT_JOINTSTATE)

      msg = msg .. pose[i + 1] .. ', '
    end

    print(msg)

    local reply = ''
    for i = 1, 6 do
      local value = int32_to_bytes(pose[i] * MULT_JOINTSTATE)
      reply = reply .. util.bytes_to_string(value)
    end
    reply = reply .. string.char(13)

    client:send(reply)
  end

  client:close()
end

function main()
  -- First arg is the address to connect to
  -- Second arg is the port to connect on

  local ip = arg[1]
  local port = tonumber(arg[2])

  if ip == nil or port == nil then
    die('Usage: ur_emu.lua <address> <port>')
  end

  while true do
    emulate_robot(ip, port)
  end
end

main()
