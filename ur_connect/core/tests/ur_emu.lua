--- Emulate the control script loaded onto the Universal Robots robot

local socket = require('socket')
local util = require('ur_connect/util')

local MULT_JOINTSTATE = 1000000
local NET_TIMEOUT = 3
local SCRIPT_PORT = 30002

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

      local value = util.bytes_to_int32(b3, b2, b1, b0)
      table.insert(pose, value / MULT_JOINTSTATE)

      msg = msg .. pose[i + 1] .. ', '
    end

    print(msg)

    local reply = ''
    for i = 1, 6 do
      local value = util.int32_to_bytes(pose[i] * MULT_JOINTSTATE)
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
