local ur_connect = require('ur_connect.core')

ur_connect.update_pose({ 4, 3, 1, 9, 3, 2 })

ur_connect.start_server('127.0.0.1', 8080)
print('Server started')

for i = 1, 10 do
  local pose = ur_connect.get_pose()

  local msg = ''

  if pose ~= nil then
    for j = 1, 6 do
      msg = msg .. pose[j] .. ', '
    end
  end

  print(msg)

  os.execute('sleep 1')
end

print('Stopping server')
ur_connect.stop_server()
print('Done')

