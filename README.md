# V-REP UR Connect

This Lua code is meant to synchronize the state of a Universal Robots UR3, UR5, or UR10 in the V-REP
robot simulator with a real robot.

Given the IP address of the robot this code will load a control program onto the robot's control
system and then accept a connection back from the robot which is subsequently used to stream joint
targets to the robot and receive the joint states from the robot.

Inside V-REP the script should be called from a threaded child script, like so:

```lua
local ur_connect = require('ur_connect')

function sysCall_threadmain()
  -- The scene contains a UR5 robot model named 'UR5'
  local robot = ur_connect.Robot(sim.getObjectHandle('UR5'), theRobotsIPAddress)

  -- Connecting causes the script to load the control program onto the robot system and accept
  -- a connection back from the robot for control
  robot:connect()

  while sim.getSimulationState() ~= sim.simulation_advancing_abouttostop do
    -- Servoing updates the joint targets for the real robot, causing it to move
    -- This should ideally be called very quickly (>100 Hz) at regular intervals, and match the
    -- tuning parameters in the servoj call of the robot control script
    robot:servo_to(robot:get_joint_angles())
  end

  -- Cleans up the network resources
  robot:disconnect()
end
```

## Installation

The **ur_connect** directory is the Lua module for connecting V-REP to a UR robot. Lua modules are
installed in V-REP by putting them in the same directory as the main application binary. On macOS
that's **/Applications/V-REP_version_and_stuff/vrep.app/Contents/MacOS**. Copy the **ur_connect**
directory there.

The module uses the [BitOp library](http://bitop.luajit.org/) so that will need to be installed too.
Make sure to compile it for Lua 5.1 because that's the version of Lua used by V-REP. To compile
BitOp do the following (on macOS):

0. Install [Homebrew](https://brew.sh/).
1. Install Lua 5.1 with `brew install lua@5.1`.
2. Download [LuaBitOp-1.0.2.zip](http://bitop.luajit.org/download/LuaBitOp-1.0.2.zip) and unzip.
3. Compile with:
  a. `gcc -I/usr/local/include/lua5.1 -dynamiclib -single_module -undefined dynamic_lookup -o bit.o bit.c`
  b. `gcc -shared -fPIC -I/usr/local/include/lua5.1 -llua5.1 -o bit.so bit.o`
4. Copy the resulting **bit.so** to the aforementioned V-REP directory.

## Documentation

Generate by running `make docs`. [LDoc](http://stevedonovan.github.io/ldoc/) must be installed.

Run tests with `make test`.
