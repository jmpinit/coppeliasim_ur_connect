# CoppeliaSim UR Connect

Control a real Universal Robots UR3, UR5, or UR10 based on the state of one
simulated in [the CoppeliaSim robot
simulator](https://www.coppeliarobotics.com/).

## How it Works

A script is sent to the robot which tells it to connect back to a server running
on the host computer and listen for commands, which it will then execute to
carry out joint moves, servo to target joint states, or freedrive.  It sends
back the current joint state of the robot so the movement can be tracked.
The server is implemented in C (in **core**) and exposes a Lua API to be used
from Lua running under CoppeliaSim.

The Lua code in CoppeliaSim makes it easy to set up the sim and real robot
twins. A transparent "ghost" robot is created to display the current state of
the real robot in CoppeliaSim.

## Installation

This project has only been tested on macOS. It was most recently tested with
CoppeliaSim 4.6.0 (rev 10).

### Installing Lua 5.3

This version of CoppeliaSim uses lua 5.3, so I installed that package from
homebrew with `brew install lua@5.3`.

Make sure that pkgconfig can find your lua installation. You might have to do
something like this (homebrew may tell you this if you have another lua
installation):

```bash
export PKG_CONFIG_PATH="/opt/homebrew/opt/lua@5.3/lib/pkgconfig"
```

### Building the Server

Build the server by going into **ur_connect/core** and running `make`. If it's
successful then you will have a **core.so** file one directory up, at
**ur_connect/core.so**.

### Installing the Server

Now you can symlink the **ur_connect** directory to these two locations:
- **/Applications/coppeliaSim.app/Contents/Resources/luarocks/lib/lua/5.3/ur_connect**
- **/Applications/coppeliaSim.app/Contents/Resources/luarocks/share/lua/5.3/ur_connect**

Now if you start CoppeliaSim and try `require('ur_connect/robot')` in the Lua
sandbox at the bottom you should see a table printed and no errors.

## Use

Inside the CoppeliaSim scene the script should be called from a threaded child
script attached to the UR5 object. Here's an example:

```lua
local Robot = require('ur_connect/robot')

function sysCall_init()
    sim = require('sim')
    sim.setStepping(true)
end

function sysCall_thread()
    -- We should be attached to a UR5
    -- Replace the first IP with that of your machine
    -- and the second one with the IP configured for your robot
    local robot = Robot(sim.getObject('.'), '172.31.1.1', '172.31.1.3')

    -- Connecting causes the script to load the control program onto the robot
    -- system and accept a connection back from the robot for control
    robot:connect()

    while true do
        while not sim.getSimulationStopping() do
            robot:servo_to(robot:get_joint_angles())
            print(robot:get_joint_angles())
            sim.step() -- resume in next simulation step
        end
        
        sim.step()
    end

    print('Disconnecting the robot')

    -- Cleans up the network resources
    robot:disconnect()
end
```

If you want to freedrive the robot you can invoke `robot:freedrive()`.
