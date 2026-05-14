#!/usr/bin/env python3

## Notes
# Got this script from the example at 192.168.1.207:18333/project/python

import time
from xarm.wrapper import XArmAPI

# Constants
ARM_IP_ADDRESS: str = '192.168.1.207'

ARM_TARGET_POSITION: dict[str, int] = {
    "x"     : 300, 
    "y"     : 0, 
    "z"     : 550, 
    "roll"  : 180, 
    "pitch" : 0, 
    "yaw"   : 0
}

# Instantiation API
arm = XArmAPI(ARM_IP_ADDRESS)
time.sleep(0.5)

# Clean error and warn
if arm.warn_code != 0:
    arm.clean_warn()
if arm.error_code != 0:
    arm.clean_error()

# Enable the robot
arm.motion_enable(enable=True)

# Set mode and state
arm.set_mode(0)
arm.set_state(0)

# Linear motion, move to pose [300,0,200,180,0,0]
arm.set_position(
    x     = ARM_TARGET_POSITION["x"], 
    y     = ARM_TARGET_POSITION["y"], 
    z     = ARM_TARGET_POSITION["z"], 
    roll  = ARM_TARGET_POSITION["roll"], 
    pitch = ARM_TARGET_POSITION["pitch"], 
    yaw   = ARM_TARGET_POSITION["yaw"], 
    speed = 100, 
    wait  = True
)

arm.disconnect()

