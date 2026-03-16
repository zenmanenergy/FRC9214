import math
from Constants import (
    modulePositions,
    moduleOffsets,
    kMaxSpeed
)
from SwerveModule import SwerveModule

# Defines the swerve drive system, cosisting of four swerve modules
class SwerveDrive: 

    def __init__(self, modules):

        # stores each swerve module
        self.modules = modules
        
        # stores the position of each module relative to the robot center
        self.wheel_positions =  modulePositions

    # Main method for controlling the robots movement (forward, strafe, and rotation)
    def drive(self, forward, strafe, rotate):
        
        # stores calculated speed and angle for each wheel
        wheel_commands = {}

        # Stores the maximum speed calculated for any wheel, used for normalization if any exceed max speed
        max_speed = 0

        # Calculates the speed and angle for each wheel based on the desired forward, strafe, and rotation inputs
        for name, pos in self.wheel_positions.items():

            # forward and backward
            vx = forward - rotate * pos["y"]
            
            # side to side
            vy = strafe + rotate * pos["x"]

            # magnitude of the velocity vector gives the speed, and the angle is calculated using atan2
            speed = math.hypot(vx, vy)

            # direction the wheel should point in degrees, normalized to [0, 360)  
            angle = math.degrees(math.atan2(vy, vx)) % 360

            # store the calculated speed and angle for this wheel
            wheel_commands[name] = {"speed": speed, "angle": angle}

            # track the maximum speed calculated for any wheel  
            max_speed = max(max_speed, speed)

        # if any wheel is going faster than 1.0 (100% of theoretical max), all wheel speeds are slowed down proportionally so the ratios stay the same
        if max_speed > 1.0:
            for cmd in wheel_commands.values():
                cmd["speed"] /= max_speed

        # Wheel by wheel, multiplies normalized speed by the maximum speed to get the actual speed command, and sends the speed and angle to each swerve module
        for name, cmd in wheel_commands.items():
            module = self.modules[name]
            module.set(cmd["speed"] * kMaxSpeed, cmd["angle"])