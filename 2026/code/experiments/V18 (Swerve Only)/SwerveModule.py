import math

# Defines a python class for a single swerve module, which involves two main components: 
# a drive and a turn motor

class SwerveModule:

    # __init__ initializes the swerve module with the given drive motor, turn motor, encoder, and offset
    def __init__(self, drive_motor, turn_motor, encoder, offset=0):

        # motor controlling forward/backward movement
        self.drive_motor = drive_motor

        #motor controlling wheel rotation (angle)
        self.turn_motor = turn_motor

        # reads the wheel's current rotation angle (in degrees)
        self.encoder = encoder

        # used to zero the wheel at setup
        self.offset = offset

        self.kP = 0.01

    # Reads encoder value, subtracts offset, returns current wheel orientation (in degrees)
    def get_angle(self):
        angle = (self.encoder.get() - self.offset) % 360 
        return angle
    
    # Sets the drive motor speed (from -1 to 1)
    def set_drive_speed(self, speed):
        self.drive_motor.set(speed)
    
    # Sets the power for the turn motor (from -1 to 1)
    def set_turn_power(self, power):
        self.turn_motor.set(power)

    # Stops both drive and turning motors immediately, used for emergency stops
    def stop(self):
        self.drive_motor.set(0)
        self.turn_motor.set(0)


    def set_angle(self, target_angle):
        
        # Reads current angle, uses function L.26
        current = self.get_angle()

        # Calculates how far the wheel needs to turn to reach the target angle
        error = target_angle - current

        # Determines which way to rotate is faster to reach target angle
        if error > 180:
            error -= 360
        elif error < -180:
            error += 360

        # Calculates motor power
        power = error * self.kP

        # Prevents sending invalid values outside of [-1, 1] to the motor
        power = max(-1, min(1, power))

        # Applies power to turn the motor
        self.turn_motor.set(power)

    def set(self, speed, angle):

        # Rotates wheel toward desired angle, using function L.44
        self.set_angle(angle)

        # Moves wheel forward/backward at desired speed, using function L.31
        self.set_drive_speed(speed)
