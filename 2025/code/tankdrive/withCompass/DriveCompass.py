
import wpilib
from wpimath.controller import PIDController
import navx
from phoenix5 import WPI_TalonSRX

class MyRobot(wpilib.TimedRobot):
    def robotInit(self):
        # Motors
        self.LeftFrontMotor = WPI_TalonSRX(1)
        self.LeftRearMotor = WPI_TalonSRX(4)
        self.RightFrontMotor = WPI_TalonSRX(3)
        self.RightRearMotor = WPI_TalonSRX(1)

        # Reverse the direction of the right motors
        self.RightFrontMotor.setInverted(True)
        self.RightRearMotor.setInverted(True)

         # NavX Sensor
        self.navx = navx.AHRS(wpilib.SPI.Port.kMXP)  # Replace with your connection type if different

        # PID Controller for turning
        self.turn_pid = PIDController(0.02, 0.0, 0.001)  # Tune PID values as needed
        self.turn_pid.setTolerance(2.0)  # Degrees tolerance
        self.turn_pid.enableContinuousInput(-180.0, 180.0)  # Handle wraparound

        # State variables
        self.turning = False
        self.autonomous_step = 0
        self.initial_turn_angle = None

        # Timer
        self.timer = wpilib.Timer()

def TurnUsingCompass(self, target_heading_offset):
    # Initialize turning by storing the starting yaw
    if self.initial_turn_angle is None:
        self.initial_turn_angle = self.navx.getYaw()

    # Calculate the target heading based on the initial yaw and offset
    target_heading = self.initial_turn_angle + target_heading_offset

    # Normalize the target heading to stay within -180 to 180 degrees
    if target_heading > 180:
        target_heading -= 360
    elif target_heading < -180:
        target_heading += 360

    # Set the target heading for the PID controller
    self.turn_pid.setSetpoint(target_heading)
    self.turning = True

def HandleTurning(self):
    current_angle = self.navx.getYaw()
    turn_output = self.turn_pid.calculate(current_angle)

    self.LeftFrontMotor.set(turn_output)
    self.LeftRearMotor.set(turn_output)
    self.RightFrontMotor.set(-turn_output)
    self.RightRearMotor.set(-turn_output)

    if self.turn_pid.atSetpoint():
        self.LeftFrontMotor.set(0)
        self.LeftRearMotor.set(0)
        self.RightFrontMotor.set(0)
        self.RightRearMotor.set(0)
        self.turning = False
        self.initial_turn_angle = None  # Reset for the next turn
        return True
    return False

def DriveForward(self, speed, duration):
    # Drive forward for a specified duration
    if self.timer.get() == 0:
        self.timer.start()
    if self.timer.get() < duration:
        self.LeftFrontMotor.set(speed)
        self.LeftRearMotor.set(speed)
        self.RightFrontMotor.set(speed)
        self.RightRearMotor.set(speed)
    else:
        self.LeftFrontMotor.set(0)
        self.LeftRearMotor.set(0)
        self.RightFrontMotor.set(0)
        self.RightRearMotor.set(0)
        self.timer.stop()
        self.timer.reset()
        return True
    return False

def autonomousInit(self):
    # Set initial state for autonomous
    self.turning = False
    self.autonomous_step = 0
    self.timer.reset()
    self.initial_turn_angle = None

def autonomousPeriodic(self):
    if self.autonomous_step == 0:  # Step 1: Drive forward
        if self.DriveForward(0.5, 2):
            self.autonomous_step = 1

    elif self.autonomous_step == 1:  # Step 2: Turn left 90 degrees
        if not self.turning:
            self.TurnUsingCompass(-90.0)
        if self.HandleTurning():
            self.autonomous_step = 2

    elif self.autonomous_step == 2:  # Step 3: Drive forward
        if self.DriveForward(0.5, 2):
            self.autonomous_step = 3

    elif self.autonomous_step == 3:  # Step 4: Turn left 90 degrees
        if not self.turning:
            self.TurnUsingCompass(-90.0)
        if self.HandleTurning():
            self.autonomous_step = 4

def teleopPeriodic(self):
    pass
if __name__ == "__main__":
    wpilib.run(MyRobot)
