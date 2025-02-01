# MyRobot.py
import wpilib
from phoenix5 import WPI_TalonSRX, NeutralMode
from DriveTurnPID import MotorGroup, DriveTurnPID

class MyRobot(wpilib.TimedRobot):
	def robotInit(self):
		# Initialize joystick on port 0
		self.joystick = wpilib.Joystick(0)
		
		# Initialize motor controllers
		self.leftFront = WPI_TalonSRX(1)
		self.leftRear = WPI_TalonSRX(4)
		self.rightFront = WPI_TalonSRX(3)
		self.rightRear = WPI_TalonSRX(2)
		
		# Set motors to Brake mode
		self.leftFront.setNeutralMode(NeutralMode.Brake)
		self.leftRear.setNeutralMode(NeutralMode.Brake)
		self.rightFront.setNeutralMode(NeutralMode.Brake)
		self.rightRear.setNeutralMode(NeutralMode.Brake)
		
		# Create motor groups
		self.leftMotors = MotorGroup([self.leftFront, self.leftRear])
		self.rightMotors = MotorGroup([self.rightFront, self.rightRear])
		
		# Create DriveTurnPID instance.
		# For example, left encoder on DIO ports (1,2), right encoder on (3,4),
		# wheel diameter 152.4 mm, and 2048 counts per revolution.
		self.driveTurnPID = DriveTurnPID(
			self.leftMotors, self.rightMotors,
			leftEncoderPorts=(1, 2), rightEncoderPorts=(3, 4),
			wheel_diameter_mm=152.4, encoder_cpr=2048
		)
		
		# Optionally, run autotuning (make sure the robot is in a safe state!)
		# Uncomment these lines to perform autotuning. (They will save new settings.)
		# print("Autotuning turning PID...")
		# self.driveTurnPID.autotune(mode='turn')
		# print("Autotuning drive PID...")
		# self.driveTurnPID.autotune(mode='drive')
		
		# Command state: None means no active command.
		self.current_command = None
	
	def teleopPeriodic(self):
		# Check joystick buttons to trigger commands.
		# Button mappings:
		# Button 1: Drive forward 4000 mm
		# Button 2: Drive backward 4000 mm
		# Button 3: Turn right 90°
		# Button 4: Turn left 90°
		if self.current_command is None:
			if self.joystick.getRawButton(1):
				self.driveTurnPID.start_drive_distance(4000)
				self.current_command = 'drive'
			elif self.joystick.getRawButton(2):
				self.driveTurnPID.start_drive_distance(-4000)
				self.current_command = 'drive'
			elif self.joystick.getRawButton(3):
				self.driveTurnPID.start_turn_degrees(90)
				self.current_command = 'turn'
			elif self.joystick.getRawButton(4):
				self.driveTurnPID.start_turn_degrees(-90)
				self.current_command = 'turn'
		else:
			# Update the active command.
			if self.current_command == 'drive':
				done = self.driveTurnPID.update_drive()
			elif self.current_command == 'turn':
				done = self.driveTurnPID.update_turn()
			else:
				done = True
			
			if done:
				self.current_command = None

if __name__ == "__main__":
	wpilib.run(MyRobot)
