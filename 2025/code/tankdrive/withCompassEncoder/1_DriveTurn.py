# MyRobot.py
import wpilib
from phoenix5 import WPI_TalonSRX, NeutralMode
from DriveTurn import MotorGroup, DriveControl

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
		
		# Create motor groups (adjust inversion as necessary)
		self.leftMotors = MotorGroup([self.leftFront, self.leftRear])
		self.rightMotors = MotorGroup([self.rightFront, self.rightRear])
		
		# Create DriveControl with sensor configuration
		# For example, left encoder is on DIO ports 1 and 2, right encoder on 3 and 4,
		# wheel diameter is 152.4 mm, and the encoder has 2048 counts per revolution.
		self.driveControl = DriveControl(
			self.leftMotors,
			self.rightMotors,
			leftEncoderPorts=(1, 2),
			rightEncoderPorts=(3, 4),
			wheel_diameter_mm=152.4,
			encoder_cpr=2048
		)
		
		# Current command state: None means no active command.
		self.current_command = None
	
	def teleopPeriodic(self):
		# If no command is running, check for button presses.
		if self.current_command is None:
			# Button mappings:
			# Button 1: Drive forward 4000 mm
			# Button 2: Drive backward 4000 mm
			# Button 3: Turn right 90°
			# Button 4: Turn left 90°
			if self.joystick.getRawButton(1):
				self.driveControl.start_drive_distance(4000, base_speed=0.4)
				self.current_command = 'drive'
			elif self.joystick.getRawButton(2):
				self.driveControl.start_drive_distance(-4000, base_speed=0.4)
				self.current_command = 'drive'
			elif self.joystick.getRawButton(3):
				self.driveControl.start_turn_degrees(90, base_speed=0.3)
				self.current_command = 'turn'
			elif self.joystick.getRawButton(4):
				self.driveControl.start_turn_degrees(-90, base_speed=0.3)
				self.current_command = 'turn'
		else:
			# Update the active command.
			if self.current_command == 'drive':
				done = self.driveControl.update_drive_distance()
			elif self.current_command == 'turn':
				done = self.driveControl.update_turn_degrees()
			else:
				done = True
			
			# If the command is finished, clear the state.
			if done:
				self.current_command = None

if __name__ == "__main__":
	wpilib.run(MyRobot)
