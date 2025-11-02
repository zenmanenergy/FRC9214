import wpilib  # FIRST Robotics library
import ctre  # Zippy wheel motor controller library
import rev  # Zippy arm motor controller library
from ctre import NeutralMode

class MyRobot(wpilib.TimedRobot):
	def robotInit(self):  # Initializes joystick, motors, and encoder
		# Joystick and motor setup
		self.DriveJoystick = wpilib.Joystick(0)  # Joystick port 0
		self.LeftFrontMotor = ctre.WPI_TalonSRX(1)
		self.LeftRearMotor = ctre.WPI_TalonSRX(2)
		self.RightFrontMotor = ctre.WPI_TalonSRX(3)
		self.RightRearMotor = ctre.WPI_TalonSRX(4)

		self.LeftFrontMotor.setNeutralMode(NeutralMode.Brake)
		self.LeftRearMotor.setNeutralMode(NeutralMode.Brake)
		self.RightFrontMotor.setNeutralMode(NeutralMode.Brake)
		self.RightRearMotor.setNeutralMode(NeutralMode.Brake)

		# Encoder setup
		self.WHEEL_DIAMETER_MM = 152.4  # mm
		self.WHEEL_CIRCUMFERENCE_MM = self.WHEEL_DIAMETER_MM * 3.141592653589793
		self.ENCODER_CPR = 2048  # Counts per revolution for the encoder

		# Robot dimensions (wheelbase diameter for rotation calculation)
		self.ROBOT_WIDTH_MM = 508  # Distance between wheels, adjust as needed
		self.ROBOT_CIRCUMFERENCE_MM = self.ROBOT_WIDTH_MM * 3.141592653589793

		# Initialize encoder: Blue (Signal B) in DIO 1, Yellow (Signal A) in DIO 2
		self.left_encoder = wpilib.Encoder(1, 2)  # Left encoder on DIO 1, 2
		self.right_encoder = wpilib.Encoder(3, 4)  # Right encoder on DIO 3, 4
		self.left_encoder.setDistancePerPulse(self.WHEEL_CIRCUMFERENCE_MM / self.ENCODER_CPR)
		self.right_encoder.setDistancePerPulse(self.WHEEL_CIRCUMFERENCE_MM / self.ENCODER_CPR)

		# Travel state
		self.target_distance = None
		self.travel_in_progress = False
		self.offset = 600
		self.RightMotorSpeedFactor = 1.0 # This is so the robot does not tilt right

	def teleopInit(self):
		# Reset encoder distance at the start of teleop
		self.left_encoder.reset()
		self.right_encoder.reset()

	def teleopPeriodic(self):
		# Periodic joystick and driving updates
		self.JoystickPeriodic()

		# Call travel function based on state
		self.checkForTravel()

	def JoystickPeriodic(self):
		self.DRIVE_BUTTON_A = self.DriveJoystick.getRawButton(1)  # A button
		self.DRIVE_BUTTON_Y = self.DriveJoystick.getRawButton(4)  # Y button

	def checkForTravel(self):
		"""
		Checks for button presses and initiates travel operations.
		"""
		# Start travel if not already in progress
		if self.DRIVE_BUTTON_Y and not self.travel_in_progress:
			self.target_distance = 6000  # Travel 3000 mm forward
			self.startTravel()
		elif self.DRIVE_BUTTON_A and not self.travel_in_progress:
			self.target_distance = -6000  # Travel 3000 mm backward
			self.startTravel()

		# Continue travel if in progress
		if self.travel_in_progress:
			self.travel_in_progress = self.travelDistance(self.target_distance)

	def startTravel(self):
		"""
		Initializes travel operation by resetting encoders.
		"""
		self.left_encoder.reset()
		self.right_encoder.reset()
		self.travel_in_progress = True
		self.RightMotorSpeedFactor = 1.0 # This is so the robot does not tilt right

		
	def travelDistance(self, distance_mm):
		"""
		Makes the robot travel a specified distance in millimeters at a constant speed.
		Uses two encoders for more accurate distance tracking.
		Returns True if the robot is still moving, False if it has reached the target distance.
		"""
		# Determine the direction of travel
		direction = 1 if distance_mm > 0 else -1
		target_distance = abs(distance_mm) - self.offset
		# Travel speed
		speed = 0.4  # Adjust this speed as necessary

		# Calculate the average distance traveled by both encoders
		left_distance = abs(self.left_encoder.getDistance())
		right_distance = abs(self.right_encoder.getDistance())
		average_distance = (left_distance + right_distance) / 2
		if left_distance > right_distance:
			self.RightMotorSpeedFactor = self.RightMotorSpeedFactor + 0.001
		else:
			self.RightMotorSpeedFactor = self.RightMotorSpeedFactor - 0.001
		# Print encoder values for debugging
		print(f"Left Distance: {left_distance:.2f} mm, Right Distance: {right_distance:.2f} mm, {self.RightMotorSpeedFactor}")

		# Check if the target distance is reached
		if average_distance < target_distance:
			# Apply constant speed to the motors
			self.LeftFrontMotor.set(direction * speed)
			self.LeftRearMotor.set(direction * speed)
			self.RightFrontMotor.set(-1 * direction * speed * self.RightMotorSpeedFactor)
			self.RightRearMotor.set(-1 * direction * speed * self.RightMotorSpeedFactor)
			return True
		else:
			# Stop the motors
			self.LeftFrontMotor.set(0)
			self.LeftRearMotor.set(0)
			self.RightFrontMotor.set(0)
			self.RightRearMotor.set(0)
			# self.applyBreak(direction)
			return False

	def applyBreak(self, direction):

		brake_speed=-0.4 * direction
		brake_duration=0.3

		self.LeftFrontMotor.set(brake_speed)
		self.LeftRearMotor.set(brake_speed)
		self.RightFrontMotor.set(-brake_speed)
		self.RightFrontMotor.set(-brake_speed)

		timer = wpilib.Timer()
		timer.start()
		while timer.get() < brake_duration:
			pass
		timer.stop()

		self.LeftFrontMotor.set(0)
		self.LeftRearMotor.set(0)
		self.RightFrontMotor.set(0)
		self.RightRearMotor.set(0)

if __name__ == "__main__":
	wpilib.run(MyRobot)
