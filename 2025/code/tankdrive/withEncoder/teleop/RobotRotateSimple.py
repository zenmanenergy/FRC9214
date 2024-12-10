import wpilib
import ctre
from ctre import NeutralMode
from math import degrees, pi

class MyRobot(wpilib.TimedRobot):
	def robotInit(self):
		# Joystick and motor setup
		self.DriveJoystick = wpilib.Joystick(0)
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
		self.WHEEL_CIRCUMFERENCE_MM = self.WHEEL_DIAMETER_MM * pi
		self.ROBOT_WIDTH_MM = 508  # Distance between wheels, adjust as needed
		self.ROBOT_CIRCUMFERENCE_MM = self.ROBOT_WIDTH_MM * pi

		self.left_encoder = wpilib.Encoder(1, 2)
		self.right_encoder = wpilib.Encoder(3, 4)
		self.left_encoder.setDistancePerPulse(self.WHEEL_CIRCUMFERENCE_MM / 2048)
		self.right_encoder.setDistancePerPulse(self.WHEEL_CIRCUMFERENCE_MM / 2048)

		# Heading tracking
		self.current_heading = 0  # Start at 0 degrees
		self.target_heading = None
		self.rotating = False

	def teleopInit(self):
		# Reset encoders and heading at the start of teleop
		self.left_encoder.reset()
		self.right_encoder.reset()
		self.current_heading = 0

	def teleopPeriodic(self):
		# Read joystick buttons
		self.DRIVE_BUTTON_X = self.DriveJoystick.getRawButton(3)  # X button
		self.DRIVE_BUTTON_A = self.DriveJoystick.getRawButton(1)  # A button
		self.DRIVE_BUTTON_B = self.DriveJoystick.getRawButton(2)  # B button
		self.DRIVE_BUTTON_Y = self.DriveJoystick.getRawButton(4)  # Y button

		# Update current heading
		self.updateHeading()

		# Handle rotation to absolute headings
		if self.DRIVE_BUTTON_X:
			self.target_heading = 270  # Face left
			self.startRotation()
		elif self.DRIVE_BUTTON_A:
			self.target_heading = 180  # Face backward
			self.startRotation()
		elif self.DRIVE_BUTTON_B:
			self.target_heading = 90  # Face right
			self.startRotation()
		elif self.DRIVE_BUTTON_Y:
			self.target_heading = 0  # Face forward
			self.startRotation()

		# Continue rotation if in progress
		if self.rotating:
			self.rotating = self.rotateToHeading()

	def updateHeading(self):
		"""
		Updates the current heading based on encoder distances.
		"""
		left_distance = self.left_encoder.getDistance()
		right_distance = self.right_encoder.getDistance()
		rotation_distance = right_distance - left_distance
		self.current_heading += (rotation_distance / self.ROBOT_CIRCUMFERENCE_MM) * 360
		self.current_heading %= 360  # Keep within 0-360 degrees

	def startRotation(self):
		"""
		Initializes rotation by resetting encoders.
		"""
		self.left_encoder.reset()
		self.right_encoder.reset()
		self.rotating = True

	def rotateToHeading(self):
		"""
		Rotates the robot to the target heading.
		Returns True if still rotating, False if the target heading is reached.
		"""
		error = self.normalizeAngle(self.target_heading - self.current_heading)

		# Print debug info
		print(f"Current: {self.current_heading:.2f}°, Target: {self.target_heading:.2f}°, Error: {error:.2f}°")

		# Rotation speed based on error
		if abs(error) > 2:  # Threshold to stop rotation
			speed = 0.4 if error > 0 else -0.4
			self.LeftFrontMotor.set(-speed)
			self.LeftRearMotor.set(-speed)
			self.RightFrontMotor.set(speed)
			self.RightRearMotor.set(speed)
			return True
		else:
			# Stop the motors
			self.LeftFrontMotor.set(0)
			self.LeftRearMotor.set(0)
			self.RightFrontMotor.set(0)
			self.RightRearMotor.set(0)
			return False

	def normalizeAngle(self, angle):
		"""
		Normalizes an angle to the range [-180, 180].
		"""
		while angle > 180:
			angle -= 360
		while angle < -180:
			angle += 360
		return angle

if __name__ == "__
