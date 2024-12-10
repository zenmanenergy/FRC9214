import wpilib
import ctre
from ctre import NeutralMode

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
		self.ROBOT_WIDTH_MM = 508  # Adjust based on your robot's wheelbase width
		self.WHEEL_DIAMETER_MM = 152.4  # mm
		self.WHEEL_CIRCUMFERENCE_MM = self.WHEEL_DIAMETER_MM * 3.141592653589793
		self.ROBOT_CIRCUMFERENCE_MM = self.ROBOT_WIDTH_MM * 3.141592653589793

		self.left_encoder = wpilib.Encoder(1, 2)
		self.right_encoder = wpilib.Encoder(3, 4)
		self.left_encoder.setDistancePerPulse(self.WHEEL_CIRCUMFERENCE_MM / 2048)
		self.right_encoder.setDistancePerPulse(self.WHEEL_CIRCUMFERENCE_MM / 2048)

		# Rotation state
		self.target_rotation_distance = None
		self.current_heading = 0  # Track the robot's current heading
		self.rotating = False

	def teleopInit(self):
		# Reset encoders and heading at the start of teleop
		self.left_encoder.reset()
		self.right_encoder.reset()
		self.current_heading = 0

	def teleopPeriodic(self):
		# Handle button presses
		self.handleJoystick()

		# Continue rotation if in progress
		if self.rotating:
			self.rotating = self.rotate()

	def handleJoystick(self):
		"""
		Checks joystick button presses and sets the target rotation.
		"""
		if not self.rotating:  # Only accept new commands if not currently rotating
			if self.DriveJoystick.getRawButton(3):  # X button
				self.setTargetRotation(90)
			elif self.DriveJoystick.getRawButton(1):  # A button
				self.setTargetRotation(180)
			elif self.DriveJoystick.getRawButton(2):  # B button
				self.setTargetRotation(270)
			elif self.DriveJoystick.getRawButton(4):  # Y button
				self.setTargetRotation(360)

	def setTargetRotation(self, target_heading):
		"""
		Sets the target heading for the robot and initializes rotation.
		"""
		# Calculate the rotation distance needed to rotate to the target heading
		rotation_angle = (target_heading - self.current_heading) % 360
		self.target_rotation_distance = (rotation_angle / 360) * self.ROBOT_CIRCUMFERENCE_MM

		# Reset encoders and start rotating
		self.left_encoder.reset()
		self.right_encoder.reset()
		self.rotating = True
		self.current_heading = target_heading

	def rotate(self):
		"""
		Rotates the robot by driving all motors in the same direction.
		Returns True if still rotating, False if the target rotation is complete.
		"""
		# Calculate the average rotation distance
		left_distance = abs(self.left_encoder.getDistance())
		right_distance = abs(self.right_encoder.getDistance())
		average_distance = (left_distance + right_distance) / 2

		# Print encoder debug info
		print(f"Current Heading: {self.current_heading}°, Left: {left_distance:.2f} mm, Right: {right_distance:.2f} mm, Average: {average_distance:.2f} mm")

		# Check if rotation is complete
		if average_distance < self.target_rotation_distance:
			# Rotate the robot (all motors in the same direction for left rotation)
			speed = 0.3  # Adjust speed as needed
			self.LeftFrontMotor.set(-speed)
			self.LeftRearMotor.set(-speed)
			self.RightFrontMotor.set(-speed)
			self.RightRearMotor.set(-speed)
			return True
		else:
			# Stop the motors
			self.LeftFrontMotor.set(0)
			self.LeftRearMotor.set(0)
			self.RightFrontMotor.set(0)
			self.RightRearMotor.set(0)
			return False

if __name__ == "__main__":
	wpilib.run(MyRobot)
