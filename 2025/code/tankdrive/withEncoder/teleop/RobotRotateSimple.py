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
		self.ROBOT_WIDTH_MM = 508  # Distance between wheels in mm
		self.ROBOT_CIRCUMFERENCE_MM = self.ROBOT_WIDTH_MM * 3.141592653589793

		self.left_encoder = wpilib.Encoder(1, 2)
		self.right_encoder = wpilib.Encoder(3, 4)
		self.left_encoder.setDistancePerPulse(self.ROBOT_CIRCUMFERENCE_MM / 2048)
		self.right_encoder.setDistancePerPulse(self.ROBOT_CIRCUMFERENCE_MM / 2048)

		# Rotation state
		self.rotating = False
		self.target_distance = 0

	def teleopInit(self):
		# Reset encoders
		self.left_encoder.reset()
		self.right_encoder.reset()

	def teleopPeriodic(self):
		# Button handling
		if not self.rotating:
			if self.DriveJoystick.getRawButton(3):  # X button
				self.startRotation(90)
			elif self.DriveJoystick.getRawButton(1):  # A button
				self.startRotation(180)
			elif self.DriveJoystick.getRawButton(2):  # B button
				self.startRotation(270)
			elif self.DriveJoystick.getRawButton(4):  # Y button
				self.startRotation(360)

		# Continue rotation if started
		if self.rotating:
			self.rotating = self.performRotation()

	def startRotation(self, angle):
		"""
		Sets up the robot to rotate to the left by the specified angle in degrees.
		"""
		self.target_distance = (angle / 360) * self.ROBOT_CIRCUMFERENCE_MM
		self.left_encoder.reset()
		self.right_encoder.reset()
		self.rotating = True
		print(f"Starting rotation for {angle}°.")

	def performRotation(self):
		"""
		Rotates the robot left until the target distance is reached.
		"""
		# Get encoder distances
		left_distance = abs(self.left_encoder.getDistance())
		right_distance = abs(self.right_encoder.getDistance())
		average_distance = (left_distance + right_distance) / 2

		print(f"{left_distance} {right_distance}")

		# Print debug info
		print(f"Target: {self.target_distance:.2f} mm, Current: {average_distance:.2f} mm")

		# Check if the robot has rotated enough
		if average_distance < self.target_distance:
			# Rotate the robot
			speed = 0.3
			self.LeftFrontMotor.set(-speed)
			self.LeftRearMotor.set(-speed)
			self.RightFrontMotor.set(-speed)
			self.RightRearMotor.set(-speed)
			return True
		else:
			# Stop the robot
			self.LeftFrontMotor.set(0)
			self.LeftRearMotor.set(0)
			self.RightFrontMotor.set(0)
			self.RightRearMotor.set(0)
			print("Rotation complete.")
			return False

if __name__ == "__main__":
	wpilib.run(MyRobot)
