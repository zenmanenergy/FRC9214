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

		# Reverse one encoder to reflect directionality
		self.left_encoder.setReverseDirection(True)  # Reverse left encoder

		# Rotation state
		self.rotating = False
		self.target_distance = 0
		self.RightSpeedFactor = 1
		self.LeftSpeedFactor = 1

	def teleopInit(self):
		# Reset encoders
		self.left_encoder.reset()
		self.right_encoder.reset()
		print("Teleop initialized: Encoders reset.")

	def teleopPeriodic(self):
		# Button handling
		if not self.rotating:
			if self.DriveJoystick.getRawButton(3):  # X button
				self.startRotation(270)
			elif self.DriveJoystick.getRawButton(1):  # A button
				self.startRotation(180)
			elif self.DriveJoystick.getRawButton(2):  # B button
				self.startRotation(90)
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
		self.target_distance = self.target_distance * 2.30
		self.left_encoder.reset()
		self.right_encoder.reset()
		self.rotating = True
		self.RightSpeedFactor = 1
		self.LeftSpeedFactor = 1
		print(f"Starting rotation for {angle}°. Target distance: {self.target_distance:.2f} mm")

	def performRotation(self):
		"""
		Rotates the robot left until both sides individually reach the target distance,
		with overshoot correction using reverse motors.
		"""
		# Get encoder distances
		left_distance = abs(self.left_encoder.getDistance())
		right_distance = abs(self.right_encoder.getDistance())

		# Calculate error and correction for speed balancing
		error = left_distance - right_distance
		self.RightSpeedFactor += abs(error) / 10000 if left_distance < right_distance else -abs(error) / 10000
		self.LeftSpeedFactor += abs(error) / 10000 if left_distance > right_distance else -abs(error) / 10000

		# Calculate remaining distances for each side
		left_remaining = self.target_distance - left_distance
		right_remaining = self.target_distance - right_distance

		# Print debug information
		print(f"Target: {self.target_distance:.2f} mm")
		print(f"Left: {left_distance:.2f} mm | Remaining: {left_remaining:.2f} mm")
		print(f"Right: {right_distance:.2f} mm | Remaining: {right_remaining:.2f} mm")
		print(f"Correction: {error:.5f} | LeftFactor: {self.LeftSpeedFactor:.3f} | RightFactor: {self.RightSpeedFactor:.3f}")

		# Determine speed for the left side
		if left_remaining > 100:  # Far from target, fast speed
			left_speed = 0.3
		elif left_remaining > 50:  # Slow down zone
			left_speed = 0.2
		elif left_remaining > 10:  # Crawl mode
			left_speed = 0.2
		elif left_remaining < -10:  # Overshot—reverse
			left_speed = -0.2
		elif abs(left_remaining) <= 10:  # Stop threshold
			left_speed = 0
		else:
			left_speed = 0  # Default crawl mode for safety

		# Determine speed for the right side
		if right_remaining > 100:  # Far from target, fast speed
			right_speed = 0.3
		elif right_remaining > 50:  # Slow down zone
			right_speed = 0.2
		elif right_remaining > 10:  # Crawl mode
			right_speed = 0.2
		elif right_remaining < -10:  # Overshot—reverse
			right_speed = -0.2
		elif abs(right_remaining) <= 10:  # Stop threshold
			right_speed = 0
		else:
			right_speed = 0  # Default crawl mode for safety

		# Apply motor speeds with corrections
		self.LeftFrontMotor.set(left_speed * self.LeftSpeedFactor)
		self.LeftRearMotor.set(left_speed * self.LeftSpeedFactor)
		self.RightFrontMotor.set(right_speed * self.RightSpeedFactor)
		self.RightRearMotor.set(right_speed * self.RightSpeedFactor)

		# Check if both sides have stopped
		left_done = abs(left_remaining) <= 10  # Stop threshold
		right_done = abs(right_remaining) <= 10  # Stop threshold

		if left_done and right_done:
			# Both sides have stopped—end rotation
			print("Rotation complete. Stopping motors.")
			self.LeftFrontMotor.set(0)
			self.LeftRearMotor.set(0)
			self.RightFrontMotor.set(0)
			self.RightRearMotor.set(0)
			return False  # Done rotating
		else:
			# Continue rotating
			return True




if __name__ == "__main__":
	wpilib.run(MyRobot)
