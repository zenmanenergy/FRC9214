import wpilib
from phoenix5 import WPI_TalonSRX, NeutralMode

class MyRobot(wpilib.TimedRobot):
	def robotInit(self):
		# Joystick and motor setup
		self.joystick = wpilib.Joystick(0)
		self.leftFront = WPI_TalonSRX(1)
		self.leftRear = WPI_TalonSRX(4)
		self.rightFront = WPI_TalonSRX(3)
		self.rightRear = WPI_TalonSRX(2)

		# Set all motors to brake mode
		self.leftFront.setNeutralMode(NeutralMode.Brake)
		self.leftRear.setNeutralMode(NeutralMode.Brake)
		self.rightFront.setNeutralMode(NeutralMode.Brake)
		self.rightRear.setNeutralMode(NeutralMode.Brake)

		# Encoder configuration (getDistance() returns millimeters)
		self.WHEEL_DIAMETER_MM = 152.4
		self.WHEEL_CIRCUMFERENCE_MM = self.WHEEL_DIAMETER_MM * 3.14159
		self.ENCODER_CPR = 2048

		self.leftEncoder = wpilib.Encoder(1, 2)
		self.rightEncoder = wpilib.Encoder(3, 4)
		distPerPulse = self.WHEEL_CIRCUMFERENCE_MM / self.ENCODER_CPR
		self.leftEncoder.setDistancePerPulse(distPerPulse)
		self.rightEncoder.setDistancePerPulse(distPerPulse)

		# Travel state
		self.targetDistance = 0
		self.traveling = False

	def teleopInit(self):
		# Reset encoders and travel state at the start of teleop
		self.leftEncoder.reset()
		self.rightEncoder.reset()
		self.traveling = False

	def teleopPeriodic(self):
		# If a travel command is not active, check for button presses
		if not self.traveling:
			# Y button drives forward; A button drives backward
			if self.joystick.getRawButton(4):
				self.startTravel(4000)
			elif self.joystick.getRawButton(1):
				self.startTravel(-4000)

		# If a travel operation is in progress, update it
		if self.traveling:
			self.traveling = self.travelDistance(self.targetDistance)

	def startTravel(self, distance):
		"""
		Resets encoders and sets the target distance.
		"""
		self.leftEncoder.reset()
		self.rightEncoder.reset()
		self.targetDistance = distance
		self.traveling = True

	def setMotors(self, leftSpeed, rightSpeed):
		"""
		Sets the motor speeds.
		"""
		self.leftFront.set(leftSpeed)
		self.leftRear.set(leftSpeed)
		self.rightFront.set(rightSpeed)
		self.rightRear.set(rightSpeed)

	def travelDistance(self, distance):
		"""
		Drives the robot until the average encoder distance meets the target.
		A simple proportional correction adjusts for differences between sides.
		Returns True if still traveling; False when the target is reached.
		"""
		# Determine travel direction and base speed
		direction = 1 if distance > 0 else -1
		baseSpeed = 0.4 * direction

		# Get distances (absolute values)
		leftDist = abs(self.leftEncoder.getDistance())
		rightDist = abs(self.rightEncoder.getDistance())
		avgDist = (leftDist + rightDist) / 2

		# Simple correction: adjust speeds based on the difference
		diff = self.leftEncoder.getDistance() - self.rightEncoder.getDistance()
		correction = 0.001 * diff  # tweak this constant as necessary
		leftSpeed = baseSpeed - correction
		rightSpeed = baseSpeed + correction

		# Continue moving if target not yet reached
		if avgDist < abs(distance):
			# Note: right motors are inverted, so we send the negative of rightSpeed
			self.setMotors(leftSpeed, -rightSpeed)
			return True
		else:
			self.setMotors(0, 0)
			return False

if __name__ == "__main__":
	wpilib.run(MyRobot)
