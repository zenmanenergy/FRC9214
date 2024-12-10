class MyRobot(wpilib.TimedRobot):
	def robotInit(self):
		# Initialization code (same as before)
		self.DriveJoystick = wpilib.Joystick(0)  # Joystick port 0
		self.LeftFrontMotor = ctre.WPI_TalonSRX(1)
		self.LeftRearMotor = ctre.WPI_TalonSRX(2)
		self.RightFrontMotor = ctre.WPI_TalonSRX(3)
		self.RightRearMotor = ctre.WPI_TalonSRX(4)

		self.WHEEL_DIAMETER_MM = 152.4  # mm
		self.WHEEL_CIRCUMFERENCE_MM = self.WHEEL_DIAMETER_MM * math.pi
		self.ENCODER_CPR = 2048

		self.ROBOT_WIDTH_MM = 508
		self.ROBOT_CIRCUMFERENCE_MM = self.ROBOT_WIDTH_MM * math.pi

		self.left_encoder = wpilib.Encoder(1, 2)
		self.right_encoder = wpilib.Encoder(3, 4)
		self.left_encoder.setReverseDirection(True)

		self.left_encoder.setDistancePerPulse(self.WHEEL_CIRCUMFERENCE_MM / self.ENCODER_CPR)
		self.right_encoder.setDistancePerPulse(self.WHEEL_CIRCUMFERENCE_MM / self.ENCODER_CPR)

		# State variables for rotation
		self.rotation_in_progress = False
		self.rotation_target_distance = 0.0

	def teleopInit(self):
		self.left_encoder.reset()
		self.right_encoder.reset()
		self.rotation_in_progress = False

	def teleopPeriodic(self):
		self.JoystickPeriodic()

		# Handle rotation
		self.handleRotation()

	def JoystickPeriodic(self):
		self.DRIVE_BUTTON_A = self.DriveJoystick.getRawButton(1)
		self.DRIVE_BUTTON_B = self.DriveJoystick.getRawButton(2)
		self.DRIVE_BUTTON_X = self.DriveJoystick.getRawButton(3)
		self.DRIVE_BUTTON_Y = self.DriveJoystick.getRawButton(4)

		# Trigger rotation
		if self.DRIVE_BUTTON_Y:
			self.startRotation(0)
		elif self.DRIVE_BUTTON_X:
			self.startRotation(90)
		elif self.DRIVE_BUTTON_A:
			self.startRotation(180)
		elif self.DRIVE_BUTTON_B:
			self.startRotation(270)

	def startRotation(self, degrees):
		"""
		Starts a rotation by setting target distances and enabling the state.
		"""
		print(f"Starting rotation: {degrees} degrees")
		self.rotation_target_distance = (degrees / 360.0) * self.ROBOT_CIRCUMFERENCE_MM
		self.left_encoder.reset()
		self.right_encoder.reset()
		self.rotation_in_progress = True

	def handleRotation(self):
		"""
		Handles ongoing rotation logic, making it non-blocking.
		"""
		if not self.rotation_in_progress:
			return

		# Check distances
		left_distance = abs(self.left_encoder.getDistance())
		right_distance = abs(self.right_encoder.getDistance())
		print(f"Left: {left_distance:.2f} mm, Right: {right_distance:.2f} mm")

		if left_distance < self.rotation_target_distance or right_distance < self.rotation_target_distance:
			# Continue rotating
			self.LeftFrontMotor.set(-0.5)  # Left motors backward
			self.LeftRearMotor.set(-0.5)
			self.RightFrontMotor.set(-0.5)  # Right motors forward
			self.RightRearMotor.set(-0.5)
		else:
			# Stop rotation
			self.LeftFrontMotor.set(0)
			self.LeftRearMotor.set(0)
			self.RightFrontMotor.set(0)
			self.RightRearMotor.set(0)
			self.rotation_in_progress = False
			print("Rotation complete")
