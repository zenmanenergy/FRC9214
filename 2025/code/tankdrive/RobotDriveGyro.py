import wpilib
import ctre
import math

class MyRobot(wpilib.TimedRobot):
	def robotInit(self):
		self.DriveJoystick = wpilib.Joystick(0)

		self.LeftFrontMotor = ctre.WPI_TalonSRX(1)
		self.LeftRearMotor = ctre.WPI_TalonSRX(2)

		self.RightFrontMotor = ctre.WPI_TalonSRX(3)
		self.RightRearMotor = ctre.WPI_TalonSRX(4)

		# Initialize the ADIS16470 sensor
		self.adis = wpilib.ADIS16470_IMU()
		self.adis.calibrate()

		# Initialize variables for sensor fusion
		self.angle = 0.0
		self.target_angle = None
		self.last_timestamp = wpilib.Timer.getFPGATimestamp()

		# Gyro drift correction variable
		self.gyro_drift = 0.0

		# Calibrate the drift value once during initialization
		self.CalibrateGyroDrift()

	def autonomousInit(self):
		return False

	def autonomousPeriodic(self):
		return False

	def teleopInit(self):
		# Reset target angle
		self.target_angle = None

	def teleopPeriodic(self):
		self.JoystickPeriodic()
		self.UpdateSensorFusion()
		self.HandleButtonPresses()
		self.DrivePeriodic()
		return False

	def JoystickPeriodic(self):
		# Apply a deadband to ignore small joystick drift
		def deadband(value, threshold=0.05):
			return value if abs(value) > threshold else 0.0

		# Joystick axis values with deadband applied
		self.DRIVE_LEFT_THUMB_UPDOWN = deadband(self.DriveJoystick.getRawAxis(1))
		self.DRIVE_RIGHT_THUMB_UPDOWN = deadband(self.DriveJoystick.getRawAxis(5))

		# Update button states
		self.button_rb = self.DriveJoystick.getRawButton(6)  # RB button
		self.button_start = self.DriveJoystick.getRawButton(8)  # Start button
		self.button_a = self.DriveJoystick.getRawButton(1)  # A button
		self.button_b = self.DriveJoystick.getRawButton(2)  # B button
		self.button_x = self.DriveJoystick.getRawButton(3)  # X button
		self.button_y = self.DriveJoystick.getRawButton(4)  # Y button

	def DrivePeriodic(self):
		left_speed = -self.DRIVE_LEFT_THUMB_UPDOWN
		right_speed = -self.DRIVE_RIGHT_THUMB_UPDOWN

		# Debug: Print joystick speeds
		print(f"Joystick left_speed: {left_speed}, right_speed: {right_speed}")

		# If the joystick is not being used and there's a target angle, override the control
		if (self.DRIVE_LEFT_THUMB_UPDOWN < 0.05 and self.DRIVE_LEFT_THUMB_UPDOWN > -0.05) and (self.DRIVE_RIGHT_THUMB_UPDOWN < 0.05 and self.DRIVE_RIGHT_THUMB_UPDOWN > -0.05) and self.target_angle is not None:
			# Determine the shortest rotation direction
			error = (self.target_angle - self.angle + 360) % 360
			if error > 180:
				error -= 360  # Rotate in the shortest direction

			# Debug: Print error and angle information
			print(f"Current Angle: {self.angle}, Target Angle: {self.target_angle}, Error: {error}")

			# If the error is large enough, rotate in the direction of the error
			if abs(error) > 2.0:  # Tolerance of 2 degrees to stop rotation
				rotation_speed = 0.2 * math.copysign(1, error)  # Rotate left or right
				left_speed = -rotation_speed
				right_speed = rotation_speed

				# Debug: Print rotation speeds
				print(f"Rotating - left_speed: {left_speed}, right_speed: {right_speed}")
			else:
				# Stop the motors when the target angle is reached
				left_speed = 0.0
				right_speed = 0.0
				self.target_angle = None  # Stop turning
				print("Target angle reached. Stopping rotation.")

		# Set motor speeds based on joystick input or turning logic
		self.LeftFrontMotor.set(left_speed)
		self.LeftRearMotor.set(left_speed)
		self.RightFrontMotor.set(-right_speed)
		self.RightRearMotor.set(-right_speed)

	def UpdateSensorFusion(self):
		try:
			# Get the current gyro rate from the IMU
			gyro_rate = self.adis.getRate() - self.gyro_drift  # Correct for drift
			# Update the angle based on gyro rate
			current_timestamp = wpilib.Timer.getFPGATimestamp()
			dt = current_timestamp - self.last_timestamp
			self.last_timestamp = current_timestamp

			# Update the angle based on gyro reading and time difference
			self.angle += gyro_rate * dt

			# Normalize the angle to stay within 0 to 360 degrees
			self.angle = self.angle % 360

			# Debug: Print the updated angle and gyro rate
			print(f"Updated Angle: {self.angle}, Gyro Rate: {gyro_rate}")
			
		except Exception as e:
			print(f"Error in UpdateSensorFusion: {e}")

	def HandleButtonPresses(self):
		# If RB button is pressed, print the current angle
		if self.button_rb:
			print(f"Current angle: {self.angle}")

		# If Start button is pressed, reset the angle to 0
		if self.button_start:
			self.angle = 0.0
			print("Angle reset to 0")

		# If Y button is pressed, turn to 0 degrees
		if self.button_y:
			self.target_angle = 0.0
			print("Turning to 0 degrees")

		# If X button is pressed, turn to 90 degrees
		if self.button_x:
			self.target_angle = 90.0
			print("Turning to 90 degrees")

		# If A button is pressed, turn to 180 degrees
		if self.button_a:
			self.target_angle = 180.0
			print("Turning to 180 degrees")

		# If B button is pressed, turn to 270 degrees
		if self.button_b:
			self.target_angle = 270.0
			print("Turning to 270 degrees")

	def CalibrateGyroDrift(self):
		# Create a timer object
		timer = wpilib.Timer()
		timer.start()

		gyro_readings = []
		for _ in range(100):
			gyro_readings.append(self.adis.getRate())

			# Wait for 10 ms
			timer.reset()
			while timer.get() < 0.01:
				pass

		self.gyro_drift = sum(gyro_readings) / len(gyro_readings)
		print(f"Calibrated gyro drift: {self.gyro_drift}")

if __name__ == "__main__":
	wpilib.run(MyRobot)
