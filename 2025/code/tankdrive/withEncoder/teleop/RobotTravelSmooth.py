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

		# Travel parameters
		self.max_speed = 0.3  # Maximum motor speed
		self.start_speed = 0.2
		speed = 0  # Minimum motor speed for smooth startup/stop
		self.accel_distance = 3000 *self.max_speed+100  # Distance (mm) over which to accelerate/decelerate
		
		self.brake_speed=-0.2
		self.brake_duration=0.3

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

		# Get current button states
		current_x_button_state = self.DriveJoystick.getRawButton(3)  # X button
		current_b_button_state = self.DriveJoystick.getRawButton(2)  # B button
		current_lb_button_state = self.DriveJoystick.getRawButton(5)  # B button
		current_rb_button_state = self.DriveJoystick.getRawButton(6)  # B button

		# Check for a new press (button transitioned from not pressed to pressed)
		if current_x_button_state and not self.last_x_button_state:
			self.max_speed = max(0.1, self.max_speed - 0.1)  # Prevent negative speeds
			self.accel_distance = 3000 *self.max_speed+100
			print(f"Decreased max speed: {self.max_speed:.2f}")
			print(f"accel_distance: {self.accel_distance:.2f}")
		if current_b_button_state and not self.last_b_button_state:
			self.max_speed = min(1.0, self.max_speed + 0.1)  # Cap at 1.0
			self.accel_distance = 3000 *self.max_speed+100
			print(f"Increased max speed: {self.max_speed:.2f}")
			print(f"accel_distance: {self.accel_distance:.2f}")
		if current_lb_button_state and not self.last_lb_button_state:
			self.accel_distance = self.accel_distance - 100  # Cap at 1.0
			print(f"accel_distance: {self.accel_distance:.2f}")
		if current_rb_button_state and not self.last_rb_button_state:
			self.accel_distance = self.accel_distance +100  # Cap at 1.0
			print(f"accel_distance: {self.accel_distance:.2f}")

		# Update last button states
		self.last_x_button_state = current_x_button_state
		self.last_b_button_state = current_b_button_state
		self.last_lb_button_state = current_lb_button_state
		self.last_rb_button_state = current_rb_button_state

	def checkForTravel(self):
		"""
		Checks for button presses and initiates travel operations.
		"""
		# Start travel if not already in progress
		if self.DRIVE_BUTTON_Y and not self.travel_in_progress:
			self.target_distance = 3000  # Travel 10 mm forward
			self.startTravel()
		elif self.DRIVE_BUTTON_A and not self.travel_in_progress:
			self.target_distance = -3000  # Travel 10 mm backward
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

	def travelDistance(self, distance_mm):
		"""
		Makes the robot travel a specified distance in millimeters at a constant speed.
		Uses two encoders for more accurate distance tracking.
		Returns True if the robot is still moving, False if it has reached the target distance.
		"""
		# Determine the direction of travel
		direction = 1 if distance_mm > 0 else -1
		target_distance = abs(distance_mm)

	
		# self.accel_distance=self.self.accel_distance

		# Calculate the average distance traveled by both encoders
		left_distance = abs(self.left_encoder.getDistance())
		right_distance = abs(self.right_encoder.getDistance())
		average_distance = (left_distance + right_distance) / 2

		# Print encoder values for debugging
		# print(f"Left Distance: {left_distance:.2f} mm, Right Distance: {right_distance:.2f} mm, Average: {average_distance:.2f} mm")
		# Check if the target distance is reached
		if average_distance < target_distance:
			# Calculate the remaining distance
			remaining_distance = target_distance - average_distance

			# Accelerate at the start
			if average_distance < self.accel_distance:
				speed = max(self.start_speed,self.max_speed * (average_distance / self.accel_distance))
			# Adjust speed based on remaining distance (deceleration)
			elif remaining_distance < self.accel_distance:
				speed = self.max_speed * (remaining_distance / self.accel_distance)
			# Maintain maximum speed in the middle
			else:
				speed = self.max_speed

			print(f"avg dist: {average_distance:.2f} mm, speed {speed:.2f}, %: {(remaining_distance / self.accel_distance):.2f}")
		
			# Apply speed to the motors
			self.LeftFrontMotor.set(direction * speed)
			self.LeftRearMotor.set(direction * speed)
			self.RightFrontMotor.set(-1 * direction * speed)
			self.RightRearMotor.set(-1 * direction * speed)
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
