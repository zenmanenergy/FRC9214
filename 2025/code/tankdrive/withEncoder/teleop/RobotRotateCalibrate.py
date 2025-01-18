import wpilib
import ctre
from ctre import NeutralMode
import csv
import json
from datetime import datetime
from adi import ADIS16470_IMU  # Replace with ADIS16448_IMU if you're using that

class MyRobot(wpilib.TimedRobot):
	def robotInit(self):
		# Joystick and motor setup
		self.DriveJoystick = wpilib.Joystick(0)  # Joystick port 0
		self.LeftFrontMotor = ctre.WPI_TalonSRX(1)
		self.LeftRearMotor = ctre.WPI_TalonSRX(2)
		self.RightFrontMotor = ctre.WPI_TalonSRX(3)
		self.RightRearMotor = ctre.WPI_TalonSRX(4)

		# Invert right-side motors to ensure positive speeds move forward
		self.LeftFrontMotor.setInverted(False)
		self.LeftRearMotor.setInverted(False)
		self.RightFrontMotor.setInverted(True)
		self.RightRearMotor.setInverted(True)

		# Set motors to brake mode
		self.LeftFrontMotor.setNeutralMode(NeutralMode.Brake)
		self.LeftRearMotor.setNeutralMode(NeutralMode.Brake)
		self.RightFrontMotor.setNeutralMode(NeutralMode.Brake)
		self.RightRearMotor.setNeutralMode(NeutralMode.Brake)

		# Encoder setup
		self.WHEEL_DIAMETER_MM = 152.4  # mm
		self.WHEEL_CIRCUMFERENCE_MM = self.WHEEL_DIAMETER_MM * 3.141592653589793
		self.ENCODER_CPR = 2048  # Counts per revolution for the encoder

		# Initialize encoders
		self.left_encoder = wpilib.Encoder(1, 2)  # Left encoder on DIO 1, 2
		self.right_encoder = wpilib.Encoder(3, 4)  # Right encoder on DIO 3, 4

		# Reverse direction of the right encoder
		self.left_encoder.setReverseDirection(False)
		self.right_encoder.setReverseDirection(True)

		# Set distance per pulse for encoders
		self.left_encoder.setDistancePerPulse(self.WHEEL_CIRCUMFERENCE_MM / self.ENCODER_CPR)
		self.right_encoder.setDistancePerPulse(self.WHEEL_CIRCUMFERENCE_MM / self.ENCODER_CPR)

		# IMU setup (ADIS16470 or ADIS16448)
		self.imu = ADIS16470_IMU()  # Initialize the IMU
		self.imu.calibrate()  # Ensure the IMU is calibrated
		self.imu.reset()  # Reset yaw angle to zero

		# Load weights from the log file or set defaults
		self.loadWeights()

		# Learning rate and drift thresholds
		self.alpha = 0.05  # Small adjustments per run
		self.acceptable_drift_mm = 20
		self.acceptable_imu_drift_deg = 2.0

		# Travel state
		self.target_angle = None
		self.travel_in_progress = False

	def teleopInit(self):
		# Reset encoder and IMU data at the start of teleop
		self.left_encoder.reset()
		self.right_encoder.reset()
		self.imu.reset()

	def teleopPeriodic(self):
		# Periodic joystick and driving updates
		self.checkForRotation()

	def checkForRotation(self):
		"""
		Checks for button presses and initiates rotation operations.
		"""
		if self.DriveJoystick.getRawButton(4):  # Y button
			self.target_angle = 180  # Rotate 180 degrees clockwise
			self.startRotation()
		elif self.DriveJoystick.getRawButton(1):  # A button
			self.target_angle = -180  # Rotate 180 degrees counterclockwise
			self.startRotation()

		# Continue rotation if in progress
		if self.travel_in_progress:
			self.travel_in_progress = self.rotateToAngle(self.target_angle)

	def startRotation(self):
		"""
		Initializes rotation operation by resetting encoders and IMU.
		"""
		self.left_encoder.reset()
		self.right_encoder.reset()
		self.imu.reset()
		self.travel_in_progress = True

	def rotateToAngle(self, angle_deg):
		"""
		Rotates the robot to a specified angle in degrees at a constant speed,
		using encoder and IMU feedback for real-time corrections.
		"""
		# Determine direction and speed
		direction = 1 if angle_deg > 0 else -1
		target_angle = abs(angle_deg)
		speed = 0.3  # Base rotation speed

		# Get sensor data
		left_distance = self.left_encoder.getDistance()
		right_distance = self.right_encoder.getDistance()
		encoder_error = left_distance - right_distance
		imu_angle = self.imu.getAngle()  # Get yaw angle in degrees

		# Calculate correction
		correction = self.calculateCorrection(encoder_error, imu_angle, "rotation")

		# Adjust motor speeds for rotation
		left_speed = speed * direction - correction
		right_speed = -speed * direction + correction

		# Apply motor commands
		self.LeftFrontMotor.set(left_speed)
		self.LeftRearMotor.set(left_speed)
		self.RightFrontMotor.set(right_speed)
		self.RightRearMotor.set(right_speed)

		# Check if target angle is reached
		if abs(imu_angle) >= target_angle:
			# Stop the motors
			self.LeftFrontMotor.set(0)
			self.LeftRearMotor.set(0)
			self.RightFrontMotor.set(0)
			self.RightRearMotor.set(0)

			# Log data
			self.logRunData(left_distance, right_distance, imu_angle, "rotation")
			return False
		return True

	def logRunData(self, left_distance, right_distance, imu_drift, movement_type):
		"""
		Logs encoder values, IMU drift, and current weights to a CSV file.
		Includes movement type for differentiation.
		"""
		with open("drift_log.csv", "a", newline="") as file:
			writer = csv.writer(file)
			timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

			# Prompt user to input manual drift (physical measurement)
			try:
				manual_drift = float(input(f"Enter the measured drift in mm/degrees (positive = right, negative = left): "))
			except ValueError:
				manual_drift = 0  # Default to 0 if no input is provided
				print("Invalid input. Using 0 for manual drift.")

			# Append all data to the log file, including current weights and movement type
			writer.writerow([
				timestamp,
				left_distance,
				right_distance,
				imu_drift,
				manual_drift,  # Log the manual drift
				self.weights_straight["encoder"] if movement_type == "straight" else self.weights_rotate["encoder"],
				self.weights_straight["imu"] if movement_type == "straight" else self.weights_rotate["imu"],
				movement_type  # Log whether this is straight driving or rotation
			])

		# Adjust weights based on manual drift and IMU drift
		self.adjustWeights(manual_drift, imu_drift, movement_type)

	def calculateCorrection(self, encoder_error, imu_drift, movement_type):
		"""
		Calculates the correction factor using dynamic weights for encoder and IMU data.
		"""
		weights = self.weights_straight if movement_type == "straight" else self.weights_rotate

		# Calculate individual corrections
		encoder_correction = encoder_error * weights["encoder"]
		imu_correction = imu_drift * weights["imu"]

		# Combine corrections
		total_correction = encoder_correction + imu_correction

		# Bound the correction factor
		total_correction = max(min(total_correction, 1), -1)
		return total_correction

	def adjustWeights(self, manual_drift, imu_drift, movement_type):
		"""
		Adjusts weights separately for straight driving or rotation based on logged data.
		"""
		weights = self.weights_straight if movement_type == "straight" else self.weights_rotate

		# Check encoder performance
		if abs(manual_drift) > self.acceptable_drift_mm:
			weights["encoder"] -= self.alpha
			weights["imu"] += self.alpha

		# Check IMU performance
		if abs(imu_drift) > self.acceptable_imu_drift_deg:
			weights["imu"] -= self.alpha
			weights["encoder"] += self.alpha

		# Normalize weights
		total_weight = weights["encoder"] + weights["imu"]
		weights["encoder"] /= total_weight
		weights["imu"] /= total_weight

		# Save updated weights
		self.saveWeights(movement_type)

	def saveWeights(self, movement_type):
		"""
		Saves the current weights for straight or rotational movement to a JSON file.
		"""
		data = {
			"straight": self.weights_straight,
			"rotation": self.weights_rotate
		}
		with open("weights.json", "w") as file:
			json.dump(data, file)

	def loadWeights(self):
		"""
		Loads weights for straight driving and rotation from a JSON file or sets defaults.
		"""
		try:
			with open("weights.json", "r") as file:
				data = json.load(file)
				self.weights_straight = data.get("straight", {"encoder": 0.7, "imu": 0.3})
				self.weights_rotate = data.get("rotation", {"encoder": 0.5, "imu": 0.5})
		except FileNotFoundError:
			# Default weights if the file doesn't exist
			self.weights_straight = {"encoder": 0.7, "imu": 0.3}
			self.weights_rotate = {"encoder": 0.5, "imu": 0.5}


if __name__ == "__main__":
	wpilib.run(MyRobot)
