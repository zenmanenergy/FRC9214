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
		self.target_distance = None
		self.travel_in_progress = False
		self.offset = 600

	def teleopInit(self):
		# Reset encoder and IMU data at the start of teleop
		self.left_encoder.reset()
		self.right_encoder.reset()
		self.imu.reset()

	def teleopPeriodic(self):
		# Periodic joystick and driving updates
		self.checkForTravel()

	def checkForTravel(self):
		"""
		Checks for button presses and initiates travel operations.
		"""
		if self.DriveJoystick.getRawButton(4):  # Y button
			self.target_distance = 3000  # Travel 3000 mm forward
			self.startTravel()
		elif self.DriveJoystick.getRawButton(1):  # A button
			self.target_distance = -3000  # Travel 3000 mm backward
			self.startTravel()

		# Continue travel if in progress
		if self.travel_in_progress:
			self.travel_in_progress = self.travelDistance(self.target_distance)

	def startTravel(self):
		"""
		Initializes travel operation by resetting encoders and IMU.
		"""
		self.left_encoder.reset()
		self.right_encoder.reset()
		self.imu.reset()
		self.travel_in_progress = True

	def travelDistance(self, distance_mm):
		"""
		Makes the robot travel a specified distance in millimeters at a constant speed,
		using encoder and IMU feedback for real-time corrections.
		"""
		# Determine direction and speed
		direction = 1 if distance_mm > 0 else -1
		target_distance = abs(distance_mm) - self.offset
		speed = 0.4  # Base travel speed

		# Get sensor data
		left_distance = self.left_encoder.getDistance()
		right_distance = self.right_encoder.getDistance()
		encoder_error = left_distance - right_distance
		imu_drift = self.imu.getAngle()  # Get yaw angle in degrees

		# Calculate correction
		correction = self.calculateCorrection(encoder_error, imu_drift)

		# Adjust motor speeds
		left_speed = speed - correction
		right_speed = speed + correction

		# Apply motor commands
		self.LeftFrontMotor.set(direction * left_speed)
		self.LeftRearMotor.set(direction * left_speed)
		self.RightFrontMotor.set(direction * right_speed)
		self.RightRearMotor.set(direction * right_speed)

		# Check if target distance is reached
		average_distance = (abs(left_distance) + abs(right_distance)) / 2
		if average_distance >= target_distance:
			# Stop the motors
			self.LeftFrontMotor.set(0)
			self.LeftRearMotor.set(0)
			self.RightFrontMotor.set(0)
			self.RightRearMotor.set(0)

			# Log data
			self.logRunData(left_distance, right_distance, imu_drift)
			return False
		return True

	def logRunData(self, left_distance, right_distance, imu_drift):
		"""
		Logs encoder values, IMU drift, and current weights to a CSV file.
		Prompts the user to input the manual drift measurement.
		"""
		with open("drift_log.csv", "a", newline="") as file:
			writer = csv.writer(file)
			timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

			# Calculate encoder-based drift for reference
			encoder_drift = left_distance - right_distance

			# Prompt user to input manual drift (physical measurement)
			try:
				manual_drift = float(input(f"Enter the measured drift in mm (positive = right, negative = left): "))
			except ValueError:
				manual_drift = 0  # Default to 0 if no input is provided
				print("Invalid input. Using 0 for manual drift.")

			# Append all data to the log file, including current weights
			writer.writerow([
				timestamp,
				left_distance,
				right_distance,
				imu_drift,
				manual_drift,  # Log the manual drift
				self.weights["encoder"],
				self.weights["imu"]
			])

		print(f"Logged: {timestamp}, Left={left_distance:.2f}, Right={right_distance:.2f}, "
				f"IMU Drift={imu_drift:.2f}, Manual Drift={manual_drift:.2f}, "
				f"Encoder Weight={self.weights['encoder']:.4f}, IMU Weight={self.weights['imu']:.4f}")

		# Adjust weights based on manual drift and IMU drift
		self.adjustWeights(manual_drift, imu_drift)

	def calculateCorrection(self, encoder_error, imu_drift):
		"""
		Calculates the correction factor using dynamic weights for encoder and IMU data.
		"""
		# Calculate individual corrections
		encoder_correction = encoder_error * self.weights["encoder"]
		imu_correction = imu_drift * self.weights["imu"]

		# Combine corrections
		total_correction = encoder_correction + imu_correction

		# Bound the correction factor
		total_correction = max(min(total_correction, 1), -1)

		# Log corrections for debugging
		print(f"Encoder Correction: {encoder_correction:.4f}, IMU Correction: {imu_correction:.4f}")
		return total_correction

	def adjustWeights(self, manual_drift_mm, logged_imu_drift_deg):
		"""
		Automatically adjusts weights based on logged manual drift and IMU drift data.
		"""
		# Check encoder performance
		if abs(manual_drift_mm) > self.acceptable_drift_mm:
			# Drift too high -> reduce encoder weight slightly
			self.weights["encoder"] -= self.alpha
			self.weights["imu"] += self.alpha
			print(f"Adjusting weights: Encoder underperforming, reducing encoder weight.")

		# Check IMU performance
		if abs(logged_imu_drift_deg) > self.acceptable_imu_drift_deg:
			# IMU drift too high -> reduce IMU weight slightly
			self.weights["imu"] -= self.alpha
			self.weights["encoder"] += self.alpha
			print(f"Adjusting weights: IMU underperforming, reducing IMU weight.")

		# Ensure weights stay within [0, 1] and sum to 1.0
		self.weights["encoder"] = max(min(self.weights["encoder"], 1.0), 0.0)
		self.weights["imu"] = max(min(self.weights["imu"], 1.0), 0.0)

		# Normalize weights to maintain a total of 1.0
		total_weight = self.weights["encoder"] + self.weights["imu"]
		self.weights["encoder"] /= total_weight
		self.weights["imu"] /= total_weight

		# Save updated weights
		self.saveWeights()

		# Log adjusted weights
		print(f"New Weights: Encoder={self.weights['encoder']:.4f}, IMU={self.weights['imu']:.4f}")

	def saveWeights(self):
		"""
		Saves the current weights to a file.
		"""
		weights_data = {
			"encoder": self.weights["encoder"],
			"imu": self.weights["imu"]
		}
		with open("weights.json", "w") as file:
			json.dump(weights_data, file)
		print(f"Saved weights: {weights_data}")

	def loadWeights(self):
		"""
		Loads the most recent weights from the log file, or sets default weights if no log exists.
		"""
		try:
			with open("drift_log.csv", "r") as file:
				reader = csv.reader(file)
				last_row = None

				# Iterate through the log to get the last row
				for row in reader:
					last_row = row

				if last_row and len(last_row) >= 6:
					# Extract weights from the last row of the log
					self.weights = {
						"encoder": float(last_row[4]),  # 5th column is encoder weight
						"imu": float(last_row[5])      # 6th column is IMU weight
					}
					print(f"Loaded weights from log: {self.weights}")
				else:
					# No valid weight data found; set default weights
					self.weights = {"encoder": 0.7, "imu": 0.3}
					print("No valid weights in log. Using default weights.")
		except FileNotFoundError:
			# Log file not found; set default weights
			self.weights = {"encoder": 0.7, "imu": 0.3}
			print("Log file not found. Using default weights.")


if __name__ == "__main__":
	wpilib.run(MyRobot)
