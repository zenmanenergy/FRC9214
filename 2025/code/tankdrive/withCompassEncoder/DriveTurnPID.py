# DriveTurnPID.py
import math
import time
import json
import os
import wpilib
from wpimath.controller import PIDController
from navx import AHRS

class MotorGroup:
	"""
	A simple wrapper for a group of motor controllers.
	Each motor must implement a set(speed) method.
	"""
	def __init__(self, motors):
		self.motors = motors
	
	def set(self, speed):
		for motor in self.motors:
			motor.set(speed)

class DriveTurnPID:
	"""
	DriveTurnPID provides PID control for driving straight a specified distance
	and turning a specified number of degrees. It also includes a simple
	relay-based autotuner (using the Ziegler-Nichols method) as well as the
	ability to save and load PID constants from a JSON file.
	"""
	def __init__(self, leftMotors, rightMotors,
		leftEncoderPorts, rightEncoderPorts,
		wheel_diameter_mm, encoder_cpr,
		navx_type=AHRS.NavXComType.kMXP_SPI, navx_update_rate=AHRS.NavXUpdateRate.k200Hz,
		settings_filename="DriveCompassPIDSettings.json"):
		# Save motor groups
		self.leftMotors = leftMotors
		self.rightMotors = rightMotors
		
		# Create and configure encoders (leftEncoderPorts and rightEncoderPorts are 2-element tuples)
		self.leftEncoder = wpilib.Encoder(leftEncoderPorts[0], leftEncoderPorts[1])
		self.rightEncoder = wpilib.Encoder(rightEncoderPorts[0], rightEncoderPorts[1])
		wheel_circumference = wheel_diameter_mm * math.pi
		distPerPulse = wheel_circumference / encoder_cpr
		self.leftEncoder.setDistancePerPulse(distPerPulse)
		self.rightEncoder.setDistancePerPulse(distPerPulse)
		
		# Create NavX sensor
		self.navx = AHRS(navx_type, navx_update_rate)
		
		# PID controllers – they will be (re)initialized when commands are started.
		self.drivePID = None
		self.turnPID = None
		
		# PID constants (tuples of (P, I, D)). These can be loaded from or saved to a file.
		self.drivePIDConstants = None
		self.turnPIDConstants = None
		self.settings_filename = settings_filename
		self.loadSettings()
		
		# State for drive commands
		self.drive_target_distance = 0
		self.drive_initial_heading = 0
		self.drive_active = False
		
		# State for turn commands
		self.turn_target_degrees = 0
		self.turn_desired_heading = 0
		self.turn_active = False
	
	def setup_drive_pid(self, P, I, D):
		"""Initialize the drive PID controller (for distance control)."""
		self.drivePID = PIDController(P, I, D)
		self.drivePID.setTolerance(10) 	# tolerance in mm
	
	def setup_turn_pid(self, P, I, D):
		"""Initialize the turn PID controller (for angle control)."""
		self.turnPID = PIDController(P, I, D)
		self.turnPID.enableContinuousInput(-180, 180)
		self.turnPID.setTolerance(2) 		# tolerance in degrees
	
	def start_drive_distance(self, distance_mm):
		"""
		Start a drive command to travel a given distance (in mm).
		"""
		self.drive_target_distance = abs(distance_mm)
		self.drive_direction = 1 if distance_mm > 0 else -1
		self.drive_initial_heading = self.navx.getYaw()
		self.leftEncoder.reset()
		self.rightEncoder.reset()
		# Use loaded PID constants or default values if not set.
		if self.drivePIDConstants is None:
			self.drivePIDConstants = (0.02, 0.0, 0.0)
		self.setup_drive_pid(*self.drivePIDConstants)
		self.drivePID.setSetpoint(self.drive_target_distance)
		self.drive_active = True
	
	def update_drive(self):
		"""
		Call repeatedly (e.g., in teleopPeriodic) to update the drive command.
		Returns True when the target distance has been reached.
		"""
		if not self.drive_active:
			return True
		
		# Average encoder distance (in mm)
		left_dist = abs(self.leftEncoder.getDistance())
		right_dist = abs(self.rightEncoder.getDistance())
		avg_distance = (left_dist + right_dist) / 2.0
		
		# Compute PID output (this output serves as the base speed)
		pid_output = self.drivePID.calculate(avg_distance)
		
		# Apply a simple heading correction based on the NavX to keep the robot straight.
		current_heading = self.navx.getYaw()
		heading_error = current_heading - self.drive_initial_heading
		# Normalize heading error to [-180, 180]
		while heading_error > 180:
			heading_error -= 360
		while heading_error < -180:
			heading_error += 360
		heading_correction = 0.01 * heading_error 	# Tune this gain as needed
		
		# Compute motor speeds (apply drive_direction for forward/backward)
		left_speed = (pid_output - heading_correction) * self.drive_direction
		right_speed = (pid_output + heading_correction) * self.drive_direction
		
		self.leftMotors.set(left_speed)
		self.rightMotors.set(right_speed)
		
		if self.drivePID.atSetpoint():
			self.leftMotors.set(0)
			self.rightMotors.set(0)
			self.drive_active = False
			return True
		return False
	
	def start_turn_degrees(self, angle_degrees):
		"""
		Start a turn command to rotate the robot by a specified number of degrees.
		"""
		self.turn_target_degrees = angle_degrees
		current_heading = self.navx.getYaw()
		self.turn_desired_heading = current_heading + angle_degrees
		# Normalize desired heading to [-180, 180]
		while self.turn_desired_heading > 180:
			self.turn_desired_heading -= 360
		while self.turn_desired_heading < -180:
			self.turn_desired_heading += 360
		if self.turnPIDConstants is None:
			self.turnPIDConstants = (0.02, 0.0, 0.0)
		self.setup_turn_pid(*self.turnPIDConstants)
		self.turnPID.setSetpoint(self.turn_desired_heading)
		self.turn_active = True
	
	def update_turn(self):
		"""
		Call repeatedly (e.g., in teleopPeriodic) to update the turn command.
		Returns True when the turn command is complete.
		"""
		if not self.turn_active:
			return True
		
		current_heading = self.navx.getYaw()
		pid_output = self.turnPID.calculate(current_heading)
		# For an in-place turn, set left and right motors to opposite outputs.
		self.leftMotors.set(-pid_output)
		self.rightMotors.set(pid_output)
		
		if self.turnPID.atSetpoint():
			self.leftMotors.set(0)
			self.rightMotors.set(0)
			self.turn_active = False
			return True
		return False
	
	def autotune(self, mode='turn', relay_amplitude=0.2, sample_time=0.02, oscillation_count=10):
		"""
		Performs a simple relay-based autotuning for the PID controller.
		Forces oscillations in the system and measures the oscillation period (Pu)
		and the ultimate gain (Ku). Then, using Ziegler-Nichols tuning rules,
		calculates and stores the PID constants.
		
		Note: This is a blocking method and should be run only in a safe, controlled
		environment (not during normal robot operation).
		
		@param mode: 'turn' or 'drive' to indicate which controller to tune.
		@param relay_amplitude: the amplitude of the relay output.
		@param sample_time: time between samples (in seconds).
		@param oscillation_count: number of zero crossings to record.
		@return: Tuple (P, I, D, Pu, Ku)
		"""
		print("Starting autotuning for mode:", mode)
		zero_crossings = []
		last_error = None
		
		# Choose measurement function and set a temporary setpoint.
		if mode == 'turn':
			initial_heading = self.navx.getYaw()
			setpoint = initial_heading + 10 	# a small offset to induce oscillation
			while setpoint > 180:
				setpoint -= 360
			while setpoint < -180:
				setpoint += 360
			measurement_func = lambda: self.navx.getYaw()
		else:
			# For drive, use encoder distance. Reset encoders first.
			self.leftEncoder.reset()
			self.rightEncoder.reset()
			setpoint = 1000 	# arbitrary target distance (mm)
			measurement_func = lambda: ((abs(self.leftEncoder.getDistance()) + abs(self.rightEncoder.getDistance())) / 2.0)
		
		control_output = relay_amplitude
		print("Autotuning in progress... Please observe oscillations.")
		start_time = time.time()
		
		# Run until we record the desired number of zero crossings.
		while len(zero_crossings) < oscillation_count:
			current_measurement = measurement_func()
			error = setpoint - current_measurement
			# Check for zero crossing
			if last_error is not None:
				if (last_error < 0 and error >= 0) or (last_error > 0 and error <= 0):
					zero_crossings.append(time.time())
					# Toggle relay output
					control_output = -control_output
			# Apply the relay control to both motor groups (simulate open-loop control)
			self.leftMotors.set(control_output)
			# Reverse output on the other side for differential drive (or turning)
			self.rightMotors.set(-control_output)
			
			last_error = error
			time.sleep(sample_time)
		
		# Stop motors after autotuning.
		self.leftMotors.set(0)
		self.rightMotors.set(0)
		
		# Compute the oscillation period Pu as the average time between zero crossings.
		periods = [zero_crossings[i+1] - zero_crossings[i] for i in range(len(zero_crossings) - 1)]
		Pu = sum(periods) / len(periods)
		# For relay-based tuning, the ultimate gain Ku is roughly:
		Ku = 4 / math.pi
		print("Autotuning complete.")
		print("Measured oscillation period Pu: {:.3f} sec".format(Pu))
		print("Estimated ultimate gain Ku: {:.3f}".format(Ku))
		
		# Calculate PID constants using Ziegler-Nichols rules.
		P = 0.6 * Ku
		I = (1.2 * Ku) / Pu
		D = 0.075 * Ku * Pu
		print("Calculated PID constants: P = {:.3f}, I = {:.3f}, D = {:.3f}".format(P, I, D))
		
		# Save the tuned constants.
		if mode == 'turn':
			self.turnPIDConstants = (P, I, D)
		else:
			self.drivePIDConstants = (P, I, D)
		
		# Save the settings to file.
		self.saveSettings()
		
		return (P, I, D, Pu, Ku)
	
	def loadSettings(self):
		"""
		Attempts to load PID settings from a JSON file.
		"""
		if os.path.exists(self.settings_filename):
			try:
				with open(self.settings_filename, 'r') as f:
					data = json.load(f)
					if "drivePIDConstants" in data:
						self.drivePIDConstants = tuple(data["drivePIDConstants"])
					if "turnPIDConstants" in data:
						self.turnPIDConstants = tuple(data["turnPIDConstants"])
					print("Loaded PID settings from", self.settings_filename)
			except Exception as e:
				print("Error loading PID settings:", e)
		else:
			print("PID settings file not found. Using defaults.")
	
	def saveSettings(self):
		"""
		Saves the current PID settings to a JSON file.
		"""
		data = {
			"drivePIDConstants": self.drivePIDConstants,
			"turnPIDConstants": self.turnPIDConstants
		}
		try:
			with open(self.settings_filename, 'w') as f:
				json.dump(data, f, indent=4)
			print("Saved PID settings to", self.settings_filename)
		except Exception as e:
			print("Error saving PID settings:", e)
