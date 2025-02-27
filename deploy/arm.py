import rev
import wpilib
from networktables import NetworkTables
from collections import deque
from wpimath.controller import PIDController


class Arm:
	def __init__(self, table):
		# Motor CAN IDs
		self.ELEVATOR_MOTOR_ID = 10
		self.WRIST_MOTOR_ID = 11
		self.SHOULDER_MOTOR_ID = 12
		self.GRABBER_MOTOR_ID = 13

		# Speeds:
		self.elevatorUpSpeedFactor = 0.4
		self.elevatorDownSpeedFactor = 0.4
		self.elevatorBreakSpeed = 0.05

		self.shoulderUpSpeedFactor = 0.3
		self.shoulderDownSpeedFactor = 0.05
		self.shoulderBreakSpeed = 0.12
		self.minShoulderBreakSpeed = 0.10
		self.maxShoulderBreakSpeed = 0.12

		self.wristUpSpeedFactor = 0.2
		self.wristDownSpeedFactor = 0.25
		self.minWristBreakSpeed = 0.02
		self.maxWristBreakSpeed = 0.03

		self.grabberInSpeedFactor = 0.5
		self.grabberOutSpeedFactor = 0.5
		self.grabberBreakSpeed = 0.0

		# Hardcoded calibration values for the shoulder
		self.shoulder_deg_per_tick = 6.395935
		self.shoulder_zero_offset = -4.738090


		self.elevator_cm_per_tick = 1.473699
		self.elevator_zero_offset = -1.214286

		# Hardcoded calibration values for the wrist (set to defaults for now)
		self.wrist_deg_per_tick = -3.716837
		self.wrist_zero_offset = 0.000000


		# Max Current Limits (Amps) - Adjust as needed
		self.MAX_CURRENT = 60
		  # Immediate shutoff threshold

		


		self.table = table

		# Create Motors
		self.elevator_motor = rev.SparkMax(self.ELEVATOR_MOTOR_ID, rev.SparkMax.MotorType.kBrushless)
		self.shoulder_motor = rev.SparkMax(self.SHOULDER_MOTOR_ID, rev.SparkMax.MotorType.kBrushless)
		self.wrist_motor = rev.SparkMax(self.WRIST_MOTOR_ID, rev.SparkMax.MotorType.kBrushless)
		self.grabber_motor = rev.SparkMax(self.GRABBER_MOTOR_ID, rev.SparkMax.MotorType.kBrushless)

		self.elevator_motor.setInverted(True)
		self.shoulder_motor.setInverted(True)
		self.wrist_motor.setInverted(True)
		self.grabber_motor.setInverted(True)

		# Encoders
		self.elevator_encoder = self.elevator_motor.getEncoder()
		self.shoulder_encoder = self.shoulder_motor.getEncoder()
		self.wrist_encoder = self.wrist_motor.getEncoder()
		self.grabber_encoder = self.grabber_motor.getEncoder()

		# Reset Encoders to 0 on Startup
		self.elevator_encoder.setPosition(0)
		self.shoulder_encoder.setPosition(0)
		self.wrist_encoder.setPosition(0)
		self.grabber_encoder.setPosition(0)

		# Store previous encoder values to detect changes
		self.prev_elevator_position = None
		self.prev_arm_angle = None
		self.prev_wrist_angle = None
		self.prev_grabber_angle = None

		self.shoulder_hold_position = None  # ✅ Fix for missing attribute
		self.wrist_hold_position = None  # Ensure wrist PID has a hold position

		self.prev_shoulder_position = self.shoulder_encoder.getPosition()
		self.shoulder_movements = deque(maxlen=5)  # Store last 5 movements to smooth braking
		self.braking_force = 0.0

		self.shoulder_pid = PIDController(0.05, 0.0, 0.01)  # Tunable values

		# Setpoint for PID (target: zero movement)
		self.shoulder_pid.setSetpoint(0.0)
		self.shoulder_pid.setTolerance(0.001)

		self.prev_wrist_angle = self.wrist_encoder.getPosition()  # Initialize to avoid NoneType error

		self.wrist_pid = PIDController(0.05, 0.0, 0.01)  # Tunable values
		self.wrist_pid.setTolerance(0.5)  # Allow small error without corrections
		self.wrist_pid.setIntegratorRange(-0.05, 0.05)  # Prevent windup

		# Last known position to hold when joystick is neutral
		self.wrist_hold_position = None
	
	def periodic(self, debug):
		# Read current encoder values
		self.real_elevator_position = (self.elevator_encoder.getPosition() - self.elevator_zero_offset) * self.elevator_cm_per_tick
		self.real_arm_angle = (self.shoulder_encoder.getPosition() - self.shoulder_zero_offset) * self.shoulder_deg_per_tick
		self.real_wrist_angle = (self.wrist_encoder.getPosition() - self.wrist_zero_offset) * self.wrist_deg_per_tick
		self.real_grabber_angle = self.grabber_encoder.getPosition()

		# Only update NetworkTables if values have changed
		if self.real_elevator_position != self.prev_elevator_position:
			self.table.putNumber("real_elevator", self.real_elevator_position)
			self.prev_elevator_position = self.real_elevator_position

		if self.real_arm_angle != self.prev_arm_angle:
			self.table.putNumber("real_arm_angle", self.real_arm_angle)
			self.prev_arm_angle = self.real_arm_angle

		if self.real_wrist_angle != self.prev_wrist_angle:
			self.table.putNumber("real_wrist_angle", self.real_wrist_angle)
			self.prev_wrist_angle = self.real_wrist_angle

		if self.real_grabber_angle != self.prev_grabber_angle:
			self.table.putNumber("real_grabber_angle", self.real_grabber_angle)
			self.prev_grabber_angle = self.real_grabber_angle

		# Print debug info only if values change
		if debug and (
			self.real_elevator_position != self.prev_elevator_position or
			self.real_arm_angle != self.prev_arm_angle or
			self.real_wrist_angle != self.prev_wrist_angle or
			self.real_grabber_angle != self.prev_grabber_angle
		):
			print(f"REAL -> Elevator: {self.real_elevator_position:.2f}, Arm: {self.real_arm_angle:.2f} degrees, "
				f"Wrist: {self.real_wrist_angle:.2f} degrees, Grabber: {self.real_grabber_angle:.2f}")

		# current_shoulder_position = self.shoulder_encoder.getPosition()
		# shoulder_movement = current_shoulder_position - self.prev_shoulder_position

		# # Compute PID output
		# brake_adjustment = self.shoulder_pid.calculate(shoulder_movement)

		# # Determine braking behavior based on angle
		# if 160 <= self.real_arm_angle <= 200:
		# 	self.shoulderBreakSpeed = 0.0  # Turn off braking in this range
		# elif self.real_arm_angle < 160:
		# 	self.shoulderBreakSpeed = max(self.minShoulderBreakSpeed, min(brake_adjustment, self.maxShoulderBreakSpeed))  # Clamp to 0.10 - 0.12
		# elif self.real_arm_angle > 200:
		# 	self.shoulderBreakSpeed = max(-1*self.maxShoulderBreakSpeed, min(brake_adjustment, -1*self.minShoulderBreakSpeed))  # Allow negative braking

		

		# # Update previous position
		# self.prev_shoulder_position = current_shoulder_position
		# Debugging information
		# if debug:
		# 	print(f"Shoulder Angle: {self.real_arm_angle:.2f}° | Shoulder Movement: {shoulder_movement:.6f} | Dynamic Break: {self.shoulderBreakSpeed:.3f}")


	# def control_motors(self, elevator_speed, shoulder_speed, wrist_speed, grabber_speed):
	# 	# self.check_current_limits()  # Ensure motors are not overloaded

	# 	# Elevator control (Left stick up/down)
	# 	if abs(elevator_speed) < 0.01:
	# 		elevator_speed=self.elevatorBreakSpeed
	# 	elif elevator_speed > 0:
	# 		elevator_speed = elevator_speed * self.elevatorUpSpeedFactor
	# 	elif elevator_speed < 0:
	# 		elevator_speed = elevator_speed * self.elevatorDownSpeedFactor
		
			


	# 	if abs(shoulder_speed) < 0.01:
	# 		shoulder_speed = self.shoulderBreakSpeed  # Apply dynamic break speed
	# 	else:
	# 		self.shoulderBreakSpeed = 0.0  # Reset braking force when moving manually
	# 		shoulder_speed *= self.shoulderUpSpeedFactor if shoulder_speed > 0 else self.shoulderDownSpeedFactor

			
			
	# 	# if abs(wrist_speed) < 0.01:
	# 	# 	wrist_speed=self.wristBreakSpeed
	# 	# elif wrist_speed > 0:
	# 	# 	wrist_speed = wrist_speed * self.wristUpSpeedFactor
	# 	# elif wrist_speed < 0:
	# 	# 	wrist_speed = wrist_speed * self.wristDownSpeedFactor

	# 		# Wrist angle limits with braking
	# 	self.real_wrist_angle = (self.wrist_encoder.getPosition() - self.wrist_zero_offset) * self.wrist_deg_per_tick

	# 	if abs(wrist_speed) < 0.01:  # Joystick is neutral, use PID to hold position
	# 		if not hasattr(self, 'wrist_hold_position') or self.wrist_hold_position == 0.0:
	# 			self.wrist_hold_position = self.real_wrist_angle  # Store current position when first neutral

	# 		# Calculate PID output to maintain the hold position
	# 		wrist_correction = self.wrist_pid.calculate(self.real_wrist_angle, self.wrist_hold_position)
	# 		self.wristBreakSpeed = max(-0.06, min(wrist_correction, 0.06))  # Clamp to prevent excessive force
	# 		wrist_speed = self.wristBreakSpeed  # Apply PID braking force
	# 	else:
	# 		# If the joystick is moved, reset hold position and allow movement
	# 		self.wrist_hold_position = self.real_wrist_angle
	# 		wrist_speed *= self.wristUpSpeedFactor if wrist_speed > 0 else self.wristDownSpeedFactor

		# Enforce wrist angle limits
		# if self.real_wrist_angle <= -10 and wrist_speed < 0:
		# 	wrist_speed = self.wristBreakSpeed  # Prevent moving below -10 degrees, apply braking
		# elif self.real_wrist_angle >= 190 and wrist_speed > 0:
		# 	wrist_speed = self.wristBreakSpeed  # Prevent moving above 190 degrees, apply braking

		

	# 	grabber_speed = grabber_speed * self.grabberInSpeedFactor
	# 	if grabber_speed < 0:
	# 		grabber_speed = grabber_speed * self.grabberOutSpeedFactor
	# 	if abs(grabber_speed) < 0.01:
	# 		grabber_speed=self.grabberBreakSpeed

		

	# 	# Set motor speeds
	# 	self.elevator_motor.set(elevator_speed)
	# 	self.shoulder_motor.set(shoulder_speed)
	# 	self.wrist_motor.set(wrist_speed)
	# 	self.grabber_motor.set(grabber_speed)
	def control_elevator(self, elevator_speed):
		"""Controls the elevator motor with braking."""
		if abs(elevator_speed) < 0.01:
			elevator_speed = self.elevatorBreakSpeed
		else:
			elevator_speed *= self.elevatorUpSpeedFactor if elevator_speed > 0 else self.elevatorDownSpeedFactor

		self.elevator_motor.set(elevator_speed)

	def control_arm(self, shoulder_speed):
		"""Controls the shoulder motor with dynamic braking and PID-based stabilization."""
		self.real_arm_angle = (self.shoulder_encoder.getPosition() - self.shoulder_zero_offset) * self.shoulder_deg_per_tick

		# Ensure previous shoulder position is initialized
		if self.prev_arm_angle is None:
			self.prev_arm_angle = self.real_arm_angle  # Initialize to avoid NoneType error

		# Detect movement direction
		shoulder_movement = self.real_arm_angle - self.prev_arm_angle

		# If the joystick is neutral, apply PID-based braking
		if abs(shoulder_speed) < 0.01:
			if self.shoulder_hold_position is None:
				self.shoulder_hold_position = self.real_arm_angle  # Store position once when joystick first goes neutral

			# Calculate PID correction to maintain position
			shoulder_correction = self.shoulder_pid.calculate(self.real_arm_angle, self.shoulder_hold_position)

			# Apply braking force within limits
			if 160 <= self.real_arm_angle <= 200:
				self.shoulderBreakSpeed = 0.0  # No braking in this range
			elif self.real_arm_angle < 160:
				self.shoulderBreakSpeed = max(self.minShoulderBreakSpeed, min(shoulder_correction, self.maxShoulderBreakSpeed))
			else:  # Arm angle > 200
				self.shoulderBreakSpeed = max(-self.maxShoulderBreakSpeed, min(shoulder_correction, -self.minShoulderBreakSpeed))

			shoulder_speed = self.shoulderBreakSpeed  # Apply PID braking only when joystick is neutral
		else:
			# Joystick is moving the arm, disable braking
			self.shoulder_hold_position = None  
			self.shoulderBreakSpeed = 0.0  # No braking when joystick is active
			shoulder_speed *= self.shoulderUpSpeedFactor if shoulder_speed > 0 else self.shoulderDownSpeedFactor

		# Update previous shoulder position
		self.prev_arm_angle = self.real_arm_angle

		self.shoulder_motor.set(shoulder_speed)



	def control_wrist(self, wrist_speed):
		print("wrist_speed",wrist_speed)
		"""Controls the wrist motor with PID holding, directional correction, and angle limits."""
		self.real_wrist_angle = (self.wrist_encoder.getPosition() - self.wrist_zero_offset) * self.wrist_deg_per_tick

		# Detect movement direction
		wrist_movement = self.real_wrist_angle - self.prev_wrist_angle

		# Ensure previous wrist position is initialized
		if self.prev_wrist_angle is None:
			self.prev_wrist_angle = self.real_wrist_angle

		# If the joystick is neutral, apply PID-based braking
		if abs(wrist_speed) < 0.01:
			if self.wrist_hold_position is None:
				self.wrist_hold_position = self.real_wrist_angle  # Store position once when joystick first goes neutral

			# Calculate PID correction
			wrist_correction = self.wrist_pid.calculate(self.real_wrist_angle, self.wrist_hold_position)

			# Apply braking force
			self.wristBreakSpeed = max(self.minWristBreakSpeed, min(wrist_correction, self.maxWristBreakSpeed))
			wrist_speed = self.wristBreakSpeed  # Apply braking only when joystick is neutral

		else:
			# Joystick is moving the wrist, disable braking
			self.wrist_hold_position = None  
			self.wristBreakSpeed = 0.0  # No braking when joystick is active
			wrist_speed *= self.wristUpSpeedFactor if wrist_speed > 0 else self.wristDownSpeedFactor

		# Ensure wrist can move freely within the allowed range
		if self.real_wrist_angle <= -10 and wrist_speed < 0:
			wrist_speed = 0  # Prevent moving below -10 degrees
		elif self.real_wrist_angle >= 280 and wrist_speed > 0:
			wrist_speed = 0  # Prevent moving above 280 degrees

		# Update previous wrist position
		self.prev_wrist_angle = self.real_wrist_angle


		self.wrist_motor.set(wrist_speed)




	def control_grabber(self, grabber_speed):
		"""Controls the grabber motor with braking."""
		grabber_speed *= self.grabberInSpeedFactor if grabber_speed > 0 else self.grabberOutSpeedFactor
		if abs(grabber_speed) < 0.01:
			grabber_speed = self.grabberBreakSpeed

		self.grabber_motor.set(grabber_speed)



	def check_current_limits(self):
		"""Check if any motor exceeds max current and stop it immediately."""
		if self.elevator_motor.getOutputCurrent() > self.MAX_CURRENT:
			print("Elevator overcurrent detected! Stopping motor.")
			self.elevator_motor.set(0)

		if self.shoulder_motor.getOutputCurrent() > self.MAX_CURRENT:
			print("Shoulder overcurrent detected! Stopping motor.")
			self.shoulder_motor.set(0)

		if self.wrist_motor.getOutputCurrent() > self.MAX_CURRENT:
			print("Wrist overcurrent detected! Stopping motor.")
			self.wrist_motor.set(0)

		if self.grabber_motor.getOutputCurrent() > self.MAX_CURRENT:
			print("Grabber overcurrent detected! Stopping motor.")
			self.grabber_motor.set(0)

	
