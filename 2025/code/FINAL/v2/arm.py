import rev
import wpilib
from networktables import NetworkTables
from collections import deque
from wpimath.controller import PIDController


class Arm:
	
	def __init__(self, table):
		self.target_elevator = None
		self.target_shoulder = None
		self.target_wrist = None
		# Motor CAN IDs
		self.ELEVATOR_MOTOR_ID = 10
		self.WRIST_MOTOR_ID = 11
		self.SHOULDER_MOTOR_ID = 12
		self.GRABBER_MOTOR_ID = 13

		# Speeds:
		self.elevatorUpSpeedFactor = 0.4
		self.elevatorDownSpeedFactor = 0.4
		self.elevatorBreakSpeed = 0.05
		self.elevator_min_height = 0
		self.elevator_max_height = 60


		self.shoulderUpSpeedFactor = 0.3
		self.shoulderDownSpeedFactor = 0.05
		self.shoulderBreakSpeed = 0.12
		self.minShoulderBreakSpeed = 0.06
		self.maxShoulderBreakSpeed = 0.06
		self.shoulder_min_angle = 10
		self.shoulder_max_angle = 140

		self.wristUpSpeedFactor = 0.2
		self.wristDownSpeedFactor = 0.25
		self.wristBreakSpeed = 0.03
		self.wrist_min_angle = -100
		self.wrist_max_angle = 220

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

		
		self.real_elevator_position = 0.0
		self.real_arm_angle = 0.0
		self.real_wrist_angle = 0.0
		self.real_grabber_angle = 0.0

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

		# Store previous encoder values to detect changes
		self.prev_elevator_position = 0
		self.prev_arm_angle = 0
		self.prev_wrist_angle = 0
		self.prev_grabber_angle = 0

		self.wrist_hold_position = None  # Ensure wrist PID has a hold position

		self.prev_shoulder_position = self.shoulder_encoder.getPosition()
		self.shoulder_movements = deque(maxlen=5)  # Store last 5 movements to smooth braking
		self.braking_force = 0.0
		self.prev_wrist_angle = self.wrist_encoder.getPosition()  # Initialize to avoid NoneType error


		# Elevator PID Controller
		self.elevator_pid = PIDController(0.05, 0.0, 0.01)  # Tunable values
		self.elevator_pid.setTolerance(1)  # Allow small error before stopping

		# Shoulder PID Controller
		self.shoulder_pid = PIDController(0.05, 0.0, 0.01)  # Tunable values
		self.shoulder_pid.setTolerance(0.001)  # Small tolerance for precision

		# Wrist PID Controller
		self.wrist_pid = PIDController(0.05, 0.0, 0.01)  # Tunable values
		self.wrist_pid.setTolerance(0.5)  # Allow small error before stopping
		self.wrist_pid.setIntegratorRange(-0.05, 0.05)  # Prevent windup


	
	def resetEncoders(self):
		"""Resets all encoders to zero."""
		print("Resetting encoders...")
		self.elevator_encoder.setPosition(0)
		self.shoulder_encoder.setPosition(0)
		self.wrist_encoder.setPosition(0)
		self.grabber_encoder.setPosition(0)
		  # Mark encoders as reset


	def periodic(self, debug):
		# Read current encoder values
		self.real_elevator_position = (self.elevator_encoder.getPosition() - self.elevator_zero_offset) * self.elevator_cm_per_tick
		self.real_arm_angle = (self.shoulder_encoder.getPosition() - self.shoulder_zero_offset) * self.shoulder_deg_per_tick
		self.real_wrist_angle = (self.wrist_encoder.getPosition() - self.wrist_zero_offset) * self.wrist_deg_per_tick
		self.real_grabber_angle = self.grabber_encoder.getPosition()


		if debug and (
			self.real_elevator_position != self.prev_elevator_position or
			self.real_arm_angle != self.prev_arm_angle or
			self.real_wrist_angle != self.prev_wrist_angle or
			self.real_grabber_angle != self.prev_grabber_angle
		):
			print(f"REAL -> Elevator: {self.real_elevator_position:.2f}, Arm: {self.real_arm_angle:.2f} degrees, "
				f"Wrist: {self.real_wrist_angle:.2f} degrees, Grabber: {self.real_grabber_angle:.2f}")

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
		

	def update_movement(self):
		"""Moves the arm toward target positions smoothly using PID controllers."""
		if self.target_elevator is None or self.target_shoulder is None or self.target_wrist is None:
			return  # No active movement target

		# Move elevator using PID
		if abs(self.real_elevator_position - self.target_elevator) > 1:
			pid_output = self.elevator_pid.calculate(self.real_elevator_position, self.target_elevator)
			elevator_speed = max(min(pid_output, 0.4), -0.4)  # Limit max speed to 0.4

			# Ensure elevator speed obeys movement limits
			if not self.limit_elevator(elevator_speed):
				elevator_speed = 0

			self.control_elevator(elevator_speed)

		# Move shoulder using PID
		if abs(self.real_arm_angle - self.target_shoulder) > 2:
			pid_output = self.shoulder_pid.calculate(self.real_arm_angle, self.target_shoulder)
			shoulder_speed = max(min(pid_output, 0.3), -0.3)  # Limit max speed to 0.3

			# Ensure shoulder speed obeys movement limits
			if not self.limit_shoulder(shoulder_speed):
				shoulder_speed = 0

			self.control_shoulder(shoulder_speed)

		# Move wrist using PID
		if abs(self.real_wrist_angle - self.target_wrist) > 2:
			pid_output = self.wrist_pid.calculate(self.real_wrist_angle, self.target_wrist)
			wrist_speed = max(min(pid_output, 0.2), -0.2)  # Limit max speed to 0.2

			# Ensure wrist speed obeys movement limits
			if not self.limit_wrist(wrist_speed):
				wrist_speed = 0

			self.control_wrist(wrist_speed)

		# Stop movement once close to target
		if (
			abs(self.real_elevator_position - self.target_elevator) <= 1 and
			abs(self.real_arm_angle - self.target_shoulder) <= 2 and
			abs(self.real_wrist_angle - self.target_wrist) <= 2
		):
			print(f"Reached preset target.")
			self.target_elevator = None
			self.target_shoulder = None
			self.target_wrist = None



	def set_target_positions(self, elevator, shoulder, wrist):
			"""Sets the target positions for movement."""
			self.target_elevator = elevator
			self.target_shoulder = shoulder
			self.target_wrist = wrist




	def limit_wrist(self, wrist_speed):
		"""Ensures wrist does not exceed its movement limits based on joystick direction."""

		# If the wrist is BELOW the minimum limit, allow ONLY upward movement
		if self.real_wrist_angle < self.wrist_min_angle and wrist_speed > 0:
			return False  # Block downward movement
		if self.real_wrist_angle < self.wrist_min_angle and wrist_speed < 0:
			return True   # Allow upward movement

		# If the wrist is ABOVE the maximum limit, allow ONLY downward movement
		if self.real_wrist_angle > self.wrist_max_angle and wrist_speed < 0:
			return False  # Block upward movement
		if self.real_wrist_angle > self.wrist_max_angle and wrist_speed > 0:
			return True   # Allow downward movement

		# Otherwise, movement is unrestricted
		return True


	def limit_shoulder(self, shoulder_speed):
		"""Ensures shoulder does not exceed its movement limits based on joystick direction."""
		if shoulder_speed > 0 and self.real_arm_angle >= self.shoulder_max_angle:
			return False  # Stop moving upward
		if shoulder_speed < 0 and self.real_arm_angle <= self.shoulder_min_angle:
			return False  # Stop moving downward
		return True

	def limit_elevator(self, elevator_speed):
		"""Ensures elevator does not exceed its movement limits based on joystick direction."""
		if elevator_speed > 0 and self.real_elevator_position >= self.elevator_max_height:
			return False  # Stop moving upward
		if elevator_speed < 0 and self.real_elevator_position <= self.elevator_min_height:
			return False  # Stop moving downward
		return True

	def control_wrist(self, wrist_speed):
		"""Controls the wrist motor with braking and limit checks."""
		if not self.limit_wrist(wrist_speed):
			wrist_speed = 0  # Prevent movement beyond limits
		elif abs(wrist_speed) < 0.01:
			wrist_speed = self.wristBreakSpeed
		elif wrist_speed > 0:
			wrist_speed *= self.wristUpSpeedFactor
		elif wrist_speed < 0:
			wrist_speed *= self.wristDownSpeedFactor

		self.wrist_motor.set(wrist_speed)

	def control_shoulder(self, shoulder_speed):
		"""Controls the shoulder motor with braking and limit checks."""
		if self.real_arm_angle < 70 or self.real_arm_angle > 120:
			self.shoulderBreakSpeed = self.minShoulderBreakSpeed
		else:
			self.shoulderBreakSpeed = self.maxShoulderBreakSpeed

		if not self.limit_shoulder(shoulder_speed):
			shoulder_speed = 0  # Prevent movement beyond limits
		elif abs(shoulder_speed) < 0.01:
			shoulder_speed = self.shoulderBreakSpeed
		elif shoulder_speed > 0:
			shoulder_speed *= self.shoulderUpSpeedFactor
		elif shoulder_speed < 0:
			shoulder_speed *= self.shoulderDownSpeedFactor

		self.shoulder_motor.set(shoulder_speed)

	def control_elevator(self, elevator_speed):
		"""Controls the elevator motor with braking and limit checks."""
		if not self.limit_elevator(elevator_speed):
			elevator_speed = 0  # Prevent movement beyond limits
		elif abs(elevator_speed) < 0.01:
			elevator_speed = self.elevatorBreakSpeed
		else:
			elevator_speed *= self.elevatorUpSpeedFactor if elevator_speed > 0 else self.elevatorDownSpeedFactor

		self.elevator_motor.set(elevator_speed)

	

	def control_grabber(self, grabber_speed):
		"""Controls the grabber motor with braking."""
		grabber_speed *= self.grabberInSpeedFactor if grabber_speed > 0 else self.grabberOutSpeedFactor
		if abs(grabber_speed) < 0.01:
			# print("stopped")
			grabber_speed = self.grabberBreakSpeed

		self.grabber_motor.set(grabber_speed)




	

	
