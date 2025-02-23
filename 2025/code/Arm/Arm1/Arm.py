import rev
import wpilib

class Arm:
	def __init__(self):
		# Motor CAN IDs
		self.ELEVATOR_MOTOR_ID = 10
		self.SHOULDER_MOTOR_ID = 11
		self.WRIST_MOTOR_ID = 12
		self.GRABBER_MOTOR_ID = 13

		# Max Current Limits (Amps) - Adjust as needed
		self.MAX_CURRENT = 30  # Immediate shutoff threshold

		# Create Motors
		self.elevator_motor = rev.CANSparkMax(self.ELEVATOR_MOTOR_ID, rev.CANSparkMax.MotorType.kBrushless)
		self.shoulder_motor = rev.CANSparkMax(self.SHOULDER_MOTOR_ID, rev.CANSparkMax.MotorType.kBrushless)
		self.wrist_motor = rev.CANSparkMax(self.WRIST_MOTOR_ID, rev.CANSparkMax.MotorType.kBrushless)
		self.grabber_motor = rev.CANSparkMax(self.GRABBER_MOTOR_ID, rev.CANSparkMax.MotorType.kBrushless)

		# Configure Motors
		self._configure_motor(self.elevator_motor)
		self._configure_motor(self.shoulder_motor)
		self._configure_motor(self.wrist_motor)
		self._configure_motor(self.grabber_motor)

		# Encoders
		self.elevator_encoder = self.elevator_motor.getEncoder()
		self.shoulder_encoder = self.shoulder_motor.getEncoder()
		self.wrist_encoder = self.wrist_motor.getEncoder()

		# Reset Encoders to 0 on Startup
		self.elevator_encoder.setPosition(0)
		self.shoulder_encoder.setPosition(0)
		self.wrist_encoder.setPosition(0)

	def _configure_motor(self, motor):
		"""Configure basic settings for each motor."""
		motor.restoreFactoryDefaults()
		motor.setIdleMode(rev.CANSparkMax.IdleMode.kBrake)
		motor.setSmartCurrentLimit(20)  # Safe current limit (soft limit)
		motor.burnFlash()

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

	def control_motors(self, elevator_speed, shoulder_speed, wrist_speed, grabber_speed):
		"""Control motors while enforcing current limits."""
		self.check_current_limits()  # Ensure motors are not overloaded

		# Set motor speeds
		self.elevator_motor.set(elevator_speed)
		self.shoulder_motor.set(shoulder_speed)
		self.wrist_motor.set(wrist_speed)
		self.grabber_motor.set(grabber_speed)

	def stop_all_motors(self):
		"""Stop all motors."""
		self.elevator_motor.set(0)
		self.shoulder_motor.set(0)
		self.wrist_motor.set(0)
		self.grabber_motor.set(0)
