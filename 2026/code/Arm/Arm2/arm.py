import rev
import wpilib
from networktables import NetworkTables

class Arm:
	def __init__(self, table):
		# Motor CAN IDs
		self.ELEVATOR_MOTOR_ID = 10
		self.WRIST_MOTOR_ID = 11
		self.SHOULDER_MOTOR_ID = 12
		self.GRABBER_MOTOR_ID = 13

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

	def periodic(self, debug):
		self.real_elevator_position=self.elevator_encoder.getPosition()
		self.real_arm_angle=self.shoulder_encoder.getPosition()
		self.real_wrist_angle= self.wrist_encoder.getPosition()
		self.real_grabber_angle=self.grabber_encoder.getPosition()

		self.table.putNumber("real_elevator", self.real_elevator_position)
		self.table.putNumber("real_arm_angle", self.real_arm_angle)
		self.table.putNumber("real_wrist_angle",self.real_wrist_angle)
		self.table.putNumber("real_grabber_angle",self.real_grabber_angle )
		if debug:
			print(f"REAL -> Elevator: {self.real_elevator_position}, Arm: {self.real_arm_angle}, Wrist: {self.real_wrist_angle}, Grabber: {self.real_grabber_angle}")



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
		# self.check_current_limits()  # Ensure motors are not overloaded

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
