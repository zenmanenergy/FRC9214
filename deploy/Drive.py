
import wpilib
from phoenix5 import WPI_TalonSRX, NeutralMode

class Arm:
	def __init__(self):
		self.LEFT_FRONT_MOTOR_ID = 1
		self.LEFT_REAR_MOTOR_ID = 2
		self.RIGHT_FRONT_MOTOR_ID = 3
		self.RIGHT_REAR_MOTOR_ID = 4


		# Create Motors
		self.left_front = WPI_TalonSRX(self.LEFT_FRONT_MOTOR_ID)
		self.left_rear = WPI_TalonSRX(self.LEFT_REAR_MOTOR_ID)
		self.right_front = WPI_TalonSRX(self.RIGHT_FRONT_MOTOR_ID)
		self.right_rear = WPI_TalonSRX(self.RIGHT_REAR_MOTOR_ID)


	def control_motors(self, left_speed, right_speed):
		"""Control motors while enforcing current limits."""
	

		self.left_front.set(left_speed)
		# self.left_rear.set(left_speed)
		# self.right_front.set(right_speed)
		# self.right_rear.set(right_speed)


	def stop_all_motors(self):
		"""Stop all motors."""
		self.left_front.set(0)
		self.left_rear.set(0)
		self.right_front.set(0)
		self.right_rear.set(0)
