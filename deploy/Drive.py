import rev
import wpilib
from phoenix5 import WPI_TalonSRX, NeutralMode

class Drive:
	def __init__(self, table):
		# Motor CAN IDs
		self.LEFT_FRONT_MOTOR_ID = 4
		self.LEFT_REAR_MOTOR_ID = 2
		self.RIGHT_FRONT_MOTOR_ID = 3
		self.RIGHT_REAR_MOTOR_ID = 1

		self.table = table

		# Create Motors
		self.left_front = WPI_TalonSRX(self.LEFT_FRONT_MOTOR_ID)
		self.left_rear = WPI_TalonSRX(self.LEFT_REAR_MOTOR_ID)
		self.right_front = WPI_TalonSRX(self.RIGHT_FRONT_MOTOR_ID)
		self.right_rear = WPI_TalonSRX(self.RIGHT_REAR_MOTOR_ID)

		self.left_front.setInverted(True)
		self.left_rear.setInverted(True)



		self.real_x_position = 0
		self.real_y_position = 0

	def periodic(self, DRIVE_LEFT_THUMB_UPDOWN, DRIVE_RIGHT_THUMB_UPDOWN):
		self.table.putNumber("real_x_position", self.real_x_position)
		self.table.putNumber("real_y_position", self.real_y_position)
		self.left_front.set(DRIVE_LEFT_THUMB_UPDOWN)
		self.left_rear.set(DRIVE_LEFT_THUMB_UPDOWN)
		self.right_front.set(DRIVE_RIGHT_THUMB_UPDOWN)
		self.right_rear.set(DRIVE_RIGHT_THUMB_UPDOWN)					



	# def control_motors(self, left_speed, right_speed):
	# 	"""Control motors while enforcing current limits."""
	
	# 	self.left_front.set(left_speed)
	# 	self.left_rear.set(left_speed)
	# 	self.right_front.set(right_speed)
	# 	self.right_rear.set(right_speed)


	# def stop_all_motors(self):
	# 	"""Stop all motors."""
	# 	self.left_front.set(0)
	# 	self.left_rear.set(0)
	# 	self.right_front.set(0)
	# 	self.right_rear.set(0)
