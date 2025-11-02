
import wpilib
from phoenix5 import WPI_TalonSRX, NeutralMode

class Drive:
	def __init__(self, DriveJoystick,left_front,left_rear,right_front,right_rear,left_encoder,right_encoder):
		self.DriveJoystick=DriveJoystick
		self.left_front=left_front
		self.left_rear=left_rear
		self.right_front=right_front
		self.right_rear=right_rear
		self.leftEncoder = left_encoder
		self.rightEncoder = right_encoder

		# Encoder configuration
		self.WHEEL_DIAMETER_MM = 152.4
		self.WHEEL_CIRCUMFERENCE_MM = self.WHEEL_DIAMETER_MM * 3.14159
		self.ENCODER_CPR = 2048
		dist_per_pulse = self.WHEEL_CIRCUMFERENCE_MM / self.ENCODER_CPR

		self.leftEncoder.setDistancePerPulse(dist_per_pulse)
		self.rightEncoder.setDistancePerPulse(dist_per_pulse)

		# Travel state
		self.traveling = False  # <-- Added initialization
		self.target_distance = 0  # <-- Added initialization
		

		

	def reset(self):
		self.traveling = False

		self.target_distance = 0
		self.set_motors(0,0)

	
	def reset_encoders(self):
		"""Resets encoder values."""
		self.leftEncoder.reset()
		self.rightEncoder.reset()

	def set_motors(self, left_speed, right_speed):
		"""Sets motor speeds."""
		self.left_front.set(left_speed)
		self.left_rear.set(left_speed)
		self.right_front.set(right_speed)
		self.right_rear.set(right_speed)

	def start_travel(self, distance):
		print('start_travel')
		"""Resets encoders, stores the starting coordinates and heading, and sets the target travel distance."""
		self.reset_encoders()
		
		
		# Adjust target distance to account for overshoot.
		# if distance >0:
		# 	overshoot_distance=self.overshoot_distance
		# else:
		# 	overshoot_distance=-1*self.overshoot_distance

		# self.target_distance = distance - overshoot_distance
		if distance > 0:
			self.target_distance = distance + self.overshoot_distance
		else:
			self.target_distance = distance - self.overshoot_distance
		self.traveling = True

	def update_travel(self, base_speed=0.4):
		print('update_travel')

		if not self.traveling:
			return False

		# Determine travel direction and base speed.
		direction = 1 if self.target_distance > 0 else -1
		base_speed = base_speed * direction

		# Get the average distance traveled (in mm) from the encoders.
		left_dist = abs(self.leftEncoder.getDistance())
		right_dist = abs(self.rightEncoder.getDistance())
		avg_dist = (left_dist + right_dist) / 2.0

		# Compute encoder-based correction.
		encoder_diff = left_dist - right_dist
		encoder_correction = 0.0001 * encoder_diff  # Tune this value if needed


		# Compute motor speeds with corrections
		left_speed = base_speed - encoder_correction 
		right_speed = base_speed + encoder_correction 


		# Continue moving if the target distance hasn't been reached.
		if avg_dist < abs(self.target_distance):
			self.set_motors(left_speed, right_speed)
			return True
		else:
			self.set_motors(0, 0)
			self.traveling = False
			return False


