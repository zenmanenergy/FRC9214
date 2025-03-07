
class Drive:
	
	def __init__(self, DriveJoystick,left_front,left_rear,right_front,right_rear,left_encoder,right_encoder):
		self.DriveJoystick=DriveJoystick
		self.left_front=left_front
		self.left_rear=left_rear
		self.right_front=right_front
		self.right_rear=right_rear
		self.leftEncoder = left_encoder
		self.rightEncoder = right_encoder

		self.DRIVE_LEFT_THUMB_UPDOWN = 0
		self.DRIVE_RIGHT_THUMB_UPDOWN = 0
		self.YButton = 0

		# Wheel & Encoder Constants
		self.WHEEL_DIAMETER_MM = 152.4  # 6-inch wheels in mm
		self.WHEEL_CIRCUMFERENCE_MM = self.WHEEL_DIAMETER_MM * 3.14159  # Circumference
		self.ENCODER_CPR = 2048  # Encoder Counts Per Revolution
		distPerPulse = self.WHEEL_CIRCUMFERENCE_MM / self.ENCODER_CPR
		self.leftEncoder.setDistancePerPulse(distPerPulse)
		self.rightEncoder.setDistancePerPulse(distPerPulse)

		self.leftSpeedFactor=0.5
		self.rightSpeedFactor=0.5

		self.isAuto = False
		self.traveling = False  # Initialize traveling status
		self.targetDistance = 0  # Initialize target distance

	def reset(self):
		self.set_motors(0, 0)
		self.isAuto = False
		self.traveling = False
		self.targetDistance = 0

	def JoyStickPeriodic(self):
		left_value = -1*self.DriveJoystick.getRawAxis(1)
		right_value = -1*self.DriveJoystick.getRawAxis(5)

		if abs(left_value) > 0.05: # this checks if the joystick is in a neutral position
			self.DRIVE_LEFT_THUMB_UPDOWN = left_value * self.leftSpeedFactor
		else:
			self.DRIVE_LEFT_THUMB_UPDOWN = 0

		if abs(right_value) > 0.05: # this checks if the joystick is in a neutral position
			self.DRIVE_RIGHT_THUMB_UPDOWN = right_value * self.rightSpeedFactor
		else:
			self.DRIVE_RIGHT_THUMB_UPDOWN = 0

		self.YButton = self.DriveJoystick.getRawButton(4)


	def periodic(self):
		self.JoyStickPeriodic()

		# If auto-traveling, follow the set distance
		if self.traveling:
			self.traveling = self.travelDistance(self.targetDistance)
		else:
			self.set_motors(self.DRIVE_LEFT_THUMB_UPDOWN, self.DRIVE_RIGHT_THUMB_UPDOWN)

		# Check if we should start auto-driving
		self.simulate_auto()

	def simulate_auto(self):
		# Start moving forward 1000 mm when Y is pressed (if not already traveling)
		if self.YButton and not self.traveling:
			print("Starting auto-drive forward 1000 mm")
			self.startTravel(1000)  # Travel forward 1000 mm


	def startTravel(self, distance_mm):
		"""Reset encoders and set travel target distance in mm."""
		self.leftEncoder.reset()
		self.rightEncoder.reset()
		self.targetDistance = distance_mm  # Already in mm due to `setDistancePerPulse`
		self.traveling = True

	def travelDistance(self, target_distance_mm):
		"""
		Drives the robot until the average encoder distance reaches the target.
		Uses proportional correction to adjust left/right speeds for straight travel.
		Returns True if still traveling, False when the target is reached.
		"""
		# Determine direction (1 = forward, -1 = backward)
		direction = 1 if target_distance_mm > 0 else -1
		baseSpeed = 0.4 * direction  # Base drive speed

		# Get encoder distances (in mm)
		leftDist = abs(self.leftEncoder.getDistance())
		rightDist = abs(self.rightEncoder.getDistance())
		avgDist = (leftDist + rightDist) / 2  # Average travel distance

		# Simple proportional correction for straight-line driving
		diff = self.leftEncoder.getDistance() - self.rightEncoder.getDistance()
		correction = 0.001 * diff  # Adjust this factor as needed
		leftSpeed = baseSpeed - correction
		rightSpeed = baseSpeed + correction

		# Keep driving if not yet reached target distance
		if avgDist < abs(target_distance_mm):
			self.set_motors(leftSpeed, -rightSpeed)  # Right motors inverted
			return True
		else:
			self.set_motors(0, 0)  # Stop when target reached
			print("Reached target distance")
			return False
		
	def set_motors(self, left_speed, right_speed):
		self.left_front.set(left_speed)
		self.left_rear.set(left_speed)
		self.right_front.set(right_speed)
		self.right_rear.set(right_speed)