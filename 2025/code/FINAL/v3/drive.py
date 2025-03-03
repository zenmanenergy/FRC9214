
class Drive:
	
	def __init__(self, DriveJoystick,left_front,left_rear,right_front,right_rear):
		self.DriveJoystick=DriveJoystick
		self.left_front=left_front
		self.left_rear=left_rear
		self.right_front=right_front
		self.right_rear=right_rear

		self.DRIVE_LEFT_THUMB_UPDOWN = 0
		self.DRIVE_RIGHT_THUMB_UPDOWN = 0

	def reset(self):
		self.set_motors(0,0)

	def JoyStickPeriodic(self):
		left_value = -1*self.DriveJoystick.getRawAxis(1)
		right_value = -1*self.DriveJoystick.getRawAxis(5)

		if abs(left_value) > 0.05: # this checks if the joystick is in a neutral position
			self.DRIVE_LEFT_THUMB_UPDOWN = left_value * 0.5
		else:
			self.DRIVE_LEFT_THUMB_UPDOWN = 0

		if abs(right_value) > 0.05: # this checks if the joystick is in a neutral position
			self.DRIVE_RIGHT_THUMB_UPDOWN = right_value * 0.5
		else:
			self.DRIVE_RIGHT_THUMB_UPDOWN = 0


	def periodic(self):
		self.JoyStickPeriodic()

		if abs(self.DRIVE_LEFT_THUMB_UPDOWN) > 0.001 or abs(self.DRIVE_RIGHT_THUMB_UPDOWN) > 0.001:
			self.set_motors(self.DRIVE_LEFT_THUMB_UPDOWN, self.DRIVE_RIGHT_THUMB_UPDOWN)

	def set_motors(self, left_speed, right_speed):
		
		self.left_front.set(left_speed)
		self.left_rear.set(left_speed)
		self.right_front.set(right_speed)
		self.right_rear.set(right_speed)