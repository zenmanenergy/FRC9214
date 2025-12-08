
class Drive:
	
	# Stores many controller functions and joystick control functions in one instance
	def __init__(self, DriveJoystick,left_front,left_rear,right_front,right_rear):
		self.DriveJoystick=DriveJoystick
		self.left_front=left_front
		self.left_rear=left_rear
		self.right_front=right_front
		self.right_rear=right_rear

		# Sets initial position of thumb sticks at 0
		self.DRIVE_LEFT_THUMB_UPDOWN = 0
		self.DRIVE_RIGHT_THUMB_UPDOWN = 0

		# These two variables set the max speed power for either side of motors
		self.leftSpeedFactor=0.5
		self.rightSpeedFactor=0.5

	def reset(self):
		self.set_motors(0,0)

	def JoyStickPeriodic(self):
		# This determines what position either thumb stick is at
		left_value = -1*self.DriveJoystick.getRawAxis(1)
		right_value = -1*self.DriveJoystick.getRawAxis(5)

		if abs(left_value) > 0.05: # this checks if the joystick is in a neutral position
			self.DRIVE_LEFT_THUMB_UPDOWN = left_value * self.leftSpeedFactor
		else:
			self.DRIVE_LEFT_THUMB_UPDOWN = 0 # If neutral, no output, robot stops

		if abs(right_value) > 0.05: # this checks if the joystick is in a neutral position
			self.DRIVE_RIGHT_THUMB_UPDOWN = right_value * self.rightSpeedFactor
		else:
			self.DRIVE_RIGHT_THUMB_UPDOWN = 0 # If neutral, no output

	# Creates a function to assign thumb sticks to drive functions, robot stops
	def periodic(self):
		self.JoyStickPeriodic()
		self.set_motors(self.DRIVE_LEFT_THUMB_UPDOWN, self.DRIVE_RIGHT_THUMB_UPDOWN)
		
	# Creates a function to assign stored speed values to the motors
	def set_motors(self, left_speed, right_speed):
		
		self.left_front.set(left_speed)
		self.left_rear.set(left_speed)
		self.right_front.set(right_speed)
		self.right_rear.set(right_speed)