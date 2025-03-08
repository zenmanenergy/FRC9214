
class Auto:
	
	def __init__(self, wpilib, left_front,left_rear,right_front,right_rear, arm, drive):
		self.left_front=left_front
		self.left_rear=left_rear
		self.right_front=right_front
		self.right_rear=right_rear
		self.arm = arm
		self.drive = drive

		self.timer = wpilib.Timer()
		self.driveStep = 0

	def reset(self):
		self.driveStep = 0
		self.timer.stop()
		self.timer.reset()

	def start(self, speed, duration):
		self.duration = duration
		self.timer.stop()  # Ensure timer is reset before starting
		self.timer.reset()
		self.timer.start()
		# self.left_front.set(speed)
		# self.left_rear.set(speed)
		# self.right_front.set(speed)
		# self.right_rear.set(speed)

	def periodic(self):

		if self.timer.get() >= 1:
			self.arm.autoPreset = "EUp"

		if self.timer.get() >= 2:
			self.arm.autoPreset = "Y"
		
		
		# if self.timer.get() == 0:  # Timer hasn't started, do nothing
		# 	return False
		
		# if self.timer.get() >= self.duration:
		# 	self.timer.stop()
		# 	self.timer.reset()
		# 	self.timer.start()
		# 	# self.left_front.set(-0.3)
		# 	# self.left_rear.set(-0.3)
		# 	# self.right_front.set(-0.3)
		# 	# self.right_rear.set(-0.3)
		# 	# self.driveStep = 1
			

		# if self.driveStep == 1 and self.timer.get() >= 0.5:
		# 	self.left_front.set(0)
		# 	self.left_rear.set(0)
		# 	self.right_front.set(0)
		# 	self.right_rear.set(0)
		# 	self.timer.stop()
		# 	self.timer.reset()
		# 	return True

		return False