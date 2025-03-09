
class Auto:
	
	def __init__(self, wpilib, left_front,left_rear,right_front,right_rear, arm, drive):
		self.left_front=left_front
		self.left_rear=left_rear
		self.right_front=right_front
		self.right_rear=right_rear
		self.arm = arm
		self.drive = drive
		self.done = False
		self.done2 = False
		self.done3 = False

		self.timer = wpilib.Timer()
		self.driveStep = 0

	def reset(self):
		self.done = False
		self.done2 = False
		self.done3 = False
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

			
		if self.timer.get() > 1 and not self.done2:
			self.done2 = True
			self.drive.start_travel(400)
			# print("38")
		if self.timer.get() >= 1.5:
			self.arm.autoPreset = "EUp"
			# print("41")
		if self.timer.get() >= 2.5:
			self.arm.autoPreset = "Y"
			# print("44")
		# print(self.done)
		if self.timer.get() > 6 and not self.done:
			self.drive.start_travel(345)
			print("move again")
			self.done = True

		if self.timer.get() >= 10:
			self.arm.autoPreset = "EDown"

		
		if self.timer.get() >= 12 and not self.done3:
			self.drive.traveling = False
			self.drive.start_travel(-175)
			self.done3 = True

		if self.timer.get() >= 12:
			self.arm.autoPreset = "EDownDown"
		
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