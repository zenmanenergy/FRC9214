
class Auto:
	# Stores many controller functions in one instance
	def __init__(self, wpilib, left_front,left_rear,right_front,right_rear):
		self.left_front=left_front
		self.left_rear=left_rear
		self.right_front=right_front
		self.right_rear=right_rear

		# Timer used to measure how long to drive
		self.timer = wpilib.Timer()

		# Optional step counter for multi-stage autos (not used here)
		self.driveStep = 0

	# Reset autonomous state and stop timing
	def reset(self):
		self.driveStep = 0
		self.timer.stop()
		self.timer.reset()

	# Begin an autonomous drive command at a fixed speed for a fixed duration
	def start(self, speed, duration):
		self.duration = duration 
		# Restart timer in a known clean state
		self.timer.stop()  # Ensure timer is reset before starting
		self.timer.reset()
		self.timer.start()

		# Apply constant motor output for the entire autonomous action
		self.left_front.set(speed)
		self.left_rear.set(speed)
		self.right_front.set(speed)
		self.right_rear.set(speed)

	# Called repeatedly during autonomous to check if the timer has expired
	def periodic(self):
		# If timer never started, autonomous action is not running
		if self.timer.get() == 0:  
			return False

		# If time is up, stop all motors and end the action
		if self.timer.get() >= self.duration:
			self.left_front.set(0)
			self.left_rear.set(0)
			self.right_front.set(0)
			self.right_rear.set(0)
			self.timer.stop()
			self.timer.reset()
			return True # autonomous action finished

		# Action still in progress
		return False