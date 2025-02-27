

class ArmCalibration:
	def __init__(self):
		self.state = "idle"
		self.joint_name = ""
		self.encoder = None
		self.measurements = []
		self.attempts = 0
		self.target_degree = 0
		self.home_position = 0.0  # Store the encoder value at home (0)
		self.waiting_for_release = False  # Ensure this variable exists


	def start_calibration(self, arm):
		"""Start the calibration process."""
		print("[CALIBRATION] Starting calibration...")

		# Print encoder's raw starting position before calibration
		print(f"Elevator encoder starting position: {arm.elevator_encoder.getPosition():.6f}")
		print(f"Shoulder encoder starting position: {arm.shoulder_encoder.getPosition():.6f}")
		print(f"Wrist encoder starting position: {arm.wrist_encoder.getPosition():.6f}")

		self.state = "waiting_for_home"
		self.state = "waiting_for_wrist_home"
		self.joint_name = "SHOULDER"
		self.encoder = arm.shoulder_encoder
		self.measurements = []
		self.attempts = 0
		self.target_degree = 0
		print(f"[CALIBRATION] Move the {self.joint_name} to the HOME position (0) and press A.")


	def update(self, joystick, arm):
		"""Non-blocking update function to progress calibration step-by-step."""
		if self.state == "idle":
			return  # Nothing to do

		# Ensure button release before proceeding
		if self.waiting_for_release:
			if not joystick.getRawButton(1):  # Button has been released
				self.waiting_for_release = False
			return  # Do nothing until the button is released

		# Check if the user presses A (button 1) to record a measurement
		if joystick.getRawButton(1):
			self.waiting_for_release = True  # Wait for button release before next step

			# SHOULDER CALIBRATION
			if self.state == "waiting_for_home":
				self.home_position = arm.shoulder_encoder.getPosition()
				print(f"[CALIBRATION] SHOULDER at HOME recorded: {self.home_position:.6f}")
				self.state = "waiting_for_90"
				print("[CALIBRATION] Move the SHOULDER to 90 and press A.")

			# elif self.state == "waiting_for_90":
			# 	self.shoulder_90_avg = arm.shoulder_encoder.getPosition()
			# 	print(f"[CALIBRATION] SHOULDER at 90 recorded: {self.shoulder_90_avg:.6f}")
			# 	self.state = "waiting_for_180"
			# 	print("[CALIBRATION] Move the SHOULDER to 180 and press A.")

			# elif self.state == "waiting_for_180":
			# 	self.shoulder_180_avg = arm.shoulder_encoder.getPosition()
			# 	print(f"[CALIBRATION] SHOULDER at 180 recorded: {self.shoulder_180_avg:.6f}")

			# 	shoulder_deg_per_tick = (180 - 90) / (self.shoulder_180_avg - self.shoulder_90_avg)
			# 	shoulder_zero_offset = self.home_position

			# 	print("\n[CALIBRATION COMPLETE - SHOULDER]")
			# 	print(f"shoulder_deg_per_tick = {shoulder_deg_per_tick:.6f}")
			# 	print(f"shoulder_zero_offset = {shoulder_zero_offset:.6f}\n")

			# 	# Move to Elevator Calibration
			# 	self.state = "waiting_for_elevator_home"
			# 	print("[CALIBRATION] Move the ELEVATOR to the lowest position (0 cm) and press A.")

			# # ELEVATOR CALIBRATION
			# elif self.state == "waiting_for_elevator_home":
			# 	self.elevator_home = arm.elevator_encoder.getPosition()
			# 	print(f"[CALIBRATION] ELEVATOR at HOME recorded: {self.elevator_home:.6f}")
			# 	self.state = "waiting_for_elevator_60"
			# 	print("[CALIBRATION] Move the ELEVATOR to 60 cm and press A.")

			# elif self.state == "waiting_for_elevator_60":
			# 	self.elevator_60 = arm.elevator_encoder.getPosition()
			# 	print(f"[CALIBRATION] ELEVATOR at 60 cm recorded: {self.elevator_60:.6f}")

			# 	elevator_cm_per_tick = 60 / (self.elevator_60 - self.elevator_home)
			# 	elevator_zero_offset = self.elevator_home

			# 	print("\n[CALIBRATION COMPLETE - ELEVATOR]")
			# 	print(f"elevator_cm_per_tick = {elevator_cm_per_tick:.6f}")
			# 	print(f"elevator_zero_offset = {elevator_zero_offset:.6f}\n")

			# 	# Move to Wrist Calibration
			# 	self.state = "waiting_for_wrist_home"
			# 	print("[CALIBRATION] Move the WRIST to the HOME position (0) and press A.")

			# WRIST CALIBRATION
			elif self.state == "waiting_for_wrist_home":
				self.wrist_home = arm.wrist_encoder.getPosition()
				print(f"[CALIBRATION] WRIST at HOME recorded: {self.wrist_home:.6f}")
				self.state = "waiting_for_wrist_90"
				print("[CALIBRATION] Move the WRIST to 90 and press A.")

			elif self.state == "waiting_for_wrist_90":
				self.wrist_90_avg = arm.wrist_encoder.getPosition()
				print(f"[CALIBRATION] WRIST at 90 recorded: {self.wrist_90_avg:.6f}")
				self.state = "waiting_for_wrist_180"
				print("[CALIBRATION] Move the WRIST to 180 and press A.")

			elif self.state == "waiting_for_wrist_180":
				self.wrist_180_avg = arm.wrist_encoder.getPosition()
				print(f"[CALIBRATION] WRIST at 180 recorded: {self.wrist_180_avg:.6f}")

				wrist_deg_per_tick = (180 - 90) / (self.wrist_180_avg - self.wrist_90_avg)
				wrist_zero_offset = self.wrist_home  # Use home as the reference

				print("\n[CALIBRATION COMPLETE - WRIST]")
				print(f"wrist_deg_per_tick = {wrist_deg_per_tick:.6f}")
				print(f"wrist_zero_offset = {wrist_zero_offset:.6f}\n")

				# Calibration complete
				self.state = "complete"
				print("[CALIBRATION] Process Complete!")


