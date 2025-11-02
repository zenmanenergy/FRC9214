class ArmCalibration:
	def __init__(self):
		self.state = "idle"
		self.joint_name = ""
		self.encoder = None
		self.measurements = []
		self.attempts = 0
		self.target_degree = 0
		self.home_position = 0.0  # Store the encoder value at home (0)
		self.waiting_for_release = False  # Prevent multiple presses

		# Store calibration results
		self.elevator_cm_per_tick = None
		self.elevator_zero_offset = None
		self.shoulder_deg_per_tick = None
		self.shoulder_zero_offset = None
		self.wrist_deg_per_tick = None
		self.wrist_zero_offset = None

	### --- START CALIBRATION FUNCTIONS --- ###
	def start_elevator_calibration(self, arm):
		"""Start the elevator calibration."""
		print("[CALIBRATION] Starting Elevator Calibration...")
		print(f"Elevator encoder starting position: {arm.elevator_encoder.getPosition():.6f}")

		self.state = "waiting_for_elevator_home"
		print("[CALIBRATION] Move the ELEVATOR to 43.5 cm and press A.")


	def update_elevator(self, joystick, arm):
		"""Handles elevator calibration: starts at 43.5 cm, ends at 100 cm."""
		
		# Automatically record starting position at 43.5 cm
		if self.state == "waiting_for_elevator_home":
			self.elevator_home = arm.elevator_encoder.getPosition()
			print(f"[CALIBRATION] ELEVATOR starts at 43.5 cm. Encoder Value: {self.elevator_home:.6f}")
			self.state = "waiting_for_elevator_100"
			print("[CALIBRATION] Move the ELEVATOR to 100 cm and press Start.")

		# Ensure button release before allowing a new press
		elif self.state == "waiting_for_elevator_100" and not joystick.getRawButton(1):
			self.waiting_for_release = False  # Reset release flag

		# Wait for button press at 100 cm
		elif self.state == "waiting_for_elevator_100" and joystick.getRawButton(1) and not self.waiting_for_release:
			self.waiting_for_release = True  # Prevent re-trigger
			self.elevator_100 = arm.elevator_encoder.getPosition()

			# Debug output
			print(f"[DEBUG] elevator_home = {self.elevator_home:.6f}, elevator_100 = {self.elevator_100:.6f}")

			# Ensure the elevator actually moved before proceeding
			if abs(self.elevator_100 - self.elevator_home) < 1.0:  # Allow some tolerance
				print("[ERROR] Elevator did not move significantly! Calibration aborted.")
				self.state = "idle"
				return  # Stop calibration

			print(f"[CALIBRATION] ELEVATOR at 100 cm recorded. Encoder Value: {self.elevator_100:.6f}")

			# Compute conversion factor
			self.elevator_cm_per_tick = 56.5 / (self.elevator_100 - self.elevator_home)
			self.elevator_zero_offset = self.elevator_home - (43.5 / self.elevator_cm_per_tick)


			# Print final calibration values
			print("\n[CALIBRATION COMPLETE - ELEVATOR]")
			print(f"elevator_cm_per_tick = {self.elevator_cm_per_tick:.6f}")
			print(f"elevator_zero_offset = {self.elevator_zero_offset:.6f}")
			print("[CALIBRATION] Elevator calibration is now complete.")

			# *** END CALIBRATION PROPERLY ***
			self.state = "idle"  # Stop calibration process





	def start_shoulder_calibration(self, arm):
		"""Start shoulder calibration sequence."""
		print("[CALIBRATION] Starting Shoulder Calibration...")
		print(f"[DEBUG] Shoulder Encoder at Start: {arm.shoulder_encoder.getPosition():.6f}")

		# Set initial state so update_shoulder() begins
		self.state = "waiting_for_shoulder_negative_45"  
		print("[CALIBRATION] Move the SHOULDER to **-45°** and press A.")


	
	def update_shoulder(self, joystick, arm):
		"""Handles shoulder calibration: move to -45° and 0°, then calculates offsets."""
		
		# Debug current state
		print(f"[DEBUG] Current State: {self.state}")

		# Step 1: Move to -45° and press A
		if self.state == "waiting_for_shoulder_negative_45":
			print(f"[DEBUG] Waiting for -45°... A Button: {joystick.getRawButton(1)}")

			if joystick.getRawButton(1):  # A Button Press
				self.shoulder_negative_45 = arm.shoulder_encoder.getPosition()
				print(f"[CALIBRATION] SHOULDER at -45° recorded. Encoder Value: {self.shoulder_negative_45:.6f}")
				self.state = "waiting_for_shoulder_0"
				print("[CALIBRATION] Now, move the SHOULDER to **0°** and press Start.")

		# Step 2: Move to 0° and press Start
		elif self.state == "waiting_for_shoulder_0":
			print(f"[DEBUG] Waiting for 0°... Start Button: {joystick.getRawButton(8)}")

			if joystick.getRawButton(8):  # Start Button Press
				self.shoulder_0 = arm.shoulder_encoder.getPosition()
				print(f"[CALIBRATION] SHOULDER at 0° recorded. Encoder Value: {self.shoulder_0:.6f}")

				# Verify movement
				if abs(self.shoulder_0 - self.shoulder_negative_45) < 1.0:
					print("[ERROR] Shoulder did not move significantly! Calibration aborted.")
					self.state = "idle"
					return

				# **🔥 FIX: Correct Zero Offset Calculation**
				self.shoulder_deg_per_tick = (-45) / (self.shoulder_negative_45 - self.shoulder_0)
				self.shoulder_zero_offset = self.shoulder_0 - (0 / self.shoulder_deg_per_tick)

				# Print results
				print("\n[CALIBRATION COMPLETE - SHOULDER]")
				print(f"shoulder_deg_per_tick = {self.shoulder_deg_per_tick:.6f}")
				print(f"shoulder_zero_offset = {self.shoulder_zero_offset:.6f}")
				print("[CALIBRATION] Shoulder calibration is now complete.")

				# 🔥 FIX: Ensure state does NOT reset
				self.state = "completed"  # Change state to prevent re-running








	def start_wrist_calibration(self, arm):
		"""Start wrist calibration by setting the initial state and printing the first instruction."""
		print("\n[CALIBRATION] GO TO 0 degrees and press Start")
		self.state = "waiting_for_wrist_start"

	def update_wrist(self, joystick, arm):
		"""Handles wrist calibration: Move to 0°, press Start. Move to -90°, press A. Then calculate offsets."""

		# Step 1: Ask the user to move the wrist to 0° and press Start
		if self.state == "waiting_for_wrist_start":
			self.state = "waiting_for_wrist_0_press"  # Move to the next step

		elif self.state == "waiting_for_wrist_0_press":
			if joystick.getRawButton(8):  # Start Button Pressed
				self.wrist_0 = arm.wrist_encoder.getPosition()
				print(f"[CALIBRATION] Start pressed - Encoder Value: {self.wrist_0:.6f}")
				print("\n[CALIBRATION] GO TO -90 degrees and press A")
				self.state = "waiting_for_wrist_negative_90"

		# Step 2: Ask the user to move to -90° and press A
		elif self.state == "waiting_for_wrist_negative_90":
			if joystick.getRawButton(1):  # A Button Pressed
				self.wrist_negative_90 = arm.wrist_encoder.getPosition()
				print(f"[CALIBRATION] A pressed - Encoder Value: {self.wrist_negative_90:.6f}")

				# Ensure the wrist actually moved
				difference = abs(self.wrist_negative_90 - self.wrist_0)

				if difference < 5.0:  # 🔥 If encoder change is too small, calibration fails
					print("[ERROR] Wrist did not move enough! Calibration aborted.")
					self.state = "idle"
					return

				# 🔥 Correct Zero Offset Calculation
				self.wrist_deg_per_tick = (-90) / (self.wrist_negative_90 - self.wrist_0)
				self.wrist_zero_offset = self.wrist_0

				# Print final calibration results
				print("\n[CALIBRATION COMPLETE - WRIST]")
				print(f"wrist_deg_per_tick = {self.wrist_deg_per_tick:.6f}")
				print(f"wrist_zero_offset = {self.wrist_zero_offset:.6f}")
				print("[CALIBRATION] Wrist calibration is now complete.")

				# End calibration
				self.state = "idle"




	def verify_encoders(self, arm):
		"""Prints current encoder values for verification when RB is pressed."""
		self.RBButton = self.ArmJoystick.getRawButton(6)
		if self.RBButton:
			print("\n[VERIFICATION] Current Encoder Values:")
			print(f"  Elevator Encoder: {arm.elevator_encoder.getPosition():.6f}")
			print(f"  Shoulder Encoder: {arm.shoulder_encoder.getPosition():.6f}")
			print(f"  Wrist Encoder: {arm.wrist_encoder.getPosition():.6f}")
			print("[VERIFICATION] Press RB anytime to check encoder values.\n")




	def update(self, joystick, arm):
		"""Call the correct calibration update function based on the active state."""
		if self.state.startswith("waiting_for_elevator"):
			self.update_elevator(joystick, arm)
		elif self.state.startswith("waiting_for_shoulder"):
			self.update_shoulder(joystick, arm)
		elif self.state.startswith("waiting_for_wrist"):
			self.update_wrist(joystick, arm)
