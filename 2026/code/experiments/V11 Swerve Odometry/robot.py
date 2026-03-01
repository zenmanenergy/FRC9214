import wpilib

from drive import SwerveDrive, SwerveModule
from joystick_drive import DriverJoystick


class Robot(wpilib.TimedRobot):
	"""Main robot container and teleop code."""

	def robotInit(self):
		"""Initialize robot and subsystems."""
		print("\n===== ROBOT INIT STARTING =====")

		# Initialize joystick
		self.driver = DriverJoystick()

		# Create swerve modules
		print("[INIT] Creating swerve modules...")
		self.front_left = SwerveModule(drive_motor_id=2, turn_motor_id=3, encoder_channel_a=0, encoder_channel_b=1, name="FL")
		self.front_right = SwerveModule(drive_motor_id=8, turn_motor_id=9, encoder_channel_a=4, encoder_channel_b=5, name="FR")
		self.rear_left = SwerveModule(drive_motor_id=4, turn_motor_id=5, encoder_channel_a=2, encoder_channel_b=3, name="RL")
		self.rear_right = SwerveModule(drive_motor_id=6, turn_motor_id=7, encoder_channel_a=6, encoder_channel_b=7, name="RR")		
		# Adjust per-motor power scaling to compensate for mechanical differences
		# Boost FL significantly, scale others down to create sync
		self.front_left.drive_power_scale = 1.35
		self.front_right.drive_power_scale = 0.94
		self.rear_left.drive_power_scale = 0.92
		self.rear_right.drive_power_scale = 0.94
		print("[INIT] Modules created")

		# Create swerve drive
		self.swerve = SwerveDrive(
			self.front_left,
			self.front_right,
			self.rear_left,
			self.rear_right,
		)
		
		# Try to load previously tuned gains
		self.swerve.load_pid_gains()
		
		# Tuning state tracking
		self.last_y_button_state = False
		
		print("===== ROBOT INIT COMPLETE =====\n")

	def teleopInit(self):
		"""Called when entering teleop mode - stop everything and display PID values."""
		print("[ROBOT] Teleop started - stopping all motors")
		if self.swerve is not None:
			self.swerve.stop()
			# Print current PID values being used
			self._print_pid_values()
	
	def _print_pid_values(self):
		"""Print all current PID values and control settings."""
		names = ["FL", "FR", "RL", "RR"]
		print("\n===== CONTROL SETTINGS =====")
		print("[CONFIG] Forward damping: 0.2")
		print("[CONFIG] Rotation damping: 0.05")
		print("[CONFIG] Forward error deadband: 1.2 counts")
		print("[CONFIG] Forward accumulated error clamp: ±100")
		print("[CONFIG] Rotation accumulated error clamp: ±5")
		print("[CONFIG] Motor command saturation: ±0.9")
		print("[CONFIG] Motor power scaling:")
		for i in range(4):
			print(f"[CONFIG]   {names[i]}: {[1.35, 0.94, 0.92, 0.94][i]}")
		
		print("\n===== CURRENT PID VALUES =====")
		print("[PID] DRIVE MOTORS:")
		for i in range(4):
			print(f"[PID]   {names[i]}: kP={self.swerve.kP[i]:.6f} kI={self.swerve.kI[i]:.8f} kD={self.swerve.kD[i]:.6f}")
		print("[PID] TURN MOTORS:")
		for i in range(4):
			print(f"[PID]   {names[i]}: kP={self.swerve.kP_turn[i]:.6f} kI={self.swerve.kI_turn[i]:.8f} kD={self.swerve.kD_turn[i]:.6f}")
		print("=============================\n")

	def teleopPeriodic(self):
		"""Handle teleop control inputs."""
		if self.swerve is None:
			return

		# Check for Y button rising edge (press, not hold)
		current_y_button_state = self.driver.get_y_button()
		if current_y_button_state and not self.last_y_button_state:
			# Rising edge detected - start tuning
			print("[ROBOT] Y button pressed - starting PID auto-tuning...")
			self.swerve.autotune_pid()  # Initialize tuning state
		self.last_y_button_state = current_y_button_state
		
		# If tuning is running, update it each cycle and skip normal driving
		if self.swerve.tuning_active:
			gains, is_complete = self.swerve.update_autotune_pid()
			if is_complete:
				# Apply the tuned per-motor values for BOTH drive and turn
				(drive_kp, drive_ki, drive_kd), (turn_kp, turn_ki, turn_kd) = gains
				self.swerve.kP = drive_kp
				self.swerve.kI = drive_ki
				self.swerve.kD = drive_kd
				self.swerve.kP_turn = turn_kp
				self.swerve.kI_turn = turn_ki
				self.swerve.kD_turn = turn_kd
				names = ["FL", "FR", "RL", "RR"]
				print("\n[ROBOT] ===== DRIVE GAINS APPLIED TO MOTOR CONTROL =====")
				for i in range(4):
					print(f"[ROBOT] {names[i]}: kP={drive_kp[i]:.6f} kI={drive_ki[i]:.8f} kD={drive_kd[i]:.6f}")
				print("[ROBOT] ===== TURN GAINS APPLIED TO MOTOR CONTROL =====")
				for i in range(4):
					print(f"[ROBOT] {names[i]}: kP={turn_kp[i]:.6f} kI={turn_ki[i]:.8f} kD={turn_kd[i]:.6f}")
				print("[ROBOT] Ready to drive!\n")
			return

		# Get joystick inputs
		forward = self.driver.get_forward()
		rotation = self.driver.get_rotation()

		## Only drive if there's actual input
		if abs(forward) > 0.0 or abs(rotation) > 0.0:
			self.swerve.drive(forward, rotation)
		else:
			self.swerve.stop()

	def disabledInit(self):
		"""Called when robot enters disabled state."""
		if self.swerve is not None:
			self.swerve.stop()


if __name__ == "__main__":
	wpilib.run(Robot)
