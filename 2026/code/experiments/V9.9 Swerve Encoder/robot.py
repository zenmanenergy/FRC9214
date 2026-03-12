"""FRC Swerve Drive Test Robot - Encoder Calibration"""
import wpilib
from wpilib import Joystick, SmartDashboard, DriverStation
from swerve_drive import SwerveDrive
from dashboard_updater import DashboardUpdater
import swerve_config as config


class Robot(wpilib.TimedRobot):
	def robotInit(self):
		import sys
		print("\n[ROBOT-INIT] Starting robot initialization...", flush=True)
		sys.stdout.flush()
		
		self.drive = SwerveDrive()
		
		print("[ROBOT-INIT] SwerveDrive created successfully", flush=True)
		sys.stdout.flush()
		
		self.joystick = Joystick(0)
		self.dashboard = DashboardUpdater(self.drive)
		
		# Test state
		self.focused = None
		self.last_focused_wheel = None  # Track last focused from web dashboard
		self.last_printed_angle = -999
		self.last_calibrate_wheel = None  # Track last calibrated wheel to detect new commands
		self.last_set_wheel_angle_name = None  # Track last wheel angle command to detect new commands
		self.driving_wheel_to_angle = None  # Current wheel being driven to target angle
		self.driving_wheel_target_angle = None  # Target angle for the wheel being driven
				# Initialize NetworkTables values with valid defaults for synchronization
		SmartDashboard.putString("calibrate_wheel", "")
		SmartDashboard.putNumber("calibrate_angle", 0)
		SmartDashboard.putString("focused_wheel", "")
		
		print("[ROBOT] Ready. Press A/B/X/Y to focus on wheels.\n")
	
	def testInit(self):
		print("[TEST] === ENTERING TEST MODE ===")
		print("[TEST] Controls:")
		print("[TEST] A/B/X/Y = Focus on rear_left/rear_right/front_left/front_right")
		print("[TEST] Left thumb up/down = Drive motor (forward/back)")
		print("[TEST] Right thumb left/right = Turn motor (rotation)")
		print("[TEST] RB = Save current position as 0°")
		print("[TEST] LB = Save current position as 90°")
		print("[TEST] BACK = Save current position as 180°")
		print("[TEST] START = Align all wheels to 0°")
		print("[TEST]")
		print("[TEST] LEVEL-BASED CALIBRATION PROCESS:")
		print("[TEST] 1. Put level on FL & RL wheels (parallel), press RB twice to save as 0°")
		print("[TEST] 2. Put level on FR & RR wheels (parallel), press RB twice to save as 0°")
		print("[TEST] 3. Rotate wheels 90°, put level on FL & FR (parallel), press LB twice to save as 90°")
		print("[TEST] 4. Put level on RL & RR (parallel), press LB twice to save as 90°")
		print("[TEST] This gives 4 calibration points per wheel for better alignment!\n")
	
	def robotPeriodic(self):
		# Update all dashboard values
		self.dashboard.update()
		
		# Publish robot enabled state
		SmartDashboard.putBoolean("Robot Enabled", DriverStation.isEnabled())
	
	def testPeriodic(self):
		# Read control commands from NetworkTables (from web dashboard)
		focused_wheel = SmartDashboard.getString("focused_wheel", "")
		align_command = SmartDashboard.getBoolean("align_command", False)
		save_zero_command = SmartDashboard.getBoolean("save_zero_command", False)
		calibrate_wheel = SmartDashboard.getString("calibrate_wheel", "")
		calibrate_angle = SmartDashboard.getNumber("calibrate_angle", -999)
		set_wheel_angle_name = SmartDashboard.getString("set_wheel_angle_name", "")
		set_wheel_angle_value = SmartDashboard.getNumber("set_wheel_angle_value", -1)
		set_wheels_direction_str = SmartDashboard.getString("set_wheels_direction", "")
		autotune_command = SmartDashboard.getBoolean("autotune_command", False)
		tuning_history_command = SmartDashboard.getBoolean("tuning_history_command", False)
		clear_tuning_command = SmartDashboard.getBoolean("clear_tuning_command", False)
		
		# Print commands received from browser
		if focused_wheel != self.last_focused_wheel:
			print(f"[BROWSER CMD] focused_wheel: {focused_wheel}")
		if align_command:
			print(f"[BROWSER CMD] align_command: True")
		if save_zero_command:
			print(f"[BROWSER CMD] save_zero_command: True")
		if calibrate_wheel:
			print(f"[BROWSER CMD] calibrate_wheel: {calibrate_wheel} at {calibrate_angle}°")
		if set_wheels_direction_str:
			print(f"[BROWSER CMD] set_wheels_direction: {set_wheels_direction_str}")
		if autotune_command:
			print(f"[BROWSER CMD] autotune_command: True")
		if tuning_history_command:
			print(f"[BROWSER CMD] tuning_history_command: True")
		if clear_tuning_command:
			print(f"[BROWSER CMD] clear_tuning_command: True")
		
		# Handle set_wheels_direction command
		if set_wheels_direction_str:
			import ast
			angles_dict = ast.literal_eval(set_wheels_direction_str)
			focused_wheel = SmartDashboard.getString("focused_wheel_preset", "")
			
			if focused_wheel and focused_wheel in angles_dict:
				# Only rotate the focused wheel using per-wheel interpolated PID gains
				target_angle = angles_dict[focused_wheel]
				self.drive.drive_wheel_to_angle(focused_wheel, target_angle)
				print(f"[DIRECTION] Rotating {focused_wheel} to {target_angle}° using interpolated gains")
			else:
				# Rotate all wheels to same angle (backward compatibility)
				target_angle = list(angles_dict.values())[0] if angles_dict else 0
				self.drive.rotate_to_angle(target_angle)
				print(f"[DIRECTION] Rotating all wheels to {target_angle}°")
			
			SmartDashboard.putString("set_wheels_direction", "")
			SmartDashboard.putString("focused_wheel_preset", "")
		
		# Detect if focus changed from web dashboard and stop previous wheel
		if focused_wheel != self.last_focused_wheel:
			self.drive.stop_all()
			self.last_focused_wheel = focused_wheel
			# If browser clears focus (empty string), clear self.focused too
			if not focused_wheel:
				self.focused = None
				print("[FOCUS] Cleared focus from browser")
		
		# Handle align command
		if align_command:
			self.drive.start_alignment()
			SmartDashboard.putBoolean("align_command", False)
		
		# Handle autotune command
		if autotune_command:
			self.drive.start_autotune()
			SmartDashboard.putBoolean("autotune_command", False)
		
		# Handle tuning history command - publish tuning history to NetworkTables
		if tuning_history_command:
			self.drive._publish_tuning_history_to_nt()
			SmartDashboard.putBoolean("tuning_history_command", False)
			print(f"[ROBOT] Published tuning history to NetworkTables\n")
		
		# Handle clear tuning command - clear tuning history
		if clear_tuning_command:
			self.drive.calibration.clear_tuning_history()
			SmartDashboard.putBoolean("clear_tuning_command", False)
			print(f"[ROBOT] Cleared tuning history\n")
		
		# Update alignment if active
		self.drive.update_alignment()
		
		# Update single-wheel alignment if active
		self.drive.update_single_wheel_alignment()
		
		# Update autotune if active
		if self.drive.autotuning:
			self.drive.update_autotune()
			# Publish current wheel being tuned
			if self.drive.autotune_gains:
				wheel_name = self.drive.autotune_gains["wheels"][self.drive.autotune_gains["current_index"]]
				SmartDashboard.putString("autotune_wheel", wheel_name)
		
		# Handle calibration command - only on change of calibrate_wheel
		if calibrate_wheel and calibrate_wheel != self.last_calibrate_wheel:
			print(f"\n[ROBOT-CHG] Detected calibrate_wheel change to: '{calibrate_wheel}'")
			self.last_calibrate_wheel = calibrate_wheel
			# Re-read the angle to ensure NetworkTables has synced
			calibrate_angle = SmartDashboard.getNumber("calibrate_angle", -999)
			print(f"[ROBOT-READ] Read calibrate_angle from NetworkTables: {calibrate_angle}")
			if calibrate_angle >= 0:
				print(f"[ROBOT-CAL] Calling set_wheel_angle('{calibrate_wheel}', {int(calibrate_angle)})")
				self.drive.set_wheel_angle(calibrate_wheel, int(calibrate_angle))
				print(f"[ROBOT-DONE] Calibration saved for {calibrate_wheel}\n")
			else:
				print(f"[ROBOT-ERR] Invalid angle {calibrate_angle}, skipping\n")
			SmartDashboard.putString("calibrate_wheel", "")
		elif not calibrate_wheel:
			self.last_calibrate_wheel = None
		
		# Handle set_wheel_angle command from dashboard buttons - only on change of set_wheel_angle_name
		if set_wheel_angle_name and set_wheel_angle_name != self.last_set_wheel_angle_name:
			print(f"\n[DASH-CHG] Detected set_wheel_angle change to: '{set_wheel_angle_name}'")
			self.last_set_wheel_angle_name = set_wheel_angle_name
			# Re-read the angle to ensure NetworkTables has synced
			set_wheel_angle_value = SmartDashboard.getNumber("set_wheel_angle_value", -1)
			print(f"[DASH-READ] Read set_wheel_angle_value from NetworkTables: {set_wheel_angle_value}")
			if set_wheel_angle_value >= 0 and set_wheel_angle_value < 360:
				print(f"[DASH-SET] Starting drive to {set_wheel_angle_value}° for {set_wheel_angle_name}")
				self.driving_wheel_to_angle = set_wheel_angle_name
				self.driving_wheel_target_angle = int(set_wheel_angle_value)
			else:
				print(f"[DASH-ERR] Invalid angle {set_wheel_angle_value}, skipping\n")
			SmartDashboard.putString("set_wheel_angle_name", "")
		elif not set_wheel_angle_name:
			self.last_set_wheel_angle_name = None
		
		# Handle save zero command
		if save_zero_command and focused_wheel:
			self.drive.set_wheel_zero(focused_wheel)
			self.drive.stop_all()
			SmartDashboard.putBoolean("save_zero_command", False)
			SmartDashboard.putString("focused_wheel", "")
			self.last_focused_wheel = None
			print(f"[ZEROING] Saved zero for {focused_wheel}\n")
		
		# Update continuous wheel drive (from dashboard arrow buttons)
		if self.driving_wheel_to_angle:
			print(f"\n[DASH-LOOP] ========== DASHBOARD MOTOR UPDATE LOOP ==========")
			print(f"[DASH-LOOP] Wheel: {self.driving_wheel_to_angle}")
			print(f"[DASH-LOOP] Target angle: {self.driving_wheel_target_angle}°")
			
			current_angle = self.drive.get_wheel_angle(self.driving_wheel_to_angle)
			print(f"[DASH-LOOP] Current angle from encoder: {current_angle}° (type: {type(current_angle).__name__})")
			
			raw_error = self.driving_wheel_target_angle - current_angle
			print(f"[DASH-LOOP] Raw error: {self.driving_wheel_target_angle}° - {current_angle}° = {raw_error}°")
			
			error = raw_error
			# Normalize error to -180 to 180 range
			if error > 180:
				error -= 360
				print(f"[DASH-LOOP] Error > 180, normalized: {raw_error}° - 360° = {error}°")
			elif error < -180:
				error += 360
				print(f"[DASH-LOOP] Error < -180, normalized: {raw_error}° + 360° = {error}°")
			else:
				print(f"[DASH-LOOP] Error in range [-180, 180]: {error}°")
			
			DASHBOARD_TOLERANCE = 1.5  # Tight enough for 5° movements, loose enough to stop
			abs_error = abs(error)
			print(f"[DASH-LOOP] Abs(error)={abs_error:.2f}° vs tolerance={DASHBOARD_TOLERANCE}°")
			
			# Check if wheel has reached target (using reasonable tolerance for dashboard)
			if abs_error < DASHBOARD_TOLERANCE:
				print(f"[DASH-LOOP] STOP CONDITION MET: abs({abs_error:.2f}) < {DASHBOARD_TOLERANCE}")
				print(f"[DASH-DONE] Wheel {self.driving_wheel_to_angle} reached target {self.driving_wheel_target_angle}°")
				self.drive.stop_all()
				self.driving_wheel_to_angle = None
				self.driving_wheel_target_angle = None
				print(f"[DASH-LOOP] Motor stopped, state cleared\n")
			else:
				print(f"[DASH-LOOP] CONTINUE CONDITION: abs({abs_error:.2f}) >= {DASHBOARD_TOLERANCE}")
				print(f"[DASH-LOOP] Calling drive_wheel_to_angle('{self.driving_wheel_to_angle}', {self.driving_wheel_target_angle})")
				# Continue driving
				self.drive.drive_wheel_to_angle(self.driving_wheel_to_angle, self.driving_wheel_target_angle)
		
		# JOYSTICK CONTROL (always active regardless of web dashboard)
		# START button - align all wheels
		if self.joystick.getRawButtonPressed(8):
			self.drive.start_alignment()
		
		# Joystick axis 1 (left thumb vertical) - drive forward/back
		left_y = -self.joystick.getRawAxis(1)  # Negative because up is positive
		
		# Joystick axis 4 (right thumb left/right) - turn rotation
		right_x = self.joystick.getRawAxis(4)
		
		# Handle wheel focus buttons (A/B/X/Y) from joystick
		for wheel_name, pin_config in config.WHEELS.items():
			if self.joystick.getRawButtonPressed(pin_config["button"]):
				if self.focused == wheel_name:
					# Toggle off
					self.drive.stop_all()
					self.focused = None
					SmartDashboard.putString("focused_wheel", "")
					print(f"[ZEROING] Focused off. Motor stopped.\n")
				else:
					# Focus on new wheel
					self.drive.stop_all()
					self.focused = wheel_name
					self.last_printed_angle = -999
					SmartDashboard.putString("focused_wheel", wheel_name)
					print(f"[ZEROING] Focused on {wheel_name}. Use left thumb up/down to drive, right thumb left/right to turn.")
		
		# Determine which wheel is focused (from browser or joystick)
		active_wheel = focused_wheel if focused_wheel else self.focused
		
		# Apply joystick controls to the focused wheel
		if active_wheel:
			# Left thumb vertical controls drive motor
			if left_y != 0:
				self.drive.set_wheel_drive_power(active_wheel, left_y * config.MOTOR_SCALE_MANUAL)
			
			# Right thumb horizontal controls turn motor
			if right_x != 0:
				self.drive.set_wheel_turn_power(active_wheel, right_x * config.MOTOR_SCALE_MANUAL)
				# Print angle only on change
				angle = self.drive.get_wheel_angle(active_wheel)
				if angle != self.last_printed_angle:
					print(f"{angle}")
					self.last_printed_angle = angle
		
		# RB button - save as 0 degrees
		if self.joystick.getRawButtonPressed(6):
			if self.focused:
				self.drive.set_wheel_zero(self.focused)
				print(f"[ZEROING] {self.focused} saved as 0°\n")
			else:
				print("[ZEROING] Must be focused first (press A/B/X/Y to focus)")
		
		# LB button - save as 90 degrees
		if self.joystick.getRawButtonPressed(5):
			if self.focused:
				self.drive.set_wheel_angle(self.focused, 90)
				print(f"[ZEROING] {self.focused} saved as 90°\n")
			else:
				print("[ZEROING] Must be focused first (press A/B/X/Y to focus)")
		
		# BACK button - save as 180 degrees
		if self.joystick.getRawButtonPressed(7):
			if self.focused:
				self.drive.set_wheel_angle(self.focused, 180)
				print(f"[ZEROING] {self.focused} saved as 180°\n")
			else:
				print("[ZEROING] Must be focused first (press A/B/X/Y to focus)")
	
	def testExit(self):
		self.drive.stop_all()
		print("[TEST] === EXITING TEST MODE ===\n")


if __name__ == "__main__":
	wpilib.run(Robot)
