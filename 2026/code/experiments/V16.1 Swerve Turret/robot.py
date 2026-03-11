"""FRC Swerve Drive Test Robot - Encoder Calibration with Shooter"""
import wpilib
from wpilib import SmartDashboard, DriverStation
from swerve_drive import SwerveDrive
from turret import Turret
from shooter import ShooterSubsystem
from copilotJoystick import CopilotJoystick
from shooter_controls import ShooterControls
from pilotJoystick import PilotJoystick
from pilot_controls import PilotControls
from dashboard_updater import DashboardUpdater
from encoder_calibration import EncoderCalibration
import swerve_config as config
import CANID


class Robot(wpilib.TimedRobot):
	def robotInit(self):
		import sys
		print("\n[ROBOT-INIT] Starting robot initialization...", flush=True)
		sys.stdout.flush()
		
		self.drive = SwerveDrive()
		
		print("[ROBOT-INIT] SwerveDrive created successfully", flush=True)
		sys.stdout.flush()
		
		self.turret = Turret(turn_canid=CANID.TURRET_TURN, encoder_dio=4)
		print("[ROBOT-INIT] Turret created successfully", flush=True)
		sys.stdout.flush()
		
		self.shooter = ShooterSubsystem(turret=self.turret)
		print("[ROBOT-INIT] Shooter created successfully", flush=True)
		sys.stdout.flush()
		
		# Load calibration for both swerve and turret
		self.calibration = EncoderCalibration()
		turret_offset = self.calibration.get_offset("turret")
		self.turret.set_zero_offset(turret_offset)
		turret_left_limit = self.calibration.get_offset("turret_left_limit")
		turret_right_limit = self.calibration.get_offset("turret_right_limit")
		self.turret.set_left_limit(turret_left_limit)
		self.turret.set_right_limit(turret_right_limit)
		print(f"[ROBOT-INIT] Loaded turret offset: {turret_offset:.1f}, left_limit: {turret_left_limit:.1f}, right_limit: {turret_right_limit:.1f}", flush=True)
		sys.stdout.flush()
		
		self.pilot_joystick = PilotJoystick(port=0, deadband=0.1)
		self.pilot_controls = PilotControls(self.drive, self.pilot_joystick)
		
		self.copilot_joystick = CopilotJoystick(port=1, deadband=0.1)
		if not self.copilot_joystick.is_available():
			print("[ROBOT-INIT] WARNING: Copilot joystick (port 1) not plugged in!", flush=True)
		self.shooter_controls = ShooterControls(self.shooter, self.copilot_joystick)
		
		self.dashboard = DashboardUpdater(self.drive, self.turret)
		
		# Test state for web dashboard integration
		self.last_focused_wheel = None
		self.last_calibrate_wheel = None  # Track last calibrated wheel to detect new commands
		self.last_set_wheel_angle_name = None  # Track last wheel angle command to detect new commands
		self.driving_wheel_to_angle = None  # Current wheel being driven to target angle
		self.driving_wheel_target_angle = None  # Target angle for the wheel being driven
		
		# Initialize NetworkTables values with valid defaults for synchronization
		SmartDashboard.putString("calibrate_wheel", "")
		SmartDashboard.putNumber("calibrate_angle", 0)
		SmartDashboard.putString("focused_wheel", "")
		SmartDashboard.putString("robot_mode", "Unknown")
		SmartDashboard.putBoolean("turret_set_zero_command", False)
		SmartDashboard.putBoolean("turret_set_left_limit_command", False)
		SmartDashboard.putBoolean("turret_set_right_limit_command", False)
		
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
		
		# Publish mode to NetworkTables so dashboard auto-detects
		SmartDashboard.putString("robot_mode", "Test")
		print("[MODE] Published robot_mode = Test to NetworkTables\n")
	
	def robotPeriodic(self):
		# Update all dashboard values
		self.dashboard.update()
		
		# Publish robot enabled state
		SmartDashboard.putBoolean("Robot Enabled", DriverStation.isEnabled())
	
	def testPeriodic(self):
		# Ensure robot_mode stays set to Test
		SmartDashboard.putString("robot_mode", "Test")
		
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
			focused_wheel_preset = SmartDashboard.getString("focused_wheel_preset", "")
			
			if focused_wheel_preset and focused_wheel_preset in angles_dict:
				target_angle = angles_dict[focused_wheel_preset]
				self.drive.drive_wheel_to_angle(focused_wheel_preset, target_angle)
				print(f"[DIRECTION] Rotating {focused_wheel_preset} to {target_angle}° using interpolated gains")
			else:
				target_angle = list(angles_dict.values())[0] if angles_dict else 0
				self.drive.rotate_to_angle(target_angle)
				print(f"[DIRECTION] Rotating all wheels to {target_angle}°")
			
			SmartDashboard.putString("set_wheels_direction", "")
			SmartDashboard.putString("focused_wheel_preset", "")
		
		# Detect if focus changed from web dashboard and stop previous wheel
		if focused_wheel != self.last_focused_wheel:
			self.drive.stop_all()
			self.last_focused_wheel = focused_wheel
			# If browser clears focus (empty string), clear pilot focus too
			if not focused_wheel:
				self.pilot_controls.focused = None
				print("[FOCUS] Cleared focus from browser")
		
		# Handle align command
		if align_command:
			self.drive.start_alignment()
			SmartDashboard.putBoolean("align_command", False)
		
		# Handle autotune command
		if autotune_command:
			self.drive.start_autotune()
			SmartDashboard.putBoolean("autotune_command", False)
		
		# Handle tuning history command
		if tuning_history_command:
			self.drive._publish_tuning_history_to_nt()
			SmartDashboard.putBoolean("tuning_history_command", False)
			print(f"[ROBOT] Published tuning history to NetworkTables\n")
		
		# Handle clear tuning command
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
			if self.drive.autotune_gains:
				wheel_name = self.drive.autotune_gains["wheels"][self.drive.autotune_gains["current_index"]]
				SmartDashboard.putString("autotune_wheel", wheel_name)
		
		# Handle calibration command - only on change of calibrate_wheel
		if calibrate_wheel and calibrate_wheel != self.last_calibrate_wheel:
			print(f"\n[ROBOT-CHG] Detected calibrate_wheel change to: '{calibrate_wheel}'")
			self.last_calibrate_wheel = calibrate_wheel
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
		
		# Handle set_wheel_angle command from dashboard buttons
		if set_wheel_angle_name and set_wheel_angle_name != self.last_set_wheel_angle_name:
			print(f"\n[DASH-CHG] Detected set_wheel_angle change to: '{set_wheel_angle_name}'")
			self.last_set_wheel_angle_name = set_wheel_angle_name
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
		
		# Handle turret set zero command
		turret_set_zero_command = SmartDashboard.getBoolean("turret_set_zero_command", False)
		if turret_set_zero_command:
			print(f"[TURRET-DEBUG] turret_set_zero_command received!")
			raw_encoder_degrees = self.turret.get_raw_encoder_degrees()
			print(f"[TURRET-DEBUG] Raw encoder angle: {raw_encoder_degrees:.1f}")
			self.turret.set_zero_offset(raw_encoder_degrees)
			self.calibration.set_offset("turret", raw_encoder_degrees)
			print(f"[TURRET-DEBUG] Saving to file...")
			self.calibration.save_calibration()
			self.turret.stop()
			SmartDashboard.putBoolean("turret_set_zero_command", False)
			print(f"[TURRET-ZEROING] Saved turret zero at encoder angle: {raw_encoder_degrees:.1f}\n")
		
		# Handle turret set left limit command
		turret_set_left_limit_command = SmartDashboard.getBoolean("turret_set_left_limit_command", False)
		if turret_set_left_limit_command:
			print(f"[TURRET-DEBUG] turret_set_left_limit_command received!")
			turret_angle = self.turret.get_angle()
			print(f"[TURRET-DEBUG] Current turret angle: {turret_angle}°")
			self.turret.set_left_limit(turret_angle)
			self.calibration.set_offset("turret_left_limit", turret_angle)
			print(f"[TURRET-DEBUG] Saving to file...")
			self.calibration.save_calibration()
			self.turret.stop()
			SmartDashboard.putBoolean("turret_set_left_limit_command", False)
			print(f"[TURRET-LIMIT] Saved turret left limit at: {turret_angle}°\n")
		
		# Handle turret set right limit command
		turret_set_right_limit_command = SmartDashboard.getBoolean("turret_set_right_limit_command", False)
		if turret_set_right_limit_command:
			print(f"[TURRET-DEBUG] turret_set_right_limit_command received!")
			turret_angle = self.turret.get_angle()
			print(f"[TURRET-DEBUG] Current turret angle: {turret_angle}°")
			self.turret.set_right_limit(turret_angle)
			self.calibration.set_offset("turret_right_limit", turret_angle)
			print(f"[TURRET-DEBUG] Saving to file...")
			self.calibration.save_calibration()
			self.turret.stop()
			SmartDashboard.putBoolean("turret_set_right_limit_command", False)
			print(f"[TURRET-LIMIT] Saved turret right limit at: {turret_angle}°\n")
		
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
			if error > 180:
				error -= 360
				print(f"[DASH-LOOP] Error > 180, normalized: {raw_error}° - 360° = {error}°")
			elif error < -180:
				error += 360
				print(f"[DASH-LOOP] Error < -180, normalized: {raw_error}° + 360° = {error}°")
			else:
				print(f"[DASH-LOOP] Error in range [-180, 180]: {error}°")
			
			DASHBOARD_TOLERANCE = 1.5
			abs_error = abs(error)
			print(f"[DASH-LOOP] Abs(error)={abs_error:.2f}° vs tolerance={DASHBOARD_TOLERANCE}°")
			
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
				self.drive.drive_wheel_to_angle(self.driving_wheel_to_angle, self.driving_wheel_target_angle)
		
		# Execute joystick-based control (pass focused_wheel from web dashboard to pilot_controls)
		self.pilot_controls.execute_test(active_wheel=focused_wheel)
		
		# SHOOTER CONTROLS
		self.shooter_controls.execute()
	
	def testExit(self):
		self.drive.stop_all()
		print("[TEST] === EXITING TEST MODE ===\n")
	
	def teleopInit(self):
		print("[TELEOP] === ENTERING TELEOP MODE ===")
		print("[TELEOP] Controls:")
		print("[TELEOP] Left Joystick: Forward/Backward and Strafe (Left/Right)")
		print("[TELEOP] Right Joystick X: Rotate Left/Right")
		print("[TELEOP] The robot will automatically orient wheels for swerve drive")
		print("[TELEOP] Release joystick to center wheels to 0°\n")
		
		# Publish mode to NetworkTables so dashboard auto-detects
		SmartDashboard.putString("robot_mode", "Teleop")
		print("[MODE] Published robot_mode = Teleop to NetworkTables\n")
		
		# Track if robot had input in previous loop
		self.had_teleop_input = False
	
	def teleopPeriodic(self):
		"""Teleop control for swerve drive"""
		# Ensure robot_mode stays set to Teleop
		SmartDashboard.putString("robot_mode", "Teleop")
		
		# Execute joystick control logic
		self.pilot_controls.execute_teleop()
		
		# Update single-wheel alignment (handles per-wheel centering)
		self.drive.update_single_wheel_alignment()
		
		# SHOOTER CONTROLS
		self.shooter_controls.execute()
	
	def teleopExit(self):
		self.drive.stop_all()
		self.shooter.stop_all()
		print("[TELEOP] === EXITING TELEOP MODE ===\n")


if __name__ == "__main__":
	wpilib.run(Robot)
