"""FRC Swerve Drive Test Robot - Encoder Calibration"""
import wpilib
from wpilib import Joystick, SmartDashboard
from swerve_drive import SwerveDrive
from dashboard_updater import DashboardUpdater
import swerve_config as config


class Robot(wpilib.TimedRobot):
	def robotInit(self):
		self.drive = SwerveDrive()
		self.joystick = Joystick(0)
		self.dashboard = DashboardUpdater(self.drive)
		
		# Test state
		self.focused = None
		self.last_focused_wheel = None  # Track last focused from web dashboard
		self.last_printed_angle = -999
		
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
	
	def testPeriodic(self):
		# Read control commands from NetworkTables (from web dashboard)
		focused_wheel = SmartDashboard.getString("focused_wheel", "")
		align_command = SmartDashboard.getBoolean("align_command", False)
		save_zero_command = SmartDashboard.getBoolean("save_zero_command", False)
		calibrate_wheel = SmartDashboard.getString("calibrate_wheel", "")
		calibrate_angle = SmartDashboard.getNumber("calibrate_angle", -999)
		set_wheels_direction_str = SmartDashboard.getString("set_wheels_direction", "")
		
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
		
		# Handle set_wheels_direction command
		if set_wheels_direction_str:
			try:
				import ast
				angles_dict = ast.literal_eval(set_wheels_direction_str)
				for wheel, angle in angles_dict.items():
					self.drive.set_wheel_angle(wheel, int(angle))
				SmartDashboard.putString("set_wheels_direction", "")
				print(f"[DIRECTION] All wheels set\n")
			except Exception as e:
				print(f"[ERROR] Failed to parse set_wheels_direction: {e}")
		
		# Detect if focus changed from web dashboard and stop previous wheel
		if focused_wheel != self.last_focused_wheel:
			self.drive.stop_all()
			self.last_focused_wheel = focused_wheel
		
		# Handle align command
		if align_command:
			self.drive.start_alignment()
			SmartDashboard.putBoolean("align_command", False)
		
		# Update alignment if active
		self.drive.update_alignment()
		
		# Handle calibration command
		if calibrate_wheel and calibrate_angle >= 0:
			self.drive.set_wheel_angle(calibrate_wheel, int(calibrate_angle))
			SmartDashboard.putString("calibrate_wheel", "")
			SmartDashboard.putNumber("calibrate_angle", -999)
			print(f"[ZEROING] Calibration saved\n")
		
		# Handle save zero command
		if save_zero_command and focused_wheel:
			self.drive.set_wheel_zero(focused_wheel)
			self.drive.stop_all()
			SmartDashboard.putBoolean("save_zero_command", False)
			SmartDashboard.putString("focused_wheel", "")
			self.last_focused_wheel = None
			print(f"[ZEROING] Saved zero for {focused_wheel}\n")
		
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
