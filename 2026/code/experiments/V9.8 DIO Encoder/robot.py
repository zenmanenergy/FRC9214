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
		print("[TEST] Controls available via web dashboard at http://localhost:9000")
		print("[TEST] Or use joystick:")
		print("[TEST] A = Focus rear left | B = Focus rear right | X = Focus front left | Y = Focus front right")
		print("[TEST] Right thumb left/right = Rotate wheel")
		print("[TEST] RB = Set this position as zero offset")
		print("[TEST] START = Align all wheels to 0\n")
	
	def robotPeriodic(self):
		# Update all dashboard values
		self.dashboard.update()
	
	def testPeriodic(self):
		# Read control commands from NetworkTables (from web dashboard)
		focused_wheel = SmartDashboard.getString("focused_wheel", "")
		align_command = SmartDashboard.getBoolean("align_command", False)
		save_zero_command = SmartDashboard.getBoolean("save_zero_command", False)
		
		# Print commands received from browser
		if focused_wheel != self.last_focused_wheel:
			print(f"[BROWSER CMD] focused_wheel: {focused_wheel}")
		if align_command:
			print(f"[BROWSER CMD] align_command: True")
		if save_zero_command:
			print(f"[BROWSER CMD] save_zero_command: True")
		
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
		
		# RB button - save zero offset
		if self.joystick.getRawButtonPressed(6):
			if self.focused:
				self.drive.set_wheel_zero(self.focused)
				self.drive.stop_all()
				self.focused = None
				SmartDashboard.putString("focused_wheel", "")
				print("[ZEROING] Focused off. Motor stopped.\n")
			else:
				print("[ZEROING] Must be focused first (press A/B/X/Y to focus)")
	
	def testExit(self):
		self.drive.stop_all()
		print("[TEST] === EXITING TEST MODE ===\n")


if __name__ == "__main__":
	wpilib.run(Robot)
