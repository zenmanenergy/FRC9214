"""FRC Swerve Drive Test Robot"""
import wpilib
from wpilib import SmartDashboard, DriverStation
from swerve.swerve_drive import SwerveDrive
from pilotJoystick import PilotJoystick
from pilot_controls import PilotControls
from dashboard.dashboard_updater import DashboardUpdater
from dashboard.calibration_mode_handler import CalibrationModeHandler
from waypoint_navigator import WaypointNavigator
from swerve.encoder_calibration import EncoderCalibration

class Robot(wpilib.TimedRobot):
	def robotInit(self):
		self.swervedrive = SwerveDrive()
		
		self.pilot_joystick = PilotJoystick(port=0, deadband=0.1)
		self.pilot_controls = PilotControls(self.swervedrive, self.pilot_joystick)
		
		self.dashboard = DashboardUpdater(self.swervedrive)
		
		self.navigator = WaypointNavigator(self.swervedrive)
		
		self.calibration_mode_handler = CalibrationModeHandler(self.swervedrive, self.pilot_controls, self.navigator)
		
	def robotPeriodic(self):
		self.dashboard.update()
		SmartDashboard.putBoolean("Robot Enabled", DriverStation.isEnabled())
	
	def autonomousInit(self):
		"""Called at start of autonomous"""
		self.navigator.stop()  # Ensure clean state
		self.swervedrive.stop_all()
		SmartDashboard.putString("robot_mode", "Autonomous")
	
	def testInit(self):
		self.calibration_mode_handler.test_init()
		
	def testPeriodic(self):
		self.calibration_mode_handler.test_periodic()
	def testExit(self):
		self.calibration_mode_handler.test_exit()
	
	def disabledInit(self):
		"""Called when robot is disabled"""
		self.navigator.stop()
		self.swervedrive.stop_all()
	
	def teleopInit(self):
		self.swervedrive.stop_all()
		self.navigator.stop()  # Ensure navigator is reset on teleop start
		self.swervedrive.aligning = False
		self.swervedrive.wheel_alignment_state.clear()
		self.swervedrive.imu.zero_heading()
		self.swervedrive.odometry.reset()
		SmartDashboard.putString("robot_mode", "Teleop")
	
	def teleopPeriodic(self):
		# Check for waypoint route command from dashboard
		if SmartDashboard.getBoolean("navigate_waypoints_command", False):
			SmartDashboard.putBoolean("navigate_waypoints_command", False)
			waypoints_json = SmartDashboard.getString("navigation_waypoints_json", "[]")
			loop = SmartDashboard.getBoolean("navigation_loop", False)
			use_spline = SmartDashboard.getBoolean("navigation_use_spline", False)
			try:
				import json
				waypoints = json.loads(waypoints_json)
				if waypoints:
					self.navigator.loop = loop
					self.navigator.set_waypoints(waypoints, use_spline=use_spline)
					self.navigator.start()
					print(f"[ROBOT] Navigation started: {len(waypoints)} waypoints loop={loop} spline={use_spline}", flush=True)
			except Exception as e:
				print(f"[ROBOT] Navigation parse error: {e}", flush=True)
		
		if SmartDashboard.getBoolean("stop_navigation_command", False):
			self.navigator.stop()
			SmartDashboard.putBoolean("stop_navigation_command", False)
			print("[ROBOT] Navigation stopped by dashboard", flush=True)
		
		# Update navigator
		self.navigator.update()
		
		# Only accept pilot input if not navigating
		if not self.navigator.is_active:
			self.pilot_controls.execute_teleop()
		
		self.swervedrive.odometry.update()
		
		# Use IMU heading directly (100% IMU, no wheel kinematics blending)
		if self.swervedrive.imu.is_ready():
			self.swervedrive.odometry.set_heading(self.swervedrive.imu.get_heading())
		
		# Debug logging
		imu_heading = self.swervedrive.imu.get_heading()
		wheel_heading = self.swervedrive.odometry.get_heading()
		wheel_delta = self.swervedrive.odometry.get_last_heading_delta()
		x, y = self.swervedrive.odometry.get_position()
		
		#print(f"[ODO] X={x:.1f} Y={y:.1f} | IMU Raw Yaw={self.swervedrive.imu.ahrs.getYaw():.1f}° IMU(0-360)={imu_heading:.1f}° Wheel={wheel_heading:.1f}° Delta={wheel_delta:.1f}° Invert={self.swervedrive.imu.invert}", flush=True)
		
		self.swervedrive.update_single_wheel_alignment()
		if SmartDashboard.getBoolean("reset_odometry_command", False):
			self.swervedrive.imu.zero_heading()
			self.swervedrive.odometry.reset()
			SmartDashboard.putBoolean("reset_odometry_command", False)
			print("[ODO] Odometry reset + IMU zeroed", flush=True)
		if SmartDashboard.getBoolean("zero_imu_command", False):
			self.swervedrive.imu.zero_heading()
			print("[IMU] Heading zeroed", flush=True)
			SmartDashboard.putBoolean("zero_imu_command", False)
		if SmartDashboard.getBoolean("toggle_imu_invert_command", False):
			self.swervedrive.imu.invert = not self.swervedrive.imu.invert
			print(f"[IMU] Invert toggled to {self.swervedrive.imu.invert}", flush=True)
			SmartDashboard.putBoolean("toggle_imu_invert_command", False)
	
	def teleopExit(self):
		self.swervedrive.stop_all()


if __name__ == "__main__":
	wpilib.run(Robot)
