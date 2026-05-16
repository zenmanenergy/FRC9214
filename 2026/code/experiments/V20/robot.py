"""FRC Swerve Drive Test Robot"""
import wpilib
from wpilib import SmartDashboard, DriverStation
from swerve.swerve_drive import SwerveDrive
from pilotJoystick import PilotJoystick
from pilot_controls import PilotControls
from dashboard.dashboard_updater import DashboardUpdater
from dashboard.calibration_mode_handler import CalibrationModeHandler
from swerve.encoder_calibration import EncoderCalibration

class Robot(wpilib.TimedRobot):
	def robotInit(self):
		self.swervedrive = SwerveDrive()
		
		self.pilot_joystick = PilotJoystick(port=0, deadband=0.1)
		self.pilot_controls = PilotControls(self.swervedrive, self.pilot_joystick)
		
		self.dashboard = DashboardUpdater(self.swervedrive)
		
		self.calibration_mode_handler = CalibrationModeHandler(self.swervedrive, self.pilot_controls)
		
	def robotPeriodic(self):
		self.dashboard.update()
		SmartDashboard.putBoolean("Robot Enabled", DriverStation.isEnabled())
	
	def testInit(self):
		self.calibration_mode_handler.test_init()
		
	def testPeriodic(self):
		self.calibration_mode_handler.test_periodic()
	def testExit(self):
		self.calibration_mode_handler.test_exit()
	
	def teleopInit(self):
		self.swervedrive.stop_all()
		self.swervedrive.aligning = False
		self.swervedrive.wheel_alignment_state.clear()
		self.swervedrive.imu.zero_heading()
		self.swervedrive.odometry.reset()
		SmartDashboard.putString("robot_mode", "Teleop")
	
	def teleopPeriodic(self):
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
		
		print(f"[ODO] X={x:.1f} Y={y:.1f} | IMU Raw Yaw={self.swervedrive.imu.ahrs.getYaw():.1f}° IMU(0-360)={imu_heading:.1f}° Wheel={wheel_heading:.1f}° Delta={wheel_delta:.1f}° Invert={self.swervedrive.imu.invert}", flush=True)
		
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
