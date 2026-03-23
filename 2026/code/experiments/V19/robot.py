"""FRC Swerve Drive Test Robot"""
import wpilib
from wpilib import SmartDashboard, DriverStation
from swerve.swerve_drive import SwerveDrive
from pilotJoystick import PilotJoystick
from pilot_controls import PilotControls
from dashboard.dashboard_updater import DashboardUpdater
from dashboard.calibration_mode_handler import CalibrationModeHandler
from swerve.encoder_calibration import EncoderCalibration
import swerve.swerve_config as config
from swerve import CANID


class Robot(wpilib.TimedRobot):
	def robotInit(self):
		self.swervedrive = SwerveDrive()
		
		self.calibration = EncoderCalibration()
		
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
		SmartDashboard.putString("robot_mode", "Teleop")
	
	def teleopPeriodic(self):
		self.pilot_controls.execute_teleop()
		self.swervedrive.odometry.update()
		self.swervedrive.update_single_wheel_alignment()
	
	def teleopExit(self):
		self.swervedrive.stop_all()


if __name__ == "__main__":
	wpilib.run(Robot)
