import wpilib

from shooter import ShooterSubsystem
from copilotJoystick import CopilotJoystick
from controls import ShooterControls


class Robot(wpilib.TimedRobot):
	def robotInit(self):
		"""Robot initialization"""
		# Create subsystems
		self.shooter_subsystem = ShooterSubsystem()
		
		# Create joystick
		self.copilot_joystick = CopilotJoystick(port=0, deadband=0.1)
		
		# Setup control bindings
		self.controls = ShooterControls(self.shooter_subsystem, self.copilot_joystick)
	
	def disabledInit(self):
		"""Called when robot is disabled"""
		self.shooter_subsystem.stop_all()
	
	def teleopPeriodic(self):
		"""Called during teleop period"""
		self.controls.execute()


if __name__ == "__main__":
	wpilib.run(Robot)
