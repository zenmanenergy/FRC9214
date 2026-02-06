import wpilib # Imports WPILIB library (FIRST Robotics robot base classes and hardware APIs)
from phoenix5 import WPI_TalonSRX, NeutralMode

from drive import Drive # Imports class "Drive" from drive.py
from odometry import Odometry # Imports class "Odometry" from odometry.py

class MyRobot(wpilib.TimedRobot):
	def robotInit(self):
		
		# Creates motor controller objects for each of the four drive motors. 
		left_front = WPI_TalonSRX(4)
		left_rear = WPI_TalonSRX(2)
		right_front = WPI_TalonSRX(3)
		right_rear = WPI_TalonSRX(1)

		# Inverts right-side motors so a positive speed makes both sides drive forward
		right_front.setInverted(True)
		right_rear.setInverted(True)

		# Sets all drive motors to brake when no power is applied (robot resists coasting)
		left_front.setNeutralMode(NeutralMode.Brake)
		left_rear.setNeutralMode(NeutralMode.Brake)
		right_front.setNeutralMode(NeutralMode.Brake)
		right_rear.setNeutralMode(NeutralMode.Brake)
		
		# Creates REV through-bore encoder objects for left and right sides
		# DIO 0-1 for left encoder, DIO 2-3 for right encoder
		left_encoder = wpilib.Encoder(0, 1)
		right_encoder = wpilib.Encoder(8,9)
		
		self.drive_joystick = wpilib.Joystick(0)

		# Create odometry first, then pass to drive
		self.odometry = Odometry(left_encoder, right_encoder, wheel_diameter_cm=15.24)
		self.drive = Drive(self.drive_joystick, left_front, left_rear, right_front, right_rear, self.odometry)
		self.Zeroed=False
		self.loop_count = 0
	
	def disabledInit(self):
		self.Zeroed=False
		self.drive.reset()
		
	def teleopInit(self):
		if not self.Zeroed:
			self.drive.reset()
			self.odometry.reset()
			self.Zeroed = True
	
	# Runs drive and arm control code every robot loop during teleop
	def teleopPeriodic(self):
		self.drive.periodic()
		self.odometry.update()
		
		# Check A button to reset odometry
		if self.drive_joystick.getRawButton(1):  # Button 1 is A button
			self.odometry.reset()
			print("Odometry reset!")
		
		# Check B button to drive to position (100, 0)
		if self.drive_joystick.getRawButton(2):  # Button 2 is B button
			self.drive.drive_to_position(100, 0)
		
		# Print position for debugging (every 50 loops = ~1 second)
		# self.loop_count += 1
		# if self.loop_count % 50 == 0:
		# 	self.odometry.print_debug_info()
		
if __name__ == "__main__":
	wpilib.run(MyRobot)

