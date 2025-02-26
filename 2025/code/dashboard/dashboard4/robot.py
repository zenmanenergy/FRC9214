import wpilib
from networktables import NetworkTables
from arm import Arm
from drive import Drive

class MyRobot(wpilib.TimedRobot):
	def robotInit(self):
		# Initialize NetworkTables
		# NetworkTables.initialize(server="roborio-9214-frc.local")
		NetworkTables.initialize(server="10.92.14.2")
		self.table = NetworkTables.getTable("robot_data")

		self.arm = Arm(self.table)
		self.drive = Drive(self.table)


		# Check if connected to NetworkTables
		if NetworkTables.isConnected():
			print("Connected to NetworkTables!")
		else:
			print("NetworkTables Connection Failed!")

		# Robot state (real values from sensors/motors)
		self.real_x_position = 0
		self.real_y_position = 0
		self.real_elevator_position = 0
		self.real_arm_angle = 0
		self.real_wrist_angle = 0
		self.real_grabber_angle = 0

		# Commanded state (received from dashboard)
		self.cmd_elevator_position = 0
		self.cmd_arm_angle = 0
		self.cmd_wrist_angle = 0
		self.cmd_grabber_angle = 0


		self.joystick = wpilib.Joystick(0)  # Using a standard joystick
		self.DriveJoystick = wpilib.Joystick(1)
		
		# Joystick:
		self.LeftThumbUPDOWN=0

		self.LeftThumbUPDOWN=0
		self.RightThumbUPDOWN=0
		self.RightThumbLEFTRIGHT=0
		self.LeftORRightTrigger=0
		


	def DrivePeriodic (self):
		self.DRIVE_LEFT_THUMB_UPDOWN = self.DriveJoystick.getRawAxis(1)*0.3
		self.DRIVE_RIGHT_THUMB_UPDOWN = self.DriveJoystick.getRawAxis(5)*0.3
		self.drive.periodic(self.DRIVE_LEFT_THUMB_UPDOWN, self.DRIVE_RIGHT_THUMB_UPDOWN)

	def JoyStickPeriodic(self):
		self.LeftThumbUPDOWN=self.joystick.getRawAxis(1)*-1  #reverse the direction, so up is positive
		if abs(self.LeftThumbUPDOWN) < 0.05:
			self.LeftThumbUPDOWN=0
		self.RightThumbUPDOWN=self.joystick.getRawAxis(5)*-1
		if abs(self.RightThumbUPDOWN) < 0.05:
			self.RightThumbUPDOWN=0
		self.RightThumbLEFTRIGHT=self.joystick.getRawAxis(4)
		if abs(self.RightThumbLEFTRIGHT) < 0.05:
			self.RightThumbLEFTRIGHT=0
		LeftTrigger=self.joystick.getRawAxis(2)*-1
		if abs(LeftTrigger) < 0.05:
			LeftTrigger=0
		RightTrigger=self.joystick.getRawAxis(3)
		if abs(RightTrigger) < 0.05:
			RightTrigger=0
		self.LeftORRightTrigger=0
		if LeftTrigger <0.1:
			self.LeftORRightTrigger=LeftTrigger
		if RightTrigger>0.1:
			self.LeftORRightTrigger=RightTrigger
			
		if abs(self.LeftORRightTrigger) < 0.05:
			self.LeftORRightTrigger=0

		# print(self.LeftThumbUPDOWN,self.RightThumbUPDOWN,self.RightThumbLEFTRIGHT,LeftTrigger,RightTrigger,self.LeftORRightTrigger)
		

	def teleopPeriodic(self):
		self.JoyStickPeriodic()

		self.DrivePeriodic()
		
		# Control motors
		self.arm.control_motors(self.LeftThumbUPDOWN,self.RightThumbUPDOWN,self.RightThumbLEFTRIGHT,self.LeftORRightTrigger)



		

		# Read dashboard commands
		self.cmd_elevator_position = self.table.getNumber("cmd_elevator", self.cmd_elevator_position)
		self.cmd_arm_angle = self.table.getNumber("cmd_arm_angle", self.cmd_arm_angle)
		self.cmd_wrist_angle = self.table.getNumber("cmd_wrist_angle", self.cmd_wrist_angle)
		self.cmd_grabber_angle = self.table.getNumber("cmd_grabber_angle", self.cmd_grabber_angle)

		# Print received commands
		print(f"CMD -> Elevator: {self.cmd_elevator_position}, Arm: {self.cmd_arm_angle}, Wrist: {self.cmd_wrist_angle}, Grabber: {self.cmd_grabber_angle}")

		# Apply commanded values to real robot movement
		# if abs(self.real_elevator_position - self.cmd_elevator_position) > 0.1:
		# 	self.real_elevator_position += (self.cmd_elevator_position - self.real_elevator_position) * 0.2
		# if abs(self.real_arm_angle - self.cmd_arm_angle) > 0.1:
		# 	self.real_arm_angle += (self.cmd_arm_angle - self.real_arm_angle) * 0.2
		# if abs(self.real_wrist_angle - self.cmd_wrist_angle) > 0.1:
		# 	self.real_wrist_angle += (self.cmd_wrist_angle - self.real_wrist_angle) * 0.2
		# if abs(self.real_grabber_angle - self.cmd_grabber_angle) > 0.1:
		# 	self.real_grabber_angle += (self.cmd_grabber_angle - self.real_grabber_angle) * 0.2

		# Get real-time data (real state)
		# self.arm.periodic(True)



		# Print real positions to confirm updates
		

if __name__ == "__main__":
	wpilib.run(MyRobot)
