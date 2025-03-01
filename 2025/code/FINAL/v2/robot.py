import wpilib
from networktables import NetworkTables
from arm import Arm
from DriveNETWORKTABLES import Drive
from arm_calibration import ArmCalibration
from Arm_Presets import ArmPresets

class MyRobot(wpilib.TimedRobot):
	def robotInit(self):
		# Initialize NetworkTables
		# NetworkTables.initialize(server="roborio-9214-frc.local")
		NetworkTables.initialize(server="10.92.14.2")
		self.table = NetworkTables.getTable("robot_data")

		self.arm = Arm(self.table)
		self.drive = Drive(self.table)
		self.calibration = ArmCalibration()
		self.ap = ArmPresets(self.arm, self.table)

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


		self.ArmJoystick = wpilib.Joystick(0)  # Using a standard joystick
		self.DriveJoystick = wpilib.Joystick(1)
		
		# Joystick:
		self.LeftThumbUPDOWN=0

		self.LeftThumbUPDOWN=0
		self.RightThumbUPDOWN=0
		self.RightThumbLEFTRIGHT=0
		self.LeftORRightTrigger=0

		self.Zeroed=False

		self.speed = 1
		self.timer = wpilib.Timer()
		self.driveStep = 0

		
	def disabledInit(self):
		self.Zeroed=False
		self.arm.target_elevator = None
		self.arm.target_wrist = None
		self.arm.target_shoulder = None

		
		

	def teleopInit(self):
		if not self.Zeroed:
			self.arm.resetEncoders()  # Reset encoders once
			self.drive.reset()
			self.Zeroed = True

	def autonomousInit(self):
		if not self.Zeroed:
			self.arm.resetEncoders()  # Reset encoders once
			self.drive.reset()
			self.Zeroed = True
		# self.drive.start_travel(100)

		self.driveStep = 0


		self.ap.gotopreset("load")

			
		


	def autonomousPeriodic(self):

		# self.drive.update_travel()
		if self.driveStep == 0:
			if self.drive_forward_for_time(-0.4, 2):
				self.driveStep += 1

		


		

		return super().autonomousPeriodic()


	def drive_forward_for_time(self, speed, duration):
		if self.timer.get() == 0:
			self.timer.start()
			self.drive.left_front.set(-speed)
			self.drive.left_rear.set(-speed)
			self.drive.right_front.set(speed)
			self.drive.right_rear.set(speed)

		if self.timer.get() >= duration:
			self.drive.left_front.set(0)
			self.drive.left_rear.set(0)
			self.drive.right_front.set(0)
			self.drive.right_rear.set(0)
			self.timer.stop()
			self.timer.reset()
			return True

		return False



	def JoyStickPeriodic(self):
		self.LeftThumbUPDOWN=self.ArmJoystick.getRawAxis(1)*-1  #reverse the direction, so up is positive
		if abs(self.LeftThumbUPDOWN) < 0.05:
			self.LeftThumbUPDOWN=0
		self.RightThumbUPDOWN=self.ArmJoystick.getRawAxis(5)*-1
		if abs(self.RightThumbUPDOWN) < 0.05:
			self.RightThumbUPDOWN=0
		self.RightThumbLEFTRIGHT=self.ArmJoystick.getRawAxis(4)
		if abs(self.RightThumbLEFTRIGHT) < 0.05:
			self.RightThumbLEFTRIGHT=0
		LeftTrigger=self.ArmJoystick.getRawAxis(2)*-1
		if abs(LeftTrigger) < 0.05:
			LeftTrigger=0
		RightTrigger=self.ArmJoystick.getRawAxis(3)
		if abs(RightTrigger) < 0.05:
			RightTrigger=0
		self.LeftORRightTrigger=0
		if LeftTrigger <0.1:
			self.LeftORRightTrigger=LeftTrigger
		if RightTrigger>0.1:
			self.LeftORRightTrigger=RightTrigger
			

		self.drive.update_travel()


		if abs(self.LeftORRightTrigger) < 0.05:
			self.LeftORRightTrigger=0

		# print(self.LeftThumbUPDOWN,self.RightThumbUPDOWN,self.RightThumbLEFTRIGHT,LeftTrigger,RightTrigger,self.LeftORRightTrigger)
		self.StartButton = self.ArmJoystick.getRawButton(8)  # Start button
		self.AButton = self.ArmJoystick.getRawButton(1)  # A button
		self.LBButton = self.ArmJoystick.getRawButton(5)  # LB button

		self.YButton = self.ArmJoystick.getRawButton(4)
		self.XButton = self.ArmJoystick.getRawButton(3)
		self.BButton = self.ArmJoystick.getRawButton(2)

		if self.AButton:
			self.ap.gotopreset("load")
			print("load")
		if self.YButton:
			if self.drive.traveling == False:
				self.drive.start_travel(100)
			self.ap.gotopreset("load")
			print("L4")
		# if self.XButton:
		# 	self.ap.gotopreset("L3")
		# 	print("L3")
		# if self.BButton:
		# 	self.ap.gotopreset("L2")
		# 	print("L2")



		self.DRIVE_LEFT_THUMB_UPDOWN = self.DriveJoystick.getRawAxis(1)*0.5
		self.DRIVE_RIGHT_THUMB_UPDOWN = self.DriveJoystick.getRawAxis(5)*0.5

		self.DRIVE_A_Button = self.DriveJoystick.getRawButton(1)

	def teleopPeriodic(self):
		self.JoyStickPeriodic()

		# self.print_arm_values()
		self.drive.set_motors(self.DRIVE_LEFT_THUMB_UPDOWN, self.DRIVE_RIGHT_THUMB_UPDOWN)
		
		# # Control motors
		self.arm.control_elevator(self.LeftThumbUPDOWN)
		self.arm.control_shoulder(self.RightThumbUPDOWN)
		self.arm.control_wrist(self.RightThumbLEFTRIGHT)
		self.arm.control_grabber(self.LeftORRightTrigger)



		

		# Read dashboard commands
		self.cmd_elevator_position = self.table.getNumber("cmd_elevator", self.cmd_elevator_position)
		self.cmd_arm_angle = self.table.getNumber("cmd_arm_angle", self.cmd_arm_angle)
		self.cmd_wrist_angle = self.table.getNumber("cmd_wrist_angle", self.cmd_wrist_angle)
		self.cmd_grabber_angle = self.table.getNumber("cmd_grabber_angle", self.cmd_grabber_angle)

		# Print received commands
		# print(f"CMD -> Elevator: {self.cmd_elevator_position}, Arm: {self.cmd_arm_angle}, Wrist: {self.cmd_wrist_angle}, Grabber: {self.cmd_grabber_angle}")

		

		# Get real-time data (real state)
		self.arm.periodic(debug=True)
		self.arm.update_movement()
		# if self.StartButton and self.AButton and self.LBButton and self.calibration.state == "idle":
		# 	print("[CALIBRATION] Triggered!")
		# 	self.calibration.start_calibration(self.arm)

		# # Update calibration process (non-blocking)
		# self.calibration.update(self.ArmJoystick, self.arm)

	def print_arm_values(self):
		"""Print the real-time values of the elevator, arm, and wrist when A is pressed on DriveJoystick."""
		if self.DRIVE_A_Button:  # A button on DriveJoystick (Joystick 1)
			# Ensure values are updated before printing
			self.arm.periodic(debug=False)  

			# Retrieve values from the arm instance
			elevator_position = (self.arm.elevator_encoder.getPosition() - self.arm.elevator_zero_offset) * self.arm.elevator_cm_per_tick
			arm_angle = (self.arm.shoulder_encoder.getPosition() - self.arm.shoulder_zero_offset) * self.arm.shoulder_deg_per_tick
			wrist_angle = (self.arm.wrist_encoder.getPosition() - self.arm.wrist_zero_offset) * self.arm.wrist_deg_per_tick

			print(f"Elevator: {elevator_position:.2f} cm, "
				f"Arm: {arm_angle:.2f} degrees, "
				f"Wrist: {wrist_angle:.2f} degrees")


		

if __name__ == "__main__":
	wpilib.run(MyRobot)
