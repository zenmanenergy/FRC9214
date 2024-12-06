import wpilib  # FIRST Robotics library
import rev  # Zippy arm motor controller library

class MyRobot(wpilib.TimedRobot):
	def robotInit(self):  # Define the controller, motors, and limit switch
		self.ArmJoystick = wpilib.Joystick(1)
		self.JackShaftMotor = rev.CANSparkMax(6, rev.CANSparkMax.MotorType.kBrushless)
		self.JackShaftMotor.setIdleMode(rev.CANSparkMax.IdleMode.kBrake)
		self.ArmSpeed = 0.5

		# Initialize encoder and limit switch
		self.encoder = self.JackShaftMotor.getEncoder()
		self.limit_switch = wpilib.DigitalInput(0)  # Assume DIO port 0 for the limit switch

		# State variables
		self.retracting = False


	def autonomousInit(self):
		return False
	
	def autonomousPeriodic(self):
		return False
	
	def teleopInit(self):
		return False
	
	def teleopPeriodic(self):
		self.JoystickPeriodic()
		self.ArmPeriodic()
		self.startButtonCheck()  # Check the start button during teleop

	def JoystickPeriodic(self):
		self.ARM_LEFT_THUMB_UPDOWN = self.ArmJoystick.getRawAxis(1)
		self.ARM_START = self.ArmJoystick.getRawButton(8)
		
	def ArmPeriodic(self):
		if not self.retracting:  # Allow manual control only if not retracting
			if self.ARM_LEFT_THUMB_UPDOWN > 0.05 or self.ARM_LEFT_THUMB_UPDOWN < -0.05:
				self.JackShaftMotor.set(1 * self.ArmSpeed * self.ARM_LEFT_THUMB_UPDOWN)
			else:
				self.JackShaftMotor.set(0)

	def startButtonCheck(self):
		# Check if the start button (button 8 here) is pressed
		if self.ARM_START and not self.retracting:
			print("Start button pressed: Retracting arm.")
			self.retracting = True  # Begin retracting process

		# If the arm is retracting, run this process
		if self.retracting:
			if self.limit_switch.get():  # If limit switch is not pressed, continue retracting
				self.JackShaftMotor.set(0.3)  # Set a slow retraction speed
			else:
				print("Limit switch activated. Stopping motor and resetting encoder.")
				self.JackShaftMotor.set(0)  # Stop the motor
				self.encoder.setPosition(0)  # Reset encoder to zero
				self.retracting = False  # End retraction process

if __name__ == "__main__":
	wpilib.run(MyRobot)
