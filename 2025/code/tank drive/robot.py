import wpilib
import ctre
import rev

class MyRobot(wpilib.TimedRobot):

    def robotInit(self):
        self.DriveJoystick = wpilib.Joystick(0)
        self.LeftFrontMotor = ctre.WPI_TalonSRX(1)
        self.LeftRearMotor = ctre.WPI_TalonSRX(2)
        
        self.RightFrontMotor = ctre.WPI_TalonSRX(3)
        self.RightRearMotor = ctre.WPI_TalonSRX(4)


    def autonomousInit(self) -> None:
        return super().autonomousInit()

    def autonomousPeriodic(self) -> None:
        return super().autonomousPeriodic()
    
    def teleopPeriodic(self):
        self.JoystickPeriodic()
        self.driveperiodic()
    
    def JoystickPeriodic(self):
        self.DRIVE_LEFT_THUMB_LEFTRIGHT = self.DriveJoystick.getRawAxis(0)
        self.DRIVE_LEFT_THUMB_UPDOWN = self.DriveJoystick.getRawAxis(1)
        self.DRIVE_RIGHT_THUMB_LEFTRIGHT = self.DriveJoystick.getRawAxis(4)
        self.DRIVE_RIGHT_THUMB_UPDOWN = self.DriveJoystick.getRawAxis(5)

        self.DRIVE_LEFT_TRIGGER=self.DriveJoystick.getRawAxis(2)
        self.DRIVE_RIGHT_TRIGGER=self.DriveJoystick.getRawAxis(3)

        self.DRIVE_A_Button=self.DriveJoystick.getRawButton(1)
        self.DRIVE_B_Button=self.DriveJoystick.getRawButton(2)
        self.DRIVE_X_Button=self.DriveJoystick.getRawButton(3)
        self.DRIVE_Y_Button=self.DriveJoystick.getRawButton(4)
        self.DRIVE_LB_Button=self.DriveJoystick.getRawButton(5)
        self.DRIVE_RB_Button=self.DriveJoystick.getRawButton(6)
    
    def driveperiodic(self):
        self.DriveSpeed=1.0
        self.setDrives(self.DriveSpeed*self.DRIVE_LEFT_THUMB_UPDOWN, self.DriveSpeed*self.DRIVE_RIGHT_THUMB_UPDOWN)

    def setDrives(self, LeftSpeed: float, RightSpeed: float):
        
        self.LeftFrontMotor.set(-LeftSpeed)
        self.LeftRearMotor.set(-LeftSpeed)
        self.RightFrontMotor.set(RightSpeed)
        self.RightRearMotor.set(RightSpeed)
        
if __name__ == "__main__":
    wpilib.run(MyRobot)