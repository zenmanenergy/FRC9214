#!/usr/bin/env python3

import wpilib
import ctre
import rev
from wpilib import SmartDashboard
# import serial
# import edu.wpi.first.wpilibj.SPI;
# import edu.wpi.first.wpilibj.interfaces.Accelerometer;
# import edu.wpi.first.wpilibj.interfaces.Gyro;

class MyRobot(wpilib.TimedRobot):
    """
    This is a short sample program demonstrating how to use the basic throttle
    mode of the TalonSRX
    """

    def robotInit(self):
        self.gyro = wpilib.ADIS16470_IMU()
        wpilib.CameraServer.launch()
        
        self.ARM_EXTEND = 72
        self.ARM_HOME =0
        self.ARM_RETRACT = 46
        self.INTAKE_STATE = "Stopped"
        
        
        
        self.SMARTArmRotation = SmartDashboard.putString("State", "")
        self.SMARTArmRotation = SmartDashboard.putNumber("Wanted Position", 0)
        self.SMARTArmRotation = SmartDashboard.putNumber("Arm Position", 0)

        
        
        self.ArmJoystick = wpilib.Joystick(0)
        self.DriveJoystick = wpilib.Joystick(1)

        self.LeftFrontMotor = ctre.WPI_TalonSRX(1)
        self.LeftRearMotor = ctre.WPI_TalonSRX(2)

        self.RightFrontMotor = ctre.WPI_TalonSRX(3)
        self.RightRearMotor = ctre.WPI_TalonSRX(4)

        self.JackShaftMotor = rev.CANSparkMax(6,rev.CANSparkMax.MotorType.kBrushless)
        self.JackShaftMotor.setIdleMode(rev.CANSparkMax.IdleMode.kBrake)
        self.JackShaftMotor.setInverted(True)

        self.JackShaftEncoder = self.JackShaftMotor.getEncoder()
        # self.JackShaftEncoder.setPosition(0)
        self.CurrentPosition = self.JackShaftEncoder.getPosition()
        # self.controller = wpilib.PIDController(1, 0, 0, self.encoder, self.JackShaftMotor)

        self.IntakeMotor = rev.CANSparkMax(7,rev.CANSparkMax.MotorType.kBrushless)
        self.IntakeMotor.setSmartCurrentLimit(19) # set the maximum current limit to 20A
        self.IntakeMotor.setSecondaryCurrentLimit(25) # set the maximum current limit during current limit duration to 25A
        # self.IntakeMotor.setSmartCurrentDuration(100) # set the current limit duration to 100ms

        
        self.ArmState="unknown"
        self.WantedArmPosition = self.ARM_RETRACT
        self.ArmRunningToPosition = False
        self.MaxArmSpeed=1.0
        self.IntakeSpeed=-0.6
        self.ReleaseSpeed=0.6

        self.AutoTimer = wpilib.Timer()
        self.AutoDirection=""
        self.AutoTurnOffInSeconds=0

        # self.Openmv_Serial_Port = serial.Serial('/dev/ttyACM0', 9600)
        # self.Openmv_Serial_Port.flushInput()

        # self.OpenMV_Serial_Port = wpilib.SerialPort(9600, '/dev/ttyS0')

        # self.OpenMV_Serial_Port.flush()

        # led = wpilib.AddressableLED(0)
        # ledData = [wpilib.CANData(0, [0, 255, 0])] * 60
        # led.setData(ledData)

        

        if self.gyro.isConnected():
            print("start calibration")
            self.gyro.calibrate()
            
            print("calibrated")
            SmartDashboard.putData("Gyro", self.gyro);
    


            # SmartDashboard.putNumber("Gyro Angle", self.gyro.getAngle());
            # SmartDashboard.putNumber("Gyro Yaw Axis angular rate", self.gyro.getRate());
            # SmartDashboard.putNumber("Gyro Filtered Y", self.gyro.getYFilteredAccelAngle());
            # SmartDashboard.putNumber("Gyro Filtered X", self.gyro.getXFilteredAccelAngle());
    
            # self.controller = PIDController(0.1, 0.01, 0.1, source=self.gyro, output=self._update_motors)
        

    def disabledPeriodic(self):
        self.LeftFrontMotor.disable()
        self.LeftRearMotor.disable()
        self.RightFrontMotor.disable()
        self.RightRearMotor.disable()
        self.IntakeMotor.disable()
        self.JackShaftMotor.disable()


    def autonomousInit(self):
        self.AutoTimer.reset()
        self.AutoTimer.start()

    def autonomousPeriodic(self):
        if(self.AutoTimer.get() < 0.5):
            self.setDrives(0.5, 0.5)
        elif(self.AutoTimer.get() < 5):
            self.setDrives(-0.4, -0.4)
        else:
            self.setDrives(0, 0)

    
    def teleopPeriodic(self):

       
        
        # data = self.OpenMV_Serial_Port.readline().decode('utf-8').strip()

        # # If we received data, print it
        # if data:
        #     print(data)
        # data = self.OpenMV_Serial_Port.read(100)
        # if data is not None:
        #     print("Received data:", data)

        self.tiltPeriodic()
        self.JoystickPeriodic()
        self.DrivePeriodic()
        self.ArmPeriodic()
        self.IntakePeriodic()
        # self.AutoBalance()
            
    def AutoBalance(self,reset=False):
        SmartDashboard.putString("TIMER", "self.AutoTimer.get()");
        if reset:
            self.AutoTimer.reset()
            self.AutoTimer.start()
            self.AutoDirection="Backward"
            self.AutoTurnOffInSeconds=1

        if self.AutoDirection=="Forward":
            SmartDashboard.putString("DRIVE", "FORWARD");
            
            self.setDrives(-0.2, -0.2)
            if(self.AutoTimer.get() >= self.AutoTurnOffInSeconds):
                self.AutoTimer.reset()
                self.AutoTimer.start()
                self.AutoDirection="Backward"

        elif self.AutoDirection=="Backward":
            SmartDashboard.putString("DRIVE", "BACKWARD");
            self.setDrives(0.2, 0.2)
            if(self.AutoTimer.get() >= self.AutoTurnOffInSeconds):
                self.AutoDirection=""
        else:
            SmartDashboard.putString("DRIVE", "OFF");
            self.setDrives(0,0)
    
        
        
    


    def tiltPeriodic(self):
        # if self.gyro.getYFilteredAccelAngle() > 
        if self.AutoDirection=="":
            if self.gyro.getYFilteredAccelAngle() > 357 and self.gyro.getYFilteredAccelAngle() < 360:
                SmartDashboard.putString("Tilt", "FLAT");
            elif self.gyro.getYFilteredAccelAngle() > 0 and self.gyro.getYFilteredAccelAngle() < 10:
                SmartDashboard.putString("Tilt", "UP");
                
            elif self.gyro.getYFilteredAccelAngle() > 350 and self.gyro.getYFilteredAccelAngle() < 357:
                SmartDashboard.putString("Tilt", "DOWN");
        if self.gyro.isConnected():
            SmartDashboard.putNumber("Gyro Angle", self.gyro.getAngle());
            SmartDashboard.putNumber("Gyro Yaw Axis angular rate", self.gyro.getRate());
            SmartDashboard.putNumber("Gyro Filtered Y", self.gyro.getYFilteredAccelAngle());
            SmartDashboard.putNumber("Gyro Filtered X", self.gyro.getXFilteredAccelAngle());
    
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


        self.ARM_LEFT_THUMB_LEFTRIGHT = self.ArmJoystick.getRawAxis(0)
        self.ARM_LEFT_THUMB_UPDOWN = self.ArmJoystick.getRawAxis(1)
        self.ARM_RIGHT_THUMB_LEFTRIGHT = self.ArmJoystick.getRawAxis(4)
        self.ARM_RIGHT_THUMB_UPDOWN = self.ArmJoystick.getRawAxis(5)

        self.ARM_LEFT_TRIGGER=self.ArmJoystick.getRawAxis(2)
        self.ARM_RIGHT_TRIGGER=self.ArmJoystick.getRawAxis(3)

        self.ARM_A_Button=self.ArmJoystick.getRawButton(1)
        self.ARM_B_Button=self.ArmJoystick.getRawButton(2)
        self.ARM_X_Button=self.ArmJoystick.getRawButton(3)
        self.ARM_Y_Button=self.ArmJoystick.getRawButton(4)
        self.ARM_LB_Button=self.ArmJoystick.getRawButton(5)
        self.ARM_RB_Button=self.ArmJoystick.getRawButton(6)
        self.ARM_START_Button=self.ArmJoystick.getRawButton(8)

        if self.ARM_START_Button:
            self.JackShaftEncoder.setPosition(0)
        
    def DrivePeriodic(self):
        # Set the motor's output to half power.
        # This takes a number from -1 (100% speed in reverse) to +1 (100%
        # speed going forward)
        self.DriveSpeed=0.75

        # High Speed
        # Set the motor's output to FULL power.
        if self.DRIVE_RB_Button:
            self.DriveSpeed=0.4

        # Slow speed
        # Set the motor's output to 1/3 power.
        if self.DRIVE_RIGHT_TRIGGER>0:
            self.DriveSpeed=0.5

        # STOP button
        # Set the motor's output to ZERO power.
        if self.DRIVE_LB_Button:
            self.DriveSpeed=0
        # self.LeftFrontMotor.set(-1*self.DriveSpeed*self.DRIVE_LEFT_THUMB_UPDOWN)
        # self.LeftRearMotor.set(-1*self.DriveSpeed*self.DRIVE_LEFT_THUMB_UPDOWN)
        # self.RightFrontMotor.set(self.DriveSpeed*self.DRIVE_RIGHT_THUMB_UPDOWN)
        # self.RightRearMotor.set(self.DriveSpeed*self.DRIVE_RIGHT_THUMB_UPDOWN)

        self.setDrives(self.DriveSpeed*self.DRIVE_LEFT_THUMB_UPDOWN, self.DriveSpeed*self.DRIVE_RIGHT_THUMB_UPDOWN)

        # if self.DRIVE_Y_Button:
        #     self.AutoDirection="Forward"
        #     self.AutoBalance(True)

    def ArmPeriodic(self):
        self.CurrentPosition = self.JackShaftEncoder.getPosition()
        self.SMARTArmRotation = SmartDashboard.putNumber("Arm Position", self.CurrentPosition)
        
        self.ArmSpeed=0.2
            

        if self.ARM_A_Button:
            self.WantedArmPosition = self.ARM_RETRACT
            self.ArmRunningToPosition = True
        
        elif self.ARM_B_Button:
            self.WantedArmPosition = self.ARM_HOME
            self.ArmRunningToPosition = True

        elif self.ARM_Y_Button:
            self.WantedArmPosition = self.ARM_EXTEND 
            self.ArmRunningToPosition = True

        elif self.ARM_X_Button:
            self.ArmRunningToPosition = False
        
        if(self.ArmRunningToPosition):
            self.setArmPosition(self.WantedArmPosition)
        elif self.ArmState == "Upper Position":
            print("Upper Position")
        else:
            if self.ARM_RIGHT_TRIGGER > 0.05:
                self.JackShaftMotor.set(self.MaxArmSpeed*self.ARM_RIGHT_TRIGGER)
            elif self.ARM_LEFT_TRIGGER > 0.05:
                self.JackShaftMotor.set(-1*self.MaxArmSpeed*self.ARM_LEFT_TRIGGER)
            else:
                self.JackShaftMotor.set(0)


    def setArmPosition(self, WantedPosition: int):
        offset = 1
        
        if WantedPosition == self.ARM_EXTEND:
            self.ArmState="Upper Position"
        elif WantedPosition == self.ARM_HOME:
            self.ArmState="Lower Position"
        elif WantedPosition == self.ARM_RETRACT:
            self.ArmState="Home"
        else:
            self.ArmState="unknown"

        self.SMARTArmRotation = SmartDashboard.putString("Arm State", ArmState)
        self.SMARTArmRotation = SmartDashboard.putNumber("Wanted Position", WantedPosition)
        
        if(self.CurrentPosition > WantedPosition + offset):
            self.JackShaftMotor.set(-0.5)
        elif(self.CurrentPosition < WantedPosition - offset):
            self.JackShaftMotor.set(0.5)
        else:
            self.JackShaftMotor.set(0)
            # if (self.CurrentPosition < 10):
            #     self.JackShaftMotor.set(-0.01)
            # else:
            #     self.JackShaftMotor.set(0.03)
            self.ArmRunningToPosition = False

    def IntakePeriodic(self):
        self.SMARTArmRotation = SmartDashboard.putNumber("self.ARM_RIGHT_THUMB_UPDOWN", self.ARM_RIGHT_THUMB_UPDOWN)
        if self.ARM_RIGHT_THUMB_UPDOWN>0.05:
            self.INTAKE_STATE = "Intake"
        
        elif self.ARM_RIGHT_THUMB_UPDOWN<-0.05:
            self.INTAKE_STATE = "Release"
        
        else:
            self.INTAKE_STATE = "Stopped"
       

        # self.IntakeSpeed=0.3

        # Intake_Current=self.IntakeMotor.getOutputCurrent()
        # self.SMARTArmRotation = SmartDashboard.putNumber("Intake Current", Intake_Current)
        # self.SMARTArmRotation = SmartDashboard.putString("Intake State", self.INTAKE_STATE)
            
        # # if Intake_Current > 5:
        # #     self.IntakeMotor.set(0)
        # # el
        
        
        if self.INTAKE_STATE=="Intake":
            self.IntakeMotor.set(self.IntakeSpeed)
        elif self.INTAKE_STATE=="Release":
            self.IntakeMotor.set(self.ReleaseSpeed)
        else:
            self.IntakeMotor.set(0)
       

    def setDrives(self, LeftSpeed: float, RightSpeed: float):
        
        self.LeftFrontMotor.set(-LeftSpeed)
        self.LeftRearMotor.set(-LeftSpeed)
        self.RightFrontMotor.set(RightSpeed)
        self.RightRearMotor.set(RightSpeed)
        
    


if __name__ == "__main__":
    wpilib.run(MyRobot)


    



