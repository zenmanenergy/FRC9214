"""
Minimal FRC Robot with YDLidar GS2 Integration
Pattern: robot.py instantiates Lidar in robotInit(), calls lidar.update() in teleopPeriodic()
"""

import wpilib
from lidar import GS2Lidar


class LidarRobot(wpilib.TimedRobot):
    """Minimal robot for LIDAR testing and integration"""
    
    def robotInit(self):
        """Initialize robot and subsystems"""
        self.lidar = GS2Lidar()
        
        # Joystick for testing
        self.joystick = wpilib.Joystick(0)
        
        # Test state
        self.last_print = 0
        
    def robotPeriodic(self):
        """Called every cycle"""
        pass
    
    def disabledInit(self):
        """Called when robot enters disabled state"""
        self.lidar.stop_scan()
    
    def disabledPeriodic(self):
        """Run during disabled"""
        pass
    
    def teleopInit(self):
        """Called at start of teleop"""
        if not self.lidar.connected:
            self.lidar.connect()
        
        if not self.lidar.scanning:
            self.lidar.start_scan()
    
    def teleopPeriodic(self):
        """Main robot loop - update subsystems"""
        
        # Update LIDAR data
        self.lidar.update()
        
        # Get joystick button inputs
        if self.joystick.getRawButtonPressed(1):  # A button
            print("[LIDAR] Resetting...")
            self.lidar.disconnect()
            self.lidar.connect()
            self.lidar.start_scan()
        
        if self.joystick.getRawButtonPressed(2):  # B button
            self.lidar.print_debug()
        
        # Periodic debug output
        current_time = wpilib.Timer.getFPGATimestamp()
        if current_time - self.last_print > 1.0:  # Print every 1 second
            self.lidar.print_debug()
            front = self.lidar.get_front_distance()
            left_obs = self.lidar.get_obstacle_left()
            right_obs = self.lidar.get_obstacle_right()
            
            print(f"[ROBOT] Front: {front}mm, Left: {left_obs}, Right: {right_obs}")
            self.last_print = current_time
    
    def testInit(self):
        """Called at start of test mode"""
        pass
    
    def testPeriodic(self):
        """Called during test mode"""
        pass


if __name__ == "__main__":
    wpilib.run(LidarRobot)
