import wpilib
from rev import SparkMax, SparkLowLevel


class ShooterSubsystem:
	def __init__(self):
		super().__init__()
		
		# Initialize motor controllers
		self.uptake_motor = SparkMax(12, SparkLowLevel.MotorType.kBrushless)
		self.turret_motor = SparkMax(14, SparkLowLevel.MotorType.kBrushless)
		self.shooter_motor = SparkMax(16, SparkLowLevel.MotorType.kBrushless)
		
		# Invert uptake motor
		self.uptake_motor.setInverted(False)
	
	def set_uptake(self, speed):
		"""Set uptake motor speed"""
		self.uptake_motor.set(speed)
	
	def set_turret(self, speed):
		"""Set turret motor speed"""
		#self.turret_motor.set(speed)
		pass
	
	def set_shooter(self, speed):
		"""Set shooter motor speed"""
		self.shooter_motor.set(speed)
	
	def stop_all(self):
		"""Stop all motors"""
		self.uptake_motor.set(0)
		self.turret_motor.set(0)
		self.shooter_motor.set(0)
