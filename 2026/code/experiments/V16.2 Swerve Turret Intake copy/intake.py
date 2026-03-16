"""
Intake subsystem control for the 2026 FRC robot.
Handles the intake motor on CANID 4.
"""

from rev import SparkMax, SparkLowLevel
from CANID import INTAKE


class Intake:
	"""Controls the intake motor"""
	
	def __init__(self):
		"""Initialize the intake motor using CANID constant."""
		self.motor = SparkMax(INTAKE, SparkLowLevel.MotorType.kBrushless)
		self.speed = 0.0
	
	def turn_on(self, speed=1.0):
		"""
		Turn on the intake motor.
		
		Args:
			speed: Motor speed from -1.0 to 1.0 (default: 1.0)
		"""
		self.speed = speed
		self.motor.set(-speed)
	
	def turn_off(self):
		"""Turn off the intake motor"""
		self.speed = 0.0
		self.motor.set(0.0)
	
	def set_speed(self, speed):
		"""
		Set the intake motor speed.
		
		Args:
			speed: Motor speed from -1.0 to 1.0
		"""
		self.speed = speed
		self.motor.set(speed)
	
	def stop(self):
		"""Stop the intake motor"""
		self.turn_off()
	
	def get_speed(self):
		"""Get the current motor speed"""
		return self.speed
