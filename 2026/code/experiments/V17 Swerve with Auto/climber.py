

import wpilib
from rev import SparkMax, SparkFlex, SparkLowLevel
import CANID


class Climber:
	def __init__(self):
		super().__init__()
		
		# Initialize motor controllers
		try:
			self.climber_motor = SparkMax(CANID.CLIMBER, SparkLowLevel.MotorType.kBrushless)
			print(f"[SHOOTER] Climber motor initialized on CAN ID {CANID.CLIMBER}", flush=True)
		except Exception as e:
			print(f"[SHOOTER] ERROR - Failed to initialize climber motor (CAN ID {CANID.CLIMBER}): {type(e).__name__}: {e}", flush=True)
			self.climber_motor = None
		
		
	
	def set_climber(self, speed):
		"""Set climber motor speed (very slow - 5% max recommended)"""
		if self.climber_motor:
			self.climber_motor.set(speed)
		#	print("climber speed", speed)
		#else:
		#	print("climber motor does not exist")
	