from wpimath.geometry import Rotation2d


class SwerveModulePosition:
	"""Represents the position of a swerve module (distance and angle)."""
	
	def __init__(self, distance_m: float = 0.0, angle: Rotation2d = None):
		"""
		Initialize module position.
		
		Args:
			distance_m: Distance traveled in meters
			angle: Wheel angle
		"""
		self.distance = distance_m
		self.angle = angle if angle is not None else Rotation2d()
