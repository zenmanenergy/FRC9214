import wpilib


class DriverJoystick:
	"""Wrapper for driver controller with deadband."""

	DEADBAND = 0.25
	FORWARD_AXIS = 1  # Left stick Y
	ROTATION_AXIS = 4  # Right stick X

	def __init__(self, port: int = 0):
		"""Initialize driver joystick."""
		self.joystick = wpilib.Joystick(port)
		self.last_forward = None
		self.last_rotation = None

	def get_forward(self) -> float:
		"""Get forward/backward speed with deadband applied."""
		# Negate because forward is negative on controller Y axis
		value = -self.joystick.getRawAxis(self.FORWARD_AXIS)
		if abs(value) > self.DEADBAND:
			if value != self.last_forward:
				print("Forward: %.3f" % value)
				self.last_forward = value
			return value
		if self.last_forward != 0.0:
			print("Forward: 0.0")
			self.last_forward = 0.0
		return 0.0
		
	def get_rotation(self) -> float:
		"""Get rotation speed with deadband applied."""
		value = self.joystick.getRawAxis(self.ROTATION_AXIS)
		if abs(value) > self.DEADBAND:
			if value != self.last_rotation:
				print("Rotation: %.3f" % value)
				self.last_rotation = value
			return value
		if self.last_rotation != 0.0:
			print("Rotation: 0.0")
			self.last_rotation = 0.0
		return 0.0
	
	def get_y_button(self) -> bool:
		"""Get Y button press (button 4 on Xbox controller)."""
		return self.joystick.getRawButton(4)
