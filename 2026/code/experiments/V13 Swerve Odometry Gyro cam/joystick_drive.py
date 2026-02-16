import wpilib


# ==============================================================================
# DRIVER JOYSTICK CONFIGURATION
# ==============================================================================


class DriverJoystickConfig:
	"""Driver joystick button and axis mappings."""

	# Port number
	PORT = 0

	# Axes
	FORWARD_AXIS = 1  # Left stick Y
	STRAFE_AXIS = 0  # Left stick X
	ROTATION_AXIS = 4  # Right stick X

	# Buttons
	SLOW_MODE_BUTTON = 1  # A button
	STOP_BUTTON = 2  # B button


# ==============================================================================
# DRIVER JOYSTICK CLASS
# ==============================================================================


class DriverJoystick:
	"""Wrapper for driver controller with deadband and easy axis access."""

	DEADBAND = 0.1

	def __init__(self, port: int = DriverJoystickConfig.PORT):
		"""
		Initialize driver joystick.

		Args:
			port: Joystick port number
		"""
		self.joystick = wpilib.Joystick(port)
		self.config = DriverJoystickConfig()

	def get_forward(self) -> float:
		"""Get forward/backward speed with deadband applied."""
		return self._apply_deadband(self.joystick.getRawAxis(self.config.FORWARD_AXIS))

	def get_strafe(self) -> float:
		"""Get left/right strafe speed with deadband applied."""
		return self._apply_deadband(self.joystick.getRawAxis(self.config.STRAFE_AXIS))

	def get_rotation(self) -> float:
		"""Get rotation speed with deadband applied."""
		return self._apply_deadband(self.joystick.getRawAxis(self.config.ROTATION_AXIS))

	def get_field_relative_button_pressed(self) -> bool:
		"""Check if field relative toggle button was pressed."""
		return self.joystick.getRawButtonPressed(self.config.FIELD_RELATIVE_BUTTON)

	def get_slow_mode_button(self) -> bool:
		"""Check if slow mode button is held."""
		return self.joystick.getRawButton(self.config.SLOW_MODE_BUTTON)

	def get_stop_button(self) -> bool:
		"""Check if stop button is held."""
		return self.joystick.getRawButton(self.config.STOP_BUTTON)

	@staticmethod
	def _apply_deadband(value: float, deadband: float = DEADBAND) -> float:
		"""
		Apply deadband to joystick input to avoid drift.

		Args:
			value: Raw joystick value (-1 to 1)
			deadband: Deadband threshold

		Returns:
			Adjusted value with deadband applied
		"""
		if abs(value) < deadband:
			return 0.0
		return value
