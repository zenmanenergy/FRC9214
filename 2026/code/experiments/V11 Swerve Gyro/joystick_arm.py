import wpilib


# ==============================================================================
# ARM/OPERATOR JOYSTICK CONFIGURATION
# ==============================================================================


class ArmJoystickConfig:
	"""Arm/Operator joystick button and axis mappings."""

	# Port number
	PORT = 1

	# Axes (adjust based on your operator controller)
	ARM_VERTICAL_AXIS = 1  # Left stick Y
	ARM_HORIZONTAL_AXIS = 0  # Left stick X
	INTAKE_AXIS = 5  # Right trigger

	# Buttons (adjust based on your operator controller)
	INTAKE_BUTTON = 1  # A button
	OUTTAKE_BUTTON = 2  # B button
	CLIMB_UP_BUTTON = 3  # Y button
	CLIMB_DOWN_BUTTON = 4  # X button


# ==============================================================================
# ARM/OPERATOR JOYSTICK CLASS
# ==============================================================================


class ArmJoystick:
	"""Wrapper for operator/arm controller."""

	DEADBAND = 0.1

	def __init__(self, port: int = ArmJoystickConfig.PORT):
		"""
		Initialize arm/operator joystick.

		Args:
			port: Joystick port number
		"""
		self.joystick = wpilib.Joystick(port)
		self.config = ArmJoystickConfig()

	def get_arm_vertical(self) -> float:
		"""Get arm vertical movement with deadband applied."""
		return self._apply_deadband(self.joystick.getRawAxis(self.config.ARM_VERTICAL_AXIS))

	def get_arm_horizontal(self) -> float:
		"""Get arm horizontal movement with deadband applied."""
		return self._apply_deadband(self.joystick.getRawAxis(self.config.ARM_HORIZONTAL_AXIS))

	def get_intake_speed(self) -> float:
		"""Get intake trigger input with deadband applied."""
		return self._apply_deadband(self.joystick.getRawAxis(self.config.INTAKE_AXIS))

	def get_intake_button(self) -> bool:
		"""Check if intake button is held."""
		return self.joystick.getRawButton(self.config.INTAKE_BUTTON)

	def get_outtake_button(self) -> bool:
		"""Check if outtake button is held."""
		return self.joystick.getRawButton(self.config.OUTTAKE_BUTTON)

	def get_climb_up_button_pressed(self) -> bool:
		"""Check if climb up button was pressed."""
		return self.joystick.getRawButtonPressed(self.config.CLIMB_UP_BUTTON)

	def get_climb_down_button_pressed(self) -> bool:
		"""Check if climb down button was pressed."""
		return self.joystick.getRawButtonPressed(self.config.CLIMB_DOWN_BUTTON)

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
