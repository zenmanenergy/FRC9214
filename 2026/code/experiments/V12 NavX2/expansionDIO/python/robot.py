"""
NavX MXP Expansion I/O Example

This program demonstrates the use of the MXP Expansion IO capabilities 
of the navX MXP, including:

DIGITAL I/O:
- Pulse-Width Modulation [PWM] (e.g., Motor Control)
- Digital Inputs (e.g., Contact Switch closure)
- Digital Outputs (e.g., Relay control)
- Quadrature Encoder (e.g., Wheel Encoder)

ANALOG I/O:
- Analog Inputs (e.g., Ultrasonic Sensor)
- Analog Input Trigger (e.g., Proximity Sensor trigger)
- Analog Trigger Counter
- Analog Output (e.g., Constant-current LED, Sound)

For more details on navX MXP Expansion I/O:
http://navx-mxp.kauailabs.com/installation/io-expansion/
"""

import wpilib
from enum import Enum


class PinType(Enum):
	DIGITAL_IO = 1
	PWM = 2
	ANALOG_IN = 3
	ANALOG_OUT = 4


class Robot(wpilib.TimedRobot):
	MXP_IO_VOLTAGE = 3.3
	MIN_AN_TRIGGER_VOLTAGE = 0.76
	MAX_AN_TRIGGER_VOLTAGE = MXP_IO_VOLTAGE - 2.0

	# navX MXP constants
	MAX_NAVX_MXP_DIGIO_PIN = 9
	MAX_NAVX_MXP_ANALOGIN_PIN = 3
	MAX_NAVX_MXP_ANALOGOUT_PIN = 1
	NUM_ROBORIO_ONBOARD_DIGIO_PINS = 10
	NUM_ROBORIO_ONBOARD_PWM_PINS = 10
	NUM_ROBORIO_ONBOARD_ANALOGIN_PINS = 4

	def robotInit(self):
		"""Initialize robot components."""
		self.stick = wpilib.Joystick(0)

		try:
			# Digital I/O
			self.pwm_out_9 = wpilib.Victor(self.get_channel_from_pin(PinType.PWM, 9))
			self.pwm_out_8 = wpilib.Jaguar(self.get_channel_from_pin(PinType.PWM, 8))
			self.dig_in_7 = wpilib.DigitalInput(self.get_channel_from_pin(PinType.DIGITAL_IO, 7))
			self.dig_in_6 = wpilib.DigitalInput(self.get_channel_from_pin(PinType.DIGITAL_IO, 6))
			self.dig_out_5 = wpilib.DigitalOutput(self.get_channel_from_pin(PinType.DIGITAL_IO, 5))
			self.dig_out_4 = wpilib.DigitalOutput(self.get_channel_from_pin(PinType.DIGITAL_IO, 4))
			self.enc_3and2 = wpilib.Encoder(
				self.get_channel_from_pin(PinType.DIGITAL_IO, 3),
				self.get_channel_from_pin(PinType.DIGITAL_IO, 2)
			)
			self.enc_1and0 = wpilib.Encoder(
				self.get_channel_from_pin(PinType.DIGITAL_IO, 1),
				self.get_channel_from_pin(PinType.DIGITAL_IO, 0)
			)

			# Analog I/O
			self.an_in_1 = wpilib.AnalogInput(self.get_channel_from_pin(PinType.ANALOG_IN, 1))
			self.an_trig_0 = wpilib.AnalogTrigger(self.get_channel_from_pin(PinType.ANALOG_IN, 0))
			self.an_trig_0_counter = wpilib.Counter(self.an_trig_0)

			self.an_out_1 = wpilib.AnalogOutput(self.get_channel_from_pin(PinType.ANALOG_OUT, 1))
			self.an_out_0 = wpilib.AnalogOutput(self.get_channel_from_pin(PinType.ANALOG_OUT, 0))

			# Configure components
			self.pwm_out_9.setSafetyEnabled(False)
			self.pwm_out_8.setSafetyEnabled(False)

			# Configure Analog Trigger
			self.an_trig_0.setAveraged(True)
			self.an_trig_0.setLimitsVoltage(self.MIN_AN_TRIGGER_VOLTAGE, self.MAX_AN_TRIGGER_VOLTAGE)

		except RuntimeError as ex:
			wpilib.reportError(f"Error instantiating MXP pin on navX MXP: {ex}", True)

	def autonomousInit(self):
		"""Initialize autonomous mode."""
		pass

	def autonomousPeriodic(self):
		"""Run autonomous for 2 seconds then stop."""
		pass

	def teleopPeriodic(self):
		"""Process joystick input and update I/O."""
		# Digital I/O
		self.pwm_out_9.set(self.stick.getX())
		self.pwm_out_8.set(self.stick.getY())
		wpilib.SmartDashboard.putBoolean("DigIn7", self.dig_in_7.get())
		wpilib.SmartDashboard.putBoolean("DigIn6", self.dig_in_6.get())
		self.dig_out_5.set(self.stick.getRawButton(1))
		self.dig_out_4.set(self.stick.getRawButton(2))
		wpilib.SmartDashboard.putNumber("Enc3and2", self.enc_3and2.get())
		wpilib.SmartDashboard.putNumber("Enc1and0", self.enc_1and0.get())

		# Analog Inputs
		wpilib.SmartDashboard.putNumber("AnalogIn1", self.an_in_1.getAverageVoltage())
		wpilib.SmartDashboard.putBoolean("AnalogTrigger0", self.an_trig_0.getTriggerState())
		wpilib.SmartDashboard.putNumber("AnalogTriggerCounter0", self.an_trig_0_counter.get())

		# Analog Outputs
		self.an_out_1.setVoltage(abs(self.stick.getX()) * self.MXP_IO_VOLTAGE)
		self.an_out_0.setVoltage(abs(self.stick.getY()) * self.MXP_IO_VOLTAGE)

	def get_channel_from_pin(self, pin_type: PinType, pin_number: int) -> int:
		"""
		Convert from a navX MXP Pin type and number to the corresponding 
		RoboRIO Channel Number.

		Args:
			pin_type: Type of pin (DIGITAL_IO, PWM, ANALOG_IN, ANALOG_OUT)
			pin_number: Pin number on the navX MXP

		Returns:
			RoboRIO channel number

		Raises:
			ValueError: If pin number is invalid
		"""
		if pin_number < 0:
			raise ValueError("Error: navX MXP I/O Pin number is negative")

		if pin_type == PinType.DIGITAL_IO:
			if pin_number > self.MAX_NAVX_MXP_DIGIO_PIN:
				raise ValueError("Error: Invalid navX MXP Digital I/O Pin #")
			return (pin_number + self.NUM_ROBORIO_ONBOARD_DIGIO_PINS +
					(4 if pin_number > 3 else 0))

		elif pin_type == PinType.PWM:
			if pin_number > self.MAX_NAVX_MXP_DIGIO_PIN:
				raise ValueError("Error: Invalid navX MXP PWM Pin #")
			return pin_number + self.NUM_ROBORIO_ONBOARD_PWM_PINS

		elif pin_type == PinType.ANALOG_IN:
			if pin_number > self.MAX_NAVX_MXP_ANALOGIN_PIN:
				raise ValueError("Error: Invalid navX MXP Analog Input Pin #")
			return pin_number + self.NUM_ROBORIO_ONBOARD_ANALOGIN_PINS

		elif pin_type == PinType.ANALOG_OUT:
			if pin_number > self.MAX_NAVX_MXP_ANALOGOUT_PIN:
				raise ValueError("Error: Invalid navX MXP Analog Output Pin #")
			return pin_number

		raise ValueError(f"Unknown pin type: {pin_type}")


if __name__ == "__main__":
	wpilib.run(Robot)
