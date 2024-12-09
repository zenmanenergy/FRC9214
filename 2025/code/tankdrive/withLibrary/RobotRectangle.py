import wpilib
import ctre
import rev
from movement import Movement  # Import the Movement class

class MyRobot(wpilib.TimedRobot):
	def robotInit(self):
		# Motor and encoder setup
		self.LeftFrontMotor = ctre.WPI_TalonSRX(1)
		self.LeftRearMotor = ctre.WPI_TalonSRX(2)
		self.RightFrontMotor = ctre.WPI_TalonSRX(3)
		self.RightRearMotor = ctre.WPI_TalonSRX(4)

		# Wheel and encoder setup
		self.WHEEL_DIAMETER_MM = 152.4
		self.WHEEL_CIRCUMFERENCE_MM = self.WHEEL_DIAMETER_MM * 3.141592653589793
		self.ENCODER_CPR = 2048
		self.ROBOT_WIDTH_MM = 508
		self.ROBOT_CIRCUMFERENCE_MM = self.ROBOT_WIDTH_MM * 3.141592653589793

		self.left_encoder = wpilib.Encoder(0, 1)
		self.right_encoder = wpilib.Encoder(2, 3)
		self.left_encoder.setDistancePerPulse(self.WHEEL_CIRCUMFERENCE_MM / self.ENCODER_CPR)
		self.right_encoder.setDistancePerPulse(self.WHEEL_CIRCUMFERENCE_MM / self.ENCODER_CPR)

		# Initialize Movement with tuning parameters
		self.movement = Movement(self, max_speed=0.6, min_speed=0.3, accel_distance=150)

		# Set the initial coordinate to (0, 0)
		self.movement.setCoordinate(0, 0)

	def autonomousInit(self):
		# Start the rectangle sequence
		self.rectangle_steps = [
			("travel", 3000),  # Move forward 3000 mm
			("rotate", 90),    # Turn left
			("travel", 2000),  # Move forward 2000 mm
			("rotate", 90),    # Turn left
			("travel", 4000),  # Move forward 4000 mm
			("rotate", 90),    # Turn left
			("travel", 2000),  # Move forward 2000 mm
			("rotate", 90),    # Turn left
			("travel", 1000),  # Move forward 1000 mm
		]
		self.current_step = 0
		self.step_in_progress = False

	def autonomousPeriodic(self):
		# Continuously update the robot's position
		self.movement.updatePosition()

		# Execute the rectangle sequence
		if self.current_step < len(self.rectangle_steps):
			step_type, value = self.rectangle_steps[self.current_step]

			if not self.step_in_progress:
				self.step_in_progress = True  # Mark step as in progress

			# Handle travel steps
			if step_type == "travel":
				if not self.movement.travelDistance(value):
					self.step_in_progress = False  # Step complete
					self.current_step += 1

			# Handle rotate steps
			elif step_type == "rotate":
				if not self.movement.rotate(value):
					self.step_in_progress = False  # Step complete
					self.current_step += 1
		else:
			print("Rectangle path complete!")

	def teleopPeriodic(self):
		# Continuously update the robot's position during teleop
		self.movement.updatePosition()

		# Example manual control or testing
		pass


if __name__ == "__main__":
	wpilib.run(MyRobot)
