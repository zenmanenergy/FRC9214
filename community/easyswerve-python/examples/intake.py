"""
EXAMPLE MECHANISM: a one-motor intake.

A pattern for adding your own mechanisms. To use it:
  1. Copy this file into the project folder (next to robot.py).
  2. Change INTAKE_CAN_ID to the SPARK MAX on your intake.
  3. Add it in robot.py (see examples/robot_more.py).

Only add it once the motor is on the robot: a SPARK MAX the code can't find
on the CAN bus reports errors every loop.
"""

import commands2
from rev import PersistMode, ResetMode, SparkBaseConfig, SparkMax, SparkMaxConfig

INTAKE_CAN_ID = 20  # change to your intake's SPARK MAX
INTAKE_SPEED = 0.6  # 0 to 1 (fraction of full power)


class Intake(commands2.Subsystem):
    def __init__(self) -> None:
        super().__init__()
        self.motor = SparkMax(INTAKE_CAN_ID, SparkMax.MotorType.kBrushless)

        config = SparkMaxConfig()
        config.setIdleMode(SparkBaseConfig.IdleMode.kBrake)
        config.smartCurrentLimit(30)
        self.motor.configure(
            config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters
        )

    # ---- commands -----------------------------------------------------------------

    def runIn(self) -> commands2.Command:
        """Spin inward while the command runs, stop when it ends."""
        return commands2.StartEndCommand(
            lambda: self.motor.set(INTAKE_SPEED), lambda: self.motor.set(0), self
        )

    def runOut(self) -> commands2.Command:
        """Spin outward (spit out) while the command runs."""
        return commands2.StartEndCommand(
            lambda: self.motor.set(-INTAKE_SPEED), lambda: self.motor.set(0), self
        )
