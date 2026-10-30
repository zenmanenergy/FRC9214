# 31: An intake mechanism

## What it does

A mechanism written properly, as a subsystem: an Intake class with commands it can do. Buttons use it in teleop, and the auto drives while the intake runs.

## On the robot

Copy this folder's robot.py over robot.py in the project folder and deploy (py -m robotpy deploy). Put the robot on the floor with about 3 m of clear space around it and someone ready to disable (Enter disables, space bar is the emergency stop), then enable in **Autonomous**. The console shows intake on, then intake off when the drive ends.

## The code to look at

```python
class Intake(commands2.Subsystem):
```

```python
def runIn(self) -> commands2.Command:
    """Pull in while running, stop when done."""
    return commands2.StartEndCommand(self.start, self.stop, self)
```

```python
return commands2.ParallelDeadlineGroup(
    self.drivetrain.driveFor(2.0, forward=0.2),
    self.intake.runIn(),
)
```

- **class Intake(commands2.Subsystem)** makes the intake a subsystem: one part of the robot, with its motor inside.
- **runIn and runOut** are methods that build commands. The rest of the code asks the intake for a command instead of touching its motor.
- **The self at the end of StartEndCommand** says this command requires the intake. If another intake command starts, this one is stopped first.
- **ParallelDeadlineGroup** runs commands at the same time. The first one is the deadline: when the drive finishes, the intake is stopped.

## Key ideas

- **Subsystems** keep each mechanism's code in one place, and requirements stop two commands from fighting over the same motor (the problem from 30).
- **Running things in parallel.** Real autos often drive and run a mechanism at the same time, for example picking up a game piece on the way.
- **This is how the drivetrain works too.** Drivetrain is a subsystem, and driveFor and turnTo are its commands.

## Try this

1. Let PathPlanner autos use the intake: create self.autos = Autos(self.drivetrain) and addCommand("intake", self.intake.runIn()) (see 20).
2. After the drive, spit out for 1 second: add self.intake.runOut().withTimeout(1.0) in a sequence after the deadline group.
3. Move the Intake class into its own file, intake.py, next to robot.py, and import it: from intake import Intake.

To go back to the starting robot, copy 01_basic/robot.py over robot.py.
