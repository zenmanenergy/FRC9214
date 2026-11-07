# 31: An intake mechanism

## What it does

A mechanism written properly, as a subsystem: an Intake class with commands it can do. Buttons use it in teleop, and the auto drives while the intake runs.

## On the robot

Copy this folder's robot.py over robot.py in the project folder and deploy (py -m robotpy deploy). Put the robot in the middle of the practice carpet, facing the long way, have someone ready to disable (Enter disables, space bar is the emergency stop), then enable in **Autonomous**. The moves are sized to fit the carpet (CARPET_LENGTH_M and CARPET_WIDTH_M in robot_settings.py). The console shows intake on, then intake off when the drive ends.

## The code to look at

```python
class Intake(commands2.Subsystem):                                         # a one-motor intake: a subsystem of its own
```

```python
def runIn(self) -> commands2.Command:                                  # a command: pull in while it runs
    return commands2.StartEndCommand(self.start, self.stop, self)      # start() at the start, stop() at the end
```

```python
seconds = carpet.fitSeconds(2.0, forward=0.2)                      # 2 s, or less to stay on the carpet
return commands2.ParallelDeadlineGroup(                            # run these at the same time, until the FIRST ends:
    self.drivetrain.driveFor(seconds, forward=0.2),                # the deadline: drive forward 2 s
    self.intake.runIn(),                                           # runs alongside, stops when the drive ends
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
