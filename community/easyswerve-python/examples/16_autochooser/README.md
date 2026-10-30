# 16: Pick an auto on the dashboard

## What it does

Show a drop-down list of autos on the dashboard, so the drive team picks one before each match without changing code.

## Run it

Copy this folder's robot.py over robot.py, run py -m robotpy sim, open Elastic (WPILib tools, connected to localhost), add the Auto chooser to a tab, pick an auto, then click **Autonomous**.

## The code to look at

```python
self.chooser = wpilib.SendableChooser()
self.chooser.setDefaultOption("Drive forward", self.driveForward)
self.chooser.addOption("Spin", self.spin)
self.chooser.addOption("Do nothing", self.doNothing)
wpilib.SmartDashboard.putData("Auto", self.chooser)
```

```python
buildAuto = self.chooser.getSelected()
return buildAuto()
```

```python
def driveForward(self):
    return self.drivetrain.driveFor(2.0, forward=0.25)
```

- **SendableChooser** is the drop-down list. setDefaultOption is the choice if nobody picks; addOption adds more.
- **Each option is a function** (self.driveForward, without parentheses), not a command. makeAuto calls the chosen function to build a fresh command.
- **putData("Auto", ...)** puts the list on the dashboard under the name Auto.

## Key ideas

- **Functions as values.** self.driveForward (no parentheses) is the function itself; self.driveForward() calls it. The chooser stores functions and calls them later.
- **Why build fresh?** A command can only run once at a time, and building it when autonomous starts means it always starts clean.
- **Do nothing is a real option.** Returning None means no auto, and makeAuto handles that.

## Try this

1. Add your square from 08 as an option: write a square method that returns the SequentialCommandGroup, then addOption("Square", self.square).
2. Change which option is the default.

To go back to the starting robot, copy 01_basic/robot.py over robot.py.
