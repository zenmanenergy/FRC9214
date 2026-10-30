# 20: Named commands in PathPlanner autos

## What it does

Let autos drawn in the PathPlanner app run your own commands, by name.

## Run it

Copy this folder's robot.py over robot.py in the project folder, run py -m robotpy sim, and click **Autonomous**. Look for "Hello from auto!" in the terminal at the end.

## The code to look at

```python
# name in the app  -> what it does
self.autos.addCommand("printHello", commands2.PrintCommand("Hello from auto!"))
self.autos.addCommand("lockWheels", self.drivetrain.lockWheels().withTimeout(1.0))
```

- **addCommand(name, command)** connects a name typed in the PathPlanner app to a real command in code.
- The Test S-Curve auto has a Named Command block called printHello at the end.

## Key ideas

- **The app doesn't know your robot's code.** It only stores names. This is how a drawn auto runs an intake, a shooter, anything.
- **Names must match exactly**, including capital letters.
- **Register in robotInit**, before autonomous starts.

## Try this

1. In the PathPlanner app, add a Named Command block called lockWheels to the end of the Test S-Curve auto. Save and run.
2. Misspell the name in the app (lockwheels). What happens? Fix it.
3. Add your own named command that prints something else.

To go back to the starting robot, copy 01_basic/robot.py over robot.py.
