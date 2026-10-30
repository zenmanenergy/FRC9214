# Swerve Robot

Code for a swerve drive robot built from REV EasySwerve modules, written in Python with RobotPy.

## The files you work with

| File | What it's for |
|---|---|
| robot.py | what the controller buttons do, and which commands autos can use |
| robot_settings.py | your robot's wiring and size (CAN IDs, ports, measurements) |
| deploy/pathplanner/ | autos, drawn in the PathPlanner app |

Everything else is the library (the easyswerve folder) and support files. You can use it without opening it, the same way you use WPILib.

---

## Day 1: Run the robot in the simulator

No robot or controller needed.

1. Install Python 3.12 or 3.13 (64-bit) from python.org, ticking "Add python.exe to PATH". Skip this if Python is already installed.
2. Open this folder in VS Code, then open a terminal: Ctrl+` (the key above Tab).
3. Install RobotPy and this project's libraries (once):

```
py -m pip install robotpy
py -m robotpy sync
```

4. Start the simulator:

```
py -m robotpy sim
```

5. In the simulator window, under Robot State, click **Autonomous**. The robot drives an S-shaped path by itself. Watch it on the Field widget, and look for "Hello from auto!" in the terminal at the end.
6. Click **Disabled** to stop. Close the window to quit.

Have a controller? Plug it in, drag it from System Joysticks onto Joystick slot 0, and click **Teleoperated**.

## Day 2: Change what a button does

Open robot.py. Each button is one line, inside robotInit:

```python
self.controller.rightBumper().whileTrue(self.drivetrain.lockWheels())
```

- the button: rightBumper, leftBumper, a, b, x, y, start, back, leftTrigger, rightTrigger
- when: whileTrue (while held), onTrue (once when pressed), toggleOnTrue (press on, press off)
- what: a command, like lockWheels(), resetHeading(), stop()

Try it: copy that line, change rightBumper to a and lockWheels to stop, then run the simulator again.

More ideas are in examples/: each folder has a robot.py you can copy over this one (examples/README.md lists them, easiest first). examples/basic/robot.py puts the original back.

## Day 3: Draw an auto

1. Install the PathPlanner app (from the Microsoft Store, or PathPlanner's GitHub releases).
2. In PathPlanner, open this project folder.
3. Open the S-Curve path, drag the points, and save.
4. Make a new auto, add your path, and save it.
5. Run the simulator. Open the dashboard (Elastic, from the WPILib tools, connected to localhost), pick your auto in the **Auto** chooser, and click Autonomous.

Autos can run commands by name. robot.py has one called printHello:

```python
self.autos.addCommand("printHello", commands2.PrintCommand("Hello from auto!"))
```

In the PathPlanner app, add a Named Command to an auto and type printHello.

## Day 4: The real robot

1. Fill in robot_settings.py: CAN IDs, encoder DIO ports, motor mounting, wheel spacing.
2. Zero the steering encoders: robot disabled, hex keys in all four modules, then press **Calibrate/Zero steering encoders** on the dashboard (docs/DRIVETRAIN.md section 4.1).
3. Wheels off the ground, enable teleop, and check that pushing forward drives every wheel forward.
4. Autotune: clear 2.5 m in front of the robot, Driver Station in **Test** mode, enable, and press **Autotune/All** on the dashboard. The robot tunes itself in about a minute.
5. Deploy from the VS Code terminal:

```
py -m robotpy deploy
```

---

## More

| Where | What |
|---|---|
| examples/ | 31 small examples, one robot.py each: autos, buttons, dashboard, mechanisms (list in examples/README.md) |
| docs/DRIVETRAIN.md | full drivetrain guide: wiring, calibration, autotune, simulator (for coaches) |
| docs/PATHPLANNER.md | full PathPlanner guide |
| docs/VISION.md | setting up cameras (PhotonVision on the Orange Pi) |
| py -m robotpy test | runs the automated tests |
