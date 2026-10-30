# Setup step 1: Robot check-up

## What it does

The first thing to run on a new robot, before anything moves. It checks that the code can talk to every part of the drivetrain (all 8 SPARK MAXes, all 4 steering encoders, the navX gyro, the battery) and catches typing mistakes in robot_settings.py, like the same CAN ID used twice. Every line says OK or PROBLEM.

## On the robot

1. Copy this folder's robot.py over robot.py in the project folder and deploy (py -m robotpy deploy).
2. Leave the robot **Disabled**. Nothing moves.
3. Read the console (Driver Station, or the terminal in VS Code). After 3 seconds it lists every part.
4. For each PROBLEM line, top first: fix the wiring or the number in robot_settings.py, deploy again, and check again.
5. When every line is OK (the steering zeros line can wait until step 4), go on.

## In the simulator

Copy this folder's robot.py over robot.py, run py -m robotpy sim, then read the terminal. (In the simulator every part answers, so you'll mostly see OK. The steering zeros line says PROBLEM until step 4.) Want something to find? The simulated robot is wired perfectly unless you set SIM_PRACTICE_MISTAKES = True near the top of easyswerve/sim/physics.py. Then it has a few wiring mistakes and its steering isn't zeroed, like a robot fresh off the build table. Work through the setup steps in order and fix robot_settings.py as they tell you.

## The code to look at

```python
results = self.drivetrain.checkup()  # a list of (ok, message)
problems = [text for ok, text in results if not ok]
```

```python
for ok, text in results:
    label = "OK      " if ok else "PROBLEM "
    print(f"  {label} {text}")
```

```python
if now - self.startTime < 3.0:
    return
if self.lastReport is not None and now - self.lastReport < 10.0:
    return
```

- **checkup()** asks every part a question and returns a list of (ok, message) pairs: True or False, plus a sentence.
- **A list comprehension** (problems = [text for ok, text in results if not ok]) keeps only the messages whose ok is False.
- **robotPeriodic** runs 50 times a second in every mode, even disabled. The two ifs make it wait 3 seconds after startup, then report every 10 seconds instead of 50 times a second.
- **Nothing moves**: there is no controller, no driveWithController and no auto in this example.

## Key ideas

- **Check before you move.** Most 'the robot is broken' problems are a wrong number in the settings or a loose wire. Finding them while disabled is safe and fast.
- **Fix from the top.** One mistake can cause several PROBLEM lines (a duplicated CAN ID can make two SPARKs look missing), so fix the first, deploy, and check again.
- **A SPARK 'answering'** means it sends data on the CAN bus with the ID you gave it. If it doesn't, either the ID in robot_settings.py is wrong or the SPARK's real ID is (set it in REV Hardware Client).

## Try this

1. In robot_settings.py, give two modules the same drive_can. What does the check-up say?
2. Real robot: unplug one steering encoder and watch its line change.
3. Real robot: what is the battery voltage now? Charge it before the tuning steps.

Next: [00_02_encodersbyhand](../00_02_encodersbyhand/README.md)

To go back to the starting robot, copy 01_basic/robot.py over robot.py.
