# Setup step 2: Turn the wheels by hand

## What it does

Check the sensors before any motor runs. You turn a module's steering, or roll a wheel, by hand; the robot says which module it thinks moved, and which way. That proves each steering encoder is on the DIO port robot_settings.py says, each one counts the right way, and each drive SPARK MAX belongs to the module the settings say.

## On the robot

1. Copy this folder's robot.py over robot.py in the project folder and deploy (py -m robotpy deploy).
2. Leave the robot **Disabled** (the motors resist a little: that's brake mode; turn slowly).
3. Front left: turn its steering about a quarter turn counterclockwise, seen from above. The console should say "FL steering encoder ... counterclockwise".
4. Roll the front left wheel a little by hand. The console should say "FL wheel".
5. Repeat for front right (FR), rear left (RL) and rear right (RR).
6. Wrong name or wrong direction? Fix robot_settings.py (Key ideas below), deploy, and check that module again.

## In the simulator

Copy this folder's robot.py over robot.py, run py -m robotpy sim, then click **Autonomous**: the simulator turns each module's steering, then each wheel, one at a time (it has no hands). Want something to find? The simulated robot is wired perfectly unless you set SIM_PRACTICE_MISTAKES = True near the top of easyswerve/sim/physics.py. Then it has a few wiring mistakes and its steering isn't zeroed, like a robot fresh off the build table. Work through the setup steps in order and fix robot_settings.py as they tell you.

## The code to look at

```python
angle = module.absoluteDeg()  # the Through Bore Encoder, degrees
```

```python
moved = self.steer[name].update(change, now)
if moved is None or abs(moved) < STEER_DEG:
    return
way = "counterclockwise" if moved > 0 else "CLOCKWISE"
```

```python
if abs(change) > self.tiny:
    self.total += change
    self.stillSince = now
    return None
if self.total != 0.0 and now - self.stillSince >= STILL_S:
```

- **absoluteDeg()** is the Through Bore Encoder's angle. **wheelMeters()** is how far the drive motor's encoder says the wheel has rolled.
- **angleChange** handles the wrap-around: going from 359 to 1 degree is a change of +2, not -358.
- **MoveWatcher** adds up one movement and reports it once the sensor has been still for 0.3 s, so you get one line per turn of your hand instead of fifty.
- **+ means counterclockwise** seen from above. That's WPILib's rule for all angles.

## Key ideas

- **Name and direction.** If you turn the front left and the console says RR, the encoder plugs are swapped (fix encoder_dio). If it says CLOCKWISE while you turned counterclockwise, that encoder reads backwards (set encoder_reversed=True for that module).
- **Wheel names prove CAN IDs.** Rolling the front left wheel should print FL. If it prints another name, the drive_can numbers are mixed up.
- **Test with the motors off.** Turning things by hand can't break anything, so it's the safest way to check the sensors.

## Try this

1. Turn one module a full turn. Does the total come out near 360?
2. Turn it quickly back and forth. Why does it print the total only at the end?
3. Change STEER_DEG to 1.0. What gets printed now when you bump the robot?
4. Simulator with SIM_PRACTICE_MISTAKES = True: which modules are on the wrong DIO port? Which encoder reads backwards?

Next: [00_03_steeronebyone](../00_03_steeronebyone/README.md)

To go back to the starting robot, copy 01_basic/robot.py over robot.py.
