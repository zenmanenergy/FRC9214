# 32: Zero the steering encoders

## What it does

Teach the robot which way each wheel points. The steering encoders measure angle, but they don't know where "straight" is until you show them. Do this on a new robot, and again whenever a module or encoder is taken apart.

## Run it

In the simulator: copy this folder's robot.py over robot.py, run py -m robotpy sim, leave it **Disabled**, and press Back on the controller (or the Calibrate button on the dashboard). On the real robot, follow the steps below.

## The code to look at

```python
self.controller.back().onTrue(self.drivetrain.zeroSteeringCommand())
```

```python
degrees = math.degrees(module.getState().angle.radians())
```

```python
return self.drivetrain.driveFor(1.5, forward=0.15)
```

- **zeroSteeringCommand** reads all four steering encoders and saves those readings as each wheel's zero. It's the same as the dashboard's Calibrate button.
- **It works while disabled.** Most commands don't run while disabled; this one is made to, because the wheels must not move while you zero them.
- **disabledPeriodic** runs 50 times a second while disabled. Here it prints each wheel's angle every 2 seconds, so you can watch the zeroing work.
- **The auto is the check:** driving slowly forward, every wheel should point straight ahead.

## Key ideas

- **Absolute encoders** know the wheel's angle the moment the robot turns on, but each encoder is mounted at some random rotation. The zero is the reading when the wheel is in a known position: that's what the hex key sets.
- **Saved, not typed.** The zeros are saved in a file on the roboRIO and loaded every time the robot starts. Redeploying code doesn't erase them.
- **The hex-key position is an X.** After zeroing, the printout shows FL 45, FR -45, RL 135, RR -135, not 0. That's expected: REV's code knows the hex-key pocket sits 45 degrees off straight, and allows for it.
- **On the real robot:** robot disabled, push a 3 mm hex key into the hole on top of each module, turn the wheel by hand until the key drops about 6 mm deeper, leave all four keys in, then press Back. Remove the keys before enabling.

## Try this

1. Run the check auto. Do all four wheels point straight ahead in the status line (FL, FR, RL, RR near 0, or 180: the same line, rolling the other way)?
2. Turn one wheel by hand while disabled (on the real robot) and watch its number change in the printout.
3. Why must the robot be disabled while zeroing? What would happen if a wheel moved during it?

To go back to the starting robot, copy 01_basic/robot.py over robot.py.
