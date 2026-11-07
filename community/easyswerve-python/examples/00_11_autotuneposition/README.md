# Setup step 11: Tune driving to a spot (position and heading PID)

## What it does

Tunes the PID that drives the robot to a spot and turns it to face a direction. driveDistance, turnTo, heading hold, driveTo and the PathPlanner autos all use it: every loop they compare where the robot IS with where it SHOULD be and ask for the speed that closes the gap. Instead of guessing P, I and D, this step finds them two ways, tries both on real moves, and keeps the set that stops closest to the target.

## On the robot

1. Do steps 7 and 8 first: this step relies on the steering and the drive speed being tuned.
2. Copy this folder's robot.py over robot.py in the project folder and deploy (py -m robotpy deploy).
3. Wheels ON the floor. On the practice carpet (its size is in robot_settings.py): put the robot at one end, facing the long way, centered side to side, with the back bumper about 20 cm from the edge. The 20 cm lets it turn on the spot without the wheels going off the carpet. Someone on the Disable button (Enter is disable, space is emergency stop).
4. Driver Station in **Test** mode, then **Enable**. About a minute and a half: it turns on the spot, drives a little forward and back, rocks back and forth, then does the check moves. Disable stops it at once (nothing is saved then).
5. Read the console for the results; they're saved automatically. Then restart the robot code (or deploy again) so driveTo and the autos use them too.
6. Try it: enable in **Autonomous**. The robot drives 1 m forward, turns left and back, drives 1 m back, and prints how far off it stopped each time.

## The code to look at

```python
tuner = self.drivetrain.autotuner                                      # the autotuner (part of the drivetrain)
self.tuneCommand = tuner.positionCommand()                             # the position tuning command
```

```python
d.driveDistance(meters),                                           # 1. forward
self.check("forward", meters, 0.0),
d.turnTo(90),                                                      # 2. face left
```

- **autotuner.positionCommand()** is the same as the dashboard's "Autotune/3 Position" button. It tunes the heading first (turning on the spot), then the position (driving forward and back).
- **makeAuto** drives 1 m, turns 90 degrees and back, and drives back, so you can see how accurate the new gains are. check() prints how far from each target the robot stopped.
- **resetOdometry(Pose2d())** says "the robot is at x = 0, y = 0 now", and zeroHeading() says "the robot faces 0 degrees now", so the targets are easy numbers.

## Key ideas

- **Way 1, the model.** The robot is asked for a sudden speed (0.6 m/s, or a spin) and the code records how it follows: a short dead time before it starts to move, then a lag while it gets up to speed. The SIMC rules (by Sigurd Skogestad) turn those two times into P, I and D.
- **Way 2, Ziegler-Nichols.** The same method as setup step 7, on the whole robot: only P, small steps, P raised each time until the robot rocks back and forth by the same amount every swing. That gives Ku and Tu, and the "no overshoot" row gives P, I and D.
- **The real test decides.** Each set does real moves: 1 m out and back, and 90 degrees and back. The one that stops closest to the target is kept (if both stop about as close, the one that rolls past less). It must stop within 2 cm and 1.5 degrees to pass.
- **Why tune on the floor?** Position control is the whole robot moving: its weight, the carpet, the wheels' grip. On blocks none of that is there.
- **I only works close to the target.** I adds a push when the robot sits a little off for a while, but only within 3 cm (3 degrees). Farther away, P does the work, so I doesn't build up during a long drive and push the robot past the target.
- **driveTo and the autos need a restart.** PathPlanner reads the gains once, when the robot code starts. driveDistance, turnTo and heading hold use the new gains right away.

## Try this

1. Read the report: which way won for heading, and which for position? Were they close?
2. Run the Autonomous check, then measure with a tape where the robot really stopped. The code says it's within a centimeter: is it? If not, redo setup step 10 (wheel diameter).
3. Run it on a different floor (carpet vs. concrete). Do the gains change?

Next: [01_basic](../01_basic/README.md)

To go back to the starting robot, copy 01_basic/robot.py over robot.py.
