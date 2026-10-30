# Setup step 8: Tune the drive motors

## What it does

Tunes the drive motors. A wheel's speed depends on voltage in a simple, steady way, so instead of Ziegler-Nichols the robot measures that relationship (feedforward): kS, the volts it takes just to get moving, and kV, the volts for each meter per second. Then it adds a small P to fix what's left. The robot drives itself up to 2 m forward and back, and saves the result.

## On the robot

1. Do steps 1 to 7 first.
2. Copy this folder's robot.py over robot.py in the project folder and deploy (py -m robotpy deploy).
3. Wheels ON the floor, at least 2.5 m of clear floor IN FRONT of the robot, and someone on the Disable button (Enter is disable, space is emergency stop).
4. Driver Station in **Test** mode, then **Enable**. About 30 seconds. Disable stops it at once (nothing is saved then).
5. Read the console for the results; they're saved automatically.

## In the simulator

Copy this folder's robot.py over robot.py, run py -m robotpy sim, and click **Test**. The simulator's true motor numbers are SIM_DRIVE_KS and SIM_DRIVE_KV at the top of easyswerve/sim/physics.py; autotune should find them almost exactly.

## The code to look at

```python
self.tuneCommand = self.drivetrain.autotuner.driveCommand()
commands2.CommandScheduler.getInstance().schedule(self.tuneCommand)
```

- **testInit** runs when the robot is enabled in Test mode. Autotune only runs in Test mode, so it can't start by accident in a match.
- **autotuner.driveCommand()** is the same as the dashboard's "Autotune/2 Drive" button. It runs voltage ramps and steps, fits the motor model, and saves the gains.
- **The report** in the console shows kS, kV and kA for each wheel and how well the model fits (R2 = 1.0 is perfect).

## Key ideas

- **Feedforward does most of the work.** If you know it takes 2.5 V per m/s, you can just send 2.5 V for 1 m/s: no waiting for an error to build up. P only cleans up what the model misses.
- **Why not Ziegler-Nichols here?** It tunes the feedback part only. A speed loop with good feedforward needs very little feedback, and making a drive wheel oscillate on purpose means the robot lurching around the floor.
- **Four wheels, one model.** All four should give about the same kS and kV. One that's very different has friction the others don't.

## Try this

1. Change SIM_DRIVE_KV in easyswerve/sim/physics.py, run it again in the simulator, and check that autotune finds the new value.
2. Real robot: compare kS of the four wheels. Which has the most friction?
3. Look at Autotune/ on the dashboard: the steering autotune button there is another way to tune the steering (P only).

Next: [00_09_gyrocheck](../00_09_gyrocheck/README.md)

To go back to the starting robot, copy 01_basic/robot.py over robot.py.
