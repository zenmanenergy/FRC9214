# Setup step 6: Run each drive motor by itself

## What it does

Checks every drive motor's direction and that all four run in sync. It points every wheel straight ahead, spins ONE wheel at a low fixed voltage (no PID), and reports how far and how fast its encoder says it went. Positive voltage should roll the wheel forward. Then all four spin together: at the same voltage they should all reach about the same speed.

## On the robot

1. Copy this folder's robot.py over robot.py in the project folder and deploy (py -m robotpy deploy).
2. Robot on blocks, wheels off the floor.
3. Enable in **Teleoperated** and hold **A** / **B** / **X** / **Y** for one wheel, or the **right bumper** for all four. Or enable in **Autonomous** to test them all in turn.
4. LOOK: does the wheel roll forward (the top of the wheel moving toward the front of the robot)?
5. Rolled backwards: flip drive_motor_bottom for that module in robot_settings.py. Deploy and test again.

## In the simulator

Copy this folder's robot.py over robot.py, run py -m robotpy sim, then click **Autonomous**. The simulated robot isn't on blocks, so it drives around a little. The console says which way each wheel really rolled. Want something to find? The simulated robot is wired perfectly unless you set SIM_PRACTICE_MISTAKES = True near the top of easyswerve/sim/physics.py. Then it has a few wiring mistakes and its steering isn't zeroed, like a robot fresh off the build table. Work through the setup steps in order and fix robot_settings.py as they tell you.

## The code to look at

```python
commands2.RunCommand(self.straighten, self.drivetrain).withTimeout(0.5),
```

```python
module.spinWheel(TEST_VOLTS)  # steering stays where straighten() put it
self.topSpeed[name] = max(self.topSpeed[name], module.wheelSpeed())
```

```python
if fastest > 0 and (fastest - slowest) / fastest > 0.15:
```

- **A SequentialCommandGroup** runs commands one after another: first straighten the wheels for 0.5 s, then spin.
- **spinWheel(volts)** runs the drive motor at a fixed voltage with no PID. **wheelSpeed()** is the encoder's speed in m/s.
- **topSpeed** keeps the fastest speed seen, using max() every loop.
- **15 %**: if the slowest wheel is more than 15 % slower than the fastest at the same voltage, something is dragging.

## Key ideas

- **The encoder can't tell you the direction is wrong.** Flipping drive_motor_bottom flips the motor and its encoder together, so the encoder always counts up for positive volts. Only your eyes can see that the wheel rolled backwards.
- **Same voltage, same speed.** Four identical motors at the same voltage should spin at the same speed. If one doesn't, the PID will push it harder all match long, and the robot pulls to one side.
- **Open loop first.** Test motors without PID before tuning PID: PID can hide a mechanical problem by working harder.

## Try this

1. At 1.5 V, about how fast should a wheel go? (Hint: the drive kV is about 2.5 V per m/s, and some voltage goes to friction.) Compare with the console.
2. Change TEST_VOLTS to 3.0. Is the speed about twice as fast?
3. Simulator with SIM_PRACTICE_MISTAKES = True: which wheel rolls backwards? Fix it.

Next: [00_07_zieglernichols](../00_07_zieglernichols/README.md)

To go back to the starting robot, copy 01_basic/robot.py over robot.py.
