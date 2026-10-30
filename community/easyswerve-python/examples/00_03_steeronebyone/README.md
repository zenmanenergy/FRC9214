# Setup step 3: Run each steering motor by itself

## What it does

The step that fixes wheels that fight each other or point the wrong way. It turns ONE module's steering motor at a low fixed voltage (no PID) and compares what two sensors saw: the motor's own sensor inside the NEO, and the Through Bore Encoder. Positive voltage should turn the wheel counterclockwise and both should count up. If they disagree, the wheel fights itself in normal driving; the console tells you which setting to change.

## On the robot

1. Copy this folder's robot.py over robot.py in the project folder and deploy (py -m robotpy deploy).
2. Put the robot on blocks so the wheels are off the floor.
3. Enable in **Teleoperated**. Hold **A** (front left), **B** (front right), **X** (rear left) or **Y** (rear right). Or enable in **Autonomous** to test all four in turn.
4. WATCH the wheel: which one turned, and which way (seen from above)?
5. Read the console and change robot_settings.py as it says. Deploy and test that module again, until all four agree and turn counterclockwise.
6. Changed encoder_reversed or encoder_dio? Then zeroing (step 4) must be done after this.

## In the simulator

Copy this folder's robot.py over robot.py, run py -m robotpy sim, then click **Autonomous** to test all four in turn. On the real robot you look at the wheel; in the simulator the console tells you which way it really turned. Want something to find? The simulated robot is wired perfectly unless you set SIM_PRACTICE_MISTAKES = True near the top of easyswerve/sim/physics.py. Then it has a few wiring mistakes and its steering isn't zeroed, like a robot fresh off the build table. Work through the setup steps in order and fix robot_settings.py as they tell you.

## The code to look at

```python
return commands2.FunctionalCommand(
    lambda: self.startTest(name),
    lambda: module.spinSteering(TEST_VOLTS),  # every loop
    lambda interrupted: self.endTest(name),
    lambda: False,
    self.drivetrain,
).withTimeout(TEST_SECONDS)
```

```python
if (motor > 0) == (mine > 0):
    print("  Motor and encoder agree. LOOK at the wheel:")
```

```python
print("  PROBLEM: motor and encoder DISAGREE (this makes the wheel fight itself). LOOK:")
print(f"    turned counterclockwise -> the encoder is backwards: flip encoder_reversed for {setting}")
print(f"    turned clockwise        -> the motor is backwards: flip turn_motor_bottom for {setting}")
```

- **spinSteering(volts)** runs the steering motor at a fixed voltage with no PID. It's the most direct test: volts in, movement out.
- **FunctionalCommand** takes four functions: start, every loop, end, and "am I finished?". withTimeout stops it after TEST_SECONDS.
- **AngleTracker** adds up the total turn of a reading that wraps around, so a turn past 360 degrees still counts right.
- **(motor > 0) == (mine > 0)** is True when both turned the same way (both positive or both negative).

## Key ideas

- **Two sensors must agree.** With the encoders on the roboRIO, the SPARK steers by the NEO's sensor and the code keeps resetting that sensor to the Through Bore Encoder's angle. If the two count in opposite directions, every reset pushes the wheel the wrong way: wheels that jitter, drift, or point wrong.
- **Your eyes are the third sensor.** The code can tell that two sensors disagree, but not which one is wrong. Only you can see which way the wheel really turned.
- **One change at a time.** Fix one setting, deploy, test that module again.

## Try this

1. Change TEST_VOLTS to 0.5. Does the wheel still turn? The voltage where it just starts to move is the steering friction (kS).
2. Simulator with SIM_PRACTICE_MISTAKES = True: find and fix every module, until all four say "agree" and turn counterclockwise.
3. Why does a backwards motor AND a backwards encoder look fine to the code? (Hint: what do both sensors see?)

Next: [00_04_zerosteering](../00_04_zerosteering/README.md)

To go back to the starting robot, copy 01_basic/robot.py over robot.py.
