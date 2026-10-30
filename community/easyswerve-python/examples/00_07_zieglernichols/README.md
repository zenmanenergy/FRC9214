# Setup step 7: Tune the steering PID with Ziegler-Nichols

## What it does

Tunes the steering P, I and D by the Ziegler-Nichols "ultimate gain" method, automatically on all four modules. With only P, it makes small steering steps and raises P each time, until the wheel stops settling and keeps swinging by the same amount. That P is the ultimate gain Ku, and one swing back and forth takes Tu. Ziegler and Nichols' table turns Ku and Tu into P, I and D. It measures the steering before and after, and saves the new gains only if they're good.

## On the robot

1. Do steps 1 to 6 first. A backwards motor or encoder makes the wheel run away (the test notices and stops).
2. Copy this folder's robot.py over robot.py in the project folder and deploy (py -m robotpy deploy).
3. Wheels ON the floor (the load they steer in a match), on a clear patch of floor. Charge the battery.
4. Enable in **Autonomous**. The wheels wiggle and buzz for about 20 seconds; the robot doesn't drive. Disable at any time to stop.
5. Read the console: Ku and Tu for each module, the gains, and before vs. after. If the after numbers are good, the gains are saved.

## The code to look at

```python
RULES = {
    #                  kP     ti     td
    "P only":         (0.50,  None,  None),
    "PI":             (0.45,  0.83,  None),
    "PID (classic)":  (0.60,  0.50,  0.125),
    "some overshoot": (0.33,  0.50,  0.33),
    "no overshoot":   (0.20,  0.50,  0.33),
}
RULE = "no overshoot"  # which row to use; try the others!
```

```python
limit = min(found, key=lambda r: r.ku)
ku, tu = limit.ku, limit.tu
```

```python
kP, ti, td = RULES[RULE]
p = kP * ku
i = p / (ti * tu) if ti else 0.0  # I = P / Ti
d = p * (td * tu) if td else 0.0  # D = P x Td
```

```python
self.addPeriodic(self.sampleSteering, 0.005)
```

- **ZieglerNicholsTest** (in easyswerve/tuning/zieglernichols.py) does steps 1 and 2: it raises P by 25 % per try and calls useResults with each module's Ku and Tu when it's done.
- **The table** is the whole Ziegler-Nichols method: P is a fraction of Ku; the I and D times (Ti and Td) are fractions of Tu. I = P / Ti and D = P x Td.
- **min(found, key=...)** picks the module that starts swinging at the lowest P. The four modules share one set of gains, so the touchiest one decides.
- **addPeriodic(..., 0.005)** runs a function every 5 ms, besides the normal 20 ms loop. The swings take only about 40 ms, too fast to see at 20 ms.
- **SteeringStepCheck** steers every wheel 45 degrees back and forth and measures overshoot (how far past the target, in % of the step) and settling time.

## Key ideas

- **Why the ultimate gain?** At Ku the loop is right on the edge between settling and swinging. Ziegler and Nichols found that backing off from that edge by fixed fractions gives good gains for many machines, without knowing anything else about them.
- **The rows trade speed for calm.** The classic PID row is fast but overshoots; "no overshoot" backs off much further. For steering, overshoot means wheels that wiggle every time they turn.
- **It runs on the SPARK's own loop.** The SPARK runs its steering PID every 1 ms, much faster than robot.py's 20 ms loop, so the test sets P on the SPARK and only watches from the roboRIO.
- **Saved for every startup.** The new gains go into a file on the roboRIO, like autotune's, and are loaded each time the robot turns on.

## Try this

1. Write down Ku and Tu. Work out the "PID (classic)" gains by hand, then set RULE = "PID (classic)" and run it again to check. Which rule gives the lowest overshoot?
2. Try RULE = "PI". Why might it never settle? (Hint: I keeps pushing until the error is zero, and D is what calms it down.)
3. Change GOOD_OVERSHOOT_PCT to 5. Do the gains still get saved?
4. Real robot: run it with the wheels on the floor, then on blocks. Is Ku different? Why?

Next: [00_08_autotunedrive](../00_08_autotunedrive/README.md)

To go back to the starting robot, copy 01_basic/robot.py over robot.py.
