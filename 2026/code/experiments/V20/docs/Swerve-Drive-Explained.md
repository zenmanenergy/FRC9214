# Swerve Drive Explained

## What's the Difference?

Most robots (like shopping carts) can only move forward/backward and turn. They have to turn before they can move sideways.

**Swerve drive is different.** Each wheel can:
1. **Spin faster or slower** to control speed (acceleration)
2. **Point in different directions** to control which way to go

This means the robot can move in ANY direction while the body rotates independently.

## How Does It Work?

Imagine you're standing with wheels for feet (yeah, weird image). 

- Each foot can point wherever you want it to
- Each foot can push with different amounts of force
- By pointing all your feet in different directions and pushing with different amounts, you can move forward AND sideways AND rotate all at the same time

That's swerve drive!

### The Parts

Each wheel on our robot has:

1. **Drive Motor** - Spins the wheel to move the robot (forward/backward)
2. **Turn Motor** - Rotates the wheel to point in different directions (left/right)
3. **Encoder** - Measures how much the wheel has turned so we know which way it's pointing
4. **Gyro (IMU)** - Tells us which way the robot's body is facing

### Movement Examples

| Need to do | How swerve does it |
|---|---|
| Move forward | All wheels point forward, all drive motors spin |
| Move sideways | All wheels point sideways, all drive motors spin |
| Rotate in place | Wheels point at corners (like ⬜), drive motors spin → robot spins |
| Move forward AND rotate | Front wheels point forward-right, back wheels point forward-left, drive motors spin |
| Complex move | Each wheel does its own thing based on math to create the perfect motion |

## Why Is This Hard to Program?

Because the computer has to figure out:
- Which direction should each wheel point?
- How fast should each wheel spin?
- How do we want the robot to move vs. rotate?

All of this happens 50 times per second while the robot is moving.

That's why we have:
- **PID Controllers** - Constantly adjust motors to hit our targets
- **Odometry** - Track where the robot actually is
- **Calibration** - Make sure we know exactly which way each wheel is pointing

## The Math Behind It

Without getting too deep: We use something called **"inverse kinematics."** 

In plain English: "Given where I want to go and how fast I want to spin, what should each wheel do?"

The robot translates your joystick input into commands for four wheels, and it does this math 50 times per second.

---

**Next:** Learn how to [control the robot using the Dashboard](Dashboard-Guide.md)
