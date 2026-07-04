# Autonomous Paths

During the autonomous phase of FRC matches, the robot drives itself. No joystick input. It needs to know exactly where to go and how to get there.

## What Is an Autonomous Path?

A path is basically a set of instructions: "Go to this spot, then this spot, then rotate to face this direction."

The robot uses math to figure out:
- Where it is right now
- Where it needs to go
- How to drive there smoothly
- When it's arrived

## How Does the Robot Know Where It Is?

The robot tracks its position using **odometry** (fancy word for "using wheels and gyro to know where you are").

**It measures:**
- How much each wheel has turned (wheel encoders)
- Which way the robot is facing (gyro/IMU)
- Speed of each wheel

**It calculates:**
- "If my wheels turned this much, I must have moved this far"
- "My gyro says I'm facing this direction"
- "Putting it all together, I'm at position X, Y and facing angle Z"

This is updated 50 times per second, so the robot always knows (pretty much) where it is.

## Following a Path: Waypoints

A **waypoint** is just a spot on the field like "(10 feet forward, 5 feet left)."

**Simple path:** Point A → Point B → Point C

**What happens:**
1. Robot calculates: "I'm here, I need to get to Point A"
2. It drives toward Point A
3. Once it reaches Point A, it moves to Point B
4. Repeat until path is done

## The Smooth Curve (Spline)

Instead of driving to each waypoint and stopping, we want smooth, continuous movement.

**Here's how:**
1. You click 2 waypoints on the dashboard
2. The dashboard draws a smooth curve between them (using Catmull-Rom math—don't worry about it)
3. The robot follows that curve in real-time:
   - It knows which point on the curve it's closest to
   - It aims for the next point on the curve
   - It moves smoothly without stopping

This means the robot can move faster and more gracefully than if it stopped at every waypoint.

## How the Robot Decides What to Do

The robot runs a **path follower** algorithm 50 times per second:

```
1. Where am I right now?  (from odometry)
2. Where should I be on the path?  (math)
3. What direction should I face?  (tangent to the curve)
4. What speed should I drive?  (based on path speed settings)
5. Send commands to wheel motors
6. Repeat 50 times per second
```

## Real vs. Planned

**In theory:** The robot follows the exact path you drew

**In practice:** 
- Wheel slip causes drift
- Gyro has slight errors
- The field is never perfectly smooth
- Motor speeds vary slightly

That's why we use **PID controllers**—they constantly adjust wheel speeds to keep the robot on track.

## Key Files

- `waypoint_navigator.py` - Handles path following
- `swerve/odometry.py` - Calculates robot position
- `swerve/swerve_drive.py` - Controls wheel motors
- `swerve/pid_controller.py` - Keeps motors at target speeds

## Testing Your Path

1. **On the dashboard:** Draw your path and click "Send to Robot"
2. **Make sure the field is clear** - Move your robot to a safe starting location
3. **Enable the robot and select Autonomous mode**
4. **Press the auto button** - Watch the robot drive your path

If it doesn't follow the path correctly:
- Check wheel calibration
- Adjust motor speed tuning constants
- Re-draw the path and try again

---

**Questions?** Check the [README](../README.md) for more info or ask your team lead.
