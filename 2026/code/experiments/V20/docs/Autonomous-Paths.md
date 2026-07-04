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

A **waypoint** is just a spot on the field with:
- **X position** - How far forward/backward (in cm)
- **Y position** - How far left/right (in cm)
- **Heading** - Which direction the robot should face when it reaches this waypoint (in degrees)

**Simple path:** Waypoint A → Waypoint B → Waypoint C

**What happens:**
1. Robot calculates: "I'm here, I need to get to Waypoint A at (200, 100) facing 45°"
2. It drives toward Waypoint A, rotating to face 45° along the way
3. Once it reaches Waypoint A, it moves to Waypoint B
4. Repeat until path is done

## The Smooth Curve (Spline)

Instead of driving to each waypoint and stopping, we want smooth, continuous movement.

**Here's how:**
1. You click 2+ waypoints on the dashboard
2. The dashboard draws a smooth curve between them (using Catmull-Rom spline math)
3. The robot follows that curve in real-time:
   - It measures how far along the curve it has traveled
   - It adjusts its position to stay ON the curve (not drift off to the side)
   - It rotates smoothly to match the curve's direction
   - It moves smoothly without stopping

This means the robot can move faster and more gracefully than if it stopped at every waypoint.

## How the Robot Decides What to Do

The robot runs a **path follower** algorithm 50 times per second:

```
1. Where am I right now?  (from odometry)
2. Where should I be on the path?  (math)
3. What direction should I face?  (tangent to the curve)
4. What speed should I drive?  (based on path speed settings)
5. How far have I traveled along the curve?  (accumulate distance)
6. Send commands to wheel motors
7. Repeat 50 times per second
```

## Real vs. Planned

**In theory:** The robot follows the exact path you drew

**In practice:** 
- Wheel slip causes drift
- Gyro has slight errors
- The field is never perfectly smooth
- Motor speeds vary slightly

That's why we use **PID controllers**—they constantly adjust wheel speeds and rotation to keep the robot on track.

---

## Functions You'll Use

### Setting Up and Running Paths

#### `set_waypoints(waypoints, use_spline=False)`

Tell the navigator where you want the robot to go.

**Parameters:**
- `waypoints` - List of waypoint dictionaries. Each waypoint should have:
  - `"x"` - Forward position in cm
  - `"y"` - Sideways position in cm  
  - `"heading"` - Direction to face in degrees (0° = forward, 90° = right)
- `use_spline` - If `True`, robot follows a smooth curve; if `False`, robot goes waypoint-to-waypoint

**Example - from robot.py:**
```python
# This happens when dashboard sends a waypoint command
import json

waypoints_json = SmartDashboard.getString("navigation_waypoints_json", "[]")
waypoints = json.loads(waypoints_json)

# waypoints might look like:
# [
#   {"x": 0, "y": 0, "heading": 0},      # Start at origin facing forward
#   {"x": 200, "y": 100, "heading": 45}, # Move to (200cm, 100cm) facing 45°
#   {"x": 400, "y": 0, "heading": 0}     # Move to (400cm, 0cm) facing forward
# ]

self.navigator.set_waypoints(waypoints, use_spline=True)
```

**What happens inside:**
- Navigator validates the waypoints
- If using spline, it creates a smooth Catmull-Rom curve through all waypoints
- Stores the path ready to execute

---

#### `start()`

Begin following the waypoints you've set.

**Returns:** `True` if started successfully, `False` if no waypoints are set

**Example:**
```python
# Set up the path first
self.navigator.set_waypoints(waypoints, use_spline=True)

# Then start it
if self.navigator.start():
    print("Navigation started!")
else:
    print("Error: no waypoints set!")
```

**What happens:**
- Navigator resets internal state
- Sets the first waypoint as the target
- Begins executing the path
- Returns `is_active = True`

---

#### `update()`

Call this every robot cycle to execute the navigation. The navigator moves the robot toward waypoints.

**Should be called every ~20ms (50 times per second)**

**Example - from robot.py teleopPeriodic():**
```python
def teleopPeriodic(self):
    # Check for waypoint command from dashboard
    if SmartDashboard.getBoolean("navigate_waypoints_command", False):
        waypoints = json.loads(SmartDashboard.getString("navigation_waypoints_json", "[]"))
        use_spline = SmartDashboard.getBoolean("navigation_use_spline", False)
        self.navigator.set_waypoints(waypoints, use_spline=use_spline)
        self.navigator.start()
    
    # Update navigator (this does all the math and moves the robot)
    self.navigator.update()
    
    # Only accept pilot input if NOT navigating
    if not self.navigator.is_active:
        self.pilot_controls.execute_teleop()
```

**What happens inside:**
1. Reads current robot position from odometry
2. Checks which waypoint we're targeting
3. Calculates angle and distance to waypoint
4. Sends commands to swerve drive to move toward it
5. Checks if we've reached the waypoint
6. Moves to next waypoint if done, or stops if all complete

---

#### `stop()`

Stop navigation immediately and stop all robot motors.

**Example:**
```python
self.navigator.stop()  # Emergency stop
```

**When to use:**
- Dashboard sends stop command
- Autonomous mode ends
- Emergency stop button pressed

---

### Checking Navigation Status

#### `is_active`

Boolean property that tells you if navigation is currently running.

**Returns:** `True` if actively following waypoints, `False` if idle

**Example:**
```python
if self.navigator.is_active:
    print("Robot is navigating")
    # Don't give joystick input
else:
    print("Robot is waiting for input")
    # OK to accept joystick input
```

---

#### `is_finished()`

Check if all waypoints have been completed.

**Returns:** `True` if navigation completed all waypoints, `False` otherwise

**Example:**
```python
if self.navigator.is_finished():
    print("Path complete!")
    self.navigator.stop()
```

---

#### `get_status()`

Get detailed information about current navigation progress.

**Returns:** Dictionary with:
- `active` - Is navigation running?
- `stage` - Internal stage (1=rotate, 2=move, 3=done)
- `waypoint` - Current waypoint number (1-indexed)
- `total` - Total number of waypoints
- `target` - Target waypoint dict (x, y, heading)
- `current_x`, `current_y` - Robot's current position
- `heading` - Robot's current heading
- `progress` - String description like "Waypoint 2/4 Driving"

**Example:**
```python
status = self.navigator.get_status()
print(f"Progress: {status['progress']}")
print(f"Target: ({status['target']['x']}, {status['target']['y']})")
print(f"Current: ({status['current_x']:.1f}, {status['current_y']:.1f})")
```

---

### Odometry (Robot Position Tracking)

The navigator uses odometry to know where it is. You can also query odometry directly:

#### `self.navigator.drive.odometry.get_position()`

Get the robot's current X, Y position.

**Returns:** Tuple of `(x, y)` in centimeters

**Example:**
```python
x, y = self.navigator.drive.odometry.get_position()
print(f"Robot is at ({x:.1f}, {y:.1f})")
```

---

#### `self.navigator.drive.odometry.get_heading()`

Get the robot's current rotation angle.

**Returns:** Heading in degrees (0-360°)

**Example:**
```python
heading = self.navigator.drive.odometry.get_heading()
print(f"Robot is facing {heading:.1f}°")
```

---

#### `self.navigator.drive.odometry.reset()`

Reset odometry to (0, 0) at heading 0°. Call this at the start of autonomous.

**Example:**
```python
def autonomousInit(self):
    self.swervedrive.odometry.reset()  # Fresh start for autonomous
    self.navigator.start_path()
```

---

### Advanced: Tuning Navigation

#### `autotune_rotation(target_heading=45.0, max_power=0.3, duration_seconds=10.0)`

Advanced function to automatically calculate the best PID gains for rotation.

**This requires:**
- Robot on the ground (not elevated)
- Clear space to rotate
- At least 10 seconds
- Robot won't accept joystick input during this

**Parameters:**
- `target_heading` - Degrees to rotate to (default 45°)
- `max_power` - Maximum motor power (default 0.3)
- `duration_seconds` - How long to run test (default 10 seconds)

**Returns:** Dictionary with:
- `success` - Did tuning work?
- `kp`, `ki`, `kd` - New PID gains (if successful)
- `period` - Oscillation period measured
- `message` - Why it failed (if failed)

**Example:**
```python
# Run autotune (takes 10+ seconds)
result = self.navigator.autotune_rotation(
    target_heading=45.0,
    max_power=0.3,
    duration_seconds=15.0
)

if result['success']:
    print(f"New gains: kp={result['kp']}, ki={result['ki']}, kd={result['kd']}")
else:
    print(f"Autotune failed: {result['message']}")
```

**What's happening:**
1. Robot aligns wheels to rotation stance (45°, 135°, 225°, 315°)
2. Rotates back and forth trying to reach target heading
3. Measures how it oscillates
4. Calculates optimal PID gains to stop overshooting
5. Saves gains for future autonomous runs

---

## How Navigation Works Step-by-Step

When you call `navigator.update()` during autonomous:

```
1. Read current position and heading from odometry
2. Load current waypoint target
3. Calculate distance to target waypoint
4. If too far away:
   - Use PID to calculate rotation needed
   - Use PID to calculate forward speed needed
   - Send drive_swerve() command to robot
5. If close enough to waypoint:
   - Move to next waypoint in list
   - Repeat
6. If all waypoints done:
   - Stop robot
   - Set is_active = False
```

## Smooth vs. Waypoint-to-Waypoint

### Without Spline (`use_spline=False`)
```
Start → Stop at WP1 → Stop at WP2 → Stop at WP3 → Done
```
Slower, but each waypoint is a clear checkpoint.

### With Spline (`use_spline=True`)
```
Start → Smooth curve through WP1 → Smooth curve through WP2 → Done
```
Faster, smoother, but slightly less predictable (can overshoot if too fast).

---

## Key Files

- `waypoint_navigator.py` - Handles path following
- `swerve/odometry.py` - Calculates robot position
- `swerve/swerve_drive.py` - Controls wheel motors
- `swerve/pid_controller.py` - Keeps motors at target speeds
- `swerve/catmull_rom.py` - Smooth curve math for splines

## Testing Your Path

1. **On the dashboard:** Draw your path and click "Send to Robot"
2. **Make sure the field is clear** - Move your robot to a safe starting location
3. **Enable the robot and select Autonomous mode**
4. **Press the auto button** - Watch the robot drive your path

If it doesn't follow the path correctly:
- Check wheel calibration
- Adjust spline deceleration distance (increase if overshooting)
- Increase PID gains if robot is too slow to respond
- Try without spline first to verify basic waypoint-following works

---

**Questions?** Check the [README](../README.md) for more info, or learn more about [testing with the simulator](Simulator-Guide.md) or ask your team lead.
