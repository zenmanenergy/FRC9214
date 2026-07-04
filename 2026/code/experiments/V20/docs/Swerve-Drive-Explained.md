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

## Functions You'll Use

### Basic Movement

#### `drive_swerve(forward, strafe, rotate)`

This is the main function for **teleop mode** (when a human is controlling the robot). It combines all three types of movement in one command.

**Parameters:**
- `forward` - How much to move forward (0.0 = stop, 1.0 = full forward, -1.0 = full backward)
- `strafe` - How much to move sideways (0.0 = none, 1.0 = full right, -1.0 = full left)
- `rotate` - How much to rotate (0.0 = no rotation, 1.0 = full left spin, -1.0 = full right spin)

**Example - from pilot_controls.py:**
```python
# This happens every time the joystick moves
left_y = joystick.get_left_y()    # Forward/backward stick
left_x = joystick.get_left_x()    # Sideways stick
right_x = joystick.get_right_x()  # Rotation stick

# Send all three to swerve drive
self.drive.drive_swerve(left_y, left_x, right_x)
```

**What happens inside:**
1. Robot calculates what each wheel should do
2. Wheels aim to their target angles using turn motors
3. Wheels spin with target speed using drive motors
4. All four wheels work together to create the desired motion

---

#### `stop_all()`

Immediately stops all motors and clears any pending commands.

**Example:**
```python
self.swervedrive.stop_all()  # Complete stop
```

**When to use:**
- When switching modes (teleop → autonomous)
- Emergency stop
- End of autonomous routine

---

#### `drive_straight(speed, target_angle=0.0)`

Drive in a straight line at a specific angle, without rotation.

**Parameters:**
- `speed` - Motor power (-1.0 to 1.0, negative = backward)
- `target_angle` - Direction to drive (0° = forward, 90° = right, 180° = backward, 270° = left)

**Example:**
```python
# Drive forward at 50% power
self.drive.drive_straight(0.5, target_angle=0.0)

# Drive sideways (strafe) at 75% power
self.drive.drive_straight(0.75, target_angle=90.0)

# Stop
self.drive.drive_straight(0.0, target_angle=0.0)
```

---

#### `drive_rotation(rotation_input)`

Rotate the robot in place while staying at a fixed orientation.

**Parameters:**
- `rotation_input` - Rotation speed (-1.0 to 1.0, positive = counter-clockwise)

**Example:**
```python
# Right stick only (no forward movement) - rotate in place
self.drive.drive_rotation(right_stick_input)
```

**How it works:**
- All four wheels point to the corners (45°, 135°, 225°, 315°)
- Wheels spin in the same direction to make the robot rotate
- Robot rotates around its center point

---

### Checking Robot State

#### `is_moving()`

Checks if the robot is currently moving or aligning.

**Returns:** `True` if moving or aligning, `False` if idle

**Example:**
```python
if self.drive.is_moving():
    print("Robot is in motion")
else:
    print("Robot is stopped")
```

---

#### `is_aligning()`

Checks if wheels are currently adjusting their angles.

**Returns:** `True` if wheels are turning to aim, `False` if aligned

**Example:**
```python
# Only give joystick input if not aligning
if not self.drive.is_aligning():
    self.drive.drive_swerve(forward, strafe, rotate)
```

---

### Getting Robot Information

#### `get_wheel_angle(wheel_name)`

Get the current angle of a specific wheel.

**Parameters:**
- `wheel_name` - One of: `"front_left"`, `"front_right"`, `"rear_left"`, `"rear_right"`

**Returns:** Current wheel angle in degrees (0-360°)

**Example:**
```python
fl_angle = self.drive.get_wheel_angle("front_left")
print(f"Front left wheel is pointing at {fl_angle}°")
```

---

#### `get_wheel_power(wheel_name)`

Get the current power being sent to a wheel's drive motor.

**Parameters:**
- `wheel_name` - Wheel to check

**Returns:** Power value (-1.0 to 1.0)

**Example:**
```python
power = self.drive.get_wheel_power("front_right")
if power > 0.5:
    print("Front right is spinning hard")
```

---

#### `get_motor_current(wheel_name)`

Get the electrical current being drawn by a wheel's motors (in Amps). This is useful for detecting if a wheel is stuck or hitting something.

**Parameters:**
- `wheel_name` - Wheel to check

**Returns:** Current in Amps

**Example:**
```python
# Detect if wheel is struggling (high current = stuck or high load)
if self.drive.get_motor_current("rear_left") > 40.0:
    print("Rear left wheel is drawing high current - might be stuck!")
```

---

### Odometry (Tracking Position)

The robot tracks its position by measuring wheel rotation and gyro heading.

#### `self.swervedrive.odometry.get_position()`

Gets the robot's current X, Y position on the field.

**Returns:** Tuple of `(x, y)` in centimeters from the starting point

**Example:**
```python
x, y = self.drive.odometry.get_position()
print(f"Robot is at X={x:.1f}cm, Y={y:.1f}cm")
```

---

#### `self.swervedrive.odometry.get_heading()`

Gets the robot's current rotation angle.

**Returns:** Heading in degrees (0-360°)

**Example:**
```python
heading = self.drive.odometry.get_heading()
print(f"Robot is facing {heading}°")
```

---

#### `self.swervedrive.odometry.reset()`

Reset the odometry to (0, 0) at heading 0°. Call this at the start of a match so you know your starting position.

**Example:**
```python
def teleopInit(self):
    self.swervedrive.odometry.reset()  # Fresh start
```

---

### Calibration (Advanced)

#### `set_wheel_zero(wheel_name)`

Set the current wheel angle as the "zero" reference point. Used during calibration.

**Example:**
```python
# Point the wheel forward and call this
self.drive.set_wheel_zero("front_left")
```

---

#### `set_wheel_angle(wheel_name, target_angle)`

Manually calibrate a wheel to point at a specific angle.

**Example:**
```python
# Tell the robot "this wheel is now pointing at 90°"
self.drive.set_wheel_angle("front_left", 90.0)
```

---

## How It All Works Together

Here's a typical teleop cycle:

1. **Get joystick input** (pilot_controls.py)
2. **Call `drive_swerve()`** with the input values
3. **Swerve calculates** what each wheel should do
4. **Wheels aim** using turn motors (uses PID controllers to get exact angles)
5. **Wheels spin** using drive motors with the right power
6. **Repeat 50 times per second**

```python
# This happens 50x per second in teleopPeriodic():
def teleopPeriodic(self):
    # Get joystick values
    forward = joystick.get_left_y()
    strafe = joystick.get_left_x()
    rotate = joystick.get_right_x()
    
    # Send to swerve
    self.swervedrive.drive_swerve(forward, strafe, rotate)
    
    # Update internal systems
    self.swervedrive.odometry.update()
    self.swervedrive.update_single_wheel_alignment()
```

---

**Next:** Learn how to [test the robot using the simulator and dashboard](Simulator-Guide.md)
