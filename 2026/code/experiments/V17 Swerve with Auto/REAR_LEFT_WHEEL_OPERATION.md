# Rear-Left Wheel Operation

## Overview
The rear-left wheel is the primary reference wheel for the swerve drive system. It controls forward/backward movement and directional turning.

## Drive Motor (Forward/Backward)
**File:** `wheel.py` lines 36-43

The drive motor receives and applies power directly:
```python
def set_drive_power(self, power):
    if self.name == "rear_left":
        if self.drive_motor:
            self.drive_motor.set(power)


```

- **Input:** Power value from kinematics (-1.0 to 1.0)
- **Processing:** Negated at calculation level (`-target_speed * config.MOTOR_SCALE_TELEOP`)
- **Output:** Motor spins proportionally to requested movement
- **Role:** Propels the wheel forward or backward based on joystick input

## Turn Motor (Rotation/Steering)
**File:** `wheel.py` lines 45-57

The turn motor controls wheel angle for direction changes:
```python
def set_turn_power(self, power):
    if self.name == "rear_left":
        if self.turn_motor:
            self.turn_motor.set(power)
```

- **Input:** PID-controlled power from alignment system
- **Processing:** Direct pass-through (no inversion)
- **Output:** Motor rotates wheel to target angle
- **Role:** Aligns wheel to calculated direction

## Angle Calculation Pipeline
**File:** `swerve_drive.py` lines 116-161

### Step 1: Kinematics Calculation (Lines 119-134)
Raw wheel vectors are calculated using swerve kinematics:
```python
vx = -forward - rotate * wheel_pos["y"]
vy = strafe + rotate * wheel_pos["x"]
angle = math.degrees(math.atan2(vy, vx))
```

### Step 2: Global Input Inversion (Line 133)
All calculated angles are globally inverted by 180°:
```python
angle = (angle + 180) % 360  # Invert all inputs globally
```

### Step 3: Per-Wheel Target Calculation (Line 161)
Each wheel's target angle is further adjusted:
```python
target_angle = (target_angle + 180) % 360  # Flip 0° and 180° positions
```

**Result:** Combined 360° transformation (cancels to original orientation with corrections)

## Alignment System
**File:** `swerve_drive.py` lines 188-196 & 351-395

The rear-left wheel maintains its angle through continuous PID feedback:

1. **Angle Storage** (Lines 188-196): Target angles are stored when `target_speed > 0.01`
2. **Alignment Update** (Lines 351+): PID controller continuously adjusts turn motor power
3. **Tolerance** (Line 391): Wheel stops turning when within `config.ALIGN_TOLERANCE` of target
4. **Hold Position** (Line 189): Stores angle even when input released, so wheel maintains last direction

## Control Flow Example: Forward Movement

1. **Input:** Joystick pushed forward (forward = 1.0, strafe = 0, rotate = 0)
2. **Kinematics:** 
   - vx = -1.0 - 0 = -1.0
   - vy = 0 + 0 = 0
   - angle = 180°
3. **Global Inversion:** angle = (180 + 180) % 360 = 0°
4. **Per-Wheel Flip:** target_angle = (0 + 180) % 360 = 180°
5. **Alignment:** Turn motor aligns wheel to 180°
6. **Drive:** Drive motor applies negative power to move forward

## Control Flow Example: Left Strafe

1. **Input:** Joystick pushed left (forward = 0, strafe = -1.0, rotate = 0)
2. **Kinematics:**
   - vx = 0 - 0 = 0
   - vy = -1.0 + 0 = -1.0
   - angle = 270° (atan2 of -1.0, 0)
3. **Global Inversion:** angle = (270 + 180) % 360 = 90°
4. **Per-Wheel Flip:** target_angle = (90 + 180) % 360 = 270°
5. **Alignment:** Turn motor aligns wheel to 270°
6. **Drive:** Drive motor applies proportional backward power to strafe left

## Motor Inversion Summary

| Motor | Inversion | Location | Reason |
|-------|-----------|----------|--------|
| Drive | Yes (-) | `swerve_drive.py` line 169 | Required to match kinematics after angle flips |
| Turn | No | `wheel.py` line 51 | Direct PID output for proper alignment |

## Key Configuration Values
- **MOTOR_SCALE_TELEOP:** Power scaling during normal driving
- **MOTOR_SCALE_ALIGN:** Power scaling during wheel alignment
- **ALIGN_TOLERANCE:** Angle error threshold before stopping alignment (degrees)
- **Encoder DIO:** Absolute position feedback for each wheel

## Behavior Notes

- **Holds Position:** When joystick is released, wheel maintains last angle (alignment stays active)
- **No Oscillation:** Proper inversion prevents motor fighting kinematics
- **Smooth Diagonals:** Combines forward/strafe inputs proportionally in kinematics
- **Rotation Support:** Right stick rotation offset is added to kinematics via `rotate * wheel_pos`


## when writing code, seperate each motor into unique if statements like the function below
```python

def set_drive_power(self, power):
    if self.name == "rear_left":
        if self.drive_motor:
            self.drive_motor.set(power)
    if self.name == "rear_right":
        if self.drive_motor:
            self.drive_motor.set(power)

    if self.name == "front_left":
        if self.drive_motor:
            self.drive_motor.set(power)

    if self.name == "front_right":
        if self.drive_motor:
            self.drive_motor.set(power)
```
## DO NOT combine if statements as replicated below
```python
def set_drive_power(self, power):
    if self.name == ["rear_left", "rear_right"]:
        if self.drive_motor:
            self.drive_motor.set(power)
```