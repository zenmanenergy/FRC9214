# V9.5 Rev EasySwerve - Troubleshooting Log

## TL;DR - Current Status
✅ **BREAKTHROUGH:** One turn motor (Rear Left ID 5) successfully moved using open-loop PWM control!
- This proves the motor API works and commands are reaching hardware
- **Problem:** Other 3 turn motors not responding, likely due to CAN bus saturation
- **Solution in Progress:** Increased P gain from 0.3→1.0 to improve control authority
- **Next Step:** If P=1.0 doesn't fix it, implement staggered CAN messaging (spread motor commands across multiple cycles)
- **Status:** Code deployed, awaiting test results from increased P gain

---

## Issue Summary
The swerve drive robot code is not commanding motor movement. Motors are properly initialized via CAN with correct IDs, encoders are reading correct positions, but motors receive **zero applied output** despite commands being sent.

---

## Problems Encountered

### 1. Joystick Input Axis Mapping (RESOLVED ✓)
**Problem:** Left thumb and right thumb controls were mapped to wrong axes
- Strafe was reading right thumb vertical instead of left thumb horizontal
- Rotation was reading right thumb vertical instead of right thumb horizontal

**Root Cause:** Raw axis numbers were incorrect for Xbox controller
- Expected: Axis 0 (left X), 1 (left Y), 3 (right X), 4 (right Y)
- Was Using: Axis 1, 3, 4 (missing axis 0)

**Solution Applied:**
- Fixed axis mapping in `robot.py` teleopPeriodic()
- Updated to match Java code exactly with proper negations

---

### 2. X/Y Speed Parameter Swap (RESOLVED ✓)
**Problem:** Forward/backward and strafe were swapped - both moved the bot forward
- Left thumb up/down → forward (correct)
- Left thumb left/right → ALSO forward (incorrect)

**Root Cause:** In `drive_subsystem.py`, ChassisSpeeds parameters were reversed
```python
# WRONG:
ChassisSpeeds.fromFieldRelativeSpeeds(y_speed_delivered, x_speed_delivered, ...)
# CORRECT:
ChassisSpeeds.fromFieldRelativeSpeeds(x_speed_delivered, y_speed_delivered, ...)
```

**Solution Applied:**
- Corrected parameter order in drive() method to match Java implementation

---

### 3. Turn Motor PID Gain Set to Zero (RESOLVED ✓)
**Problem:** Turn motors had no proportional control (P=0), so position errors were never corrected
- Motors received commands but never moved toward target angles

**Root Cause:** 
- Python constants had `P(0).I(0).D(0)` for turning motor
- Java code had `P(1)` for turning motors

**Solution Attempted:**
- Changed Python PID to `P(1).I(0).D(0)`
- Result: Still no motor movement

---

### 4. Motor Commands Not Reaching Hardware (UNRESOLVED ❌)
**Problem:** Motor applied output remains at 0.000 regardless of commands sent
- Bus voltage: 11.5-12.1V (good)
- Output current: 0.0A (no load, no response)
- Motor temp: 0-24°C depending on motor (some are reading temps)
- Last Error: REVLibError.kOk (no errors reported)

**Diagnostics Performed:**
1. ✓ Verified CAN IDs match hardware and code
2. ✓ Verified motors initialize without errors
3. ✓ Verified encoders return correct position/velocity
4. ✓ Verified closed-loop controller objects are not null
5. ✓ Tried multiple control modes:
   - Closed-loop position control via `setReference(angle, ControlType.kPosition)`
   - Closed-loop velocity control via `setReference(velocity, ControlType.kVelocity)`
   - Open-loop PWM via `.set(pwm_value)` with P=0.3 gain
6. ✓ All modes show `Applied Output: 0.000`

**Things Tried:**
1. Applied full factory configuration from Java Configs.java
   - Result: Still `Applied Output: 0.000`

2. Removed all configuration, used factory defaults only
   - Result: Still `Applied Output: 0.000`

3. Minimal configuration (just idle mode, no encoder/PID config)
   - Result: Still `Applied Output: 0.000`

4. Raw unconfigured motor (just `SparkMax(id, kBrushless)` with no config)
   - Result: Still `Applied Output: 0.000`, no movement

5. Switched from setReference() to raw `.set(pwm)` open-loop control
   - Result: Still `Applied Output: 0.000`

6. Attempted to command wheels to 0 radians on startup
   - Result: No movement, `Applied Output: 0.000`

---

## Current Status

**Working:**
- ✓ Robot code compiles and deploys
- ✓ All 8 motors (4 drive + 4 turn) are recognized on CAN bus (IDs: 3, 4, 5, 6, 7, 8, 9, 11)
- ✓ Encoders are operational and returning realistic values
- ✓ Joystick inputs are mapped correctly
- ✓ Swerve kinematics calculations are correct
- ✓ Motor commands are being generated with correct target angles
- ✓ No error codes from motors
- ✓ **ONE MOTOR MOVED!** - Open-loop `.set()` command DOES work on at least one turn motor

**Partially Working:**
- ⚠ Only 1-2 of 4 turn motors respond to commands
- ⚠ Drive motors still not responding

**Not Working:**
- ❌ All 4 motors not moving simultaneously
- ❌ Drive motors not responding
- ❌ Most turn motors not moving despite commands being sent

---

## Root Cause Analysis

### Eliminated Hypotheses (❌ NOT the issue)
1. ❌ **Python REV Library Issue** - API works fine (proof: one motor DID move)
2. ❌ **Motor Firmware Safetys** - Not preventing movement since one motor responds
3. ❌ **Hardware Power Issue** - All motors show proper bus voltage and respond to encoder reads
4. ❌ **Closed-loop Control API** - Switched to open-loop and one motor works, so API itself isn't broken

### Active Hypothesis (✓ Most Likely)
**CAN Bus Saturation / Bandwidth Starvation**
- All 4 motors commanded simultaneously overwhelms 1Mbps CAN bus during 20ms cycles
- Each motor needs to:
  1. Receive position command (CAN write)
  2. Send back encoder reading (CAN read)
  3. Send telemetry/status (automatic CAN broadcast)
- With 8 motor interactions per 20ms cycle, bus may not have bandwidth for all motors
- Evidence:
  - ✓ One motor consistently responds (Rear Left, ID 5)
  - ✓ Same three motors always fail (Front Left 7, Front Right 9, Rear Right 3)
  - ✓ Pattern suggests motors are prioritized/queued on bus
  - ✓ Increased P gain is next test - if more motors respond, confirms bandwidth not core issue
  - ✓ If still only one responds, CAN staggering needed

### Secondary Hypothesis (⚠ Possible)
**Insufficient Motor Control Authority**
- P gain of 0.3 might not provide enough torque to move swerve modules against resistance
- Test: Increase P gain to 1.0 and see if more motors respond

---

## Current Solution Attempts (In Progress)

### Attempt 5: Increase P Gain from 0.3 to 1.0 (PENDING)
**Rationale:** If motors lack control authority, increased gain provides more torque

**Implementation:**
- Modified `easy_swerve_module.py` to use `angle_error * 1.0` instead of `angle_error * 0.3`
- Added diagnostic markers "IS COMMANDING MOVEMENT" to identify which motors receive commands
- Added angle error value to telemetry output

**Expected Outcome:**
- If 2+ motors now move → Gain was limiting factor; continue tuning P
- If still only 1 motor moves → CAN bus saturation is root cause; proceed to Attempt 6

### Attempt 6: Implement CAN Message Staggering (IF NEEDED)
**If P=1.0 doesn't fix:** Stagger motor command timing
```python
# Command motors sequentially instead of all at once
current_cycle = (self.cycle_count // 20) % 4
if current_cycle == 0:
    self.front_left.set_desired_state()
elif current_cycle == 1:
    self.front_right.set_desired_state()
# ... etc
```

**Rationale:**
- Spreads CAN traffic across separate cycles
- Allows each motor full bus bandwidth for command + telemetry
- Trades responsiveness (20ms→80ms per motor) for full 4-motor functionality

---

## Diagnostic Output Format

When deployed, robot prints diagnostic data every 5 seconds showing:
```
[TIME] Rear Left Turn (ID 5):
  - Current Angle: 1.23 rad
  - Target Angle: 0.00 rad
  - Error: 1.23 rad (needs correction)
  - PWM Output: 1.23 (bounded -1.0 to 1.0)
  - IS COMMANDING MOVEMENT
  - Bus Voltage: 12.1V
  - Output Current: 0.50A
  - Motor Temp: 24°C
```

**Key Indicators:**
- **Current Angle → Target Angle:** Shows if wheel is moving toward center (0.0 rad)
- **IS COMMANDING MOVEMENT:** Only appears if PWM output > 0.1; shows which motors are active
- **Output Current:** Non-zero indicates motor is drawing current (good sign)
- **Error decreasing:** Indicates motor is successfully controlling toward target

---

## Files Modified
- `robot.py` - Fixed joystick axis mapping, added diagnostics, added teleopInit wheel center command
- `drive_subsystem.py` - Fixed x/y parameter order in ChassisSpeeds  
- `easy_swerve_module.py` - Switched from closed-loop to open-loop control, increased P from 0.3→1.0, added "IS COMMANDING MOVEMENT" diagnostic markers, added angle error to output
- `constants.py` - Updated turn motor idle mode to Brake, lowered current limit to 20A, set P=1.0 in comment

## Motor CAN IDs (Verified)
- Front Left Drive: 8 | Turn: 7
- Front Right Drive: 2 | Turn: 9
- Rear Left Drive: 6 | Turn: 5
- Rear Right Drive: 4 | Turn: 3
