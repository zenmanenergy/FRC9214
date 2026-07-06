# Spline Path Following - Overshoot Investigation

**Date**: 2026-07-03  
**Issue**: Robot overshoots end waypoints by ~50-60cm, especially in Y direction

## Current Problem

Path: (0,0) → (93,90) → (239,111) with heading 0° throughout  
Expected: Robot follows smooth curve and stops at end point  
Actual: Robot overshoots to approximately (254, 167) - missing target by 56cm in Y

### Observations
- Lateral error showing 41-50cm (clamped at 50cm) throughout end of path
- Robot is moving in correct general direction but not stopping when it should
- At end, robot rotates left/right multiple times to find heading = 0°
- Path should only require strafing (Y correction), minimal heading change

## Changes Applied So Far

### 1. Fixed Heading Calculation (Latest)
**File**: `swerve/catmull_rom.py`
- Added `interpolate_heading(segment_idx, t)` method
- Now uses **waypoint headings** for desired heading, not tangent direction
- If both waypoints have heading=0°, desired heading stays 0° throughout path
- Handles heading wrap-around correctly (350° → 10° smooth interpolation)

### 2. Path Following Algorithm (Latest)
**File**: `waypoint_navigator.py` → `_update_spline()`
- **Forward**: Always moves along tangent direction (unit vector along curve)
- **Strafe**: Proportional correction based on lateral error from path
- **Heading**: Interpolated from waypoint headings (stable, no oscillation)

### 3. Parameter Tuning
**File**: `waypoint_navigator.py`
```python
# Braking distance increased
self.decel_distance = 150.0  # Was 80cm, now starts braking earlier

# Strafe smoothing made faster
self.smooth_robot_strafe = self.smooth_robot_strafe * 0.3 + raw_robot_strafe * 0.7
# Was: 0.5/0.5 split, now 70% new / 30% old for faster response

# Lateral error gain
lateral_correction = (lateral_error * 0.05)  # 20cm error = 1.0 strafe
```

## Current Investigation: Sign Flip Test

**File**: `waypoint_navigator.py` line ~110  
**Change**: Negated lateral correction direction

```python
# OLD: raw_robot_strafe = (perp_x * sin_h + perp_y * cos_h) * lateral_correction
# NEW: raw_robot_strafe = (perp_x * sin_h + perp_y * cos_h) * (-lateral_correction)
```

**Why**: The perpendicular vector calculation might be backwards. If robot overshoots RIGHT and needs to strafe LEFT to correct, but is strafing RIGHT instead, the perpendicular direction is inverted.

**Debug Output Added**:
```
[NAVIGATOR-SPLINE-DEBUG] LatErr_raw=50.0cm LatCorr=1.000 RawStrafe=0.800 SmoothStrafe=0.750
```

## What to Look For When Testing

### Scenario: Path overshoots in +Y direction (right on field)
Ideal behavior:
- `LatErr_raw` shows positive values (drifting right)
- `RawStrafe` should be **negative** (command to strafe LEFT to correct)
- Robot should visibly strafe toward the path, not away from it

### If Sign is Correct ✓
- Lateral error should decrease as robot moves along path
- Robot should stop closer to target (less overshoot)
- **Next step**: Increase lateral error gain (try 0.07 or 0.08) for more aggressive correction

### If Sign is Wrong ✗
- Robot will strafe away from path even more
- Lateral error will grow toward 50cm clamp
- **Action needed**: Flip sign back (remove the `-`)

### If Sign Doesn't Matter
- Lateral error stays constant (~50cm) regardless
- **Problem is elsewhere**: Possibly frame conversion, speed scaling, or distance measurement

## Debug Log Checklist

When you test, capture logs showing:
1. **Start of path**: First 5 lines of `[NAVIGATOR-SPLINE-DEBUG]`
2. **Middle of path**: 5 lines around 50% distance
3. **End of path**: Last 5 lines before reaching target

Look for patterns in:
```
LatErr_raw:    Should decrease toward 0 (ideal: 50 → 30 → 10 → 5)
LatCorr:       Should scale with LatErr_raw (proportional)
RawStrafe:     Should change sign if correcting (not stuck same direction)
SmoothStrafe:  Should smooth out RawStrafe spikes
```

## If Overshoot Still Persists

### Possible Root Causes to Investigate

1. **Distance measurement wrong**
   - `Dist=237.6/278.5cm` — is this accurate?
   - Check if odometry is drifting significantly

2. **Velocity profiling not aggressive enough**
   - Current: decel starts at 150cm, linear reduction to target
   - Try: Exponential decel or increase decel_rate from 0.05 to 0.10

3. **Lateral error saturated**
   - If `LatErr_raw` always shows 50.0cm (clamped), strafe correction can't get stronger
   - Try: Increase clamp from 50cm to 100cm, OR
   - Increase gain from 0.05 to 0.10 (double the correction power)

4. **Frame conversion wrong**
   - Forward/strafe values calculated in field frame but applied to robot frame
   - The coordinate transform might have a sign error
   - Check: When robot needs to go +X in field frame, does it actually move +X?

## Quick Fixes to Try (In Order)

1. **Already applied**: Sign flip on lateral_correction
2. **If still overshooting**: Remove clamp on lateral_error (change 50cm → 100cm)
3. **If still overshooting**: Double lateral gain (0.05 → 0.10)
4. **If still overshooting**: Increase decel_distance to 200cm
5. **If ALL above fail**: Investigate distance measurement accuracy

## Files Modified

- `swerve/catmull_rom.py` - Added `interpolate_heading()` method
- `waypoint_navigator.py` - Rewrote `_update_spline()`, added debug output

## Next Session Action Items

1. Run path test, capture `[NAVIGATOR-SPLINE-DEBUG]` logs
2. Analyze lateral error trend (decreasing = correct direction)
3. Apply recommended fix based on trend
4. Re-test until lateral error drops to <10cm by path end
5. Verify overshoot reduced to <10cm total
