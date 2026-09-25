# Plan: Replace Catmull-Rom Splines with a Single Unified Straight-Line + Heading-Blend Path Follower

Status: PLANNING ONLY - no code has been changed yet.
Scope: `2026/code/experiments/V22/` only. `V20`/`V21` copies are left untouched.

## 1. Decisions locked in from review

1. Rename all spline-related identifiers (files, classes, methods, variables, SmartDashboard keys, wire-protocol fields, JS names) - nothing should still say "spline" or "catmull" when this is done.
2. `sampleSpline()` in `dashboard_ws.html` has no call sites anywhere in the codebase (confirmed via search) -> **delete it outright**, do not port it to a `sampleLinearPath()` equivalent.
3. **No more `use_spline` mode flag at all.** Today there are two different, inconsistent path-following implementations selected by a boolean (`WaypointNavigator.update()` discrete stage-mode vs `WaypointNavigator._update_spline()`), and `SwerveDrive.follow_path()/update_autonomous()` is a **third**, separate implementation that isn't even wired up to anything live. This plan collapses all of that into **one algorithm** that handles 1 waypoint or many, with no branching.
4. Scope is `2026/code/experiments/V22/` only.

## 2. Current state (confirmed by tracing the whole call chain)

- Dashboard JS `runWaypoints()` -> `sendCommand("navigate_waypoints", {..., use_spline: waypointList.length >= 2})`
- `dashboard_server.py` forwards `use_spline` into SmartDashboard key `navigation_use_spline`
- `robot.py` reads `navigation_use_spline` and calls `navigator.set_waypoints(waypoints, use_spline=use_spline)`
- `WaypointNavigator.update()` branches: if `use_spline` -> `_update_spline()` (builds `CatmullRomSpline`, walks cumulative arc-length distance, no dwell support, ends the whole run at the last waypoint); else -> the per-waypoint stage loop (`stage 1` drive+rotate to next waypoint using dx/dy vector math it already computes in a straight line, `stage 2` dwell, then `_advance_waypoint()` which supports looping and per-waypoint dwell).
- Since the dashboard always sends `use_spline=True` once there are 2+ waypoints, **`_update_spline()` is the one actually driving real multi-waypoint routes today**, and it is missing dwell/loop-safe stopping. The discrete stage loop is more correct/complete (dwell, loop, advance) but only ever runs for a single waypoint in practice, and it snaps its rotation PID directly at the final heading instead of blending from wherever the robot currently is.
- `SwerveDrive.follow_path()` / `update_autonomous()` / `is_path_complete()` build their own `CatmullRomSpline` and are never called from `robot.py` - they only exist as documented public API / docstring examples. `update_autonomous()` currently sets `rotate = 0.0` with a comment "use heading from path" - it's a dead stub that never actually applies rotation.

Conclusion: the discrete per-waypoint stage loop (straight-line vector math, dwell, loop, advance) is the right foundation to keep. We delete the spline-distance-walking implementation entirely rather than reconciling two systems.

## 3. New shared helper module: `swerve/heading_math.py`

Small, stateless, reusable functions - replaces at least 5 separate copy-pasted angle-wrap/lerp implementations found in `catmull_rom.py`, three places in `waypoint_navigator.py`, and the JS `lerpAngle()`:

```python
def shortest_angle_diff(a: float, b: float) -> float:
	"""Signed shortest difference b - a, wrapped to [-180, 180]."""
	diff = (b - a) % 360.0
	if diff > 180.0:
		diff -= 360.0
	return diff

def lerp_angle(a: float, b: float, t: float) -> float:
	"""Shortest-path interpolation from heading a to heading b, wrapped to [0, 360)."""
	result = a + shortest_angle_diff(a, b) * t
	return result % 360.0
```

Used by both `waypoint_navigator.py` and `swerve_drive.py` so there is exactly one implementation of angle wrapping in the whole system (directly addresses "no edge cases").

## 4. Core unified algorithm (used everywhere waypoints are followed)

For the **current leg** (robot's position when the leg started -> the next target waypoint):

- On leg start (navigation `start()`, or whenever `_advance_waypoint()` moves to a new target): capture
  - `leg_start_heading` = robot's actual current heading right now (not the waypoint's declared heading)
  - `leg_start_x, leg_start_y` = robot's actual current position right now
  - `leg_target_heading` = the target waypoint's declared heading
  - `leg_total_distance` = straight-line distance from `(leg_start_x, leg_start_y)` to the target waypoint (used only to compute progress; the direction vector is recomputed live from current position every tick, so drift doesn't accumulate)
- Every update tick while driving toward that target:
  - `distance_to_target` = live straight-line distance from current position to target (as today)
  - `progress = clamp(1 - distance_to_target / leg_total_distance, 0.0, 1.0)` (guard `leg_total_distance` near 0 -> `progress = 1.0`)
  - `desired_heading = lerp_angle(leg_start_heading, leg_target_heading, progress)`
  - `heading_error = shortest_angle_diff(current_heading, desired_heading)` fed into the existing rotation PID
  - forward/strafe = existing straight-line-to-target vector math (unchanged - already correct)
  - Single combined `drive_swerve(forward, strafe, rotate)` call, same as today
- Dwell, loop, advance-to-next-waypoint, and the two-constraint velocity profile (decel approaching this waypoint + decel approaching the next mandatory stop) are **all kept exactly as they exist today** in the stage loop - they already work correctly and are unrelated to the curve-vs-line question.

This single algorithm now correctly handles 1 waypoint and N waypoints identically - no flag, no branch.

## 5. File-by-file change list

### `swerve/heading_math.py` (new)
- `shortest_angle_diff()`, `lerp_angle()` as above.

### `swerve/catmull_rom.py`
- Deleted. No replacement spline/path class is needed - straight-line vector math to a target position already exists inline in the stage loop and needs no supporting class.

### `swerve/__init__.py`
- Remove `from .catmull_rom import CatmullRomSpline` and its `__all__` entry.
- Add `from .heading_math import shortest_angle_diff, lerp_angle` and export them.
- Update module docstring: drop "Autonomous path following with Catmull-Rom splines" -> "Autonomous path following with straight-line waypoint legs and gradual heading blending".

### `waypoint_navigator.py`
- Remove: `from swerve.catmull_rom import CatmullRomSpline`, `self.spline`, `self.use_spline`, `self.spline_total_distance`, `self.distance_traveled_along_spline`, `self.last_robot_x/y`, `self.spline_start_x/y`, `self.smooth_robot_forward/strafe`, `self.smooth_desired_heading`, and the entire `_update_spline()` method.
- `set_waypoints(waypoints)` - drop the `use_spline` parameter entirely.
- `update()` - remove the `if self.use_spline: self._update_spline(); return` branch. Only the stage loop remains.
- Add leg-start-state fields (`leg_start_heading`, `leg_start_x`, `leg_start_y`, `leg_total_distance`) initialized in `start()` and refreshed in `_advance_waypoint()`.
- In stage 1: replace the direct `angle_diff = target_angle - current_heading` (which snaps toward the final heading immediately) with the progress-based `desired_heading` blend from section 4, using `shortest_angle_diff`/`lerp_angle` from the new helper module.
- Rename any remaining "spline" wording in comments/log strings/docstrings.

### `swerve/swerve_drive.py`
- Remove `from .catmull_rom import CatmullRomSpline`.
- `follow_path(waypoints, speed)`: store `self.path_waypoints = waypoints`, `self.path_leg_index = 0`; capture leg-start state the same way as the navigator.
- `update_autonomous()`: reimplement using the section-4 algorithm (currently `rotate = 0.0` is a dead stub - this actually fixes a real bug) and advance `path_leg_index` through `self.path_waypoints` instead of walking a spline's arc length.
- `publish_path_to_dashboard()`: publish the raw `self.path_waypoints` x/y/heading arrays directly instead of `self.path.sample_path(step_cm=5.0)` - a straight segment only needs its two endpoints to render correctly, no resampling needed.
- Update docstrings mentioning "Catmull-Rom".

### `robot.py`
- Remove `use_spline = SmartDashboard.getBoolean("navigation_use_spline", False)`.
- `self.navigator.set_waypoints(waypoints, use_spline=use_spline)` -> `self.navigator.set_waypoints(waypoints)`.
- Drop `spline={use_spline}` from the print statement.

### `dashboard/dashboard_server.py`
- Remove `use_spline = value.get("use_spline", False)` and `dashboard.table.putBoolean("navigation_use_spline", use_spline)`.
- Drop `spline={use_spline}` from the print/response strings.

### `dashboard/templates/dashboard_ws.html`
- `runWaypoints()`: remove the `useSpline` variable and the `use_spline` field sent in `sendCommand("navigate_waypoints", ...)`; update the status message text.
- `drawFieldMap()`: replace the Catmull-Rom -> cubic Bezier preview block (`chain` extension, control-point math, `bezierCurveTo`) with a plain polyline: `moveTo` first waypoint's canvas point, `lineTo` each subsequent one in order, add the closing segment back to point 0 when loop is checked.
- Delete `sampleSpline()` entirely (confirmed dead code, per decision 2).
- Update the stray comment "Python will use Catmull-Rom to calculate smooth path in real-time." to describe straight-line legs instead.

### Docs (existing files updated, no new docs created besides this plan)
- `docs/SWERVE_MODULES.md` - remove the `catmull_rom.py` section, describe the unified leg-based follower and `heading_math.py` instead; update the architecture diagram line.
- `docs/WAYPOINT_NAVIGATOR.md` - remove "Spline Following Mode" as a separate mode; describe the single leg-based algorithm with heading blending; remove `CatmullRomSpline` references.
- `docs/DASHBOARDS.md` - remove the `navigation_use_spline` SmartDashboard key and its mention in the troubleshooting checklist.
- `swerve/README.md` - remove the `CatmullRomSpline` usage/API section and the feature bullet; document `heading_math.py` instead.
- `README.md` (V22 root) - update the feature bullet mentioning Catmull-Rom.

## 6. Net deletions vs additions

- **Deleted**: `swerve/catmull_rom.py`, `WaypointNavigator._update_spline()` and all its supporting state, the `use_spline`/`navigation_use_spline` flag end-to-end (JS -> dashboard server -> SmartDashboard -> robot.py -> navigator), the JS Bezier preview + dead `sampleSpline()`.
- **Added**: `swerve/heading_math.py` (2 small pure functions), a handful of `leg_start_*` fields on `WaypointNavigator` and `SwerveDrive`, and the progress-based heading blend in the existing stage-1 loop.
- Net result is less code than today, one algorithm instead of three, and no mode flag to keep in sync across JS/server/robot/navigator.

## 7. Suggested implementation order

1. Add `swerve/heading_math.py`.
2. Update `waypoint_navigator.py`: remove spline mode/branch, add leg-start tracking + heading blend to the stage loop, drop `use_spline` param.
3. Update `robot.py` and `dashboard/dashboard_server.py` to stop reading/forwarding `use_spline`.
4. Update `dashboard_ws.html`: drop `use_spline` from the outgoing command, replace curve preview with straight polyline, delete `sampleSpline()`.
5. Update `swerve/swerve_drive.py`: drop `CatmullRomSpline`, reimplement `follow_path()`/`update_autonomous()` on the same leg algorithm, fix the dead rotation stub, simplify `publish_path_to_dashboard()`.
6. Update `swerve/__init__.py` exports.
7. Delete `swerve/catmull_rom.py`.
8. Update the four docs files listed above.
9. Manual test: single-waypoint "go here" command, multi-waypoint route with a dwell in the middle, and a looped route - confirm straight segments, confirm heading always ends correct at each stop, confirm no snap/jump when the robot starts a leg off-heading.
