# Add Continuous Autonomous Looping for Sim Harness

## Summary
Implement a new autonomy mode that continuously cycles through waypoint goals while robot mode is `autonomous`. Keep existing single-goal behavior unchanged by default in `navigation.launch.py`, and enable the loop mode by default only in `sim_harness.launch.py`.

## Goals
- Provide true autonomous continuous motion (no repeated stop/start from single fixed goal).
- Reuse existing Nav2 `NavigateToPose` action path.
- Preserve current safety behavior: cancel active goal when mode exits autonomous.
- Keep runtime/competition launch defaults unchanged.

## Scope
- In scope:
  - `team9214_ws/src/arena_bringup/arena_bringup/autonomy_mode_manager.py`
  - `team9214_ws/src/arena_bringup/launch/navigation.launch.py`
  - `team9214_ws/src/arena_bringup/launch/sim_harness.launch.py`
  - `team9214_ws/src/arena_bringup/README.md`
- Out of scope:
  - BT XML changes
  - New Nav2 action types (`NavigateThroughPoses`, waypoint follower action)
  - Runtime default behavior changes outside sim harness

## Interface Changes (Public/Launch Parameters)
Add new parameters to `autonomy_mode_manager`:
- `goal_mode` (`string`, default: `single_goal`)
- Allowed values: `single_goal`, `loop_waypoints`
- `waypoints` (`string`, default: `""`)
- Format: `x,y,yaw;x,y,yaw;...` in `goal_frame_id` frame
- `loop_on_success` (`bool`, default: `true`)
- `skip_failed_waypoint` (`bool`, default: `true`)

Keep existing parameters unchanged:
- `goal_x`, `goal_y`, `goal_yaw`, `goal_behavior_tree`, `cancel_on_mode_exit`, etc.

Add corresponding launch arguments in `navigation.launch.py` (defaults preserve old behavior):
- `autonomy_goal_mode:=single_goal`
- `autonomy_waypoints:=`
- `autonomy_loop_on_success:=true`
- `autonomy_skip_failed_waypoint:=true`

Enable sim defaults in `sim_harness.launch.py`:
- `autonomy_goal_mode:=loop_waypoints`
- `autonomy_waypoints:=1.0,1.0,0.0;3.0,1.0,0.0;3.0,2.0,1.57;1.0,2.0,3.14`
- `autonomy_loop_on_success:=true`
- `autonomy_skip_failed_waypoint:=true`

## Behavior Specification
- Mode transition into autonomous:
  - If `goal_mode=single_goal`: current behavior unchanged (send one goal).
  - If `goal_mode=loop_waypoints`: parse waypoint list, start at index `0`, send goal.
- Goal result handling in `loop_waypoints`:
  - `SUCCEEDED`: advance to next index (wrap around) and send next goal immediately.
  - `ABORTED` or `UNKNOWN`:
    - If `skip_failed_waypoint=true`: advance and continue loop.
    - Else: stop loop and wait for next autonomous transition.
  - `CANCELED`:
    - If cancellation caused by mode exit: stop loop.
    - Otherwise treat as failure using `skip_failed_waypoint` policy.
- Mode exit from autonomous:
  - If `cancel_on_mode_exit=true`, cancel active goal and stop loop state.
- Empty or invalid waypoint string:
  - Log error once on mode entry and do not send goals.

## Parsing/Validation Rules
- Waypoint token format: exactly 3 comma-separated floats.
- Reject malformed entries with explicit error logs including token index.
- Require at least 2 valid waypoints for `loop_waypoints`; otherwise refuse to start.
- Normalize yaw as-is (no transformation needed).

## Implementation Steps
1. Extend `autonomy_mode_manager.py` parameter declarations and internal state:
- Add mode enum/string handling.
- Add waypoint list storage and current index.
- Add parser helper: `parse_waypoints(str) -> list[(x,y,yaw)]`.
2. Refactor goal creation:
- Reuse one helper to build `NavigateToPose.Goal` from `(x,y,yaw)` + optional BT path.
3. Update result callback logic:
- Implement loop continuation policy from status + `skip_failed_waypoint`.
4. Update `navigation.launch.py`:
- Declare new launch args.
- Pass new args into `autonomy_mode_manager` node parameters.
- Keep defaults backward compatible (`single_goal`).
5. Update `sim_harness.launch.py`:
- Declare sim-specific defaults for loop mode and rectangle waypoints.
- Pass through to navigation launch args.
6. Update docs:
- Add “continuous autonomous loop” section with CLI examples and waypoint format.

## Test Plan
- Unit-level (Python):
  - Parse valid waypoint string.
  - Parse malformed waypoint string (non-float, missing fields).
  - Reject too-few waypoints for loop mode.
- Functional sim checks:
  - Launch sim harness defaults and confirm repeated goal acceptance/completion cycles.
  - Confirm robot keeps moving around rectangle without mode toggling.
  - Switch mode out of autonomous and verify immediate cancel + stop.
  - Re-enter autonomous and verify loop restarts from first waypoint.
  - Inject an unreachable waypoint and verify “skip failed and continue” behavior.
- Regression checks:
  - `goal_mode=single_goal` behaves exactly like current implementation.
  - Existing single-goal launch commands still work unchanged.

## Acceptance Criteria
- In sim harness default run, robot continuously cycles route while in autonomous.
- No manual re-publish of `/robot_mode` required to continue motion.
- Exiting autonomous stops/cancels navigation cleanly.
- Existing single-goal behavior remains intact in navigation launch defaults.

## Assumptions and Defaults
- Sim-only default enablement was chosen.
- Initial path shape is compact center rectangle:
  - `(1,1,0) -> (3,1,0) -> (3,2,1.57) -> (1,2,3.14)`.
- Failure policy is “skip failed waypoint and continue”.
- Loop stops only on mode exit (or invalid configuration).

## Implementation Notes

Patched continuous waypoint-loop autonomy mode to autonomy_mode_manager:

- New params: goal_mode, waypoints, loop_on_success, skip_failed_waypoint
- Added waypoint parsing/validation (x,y,yaw;x,y,yaw;...)
- Added loop behavior on result status (SUCCEEDED advances, failures can skip)
- Preserved existing single_goal behavior as default
- Files: autonomy_mode_manager.py (line 36), autonomy_mode_manager.py (line 97), autonomy_mode_manager.py (line 182), autonomy_mode_manager.py (line 227)
- Wired new launch arguments in navigation launch (backward-compatible defaults):

- Added args:
  autonomy_goal_mode:=single_goal
  autonomy_waypoints:=
  autonomy_loop_on_success:=True
  autonomy_skip_failed_waypoint:=True
- Passed through to autonomy node params
- File: navigation.launch.py (line 70), navigation.launch.py (line 274), navigation.launch.py (line 586), navigation.launch.py (line 675)
- Enabled loop mode by default only in sim harness:
  autonomy_goal_mode:=loop_waypoints
  autonomy_waypoints:=1.0,1.0,0.0;3.0,1.0,0.0;3.0,2.0,1.57;1.0,2.0,3.14
  autonomy_loop_on_success:=true
  autonomy_skip_failed_waypoint:=true
- Passed through to included navigation.launch.py
- File: sim_harness.launch.py (line 52), sim_harness.launch.py (line 93), sim_harness.launch.py (line 149)
- Updated docs with continuous loop section + new args:
  - File: README.md (line 49), README.md (line 106)
- Validation run
  - autonomy_mode_manager.py
  - colcon build --packages-select arena_bringup (success)

- Notes
Existing single-goal commands remain valid because navigation.launch.py default is still autonomy_goal_mode:=single_goal.