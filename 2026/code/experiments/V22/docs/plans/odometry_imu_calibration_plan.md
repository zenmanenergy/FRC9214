# Odometry + IMU Dead-Reckoning Calibration - Design Plan

## 1. Goal

Improve dead-reckoning accuracy (odometry position + IMU heading) across a range
of speeds, so that commanding the robot to "go to X, Y, heading" lands as close
to the true target as possible. Add a guided calibration routine, driven from
the dashboard, that measures the error between "what the robot thinks happened"
and "what actually happened," computes correction factors, and persists them so
they are applied every match/session.

## 2. The core problem with the original idea

The original idea (drive forward/back repeatedly, compare "where it was
supposed to go" vs "where odometry says it went") has one issue: **odometry is
computed entirely from the same wheel encoders / IMU we're trying to
calibrate.** There is no independent ground truth in that loop, so:

- If the wheel-to-distance scale factor is wrong by some amount, "commanded
  distance" and "odometry-reported distance" will always agree with each
  other (both are derived from the same encoder counts) - comparing them
  tells us nothing about real-world scale error.
- An "out-and-back" test (drive forward D, then back D) will always return the
  robot very close to its literal start point even if the scale is wrong,
  because the same wrong scale factor is applied symmetrically in both
  directions. It can reveal drift/backlash/heading-related asymmetry, but not
  a pure scale error.

**Conclusion: calibrating translation scale requires a real-world, external
reference (a physically measured distance).** There isn't a way around this
without adding a vision/AprilTag system or other absolute sensor.

Rotation is different and has a neat trick available (see 3.2): if you command
an *exact whole number of full rotations* (e.g. spin 5.000 x 360 degrees), the
true physical heading change is known exactly (it's a multiple of 360 deg) with
no measuring equipment needed - you just need to see whether the robot's
"front" mark lines back up with a mark on the floor. That gives a real ground
truth for free, which is why the plan below treats rotation and translation
calibration a differently.

## 3. Proposed calibration routines

### 3.1 Translation ("distance scale") calibration - needs a measured track

Setup: a straight line on the floor as a start line, and one or more marks at
known distances (e.g. 2 m, 4 m), measured once with a tape measure.

Per trial:
1. Dashboard operator picks a target distance and a power/speed level
   (e.g. 25/50/75/100%) and direction (forward/backward).
2. Robot drives that commanded distance (existing drive/power code, no new
   drive logic needed) and reports the odometry-computed distance traveled.
3. Operator reads where the robot actually stopped relative to the marked
   line and types the actual distance into the dashboard.
4. System computes `scale_factor = actual_distance / odometry_distance` for
   that trial and appends it to a trial history (speed, direction, commanded,
   odometry-reported, actual, computed factor, timestamp).
5. Repeat across multiple speeds/distances/directions. Running average (with
   outlier rejection) becomes the candidate correction. A short validation
   pass (re-run 2-3 trials with the candidate factor applied) confirms it
   converged before saving.

This directly calibrates effective wheel diameter / drivetrain scale - the
main source of "drives too far / not far enough" dead-reckoning error.

### 3.2 Rotation ("heading scale") calibration - the "N-spin" test

Setup: put a small mark/tape flag on the chassis and a matching mark on the
floor indicating the robot's starting orientation.

Per trial:
1. Operator picks a whole number of rotations `N` (e.g. 3, 5, 8) and a
   rotation power/speed level.
2. Robot performs a closed-loop, in-place spin totalling `N x 360 deg`,
   tracked independently by (a) wheel/odometry-derived heading and (b) the
   IMU. Both totals are logged.
3. True total rotation is defined as `N x 360 deg` by construction. Operator
   visually checks the mark alignment at the end; if it's not perfectly
   aligned, they enter the small residual angle (e.g. "-3 degrees" if it
   undershot) - this is a much smaller, easier-to-eyeball number than
   measuring a full rotation.
4. `true_total = N * 360 + residual`. Compute:
   - `imu_scale_factor = true_total / imu_reported_total`
   - `odometry_rotation_scale_factor = true_total / odometry_reported_total`
     (equivalent to correcting effective trackwidth/wheelbase)
5. Repeat at a few speeds/`N` values; average, flag high variance (possible
   wheel slip - test on a higher-grip surface) rather than silently
   averaging bad data.

**Important safety note:** the existing `autotune_rotation.py` / rotation PID
autotune explicitly requires the robot to be elevated (wheels off the ground).
This calibration is the opposite - it must run with the robot on the ground,
driving normally, since we're measuring real-world motion. The wizard UI
should warn about this clearly so nobody confuses the two routines.

### 3.3 Convergence / "good enough" loop

Both routines are framed as: run trial -> compute error -> update running
correction -> re-validate -> stop once error stays under a target threshold
for a few consecutive trials (or the operator manually accepts the result).
This satisfies the "run multiple times and improve until accurate enough"
part of the request, without needing an open-ended automatic loop that drives
the robot around unsupervised.

## 4. Where correction factors get applied

- `swerve/swerve_config.py` currently hard-codes `ROBOT_TRACKWIDTH_CM`,
  `ROBOT_WHEELBASE_CM`, and wheel circumference/gear-ratio constants used by
  `swerve_odometry.py`. Rather than editing that file automatically, the
  computed scale factors would be stored as multipliers in the existing
  calibration JSON (same file `EncoderCalibration` already manages) and
  applied at runtime when odometry does its distance/rotation math.
- `swerve/swerve_imu.py` gets a persisted `imu_scale_factor` (and keeps the
  existing `invert` flag) applied when converting raw yaw deltas to heading.
- Everything routes through the same `EncoderCalibration` load/save pattern
  already used for wheel offsets and PID gains, just with new keys/sections,
  e.g. `odometry_calibration: { translation_scale_factor, rotation_scale_factor,
  imu_scale_factor, trial_history: [...], last_calibrated }`.

## 5. Robot-side architecture

- New handler methods alongside the existing ones in
  `dashboard/calibration_mode_handler.py` (same poll-and-clear NT-flag
  pattern used by `_handle_autotune_commands`, etc.):
  - `_handle_translation_calibration_command`
  - `_handle_rotation_calibration_command`
  - Both mutually exclusive with the existing tuner/alignment/autotune
    routines (only one guided routine active at a time, same convention
    `SwerveTuner.is_active()` already follows).
- A small state machine per routine (modeled directly on `SwerveTuner`'s
  step/state-dict pattern): `idle -> running_trial -> awaiting_operator_input
  -> trial_recorded -> (next trial | converged/done)`.
- Only runs in Test/Calibration mode, matching every other
  calibration/autotune feature in this codebase.

## 6. Dashboard UI

- New wizard screen modeled on `dashboard/templates/calibration.html`'s
  existing step-array pattern (`STEPS[]`, step counter, instruction box,
  Confirm/Exit), reusing the same WebSocket command/telemetry plumbing
  (`sendCommand`, `handleWebSocketMessage`) already used everywhere else in
  the dashboard - no new communication mechanism needed.
- Two guided flows (Translation, Rotation), each showing:
  - Current step / instructions (e.g. "Align robot to start line", "Enter
    actual distance measured").
  - A running trial history table (speed, commanded, odometry value, actual
    value entered, computed error %).
  - A simple live chart (reuse the existing canvas-drawing approach) plotting
    error % per trial, so the operator can see it converging.
  - Current best correction factor, with explicit **Apply & Save** /
    **Discard** buttons - nothing is persisted to the robot automatically
    without operator confirmation.
- New WS commands (`start_translation_calibration`, `start_rotation_calibration`,
  `submit_actual_distance`, `submit_rotation_residual`, `cancel_calibration`,
  `apply_calibration`, `discard_calibration`) map to NT flags exactly the way
  existing commands like `autotune_rotation` already do in
  `dashboard_server.py`.

## 7. Testing

- Pure-math parts (scale factor computation, running average/outlier
  rejection, convergence check) are unit-testable with no hardware, following
  the existing patterns in `swerve/unit_tests/` (e.g.
  `test_swerve_odometry.py`, `test_swerve_imu.py`, `test_swerve_tune.py`).
- Hardware-in-the-loop parts (the actual guided drive/spin routines) get
  smoke-tested on the real robot per the rollout order below.

## 8. Suggested rollout order

1. Persistence layer additions to `EncoderCalibration` (new keys, get/set/
   history methods) - no behavior change yet.
2. Translation calibration routine end-to-end (handler + dashboard wizard),
   since distance error is usually the bigger contributor to dead-reckoning
   drift and it's simpler (no "N-spin" bookkeeping).
3. Apply the saved translation scale factor inside `swerve_odometry.py`.
4. Rotation ("N-spin") calibration routine end-to-end.
5. Apply the saved IMU/rotation scale factors inside `swerve_imu.py` /
   `swerve_odometry.py`.
6. Optional follow-up: if trials show scale error varies meaningfully with
   speed, consider a speed-dependent model (same regression-vs-history idea
   `EncoderCalibration` already uses for PID gains vs. battery voltage). Avoid
   building this upfront unless the simple single-factor approach proves
   insufficient.

## 9. Open questions for you

1. **Ground truth for distance** - how do you want to measure "actual
   distance traveled" in practice? A taped/marked straight track with known
   distances (simplest, no new hardware), or do you have (or plan to add) a
   vision/AprilTag system that could give this automatically? If it's manual
   measurement, is there a clear straight run of at least ~4-5 m available at
   your practice space?
2. **Ground truth for rotation** - are you OK with the "mark the chassis +
   mark the floor, spin a whole number of rotations, eyeball/measure the
   small residual" method, or would you prefer something like a printed
   protractor/turntable mat for a more precise readout?
3. **Scope** - do you want to calibrate wheel/track-based odometry, the IMU,
   or both (this plan assumes both, since you mentioned both)? Is the
   complementary filter blend (`IMU_WEIGHT = 0.95` in `swerve_imu.py`)
   something you also want tuned as part of this, or strictly out of scope?
4. **Per-wheel vs. global scale** - is a single global translation scale
   factor sufficient, or do you suspect individual wheels differ enough
   (uneven wear, mismatched tread) to need independent per-wheel scale
   corrections? This adds complexity to both the routine and the data model.
5. **Accuracy target** - what error would you consider "accurate enough" to
   stop iterating (e.g. within 1-2% of commanded distance, within 1-2 degrees
   over several rotations)? This becomes the convergence/stop criterion.
6. **Speed dependence** - do you want the calibration to explicitly test
   whether the scale error changes with speed (more trials, more setup work),
   or is a single "good enough at all speeds" factor acceptable for now?
7. **Operator workflow** - is it acceptable that this requires a person on
   the field entering measurements by hand each trial (like the existing
   wheel-alignment wizard requires), or were you hoping for something that
   runs unattended? (Fully unattended isn't possible without an external
   ground-truth sensor, per section 2.)
8. **Timeline/priority** - should I implement translation calibration first
   as a smaller, standalone change, or do you want both translation and
   rotation calibration delivered together?
