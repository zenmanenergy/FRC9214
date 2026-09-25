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
without a vision/AprilTag system, which you've confirmed is not available yet
(and is explicitly deferred - see decisions below).

Rotation is different and has a neat trick available (see section 4): if you
command an *exact whole number of full rotations* (e.g. spin 5.000 x 360
degrees), the true physical heading change is known exactly (it's a multiple
of 360 deg) with no measuring equipment needed - you just need to see whether
the robot's "front" mark lines back up with a mark on the floor. That gives a
real ground truth for free.

## 3. Decisions from your answers

| # | Topic | Decision |
|---|-------|----------|
| 1 | Ground truth / space | Manual tape-measure/ruler readings only (no AprilTags yet - deferred to later). Usable test area is ~1-3 m per leg. |
| 2 | Rotation ground truth | Eyeballing the mark alignment is fine. |
| 3 | Odometry vs IMU scope | Calibrate both. They're measured from the *same* physical test motions (see section 4/5) so they end up calibrated together, even though the math for each is distinct. |
| 4 | Per-wheel granularity | Yes - each of the 4 wheels gets its own distance-scale correction, not one global number. |
| 5 | Accuracy target | 1-2%, but treat as a **tunable setting** in the dashboard (not hard-coded) since you're not fully sure yet - easy to loosen/tighten later without a code change. |
| 6 | Speed dependence | Must be measured, not assumed. Every trial records its speed level so we can later tell whether correction factors vary meaningfully with speed. |
| 7 | Workflow | Human-in-the-loop, one trial at a time: run -> human measures result and types it in -> human resets the robot to the start position/orientation -> system recalculates and *live-applies* the updated correction -> human presses Run again to immediately validate it -> repeat until under threshold. |

Also folding in your new point about heading during translation: if the robot
is told to drive straight and instead curves or ends up rotated, that is a
real dead-reckoning problem too, not just a distance-scale problem. The plan
below captures that as **perpendicular drift** measurements on every
translation trial, not just distance.

One more decision: this feature gets its **own new dashboard page**
(`odometry_calibration.html`), not added into `calibration.html`. The existing
wheel-alignment wizard stays completely untouched.

## 4. Why rotation gets calibrated first

You suggested calibrating rotation before translation - that's the right call,
for two reasons:

1. Translation trials need a trustworthy way to attribute "it didn't go where
   I told it to" to either (a) distance scale error or (b) it actually turned
   while driving. Having an already-calibrated IMU/odometry heading reading
   on the dashboard during translation trials makes that split much easier to
   read at a glance, on top of your own eyeball assessment of end-heading.
2. The per-wheel math for translation drift (section 6) assumes we already
   trust "how much did the robot actually rotate," so it should be solved
   first, independently, using the N-spin method - not re-derived from noisy
   translation data.

## 5. Rotation ("heading scale") calibration - the "N-spin" test

Setup: a small mark/tape flag on the chassis and a matching mark on the floor
indicating the robot's starting orientation.

Per trial:
1. Operator picks a whole number of rotations `N` (e.g. 3, 5, 8) and a
   rotation power/speed level.
2. Robot performs a closed-loop, in-place spin totalling `N x 360 deg`,
   tracked independently by (a) wheel/odometry-derived heading and (b) the
   IMU. Both totals are logged.
3. True total rotation is defined as `N x 360 deg` by construction. Operator
   visually checks the mark alignment at the end and types in the small
   residual angle (e.g. "-3 degrees" if it undershot) - a much easier number
   to eyeball than measuring a full rotation.
4. `true_total = N * 360 + residual`. Compute:
   - `imu_scale_factor = true_total / imu_reported_total`
   - `odometry_rotation_scale_factor = true_total / odometry_reported_total`
     (equivalent to correcting effective trackwidth/wheelbase)
5. Repeat at a few speeds/`N` values; average, and flag high variance
   (possible wheel slip - retest on higher-grip surface) rather than silently
   averaging bad data.

Both numbers come from the *same physical trial*, which is the "calibrated
together" behavior you asked for in decision 3 - one motion, two derived
scale factors (odometry's and the IMU's), stored side by side.

**Important safety note:** the existing `autotune_rotation.py` / rotation PID
autotune explicitly requires the robot to be elevated (wheels off the ground).
This calibration is the opposite - it must run with the robot on the ground,
driving normally, since we're measuring real-world motion. The wizard UI
should warn about this clearly so nobody confuses the two routines.

## 6. Translation calibration - a 7-level test suite, iterated per speed

You expanded this into a progressive suite of tests, from simple single-leg
moves up to compound/reversal moves that expose backlash - each with a
user-set distance `X` (meters, typed into the dashboard):

| Level | Test | Purpose |
|-------|------|---------|
| 1 | Forward `X` | Baseline forward scale/drift (single leg). |
| 2 | Right `X` | Baseline strafe scale/drift (single leg). |
| 3 | Diagonal 45 deg `X` | Baseline diagonal scale/drift (single leg), also cross-checks levels 1-2. |
| 4 | Forward `X` -> Backward `X` | Out-and-back on the forward/back axis - isolates **reversal backlash**, since a pure symmetric scale error cancels out on a round trip (section 2) but a direction-reversal loss does not. |
| 5 | Right `X` -> Left `X` | Same idea, on the strafe axis. |
| 6 | Diagonal 45 deg `X` -> Diagonal -45 deg `X` | Same idea, on the diagonal axis (see 6.1 for the assumption on what "-45" means here). |
| 7 | Forward `X` -> Right `X` (compound) | **Validation test**, not a fitting step - confirms the individually-calibrated corrections still add up correctly on a multi-leg path, since real autonomous paths are always multi-leg. |

Levels 1-3 are what actually produce the per-wheel scale-vs-speed data (section
6.3). Levels 4-6 produce a separate **backlash correction** (section 6.2).
Level 7 is run last, purely to confirm the calibrated model holds up - if it
doesn't, that's a sign of an interaction effect (e.g. heading drift between
legs) worth investigating separately rather than something to auto-correct.

### 6.1 Confirmed: level 6 is an out-and-back on the same diagonal line

Levels 4 and 5 are literal reversals (backward is the exact opposite direction
of forward; left is the exact opposite of right). A literal opposite of
"diagonal 45 degrees" would be 225 degrees, not -45 (which is actually a
different diagonal quadrant, roughly "right and back" if 45 is "right and
forward"). You confirmed the intent is out-and-back, consistent with levels
4/5, so level 6 is: drive the 45-degree diagonal `X` meters out, then drive
the 225-degree direction `X` meters back to start (labeled "-45" informally,
same as how you described it).

### 6.2 What gets measured per trial (tape measure, per your decision)

- **Along-axis distance actually traveled** (measured with the tape measure
  along the commanded direction of that leg).
- **Perpendicular drift** - how far off the straight commanded line the robot
  ended up, signed (left/right for forward/backward, forward/back for
  right/left, and perpendicular-to-the-diagonal for the diagonal tests).
- For levels 4-6 specifically: the **return leg's shortfall or overshoot**,
  computed after already accounting for that axis's known scale factor from
  levels 1-3 at the same speed - whatever distance is left unexplained by the
  scale factor alone is attributed to a **reversal backlash distance** (a
  fixed "distance lost when the drivetrain changes direction," similar in
  spirit to backlash compensation on a gearbox), stored per axis and checked
  for consistency across axes/wheels.
- For level 7: the final compound-path position/heading, compared against
  what the already-calibrated model predicts - this is a pass/fail sanity
  check, not a new number to solve for.

### 6.3 Verifying the diagonal math

You described the expected result for a 1 m diagonal-45 move starting at
(0,0) as landing at `(sqrt(2)/2, sqrt(2)/2)` - that's the correct vector
decomposition (`X * cos(45deg)`, `X * sin(45deg)`) in the same x/y convention
already used by `swerve_odometry.py` and the existing field map. The
dashboard's planned-path overlay (section 7) will draw exactly this point for
any `X`, so you can visually sanity-check it against the tape measurements
before trusting the drift numbers.

### 6.4 How per-wheel corrections get adjusted

Rather than one batch calculation, this is an iterative feedback loop - it
directly matches "run it over and over until the right values are reached":

- All 4 wheels are commanded to the same angle/speed during pure translation,
  so a *consistent* drift pattern is what reveals a per-wheel mismatch:
  - Forward-test drift (left/right) mainly indicates a left-side vs.
    right-side wheel mismatch -> nudges `front_left`/`rear_left` scale
    against `front_right`/`rear_right` scale.
  - Strafe-test drift (front/back) mainly indicates a front vs. rear wheel
    mismatch -> nudges `front_left`/`front_right` scale against
    `rear_left`/`rear_right` scale.
  - The diagonal test exercises all 4 wheels at a different angle and acts as
    a **cross-check** - if forward and strafe both converge but the diagonal
    test still drifts, that indicates a wheel-specific (not purely axis-based)
    error that the first two tests couldn't isolate on their own, and it gets
    nudged too.
- After each trial, the system computes a small proportional adjustment to
  the relevant wheel scale factor(s) (same idea as the relay/step-based
  adjustment `SwerveTuner` already does elsewhere in this codebase), applies
  it live, and the operator immediately re-runs that same test to confirm the
  adjustment helped.
- Because correcting one test can slightly perturb another (e.g. fixing
  left/right balance can slightly change the diagonal result), the operator
  cycles through **level 1 -> 2 -> 3 -> 1 -> ...** until all three atomic
  tests are simultaneously under the accuracy threshold, before moving on to
  levels 4-6 (backlash) and finally level 7 (validation).

### 6.5 Speed dependence -> a function, not just a number

Per your answer to question 3, add a **4th speed preset** so the fit can
actually distinguish a straight line from a curve (3 points can't confirm
curvature - a quadratic would fit any 3 points exactly with no way to check
it). Default speed presets: **25% / 50% / 75% / 100%**. Given the 1-3 m space,
100% will be heavily dominated by the accel/decel ramp (section 6.6) - worth
watching for noisy/inconsistent results at that level rather than distrusting
the whole curve fit if only the top point looks odd.

Levels 1-3 (the atomic tests) get run at all 4 speeds to build the per-wheel
`(speed, scale_factor)` data, fit with a curve - starting with a simple linear
regression (reusing the exact `_linear_regression` pattern
`encoder_calibration.py` already uses for PID-gain-vs-battery-voltage), and
only moving to a polynomial fit if the data clearly shows curvature. The
result is a small per-wheel function `scale_factor(speed) ->` correction,
looked up/interpolated at runtime for whatever speed is actually commanded -
mirroring the existing `get_interpolated_gains(battery_voltage)` pattern
exactly, just with speed as the input instead of voltage.

Levels 4-6 (backlash) are mechanically more likely to be a fixed distance
regardless of speed (it's slack in the gears, not a scaling effect). Since you
left this call to me: default to measuring backlash once at a moderate speed
(50%), and only expand to the full 4-speed matrix if that single measurement
turns out to be large enough to matter (comparable to or bigger than the
accuracy target) or looks inconsistent across the axes - keeps the common
case quick without silently ignoring a real speed-dependent backlash if one
shows up.

### 6.6 Space constraint note

With only 1-3 m of travel, the drive's existing acceleration/deceleration
ramp (`_apply_smooth_acceleration` in `swerve_drive.py`) will dominate a
meaningful chunk of each trial - worth keeping in mind when interpreting
results, especially at 75-100%.

## 7. Iterative human-in-the-loop workflow (with field-map visualization)

This matches the flow you described, applied to all three routines (rotation,
translation, and the level-7 validation pass), now with the visual field-map
feedback you asked for:

```mermaid
flowchart TD
    A[Operator sets level, X distance or N, speed] --> R[Field map shows a box\nin the lower-left = start pose]
    R --> P[Field map draws the planned/expected path\n(incl. diagonal math check) for operator to verify]
    P --> B[Press Run]
    B --> C[Robot executes the leg(s), then stops]
    C --> T[Field map draws the actual traveled path\nover the planned path]
    T --> D[Operator measures result with tape measure:\ndistance + perpendicular drift]
    D --> E[Operator types measured values into dashboard]
    E --> F[System nudges the relevant per-wheel scale factor(s)\nor backlash value and live-applies them]
    F --> G[Dashboard shows updated values + new predicted error]
    G --> H[Field map shows the start box again;\noperator physically resets robot to it]
    H --> I{This level AND earlier levels at\nthis speed under threshold?}
    I -- No --> A
    I -- Yes, more levels/speeds left --> A
    I -- Yes, all done --> K[Fit speed -> scale-factor curve per wheel;\nrun level 7 as final validation]
    K --> J[Operator reviews history, clicks Apply and Save]
```

Key behaviors:
- The field map always shows a fixed reference box in the lower-left corner
  representing "start here, facing this way" - the same spot/heading every
  time, so the operator can consistently reposition the robot by eye.
- Once a level's distance/direction is entered, the map draws the *planned*
  path (each leg, in order, for multi-leg levels) before the robot moves, so
  the operator can sanity-check it before pressing Run.
- After the robot stops, the map overlays the *actual* path (from odometry
  telemetry recorded during the move) next to the planned one, making drift
  visually obvious in addition to the tape-measure numbers.
- The candidate correction is **live-applied** to the next trial automatically
  (the whole point is to immediately verify the updated number helps), but
  nothing is written to the persisted calibration file until the operator
  explicitly clicks **Apply & Save**.
- **Discard/Reset** is available at any point to throw out the in-progress
  session and start over.
- Every trial (used or discarded) is kept in a visible history table so you
  can see the trend, not just the final number.

## 8. Where correction factors get applied

- `swerve/swerve_config.py` currently hard-codes `ROBOT_TRACKWIDTH_CM`,
  `ROBOT_WHEELBASE_CM`, and wheel circumference/gear-ratio constants used by
  `swerve_odometry.py`. Rather than editing that file automatically, the
  computed scale factors are stored as multipliers in the existing
  calibration JSON (same file `EncoderCalibration` already manages) and
  applied at runtime when odometry does its per-wheel distance/rotation math.
- `swerve/swerve_imu.py` gets a persisted `imu_scale_factor` (and keeps the
  existing `invert` flag) applied when converting raw yaw deltas to heading.
- Everything routes through the same `EncoderCalibration` load/save pattern
  already used for wheel offsets and PID gains, with new keys/sections, e.g.:
  - `rotation_calibration: { imu_scale_factor, odometry_rotation_scale_factor,
    accuracy_target_pct, trial_history: [...] }`
  - `translation_calibration: { wheel_scale_curve: {front_left: [(speed,
    factor), ...], front_right: [...], rear_left: [...], rear_right: [...]},
    reversal_backlash_cm: {forward_axis, strafe_axis, diagonal_axis},
    accuracy_target_pct, trial_history: [...] }`
  - Trial history entries record level (1-7), commanded distance/N, speed
    level, measured distance/residual, perpendicular drift, and the
    resulting scale factor/backlash adjustment(s) - enough to refit the
    speed -> scale-factor curve later without re-running trials.
  - Since you expect to redo this calibration multiple times over the
    season (wheel wear, swapped parts, etc.), trial history is **appended
    across sessions with a timestamp**, not overwritten - each full
    calibration run becomes a labeled entry, so you can later compare
    "January factors" vs. "March factors" and see how much the robot has
    drifted mechanically, the same way `pid_tuning_history` already
    accumulates across autotune runs.

## 9. Robot-side architecture

- New handler methods alongside the existing ones in
  `dashboard/calibration_mode_handler.py` (same poll-and-clear NT-flag
  pattern used by `_handle_autotune_commands`, etc.):
  - `_handle_rotation_calibration_command`
  - `_handle_translation_calibration_command`
  - Both mutually exclusive with the existing tuner/alignment/autotune
    routines (only one guided routine active at a time, same convention
    `SwerveTuner.is_active()` already follows).
- A small state machine per routine (modeled directly on `SwerveTuner`'s
  step/state-dict pattern): `idle -> running_trial -> awaiting_operator_input
  -> live_update_applied -> awaiting_reset_confirmation -> (next trial |
  converged/done)`. The translation state machine additionally tracks which
  of the 7 levels and which speed preset is active.
- Only runs in Test/Calibration mode, matching every other
  calibration/autotune feature in this codebase.

## 10. Dashboard UI - new, separate page

Per your request, this does **not** modify `calibration.html` (which stays
exactly as-is for wheel alignment). Instead:

- A new template file, e.g. `dashboard/templates/odometry_calibration.html`,
  is added alongside it, following the same conventions: same WebSocket
  connection/reconnect logic, `sendCommand`/`handleWebSocketMessage` plumbing,
  and overall visual style as `calibration.html` and `dashboard_ws.html`.
- `dashboard_server.py` gets one new route (parallel to the existing
  `/calibration` route) to serve this page, e.g. `/odometry_calibration`.
- `dashboard_ws.html` gets a single new link/button next to the existing
  "Calibration Wizard" link (same section) pointing to the new page - the
  only change to that file, no behavior changes to existing features.
- The new page has two sub-sections (Rotation, Translation), each with:
  - Rotation: `N` picker, speed picker, Run button, residual-angle input,
    live `imu_scale_factor` / `odometry_rotation_scale_factor` readout.
  - Translation: level selector (1-7, with a short description of each), `X`
    distance input, speed picker, Run button, inputs for measured distance
    and perpendicular drift, a per-wheel scale factor readout (4 numbers)
    plus the current backlash values, which speed level is currently being
    tuned, and a trial history table.
  - A **field map canvas** (modeled on the existing `#map` tab's
    `mapCanvas`/`drawFieldMap()` in `dashboard_ws.html`) that draws: the
    fixed lower-left start box, the planned path (all legs, in order) once a
    level is configured, and the actual traveled path after each run - see
    section 7.
  - An editable accuracy-target field (decision 5), a live per-trial error
    chart, and explicit **Apply & Save** / **Discard** buttons - nothing is
    persisted without operator confirmation.
  - Since this will be run repeatedly over the season, the page also shows a
    **past-sessions list** (timestamp + summary of what changed) so you can
    jump straight into a fresh run without re-reading this whole plan every
    time, and compare against the last time it was calibrated.
- New WS commands (`start_rotation_calibration`, `start_translation_calibration`,
  `submit_rotation_residual`, `submit_translation_result`, `confirm_reset`,
  `cancel_calibration`, `apply_calibration`, `discard_calibration`,
  `set_calibration_accuracy_target`) map to NT flags exactly the way existing
  commands like `autotune_rotation` already do in `dashboard_server.py`.

## 11. Testing

- Pure-math parts (scale factor adjustment step, backlash extraction, the
  speed -> scale-factor curve fit, convergence check) are unit-testable with
  no hardware, following the existing patterns in `swerve/unit_tests/` (e.g.
  `test_swerve_odometry.py`, `test_swerve_imu.py`, `test_swerve_tune.py`).
- Hardware-in-the-loop parts (the actual guided drive/spin routines) get
  smoke-tested on the real robot per the rollout order below.

## 12. Suggested rollout order

1. Persistence layer additions to `EncoderCalibration` (new keys, get/set/
   history methods) - no behavior change yet.
2. Rotation ("N-spin") calibration routine end-to-end (new page + handler) -
   calibrated first per section 4.
3. Apply the saved `imu_scale_factor` / `odometry_rotation_scale_factor`
   inside `swerve_imu.py` / `swerve_odometry.py`.
4. Translation calibration levels 1-3 end-to-end at a single speed first
   (field-map visualization + iterative per-wheel adjustment loop), to
   validate the mechanism before expanding scope.
5. Expand levels 1-3 across all 4 speed presets and add the speed ->
   scale-factor curve fit per wheel.
6. Add levels 4-6 (backlash detection) and level 7 (compound-path
   validation).
7. Apply the saved per-wheel scale curves and backlash values inside
   `swerve_odometry.py`.

## 13. Final decisions from this round

| # | Topic | Decision |
|---|-------|----------|
| 1 | Level 6 interpretation | Confirmed out-and-back along the same 45-degree line (see 6.1). |
| 2 | Backlash test speed coverage | Left to my judgment - default to measuring once at 50%, expand to all 4 speeds only if that reading is large/inconsistent (see 6.5). |
| 3 | Repeated use | You expect to redo this multiple times over the season - trial history is appended across sessions (not overwritten), and the dashboard keeps a past-sessions list so recalibrating later is quick (see sections 8 and 10). |

All open questions from earlier rounds are now resolved. This plan is ready
to move into implementation planning whenever you want to proceed.
