# Swerve Unit Tests

Unit tests for every module in `swerve/`, covering the pure math (heading
wrapping, PID, odometry kinematics), config validation, and the hardware
wrapper classes (`SwerveWheel`, `SwerveDrive`, `SwerveIMU`, `SwerveTuner`,
`EncoderCalibration`).

## Requirements

The tests run through robotpy's simulated HAL, so no real robot or RoboRIO
is needed, but the robotpy packages must be installed:

```
pip install -r requirements.txt
```

This installs `pytest`, `robotpy` (wpilib), `robotpy-rev`, and
`robotpy-navx`, which are all required by the test suite.

## Running the tests

From the project root (`V22/`):

```
python -m pytest swerve/unit_tests
```

Run with more detail (shows each test name and pass/fail):

```
python -m pytest swerve/unit_tests -v
```

Run a single test file:

```
python -m pytest swerve/unit_tests/test_heading_math.py
```

Run a single test class or test function:

```
python -m pytest swerve/unit_tests/test_pid_controller.py::TestCalculate
python -m pytest swerve/unit_tests/test_pid_controller.py::TestCalculate::test_pure_proportional
```

Stop at the first failure and show full tracebacks:

```
python -m pytest swerve/unit_tests -x --tb=long
```

Run with a coverage report (requires `pytest-cov`, already listed as a
plugin if installed):

```
python -m pytest swerve/unit_tests --cov=swerve
```

## Test files

- `test_heading_math.py` - `shortest_angle_diff`, `lerp_angle`
- `test_pid_controller.py` - `PIDController.calculate`, `reset`, `set_gains`, `autotune`
- `test_swerve_config.py` - validates the `WHEELS` dict and control constants
- `test_swerve_wheel.py` - `SwerveWheel` angle/offset math, drive/turn power, encoder readouts
- `test_swerve_odometry.py` - `SwerveOdometry` pose getters/setters and wheel-kinematics `update()`
- `test_swerve_imu.py` - `SwerveIMU` heading conversion, status checks, `fuse_heading`
- `test_encoder_calibration.py` - `EncoderCalibration` persistence, offsets, gain regression/interpolation
- `test_swerve_tune.py` - `SwerveTuner` autotune state machine
- `test_swerve_drive.py` - `SwerveDrive` movement state, path following/recording, motor current monitoring
- `fakes.py` - shared lightweight test doubles (`FakeWheel`, `FakeMotor`, `FakeAHRS`, `FakeCalibration`) used instead of real CAN/DIO hardware so tests are fast and deterministic

## Notes

- `SwerveWheel` and `SwerveDrive` tests create their real hardware objects
  once per test file (module-scoped fixtures) because simulated DIO/CAN
  resources cannot be re-allocated once claimed within the same process.
  Motor objects are then swapped out with `FakeMotor` per test so behavior
  doesn't depend on the installed rev SDK version.
- `SwerveIMU` tests replace the real `navx.AHRS` object with `FakeAHRS` so
  heading/calibration behavior is deterministic instead of depending on the
  navX simulator's real calibration timing.
- `EncoderCalibration` tests monkeypatch `OFFSET_FILE` to a pytest `tmp_path`
  so no real calibration file on disk is touched.
