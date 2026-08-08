# Dashboard Systems

## Overview
The V20 robot uses multiple dashboard systems to monitor and control the swerve drive during operation. These include real-time motor feedback via SmartDashboard (NetworkTables) and a browser-based web interface for advanced features.

## Dashboard Architecture

### SmartDashboard (NetworkTables)
The primary real-time monitoring and tuning interface used by WPILib. All values are published to a centralized NetworkTables instance that the FRC driver station reads and displays.

**Connected to Robot at**: Team 9214 NetworkTables

### Teleop Dashboard (Web Interface)
A Flask-based web server that provides a real-time browser interface with WebSocket communication for low-latency updates.

**Hosted at**: `localhost:5000` (or robot IP when deployed)

**Features**:
- Real-time wheel angle visualization
- Motor power display
- Navigation waypoint creation and editing
- Calibration mode controls
- Autonomous routine selection

## SmartDashboard Integration

### DashboardUpdater (`dashboard_updater.py`)

**Purpose**: Publishes real-time telemetry every robot cycle

**Telemetry Published**:

#### Wheel Feedback
- `FR Angle`, `RR Angle`, `RL Angle`, `FL Angle` - Wheel steering angles (0-360°)
- `FR Power`, `RR Power`, `RL Power`, `FL Power` - Motor drive power (-1.0 to 1.0)

#### Odometry
- `Distance Centimeters` - Total distance traveled (cm)
- `Distance Meters` - Total distance traveled (m)
- `Odometry X` - Robot position X (cm)
- `Odometry Y` - Robot position Y (cm)
- `Odometry Heading` - Robot heading (0-360°)

#### IMU Diagnostics
- `IMU Connected` - Whether NavX-MXP is detected
- `IMU Calibrating` - Gyro calibration in progress
- `IMU Inverted` - Heading direction reversed
- `IMU Raw Yaw` - Raw gyro measurement
- `IMU Heading (0-360)` - Corrected heading (0-360°)

#### System Status
- `Robot Enabled` - FRC enable state
- `robot_mode` - Current mode ("Autonomous", "Teleop", "Disabled")
- `Counter` - Update cycle counter (diagnostics)

### Live Tuning Values

**Navigation Gains** (updated per cycle):
- `Nav_kP_drive` - Proportional gain for forward movement
- `Nav_kP_rotate` - Proportional gain for rotation
- `Nav_pos_tolerance` - How close to waypoint (cm)
- `Nav_rot_tolerance` - How close to heading (degrees)

**Navigation Commands** (written by dashboard, read by robot):
- `navigate_waypoints_command` - Start navigation (boolean)
- `navigation_waypoints_json` - Waypoint list (JSON string)
- `navigation_loop` - Loop at final waypoint (boolean)
- `navigation_use_spline` - Use smooth curve interpolation (boolean)
- `navigation_max_speed` - Speed limit 0.0-1.0 (float)
- `stop_navigation_command` - Stop navigation (boolean)

## Teleop Web Dashboard

### Server Architecture

**DashboardServer** (`dashboard_server.py`)
- Flask web server with WebSocket support
- Listens to NetworkTables for robot telemetry
- Broadcasts updates to all connected browsers via WebSocket
- Runs in background thread during match

**Initialization**:
```python
server = DashboardServer()
server.start_nt_listener()  # Starts background thread
server.run()  # Starts Flask web server on port 5000
```

### Real-Time Updates

**Update Frequency**: ~50Hz from robot
- Robot publishes telemetry to NetworkTables
- Dashboard server reads NetworkTables thread
- Updates streamed to web browser via WebSocket
- Minimal latency (~100ms typical)

**Data Sync**:
```json
{
  "type": "state",
  "angles": {"front_right": 45, "rear_right": 120, ...},
  "power": {"front_right": 0.5, "rear_right": -0.3, ...},
  "counter": 1234,
  "robot_enabled": true,
  "robot_mode": "Teleop"
}
```

### Web Interface Templates

**Location**: `dashboard/templates/` (Flask template directory)

**Pages Available**:
- Telemetry view - Real-time wheel angles and power
- Position tracking - Odometry X/Y/Heading
- Navigation control - Waypoint editor and route commands
- Calibration tools - Wheel alignment and PID tuning

## Calibration Mode Handler

### CalibrationModeHandler (`calibration_mode_handler.py`)

**Purpose**: Special test mode for tuning and calibration

**Modes**:
- Wheel alignment - Verify all wheels turn to known angles
- Motor direction testing - Confirm drive directions are correct
- PID tuning - Measure response to commands
- Encoder offset calibration - Account for mechanical variations

**Activation**: Triggered in test mode (when connected to driver station with Test mode selected)

**Access**: Dashboard provides UI to select calibration tests

## Dashboard Update Flow

```
Robot Hardware (wheels, IMU)
         ↓
SwerveDrive (processes state)
         ↓
DashboardUpdater (formats telemetry)
         ↓
NetworkTables (broadcasts over network)
    ↙          ↖
FRC Dashboard  Teleop Web Server
(driver station)   (Flask + WebSocket)
               ↓
          Web Browser (real-time viz)
```

## Key Dashboard Values for Debugging

### Identifying Motor Issues
1. Check `IMU Connected` - if false, gyro is disconnected
2. Check `FR Power`, `RR Power`, etc. - verify motors respond to commands
3. Check `FR Angle`, `RR Angle`, etc. - verify steering angle feedback

### Tracking Autonomous Movement
1. Monitor `Odometry X` and `Odometry Y` - should move smoothly toward waypoint
2. Watch `Odometry Heading` - should lock onto target rotation
3. Compare `IMU Heading (0-360)` with `Odometry Heading` - should track closely

### Verifying Navigation
1. Check `navigation_use_spline` and waypoint JSON format
2. Monitor `robot_mode` to confirm "Autonomous" when running route
3. Track `Odometry X/Y` against expected waypoint locations

## Configuration & Deployment

### Test Mode Calibration
- Driver station must be in **Test** mode
- `testInit()` called when entering test mode
- `testPeriodic()` runs calibration handler each cycle
- `testExit()` cleans up when exiting test mode

### Match Mode Telemetry
- SmartDashboard updates every robot cycle (~50Hz)
- Web dashboard only enabled if Flask/WebSocket available
- No impact on match performance

## SmartDashboard vs Web Dashboard

| Feature | SmartDashboard | Web Dashboard |
|---------|----------------|---------------|
| Lag | ~200ms | ~100ms |
| Network | WiFi (team network) | WiFi or Ethernet |
| Complexity | Simple | Rich visualization |
| Tuning | Real-time numeric inputs | Slider controls |
| Graphs | Yes (in driver station) | Can add custom |
| Mobile | No | Yes (browser responsive) |
| Setup | Built-in (WPILib) | Requires Flask installation |

## Future Enhancements

- Historical logging of odometry and motor currents
- Autonomy path playback and visualization
- AI-based tuning recommendations
- Camera integration (vision processing display)
- Performance profiling graphs
