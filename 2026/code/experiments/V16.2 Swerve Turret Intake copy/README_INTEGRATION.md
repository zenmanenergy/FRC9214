# Team 9214 Swerve Drive with X,Y Navigation Dashboard

This is the integrated swerve drive robot code with web-based dashboard for autonomous X,Y positioning.

## System Architecture

```
┌─────────────────┐
│   RobotRIO      │
│   robot.py      │
│   +Navigation   │ ◄─── NetworkTables (Team 9214)
└─────────────────┘
         ▲
         │ Network
         │
         ▼
┌─────────────────────────────────┐
│  Laptop (Development)            │
│  robot_dashboard_server.py       │
│  ◄─ Flask WebSocket Server      │
└─────────────────────────────────┘
         ▲
         │ WebSocket (ws://)
         │
         ▼
┌─────────────────────────────────┐
│  Browser                         │
│  templates/robot_dashboard.html  │
│  + Field Map Visualization      │
│  + Real-time Robot State        │
│  + X,Y Navigation Control       │
└─────────────────────────────────┘
```

## Quick Start

### 1. Deploy Robot Code to RoboRIO

```bash
cd "2026\code\experiments\V16.2 Swerve Turret Intake copy"
./deploy.bat
```

The robot will start with NavigationTables server running and publishing state.

### 2. Run Dashboard Server (on development laptop)

```bash
cd "2026\code\experiments\V16.2 Swerve Turret Intake copy"
python robot_dashboard_server.py 9214
```

Or use the batch file:
```bash
start_dashboard.bat
```

### 3. Open Browser

Navigate to: **http://localhost:5000**

## How to Use the Dashboard

### Manual Joystick Control
- **Left Stick (Y)**: Forward/Backward
- **Left Stick (X)**: Strafe Left/Right  
- **Right Stick (X)**: Rotate Left/Right

### Autonomous Navigation (X,Y Positioning)

#### Method 1: Click on Field Map
1. Click anywhere on the blue field in the dashboard
2. Green circle appears showing target
3. Robot automatically navigates to that position

#### Method 2: Manual Coordinate Entry
1. Enter Target X (cm) in the input field
2. Enter Target Y (cm) in the input field
3. Click "Navigate" button
4. Robot drives to coordinates

#### Stop Navigation
- Click "Stop" button on dashboard
- Or take manual control with joystick (navigation pauses)

## Components

### `navigation.py`
Autonomous navigation module that converts X,Y targets into swerve drive commands.

**Key Methods:**
- `navigate_to(x, y)` - Start navigation to coordinates
- `update()` - Call every robot cycle to execute navigation
- `update_position(dx, dy, dheading)` - Update odometry (integrate with encoders)
- `stop()` - Stop navigation

**Parameters:**
- `MAX_SPEED = 0.8` - Maximum swerve power
- `ROTATION_SPEED = 0.5` - Maximum rotation power
- `POSITION_TOLERANCE = 5` cm - How close to target before done
- `HEADING_TOLERANCE = 5` degrees - How close to face target heading
- `TIMEOUT = 20` seconds - Max time to reach target

### `robot_dashboard_server.py`
Flask web server that bridges dashboard UI to robot NetworkTables.

**Endpoints:**
- `GET /` - Serves dashboard HTML
- `WS /ws` - WebSocket for real-time communication

**Commands:**
- `navigate_to` - Set navigation target from browser

### `templates/robot_dashboard.html`
Web UI with field visualization and controls.

**Sections:**
1. **Status Bar** - Connection status, robot mode, enabled state
2. **Field Map** - Visual representation of field with robot position
3. **Position & Navigation** - Current coordinates, heading, navigation controls
4. **Wheel States** - Real-time angle and power for each wheel

## NetworkTables Keys

The robot publishes these to SmartDashboard:

| Key | Type | Description |
|-----|------|-------------|
| `Robot X` | Number | Current X position (cm) |
| `Robot Y` | Number | Current Y position (cm) |
| `Robot Heading` | Number | Current heading (degrees, 0-360) |
| `Nav Target X` | Number | Target X position (cm) |
| `Nav Target Y` | Number | Target Y position (cm) |
| `Nav Active` | Boolean | Navigation currently running |
| `Nav Distance` | Number | Distance to target (cm) |
| `Nav Angle Error` | Number | Heading error to target (degrees) |
| `Nav Forward` | Number | Current forward power (-1 to 1) |
| `Nav Strafe` | Number | Current strafe power (-1 to 1) |
| `robot_mode` | String | "Teleop", "Test", or "Auto" |
| `Robot Enabled` | Boolean | Robot enabled state |

### Wheel States (published by DashboardUpdater)

| Key | Type | Description |
|-----|------|-------------|
| `{wheel}_angle` | Number | Wheel angle in degrees (0-360) |
| `{wheel}_power` | Number | Wheel drive power (-1 to 1) |

Where `{wheel}` is: `front_left`, `front_right`, `rear_left`, `rear_right`

## Integration with Existing Code

The navigation system integrates seamlessly:

1. **robot.py** - Reads nav commands from NT, calls navigator.update()
2. **SwerveDrive.drive_swerve()** - Already supports X,Y velocity commands
3. **Encoders/Gyro** - Update odometry via navigator.update_position()
4. **Dashboard** - Reads telemetry via DashboardUpdater

## Next Steps to Integrate Odometry

The navigation currently uses dead reckoning. To add real odometry:

### 1. Add Encoder Feedback
In `robot.py` teleopPeriodic():
```python
# Get wheel encoder deltas (in cm)
delta_x, delta_y = self.drive.get_position_delta()
delta_heading = self.drive.get_heading_delta()  # from gyro

# Update navigator's position estimate
self.navigator.update_position(delta_x, delta_y, delta_heading)
```

### 2. Add Gyro for Heading
Replace dead reckoning heading:
```python
gyro_heading = self.drive.get_gyro_heading()  # Get from navX/Pigeon gyro
self.navigator.heading = gyro_heading
```

### 3. Reset Position at Match Start
```python
# In autonomousInit()
self.navigator.set_position(x=0, y=0, heading=0)  # Your starting position
```

## Testing Checklist

- [ ] Robot code deploys without errors
- [ ] Dashboard server starts and connects to robot
- [ ] Browser shows correct robot position on field
- [ ] Joystick controls work in manual mode
- [ ] Clicking field map sends navigation command
- [ ] Robot moves toward clicked position
- [ ] Robot stops when reaching target (within tolerance)
- [ ] Wheel angles/powers display correctly
- [ ] Navigation can be cancelled with joystick

## Troubleshooting

### Dashboard won't connect to robot
1. Check robot is powered on and connected to network
2. Verify team number: `python robot_dashboard_server.py 9214`
3. Check firewall allows localhost:5000
4. Verify robot.py is running (RoboRIO SSH: `ps aux | grep python`)

### Robot doesn't respond to navigation commands
1. Check "Nav Active" shows as "Yes" in dashboard
2. Verify robot is in Teleop mode
3. Check robot is enabled in Driver Station
4. Check odometry values are updating
5. Check robot can move with joystick (basic movement works)

### Navigation target doesn't appear on map
1. Refresh browser (F5)
2. Check browser console for errors (F12)
3. Verify WebSocket connection shows as "Connected"

### Robot drifts from target
1. May indicate odometry calibration needed
2. Check wheel power values are uniform
3. Verify gyro is calibrated
4. Increase `POSITION_TOLERANCE` temporarily for testing

## Configuration

Edit these in `navigation.py` to adjust behavior:

```python
self.MAX_SPEED = 0.8  # 0-1, max forward/strafe power
self.ROTATION_SPEED = 0.5  # 0-1, max rotation power
self.POSITION_TOLERANCE = 5  # cm, distance to target
self.HEADING_TOLERANCE = 5  # degrees, rotation accuracy
self.TIMEOUT = 20  # seconds, max navigation time
```

## Advanced Usage

### Setting Custom Field Coordinates
Robot position uses field coordinate system:
- Origin (0,0) = bottom-left corner
- +X = right direction
- +Y = forward direction
- Field is 1097cm × 548cm

### Monitoring Navigation Progress
Watch these values in dashboard during navigation:
- **Nav Distance** - Should decrease over time
- **Nav Forward/Strafe** - Power being applied
- **Nav Angle Error** - Heading adjustment needed
- **Robot X/Y** - Position should move toward target

### Multiple Consecutive Targets
Current version supports single target. For waypoints:
1. Navigate to first target
2. When done, set new target
3. Repeat

Future: See `waypoint_navigator.py` for multi-point support.

## Files Overview

```
V16.2 Swerve Turret Intake copy/
├── robot.py                 # Main robot code (MODIFIED)
├── swerve_drive.py         # Swerve kinematics
├── navigation.py           # X,Y navigation module (NEW)
├── robot_dashboard_server.py # Web server (NEW)
├── templates/
│   └── robot_dashboard.html # Web UI (NEW)
├── start_dashboard.bat      # Quick start script (NEW)
├── deploy.bat              # Deploy to RoboRIO
├── pilot_controls.py       # Joystick input handling
├── pilot_joystick.py       # Joystick hardware interface
├── dashboard_updater.py    # Telemetry to NetworkTables
└── ... (other subsystems)
```

## Contact

Team 9214 - Honking Narwhals
