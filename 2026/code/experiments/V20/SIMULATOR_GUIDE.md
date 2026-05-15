# Robot Simulator & Waypoint Navigator - Complete Guide

## Overview

This system lets you test your odometry, navigation, and dashboard WITHOUT a physical robot. It includes:

1. **RobotSimulator** - Simulates wheel physics and odometry
2. **WaypointNavigator** - Stage-based autonomous navigation (0-3 loop system)
3. **Dashboard Integration** - Live visualization of simulated robot
4. **Test Suite** - Validate everything works

---

## Quick Start (3 Steps)

### Step 1: Start the Simulator
```bash
cd c:\Users\default.LAPTOP-DHF9KHL4\Documents\GitHub\FRC9214\2026\code\experiments\V20
python robot_sim_server.py
```

You should see:
```
[SIM] Connecting to NetworkTables...
[SIM] Connected to SmartDashboard
[SIM] Starting simulation... (Press Ctrl+C to stop)
[SIM] Open dashboard at http://localhost:5000
```

### Step 2: Start the Dashboard (new terminal)
```bash
cd c:\Users\default.LAPTOP-DHF9KHL4\Documents\GitHub\FRC9214\2026\code\experiments\V20
python dashboard/dashboard_server.py
```

### Step 3: Open Dashboard
- Browser: http://localhost:5000
- Click **"Field Map"** tab
- You'll see a blue dot moving (the simulated robot)
- Odometry values update live!

---

## How to Use Click-to-Navigate

### Test with Simulator Running

1. **Open Field Map tab** in dashboard
2. **Click on the field** to set target (green circle appears)
3. **Simulator attempts to navigate there** - watch the blue robot move!

### What Happens (Stage System)
- **Stage 1:** Robot rotates to face target
- **Stage 2:** Robot moves forward
- **Stage 3:** Robot dwells at waypoint, then advances

Each stage has a timeout. If it fails, the navigator skips to next waypoint.

---

## Running Tests

### Test All Features
```bash
python test_simulator.py
```

This runs:
- **Test 1:** Simple forward movement
- **Test 2:** Multi-waypoint navigation (square pattern)
- **Test 3:** In-place rotation

Output shows position/heading at each step.

### Test Waypoints Programmatically
```python
from robot_simulator import RobotSimulator
from waypoint_navigator import WaypointNavigator

# Create robot and navigator
robot = RobotSimulator()
nav = WaypointNavigator(robot)

# Set waypoints
nav.set_waypoints([
    {"x": 100, "y": 0},    # 1 meter forward
    {"x": 100, "y": 100},  # 1 meter right
    {"x": 0, "y": 0}       # Back to origin
])

# Start navigation
nav.start()

# Simulation loop (50Hz = 20ms per frame)
while not nav.is_finished():
    robot.update()  # Physics step
    nav.update()    # Navigation step
    time.sleep(0.02)
```

---

## Understanding the Stage-Based Loop

The **0-3 loop system** works like this:

```
STAGE 0: Idle
  └─→ STAGE 1: Rotate to Target
      └─→ STAGE 2: Move Forward
          └─→ STAGE 3: Dwell/Complete
              └─→ Back to STAGE 1 (next waypoint)
```

### Stage Details

| Stage | What It Does | Timeout | Success Condition |
|-------|-------------|---------|------------------|
| 0 | Idle (not navigating) | - | - |
| 1 | Rotate to face target | 10s | `heading ±5°` |
| 2 | Drive forward to waypoint | 10s | `distance <10cm` |
| 3 | Dwell at waypoint | 0.5s | Wait, then advance |

### Tuning Navigation

Edit these in `WaypointNavigator.__init__()`:

```python
self.rotation_tolerance = 5.0      # degrees - tighter = more precise
self.position_tolerance = 10.0     # cm - tighter = more precise
self.max_rotation_speed = 0.5      # power 0-1 - faster rotation
self.max_move_speed = 0.8          # power 0-1 - faster movement
self.timeout_per_stage = 10.0      # seconds per stage
```

---

## Architecture

### RobotSimulator
- Simulates 4 swerve wheels
- Smooth angle rotation (180°/sec)
- Smooth power acceleration
- Basic odometry calculation
- Updates at 50Hz

### WaypointNavigator  
- Manages waypoint list
- Implements 3-stage loop
- Calculates distance/angle to target
- Handles timeouts and errors
- Publishes status (for dashboard)

### SimulatorServer (robot_sim_server.py)
- Connects to NetworkTables
- Runs simulator loop
- Updates dashboard values
- Broadcasts at 50Hz

### Dashboard Integration
- Receives odometry via NetworkTables
- Displays live robot position
- Shows navigation target (green circle)
- Sends click commands to simulator

---

## Simulation vs Real Robot

| Feature | Simulator | Real Robot |
|---------|-----------|-----------|
| Wheel Physics | Smooth, ideal | Physics, friction, lag |
| Odometry | Perfect (no error) | Encoder drift, IMU noise |
| Movement | Instant acceleration | Ramped, load-dependent |
| Positioning | No compliance | Compressible mechanisms |
| Communication | NetworkTables | FRC/RIO |

**The simulator is great for:**
- ✓ Testing navigation logic
- ✓ Validating dashboard
- ✓ Tuning PID gains
- ✓ Understanding behavior

**You still need real robot testing for:**
- ✗ Actual performance metrics
- ✗ Mechanical issues
- ✗ Sensor calibration
- ✗ Competition readiness

---

## Troubleshooting

### "No module named 'ntcore'"
```bash
pip install robotpy-ctre
# or
pip install ntcore
```

### Simulator won't connect to dashboard
- Make sure dashboard is running on http://localhost:5000
- Check NetworkTables is connecting: look for "[SIM] Connected" message
- Restart both servers

### Robot not moving in dashboard
- Check simulator is running (should see "[SIM] Cycle" messages)
- Check dashboard Field Map tab is open
- Verify odometry values are changing (not all zeros)

### Navigation fails
- Check timeouts - may be too short
- Reduce tolerance values (easier to reach)
- Check waypoints are reasonable (not off field)

---

## Next Steps

1. **Integrate into your robot code:**
   ```python
   from waypoint_navigator import WaypointNavigator
   
   nav = WaypointNavigator(self.swervedrive)
   nav.set_waypoints([...])
   nav.start()
   ```

2. **Add dashboard commands for navigation:**
   - Button to start autonomous
   - Button to clear waypoints
   - List of waypoints

3. **Test on real robot:**
   - Tune timeouts/tolerances
   - Validate odometry accuracy
   - Check motor performance

4. **Enhance simulator:**
   - Add friction/slipping
   - Add IMU drift simulation
   - Add sensor noise

---

## Files

| File | Purpose |
|------|---------|
| `robot_simulator.py` | Core simulator (RobotSimulator, SimulatedWheel, SimulatedOdometry) |
| `waypoint_navigator.py` | Navigation logic (WaypointNavigator with 0-3 loop) |
| `robot_sim_server.py` | Standalone simulator that feeds NetworkTables |
| `test_simulator.py` | Test suite with example usage |
| `SIMULATOR_GUIDE.md` | This file |

---

## Example: Full Autonomous Sequence

```python
from robot_simulator import RobotSimulator
from waypoint_navigator import WaypointNavigator
import ntcore
import time

# Setup
robot = RobotSimulator()
nav = WaypointNavigator(robot)
nt = ntcore.NetworkTableInstance.getDefault().getTable("SmartDashboard")

# Define mission
mission_waypoints = [
    {"x": 0, "y": 0},       # Start at origin
    {"x": 150, "y": 150},   # Corner 1
    {"x": 300, "y": 0},     # Corner 2
    {"x": 0, "y": 0}        # Return to start
]

nav.set_waypoints(mission_waypoints)
nav.start()

# Autonomous loop (50Hz)
start_time = time.time()
loop_count = 0

while not nav.is_finished() and (time.time() - start_time) < 120:
    robot.update()
    nav.update()
    
    # Publish to dashboard
    state = robot.get_state()
    nt.putNumber("Odometry X", state["odometry"]["x"])
    nt.putNumber("Odometry Y", state["odometry"]["y"])
    nt.putNumber("Odometry Heading", state["odometry"]["heading"])
    
    loop_count += 1
    time.sleep(0.02)

elapsed = time.time() - start_time
print(f"Mission complete in {elapsed:.1f}s ({loop_count} cycles)")
```

---

**Happy simulating!** 🤖
