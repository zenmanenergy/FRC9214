# Dashboard Guide

The dashboard is a web interface for controlling and testing the robot. It lets you:
- Drive the robot manually
- Test individual wheels
- Plan autonomous paths
- View real-time robot data
- Calibrate wheel offsets

## Accessing the Dashboard

1. **Make sure the robot is connected to your computer** (via USB or network)
2. **Run the dashboard server:**
   ```
   cd dashboard
   python dashboard_server.py
   ```
3. **Open your browser to:** `http://localhost:5000`

You should see the dashboard interface.

## Main Views

### Teleop (Manual Control)

This is where you drive the robot by hand using a joystick.

- **Forward/Backward** - Move the joystick forward/back
- **Left/Right** - Move the joystick left/right  
- **Rotate** - Use the rotation control to spin

The dashboard shows:
- Robot position and angle
- Current wheel speeds
- Joystick input values
- Battery voltage

### Autonomous Path Planner

This is where you tell the robot where to go on its own.

**How to create a path:**
1. Click on the field to place waypoints (click 2-4 spots)
2. The dashboard draws a smooth curve through your waypoints
3. Click "Send to Robot" to save the path
4. During autonomous, the robot follows that path

**What's happening behind the scenes:**
- The dashboard calculates a smooth curve (Catmull-Rom spline) between your waypoints
- The robot follows this curve in real-time
- It uses its gyro to stay oriented and wheel encoders to track distance

### Calibration Mode

Before the robot can drive accurately, we need to calibrate the wheel offsets. 

This tells the robot: "When I say point forward, these are the exact wheel angles."

**You probably won't use this unless:**
- The team asks you to re-calibrate wheels
- Wheels are installed in a different orientation

## Real-Time Data

The dashboard shows live data like:
- Robot's current position (X, Y, angle)
- Each wheel's speed and angle
- IMU (gyro) angle
- Battery voltage
- Network connection status

This helps you see what the robot is actually doing vs. what you told it to do.

## Troubleshooting

| Problem | Solution |
|---|---|
| Dashboard won't load | Make sure dashboard_server.py is running and check `http://localhost:5000` |
| Robot doesn't respond to joystick | Check that joystick is plugged in and configured in pilot_controls.py |
| Path doesn't follow correctly | Try re-calibrating wheels, or check if wheel speeds need tuning |
| Wheels point wrong direction | Wheel offsets may need calibration |

---

**Next:** Learn how the robot [plans and follows autonomous paths](Autonomous-Paths.md)
