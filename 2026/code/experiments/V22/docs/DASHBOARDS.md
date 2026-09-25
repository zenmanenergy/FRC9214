# Dashboard Systems

## What it is

The robot has one dashboard: a web page you open in a browser (`localhost:5000`, or the robot's IP on match day) that lets you watch and control the swerve drive without needing the FRC driver station software for anything beyond enabling the robot.

Under the hood, the robot and the dashboard don't talk to each other directly. The robot continuously publishes its status (wheel angles, position, gyro heading, etc.) to NetworkTables, which is the standard FRC network layer for sharing values between the robot and anything else on the network. The dashboard just reads those values out of NetworkTables and draws them on screen, and when you click a button in the browser, it writes a value back into NetworkTables for the robot to pick up on its next cycle. So it's a two-way conversation, just relayed through NetworkTables instead of a direct connection.

## What you can do with it

The dashboard has three pages:

- **Main dashboard** - the everyday view. Shows each wheel's current angle and power live, shows the robot's tracked position/heading, lets you build and run autonomous waypoint routes, and lets you record/export a driven path so it can be replayed or reviewed later.
- **Wheel calibration wizard** - a guided, one-wheel-at-a-time flow for zeroing out each swerve wheel so "0 degrees" actually points the same direction on every wheel.
- **Odometry + IMU calibration** - two guided routines (one for rotation, one for driving straight) that correct small drift/scale errors in how the robot estimates its own position and heading, based on measurements you feed back in during the test.

There are also a handful of test-mode-only tools (PID autotuning for the wheels and for rotation) that are only usable when the robot is in Test mode, as a safety measure.

## How the pieces fit together

```
Robot (wheels, gyro, position tracking)
        |
   publishes status / reads commands
        |
   NetworkTables  <-- the shared "bulletin board" both sides read/write
        |
   Dashboard server (reads status, relays button clicks back)
        |
   Browser (what you actually look at and click)
```

Because the robot and dashboard are only connected through NetworkTables, the dashboard can run on any laptop on the same network - it doesn't need to run on the robot itself, and a slow or disconnected dashboard can't crash or block the robot.

## Where things live

- `dashboard/dashboard_server.py` - runs the web server and keeps NetworkTables and the browser in sync
- `dashboard/calibration_mode_handler.py` - runs on the robot side during Test mode and carries out whatever the calibration pages ask for
- `dashboard/templates/` - the actual web pages (main dashboard, wheel wizard, odometry/IMU calibration)
- `dashboard/recorded_paths/` - saved path recordings exported from the main dashboard
- `dashboard/run.bat` - installs what's needed and starts the dashboard server
