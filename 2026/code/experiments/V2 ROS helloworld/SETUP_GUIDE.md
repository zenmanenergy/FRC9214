# NetworkTables Bridge Setup Guide

## Overview
This setup allows your RoboRIO (FRC robot) to communicate with an Ubuntu computer running ROS2 using NetworkTables as the data bridge.

**Architecture:**
- **Ubuntu Computer (10.92.14.100)**: Runs as NetworkTables **SERVER** and publishes "hello world" via ROS2
- **RoboRIO**: Runs as NetworkTables **CLIENT** and subscribes to messages from Ubuntu

---

## Installation on Ubuntu Computer

### 1. Install Required Python Package
```bash
pip install pynetworktables
```

### 2. Run the ROS2 Publisher Node
From the ROS2 environment on Ubuntu, run:
```bash
cd /path/to/ros_hello_publisher.py
python3 ros_hello_publisher.py
```

Or if you have a ROS2 workspace, integrate the script into a ROS2 package.

**Expected Output:**
```
[INFO] [rclpy]: RCL initialized
Hello Publisher initialized
NetworkTables server started at 0.0.0.0:5800
Published: hello world
Published: hello world
...
```

---

## Installation on RoboRIO

### 1. Install NetworkTables Python Library
Deploy to RoboRIO using pip:
```bash
python -m pip install pynetworktables --target=/opt/frc/lib/site-packages/
```

Or if using WPILib's Python environment:
```bash
python -m pip install pynetworktables
```

### 2. Deploy robot.py to RoboRIO
Use the standard FRC deployment process:
- Copy robot.py and all dependencies to the RoboRIO
- The modified `robot.py` already includes NetworkTables client initialization

---

## How It Works

### On Ubuntu (Server):
1. `ros_hello_publisher.py` initializes as a NetworkTables **server** on port 5800
2. Every 1 second, it writes "hello world" to the `ros_data/message` entry
3. The message can be retrieved by any NetworkTables client

### On RoboRIO (Client):
1. `robot.py` initializes NetworkTables as a **client** connecting to 10.92.14.100:5800
2. In `teleopPeriodic()`, it reads the `ros_data/message` entry
3. If a message is received, it prints: `[RoboRIO] Received from ROS2: hello world`

---

## Troubleshooting

### Connection Issues
- **Verify network connectivity**: Ping 10.92.14.100 from RoboRIO
  ```bash
  ping 10.92.14.100
  ```
- **Check firewall**: Ensure port 5800 is open on Ubuntu
  ```bash
  sudo ufw allow 5800
  ```

### No Messages Received
1. Verify Ubuntu node is running (check for "Published:" logs)
2. Check RoboRIO console output during teleopPeriodic
3. Verify correct IP address: 10.92.14.100
4. Ensure both are on the same network

### Debug: Monitor NetworkTables
On any computer, use NT Viewer to monitor entries:
```bash
python -m networkTables.viewer
```

---

## Extending the Bridge

### Add More Data from ROS2 → RoboRIO
In `ros_hello_publisher.py`, add more entries:
```python
self.table.putString("status", "running")
self.table.putNumber("timestamp", time.time())
```

In `robot.py`, retrieve them:
```python
status = self.ros_table.getString("status", "")
timestamp = self.ros_table.getNumber("timestamp", 0.0)
```

### Send Data from RoboRIO → ROS2
In `robot.py`:
```python
self.ros_table.putString("robot_state", "enabled")
```

In `ros_hello_publisher.py`, read it:
```python
state = self.table.getString("robot_state", "unknown")
self.get_logger().info(f"Robot state: {state}")
```

---

## Key Configuration Parameters
- **Server IP**: 0.0.0.0 (Ubuntu listens on all interfaces)
- **Server Port**: 5800
- **Client IP**: 10.92.14.100 (Ubuntu computer IP)
- **Table Name**: `ros_data`
- **Message Key**: `message`
- **Update Rate**: 1 Hz (1 second interval on Ubuntu)

---

## Files Modified/Created
- `robot.py` - Updated with NetworkTables client initialization
- `ros_hello_publisher.py` - New ROS2 node that publishes to NetworkTables
