# Ball Cluster Vision System - FRC 2026

A vision system that finds groups of 150mm yellow balls on the field and tells the robot where they are.

## How It Works

### Step 1: Look for Yellow
Two cameras point at the ground and look for yellow objects. Each camera scans its image and finds anything that looks yellow.

### Step 2: Match Between Cameras
The system compares what the left camera sees with what the right camera sees. When it finds the same ball in both cameras, it knows it found a match.

### Step 3: Figure Out Distance
By comparing where a ball appears in each camera, the system calculates how far away it is and which direction (left or right).

### Step 4: Group Nearby Balls
Instead of tracking individual balls, the system groups balls that are close together into clusters. This ignores scattered balls and focuses on the main pile of balls on the field.

### Step 5: Find the Biggest Group
The system identifies which cluster has the most balls and treats that as the target.

### Step 6: Tell the Robot
The location of the target cluster is sent to the RoboRio so the robot can drive toward it.

## Main Files

- **`ball_cluster_vision.py`** - The main system that does all the vision work
- **`test_vision_simulator.py`** - A test program that simulates the RoboRio so you can test without the actual robot
- **`yellow_ball_detector.py`** - A simpler single-camera version (doesn't need stereo)

## Quick Start

### Testing Without a Robot
```bash
python test_vision_simulator.py
```
This runs a test with simulated cameras and robot movement.

### Using Real Cameras
```python
from ball_cluster_vision import BallClusterVision

vision = BallClusterVision(server_ip="10.92.14.2")

while True:
    left_frame = left_camera.read()
    right_frame = right_camera.read()
    vision.update(left_frame, right_frame)
    # Target info is now available on NetworkTables
```

## What You Need

- 2 cameras set up as a stereo pair
- Python 3.10+
- Libraries: OpenCV, scikit-learn, pynetworktables

## What It Sends to the Robot

When the system finds a target:
- **Position** - Where the group of balls is (X and Y on the field)
- **Direction** - Which way to face to look at the balls
- **Confidence** - How many balls are in the group (more balls = more confident)

## Simple Ball Detection (Single Camera)

If you just need to detect yellow balls without stereo:
```bash
python yellow_ball_detector.py
```

Press 'c' to toggle clustering, 's' to save calibration images, 'q' to quit.

