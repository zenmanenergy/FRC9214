# Ball Cluster Vision System - FRC 2026

A comprehensive stereo vision system for detecting, clustering, and tracking 150mm yellow balls on the field for FIRST Robotics Challenge 2026.

## Overview

This vision system processes stereo camera feeds to:
- **Detect** yellow balls using HSV color filtering
- **Stereo Match** balls between left and right cameras
- **Triangulate** 3D positions from stereo disparity
- **Cluster** nearby balls using DBSCAN algorithm
- **Track** the largest ball cluster with temporal smoothing
- **Publish** target position and confidence to NetworkTables

## Files

### Core Vision System
- **`ball_cluster_vision.py`** - Main vision processing class
  - Ball detection pipeline (HSV filtering, morphological ops, contour analysis)
  - Stereo matching algorithm (left/right correlation)
  - 3D point triangulation from disparity
  - DBSCAN clustering for grouping nearby balls
  - Cluster locking for temporal stability
  - NetworkTables integration for RoboRio communication
  - Debug visualization methods

### Testing & Simulation
- **`test_vision_simulator.py`** - Test harness with RoboRio simulation
  - Mock NetworkTables (no RoboRio hardware needed)
  - Synthetic test frame generation with realistic yellow balls
  - Robot motion simulator for pose updates
  - Full pipeline testing and performance profiling
  - Debug output and statistics tracking

### Simple Detection (Alternative)
- **`yellow_ball_detector.py`** - Standalone single-camera ball detector
  - Single camera ball detection (no stereo)
  - Ball clustering by proximity
  - Interactive color calibration tool
  - Real-time visualization

## System Architecture

```
Left Camera Frame  →  Ball Detection  ┐
                                       ├→ Stereo Matching → 3D Points → DBSCAN Clustering → Cluster Lock
Right Camera Frame → Ball Detection  ┘
                                                                              ↓
                                                        Field Frame Conversion + Temporal Smoothing
                                                                              ↓
                                                              NetworkTables (RoboRio)
```

## Installation

### Prerequisites
- Python 3.10+
- Virtual environment (recommended)

### Dependencies
```bash
pip install opencv-python scikit-learn pynetworktables numpy
```

Or install from requirements:
```bash
pip install -r requirements.txt
```

## Configuration

### Camera Parameters (in `ball_cluster_vision.py`)
```python
self.image_width = 640              # Camera image width (pixels)
self.cx = 320.0                     # Principal point x (image center)
self.fx = 700.0                     # Focal length (pixels) - CALIBRATE FOR YOUR CAMERAS
self.baseline = 0.60                # Stereo baseline distance (meters)
```

### Ball Detection Parameters
```python
self.yellow_lower = np.array([20, 100, 100])   # HSV lower bounds
self.yellow_upper = np.array([35, 255, 255])   # HSV upper bounds
self.min_radius_px = 6                         # Minimum ball radius
self.max_radius_px = 200                       # Maximum ball radius
self.min_circularity = 0.6                     # Circularity threshold (0-1)
```

### Range Limits
```python
self.min_range = 0.4                # Minimum detection range (meters)
self.max_range = 6.0                # Maximum detection range (meters)
```

### Clustering Parameters
```python
self.cluster_eps = 0.35             # Clustering distance threshold (meters)
self.cluster_min = 6                # Minimum balls per cluster
```

### Temporal Smoothing
```python
self.alpha = 0.3                    # Smoothing factor (0.0-1.0)
                                    # Higher = more responsive, more noise
                                    # Lower = smoother, more lag
```

### Cluster Locking
```python
self.max_missing_frames = 10        # Max frames before losing lock
self.min_confidence = 5             # Min cluster size to maintain lock
```

## Usage

### Basic Usage (with RoboRio)
```python
from ball_cluster_vision import BallClusterVision

# Initialize vision system
vision = BallClusterVision(server_ip="10.92.14.2", enable_debug=True)

# In main loop
while True:
    left_frame = left_camera.read()
    right_frame = right_camera.read()
    
    vision.update(left_frame, right_frame)
    
    # Vision system publishes to NetworkTables:
    # - target/valid (bool)
    # - target/x (float) - field frame
    # - target/y (float) - field frame
    # - target/heading (float) - radians
    # - target/confidence (int) - number of balls in cluster
```

### Testing Without RoboRio
```bash
python test_vision_simulator.py
```

This runs the complete vision pipeline with:
- Mocked NetworkTables (no RoboRio needed)
- Simulated robot motion
- Synthetic test frames with yellow balls
- Performance metrics and debug output

### Single Camera Ball Detection
```python
from yellow_ball_detector import YellowBallDetector

detector = YellowBallDetector(camera_id=0)

# Interactive color calibration
detector.adjust_color_range()

# Run detection
detector.run()
```

**Controls:**
- `q` - Quit
- `s` - Save calibration images
- `c` - Toggle clustering (for single camera)

## NetworkTables Integration

### Published Topics
The vision system publishes to the `vision` table on NetworkTables:

**Target Information:**
- `target/valid` (boolean) - Is a valid target locked?
- `target/x` (number) - Target X in field frame (meters)
- `target/y` (number) - Target Y in field frame (meters)
- `target/heading` (number) - Target heading (radians)
- `target/confidence` (number) - Number of balls in target cluster

**Debug Topics** (when debugging):
- `debug/left_balls` (number)
- `debug/right_balls` (number)
- `debug/stereo_matches` (number)
- `debug/valid_points` (number)
- `debug/clusters_found` (number)
- `debug/largest_cluster_size` (number)

### Robot Pose Input
The vision system reads robot pose from NetworkTables:
- `robot/x` (number) - Robot X position (meters)
- `robot/y` (number) - Robot Y position (meters)
- `robot/heading` (number) - Robot heading (radians)

## Calibration

### Color Calibration
Run the interactive calibration tool:
```python
from yellow_ball_detector import YellowBallDetector

detector = YellowBallDetector()
detector.adjust_color_range()  # Use trackbars to adjust HSV ranges
```

Adjustable ranges:
- **Hue**: 0-180 (yellow is ~15-35)
- **Saturation**: 0-255 (100-255 for vibrant colors)
- **Value**: 0-255 (100-255 for bright colors)

### Camera Calibration
To calibrate focal length `fx` and baseline:
1. Place a ball at known distance D
2. Measure pixel disparity between cameras
3. Use formula: `baseline = (fx * baseline_physical) / disparity`

## Performance

### Typical Performance (from testing)
- **Frame Rate**: 12-15 FPS
- **Latency**: ~70ms per frame
- **Memory**: <100MB
- **CPU**: ~20-30% on single core

### Optimization Tips
- Increase `min_radius_px` to filter small noise faster
- Increase `cluster_eps` to merge nearby clusters
- Increase `alpha` for faster response time
- Use lower resolution images for faster processing

## Debug Visualization

When `enable_debug=True`, the system displays:

1. **Stereo Detection & Matching Window**
   - Green circles: detected balls
   - Blue lines: stereo matches
   - Statistics: ball counts

2. **Clustering Top-Down View**
   - Bird's eye perspective
   - Grid shows distance (0-6m forward, ±3m left/right)
   - Red/white: target cluster
   - Green: outlier points

3. **Vision Stats Panel**
   - Real-time metrics
   - Detection counts
   - Cluster information
   - Lock status

## Troubleshooting

### No balls detected
- Check color calibration (use `adjust_color_range()`)
- Verify lighting conditions
- Increase `yellow_lower` saturation if colors are washed out

### Low stereo matches
- Check camera calibration (baseline and focal length)
- Ensure cameras are properly synchronized
- Verify matching algorithm sensitivity

### Cluster lock lost frequently
- Increase `max_missing_frames` for patience
- Lower `cluster_eps` for stricter grouping
- Increase `alpha` for smoother tracking

### NetworkTables not connecting
- Verify RoboRio IP address
- Check firewall settings
- Ensure RoboRio is on the network

## Hardware Requirements

- **Cameras**: 2x USB or Ethernet (stereo setup)
- **CPU**: Multi-core processor recommended (Intel i7+ or equivalent)
- **Memory**: 2GB+ RAM
- **Network**: Connection to RoboRio (Ethernet recommended for low latency)

## Future Improvements

- [ ] Multi-object tracking (track multiple clusters simultaneously)
- [ ] Deep learning ball detection (ResNet/YOLO)
- [ ] GPU acceleration (CUDA/OpenCL)
- [ ] Field frame visualization overlay
- [ ] Real-time calibration from UI
- [ ] Ball trajectory prediction
- [ ] Adaptive color range based on lighting

## References

- OpenCV Documentation: https://docs.opencv.org/
- scikit-learn DBSCAN: https://scikit-learn.org/stable/modules/generated/sklearn.cluster.DBSCAN.html
- pynetworktables: https://github.com/robotpy/pynetworktables
- FIRST Robotics: https://www.firstinspires.org/

## License

Part of FRC Team 9214 codebase. For internal use.

## Support

For issues or questions, contact the FRC 9214 vision subsystem lead.
