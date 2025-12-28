# YDLidar Setup for FRC 9214

## Quick Start

### 1. Basic Usage

```python
from lidar import YDLidar
import time

# Create and connect
lidar = YDLidar()
if lidar.connect():
    # Read a single scan
    scan = lidar.get_scan()
    if scan:
        print(f"Got {len(scan)} points")
        for angle, distance in scan[:5]:
            print(f"  Angle: {angle:.1f}°, Distance: {distance:.3f}m")
    
    # Or use continuous reading
    lidar.start_reading()
    time.sleep(1)
    latest = lidar.get_latest()
    lidar.stop_reading()
    
    lidar.disconnect()
```

## Files

### lidar.py
**Main working implementation.** Use this in your robot code.

Features:
- Auto-detect serial port
- Single scan reading
- Continuous background reading
- Convert to Cartesian coordinates (x, y)
- Minimal dependencies (pyserial, numpy)
- Thread-safe

**Example:**
```python
lidar = YDLidar()
lidar.connect()
points = lidar.get_cartesian()  # (x, y) in meters
lidar.disconnect()
```

### ydlidar_wrapper.py
Wrapper for the official YDLidar-SDK (requires compilation).

### ydlidar_direct.py
Alternative direct protocol implementation.

### eai_lidar_reader.py
Original EAI lidar implementation (different protocol).

## Installation

### Requirements
```bash
pip install pyserial numpy
```

### USB Driver
The YDLidar uses a CP210x USB-to-Serial chip. Driver should be automatic on Windows.

If you need manual driver installation:
- Download from: https://www.silabs.com/developers/usb-to-uart-bridge-vcp-drivers
- Or search "CP210x driver" for your OS

## Hardware Setup

1. **Connect YDLidar to USB**
   - The device should appear as `COM3` (or similar) on Windows
   - Check Device Manager under "Ports (COM & LPT)"

2. **Power Requirements**
   - YDLidar typically needs 5V power
   - Check your sensor's manual for power specifications

## Data Format

Each scan returns a list of tuples: `(angle_degrees, distance_meters)`

```python
scan = lidar.get_scan()
# Returns: [(0.0, 2.543), (0.5, 2.512), ...]
#          angle in degrees, distance in meters
```

### Cartesian Coordinates
Convert to x, y coordinates:
```python
points_xy = lidar.get_cartesian()
# Returns numpy array: [[x1, y1], [x2, y2], ...]
#                      in meters
```

## Troubleshooting

### No data received
1. Check the USB connection
2. Verify the port in Device Manager
3. The sensor may need 2-3 seconds to boot after power-on
4. Try different baud rates (115200, 19200)

### "No port found"
1. Check Device Manager for USB devices
2. Install CP210x driver if needed
3. Manually specify port: `YDLidar(port='COM3')`

### Permission error
- On Linux: Add user to `dialout` group
  ```bash
  sudo usermod -a -G dialout $USER
  ```

## Integration with FRC

### WPILib Integration
```python
from robotpy_ext.misc import MovingAverage
from lidar import YDLidar

class Robot(wpilib.TimedRobot):
    def robotInit(self):
        self.lidar = YDLidar()
        self.lidar.connect()
        self.lidar.start_reading()
    
    def robotPeriodic(self):
        scan = self.lidar.get_latest()
        if scan:
            # Process lidar data
            pass
    
    def disabledInit(self):
        self.lidar.disconnect()
```

## SDK Source

The official YDLidar SDK is in `../YDLidar-SDK/`
- Python bindings in `YDLidar-SDK/python/`
- Examples in `YDLidar-SDK/python/examples/`
- C++ source in `YDLidar-SDK/src/`

## References

- YDLidar GitHub: https://github.com/YDLIDAR/YDLidar-SDK
- Protocol documentation: See `YDLidar-SDK/doc/`
- Product docs: `../DFR1030 Development Manual Update.pdf`

## License

Code in this directory (lidar.py, ydlidar_*.py) is for FRC 9214.
YDLidar-SDK has its own license in the SDK folder.
