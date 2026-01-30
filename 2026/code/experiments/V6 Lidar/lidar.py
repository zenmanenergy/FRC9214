"""
YDLidar GS2 Sensor Interface
Manages continuous scanning and data collection from GS2 LIDAR sensor
Designed for integration with robot.py using serial communication
"""

import serial
import struct
import time
import threading
from typing import Optional, List, Tuple


class GS2Lidar:
    """YDLidar GS2 (DFR1030) Sensor Interface for RoboRIO/Robot Integration"""
    
    BAUD_RATE = 921600
    TIMEOUT = 1.0
    
    # Commands
    CMD_ADDRESS = 0x60
    CMD_PARAMS = 0x61
    CMD_VERSION = 0x62
    CMD_SCAN = 0x63
    CMD_STOP = 0x64
    
    def __init__(self, port=None):
        """
        Initialize LIDAR connection
        
        Args:
            port: Serial port (e.g., 'COM3' on Windows, '/dev/ttyUSB0' on RoboRIO)
                  If None, will auto-detect
        """
        self.port = port or self._find_port()
        self.ser = None
        self.connected = False
        
        # Latest scan data
        self.latest_distances = []
        self.latest_intensities = []
        self.frame_count = 0
        
        # Threading for background scanning
        self.scanning = False
        self.scan_thread = None
        
    @staticmethod
    def _find_port():
        """Auto-detect GS2 port"""
        try:
            import serial.tools.list_ports
            ports = serial.tools.list_ports.comports()
            for port in ports:
                if 'CP210x' in port.description or 'USB' in port.description:
                    return port.device
        except:
            pass
        # Fallback ports
        return None  # Will raise error if not found
    
    def connect(self) -> bool:
        """Connect to LIDAR"""
        try:
            if not self.port:
                print("[ERR] No LIDAR port found")
                return False
            
            self.ser = serial.Serial(
                self.port,
                self.BAUD_RATE,
                timeout=self.TIMEOUT,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE
            )
            time.sleep(1)
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            time.sleep(0.2)
            
            self.connected = True
            print(f"[OK] LIDAR connected on {self.port}")
            return True
        except Exception as e:
            print(f"[ERR] LIDAR connection failed: {e}")
            self.connected = False
            return False
    
    def disconnect(self):
        """Disconnect from LIDAR"""
        if self.connected:
            self.stop_scan()
            if self.ser:
                self.ser.close()
            self.connected = False
            print("[OK] LIDAR disconnected")
    
    @staticmethod
    def _calc_checksum(data):
        """XOR checksum"""
        result = 0
        for b in data:
            result ^= b
        return result & 0xFF
    
    def _send_cmd(self, cmd_type, data=b''):
        """Send command"""
        packet = bytearray([0xA5, 0xA5, 0xA5, 0xA5, 0x00, cmd_type])
        packet.extend(struct.pack('<H', len(data)))
        packet.extend(data)
        packet.append(self._calc_checksum(packet))
        self.ser.write(packet)
    
    def _read_response(self, timeout=0.5):
        """Read raw response"""
        start = time.time()
        buffer = bytearray()
        
        while time.time() - start < timeout:
            if self.ser.in_waiting:
                buffer.extend(self.ser.read(self.ser.in_waiting))
            time.sleep(0.001)
        
        return buffer
    
    def get_device_info(self) -> dict:
        """Get device address and version"""
        info = {}
        
        # Get address
        self._send_cmd(self.CMD_ADDRESS)
        time.sleep(0.1)
        response = self._read_response()
        if len(response) >= 9:
            info['address'] = response[4]
        
        # Get version
        self._send_cmd(self.CMD_VERSION)
        time.sleep(0.1)
        response = self._read_response()
        if len(response) >= 30:
            info['version'] = response[10:13].hex()
            info['serial'] = response[13:29].hex()
        
        return info
    
    def start_scan(self) -> bool:
        """Start continuous scanning"""
        if not self.connected:
            return False
        
        self._send_cmd(self.CMD_SCAN)
        time.sleep(0.2)
        
        response = self._read_response(timeout=0.5)
        if response and len(response) > 5 and response[5] == self.CMD_SCAN:
            self.scanning = True
            print("[OK] LIDAR scan started")
            return True
        
        print("[ERR] LIDAR scan failed to start")
        return False
    
    def stop_scan(self):
        """Stop scanning"""
        if self.scanning:
            self._send_cmd(self.CMD_STOP)
            time.sleep(0.2)
            self._read_response(timeout=0.3)
            self.scanning = False
            print("[OK] LIDAR scan stopped")
    
    def _read_frame(self) -> Optional[Tuple[List[int], List[int]]]:
        """
        Read one scan frame
        Returns: (distances_mm, intensities) or None
        """
        response = self._read_response(timeout=0.5)
        
        # Find frame header
        for i in range(len(response) - 10):
            if (response[i:i+4] == bytes([0xA5, 0xA5, 0xA5, 0xA5]) and
                response[i+5] == self.CMD_SCAN):
                
                frame_start = i + 12
                
                if frame_start + 320 > len(response):
                    return None
                
                distances = []
                intensities = []
                
                for j in range(160):
                    offset = frame_start + j * 2
                    byte1 = response[offset]
                    byte2 = response[offset + 1]
                    
                    combined = byte1 | (byte2 << 8)
                    distance = combined & 0x1FF
                    intensity = (combined >> 9) & 0x7F
                    
                    distances.append(distance)
                    intensities.append(intensity)
                
                return (distances, intensities)
        
        return None
    
    def update(self):
        """Update latest scan data (called from robot periodic)"""
        if not self.scanning:
            return
        
        result = self._read_frame()
        if result:
            self.latest_distances, self.latest_intensities = result
            self.frame_count += 1
    
    def get_latest_scan(self) -> Tuple[List[int], List[int]]:
        """Get most recent scan data"""
        return (self.latest_distances, self.latest_intensities)
    
    def get_frame_count(self) -> int:
        """Get total frames received"""
        return self.frame_count
    
    def get_front_distance(self, degrees_fov=20) -> int:
        """
        Get average distance in front of robot (center FOV)
        
        Args:
            degrees_fov: Field of view in degrees (default 20 degrees = +/- 10)
        
        Returns:
            Distance in mm (0 if no valid data)
        """
        if not self.latest_distances or len(self.latest_distances) < 160:
            return 0
        
        # Center 160 points out of 100 degree FOV = 0.625 degrees per point
        # Front-center is around point 80
        degrees_per_point = 100.0 / 160
        points_to_average = int(degrees_fov / degrees_per_point)
        
        center_idx = 80
        start_idx = max(0, center_idx - points_to_average // 2)
        end_idx = min(160, center_idx + points_to_average // 2)
        
        valid_distances = [d for d in self.latest_distances[start_idx:end_idx] if d > 0]
        
        if valid_distances:
            return int(sum(valid_distances) / len(valid_distances))
        return 0
    
    def get_obstacle_left(self) -> bool:
        """Check if obstacle on left side"""
        if not self.latest_distances:
            return False
        
        # Left side: points 0-50
        left_distances = [d for d in self.latest_distances[0:50] if d > 0 and d < 500]
        return len(left_distances) > 10
    
    def get_obstacle_right(self) -> bool:
        """Check if obstacle on right side"""
        if not self.latest_distances:
            return False
        
        # Right side: points 110-160
        right_distances = [d for d in self.latest_distances[110:160] if d > 0 and d < 500]
        return len(right_distances) > 10
    
    def print_debug(self):
        """Print debug information"""
        distances, intensities = self.get_latest_scan()
        if distances:
            valid = [d for d in distances if d > 0]
            if valid:
                print(f"[LIDAR] Frame {self.frame_count}: {len(valid)}/160 points, "
                      f"range: {min(valid)}-{max(valid)}mm, "
                      f"front: {self.get_front_distance()}mm")
