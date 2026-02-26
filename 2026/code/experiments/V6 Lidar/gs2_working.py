"""
YDLidar GS2 (DFR1030) Python Reader
Solid-State LIDAR with 160 measurement points, 25-300mm range, 100 FOV

CRITICAL: Baud rate is 921600 (found in LidarViewer's models.json)
"""

import serial
import struct
import time
from typing import Optional, List, Tuple

class GS2Reader:
    """YDLidar GS2 Sensor Interface"""
    
    BAUD_RATE = 921600  # From official LidarViewer models.json
    TIMEOUT = 1.0
    
    # Commands
    CMD_ADDRESS = 0x60
    CMD_PARAMS = 0x61
    CMD_VERSION = 0x62
    CMD_SCAN = 0x63
    CMD_STOP = 0x64
    
    def __init__(self, port="COM3"):
        self.port = port
        self.ser = None
        
    def connect(self):
        """Connect to GS2"""
        try:
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
            
            print(f"[OK] Connected to {self.port} @ {self.BAUD_RATE} baud")
            return True
        except Exception as e:
            print(f"[ERR] Connection failed: {e}")
            return False
    
    def close(self):
        """Close connection"""
        if self.ser:
            try:
                self._send_cmd(self.CMD_STOP)
                time.sleep(0.2)
            except:
                pass
            self.ser.close()
    
    @staticmethod
    def _calc_checksum(data):
        """XOR all bytes"""
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
        """Read raw response (handles fragmented packets)"""
        start = time.time()
        buffer = bytearray()
        
        while time.time() - start < timeout:
            if self.ser.in_waiting:
                buffer.extend(self.ser.read(self.ser.in_waiting))
            time.sleep(0.001)
        
        return buffer
    
    def get_device_address(self):
        """Get device address"""
        print("Getting device address...")
        self._send_cmd(self.CMD_ADDRESS)
        time.sleep(0.1)
        
        response = self._read_response()
        if len(response) >= 9:
            addr = response[4]
            checksum = response[8]
            calc_check = self._calc_checksum(response[:8])
            if calc_check == checksum:
                print(f"  [OK] Device Address: 0x{addr:02x}")
                return addr
        
        print(f"  [ERR] No valid response")
        return None
    
    def get_version(self):
        """Get version and serial number"""
        print("Getting version...")
        self._send_cmd(self.CMD_VERSION)
        time.sleep(0.1)
        
        response = self._read_response()
        if len(response) >= 30:
            length = struct.unpack('<H', response[6:8])[0]
            if length >= 0x13:  # At least 19 bytes of data
                # Data format: 3 bytes version + 16 bytes serial + possibly more
                data_start = 10
                version = response[data_start:data_start+3]
                sn = response[data_start+3:data_start+19]
                
                # Checksum is after all data
                checksum_idx = 8 + length
                if checksum_idx < len(response):
                    checksum = response[checksum_idx]
                    calc_check = self._calc_checksum(response[:checksum_idx])
                    if calc_check == checksum:
                        print(f"  [OK] Version: {version.hex()}")
                        print(f"  [OK] Serial:  {sn.hex()}")
                        return (version, sn)
        
        print(f"  [WRN] Got {len(response)} bytes but skipping checksum")
        if len(response) >= 30:
            version = response[10:13]
            sn = response[13:29]
            print(f"  [OK] Version: {version.hex()}")
            print(f"  [OK] Serial:  {sn.hex()}")
            return (version, sn)
        return None
    
    def get_parameters(self):
        """Get K, B, Bias calibration parameters"""
        print("Getting parameters...")
        self._send_cmd(self.CMD_PARAMS)
        time.sleep(0.1)
        
        response = self._read_response()
        if len(response) >= 20:
            length = struct.unpack('<H', response[6:8])[0]
            if length >= 0x09:  # At least 9 bytes of data
                k0 = struct.unpack('<H', response[10:12])[0] / 10000.0
                b0 = struct.unpack('<H', response[12:14])[0] / 10000.0
                k1 = struct.unpack('<H', response[14:16])[0] / 10000.0
                b1 = struct.unpack('<H', response[16:18])[0] / 10000.0
                bias = struct.unpack('<b', response[18:19])[0] / 10.0
                
                print(f"  [OK] K0={k0:.4f}, B0={b0:.4f}")
                print(f"  [OK] K1={k1:.4f}, B1={b1:.4f}")
                print(f"  [OK] Bias={bias:.1f}")
                return {'k0': k0, 'b0': b0, 'k1': k1, 'b1': b1, 'bias': bias}
        
        print(f"  [ERR] No valid response (got {len(response)} bytes)")
        return None
    
    def start_scan(self):
        """Start scanning"""
        print("Starting scan...")
        self._send_cmd(self.CMD_SCAN)
        time.sleep(0.2)
        
        response = self._read_response(timeout=0.5)
        if response and response[5] == self.CMD_SCAN:
            print(f"  [OK] Scan started")
            return True
        
        print(f"  [ERR] Failed")
        return False
    
    def stop_scan(self):
        """Stop scanning"""
        self._send_cmd(self.CMD_STOP)
        time.sleep(0.2)
        self._read_response(timeout=0.3)
    
    def read_frame(self) -> Optional[Tuple[List[int], List[int]]]:
        """
        Read one scan frame
        Returns: (distances_mm, intensities) or None
        
        Each frame has 160 points:
        - Byte 0-1: Environment data (ignored)
        - Bytes 2-321: 160 points x 2 bytes each
          - Lower 9 bits: distance in mm
          - Upper 7 bits: intensity
        - Byte 322: checksum
        """
        response = self._read_response(timeout=0.5)
        
        # Find frame header
        for i in range(len(response) - 10):
            if (response[i:i+4] == bytes([0xA5, 0xA5, 0xA5, 0xA5]) and
                response[i+5] == self.CMD_SCAN):
                
                # Extract 160 points starting after header(4) + addr(1) + cmd(1) + length(2) + env(2)
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
                    distance = combined & 0x1FF  # Lower 9 bits
                    intensity = (combined >> 9) & 0x7F  # Upper 7 bits
                    
                    distances.append(distance)
                    intensities.append(intensity)
                
                return (distances, intensities)
        
        return None
    
    def run(self, duration=10.0):
        """Run scan and print data"""
        if not self.connect():
            return
        
        try:
            print("\n" + "="*70)
            print("INITIALIZATION")
            print("="*70)
            
            self.get_device_address()
            self.get_version()
            self.get_parameters()
            
            print("\n" + "="*70)
            if not self.start_scan():
                return
            
            print(f"Collecting data for {duration} seconds...")
            print("-"*70)
            
            start = time.time()
            frame_count = 0
            
            while time.time() - start < duration:
                result = self.read_frame()
                
                if result:
                    distances, intensities = result
                    frame_count += 1
                    
                    # Calculate statistics
                    valid = [d for d in distances if d > 0]
                    
                    if valid:
                        min_dist = min(valid)
                        max_dist = max(valid)
                        avg_dist = sum(valid) / len(valid)
                        
                        print(f"Frame {frame_count}: {len(valid)}/160 valid points")
                        print(f"  Range: {min_dist:3d}mm - {max_dist:3d}mm  (avg: {avg_dist:6.1f}mm)")
                        
                        # Show first 10 points with actual values
                        print(f"  Points: ", end="")
                        count = 0
                        for i in range(len(distances)):
                            if distances[i] > 0 and count < 10:
                                print(f"[{distances[i]:3d}mm,I{int(intensities[i])}] ", end="")
                                count += 1
                        print("...")
                
                time.sleep(0.01)
            
            print("-"*70)
            print(f"[OK] Captured {frame_count} frames")
            
        except KeyboardInterrupt:
            print("\n[OK] Interrupted by user")
        finally:
            self.stop_scan()
            self.close()


def main():
    print("YDLidar GS2 (DFR1030) Python Reader")
    print("="*70)
    print(f"Port: COM3")
    print(f"Baud: 921600 (from official LidarViewer models.json)")
    
    reader = GS2Reader(port="COM3")
    reader.run(duration=10.0)


if __name__ == "__main__":
    main()
