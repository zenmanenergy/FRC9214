"""
YDLidar GS2 WebSocket Server
Streams LIDAR scan data to connected clients via WebSocket
Serves visualizer web page on localhost:8765
"""

import asyncio
import json
import math
import aiohttp
from aiohttp import web
import sys
import os
from datetime import datetime
from pathlib import Path

# Add parent directory to path to import GS2Lidar
parent_dir = str(Path(__file__).parent.parent)
sys.path.insert(0, parent_dir)
from lidar import GS2Lidar


class LidarWebSocketServer:
    """WebSocket server for broadcasting LIDAR data"""
    
    def __init__(self, host='0.0.0.0', port=8765):
        self.host = host
        self.port = port
        self.lidar = None
        self.clients = set()
        self.running = False
        self.app = web.Application()
        
        # Setup routes
        self.app.router.add_get('/', self.handle_http)
        self.app.router.add_get('/ws', self.handle_websocket)
        
    async def handle_http(self, request):
        """Serve HTML for HTTP GET requests"""
        html_path = Path(__file__).parent / 'index.html'
        try:
            with open(html_path, 'r') as f:
                html_content = f.read()
            return web.Response(text=html_content, content_type='text/html')
        except Exception as e:
            return web.Response(text=f'File not found: {e}', status=404)
    
    async def handle_websocket(self, request):
        """Handle WebSocket connections"""
        ws = web.WebSocketResponse()
        await ws.prepare(request)
        
        self.clients.add(ws)
        print(f"[WS] Client connected. Total: {len(self.clients)}")
        
        try:
            # Keep connection open
            async for msg in ws:
                if msg.type == aiohttp.WSMsgType.ERROR:
                    print(f'[WS] Error: {ws.exception()}')
        finally:
            self.clients.discard(ws)
            print(f"[WS] Client disconnected. Total: {len(self.clients)}")
        
        return ws
    
    async def broadcast_frame(self):
        """Continuously read LIDAR frames and broadcast to all clients"""
        frame_count = 0
        
        while self.running:
            try:
                # Update LIDAR data
                self.lidar.update()
                
                distances, intensities = self.lidar.get_latest_scan()
                
                if distances and len(distances) == 160:
                    frame_count += 1
                    
                    # Convert to cartesian coordinates for visualization
                    # 160 points over 100 degree FOV = 0.625 degrees per point
                    # Center at 50 degrees, so range is -50 to +50 degrees
                    points = []
                    for i, (distance, intensity) in enumerate(zip(distances, intensities)):
                        if distance > 0:  # Only include valid points
                            angle_degrees = -50 + (i * 0.625)  # -50 to +50 degrees
                            angle_radians = angle_degrees * (3.14159 / 180)
                            
                            # Cartesian coordinates (mm)
                            x = distance * math.sin(angle_radians)
                            y = distance * math.cos(angle_radians)
                            
                            points.append({
                                'x': x,
                                'y': y,
                                'distance': distance,
                                'intensity': intensity,
                                'angle': angle_degrees,
                                'index': i
                            })
                    
                    frame_data = {
                        'type': 'lidar_frame',
                        'frame_count': frame_count,
                        'timestamp': datetime.now().isoformat(),
                        'total_points': len(distances),
                        'valid_points': len(points),
                        'points': points,
                        'front_distance': self.lidar.get_front_distance(),
                        'obstacle_left': self.lidar.get_obstacle_left(),
                        'obstacle_right': self.lidar.get_obstacle_right()
                    }
                    
                    # Broadcast to all connected clients
                    if self.clients:
                        message = json.dumps(frame_data)
                        disconnected = []
                        for client in self.clients:
                            try:
                                await client.send_str(message)
                            except Exception as e:
                                print(f"[WS] Error sending to client: {e}")
                                disconnected.append(client)
                        
                        # Remove disconnected clients
                        for client in disconnected:
                            self.clients.discard(client)
                
                # Poll at ~40 Hz (25ms per frame)
                await asyncio.sleep(0.025)
                
            except Exception as e:
                print(f"[ERR] Error broadcasting frame: {e}")
                await asyncio.sleep(0.1)
    
    async def main(self):
        """Main server loop"""
        print(f"[WS] Initializing LIDAR...")
        
        # Connect to LIDAR
        self.lidar = GS2Lidar()
        if not self.lidar.connect():
            print("[ERR] Failed to connect to LIDAR")
            return False
        
        if not self.lidar.start_scan():
            print("[ERR] Failed to start LIDAR scan")
            return False
        
        print(f"[WS] LIDAR connected on {self.lidar.port}")
        print(f"[WS] Starting HTTP+WebSocket server on http://{self.host}:{self.port}")
        print(f"[WS] Open browser to http://localhost:{self.port}")
        
        self.running = True
        
        # Start broadcast task
        broadcast_task = asyncio.create_task(self.broadcast_frame())
        
        # Start aiohttp server
        runner = web.AppRunner(self.app)
        await runner.setup()
        site = web.TCPSite(runner, self.host, self.port)
        await site.start()
        
        try:
            # Run until interrupted
            await asyncio.sleep(3600)  # Keep running for up to 1 hour
        except KeyboardInterrupt:
            print("\n[WS] Shutting down...")
        finally:
            self.running = False
            broadcast_task.cancel()
            await runner.cleanup()
            self.lidar.disconnect()


async def main():
    """Entry point"""
    server = LidarWebSocketServer(host='0.0.0.0', port=8765)
    success = await server.main()
    return success


if __name__ == '__main__':
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n[WS] Server stopped")
    except Exception as e:
        print(f"[ERR] Server error: {e}")
