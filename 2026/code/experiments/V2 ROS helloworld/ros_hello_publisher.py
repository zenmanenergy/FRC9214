#!/usr/bin/env python3
"""
Simple NetworkTables Server that publishes "hello world"
Run this on the Ubuntu computer (10.92.14.100)
"""

from networktables import NetworkTables
import time

def main():
    # Initialize NetworkTables as a server
    print("Starting NetworkTables server...")
    NetworkTables.initialize(server="0.0.0.0")
    
    # Get or create the ros_data table
    table = NetworkTables.getTable("ros_data")
    
    print("NetworkTables server started!")
    print("Publishing 'hello world' every 1 second...")
    
    try:
        while True:
            # Publish the hello world message
            table.putString("message", "hello world")
            print("Published: hello world")
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nShutting down...")
        NetworkTables.shutdown()

if __name__ == '__main__':
    main()
