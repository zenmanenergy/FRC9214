#!/usr/bin/env python3
"""
Test script to verify NetworkTables connection
"""

from networktables import NetworkTables
import time

print("Connecting to NetworkTables at 10.92.14.200...")
NetworkTables.initialize(server="10.92.14.200")

print("Waiting for connection...")
time.sleep(2)

table = NetworkTables.getTable("ros_data")

print("Reading message...")
message = table.getString("message", "NOT FOUND")
print(f"Message: {message}")

if message == "hello world":
    print("SUCCESS! Connected and receiving data!")
else:
    print("FAILED - Not receiving expected message")

NetworkTables.shutdown()
