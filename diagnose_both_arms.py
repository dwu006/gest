#!/usr/bin/env python3
"""Test both arms to identify the problem"""

from lerobot.robots.xlerobot import XLerobotConfig

# Get the motor configs from XLerobot
config = XLerobotConfig()

print("=" * 70)
print("Motor Configuration from XLerobotConfig:")
print("=" * 70)
print(f"Port1 (ACM0): {config.port1}")
print(f"Port2 (ACM1): {config.port2}")
print(f"Motors on port1: {list(config.motors_port1.keys())}")
print(f"Motors on port2: {list(config.motors_port2.keys())}")

# Now test with actual robot connection
from lerobot.robots.xlerobot import XLerobot

print("\n" + "=" * 70)
print("Testing LEFT ARM (ACM0) - attempting connection")
print("=" * 70)

robot = XLerobot(config)

# Try to connect just bus1 (left arm)
try:
    robot.bus1._connect_with_ping()
    print(f"✓ Connected to bus1 (ACM0)")
    print(f"Found motors: {robot.bus1.motor_names}")
    
    for motor_name in robot.bus1.motor_names:
        try:
            pos = robot.bus1.read("Present_Position", motor_name)
            print(f"  {motor_name}: Position = {pos:.2f}")
        except Exception as e:
            print(f"  {motor_name}: ✗ {str(e)[:80]}")
    
    robot.bus1.disconnect()
except Exception as e:
    print(f"✗ Bus1 connection failed: {e}")

print("\n" + "=" * 70)
print("Testing RIGHT ARM (ACM1) - attempting connection")
print("=" * 70)

# Try to connect just bus2 (right arm)
try:
    robot.bus2._connect_with_ping()
    print(f"✓ Connected to bus2 (ACM1)")
    print(f"Found motors: {robot.bus2.motor_names}")
    
    for motor_name in robot.bus2.motor_names:
        try:
            pos = robot.bus2.read("Present_Position", motor_name)
            print(f"  {motor_name}: Position = {pos:.2f}")
        except Exception as e:
            print(f"  {motor_name}: ✗ {str(e)[:80]}")
    
    robot.bus2.disconnect()
except Exception as e:
    print(f"✗ Bus2 connection failed: {e}")

print("\n" + "=" * 70)
print("Diagnosis complete")
print("=" * 70)
