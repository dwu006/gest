#!/usr/bin/env python3
"""Ping motors using lerobot's actual motor bus"""
from lerobot.motors.feetech import FeetechMotorsBus

print("=" * 70)
print("PINGING MOTORS ON BOTH PORTS")
print("=" * 70)

# Test ACM0 (Left arm + head)
print("\nTesting /dev/ttyACM0 (Left arm + head)...")
try:
    from lerobot.robots.xlerobot import XLerobot, XLerobotConfig
    config = XLerobotConfig()
    robot = XLerobot(config)
    
    # Just try to connect to see what happens
    robot.bus1._connect_with_ping()
    print(f"✓ Found motors on bus1: {robot.bus1.motor_names}")
    robot.bus1.disconnect()
except Exception as e:
    print(f"✗ Bus1 error: {e}")

# Test ACM1 (Right arm + base)
print("\nTesting /dev/ttyACM1 (Right arm + base)...")
try:
    robot.bus2._connect_with_ping()
    print(f"✓ Found motors on bus2: {robot.bus2.motor_names}")
    robot.bus2.disconnect()
except Exception as e:
    print(f"✗ Bus2 error: {e}")

print("\n" + "=" * 70)
