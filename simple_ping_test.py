#!/usr/bin/env python3
"""Simple ping test for both ports"""
import subprocess
import time

print("=" * 70)
print("SIMPLE HARDWARE TEST")
print("=" * 70)

# Power cycle recommendation
print("\n⚡ RECOMMENDED: Power cycle the robot arms:")
print("1. Turn OFF power to both arms")
print("2. Wait 10 seconds")
print("3. Turn ON power to both arms")
print("4. Wait 5 seconds for motors to initialize")
print("\nPress ENTER after power cycling (or just press ENTER to skip)...")
input()

print("\nTesting serial connections...")

# Test if ports are accessible
for port in ["/dev/ttyACM0", "/dev/ttyACM1"]:
    print(f"\nTesting {port}...")
    try:
        result = subprocess.run(
            ["python", "-c", 
             f"from scservo_sdk import *; "
             f"p = PortHandler('{port}'); "
             f"print('✓ Port opened') if p.openPort() and p.setBaudRate(1000000) else print('✗ Failed'); "
             f"p.closePort()"],
            capture_output=True,
            text=True,
            timeout=3
        )
        print(result.stdout.strip() if result.stdout else result.stderr.strip())
    except Exception as e:
        print(f"✗ Error: {e}")

print("\n" + "=" * 70)
print("If both ports opened successfully, the USB connections are fine.")
print("If not, check:")
print("  1. USB cables are firmly plugged in")
print("  2. Motor power supplies are ON")
print("  3. Motor controller boards have power LEDs lit")
print("=" * 70)
