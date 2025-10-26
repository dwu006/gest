#!/usr/bin/env python3
import sys
sys.path.insert(0, '/home/lekiwi/lerobot/src')

from scservo_sdk import PortHandler, PacketHandler, COMM_SUCCESS, COMM_RX_TIMEOUT

# Motor configuration
BAUDRATE = 1000000
PROTOCOL_VERSION = 0
DEVICENAME = '/dev/ttyACM0'  # Left arm port

# Initialize port and packet handler
portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)

if not portHandler.openPort():
    print(f"✗ Failed to open port {DEVICENAME}")
    sys.exit(1)

if not portHandler.setBaudRate(BAUDRATE):
    print(f"✗ Failed to set baudrate to {BAUDRATE}")
    sys.exit(1)

print("=" * 70)
print(f"Testing motors on {DEVICENAME} (LEFT ARM)")
print("=" * 70)

# Test each motor ID from 1-10
for motor_id in range(1, 11):
    print(f"\nTesting Motor ID {motor_id}...", end=" ")
    
    # Try to ping the motor
    model_number, comm_result, error = packetHandler.ping(portHandler, motor_id)
    
    if comm_result == COMM_SUCCESS:
        print(f"✓ FOUND - Model: {model_number}")
        
        # Try to read position
        pos, comm_result2, error2 = packetHandler.read2ByteTxRx(portHandler, motor_id, 56)
        if comm_result2 == COMM_SUCCESS:
            print(f"  Position: {pos}")
        else:
            print(f"  ⚠ Can't read position: {packetHandler.getRxPacketError(error2)}")
    else:
        if comm_result == COMM_RX_TIMEOUT:
            print("✗ NO RESPONSE (timeout)")
        else:
            result_str = packetHandler.getTxRxResult(comm_result)
            error_str = packetHandler.getRxPacketError(error) if error != 0 else ""
            print(f"✗ ERROR: {result_str} {error_str}")

portHandler.closePort()
print("\n" + "=" * 70)
print("Diagnosis complete")
print("=" * 70)
