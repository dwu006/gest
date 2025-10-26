#!/usr/bin/env python3
"""Direct scan of motors using scservo SDK"""
from scservo_sdk import PortHandler, PacketHandler, COMM_SUCCESS

def scan_port(port_name, baudrate=1000000):
    print(f"\n{'='*70}")
    print(f"Scanning {port_name}")
    print('='*70)
    
    port = PortHandler(port_name)
    
    if not port.openPort():
        print(f"✗ Cannot open {port_name}")
        return
    
    if not port.setBaudRate(baudrate):
        print(f"✗ Cannot set baudrate")
        port.closePort()
        return
    
    print(f"✓ Port opened at {baudrate} baud")
    
    # Try both protocol versions
    for protocol in [0, 1, 2]:
        print(f"\n--- Testing Protocol Version {protocol} ---")
        handler = PacketHandler(protocol)
        found = []
        
        for motor_id in range(1, 11):
            model, result, error = handler.ping(port, motor_id)
            if result == COMM_SUCCESS:
                found.append((motor_id, model))
                print(f"  Motor {motor_id}: ✓ Model {model}")
        
        if found:
            print(f"\n✓ Found {len(found)} motors with protocol {protocol}")
            break
        else:
            print(f"  No motors found with protocol {protocol}")
    
    if not found:
        print(f"\n✗ NO MOTORS FOUND ON {port_name}")
        print("Possible issues:")
        print("  - Motor controller board not powered")
        print("  - Wrong baudrate (trying 1000000)")
        print("  - Damaged cable or connectors")
        print("  - Motors not daisy-chained properly")
    
    port.closePort()

# Scan both ports
scan_port("/dev/ttyACM0")  # Left arm
scan_port("/dev/ttyACM1")  # Right arm

print(f"\n{'='*70}")
print("Scan complete")
print('='*70)
