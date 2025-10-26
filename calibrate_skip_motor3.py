from lerobot.robots.xlerobot import XLerobot, XLerobotConfig
import lerobot.motors.motors_bus as motors_bus
import lerobot.motors.feetech.feetech as feetech_module

# Save originals
original_assert_motors_exist = motors_bus.MotorsBus._assert_motors_exist
original_assert_same_firmware = feetech_module.FeetechMotorsBus._assert_same_firmware

def patched_assert_motors_exist(self):
    """Skip verification - motor 3 has overload error but others work"""
    if "/dev/ttyACM0" in str(self.port):
        print(f"[PATCHED] Skipping motor verification for {self.port} (motor 3 broken)")
        return
    return original_assert_motors_exist(self)

def patched_assert_same_firmware(self):
    """Skip firmware check"""
    if "/dev/ttyACM0" in str(self.port):
        print(f"[PATCHED] Skipping firmware check for {self.port}")
        return
    return original_assert_same_firmware(self)

motors_bus.MotorsBus._assert_motors_exist = patched_assert_motors_exist
feetech_module.FeetechMotorsBus._assert_same_firmware = patched_assert_same_firmware

print("=" * 70)
print("CALIBRATION - Skipping motor 3 checks")
print("ACM0 = LEFT ARM + HEAD (motor 3 broken but will skip it)")
print("ACM1 = RIGHT ARM + BASE")
print("=" * 70)

robot_config = XLerobotConfig()
robot = XLerobot(robot_config)

try:
    robot.connect(calibrate=True)
    print("\n✓ Calibration complete!")
except Exception as e:
    print(f"\n✗ Failed: {e}")
    import traceback
    traceback.print_exc()
finally:
    try:
        robot.disconnect()
    except:
        pass
