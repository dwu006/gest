from lerobot.robots.xlerobot import XLerobot, XLerobotConfig
import lerobot.motors.motors_bus as motors_bus
import lerobot.motors.feetech.feetech as feetech_module

# Patch to skip left arm (ACM1) completely - it has the broken motor
original_assert_motors_exist = motors_bus.MotorsBus._assert_motors_exist
original_assert_same_firmware = feetech_module.FeetechMotorsBus._assert_same_firmware
original_is_calibrated = feetech_module.FeetechMotorsBus.is_calibrated

def patched_assert_motors_exist(self):
    if "/dev/ttyACM1" in str(self.port):  # ACM1 is LEFT ARM (broken)
        print(f"[PATCHED] Skipping motor check for {self.port} (left arm broken)")
    return

def patched_assert_same_firmware(self):
    if "/dev/ttyACM1" in str(self.port):
        print(f"[PATCHED] Skipping firmware check for {self.port}")
    return

@property
def patched_is_calibrated(self):
    """Skip calibration check for left arm"""
    if "/dev/ttyACM1" in str(self.port):  # ACM1 is LEFT ARM (broken)
        print(f"[PATCHED] Pretending {self.port} is calibrated (left arm broken)")
        return True  # Pretend it's calibrated so it doesn't try to read
    else:
        return original_is_calibrated.fget(self)

motors_bus.MotorsBus._assert_motors_exist = patched_assert_motors_exist
feetech_module.FeetechMotorsBus._assert_same_firmware = patched_assert_same_firmware
feetech_module.FeetechMotorsBus.is_calibrated = patched_is_calibrated

print("=" * 70)
print("Calibrating RIGHT ARM (ACM0), HEAD, and BASE only...")
print("Left arm (ACM1) will be skipped due to broken motor 3")
print("=" * 70)

robot_config = XLerobotConfig()
robot = XLerobot(robot_config)

try:
    # Connect with calibration
    robot.connect(calibrate=True)
    print("\n✓ Right arm calibrated successfully!")
    print("Calibration data saved. Now you can run the teleop script.")
except Exception as e:
    print(f"✗ Calibration failed: {e}")
    import traceback
    traceback.print_exc()
finally:
    try:
        robot.disconnect()
    except:
        pass
