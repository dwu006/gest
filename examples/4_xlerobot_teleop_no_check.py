import sys
import tty
import termios
import time
import numpy as np
from lerobot.robots.xlerobot import XLerobot, XLerobotConfig
from lerobot.model.SO101Robot import SO101Kinematics

# Patch the motor bus to skip model and firmware verification
import lerobot.motors.motors_bus as motors_bus
import lerobot.motors.feetech.feetech as feetech_module

original_assert_motors_exist = motors_bus.MotorsBus._assert_motors_exist
original_assert_same_firmware = feetech_module.FeetechMotorsBus._assert_same_firmware

def patched_assert_motors_exist(self):
    """Skip motor model verification"""
    print(f"[PATCHED] Skipping motor model check for {self.port}")
    return

def patched_assert_same_firmware(self):
    """Skip firmware version check"""
    print(f"[PATCHED] Skipping firmware version check for {self.port}")
    return

motors_bus.MotorsBus._assert_motors_exist = patched_assert_motors_exist
feetech_module.FeetechMotorsBus._assert_same_firmware = patched_assert_same_firmware

# Joint maps
LEFT_JOINT_MAP = {
    "shoulder_pan": "left_arm_shoulder_pan",
    "shoulder_lift": "left_arm_shoulder_lift",
    "elbow_flex": "left_arm_elbow_flex",
    "wrist_flex": "left_arm_wrist_flex",
    "wrist_roll": "left_arm_wrist_roll",
    "gripper": "left_arm_gripper",
}

RIGHT_JOINT_MAP = {
    "shoulder_pan": "right_arm_shoulder_pan",
    "shoulder_lift": "right_arm_shoulder_lift",
    "elbow_flex": "right_arm_elbow_flex",
    "wrist_flex": "right_arm_wrist_flex",
    "wrist_roll": "right_arm_wrist_roll",
    "gripper": "right_arm_gripper",
}

HEAD_MOTOR_MAP = {
    "head_motor_1": "head_motor_1",
    "head_motor_2": "head_motor_2",
}

class SimpleHeadControl:
    def __init__(self, initial_obs, kp=0.81):
        self.kp = kp
        self.degree_step = 1
        self.target_positions = {
            "head_motor_1": initial_obs.get("head_motor_1.pos", 0.0),
            "head_motor_2": initial_obs.get("head_motor_2.pos", 0.0),
        }
        self.zero_pos = {"head_motor_1": 0.0, "head_motor_2": 0.0}

    def move_to_zero_position(self, robot):
        print("[HEAD] Moving to Zero Position")
        self.target_positions = self.zero_pos.copy()
        for _ in range(50):
            action = self.p_control_action(robot)
            robot.send_action(action)
            time.sleep(0.02)

    def handle_key(self, key, robot):
        if key == ',':
            self.target_positions["head_motor_1"] += self.degree_step
            print(f"[HEAD] head_motor_1+: {self.target_positions['head_motor_1']}")
        elif key == '.':
            self.target_positions["head_motor_1"] -= self.degree_step
            print(f"[HEAD] head_motor_1-: {self.target_positions['head_motor_1']}")
        elif key == '<':
            self.target_positions["head_motor_2"] += self.degree_step
            print(f"[HEAD] head_motor_2+: {self.target_positions['head_motor_2']}")
        elif key == '>':
            self.target_positions["head_motor_2"] -= self.degree_step
            print(f"[HEAD] head_motor_2-: {self.target_positions['head_motor_2']}")
        elif key == '?':
            self.move_to_zero_position(robot)

    def p_control_action(self, robot):
        obs = robot.get_observation()
        action = {}
        for motor in self.target_positions:
            current = obs.get(f"{HEAD_MOTOR_MAP[motor]}.pos", 0.0)
            error = self.target_positions[motor] - current
            control = self.kp * error
            action[f"{HEAD_MOTOR_MAP[motor]}.pos"] = current + control
        return action

class SimpleTeleopArm:
    def __init__(self, kinematics, joint_map, initial_obs, prefix="left", kp=0.81):
        self.kinematics = kinematics
        self.joint_map = joint_map
        self.prefix = prefix
        self.kp = kp
        
        self.joint_positions = {
            "shoulder_pan": initial_obs[f"{prefix}_arm_shoulder_pan.pos"],
            "shoulder_lift": initial_obs[f"{prefix}_arm_shoulder_lift.pos"],
            "elbow_flex": initial_obs[f"{prefix}_arm_elbow_flex.pos"],
            "wrist_flex": initial_obs[f"{prefix}_arm_wrist_flex.pos"],
            "wrist_roll": initial_obs[f"{prefix}_arm_wrist_roll.pos"],
            "gripper": initial_obs[f"{prefix}_arm_gripper.pos"],
        }
        
        self.current_x = 0.1629
        self.current_y = 0.1131
        self.pitch = 0.0
        self.degree_step = 3
        self.xy_step = 0.0081
        
        self.target_positions = self.joint_positions.copy()

    def move_to_zero_position(self, robot):
        print(f"[{self.prefix}] Moving to Zero Position")
        zero_pos = {k: 0.0 for k in self.target_positions}
        self.target_positions = zero_pos
        
        for _ in range(100):
            action = self.p_control_action(robot)
            if self.prefix == "left":
                robot.send_action({**action, **{}, **{}, **{}})
            else:
                robot.send_action({**{}, **action, **{}, **{}})
            time.sleep(0.02)

    def handle_key(self, key, robot):
        moved = False
        
        if key in ['w', 's', 'a', 'd', '8', '2', '4', '6']:
            if key == 'w' or key == '8':
                self.current_x += self.xy_step
                moved = True
                print(f"[{self.prefix}] X+: {self.current_x:.4f}")
            elif key == 's' or key == '2':
                self.current_x -= self.xy_step
                moved = True
                print(f"[{self.prefix}] X-: {self.current_x:.4f}")
            elif key == 'a' or key == '4':
                self.current_y += self.xy_step
                moved = True
                print(f"[{self.prefix}] Y+: {self.current_y:.4f}")
            elif key == 'd' or key == '6':
                self.current_y -= self.xy_step
                moved = True
                print(f"[{self.prefix}] Y-: {self.current_y:.4f}")
        
        elif key in ['q', 'e', '7', '9']:
            if key == 'q' or key == '7':
                self.target_positions["shoulder_pan"] += self.degree_step
                print(f"[{self.prefix}] shoulder_pan+: {self.target_positions['shoulder_pan']}")
            elif key == 'e' or key == '9':
                self.target_positions["shoulder_pan"] -= self.degree_step
                print(f"[{self.prefix}] shoulder_pan-: {self.target_positions['shoulder_pan']}")
        
        elif key in ['r', 'f', '/', '*']:
            if key == 'r' or key == '/':
                self.target_positions["wrist_roll"] += self.degree_step
                print(f"[{self.prefix}] wrist_roll+: {self.target_positions['wrist_roll']}")
            elif key == 'f' or key == '*':
                self.target_positions["wrist_roll"] -= self.degree_step
                print(f"[{self.prefix}] wrist_roll-: {self.target_positions['wrist_roll']}")
        
        elif key in ['t', 'g', '+', '-']:
            if key == 't' or key == '+':
                self.target_positions["gripper"] += self.degree_step
                print(f"[{self.prefix}] gripper open: {self.target_positions['gripper']}")
            elif key == 'g' or key == '-':
                self.target_positions["gripper"] -= self.degree_step
                print(f"[{self.prefix}] gripper close: {self.target_positions['gripper']}")
        
        elif key in ['z', 'x', '1', '3']:
            if key == 'z' or key == '1':
                self.pitch += self.degree_step
                print(f"[{self.prefix}] pitch+: {self.pitch}")
            elif key == 'x' or key == '3':
                self.pitch -= self.degree_step
                print(f"[{self.prefix}] pitch-: {self.pitch}")
        
        elif key in ['c', '0']:
            self.move_to_zero_position(robot)
            return
        
        if moved:
            try:
                joint2, joint3 = self.kinematics.inverse_kinematics(self.current_x, self.current_y)
                self.target_positions["shoulder_lift"] = joint2
                self.target_positions["elbow_flex"] = joint3
            except Exception as e:
                print(f"[{self.prefix}] IK failed: {e}")
        
        self.target_positions["wrist_flex"] = (
            -self.target_positions["shoulder_lift"]
            -self.target_positions["elbow_flex"]
            + self.pitch
        )

    def p_control_action(self, robot):
        obs = robot.get_observation()
        current = {j: obs[f"{self.prefix}_arm_{j}.pos"] for j in self.joint_map}
        action = {}
        for j in self.target_positions:
            error = self.target_positions[j] - current[j]
            control = self.kp * error
            action[f"{self.joint_map[j]}.pos"] = current[j] + control
        return action

def get_key():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

def print_controls():
    print("=" * 70)
    print("XLeRobot Complete Teleoperation (No Motor Check)")
    print("=" * 70)
    print("\n📍 LEFT ARM: W/S/A/D, Q/E, R/F, T/G, Z/X, C")
    print("📍 RIGHT ARM: 8/2/4/6, 7/9, /*, +-, 1/3, 0")
    print("🎯 HEAD: ,/., </>  , ?")
    print("🚗 BASE: I/K (fwd/back), J/L (strafe), U/O (rotate), H (stop)")
    print("⚙️  ESC - Exit")
    print("=" * 70)

def create_base_rotation_action(robot, direction='cw', velocity=50):
    base_motors = robot.base_motors
    action = {}
    if direction == 'cw':
        for motor in base_motors:
            action[f"{motor}.vel"] = velocity
        print(f"[BASE] Rotating clockwise")
    elif direction == 'ccw':
        for motor in base_motors:
            action[f"{motor}.vel"] = -velocity
        print(f"[BASE] Rotating counter-clockwise")
    return action

def main():
    print_controls()
    
    robot_config = XLerobotConfig()
    robot = XLerobot(robot_config)
    
    try:
        robot.connect()
        print("\n[MAIN] ✓ Successfully connected to robot")
    except Exception as e:
        print(f"\n[MAIN] ✗ Failed to connect: {e}")
        import traceback
        traceback.print_exc()
        return
    
    obs = robot.get_observation()
    kin_left = SO101Kinematics()
    kin_right = SO101Kinematics()
    
    left_arm = SimpleTeleopArm(kin_left, LEFT_JOINT_MAP, obs, prefix="left")
    right_arm = SimpleTeleopArm(kin_right, RIGHT_JOINT_MAP, obs, prefix="right")
    head_control = SimpleHeadControl(obs)
    
    print("\n[INIT] Moving arms to zero position...")
    left_arm.move_to_zero_position(robot)
    right_arm.move_to_zero_position(robot)
    
    print("\n✓ Ready! Press keys to control the robot...\n")
    
    LEFT_KEYS = set('wsadqerftgzxc')
    RIGHT_KEYS = set('824679/*+-103')
    HEAD_KEYS = set(',.<>?')
    BASE_KEYS = set('ijklhuo')
    
    try:
        while True:
            key = get_key()
            
            if key == '\x1b':
                print("\nExiting...")
                break
            
            key_lower = key.lower()
            
            if key_lower in LEFT_KEYS:
                left_arm.handle_key(key_lower, robot)
            elif key in RIGHT_KEYS:
                right_arm.handle_key(key, robot)
            elif key in HEAD_KEYS:
                head_control.handle_key(key, robot)
            elif key_lower in BASE_KEYS:
                base_action = {}
                if key_lower == 'i':
                    print("[BASE] Move forward")
                    keyboard_keys = np.array(['i'])
                    base_action = robot._from_keyboard_to_base_action(keyboard_keys) or {}
                elif key_lower == 'k':
                    print("[BASE] Move backward")
                    keyboard_keys = np.array(['k'])
                    base_action = robot._from_keyboard_to_base_action(keyboard_keys) or {}
                elif key_lower == 'j':
                    print("[BASE] Strafe left")
                    keyboard_keys = np.array(['j'])
                    base_action = robot._from_keyboard_to_base_action(keyboard_keys) or {}
                elif key_lower == 'l':
                    print("[BASE] Strafe right")
                    keyboard_keys = np.array(['l'])
                    base_action = robot._from_keyboard_to_base_action(keyboard_keys) or {}
                elif key_lower == 'u':
                    base_action = create_base_rotation_action(robot, 'ccw', velocity=50)
                elif key_lower == 'o':
                    base_action = create_base_rotation_action(robot, 'cw', velocity=50)
                elif key_lower == 'h':
                    print("[BASE] Stop")
                    try:
                        robot.stop_base()
                    except:
                        pass
                    base_action = {}
            else:
                base_action = {}
            
            left_action = left_arm.p_control_action(robot)
            right_action = right_arm.p_control_action(robot)
            head_action = head_control.p_control_action(robot)
            
            action = {**left_action, **right_action, **head_action, **base_action}
            robot.send_action(action)
            
    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
    except Exception as e:
        print(f"\n\nError: {e}")
        import traceback
        traceback.print_exc()
    finally:
        print("\n[SHUTDOWN] Disconnecting robot...")
        try:
            robot.disconnect()
        except:
            pass
        print("✓ Teleoperation ended.")

if __name__ == "__main__":
    main()
