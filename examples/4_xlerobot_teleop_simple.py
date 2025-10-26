import sys
import tty
import termios
import time
from lerobot.robots.xlerobot import XLerobot, XLerobotConfig
from lerobot.model.SO101Robot import SO101Kinematics

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
            robot.send_action(action)
            time.sleep(0.02)

    def handle_key(self, key):
        moved = False
        
        if key == 'w':
            self.current_x += self.xy_step
            moved = True
            print(f"[{self.prefix}] X+: {self.current_x:.4f}")
        elif key == 's':
            self.current_x -= self.xy_step
            moved = True
            print(f"[{self.prefix}] X-: {self.current_x:.4f}")
        elif key == 'a':
            self.current_y += self.xy_step
            moved = True
            print(f"[{self.prefix}] Y+: {self.current_y:.4f}")
        elif key == 'd':
            self.current_y -= self.xy_step
            moved = True
            print(f"[{self.prefix}] Y-: {self.current_y:.4f}")
        elif key == 'q':
            self.target_positions["shoulder_pan"] += self.degree_step
            print(f"[{self.prefix}] shoulder_pan: {self.target_positions['shoulder_pan']}")
        elif key == 'e':
            self.target_positions["shoulder_pan"] -= self.degree_step
            print(f"[{self.prefix}] shoulder_pan: {self.target_positions['shoulder_pan']}")
        elif key == 't':
            self.target_positions["gripper"] += self.degree_step
            print(f"[{self.prefix}] gripper open: {self.target_positions['gripper']}")
        elif key == 'g':
            self.target_positions["gripper"] -= self.degree_step
            print(f"[{self.prefix}] gripper close: {self.target_positions['gripper']}")
        elif key == 'c':
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

def main():
    print("=" * 50)
    print("Simple Robot Teleoperation")
    print("=" * 50)
    print("\nLeft Arm Controls:")
    print("  W/S - Move forward/backward")
    print("  A/D - Move left/right")
    print("  Q/E - Shoulder pan +/-")
    print("  T/G - Gripper open/close")
    print("  C   - Reset to zero")
    print("  X   - Exit")
    print("=" * 50)
    
    robot_config = XLerobotConfig()
    robot = XLerobot(robot_config)
    
    try:
        robot.connect()
        print("[MAIN] Successfully connected to robot")
    except Exception as e:
        print(f"[MAIN] Failed to connect: {e}")
        return
    
    obs = robot.get_observation()
    kin_left = SO101Kinematics()
    left_arm = SimpleTeleopArm(kin_left, LEFT_JOINT_MAP, obs, prefix="left")
    
    left_arm.move_to_zero_position(robot)
    
    print("\nReady! Press keys to control the robot...")
    
    try:
        while True:
            key = get_key().lower()
            
            if key == 'x':
                print("\nExiting...")
                break
            
            if key in ['w', 's', 'a', 'd', 'q', 'e', 't', 'g', 'c']:
                left_arm.handle_key(key)
                action = left_arm.p_control_action(robot)
                robot.send_action(action)
            
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    finally:
        robot.disconnect()
        print("Teleoperation ended.")

if __name__ == "__main__":
    main()
