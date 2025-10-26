import sys
import time
import numpy as np
from lerobot.robots.xlerobot import XLerobot, XLerobotConfig

# Joint maps for follower
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

class LeaderFollowerTeleop:
    def __init__(self, leader_robot, follower_robot, mode="direct"):
        """
        mode: "direct" - copy positions directly
              "velocity" - use velocity control with smoothing
        """
        self.leader = leader_robot
        self.follower = follower_robot
        self.mode = mode
        self.running = False
        
        # For velocity mode
        self.alpha = 0.3  # smoothing factor
        self.prev_follower_targets = None
        
    def get_leader_positions(self):
        """Read current leader arm positions"""
        obs = self.leader.get_observation()
        
        left_positions = {
            "shoulder_pan": obs.get("left_arm_shoulder_pan.pos", 0.0),
            "shoulder_lift": obs.get("left_arm_shoulder_lift.pos", 0.0),
            "elbow_flex": obs.get("left_arm_elbow_flex.pos", 0.0),
            "wrist_flex": obs.get("left_arm_wrist_flex.pos", 0.0),
            "wrist_roll": obs.get("left_arm_wrist_roll.pos", 0.0),
            "gripper": obs.get("left_arm_gripper.pos", 0.0),
        }
        
        right_positions = {
            "shoulder_pan": obs.get("right_arm_shoulder_pan.pos", 0.0),
            "shoulder_lift": obs.get("right_arm_shoulder_lift.pos", 0.0),
            "elbow_flex": obs.get("right_arm_elbow_flex.pos", 0.0),
            "wrist_flex": obs.get("right_arm_wrist_flex.pos", 0.0),
            "wrist_roll": obs.get("right_arm_wrist_roll.pos", 0.0),
            "gripper": obs.get("right_arm_gripper.pos", 0.0),
        }
        
        return left_positions, right_positions
    
    def create_follower_action(self, left_positions, right_positions):
        """Convert leader positions to follower action dict"""
        action = {}
        
        # Left arm
        for joint_name, motor_name in LEFT_JOINT_MAP.items():
            action[f"{motor_name}.pos"] = left_positions[joint_name]
        
        # Right arm
        for joint_name, motor_name in RIGHT_JOINT_MAP.items():
            action[f"{motor_name}.pos"] = right_positions[joint_name]
        
        return action
    
    def smooth_action(self, new_action):
        """Apply exponential smoothing to reduce jitter"""
        if self.prev_follower_targets is None:
            self.prev_follower_targets = new_action
            return new_action
        
        smoothed = {}
        for key in new_action:
            smoothed[key] = (self.alpha * new_action[key] + 
                           (1 - self.alpha) * self.prev_follower_targets[key])
        
        self.prev_follower_targets = smoothed
        return smoothed
    
    def run(self, hz=50, duration=None):
        """
        Run leader-follower control loop
        hz: control frequency
        duration: run for specified seconds (None = infinite)
        """
        self.running = True
        dt = 1.0 / hz
        start_time = time.time()
        iteration = 0
        
        print("\n" + "="*70)
        print("🤖 Leader-Follower Teleoperation Active")
        print("="*70)
        print(f"Mode: {self.mode}")
        print(f"Control Frequency: {hz} Hz")
        print("Press Ctrl+C to stop")
        print("="*70 + "\n")
        
        try:
            while self.running:
                loop_start = time.time()
                
                # Read leader positions
                left_pos, right_pos = self.get_leader_positions()
                
                # Create follower action
                action = self.create_follower_action(left_pos, right_pos)
                
                # Apply smoothing if in velocity mode
                if self.mode == "velocity":
                    action = self.smooth_action(action)
                
                # Send to follower
                self.follower.send_action(action)
                
                # Print status every second
                if iteration % hz == 0:
                    print(f"[{time.time()-start_time:.1f}s] Left shoulder_pan: {left_pos['shoulder_pan']:.2f}°, "
                          f"Right shoulder_pan: {right_pos['shoulder_pan']:.2f}°")
                
                iteration += 1
                
                # Check duration
                if duration and (time.time() - start_time) >= duration:
                    print(f"\n✓ Completed {duration}s teleoperation")
                    break
                
                # Maintain loop rate
                elapsed = time.time() - loop_start
                if elapsed < dt:
                    time.sleep(dt - elapsed)
                
        except KeyboardInterrupt:
            print("\n\n⚠️  Interrupted by user")
        except Exception as e:
            print(f"\n\n❌ Error: {e}")
            import traceback
            traceback.print_exc()
        finally:
            self.running = False
            print("\n✓ Leader-follower control stopped")

def main():
    print("\n" + "="*70)
    print("XLeRobot Leader-Follower Teleoperation Setup")
    print("="*70)
    
    # Initialize leader robot
    print("\n[1/4] Connecting to LEADER robot...")
    leader_config = XLerobotConfig()
    # If your leader has different connection settings, modify here:
    # leader_config.port = "/dev/ttyUSB0" or whatever
    leader_robot = XLerobot(leader_config)
    
    try:
        leader_robot.connect()
        print("     ✓ Leader connected")
    except Exception as e:
        print(f"     ✗ Failed to connect to leader: {e}")
        return
    
    # Initialize follower robot
    print("\n[2/4] Connecting to FOLLOWER robot...")
    follower_config = XLerobotConfig()
    # Modify follower connection settings if needed:
    # follower_config.port = "/dev/ttyUSB1" or different IP/port
    follower_robot = XLerobot(follower_config)
    
    try:
        follower_robot.connect()
        print("     ✓ Follower connected")
    except Exception as e:
        print(f"     ✗ Failed to connect to follower: {e}")
        leader_robot.disconnect()
        return
    
    # Create teleoperation controller
    print("\n[3/4] Initializing teleoperation controller...")
    teleop = LeaderFollowerTeleop(
        leader_robot=leader_robot,
        follower_robot=follower_robot,
        mode="direct"  # or "velocity" for smoother motion
    )
    print("     ✓ Controller ready")
    
    # Start teleoperation
    print("\n[4/4] Starting teleoperation...")
    print("\n💡 TIP: Move the leader arms - follower will mirror movements")
    print("     Left leader arm → Left follower arm")
    print("     Right leader arm → Right follower arm\n")
    
    try:
        teleop.run(hz=50)  # 50 Hz control loop
    finally:
        print("\n[SHUTDOWN] Disconnecting robots...")
        try:
            leader_robot.disconnect()
            print("✓ Leader disconnected")
        except:
            pass
        try:
            follower_robot.disconnect()
            print("✓ Follower disconnected")
        except:
            pass
        print("\n✓ Teleoperation ended")

if __name__ == "__main__":
    main()
