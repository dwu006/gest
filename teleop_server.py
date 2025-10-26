"""
XLeRobot Wireless Teleoperation - SERVER SIDE (Robot RPi)
Receives commands over network and controls follower robot
"""

import sys
import time
import socket
import json
import threading
import numpy as np
from lerobot.robots.xlerobot import XLerobot, XLerobotConfig

# Joint maps for follower arms
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

class HeadController:
    def __init__(self, initial_obs, kp=0.81):
        self.kp = kp
        self.degree_step = 1
        self.target_positions = {
            "head_motor_1": initial_obs.get("head_motor_1.pos", 0.0),
            "head_motor_2": initial_obs.get("head_motor_2.pos", 0.0),
        }
        self.zero_pos = {"head_motor_1": 0.0, "head_motor_2": 0.0}

    def handle_key(self, key):
        """Update head targets based on keyboard command"""
        if key == ',':  # head_motor_1+
            self.target_positions["head_motor_1"] += self.degree_step
            print(f"[HEAD] motor_1+: {self.target_positions['head_motor_1']}")
        elif key == '.':  # head_motor_1-
            self.target_positions["head_motor_1"] -= self.degree_step
            print(f"[HEAD] motor_1-: {self.target_positions['head_motor_1']}")
        elif key == '<':  # head_motor_2+
            self.target_positions["head_motor_2"] += self.degree_step
            print(f"[HEAD] motor_2+: {self.target_positions['head_motor_2']}")
        elif key == '>':  # head_motor_2-
            self.target_positions["head_motor_2"] -= self.degree_step
            print(f"[HEAD] motor_2-: {self.target_positions['head_motor_2']}")
        elif key == '?':  # reset head
            self.target_positions = self.zero_pos.copy()
            print("[HEAD] Reset to zero")

    def get_action(self, robot):
        """Generate P-control action for head"""
        obs = robot.get_observation()
        action = {}
        for motor in self.target_positions:
            current = obs.get(f"{HEAD_MOTOR_MAP[motor]}.pos", 0.0)
            error = self.target_positions[motor] - current
            control = self.kp * error
            action[f"{HEAD_MOTOR_MAP[motor]}.pos"] = current + control
        return action

class TeleopServer:
    def __init__(self, port=5555):
        self.port = port
        self.server_sock = None
        self.client_sock = None
        self.running = False
        
        # Follower robot
        self.robot = None
        self.head_controller = None
        
        # Command buffer
        self.latest_command = None
        self.lock = threading.Lock()
        
        # Smoothing for arms
        self.alpha = 0.3
        self.prev_left_targets = None
        self.prev_right_targets = None
        
    def connect_robot(self):
        """Connect to follower robot"""
        print("[SERVER] Connecting to follower robot...")
        robot_config = XLerobotConfig()
        self.robot = XLerobot(robot_config)
        self.robot.connect()
        
        # Initialize head controller
        obs = self.robot.get_observation()
        self.head_controller = HeadController(obs)
        
        print("[SERVER] ✓ Follower robot connected")
        
    def start_server(self):
        """Start TCP server"""
        print(f"[SERVER] Starting server on port {self.port}...")
        self.server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_sock.bind(('0.0.0.0', self.port))
        self.server_sock.listen(1)
        print(f"[SERVER] ✓ Listening on 0.0.0.0:{self.port}")
        
    def wait_for_client(self):
        """Wait for client connection"""
        print("[SERVER] Waiting for client connection...")
        self.client_sock, client_addr = self.server_sock.accept()
        self.client_sock.settimeout(1.0)
        print(f"[SERVER] ✓ Client connected from {client_addr}")
        
    def command_receiver_thread(self):
        """Thread that receives commands from client"""
        print("[SERVER] Command receiver thread started")
        buffer = ""
        
        while self.running:
            try:
                data = self.client_sock.recv(4096).decode('utf-8')
                if not data:
                    print("[SERVER] Client disconnected")
                    self.running = False
                    break
                
                buffer += data
                
                # Process complete JSON messages (newline-delimited)
                while '\n' in buffer:
                    line, buffer = buffer.split('\n', 1)
                    if line.strip():
                        try:
                            command = json.loads(line)
                            with self.lock:
                                self.latest_command = command
                        except json.JSONDecodeError as e:
                            print(f"[SERVER] JSON decode error: {e}")
                            
            except socket.timeout:
                continue
            except Exception as e:
                print(f"[SERVER] Receiver error: {e}")
                self.running = False
                break
                
    def create_arm_action(self, left_positions, right_positions):
        """Convert arm positions to action dict with smoothing"""
        action = {}
        
        # Left arm
        if left_positions:
            for joint_name, motor_name in LEFT_JOINT_MAP.items():
                target = left_positions[joint_name]
                
                # Apply smoothing
                if self.prev_left_targets and joint_name in self.prev_left_targets:
                    target = (self.alpha * target + 
                             (1 - self.alpha) * self.prev_left_targets[joint_name])
                
                action[f"{motor_name}.pos"] = target
                
            # Store for next iteration
            if not self.prev_left_targets:
                self.prev_left_targets = {}
            for joint_name in left_positions:
                self.prev_left_targets[joint_name] = action[f"{LEFT_JOINT_MAP[joint_name]}.pos"]
        
        # Right arm
        if right_positions:
            for joint_name, motor_name in RIGHT_JOINT_MAP.items():
                target = right_positions[joint_name]
                
                # Apply smoothing
                if self.prev_right_targets and joint_name in self.prev_right_targets:
                    target = (self.alpha * target + 
                             (1 - self.alpha) * self.prev_right_targets[joint_name])
                
                action[f"{motor_name}.pos"] = target
                
            # Store for next iteration
            if not self.prev_right_targets:
                self.prev_right_targets = {}
            for joint_name in right_positions:
                self.prev_right_targets[joint_name] = action[f"{RIGHT_JOINT_MAP[joint_name]}.pos"]
        
        return action
    
    def handle_keyboard_command(self, key):
        """Process keyboard command for base or head"""
        base_action = {}
        
        # Head control
        HEAD_KEYS = set(',.<>?')
        if key in HEAD_KEYS:
            self.head_controller.handle_key(key)
            return {}
        
        # Base control
        key_lower = key.lower()
        
        if key_lower == 'i':  # Forward
            print("[BASE] Forward")
            keyboard_keys = np.array(['i'])
            base_action = self.robot._from_keyboard_to_base_action(keyboard_keys) or {}
        elif key_lower == 'k':  # Backward
            print("[BASE] Backward")
            keyboard_keys = np.array(['k'])
            base_action = self.robot._from_keyboard_to_base_action(keyboard_keys) or {}
        elif key_lower == 'j':  # Turn left
            print("[BASE] Turn left")
            keyboard_keys = np.array(['j'])
            base_action = self.robot._from_keyboard_to_base_action(keyboard_keys) or {}
        elif key_lower == 'l':  # Turn right
            print("[BASE] Turn right")
            keyboard_keys = np.array(['l'])
            base_action = self.robot._from_keyboard_to_base_action(keyboard_keys) or {}
        elif key_lower == 'u':  # Rotate left
            print("[BASE] Rotate left")
            keyboard_keys = np.array(['u'])
            base_action = self.robot._from_keyboard_to_base_action(keyboard_keys) or {}
        elif key_lower == 'o':  # Rotate right
            print("[BASE] Rotate right")
            keyboard_keys = np.array(['o'])
            base_action = self.robot._from_keyboard_to_base_action(keyboard_keys) or {}
        elif key_lower == 'h':  # Stop
            print("[BASE] Stop")
            try:
                self.robot.stop_base()
            except:
                pass
            base_action = {}
        
        return base_action
    
    def control_loop_thread(self):
        """Thread that controls the robot"""
        print("[SERVER] Control loop thread started")
        rate = 50  # Hz
        dt = 1.0 / rate
        iteration = 0
        
        while self.running:
            try:
                loop_start = time.time()
                
                # Get latest command
                with self.lock:
                    command = self.latest_command
                
                # Process command
                arm_action = {}
                base_action = {}
                
                if command:
                    # Handle arm positions
                    left_pos = command.get("left_arm")
                    right_pos = command.get("right_arm")
                    arm_action = self.create_arm_action(left_pos, right_pos)
                    
                    # Handle keyboard command
                    kb_cmd = command.get("keyboard")
                    if kb_cmd:
                        base_action = self.handle_keyboard_command(kb_cmd)
                
                # Get head action
                head_action = self.head_controller.get_action(self.robot)
                
                # Combine and send
                final_action = {**arm_action, **head_action, **base_action}
                if final_action:
                    self.robot.send_action(final_action)
                
                # Status print every 2 seconds
                if iteration % (rate * 2) == 0 and command:
                    print(f"[SERVER] Active - receiving commands")
                
                iteration += 1
                
                # Maintain rate
                elapsed = time.time() - loop_start
                if elapsed < dt:
                    time.sleep(dt - elapsed)
                    
            except Exception as e:
                print(f"[SERVER] Control loop error: {e}")
                time.sleep(0.1)
                
    def run(self):
        """Main server loop"""
        print("\n" + "="*70)
        print("🤖 XLeRobot Wireless Teleoperation - SERVER")
        print("="*70)
        
        try:
            # Connect to robot
            self.connect_robot()
            
            # Start server
            self.start_server()
            
            # Wait for client
            self.wait_for_client()
            
            print("\n[SERVER] Starting control threads...")
            self.running = True
            
            # Start threads
            receiver_thread = threading.Thread(target=self.command_receiver_thread, daemon=True)
            control_thread = threading.Thread(target=self.control_loop_thread, daemon=True)
            
            receiver_thread.start()
            control_thread.start()
            
            print("\n" + "="*70)
            print("✓ SERVER READY - Robot under control")
            print("="*70 + "\n")
            
            # Wait for threads
            while self.running:
                time.sleep(0.1)
                
        except KeyboardInterrupt:
            print("\n[SERVER] Interrupted by user")
        except Exception as e:
            print(f"\n[SERVER] Error: {e}")
            import traceback
            traceback.print_exc()
        finally:
            self.running = False
            print("\n[SERVER] Shutting down...")
            
            if self.robot:
                try:
                    self.robot.disconnect()
                    print("[SERVER] ✓ Robot disconnected")
                except:
                    pass
                    
            if self.client_sock:
                try:
                    self.client_sock.close()
                except:
                    pass
                    
            if self.server_sock:
                try:
                    self.server_sock.close()
                except:
                    pass
                    
            print("[SERVER] ✓ Server stopped")

def main():
    port = int(sys.argv[1]) if len(sys.argv) > 1 else 5555
    
    server = TeleopServer(port)
    server.run()

if __name__ == "__main__":
    main()