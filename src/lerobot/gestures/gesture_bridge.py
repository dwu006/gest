"""
Gesture to VLA Bridge
Converts hand gestures to natural language instructions for LeRobot VLA models
Integrates with SmoLVLA for real-time robot control
"""

import json
from datetime import datetime
from pathlib import Path
import numpy as np
import torch


class GestureToVLABridge:
    """
    Bridge between gesture recognition and LeRobot VLA models
    Sends gesture commands as natural language instructions
    """

    # Gesture ID mapping
    STOP = 0    # Open palm
    PICK = 1    # Fist/Close
    GOTO = 2    # Pointer

    def __init__(self, output_dir="lerobot_data", robot=None, policy=None, device="cuda"):
        """
        Args:
            output_dir: Directory to save gesture commands for LeRobot
            robot: Optional LeRobot robot instance for direct control
            policy: Optional SmoLVLA policy for VLA inference
            device: Device to run inference on ('cuda' or 'cpu')
        """
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(exist_ok=True)

        self.episode_file = self.output_dir / f"episode_{datetime.now().strftime('%Y%m%d_%H%M%S')}.jsonl"
        self.command_count = 0
        self.robot = robot
        self.policy = policy
        self.device = device

        # For AOI persistence (point -> pick workflow)
        self.last_aoi = None

        print(f"\nGesture-to-VLA Bridge initialized")
        print(f"Output directory: {self.output_dir}")
        print(f"Episode file: {self.episode_file}")
        print(f"Robot integration: {'ENABLED' if robot else 'DISABLED (logging only)'}")
        print(f"Policy integration: {'ENABLED' if policy else 'DISABLED (logging only)'}")
        print(f"Device: {device}\n")

    def gesture_to_instruction(self, gesture_id, aoi_coords=None):
        """
        Convert gesture + AOI to natural language instruction for VLA

        Args:
            gesture_id: 0=STOP, 1=PICK, 2=GOTO
            aoi_coords: [x, y, z] in meters from camera (optional)

        Returns:
            dict with instruction and metadata
        """
        # Update AOI if provided
        if aoi_coords is not None:
            self.last_aoi = aoi_coords

        if gesture_id == self.STOP:
            return {
                "instruction": "Stop all motion immediately and hold your current position",
                "gesture": "STOP",
                "gesture_id": gesture_id,
                "priority": "EMERGENCY",
                "aoi": None
            }

        elif gesture_id == self.GOTO:
            if aoi_coords is None:
                return None  # Can't navigate without target

            x, y, z = aoi_coords
            return {
                "instruction": f"Navigate to the position at coordinates x={x:.2f}, y={y:.2f}, z={z:.2f} meters",
                "gesture": "GOTO",
                "gesture_id": gesture_id,
                "priority": "NORMAL",
                "aoi": {
                    "center": aoi_coords.tolist() if isinstance(aoi_coords, np.ndarray) else aoi_coords,
                    "type": "point"
                }
            }

        elif gesture_id == self.PICK:
            # Use last AOI if available (point then pick workflow)
            target = self.last_aoi if self.last_aoi is not None else aoi_coords

            if target is None:
                return {
                    "instruction": "Pick up the nearest graspable object in front of you",
                    "gesture": "PICK",
                    "gesture_id": gesture_id,
                    "priority": "NORMAL",
                    "aoi": None
                }
            else:
                x, y, z = target
                return {
                    "instruction": f"Pick up the object at position x={x:.2f}, y={y:.2f}, z={z:.2f} meters",
                    "gesture": "PICK",
                    "gesture_id": gesture_id,
                    "priority": "NORMAL",
                    "aoi": {
                        "center": target.tolist() if isinstance(target, np.ndarray) else target,
                        "type": "point"
                    }
                }

        return None

    def send_to_vla(self, instruction_data, rgb_frame, depth_frame=None):
        """
        Send instruction to VLA for execution

        Args:
            instruction_data: Dict from gesture_to_instruction
            rgb_frame: RGB image (numpy array)
            depth_frame: Optional depth map

        Returns:
            dict: VLA response or None if failed
        """
        if instruction_data is None:
            return None

        # Prepare data for LeRobot
        vla_command = {
            "timestamp": datetime.now().isoformat(),
            "command_id": self.command_count,
            "instruction": instruction_data["instruction"],
            "gesture": instruction_data["gesture"],
            "gesture_id": instruction_data["gesture_id"],
            "priority": instruction_data["priority"],
            "aoi": instruction_data.get("aoi"),
            "image_shape": rgb_frame.shape if rgb_frame is not None else None,
            "has_depth": depth_frame is not None
        }

        # Log to file
        self._log_command(vla_command)

        # Send to SmoLVLA policy if available
        if self.policy:
            try:
                # Prepare observation for SmoLVLA
                # SmoLVLA expects: visual data, robot state, natural language instruction
                observation = self._prepare_smolvla_observation(
                    rgb_frame,
                    depth_frame,
                    instruction_data["instruction"]
                )

                # Run VLA inference
                with torch.no_grad():
                    action = self.policy.select_action(observation)

                # Send action to robot if available
                if self.robot:
                    self.robot.send_action(action)

                vla_command["action_output"] = action.cpu().numpy().tolist() if torch.is_tensor(action) else action

            except Exception as e:
                print(f"VLA policy execution failed: {e}")
                return None

        # Console output
        print(f"\n{'='*70}")
        print(f"[VLA Command #{self.command_count}]")
        print(f"Gesture: {instruction_data['gesture']}")
        print(f"Priority: {instruction_data['priority']}")
        print(f"Instruction: {instruction_data['instruction']}")
        if instruction_data.get('aoi'):
            print(f"Target AOI: {instruction_data['aoi']}")
        print(f"{'='*70}\n")

        self.command_count += 1

        return {"status": "logged", "command_id": self.command_count - 1}

    def _prepare_smolvla_observation(self, rgb_frame, depth_frame, instruction):
        """
        Prepare observation dict for SmoLVLA policy

        Args:
            rgb_frame: RGB image (numpy array)
            depth_frame: Depth image (numpy array, optional)
            instruction: Natural language instruction (string)

        Returns:
            dict: Observation formatted for SmoLVLA
        """
        import torchvision.transforms as transforms

        # Convert RGB to tensor and normalize
        transform = transforms.Compose([
            transforms.ToPILImage(),
            transforms.Resize((224, 224)),  # SmoLVLA expected size
            transforms.ToTensor(),
            transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])
        ])

        image_tensor = transform(rgb_frame).unsqueeze(0).to(self.device)

        observation = {
            "observation.images.cam_high": image_tensor,  # Main camera view
            "observation.state": self._get_robot_state(),  # Robot state (if available)
            "language_instruction": instruction  # Natural language task
        }

        return observation

    def _get_robot_state(self):
        """
        Get current robot state if robot is available

        Returns:
            torch.Tensor: Robot joint positions/velocities or zero tensor
        """
        if self.robot and hasattr(self.robot, 'get_state'):
            try:
                state = self.robot.get_state()
                return torch.tensor(state, dtype=torch.float32).unsqueeze(0).to(self.device)
            except:
                pass

        # Return zero tensor if no robot state available
        # Adjust size based on your robot's DoF
        return torch.zeros((1, 7), dtype=torch.float32).to(self.device)

    def _log_command(self, command_data):
        """Log command to JSONL file for LeRobot dataset"""
        try:
            with open(self.episode_file, 'a') as f:
                f.write(json.dumps(command_data) + '\n')
        except Exception as e:
            print(f"Failed to log command: {e}")
