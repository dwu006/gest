# Copyright 2024 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from dataclasses import dataclass, field

from lerobot.cameras.configs import CameraConfig, Cv2Rotation, ColorMode
from lerobot.cameras.opencv.configuration_opencv import OpenCVCameraConfig
from lerobot.cameras.realsense import RealSenseCamera, RealSenseCameraConfig

from ..config import RobotConfig

########################################################
# Code for automatically assigning ports to the robot
import sys
import os
import glob
import argparse
from lerobot.motors.feetech.feetech import FeetechMotorsBus



def get_motor_count(port: str, protocol_version: int = 0) -> dict:
    """
    Get the number of Feetech motors connected to a given port and their IDs.
    
    Args:
        port (str): The port to check (e.g., '/dev/ttyACM0')
        protocol_version (int): Protocol version to use (0 or 1). Default is 0.
    
    Returns:
        dict: A dictionary with keys:
            - 'n_motors' (int): The number of motors found
            - 'motor_ids' (list): List of motor IDs found
            Returns {'n_motors': 0, 'motor_ids': []} if no motors found or on error.
    """
    try:
        # Create an instance of FeetechMotorsBus with an empty motors dict
        feetech_bus = FeetechMotorsBus(port=port, motors={}, protocol_version=protocol_version)
        
        # Open the port
        if not feetech_bus.port_handler.openPort():
            return {'n_motors': 0, 'motor_ids': []}
        
        # Set baudrate
        if not feetech_bus.port_handler.setBaudRate(feetech_bus.default_baudrate):
            feetech_bus.port_handler.closePort()
            return {'n_motors': 0, 'motor_ids': []}
        
        # Scan for motors based on protocol version
        if protocol_version == 0:
            # Protocol 0 supports broadcast ping
            motor_ids = feetech_bus.broadcast_ping()
        else:
            # Protocol 1 requires sequential ping
            import scservo_sdk as scs
            motor_ids = {}
            for id_ in range(scs.MAX_ID + 1):
                model = feetech_bus.ping(id_)
                if model is not None:
                    motor_ids[id_] = model
        
        # Close the port
        feetech_bus.port_handler.closePort()
        
        # Return the result as a dictionary
        if motor_ids:
            ids_list = list(motor_ids.keys())
            return {'n_motors': len(ids_list), 'motor_ids': ids_list}
        else:
            return {'n_motors': 0, 'motor_ids': []}
        
    except Exception as e:
        # Silently handle errors and return empty result
        return {'n_motors': 0, 'motor_ids': []}



def find_port_by_motors(n_motors: int, protocol_version: int = 0, verbose: bool = False) -> str | None:
    """
    Find a port that has exactly the specified number of motors with sequential IDs.
    
    Args:
        n_motors (int): The number of motors to look for (e.g., 8 or 9)
        protocol_version (int): Protocol version to use (0 or 1). Default is 0.
        verbose (bool): If True, print progress information. Default is False.
    
    Returns:
        str | None: The port path (e.g., '/dev/ttyACM0') if found, None otherwise
    """
    # Find all /dev/ttyACM* ports
    ports = glob.glob('/dev/ttyACM*')
    ports.sort()  # Sort for consistent ordering
    
    if verbose:
        print(f"Searching for port with {n_motors} motors...")
        print(f"Found {len(ports)} port(s) to check: {ports}")
        print()
    
    # Expected motor IDs: [1, 2, 3, ..., n_motors]
    expected_motor_ids = list(range(1, n_motors + 1))
    
    # Check each port
    for port in ports:
        if verbose:
            print(f"Checking {port}...")
        
        try:
            result = get_motor_count(port, protocol_version)
            found_n_motors = result['n_motors']
            motor_ids = result['motor_ids']
            
            if verbose:
                print(f"  Found {found_n_motors} motor(s): {motor_ids}")
            
            # Check if this port has the exact configuration we're looking for
            if motor_ids == expected_motor_ids:
                if verbose:
                    print(f"  ✓ Match found! Port has {n_motors} motors with IDs {motor_ids}")
                    print()
                return port
            else:
                if verbose:
                    print(f"  ✗ Not a match")
        
        except Exception as e:
            if verbose:
                print(f"  Error checking port: {e}")
        
        if verbose:
            print()
    
    # No matching port found
    if verbose:
        print(f"No port found with {n_motors} motors and IDs {expected_motor_ids}")
    
    return None


########################################################



def xlerobot_cameras_config() -> dict[str, CameraConfig]:
    return {
        #"left_wrist": OpenCVCameraConfig(
        #    index_or_path="/dev/video4", fps=30, width=640, height=480, rotation=Cv2Rotation.NO_ROTATION
        #),

        #"right_wrist": OpenCVCameraConfig(
        #     index_or_path="/dev/video0", fps=30, width=640, height=480, rotation=Cv2Rotation.NO_ROTATION
        #),  

       # "head(RGDB)": OpenCVCameraConfig(
        #     index_or_path="/dev/v4l/by-id/usb-Intel_R__RealSense_TM__Depth_Camera_455_Intel_R__RealSense_TM__Depth_Camera_455-video-index0", fps=30, width=640, height=480, rotation=Cv2Rotation.NO_ROTATION
        # ),                     
        
        # "head": RealSenseCameraConfig(
        #     serial_number_or_name="125322060037",  # Replace with camera SN
        #     fps=30,
        #     width=1280,
        #     height=720,
        #     color_mode=ColorMode.BGR, # Request BGR output
        #     rotation=Cv2Rotation.NO_ROTATION,
        #     use_depth=True
        # ),
    }


@RobotConfig.register_subclass("xlerobot")
@dataclass
class XLerobotConfig(RobotConfig):
    
    port1: str = "/dev/ttyACM0"  # port to connect to the bus (so101 + head camera)
    port2: str = "/dev/ttyACM1"  # port to connect to the bus (same as lekiwi setup)
    auto_detect_ports: bool = True # If True, will automatically detect the ports for the robot
    disable_torque_on_disconnect: bool = True

    # `max_relative_target` limits the magnitude of the relative positional target vector for safety purposes.
    # Set this to a positive scalar to have the same value for all motors, or a list that is the same length as
    # the number of motors in your follower arms.
    max_relative_target: int | None = None

    cameras: dict[str, CameraConfig] = field(default_factory=xlerobot_cameras_config)

    # Set to `True` for backward compatibility with previous policies/dataset
    use_degrees: bool = False

    teleop_keys: dict[str, str] = field(
        default_factory=lambda: {
            # Movement
            "forward": "i",
            "backward": "k",
            "left": "j",
            "right": "l",
            "rotate_left": "u",
            "rotate_right": "o",
            # Speed control
            "speed_up": "n",
            "speed_down": "m",
            # quit teleop
            "quit": "b",
        }
    )

########################################################


    #######################
    # Auto-detect ports if enabled
    #######################
    def __post_init__(self):
        # Call parent __post_init__ first
        super().__post_init__()
        
        # Auto-detect ports if enabled
        if self.auto_detect_ports:
            detected_port1 = find_port_by_motors(8, protocol_version=0, verbose=False)
            detected_port2 = find_port_by_motors(9, protocol_version=0, verbose=False)
            
            if detected_port1 is not None:
                self.port1 = detected_port1
            else:
                print(f"Warning: Could not auto-detect port1 (8 motors). Using default: {self.port1}")
            
            if detected_port2 is not None:
                self.port2 = detected_port2
            else:
                print(f"Warning: Could not auto-detect port2 (9 motors). Using default: {self.port2}")
            
            print(f"Port assignment: port1={self.port1}, port2={self.port2}")

########################################################


@dataclass
class XLerobotHostConfig:
    # Network Configuration
    port_zmq_cmd: int = 5555
    port_zmq_observations: int = 5556

    # Duration of the application
    connection_time_s: int = 3600

    # Watchdog: stop the robot if no command is received for over 0.5 seconds.
    watchdog_timeout_ms: int = 500

    # If robot jitters decrease the frequency and monitor cpu load with `top` in cmd
    max_loop_freq_hz: int = 30

@RobotConfig.register_subclass("xlerobot_client")
@dataclass
class XLerobotClientConfig(RobotConfig):
    # Network Configuration
    remote_ip: str
    port_zmq_cmd: int = 5555
    port_zmq_observations: int = 5556

    teleop_keys: dict[str, str] = field(
        default_factory=lambda: {
            # Movement
            "forward": "i",
            "backward": "k",
            "left": "j",
            "right": "l",
            "rotate_left": "u",
            "rotate_right": "o",
            # Speed control
            "speed_up": "n",
            "speed_down": "m",
            # quit teleop
            "quit": "b",
        }
    )

    cameras: dict[str, CameraConfig] = field(default_factory=xlerobot_cameras_config)

    polling_timeout_ms: int = 15
    connect_timeout_s: int = 5
