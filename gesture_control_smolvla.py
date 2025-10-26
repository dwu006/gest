#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
LeRobot Gesture Control with SmoLVLA
Real-time hand gesture recognition integrated with SmoLVLA VLA model
"""

import sys
import csv
import copy
import argparse
from pathlib import Path

import cv2 as cv
import numpy as np
import mediapipe as mp
import pyrealsense2 as rs
import torch

# Add paths for gesture recognition modules
sys.path.insert(0, str(Path(__file__).parent))
sys.path.insert(0, str(Path(__file__).parent / "src"))

from utils import CvFpsCalc
from model import KeyPointClassifier
from lerobot.gestures import GestureToVLABridge, SimpleVisualizer


def load_smolvla_policy(policy_path, device="cuda"):
    """
    Load SmoLVLA policy for inference

    Args:
        policy_path: Path to fine-tuned SmoLVLA model or 'lerobot/smolvla_base'
        device: Device to run on ('cuda' or 'cpu')

    Returns:
        policy: Loaded SmoLVLA policy
    """
    try:
        from lerobot.common.policies.factory import make_policy

        print(f"Loading SmoLVLA policy from: {policy_path}")

        # Load the policy
        policy = make_policy(
            hydra_cfg=None,  # Will be loaded from checkpoint
            pretrained=policy_path,
            device=device
        )

        policy.eval()  # Set to evaluation mode
        print("SmoLVLA policy loaded successfully!")

        return policy

    except ImportError as e:
        print(f"Error: Could not import LeRobot policy factory")
        print(f"Make sure LeRobot is installed: pip install lerobot")
        print(f"Details: {e}")
        return None
    except Exception as e:
        print(f"Error loading SmoLVLA policy: {e}")
        print("\nTips:")
        print("1. For base model: --policy_path lerobot/smolvla_base")
        print("2. For fine-tuned: --policy_path YOUR_HF_USER/your_model")
        print("3. Make sure model is downloaded or you're logged into HuggingFace")
        return None


def get_args():
    parser = argparse.ArgumentParser(description="LeRobot Gesture Control with SmoLVLA")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--table_distance", type=float, default=0.8,
                        help="Table distance in meters")
    parser.add_argument("--policy_path", type=str, default="lerobot/smolvla_base",
                        help="Path to SmoLVLA model (HuggingFace repo or local)")
    parser.add_argument("--robot", type=str, default=None,
                        help="Robot configuration name")
    parser.add_argument("--output_dir", type=str, default="lerobot_data",
                        help="Output directory for episode data")
    parser.add_argument("--stability_threshold", type=int, default=5,
                        help="Frames gesture must be stable before triggering")
    parser.add_argument("--device", type=str, default="cuda" if torch.cuda.is_available() else "cpu",
                        help="Device for inference (cuda/cpu)")
    parser.add_argument("--no_policy", action="store_true",
                        help="Run without loading policy (logging only)")
    args = parser.parse_args()
    return args


def calc_landmark_list(image, landmarks):
    """Extract 2D landmark coordinates"""
    image_width, image_height = image.shape[1], image.shape[0]
    landmark_point = []

    for landmark in landmarks.landmark:
        landmark_x = min(int(landmark.x * image_width), image_width - 1)
        landmark_y = min(int(landmark.y * image_height), image_height - 1)
        landmark_point.append([landmark_x, landmark_y])

    return landmark_point


def pre_process_landmark(landmark_list):
    """Normalize landmarks for classifier"""
    temp_landmark_list = copy.deepcopy(landmark_list)

    # Relative coordinates
    base_x, base_y = temp_landmark_list[0]
    for i in range(len(temp_landmark_list)):
        temp_landmark_list[i][0] -= base_x
        temp_landmark_list[i][1] -= base_y

    # Flatten
    temp_landmark_list = [coord for point in temp_landmark_list for coord in point]

    # Normalize
    max_value = max(list(map(abs, temp_landmark_list)))
    temp_landmark_list = [n / max_value for n in temp_landmark_list]

    return temp_landmark_list


def calculate_pointing_vector(landmark_list, depth_frame, depth_scale, intrinsics):
    """Calculate 3D pointing ray from finger"""
    if len(landmark_list) < 9:
        return None, None

    finger_base_2d = landmark_list[5]
    finger_tip_2d = landmark_list[8]

    depth_base = depth_frame.get_distance(finger_base_2d[0], finger_base_2d[1])
    depth_tip = depth_frame.get_distance(finger_tip_2d[0], finger_tip_2d[1])

    if depth_base == 0 or depth_tip == 0:
        return None, None

    point_base_3d = rs.rs2_deproject_pixel_to_point(intrinsics, finger_base_2d, depth_base)
    point_tip_3d = rs.rs2_deproject_pixel_to_point(intrinsics, finger_tip_2d, depth_tip)

    ray_origin = np.array(point_tip_3d)
    ray_direction = ray_origin - np.array(point_base_3d)

    magnitude = np.linalg.norm(ray_direction)
    if magnitude < 0.001:
        return None, None

    ray_direction = ray_direction / magnitude
    return ray_origin, ray_direction


def ray_plane_intersection(ray_origin, ray_direction, plane_normal, plane_point):
    """Find where ray intersects plane"""
    denom = np.dot(ray_direction, plane_normal)

    if abs(denom) < 1e-6:
        return None

    t = np.dot(plane_point - ray_origin, plane_normal) / denom

    if t < 0:
        return None

    intersection = ray_origin + t * ray_direction
    return intersection


def main():
    args = get_args()
    table_distance = args.table_distance

    # Load SmoLVLA policy if not disabled
    policy = None
    if not args.no_policy:
        policy = load_smolvla_policy(args.policy_path, args.device)
        if policy is None:
            print("\nContinuing without policy (logging mode only)")
            print("Use --no_policy flag to suppress this warning\n")

    # Initialize RealSense
    print("Initializing RealSense camera...")
    pipeline = rs.pipeline()
    config = rs.config()

    try:
        config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
        profile = pipeline.start(config)
    except RuntimeError as e:
        print(f"Error starting camera: {e}")
        return

    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()
    depth_intrinsics = profile.get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()
    align = rs.align(rs.stream.color)

    print(f"RealSense initialized. Depth scale: {depth_scale}")

    # Initialize MediaPipe
    mp_hands = mp.solutions.hands
    hands = mp_hands.Hands(
        static_image_mode=False,
        max_num_hands=1,
        min_detection_confidence=0.7,
        min_tracking_confidence=0.5
    )

    # Load gesture classifier
    keypoint_classifier = KeyPointClassifier()
    with open('model/keypoint_classifier/keypoint_classifier_label.csv', encoding='utf-8-sig') as f:
        keypoint_classifier_labels = [row[0] for row in csv.reader(f)]

    # Initialize robot (if specified)
    robot = None
    if args.robot:
        try:
            from lerobot import make_robot
            robot = make_robot(args.robot)
            print(f"Robot '{args.robot}' initialized")
        except Exception as e:
            print(f"Could not initialize robot: {e}")

    # Initialize VLA bridge with SmoLVLA
    vla_bridge = GestureToVLABridge(
        output_dir=args.output_dir,
        robot=robot,
        policy=policy,
        device=args.device
    )

    # FPS
    cvFpsCalc = CvFpsCalc(buffer_len=10)

    # Table plane
    table_plane_normal = np.array([0.0, 0.0, 1.0])
    table_plane_point = np.array([0.0, 0.0, table_distance])

    # Gesture tracking
    last_gesture_id = None
    gesture_stable_frames = 0
    STABILITY_THRESHOLD = args.stability_threshold

    print("\n" + "="*70)
    print("LeRobot Gesture Control with SmoLVLA")
    print("="*70)
    print("Gestures:")
    print("  OPEN PALM  -> STOP (emergency halt)")
    print("  POINTER    -> GOTO (navigate to location)")
    print("  FIST       -> PICK (grasp object)")
    print(f"\nSmoLVLA Policy: {'ACTIVE' if policy else 'INACTIVE (logging only)'}")
    print(f"Device: {args.device}")
    print(f"Stability threshold: {STABILITY_THRESHOLD} frames")
    print("\nControls:")
    print("  ESC: Exit and save episode")
    print("="*70 + "\n")

    try:
        while True:
            fps = cvFpsCalc.get()

            # Get frames
            frames = pipeline.wait_for_frames()
            aligned_frames = align.process(frames)
            depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()

            if not depth_frame or not color_frame:
                continue

            image = np.asanyarray(color_frame.get_data())
            depth_image = np.asanyarray(depth_frame.get_data())

            # Flip for mirror display
            image = cv.flip(image, 1)
            depth_image = cv.flip(depth_image, 1)
            debug_image = copy.deepcopy(image)

            # Check keys
            key = cv.waitKey(10)
            if key == 27:  # ESC
                break

            # MediaPipe hand detection
            image_rgb = cv.cvtColor(image, cv.COLOR_BGR2RGB)
            results = hands.process(image_rgb)

            aoi_coords_3d = None
            gesture_id = None
            gesture_name = None

            if results.multi_hand_landmarks is not None:
                for hand_landmarks in results.multi_hand_landmarks:
                    landmark_list = calc_landmark_list(debug_image, hand_landmarks)
                    pre_processed = pre_process_landmark(landmark_list)
                    gesture_id = keypoint_classifier(pre_processed)
                    gesture_name = keypoint_classifier_labels[gesture_id]

                    # If pointing, calculate AOI
                    if gesture_id == 2:
                        try:
                            ray_origin, ray_direction = calculate_pointing_vector(
                                landmark_list, depth_frame, depth_scale, depth_intrinsics
                            )

                            if ray_origin is not None and ray_direction is not None:
                                aoi_coords_3d = ray_plane_intersection(
                                    ray_origin, ray_direction,
                                    table_plane_normal, table_plane_point
                                )
                        except:
                            aoi_coords_3d = None

                    # Draw hand landmarks
                    mp.solutions.drawing_utils.draw_landmarks(
                        debug_image,
                        hand_landmarks,
                        mp_hands.HAND_CONNECTIONS
                    )

            # Gesture stability check and VLA command
            if gesture_id is not None:
                if gesture_id == last_gesture_id:
                    gesture_stable_frames += 1
                else:
                    gesture_stable_frames = 0
                    last_gesture_id = gesture_id

                # Send to VLA when stable
                if gesture_stable_frames == STABILITY_THRESHOLD:
                    instruction_data = vla_bridge.gesture_to_instruction(
                        gesture_id, aoi_coords_3d
                    )

                    if instruction_data is not None:
                        vla_bridge.send_to_vla(
                            instruction_data, image, depth_image
                        )

                # Draw visual feedback
                SimpleVisualizer.draw_gesture_info(
                    debug_image, gesture_id, gesture_name
                )
            else:
                last_gesture_id = None
                gesture_stable_frames = 0

            # FPS display
            cv.putText(debug_image, f"FPS: {fps}", (10, debug_image.shape[0] - 40),
                      cv.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
            cv.putText(debug_image, f"Commands: {vla_bridge.command_count}",
                      (10, debug_image.shape[0] - 10),
                      cv.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)

            # Display
            cv.imshow('LeRobot Gesture Control', debug_image)

    finally:
        pipeline.stop()
        cv.destroyAllWindows()
        print(f"\nSession ended. Total commands: {vla_bridge.command_count}")
        print(f"Episode data saved to: {vla_bridge.episode_file}")


if __name__ == '__main__':
    main()
