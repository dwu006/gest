#!/usr/bin/env python
"""
Test SmoLVLA Policy Inference with Gestures
Simulates gesture input → SmoLVLA inference pipeline
"""

import sys
import numpy as np
import torch
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
sys.path.insert(0, str(Path(__file__).parent / "src"))

from lerobot.gestures import GestureToVLABridge


def create_mock_observation():
    """Create mock RGB and depth frames for testing"""
    # Simulate 720p RGB image from RealSense
    rgb_frame = np.random.randint(0, 255, (720, 1280, 3), dtype=np.uint8)
    depth_frame = np.random.randint(0, 5000, (720, 1280), dtype=np.uint16)
    return rgb_frame, depth_frame


def test_observation_preparation():
    """Test that observations are prepared correctly for SmoLVLA"""
    print("\n" + "="*70)
    print("TEST 1: Observation Preparation for SmoLVLA")
    print("="*70)

    try:
        # Create bridge without loading actual policy
        bridge = GestureToVLABridge(
            output_dir="test_smolvla",
            policy=None,
            device="cpu"
        )

        # Create mock data
        rgb_frame, depth_frame = create_mock_observation()
        instruction = "Navigate to the position at coordinates x=0.50, y=0.30, z=0.80 meters"

        # Test observation preparation
        observation = bridge._prepare_smolvla_observation(rgb_frame, depth_frame, instruction)

        print("\n[PASS] Observation prepared successfully!")
        print(f"\nObservation keys: {list(observation.keys())}")
        print(f"Image tensor shape: {observation['observation.images.cam_high'].shape}")
        print(f"Robot state shape: {observation['observation.state'].shape}")
        print(f"Instruction: {observation['language_instruction'][:80]}...")

        # Verify shapes
        assert observation['observation.images.cam_high'].shape == torch.Size([1, 3, 224, 224]), \
            f"Image shape incorrect: {observation['observation.images.cam_high'].shape}"
        assert observation['observation.state'].shape[0] == 1, \
            f"State batch size incorrect: {observation['observation.state'].shape}"
        assert isinstance(observation['language_instruction'], str), \
            "Instruction must be string"

        print("\n[PASS] All observation shapes correct!")
        return True

    except Exception as e:
        print(f"\n[FAIL] Test failed: {e}")
        import traceback
        traceback.print_exc()
        return False


def test_gesture_to_observation_pipeline():
    """Test full pipeline: gesture -> instruction -> observation"""
    print("\n" + "="*70)
    print("TEST 2: Gesture -> Instruction -> Observation Pipeline")
    print("="*70)

    try:
        bridge = GestureToVLABridge(
            output_dir="test_smolvla",
            policy=None,
            device="cpu"
        )

        # Test all gesture types
        gestures = [
            (2, np.array([0.5, 0.3, 0.8]), "GOTO"),
            (1, np.array([0.5, 0.3, 0.8]), "PICK"),
            (0, None, "STOP")
        ]

        for gesture_id, aoi, gesture_name in gestures:
            print(f"\n--- Testing {gesture_name} gesture (ID: {gesture_id}) ---")

            # Generate instruction
            instruction_data = bridge.gesture_to_instruction(gesture_id, aoi)
            print(f"Instruction: {instruction_data['instruction'][:80]}...")

            # Create mock frames
            rgb_frame, depth_frame = create_mock_observation()

            # Prepare observation
            observation = bridge._prepare_smolvla_observation(
                rgb_frame, depth_frame, instruction_data['instruction']
            )

            print(f"[PASS] {gesture_name} observation prepared")
            print(f"  - Image: {observation['observation.images.cam_high'].shape}")
            print(f"  - State: {observation['observation.state'].shape}")
            print(f"  - Instruction length: {len(observation['language_instruction'])} chars")

        print("\n[PASS] All gestures processed successfully!")
        return True

    except Exception as e:
        print(f"\n[FAIL] Test failed: {e}")
        import traceback
        traceback.print_exc()
        return False


def test_smolvla_mock_inference():
    """Test mock SmoLVLA inference (without actual model)"""
    print("\n" + "="*70)
    print("TEST 3: Mock SmoLVLA Inference")
    print("="*70)

    print("\nNOTE: This test uses a mock policy (not real SmoLVLA)")
    print("To test with real SmoLVLA, run:")
    print("  python gesture_control_smolvla.py --policy_path lerobot/smolvla_base")

    try:
        # Create mock policy that returns random actions
        class MockSmoLVLAPolicy:
            def select_action(self, observation):
                """Mock action selection"""
                # Simulate 7-DOF robot action
                return torch.randn(1, 7)

            def eval(self):
                pass

        mock_policy = MockSmoLVLAPolicy()

        # Create bridge with mock policy
        bridge = GestureToVLABridge(
            output_dir="test_smolvla",
            policy=mock_policy,
            device="cpu"
        )

        # Simulate gesture → VLA pipeline
        print("\n--- Simulating GOTO gesture ---")
        gesture_id = 2
        aoi = np.array([0.5, 0.3, 0.8])

        # Generate instruction
        instruction_data = bridge.gesture_to_instruction(gesture_id, aoi)
        print(f"Instruction: {instruction_data['instruction']}")

        # Create mock frames
        rgb_frame, depth_frame = create_mock_observation()

        # Prepare observation
        observation = bridge._prepare_smolvla_observation(
            rgb_frame, depth_frame, instruction_data['instruction']
        )

        # Run mock inference
        print("\nRunning mock policy inference...")
        with torch.no_grad():
            action = mock_policy.select_action(observation)

        print(f"[PASS] Mock policy returned action: {action.shape}")
        print(f"  Action values: {action.cpu().numpy()[0][:3]}... (first 3 of 7)")

        # Verify action shape
        assert action.shape == torch.Size([1, 7]), f"Action shape incorrect: {action.shape}"

        print("\n[PASS] Mock inference successful!")
        print("\nWith real SmoLVLA, this action would be sent to the robot.")
        return True

    except Exception as e:
        print(f"\n[FAIL] Test failed: {e}")
        import traceback
        traceback.print_exc()
        return False


def test_load_real_smolvla():
    """Attempt to load real SmoLVLA model"""
    print("\n" + "="*70)
    print("TEST 4: Load Real SmoLVLA (Optional)")
    print("="*70)

    print("\nAttempting to load SmoLVLA base model...")
    print("This requires:")
    print("  1. HuggingFace account login (huggingface-cli login)")
    print("  2. LeRobot installed (pip install lerobot)")
    print("  3. Internet connection to download model")

    try:
        from lerobot.common.policies.factory import make_policy

        print("\n[PASS] LeRobot policy factory imported")

        # Try to load SmoLVLA base model
        print("\nLoading lerobot/smolvla_base (this may take a few minutes)...")

        policy = make_policy(
            hydra_cfg=None,
            pretrained="lerobot/smolvla_base",
            device="cpu"  # Use CPU for testing
        )

        policy.eval()

        print("[PASS] SmoLVLA base model loaded successfully!")
        print(f"  Policy type: {type(policy).__name__}")

        # Test with real policy
        print("\n--- Testing with real SmoLVLA ---")

        bridge = GestureToVLABridge(
            output_dir="test_smolvla",
            policy=policy,
            device="cpu"
        )

        # Simulate gesture
        instruction_data = bridge.gesture_to_instruction(2, np.array([0.5, 0.3, 0.8]))
        rgb_frame, depth_frame = create_mock_observation()

        observation = bridge._prepare_smolvla_observation(
            rgb_frame, depth_frame, instruction_data['instruction']
        )

        # Run real inference
        print("Running real SmoLVLA inference...")
        with torch.no_grad():
            action = policy.select_action(observation)

        print(f"[PASS] Real SmoLVLA inference successful!")
        print(f"  Action shape: {action.shape}")
        print(f"  Action values: {action.cpu().numpy()[0][:3]}... (first 3)")

        return True

    except ImportError:
        print("\n[WARN] LeRobot not installed")
        print("  Install with: pip install lerobot")
        return False

    except Exception as e:
        print(f"\n[WARN] Could not load SmoLVLA: {e}")
        print("\nThis is expected if:")
        print("  - Not logged into HuggingFace")
        print("  - No internet connection")
        print("  - Model not downloaded yet")
        print("\nTo use SmoLVLA:")
        print("  1. huggingface-cli login")
        print("  2. python gesture_control_smolvla.py --policy_path lerobot/smolvla_base")
        return False


def main():
    print("\n" + "="*70)
    print("SmoLVLA Policy Inference Testing with Gestures")
    print("="*70)

    results = []

    # Test 1: Observation Preparation
    results.append(("Observation Preparation", test_observation_preparation()))

    # Test 2: Full Pipeline
    results.append(("Gesture Pipeline", test_gesture_to_observation_pipeline()))

    # Test 3: Mock Inference
    results.append(("Mock Inference", test_smolvla_mock_inference()))

    # Test 4: Real SmoLVLA (optional)
    results.append(("Real SmoLVLA", test_load_real_smolvla()))

    # Print summary
    print("\n" + "="*70)
    print("TEST SUMMARY")
    print("="*70)

    for test_name, passed in results:
        status = "[PASS]" if passed else "[FAIL]"
        print(f"{status:12} - {test_name}")

    passed_count = sum(1 for _, p in results if p)
    total_count = len(results)

    print(f"\nTotal: {passed_count}/{total_count} tests passed")

    if passed_count >= 3:  # First 3 tests are critical
        print("\n[PASS] Core functionality working! System ready for SmoLVLA deployment.")
    else:
        print("\n[FAIL] Some core tests failed. Check errors above.")


if __name__ == "__main__":
    main()
