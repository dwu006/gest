# SmoLVLA Gesture Control - Setup Guide

Complete guide to set up gesture-controlled SmoLVLA for robot manipulation.

## Quick Start

### 1. Install Dependencies

```bash
# Install LeRobot
pip install lerobot

# Install gesture recognition dependencies
pip install -r gesture_requirements.txt
```

### 2. Run Without Policy (Logging Mode)

Test gesture recognition and log commands for training:

```bash
python gesture_control_smolvla.py --no_policy
```

### 3. Run With SmoLVLA Base Model

Use the pretrained base model (no fine-tuning):

```bash
python gesture_control_smolvla.py --policy_path lerobot/smolvla_base
```

### 4. Run With Fine-Tuned Model

After fine-tuning SmoLVLA on your robot:

```bash
python gesture_control_smolvla.py --policy_path YOUR_HF_USER/your_finetuned_model
```

## Complete Workflow

### Phase 1: Data Collection (Gesture Logging)

1. **Record Gesture Episodes**
   ```bash
   python gesture_control_smolvla.py --no_policy --output_dir training_data
   ```

2. **Perform Gestures**
   - GOTO: Point to locations
   - PICK: Make fist to grasp
   - STOP: Open palm to halt

3. **Collect ~50 Episodes**
   - Distribute across task variations
   - Episodes saved to `training_data/episode_TIMESTAMP.jsonl`

### Phase 2: Fine-Tune SmoLVLA

1. **Convert Logs to LeRobot Dataset Format**
   ```python
   # TODO: Create conversion script
   # Convert JSONL logs to LeRobot dataset format
   ```

2. **Fine-Tune SmoLVLA**
   ```bash
   lerobot-train \
     --policy.path=lerobot/smolvla_base \
     --dataset.repo_id=YOUR_HF_USER/gesture_dataset \
     --batch_size=64 \
     --steps=20000
   ```

   Training time: ~4 hours on A100 GPU

3. **Push to HuggingFace Hub**
   ```bash
   huggingface-cli login
   # Model automatically pushed during training
   ```

### Phase 3: Deploy with Gestures

1. **Run Gesture Control with Fine-Tuned Model**
   ```bash
   python gesture_control_smolvla.py \
     --policy_path YOUR_HF_USER/gesture_smolvla \
     --robot your_robot_config
   ```

2. **Use Gestures for Real-Time Control**
   - Gestures → Natural Language → SmoLVLA → Robot Actions

## Configuration Options

### Basic Options
```bash
--policy_path lerobot/smolvla_base    # HuggingFace model path
--no_policy                            # Run without loading policy
--device cuda                          # Use GPU (cuda) or CPU
--output_dir lerobot_data             # Where to save episodes
```

### Camera & Detection
```bash
--width 640                           # Camera width
--height 480                          # Camera height
--table_distance 0.8                  # Distance to work surface (meters)
--stability_threshold 5               # Frames to hold gesture
```

### Robot Integration
```bash
--robot koch                          # Robot configuration name
```

## How It Works

### Gesture → Instruction Pipeline

1. **Gesture Detection**
   - MediaPipe detects hand landmarks
   - Gesture classifier identifies: STOP/GOTO/PICK

2. **3D Localization**
   - RealSense depth camera
   - Ray casting from finger to table plane
   - Calculates 3D coordinates (x, y, z)

3. **Natural Language Generation**
   - GOTO: "Navigate to position x=0.5, y=0.3, z=0.8 meters"
   - PICK: "Pick up object at position x=0.5, y=0.3, z=0.8 meters"
   - STOP: "Stop all motion immediately"

4. **SmoLVLA Inference**
   - Observation: {image, robot_state, instruction}
   - Output: Action chunk for robot
   - Robot executes action

### Data Flow

```
Hand Gesture
    ↓
MediaPipe (2D landmarks)
    ↓
RealSense Depth (3D coordinates)
    ↓
Natural Language Instruction
    ↓
SmoLVLA Policy
    ↓
Robot Action
```

## File Structure

```
gest/
├── gesture_control_smolvla.py          ← Main script with SmoLVLA
├── gesture_control.py                  ← Basic version (no SmoLVLA)
├── gesture_requirements.txt            ← Dependencies
├── SMOLVLA_SETUP.md                   ← This file
├── GESTURE_README.md                  ← Gesture integration docs
│
├── src/lerobot/gestures/              ← Gesture module
│   ├── gesture_bridge.py              ← VLA integration
│   └── visualizer.py                  ← Visual feedback
│
├── model/                             ← Gesture classifier
│   └── keypoint_classifier/
│
├── utils/                             ← Utilities
│
└── lerobot_data/                      ← Episode logs
    └── episode_TIMESTAMP.jsonl
```

## Troubleshooting

### SmoLVLA Not Loading
```
Error: Could not import LeRobot policy factory
```
**Solution**: Install LeRobot: `pip install lerobot`

### CUDA Out of Memory
```
RuntimeError: CUDA out of memory
```
**Solution**: Use CPU: `--device cpu`

### Camera Not Found
```
Error starting camera: No device connected
```
**Solution**: Check RealSense connection, close other camera apps

### Gesture Not Triggering Policy
**Check**:
- Policy loaded successfully (check startup logs)
- Gesture held for stability_threshold frames (default 5)
- No errors in console output

## Advanced Usage

### Custom Robot Integration

```python
# In gesture_control_smolvla.py, modify:
from lerobot import make_robot

robot = make_robot("your_robot_config")

# Robot config should define:
# - get_state(): Return current joint positions
# - send_action(action): Execute action from policy
```

### Adjust Image Preprocessing

Edit `gesture_bridge.py`, `_prepare_smolvla_observation()`:
- Image size (default 224x224)
- Normalization values
- Camera mapping

### Multi-Camera Setup

```python
observation = {
    "observation.images.cam_high": high_cam_tensor,
    "observation.images.cam_wrist": wrist_cam_tensor,
    "observation.state": robot_state,
    "language_instruction": instruction
}
```

## Dataset Format

Episode logs saved as JSONL:
```json
{
  "timestamp": "2025-01-15T10:30:45",
  "command_id": 0,
  "instruction": "Navigate to position x=0.50, y=0.30, z=0.80 meters",
  "gesture": "GOTO",
  "gesture_id": 2,
  "priority": "NORMAL",
  "aoi": {"center": [0.5, 0.3, 0.8], "type": "point"},
  "action_output": [0.1, 0.2, ...],  # If policy ran
  "image_shape": [720, 1280, 3],
  "has_depth": true
}
```

## Performance Tips

1. **Use GPU**: SmoLVLA runs much faster on CUDA
2. **Adjust Stability**: Lower threshold for faster response
3. **Fine-Tune**: Base model may not work well - fine-tune on your robot
4. **Lighting**: Ensure good lighting for gesture detection

## Next Steps

1. ✅ Collect 50+ episodes with gestures
2. ✅ Fine-tune SmoLVLA on your robot/task
3. ✅ Deploy fine-tuned model with gesture control
4. ✅ Iterate: collect more data for better performance

## Resources

- **LeRobot Docs**: https://huggingface.co/docs/lerobot
- **SmoLVLA**: https://huggingface.co/docs/lerobot/smolvla
- **HuggingFace Hub**: https://huggingface.co/lerobot/smolvla_base

## License

Apache 2.0 (same as LeRobot)
