# Gesture Control Integration for LeRobot

This module integrates hand gesture recognition with LeRobot's VLA models for intuitive robot control.

## Features

- **Real-time Hand Gesture Recognition** using MediaPipe
- **3D Spatial Awareness** via Intel RealSense depth camera
- **Natural Language Instructions** for VLA models
- **Episode Logging** for LeRobot dataset collection

## Gesture Mappings

| Gesture | ID | Action | Description |
|---------|-----|--------|-------------|
| Open Palm | 0 | STOP | Emergency halt - stops all robot motion |
| Pointer | 2 | GOTO | Navigate to pointed location in 3D space |
| Fist/Close | 1 | PICK | Grasp object at location |

## Installation

1. Install gesture recognition dependencies:
```bash
pip install -r gesture_requirements.txt
```

2. Install LeRobot (if not already installed):
```bash
pip install -e .
```

## Usage

### Basic Mode (Logging Only)
```bash
python gesture_control.py
```

### With Robot Integration
```bash
python gesture_control.py --robot <robot_name>
```

### Custom Settings
```bash
python gesture_control.py \
    --output_dir my_episodes \
    --table_distance 1.0 \
    --stability_threshold 3
```

## File Structure

```
gest/
├── gesture_control.py              # Main entry point
├── gesture_requirements.txt        # Gesture-specific dependencies
├── model/                          # Gesture classifier models
│   └── keypoint_classifier/
├── utils/                          # Utility functions
├── src/lerobot/gestures/          # Gesture integration module
│   ├── __init__.py
│   ├── gesture_bridge.py          # VLA bridge
│   └── visualizer.py              # Visual feedback
└── lerobot_data/                  # Episode output directory
```

## Integration with LeRobot

The gesture system is designed to work seamlessly with LeRobot's existing infrastructure:

### 1. Direct Robot Control
```python
from lerobot import make_robot
from lerobot.gestures import GestureToVLABridge

robot = make_robot("your_robot_config")
bridge = GestureToVLABridge(robot=robot)
```

### 2. VLA Policy Integration
The gesture bridge converts gestures to natural language instructions that can be fed directly to LeRobot's VLA models (like SmoLVLA).

### 3. Dataset Collection
All gesture commands are logged in JSONL format for training:
```json
{
  "timestamp": "2025-01-15T10:30:45",
  "command_id": 0,
  "instruction": "Navigate to position x=0.50, y=0.30, z=0.80 meters",
  "gesture": "GOTO",
  "gesture_id": 2,
  "aoi": {"center": [0.5, 0.3, 0.8], "type": "point"}
}
```

## Workflow Examples

### Point → Pick Workflow
1. Make **GOTO gesture** (pointer) at object location
2. System calculates 3D coordinates
3. Make **PICK gesture** (fist)
4. System uses last pointed location for picking

### Emergency Stop
- Make **STOP gesture** (open palm) anytime
- Immediate halt command sent with EMERGENCY priority

## Hardware Requirements

- **Intel RealSense D455** (or compatible depth camera)
- **Webcam** (if not using RealSense RGB)
- Computer with USB 3.0

## Configuration

### Gesture Classifier
The gesture classifier is pre-trained on 3 gestures. To retrain or add gestures:
1. Collect training data
2. Update `model/keypoint_classifier/`
3. Update `keypoint_classifier_label.csv`

### Camera Calibration
Adjust `table_distance` parameter to match your setup:
```python
--table_distance 0.8  # Distance to work surface in meters
```

## API Reference

### GestureToVLABridge

```python
bridge = GestureToVLABridge(
    output_dir="lerobot_data",  # Episode data directory
    robot=None                   # LeRobot robot instance (optional)
)

# Convert gesture to instruction
instruction = bridge.gesture_to_instruction(
    gesture_id=2,                # 0=STOP, 1=PICK, 2=GOTO
    aoi_coords=[0.5, 0.3, 0.8]  # 3D coordinates (optional)
)

# Send to VLA/robot
response = bridge.send_to_vla(
    instruction,
    rgb_frame,    # numpy array
    depth_frame   # numpy array (optional)
)
```

### SimpleVisualizer

```python
from lerobot.gestures import SimpleVisualizer

SimpleVisualizer.draw_gesture_info(
    image,        # OpenCV image
    gesture_id,   # 0, 1, or 2
    gesture_name  # Optional custom name
)
```

## Troubleshooting

### Camera Not Found
```
Error starting camera: No device connected
```
**Solution**: Check RealSense connection, close other camera apps

### Gesture Not Detected
**Solution**:
- Ensure good lighting
- Keep hand fully visible
- Hold gesture for ~5 frames (stability threshold)

### Import Errors
**Solution**: Make sure you're in the gest directory and have installed dependencies

## Contributing

To add new gestures:
1. Collect training samples
2. Train classifier model
3. Update gesture mappings in `gesture_bridge.py`
4. Update documentation

## License

Same as LeRobot (Apache 2.0)
