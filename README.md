# GEST: Gesture-Enabled System for Teleoperation 

**Cal Hacks 12.0 Project | Sponsored by BitRobot**

## Setup

Install LeRobot following their [installation guide](https://huggingface.co/docs/lerobot/installation) on BOTH your local machine and a target machine (if using one)

Follow the [XLeRobot Docs](https://xlerobot.readthedocs.io/en/latest/software/getting_started/install.html) and **move the files from XLeRobot folder to your existing LeRobot folder** (important!)

*Teaching robots through gestures and demonstrations*

A system that lets robots learn manipulation tasks through gesture recognition and human demonstrations, powered by vision-language-action models.

---
## Overview

GEST bridges the gap between humans and robots through natural interaction. Instead of writing complex code, you simply show the robot what to do through gestures and demonstrations. The robot watches, learns, and then performs the task autonomously.

**The Challenge:** Programming robots for everyday tasks is hard and requires expertise.

**Our Approach:** Let people teach robots the way they'd teach another person - through gestures and demonstrations. The robot learns from watching and can then do the task on its own.

We use HuggingFace's Small VLA model to enable robots to understand visual scenes and take appropriate actions based on what they've learned.

---

## Demo

### Pouring Water
![Pouring Water Demo](gifs/pouring_water.gif)

The robot learned to pour water between cups by watching human demonstrations through teleoperation.

### Cleaning Table
![Cleaning Table Demo](gifs/cleaning_table.gif)

After watching a human clean a table with tissues, the robot learned to replicate the motion.

---

## How It Works

Our system has three main phases:

### Phase 1: Gesture Detection
The robot watches for a specific hand gesture (like a closed fist) using its camera and MediaPipe hand tracking. When it sees the gesture, it knows you want to demonstrate a task.

### Phase 2: Teleoperation and Recording
You control the robot remotely using leader arms and a keyboard to demonstrate the task. As you do this, the robot records everything:
- What its joints are doing
- What actions you're sending
- What its camera sees

This creates a rich dataset that captures both the physical motion and the visual context.

### Phase 3: Learning and Autonomous Execution
The recorded demonstrations are fed into a Vision-Language-Action model. The model learns the relationship between what the robot sees and what actions it should take. Once trained, the robot can perform the task autonomously when it sees the same gesture.

---

## System Architecture

```
Human Operator
    |
    |-- Hand Gesture (detected by MediaPipe)
    |-- Leader Arms + Keyboard (teleoperation)
    |
    v
Laptop (Client)
    |-- gesture_detector.py
    |-- teleop_client.py
    |
    v (WiFi, 50Hz)
    v
Robot (Server)
    |-- teleop_server.py
    |-- Follower arms, base, head
    |-- Records: state + actions + camera feed
    |
    v
Dataset
    |-- Episodes with synchronized data
    |
    v
Training Pipeline
    |-- HuggingFace Small VLA
    |-- Learns vision-action mapping
    |
    v
Trained Model
    |-- Gesture triggers learned behavior
    |-- Robot performs task autonomously
```

### Data Flow During Recording

When you demonstrate a task, the system records:
- **Robot State**: Joint positions and velocities at 50Hz
- **Actions**: The commands being sent to the motors
- **Visual Data**: RGB video from the head camera at 30Hz
- **Timestamps**: To keep everything synchronized

All of this gets saved in a format that the VLA model can learn from.

---

## Installation

### Prerequisites

- Python 3.10 or higher
- Ubuntu 22.04 (for robot control)
- GPU with CUDA (for training)
- Intel RealSense camera
- XLeRobot hardware with leader arms

### Setup

**Clone the repository:**
```bash
git clone https://github.com/yourusername/gesturebot.git
cd gesturebot
```

**Install dependencies:**
```bash
conda create -n gesturebot python=3.10
conda activate gesturebot
pip install lerobot mediapipe opencv-python pyrealsense2
```

Do this on both your laptop (client) and the robot (server).

**Configure network:**
```bash
export ROBOT_IP=xle2.local  # or your robot's IP address
ping $ROBOT_IP  # verify connection
```

---

## Usage

### Running Teleoperation

**On the robot:**
```bash
ssh user@robot_ip
conda activate gesturebot
python teleop_server.py
```

**On your laptop:**
```bash
conda activate gesturebot
sudo chmod 666 /dev/ttyACM*  # fix USB permissions
python teleop_client.py $ROBOT_IP
```

**Controls:**
- Right arm: Move the leader arm physically
- Left arm: W/S/A/D keys for movement, Q/E/R/F/T/G for joints
- Base: I/K/J/L keys for movement
- Head: comma and period keys
- Exit: ESC

### Running with Gestures

```bash
python demo_gesture_pickup.py --gesture fist
```

Make a fist, and the robot will execute the learned behavior.

---

## Data Collection

### Recording Episodes

You'll need three terminal windows:

**Terminal 1 - Robot server:**
```bash
ssh user@robot_ip
conda activate gesturebot
python teleop_server.py
```

**Terminal 2 - Recording script:**
```bash
ssh user@robot_ip
conda activate gesturebot

python -m lerobot.scripts.record \
    --robot-path lerobot.robots.xlerobot \
    --fps 30 \
    --repo-id gesture_task_data \
    --num-episodes 50 \
    --episode-time-s 90
```

**Terminal 3 - Control from laptop:**
```bash
python teleop_client.py $ROBOT_IP
```

### How to Record

For each demonstration:

1. Place objects in their starting positions
2. Make the trigger gesture
3. Use the leader arm and keyboard to show the robot what to do
4. Complete the entire task (pick, move, place, return home)
5. The system automatically saves after 90 seconds
6. Reset objects and repeat

The goal is to be consistent but not robotic - natural variations in how you perform the task actually help the model learn better.

### What Gets Saved

```
datasets/gesture_task_data/
├── episode_000/
│   ├── data.hdf5           # robot joint data and actions
│   └── videos/
│       └── head_camera.mp4 # what the robot saw
├── episode_001/
└── ...
```

## Technical Stack

### Hardware
- XLeRobot dual-arm mobile manipulator
- SO-ARM101 leader arms (6 degrees of freedom each)
- Intel RealSense D435i depth camera
- Raspberry Pi 4 for robot control

### Software
- **Core Framework**: lerobot by HuggingFace
- **Vision Model**: HuggingFace Small VLA
- **Gesture Detection**: MediaPipe Hands
- **Camera Interface**: pyrealsense2
- **Communication**: TCP/IP over WiFi

### Key Libraries
```
lerobot
mediapipe
opencv-python
pyrealsense2
torch
transformers
```

---

Building this system taught us a lot about the gap between demonstrations and autonomous behavior. The robot doesn't just memorize motions - it learns to understand visual context and adapt its actions accordingly.

The VLA model is particularly interesting because it processes both vision and action in a unified way. This means the robot can learn tasks that depend on visual feedback, like aligning a cup under a pour or finding the edge of a table to clean.

We also learned that data quality matters more than quantity. Fifty smooth, consistent demonstrations work better than a hundred sloppy ones.

---

## Limitations and Future Work

**Current limitations:**
- Works best with objects in similar positions to training data
- Single-task models (each task needs its own training)
- Requires stable lighting conditions
- Limited to tabletop manipulation tasks

**What's next:**
- Multi-task learning: one model that handles multiple tasks
- Better generalization to new object positions
- Integration with language instructions
- Continuous learning from mistakes

---

## Team

Cal Hacks 12.0 Team:

- [Your Name] - System Integration
- [Team Member 2] - ML and Training
- [Team Member 3] - Computer Vision
- [Team Member 4] - Robot Hardware

---

## Acknowledgments

Thanks to BitRobot for sponsoring this project and Cal Hacks 12.0 for hosting an amazing event.

This project builds on work from the robotics community, particularly:
- HuggingFace for the lerobot framework and Small VLA model
- Google for MediaPipe hand tracking
- Intel for the RealSense SDK

---

## Citation

If you find this work useful:

```bibtex
@misc{gesturebot2024,
  title={GestureBot: Teaching Robots Through Natural Demonstrations},
  author={[Your Team]},
  year={2024},
  howpublished={Cal Hacks 12.0}
}
```
---

Made at Cal Hacks 12.0 | Sponsored by BitRobot
