# GEST: Gesture-Enabled System for Teleoperation 

**Cal Hacks 12.0 Project | Sponsored by BitRobot**

*Teaching robots through gestures and demonstrations*
A system that lets robots learn manipulation tasks through gesture recognition and human demonstrations, powered by vision-language-action models.

## Table of Contents
- [Overview](https://github.com/dwu006/gest?tab=readme-ov-file#overview)
- [Demo](https://github.com/dwu006/gest?tab=readme-ov-file#demo)
- [How it works](https://github.com/dwu006/gest?tab=readme-ov-file#how-it-works)
- [System Architecture](https://github.com/dwu006/gest?tab=readme-ov-file#system-architecture)
- [Data Collection Pipeline](https://github.com/dwu006/gest?tab=readme-ov-file#data-collection-pipeline)
- [Installation](https://github.com/dwu006/gest?tab=readme-ov-file#installation)
    - [Prerequisites](https://github.com/dwu006/gest?tab=readme-ov-file#prerequisites)
    - [Setup](https://github.com/dwu006/gest?tab=readme-ov-file#setup)
- [Usage](https://github.com/dwu006/gest?tab=readme-ov-file#usage)
  - [Running Teleoperation](https://github.com/dwu006/gest?tab=readme-ov-file#running-teleoperation)
      - [Via Terminal](https://github.com/dwu006/gest?tab=readme-ov-file#via-terminal)
      - [Via Script](https://github.com/dwu006/gest?tab=readme-ov-file#via-script)     
  - [Running Gestures](https://github.com/dwu006/gest?tab=readme-ov-file#running-with-gestures)
- [Data Collection](https://github.com/dwu006/gest?tab=readme-ov-file#data-collection)
    - [Recording Episodes](https://github.com/dwu006/gest?tab=readme-ov-file#recording-episodes)
    - [How to Record](https://github.com/dwu006/gest?tab=readme-ov-file#how-to-record)
    - [Data Format](https://github.com/dwu006/gest?tab=readme-ov-file#what-gets-saved)
- [LLM Agent Control](https://github.com/dwu006/gest?tab=readme-ov-file#llm-agent-control)
- [Technical Stack](https://github.com/dwu006/gest?tab=readme-ov-file#technical-stack)
- [Limitations & Future Work](https://github.com/dwu006/gest?tab=readme-ov-file#limitations-and-future-work)
- [Acknowledgments](https://github.com/dwu006/gest?tab=readme-ov-file#acknowledgments)
- [Citation](https://github.com/dwu006/gest?tab=readme-ov-file#citation)

## Overview

GEST bridges the gap between humans and robots through natural interaction. Instead of writing complex code, you simply show the robot what to do through gestures and demonstrations. The robot watches, learns, and then performs the task autonomously.

**The Challenge:** Programming robots for everyday tasks is hard and requires expertise.

**Our Approach:** Let people teach robots the way they'd teach another person - through gestures and demonstrations. The robot learns from watching and can then do the task on its own.

We use HuggingFace's SmolVLA model to enable robots to understand visual scenes and take appropriate actions based on what they've learned.

---

## Demo

### Pouring Water
![Pouring Water Demo](pouring_water.gif)

The robot learned to pour water between cups by watching human demonstrations through teleoperation.

### Cleaning Table
![Cleaning Table Demo](cleaning_table.gif)

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
    |-- HuggingFace SmolVLA
    |-- Learns vision-action mapping
    |
    v
Trained Model
    |-- Gesture triggers learned behavior
    |-- Robot performs task autonomously
```

### Data Collection Pipeline

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

## Wireless

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
- Base: I/K/J/L/U/O keys for movement (front,back,left,right,rotate-left,rotate-right)
- Head: comma and period keys
- Exit: ESC

## Wired

Connect Leader arms to your XLeRobot

### Via Terminal
You will need to run this twice (once for each arm)
```bash
conda activate lerobot
python -m lerobot.teleoperate \
    --robot.type=so101_follower \
    --robot.port=/dev/ttyACM1 \
    --robot.id=my_awesome_xle \
    --teleop.type=so101_leader \
    --teleop.port=/dev/ttyACM2 \
    --teleop.id=my_awesome_xle \
    --display_data=true
```

### Via Script
Allow dual arm control
```bash
conda activate lerobot
cd examples
python teleop_bimanual.py
```

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

### Data Format

```
datasets/
├── data/
│   ├── episode_000/
│   │   └── episode_000.parquet              # numeric data: robot observations + actions
│   └── ...                                  # future episodes
│
├── meta/
│   ├── info.json                            # global dataset metadata
│   └── ...                                  # optional extra meta files (schema, stats, etc.)
│
└── videos/
    ├── observations.images.front/
    │   ├── episode_000.mp4                  # front camera video
    │   └── ...                              # future episodes
    │
    ├── observations.images.left/
    │   ├── episode_000.mp4                  # left camera video
    │   └── ...                              # future episodes
    │
    └── observations.images.right/
        ├── episode_000.mp4                  # right camera video
        └── ...                              # future episodes
```
## LLM Agent Control

If you haven't 
```bash
conda activate lerobot
pip install robocrew
```
Create a .env file with your Google Gemini API
```bash
GOOGLE_API_KEY=<your-api-key>
```
Update your task description in llm_agent.py and run
```bash
cd examples
python llm_agent.py
```
If the above cmd doesn't work, run:
```bash
export GOOGle_API_KEY=<your-api-key>
python llm_agent.py
```

## Technical Stack

### Hardware
- XLeRobot dual-arm mobile manipulator
- SO-ARM101 leader arms (6 degrees of freedom each)
- Intel RealSense D435i depth camera
- Raspberry Pi for robot control

### Software
- **Core Framework**: lerobot by HuggingFace
- **Vision Model**: HuggingFace Small VLA
- **Gesture Detection**: MediaPipe Hands
- **Camera Interface**: pyrealsense2
- **Communication**: TCP/IP over WiFi

### Key Libraries
```
lerobot
robocrew
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
- Single-task models (each task needs its own training)
- Requires stable lighting conditions
- Limited to tabletop manipulation tasks

**What's next:**
- Multi-task learning: one model that handles multiple tasks
- Better generalization to new object positions
- Integration with language instructions
- Continuous learning from mistakes

---

## Acknowledgments

Thanks to BitRobot for sponsoring this project and Cal Hacks 12.0 for hosting an amazing event!

This project builds on work from the robotics community, particularly:
- HuggingFace for the lerobot framework and SmolVLA model
- Google for MediaPipe hand tracking
- Intel for the RealSense SDK

---

## Citation

If you find this work useful:

```bibtex
@misc{gesturebot2024,
  title={GEST: Gesture-Enabled System for Teleoperation},
  author={[GEST]},
  year={2025},
  howpublished={Cal Hacks 12.0}
}
```
---

Made at Cal Hacks 12.0 | Sponsored by BitRobot
