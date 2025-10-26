from lerobot.cameras.opencv.configuration_opencv import OpenCVCameraConfig
from lerobot.cameras.realsense.configuration_realsense import RealSenseCameraConfig
from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.datasets.utils import hw_to_dataset_features
from pathlib import Path
import shutil

# Bimanual Follower Robot & Teleoperator Robot Config and Class import
from lerobot.teleoperators.bi_so100_leader import BiSO100Leader, BiSO100LeaderConfig
from lerobot.robots.bi_so100_follower import BiSO100Follower, BiSO100FollowerConfig

from lerobot.utils.control_utils import init_keyboard_listener
from lerobot.utils.utils import log_say
from lerobot.utils.visualization_utils import _init_rerun
from lerobot.record import record_loop
from lerobot.processor import make_default_processors

NUM_EPISODES = 5
FPS =15
EPISODE_TIME_SEC = 40
RESET_TIME_SEC = 30
TASK_DESCRIPTION = "Test"
REPO_ID = "dwux/record-test12"
DATASET_ROOT = "/home/lekiwi/.cache/huggingface/lerobot/dwux/record-test12"

# Setup Port
LEFT_ARM_PORT_FOLLOWER = "/dev/ttyACM0"
RIGHT_ARM_PORT_FOLLOWER = "/dev/ttyACM1"
LEFT_ARM_PORT_LEADER = "/dev/ttyACM3"
RIGHT_ARM_PORT_LEADER = "/dev/ttyACM2"

# Create the robot and teleoperator configurations
# Setup Camera - Using RealSense for top camera only
# RealSense camera with serial number "117222250334" for top view
camera_config = {
    # "top": RealSenseCameraConfig(
    #     serial_number_or_name="117222250334",
    #     width=640,
    #     height=480,
    #     fps=15
    # ),
}

# Setup Follower Robot using BiSO100FollowerConfig
robot_config = BiSO100FollowerConfig(
    left_arm_port=LEFT_ARM_PORT_FOLLOWER,
    right_arm_port=RIGHT_ARM_PORT_FOLLOWER,
    id="my_awesome_xle",
    cameras=camera_config,
)

# Setup Leader Robot using BiSO100LeaderConfig
teleop_config = BiSO100LeaderConfig(
    left_arm_port=LEFT_ARM_PORT_LEADER,
    right_arm_port=RIGHT_ARM_PORT_LEADER,
    id="my_awesome_xle",
)

# Initialize the robot and teleoperator
robot = BiSO100Follower(robot_config)
teleop = BiSO100Leader(teleop_config)

# Create the default processors for record_loop
teleop_action_processor, robot_action_processor, robot_observation_processor = make_default_processors()

# Configure the dataset features
action_features = hw_to_dataset_features(robot.action_features, "action")
obs_features = hw_to_dataset_features(robot.observation_features, "observation")
dataset_features = {**action_features, **obs_features}

# Create the dataset - always create fresh locally
# Delete the old dataset if it exists to avoid conflicts
dataset_root_path = Path(DATASET_ROOT)
if dataset_root_path.exists():
    print(f"Removing old incomplete dataset at {DATASET_ROOT}")
    shutil.rmtree(DATASET_ROOT)

print(f"Creating new dataset at {DATASET_ROOT}")
dataset = LeRobotDataset.create(
    repo_id=REPO_ID,
    fps=FPS,
    features=dataset_features,
    robot_type=robot.name,
    use_videos=True,
    image_writer_threads=4,
    root=DATASET_ROOT,
)

# Initialize the keyboard listener and return visualization
_, events = init_keyboard_listener()
_init_rerun(session_name="recording")

# Connect the robot and teleoperator
robot.connect()
teleop.connect()

episode_idx = 0
while episode_idx < NUM_EPISODES and not events["stop_recording"]:
    print(f"Recording episode {episode_idx + 1} of {NUM_EPISODES}")

    record_loop(
        robot=robot,
        events=events,
        fps=FPS,
        teleop=teleop,
        dataset=dataset,
        control_time_s=EPISODE_TIME_SEC,
        single_task=TASK_DESCRIPTION,
        display_data=True,
        teleop_action_processor=teleop_action_processor,
        robot_action_processor=robot_action_processor,
        robot_observation_processor=robot_observation_processor,
    )

    print('-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-')
    print('-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-')

    # Reset the environment if not stopping or re-recording
    if not events["stop_recording"] and (episode_idx < NUM_EPISODES - 1 or events["rerecord_episode"]):
        print("Reset the environment")
        record_loop(
            robot=robot,
            events=events,
            fps=FPS,
            teleop=teleop,
            dataset=dataset,
            control_time_s=RESET_TIME_SEC,
            single_task=TASK_DESCRIPTION,
            display_data=True,
            teleop_action_processor=teleop_action_processor,
            robot_action_processor=robot_action_processor,
            robot_observation_processor=robot_observation_processor,
        )

    if events["rerecord_episode"]:
        print("Re-recording episode")
        events["rerecord_episode"] = False
        events["exit_early"] = False
        dataset.clear_episode_buffer()
        continue

    dataset.save_episode()
    episode_idx += 1

# Clean up
print("Stop recording")
robot.disconnect()
teleop.disconnect()

# Save dataset locally (don't push to hub by default)
print(f"Dataset saved locally at {DATASET_ROOT}")
print("To push to HuggingFace later, you can run: dataset.push_to_hub()")