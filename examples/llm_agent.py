from robocrew.core.LLMAgent import LLMAgent
from robocrew.core.tools import finish_task
from robocrew.robots.XLeRobot.tools import create_move_forward, create_turn_left, create_turn_right
from robocrew.robots.XLeRobot.wheel_controls import XLeRobotWheels

# Set up wheels
sdk = XLeRobotWheels.connect_serial("/dev/ttyACM0")     # provide the right arm usb port - the arm connected to wheels
wheel_controller = XLeRobotWheels(sdk)

print("Wheel controller connected")

# Create movement tools
move_forward = create_move_forward(wheel_controller)
turn_left = create_turn_left(wheel_controller)
turn_right = create_turn_right(wheel_controller)

print("Movement tools created")

# Create agent
agent = LLMAgent(
    model="google_genai:gemini-robotics-er-1.5-preview",
    tools=[move_forward, turn_left, turn_right, finish_task],
    main_camera_usb_port="/dev/video1",     # provide usb port main camera connected to
    camera_fov=110,
)
print("Agent created")

agent.task = "Don't move"

try:
    agent.go()
except KeyboardInterrupt:
    print("\n\nInterrupted by user (Ctrl+C)")
    wheel_controller._stop_all()
    print("Wheels stopped")
except Exception as e:
    print(f"\n\nError occurred: {e}")
    wheel_controller._stop_all()
    print("Wheels stopped")
finally:
    print("Cleaning up...") 