#!/usr/bin/env python3
import cv2
from lerobot.cameras.realsense.camera_realsense import RealSenseCamera
from lerobot.cameras.realsense.configuration_realsense import RealSenseCameraConfig
from lerobot.cameras.configs import ColorMode

# Configure camera
config = RealSenseCameraConfig(
    serial_number_or_name='117222250334',
    fps=15,
    width=640,
    height=480,
    color_mode=ColorMode.RGB,
    use_depth=False
)

# Start camera
camera = RealSenseCamera(config)
camera.connect()

print("Camera viewer started. Press 'q' to quit.")

try:
    while True:
        frame = camera.read()
        # Convert RGB to BGR for OpenCV display
        frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        cv2.imshow('RealSense D455 Camera', frame_bgr)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
finally:
    camera.disconnect()
    cv2.destroyAllWindows()
    print("Camera disconnected.")
