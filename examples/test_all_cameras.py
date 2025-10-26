import cv2
import numpy as np
from pathlib import Path

def test_camera(camera_id):
    """Test a camera and return its info"""
    cap = cv2.VideoCapture(camera_id)
    
    if not cap.isOpened():
        return None
    
    # Get camera properties
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = cap.get(cv2.CAP_PROP_FPS)
    
    # Try to read a frame
    ret, frame = cap.read()
    
    cap.release()
    
    if ret and frame is not None:
        return {
            "camera_id": camera_id,
            "width": width,
            "height": height,
            "fps": fps,
            "working": True,
            "frame_shape": frame.shape,
        }
    else:
        return {
            "camera_id": camera_id,
            "width": width,
            "height": height,
            "fps": fps,
            "working": False,
            "frame_shape": None,
        }

def save_frame(camera_id, output_dir):
    """Capture and save a frame from a camera"""
    cap = cv2.VideoCapture(camera_id)
    
    if not cap.isOpened():
        return False
    
    ret, frame = cap.read()
    cap.release()
    
    if ret and frame is not None:
        output_path = output_dir / f"camera_{camera_id}.jpg"
        cv2.imwrite(str(output_path), frame)
        return True
    return False

def main():
    print("=" * 70)
    print("Camera Testing Script")
    print("=" * 70)
    print()
    
    # Test cameras from /dev/video0 to /dev/video40
    working_cameras = []
    broken_cameras = []
    
    print("Testing cameras...")
    for i in range(40):
        result = test_camera(f"/dev/video{i}")
        if result:
            if result["working"]:
                working_cameras.append(result)
                print(f"✓ /dev/video{i}: Working ({result['width']}x{result['height']} @ {result['fps']}fps)")
            else:
                broken_cameras.append(result)
                print(f"✗ /dev/video{i}: Opened but can't read frames")
    
    print()
    print("=" * 70)
    print("SUMMARY")
    print("=" * 70)
    print(f"Working cameras: {len(working_cameras)}")
    print(f"Broken/No frames: {len(broken_cameras)}")
    print()
    
    if working_cameras:
        print("Working Cameras:")
        for cam in working_cameras:
            print(f"  /dev/video{cam['camera_id']}: {cam['width']}x{cam['height']} @ {cam['fps']}fps")
        print()
        
        # Save test frames
        output_dir = Path("camera_test_images")
        output_dir.mkdir(exist_ok=True)
        
        print(f"Saving test frames to {output_dir}/...")
        for cam in working_cameras:
            success = save_frame(cam['camera_id'], output_dir)
            if success:
                print(f"  ✓ Saved frame from /dev/video{cam['camera_id']}")
        
        print()
        print("Test frames saved!")
    else:
        print("No working cameras found!")

if __name__ == "__main__":
    main()

