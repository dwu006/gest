"""
Gesture Visualization
Simple visual feedback for gesture recognition
"""

import cv2 as cv


class SimpleVisualizer:
    """Minimal visual feedback for all gestures"""

    COLORS = {
        0: (0, 0, 255),    # STOP - Red
        1: (255, 0, 255),  # PICK - Magenta
        2: (0, 255, 0)     # GOTO - Green
    }

    NAMES = {
        0: "STOP",
        1: "PICK",
        2: "GOTO"
    }

    @staticmethod
    def draw_gesture_info(image, gesture_id, gesture_name=None):
        """
        Draw gesture name at top of image

        Args:
            image: Image to draw on
            gesture_id: Current gesture ID
            gesture_name: Optional custom name of gesture
        """
        h, w = image.shape[:2]
        color = SimpleVisualizer.COLORS.get(gesture_id, (128, 128, 128))
        name = gesture_name if gesture_name else SimpleVisualizer.NAMES.get(gesture_id, "UNKNOWN")

        # Top bar with gesture name
        cv.rectangle(image, (0, 0), (w, 60), color, -1)
        cv.putText(image, f"{name}", (10, 40),
                  cv.FONT_HERSHEY_SIMPLEX, 1.2, (255, 255, 255), 3)
