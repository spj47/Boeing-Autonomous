# Import libraries
import os
from typing import Optional, Tuple, Any

# YOLO model (Ultralytics)
from ultralytics import YOLO

# Camera wrapper
from camera_interface import Webcam


class YOLODetector:
    # YOLO wrapper for camera-based cone detection
    def __init__(self, model_path: Optional[str] = None, rgb: bool = True) -> None:
        # Build a safe default model path so Linux (Pi) case-sensitivity never breaks us
        if model_path is None:
            base_dir = os.path.dirname(os.path.abspath(__file__))

            # Prefer sensors/best.pt relative to project root
            candidate_1 = os.path.join(base_dir, "sensors", "best.pt")
            candidate_2 = os.path.join(base_dir, "best.pt")

            if os.path.exists(candidate_1):
                model_path = candidate_1
            elif os.path.exists(candidate_2):
                model_path = candidate_2
            else:
                raise FileNotFoundError(
                    "YOLO model not found. Please provide model_path in config.yaml."
                )

        # Load YOLO model
        self.model = YOLO(model_path)

        # Initialize camera interface
        self.cam = Webcam()

        # If True, convert frame to RGB before inference
        self.rgb = rgb

        # Track camera open state to prevent double-open bugs
        self.is_open = False

        # Open camera immediately so we fail early if device is missing
        self.open()

    def open(self) -> None:
        # Open camera only if not already opened
        if not self.is_open:
            self.cam.open()
            self.is_open = True

    def close(self) -> None:
        # Close camera safely so device is released between runs
        if self.is_open:
            self.cam.close()
            self.is_open = False

    def get_frame(self) -> Any:
        # Capture one frame from the camera
        if not self.is_open:
            raise RuntimeError("Camera is not open.")

        return self.cam.take_picture(rgb=self.rgb)

    def detect(self) -> Tuple[Any, Any, Any]:
        # Run YOLO inference on a captured frame
        frame = self.get_frame()

        # Perform model prediction (disable verbose spam)
        results = self.model.predict(frame, verbose=False)

        # Create annotated debug image for visualization
        annotated = results[0].plot()

        return frame, annotated, results