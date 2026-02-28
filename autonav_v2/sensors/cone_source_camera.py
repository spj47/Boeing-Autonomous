# Import libraries
import math
import time
from dataclasses import dataclass
from typing import List, Optional, Tuple

# Cone data type
from sensors.cone_source import ConeBox

# YOLO wrapper (camera + model inference)
from yolo import YOLODetector


@dataclass
class CameraConeConfig:
    # Camera geometry for simple ground-plane projection
    camera_height_m: float = 0.8
    camera_pitch_deg: float = 0.0
    horizontal_fov_deg: float = 70.0
    vertical_fov_deg: float = 55.0

    # Placeholder cone dimensions so downstream code has consistent fields
    assumed_cone_w_m: float = 0.15
    assumed_cone_d_m: float = 0.15
    assumed_cone_h_m: float = 0.0

    # Reject detections that project unrealistically close/far
    min_forward_m: float = 0.25
    max_forward_m: float = 30.0


class CameraConeSource:
    # Camera-only cone source for push testing
    def __init__(
        self,
        model_path: Optional[str] = None,
        rgb: bool = True,
        config: Optional[CameraConeConfig] = None,
    ) -> None:
        # Store config so projection behavior can be tuned from config.yaml
        self.cfg = config or CameraConeConfig()

        # Initialize YOLO detector (opens the camera internally)
        self.detector = YOLODetector(model_path=model_path, rgb=rgb)

        # Track frame counters and timestamps for debug / logging
        self._frame_id = 0
        self._last_frame_time_s = 0.0

    def close(self) -> None:
        # Clean up camera resources so the device is released between runs
        self.detector.close()

    def _image_to_ground(
        self, px: float, py: float, img_w: int, img_h: int
    ) -> Tuple[Optional[float], Optional[float]]:
        # Convert an image pixel into an approximate ground-plane (forward, right) point
        h_fov = math.radians(self.cfg.horizontal_fov_deg)
        v_fov = math.radians(self.cfg.vertical_fov_deg)
        pitch = math.radians(self.cfg.camera_pitch_deg)

        # Normalize pixel coordinates to [-1, 1]
        nx = (px / float(img_w)) * 2.0 - 1.0
        ny = 1.0 - (py / float(img_h)) * 2.0

        # Convert normalized coordinates into ray angles
        theta_x = nx * (h_fov / 2.0)
        theta_y = ny * (v_fov / 2.0)

        # Combine camera pitch with vertical ray angle
        total_vertical_angle = pitch + theta_y

        # If the ray never intersects the ground in front of the camera, reject it
        if total_vertical_angle <= 0.0:
            return None, None

        # Project ray onto ground plane assuming a flat surface
        forward_m = self.cfg.camera_height_m / math.tan(total_vertical_angle)
        right_m = forward_m * math.tan(theta_x)

        # Reject projections that are outside our useful range
        if not (self.cfg.min_forward_m <= forward_m <= self.cfg.max_forward_m):
            return None, None

        return forward_m, right_m

    def get_frame(self) -> Tuple[List[ConeBox], float, str]:
        # Run one camera + YOLO inference step and return cone boxes with metadata
        frame_time_s = time.time()

        # Run detection (returns raw frame, annotated frame, and YOLO results)
        frame, _annotated, results = self.detector.detect()

        # Get image dimensions for pixel->ground projection math
        img_h, img_w = frame.shape[0], frame.shape[1]

        # Build ConeBox outputs that match the rest of autonav_v2
        cone_boxes: List[ConeBox] = []

        # Walk each detection box from YOLO
        for box in results[0].boxes:
            # Check class label name so we only keep "cone" detections
            cls_id = int(box.cls[0])
            label = str(self.detector.model.names.get(cls_id, ""))
            if label.lower() != "cone":
                continue

            # Extract the bounding box corners (x1,y1,x2,y2)
            x1, y1, x2, y2 = box.xyxy[0]

            # Use bottom-center as a better ground contact point than bbox center
            cx_px = float((x1 + x2) / 2.0)
            cy_px = float(y2)

            # Convert the pixel into an estimated (forward, right) ground point
            forward_m, right_m = self._image_to_ground(cx_px, cy_px, img_w, img_h)
            if forward_m is None or right_m is None:
                continue

            # Package into ConeBox: (xc, yc, zc, w, d, h)
            # We treat xc as right_m and yc as forward_m to match downstream logic
            cone_boxes.append(
                (
                    right_m,
                    forward_m,
                    0.0,
                    self.cfg.assumed_cone_w_m,
                    self.cfg.assumed_cone_d_m,
                    self.cfg.assumed_cone_h_m,
                )
            )

        # Increment frame counters and return results with metadata
        self._frame_id += 1
        self._last_frame_time_s = frame_time_s
        return cone_boxes, frame_time_s, f"cam_{self._frame_id}"

    def get_cones(self) -> List[ConeBox]:
        # Compatibility helper for code paths that only expect a cone list
        cones, _t, _id = self.get_frame()
        return cones