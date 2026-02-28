# Import libraries
import time
from dataclasses import dataclass
from typing import List, Optional, Tuple

# Cone data type
from sensors.cone_source import ConeBox

# LiDAR clustering / fallback detector (LD14 -> clustered bounding boxes)
from LIDAR_Fallback import LidarObjectDetector


@dataclass
class LidarConeConfig:
    # Serial port to the LD14 on the Pi. Common examples:
    #   "/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/serial0"
    port: str = "/dev/ttyUSB0"
    baudrate: int = 230400

    # Cluster tuning (passed through to detector)
    min_cluster_size: int = 5
    cluster_distance_threshold: float = 0.08

    # Filter objects by center distance (meters)
    min_range_m: float = 0.20
    max_range_m: float = 30.0

    # Optional additional filters for “reasonable” boxes
    max_box_width_m: float = 1.5
    max_box_depth_m: float = 1.5


class LidarConeSource:
    # LiDAR-only cone/obstacle source for push testing
    def __init__(self, config: Optional[LidarConeConfig] = None) -> None:
        # Store config so clustering and filters can be tuned from config.yaml
        self.cfg = config or LidarConeConfig()

        # Initialize LiDAR object detector (opens the serial port internally)
        # This detector clusters LD14 points into bounding boxes in meters.
        self.detector = LidarObjectDetector(
            port=self.cfg.port,
            baudrate=self.cfg.baudrate,
            min_cluster_size=self.cfg.min_cluster_size,
            cluster_distance_threshold=self.cfg.cluster_distance_threshold,
        )

        # Track frame counters and timestamps for debug / logging
        self._frame_id = 0
        self._last_frame_time_s = 0.0

    def close(self) -> None:
        # Clean up serial resources so the device is released between runs
        self.detector.close()

    def get_frame(self) -> Tuple[List[ConeBox], float, str]:
        # Read one full LiDAR detection cycle and return cone boxes with metadata
        frame_time_s = time.time()

        # Run clustering to detect object bounding boxes
        objects = self.detector.detect_objects()

        # Convert bounding boxes into ConeBox tuples used by autonav_v2
        cones: List[ConeBox] = []

        for obj in objects:
            # Extract bounding box edges
            min_x = float(obj["min_x"])
            max_x = float(obj["max_x"])
            min_y = float(obj["min_y"])
            max_y = float(obj["max_y"])

            # Compute box center (meters)
            xc = 0.5 * (min_x + max_x)  # right (+) / left (-) in LiDAR frame
            yc = 0.5 * (min_y + max_y)  # forward/back depending on your LiDAR mapping

            # Compute box extents (meters)
            w = max_x - min_x
            d = max_y - min_y

            # Reject objects that are too close or too far away
            rng = (xc * xc + yc * yc) ** 0.5
            if rng < self.cfg.min_range_m or rng > self.cfg.max_range_m:
                continue

            # Reject boxes that are unrealistically large for our use case
            if w > self.cfg.max_box_width_m or d > self.cfg.max_box_depth_m:
                continue

            # Package into ConeBox: (xc, yc, zc, w, d, h)
            cones.append((xc, yc, 0.0, w, d, 0.0))

        # Increment frame counters and return results with metadata
        self._frame_id += 1
        self._last_frame_time_s = frame_time_s
        return cones, frame_time_s, f"lidar_{self._frame_id}"

    def get_cones(self) -> List[ConeBox]:
        # Compatibility helper for code paths that only expect a cone list
        cones, _t, _id = self.get_frame()
        return cones