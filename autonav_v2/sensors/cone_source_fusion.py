# Import libraries
import time
from dataclasses import dataclass
from typing import List, Optional, Tuple

# Cone data type
from sensors.cone_source import ConeBox

# Sensor fusion pipeline (camera + LiDAR)
from Sensors_Main import Sensors


@dataclass
class FusionConfig:
    # LD14 port on Pi: "/dev/ttyUSB0" is common; set this in config.yaml.
    lidar_port: str = "/dev/ttyUSB0"

    # Only output obstacles tagged as cones by fusion matching
    cones_only: bool = True

    # Optional filters to keep junk out of nav
    min_range_m: float = 0.20
    max_range_m: float = 30.0

    # If you want to cap how many cones go into nav/log
    max_cones: int = 50


class FusionConeSource:
    # Fusion cone source for push testing (camera labels + LiDAR geometry)
    def __init__(self, config: Optional[FusionConfig] = None) -> None:
        # Store config so filtering behavior can be tuned from config.yaml
        self.cfg = config or FusionConfig()

        # Initialize the fusion pipeline (opens LiDAR serial + camera/YOLO internally)
        self.sensors = Sensors(self.cfg.lidar_port)

        # Track frame counters and timestamps for debug / logging
        self._frame_id = 0
        self._last_frame_time_s = 0.0

    def close(self) -> None:
        # Clean up subcomponents so the camera and serial port are released between runs
        # Sensors_Main doesn't define a close() method, so we safely close parts if present.

        try:
            if hasattr(self.sensors, "yolo") and hasattr(self.sensors.yolo, "close"):
                self.sensors.yolo.close()
        except Exception:
            pass

        try:
            if hasattr(self.sensors, "lidar") and hasattr(self.sensors.lidar, "close"):
                self.sensors.lidar.close()
        except Exception:
            pass

    @staticmethod
    def _bbox_to_conebox(box: dict) -> ConeBox:
        # Convert a fused LiDAR bounding box into a ConeBox tuple
        min_x = float(box["min_x"])
        max_x = float(box["max_x"])
        min_y = float(box["min_y"])
        max_y = float(box["max_y"])

        # Compute box center (meters)
        xc = 0.5 * (min_x + max_x)
        yc = 0.5 * (min_y + max_y)

        # Compute box extents (meters)
        w = max_x - min_x
        d = max_y - min_y

        # Package into ConeBox: (xc, yc, zc, w, d, h)
        return (xc, yc, 0.0, w, d, 0.0)

    def get_frame(self) -> Tuple[List[ConeBox], float, str]:
        # Run one fusion step (YOLO + LiDAR) and return cone boxes with metadata

        # Sensors_Main returns (timestamp, obstacles) where each obstacle has:
        #   min_x/max_x/min_y/max_y and is_cone True/False.
        timestamp, obstacles = self.sensors.GetObstacles(useCamera=True)

        # Use sensor timestamp if provided; otherwise fall back to wall time
        frame_time_s = float(timestamp) if timestamp is not None else time.time()

        # Convert fused obstacles into ConeBoxes
        cones: List[ConeBox] = []

        for box in obstacles:
            # Optionally filter to cones only (as labeled by camera matching)
            is_cone = bool(box.get("is_cone", False))
            if self.cfg.cones_only and not is_cone:
                continue

            # Convert bbox dictionary into ConeBox tuple
            conebox = self._bbox_to_conebox(box)

            # Range filter on center distance to reduce junk
            xc, yc = conebox[0], conebox[1]
            rng = (xc * xc + yc * yc) ** 0.5
            if rng < self.cfg.min_range_m or rng > self.cfg.max_range_m:
                continue

            cones.append(conebox)

            # Cap cone output so a bad frame cannot explode logs or computations
            if len(cones) >= self.cfg.max_cones:
                break

        # Increment frame counters and return results with metadata
        self._frame_id += 1
        self._last_frame_time_s = frame_time_s
        return cones, frame_time_s, f"fusion_{self._frame_id}"

    def get_cones(self) -> List[ConeBox]:
        # Compatibility helper for code paths that only expect a cone list
        cones, _t, _id = self.get_frame()
        return cones