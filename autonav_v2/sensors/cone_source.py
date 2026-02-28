# Import libraries
from typing import List, Protocol, Tuple


# A cone detection is represented as:
# (xc, yc, zc, w, d, h)
#
# xc, yc, zc are the cone center position in the sensor frame (meters)
# w, d, h are the bounding box dimensions (meters), if available
ConeBox = Tuple[float, float, float, float, float, float]


class ConeSource(Protocol):

    # This is the common interface for anything that provides cones to navigation
    # File replay, LiDAR pipelines, camera pipelines, and fused pipelines should all match this

    def get_cones(self) -> List[ConeBox]:
        # Return the current list of detected cones as ConeBox tuples
        ...