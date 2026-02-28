# Import libraries
from dataclasses import dataclass
from typing import Tuple


@dataclass
class FrameConfig:

    # Defines how we interpret raw sensor coordinates
    # forward_axis tells us which sensor axis represents vehicle forward
    # forward_sign corrects flipped sensor orientation
    # right_sign ensures +right always means vehicle right

    forward_axis: str
    forward_sign: int
    right_sign: int


def to_vehicle_frame(
    xc: float,
    yc: float,
    frame_config: FrameConfig,
) -> Tuple[float, float]:

    # Validate configuration early so misconfigurations fail fast
    if frame_config.forward_axis not in ("x", "y"):
        raise ValueError("forward_axis must be 'x' or 'y'")

    # If sensor forward is aligned with its y-axis
    if frame_config.forward_axis == "y":

        # Apply sign corrections so forward and right follow vehicle conventions
        forward_m = frame_config.forward_sign * yc
        right_m = frame_config.right_sign * xc

    else:
        # Sensor forward is aligned with its x-axis
        forward_m = frame_config.forward_sign * xc
        right_m = frame_config.right_sign * yc

    # Return coordinates in vehicle frame
    # right_m > 0 means vehicle right
    # forward_m > 0 means vehicle forward
    return right_m, forward_m