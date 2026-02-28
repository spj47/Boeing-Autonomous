# Import libraries
import math


def steer_deg_from_target(
    right_m: float,
    fwd_m: float,
    lookahead_m: float,
) -> float:

    # Clamp lookahead so we do not blow up on tiny values
    lookahead_dist_m = max(0.25, float(lookahead_m))

    # Compute distance to the target so we can estimate curvature safely
    target_dist_m = math.hypot(right_m, fwd_m)

    # Protect against divide-by-zero when the target is extremely close
    target_dist_m = max(1e-3, target_dist_m)

    # Estimate curvature using a simple pure pursuit approximation
    # Larger lateral offset or closer target should create a stronger steering response
    curvature = 2.0 * right_m / (target_dist_m * target_dist_m)

    # Convert curvature into a stable steering angle proxy using lookahead
    steering_rad = math.atan(curvature * lookahead_dist_m)

    # Output degrees so it is easier to reason about and easier to map to hardware
    steering_deg = math.degrees(steering_rad)
    return float(steering_deg)


def steer_percent_from_deg(
    steering_deg: float,
    max_wheel_deg: float,
    invert: bool,
) -> int:

    # Clamp max wheel angle so we never divide by zero
    max_allowed_deg = max(1e-3, float(max_wheel_deg))

    # Clamp requested steering to what the hardware can actually do
    clamped_steering_deg = max(
        -max_allowed_deg,
        min(max_allowed_deg, float(steering_deg)),
    )

    # Map [-max_deg, +max_deg] into [0, 100] with 50 as centered
    steering_percent = 50.0 + (clamped_steering_deg / max_allowed_deg) * 50.0

    # Allow inversion so wiring differences do not require changing math logic
    if invert:
        steering_percent = 100.0 - steering_percent

    # Clamp one more time in case rounding pushes us out of bounds
    steering_percent = max(0.0, min(100.0, steering_percent))

    # Return an integer command because serial protocols are usually integer-based
    return int(round(steering_percent))