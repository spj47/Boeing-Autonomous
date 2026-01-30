# -- Scan to Point Cloud Test --
import numpy as np


# -- Helper functions --
def lidar_scan_to_pointcloud(rotation_points, tilt_deg, z_offset_lidar):
    """
    Convert a single 2D LiDAR rotation scan into a 3D point cloud.

    Args:
        rotation_points : list of tuples
                          LiDAR scan data in the form:
                          [(x, y, intensity), ...]

                          Coordinates are assumed to already be expressed in the
                          LiDAR’s local 2D coordinate frame.

        tilt_deg : float
                   Servo tilt angle in degrees.

        z_offset_lidar : float
                         Vertical offset of the LiDAR from the rotation axis (meters).

    Returns:
        pts_3d : (N, 3) np.ndarray
                        3D point cloud in the LiDAR frame after tilt projection.

        intensities : (N,) np.ndarray
                           Corresponding intensity values.
    """
    if not rotation_points:
        return None, None

    # Unpack scan data
    xs, ys, intensities = zip(*rotation_points)
    xs = np.asarray(xs)
    ys = np.asarray(ys)
    intensities = np.asarray(intensities)

    # Convert tilt angle to radians
    phi = np.deg2rad(tilt_deg)
    sin_phi = np.sin(phi)
    cos_phi = np.cos(phi)

    # Project 2D scan into 3D using tilt angle
    x3d = xs
    y3d = cos_phi * ys + (z_offset_lidar * sin_phi)
    z3d = sin_phi * ys + (z_offset_lidar * cos_phi)

    pts_3d = np.column_stack((x3d, y3d, z3d))
    return pts_3d, intensities


# -- Test cases --
print("Running test...")

known_points = [
    (0, 0, 0)
]

pts, intensities = lidar_scan_to_pointcloud(
    known_points,
    tilt_deg=0,
    z_offset_lidar=0.0381
)

print("3D Points:")
print(pts)
