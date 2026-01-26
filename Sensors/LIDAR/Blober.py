import numpy as np
from collections import defaultdict
from itertools import product


# Predining Neighbors offset to reduce overhead
NEIGHBOR_OFFSETS_26 = [
    (dx, dy, dz)
    for dx, dy, dz in product([-1, 0, 1], repeat=3)
    if (dx, dy, dz) != (0, 0, 0)
]


def voxelize(points, voxel_size=0.02, min_points=5):
    """
    This takes in the 3D point cloud (points), divides it into voxel_size cubes, checks if there are 
    at least min_points in that cube; and, if there are, then that cube is filled and
    all points in that cube are discarded. 
        This helps a ton with reducing computation in blobing AND to reduce noise.

    Args:
        points (np.ndarray): Array of 3D vectors that make up the point cloud
        voxel_size (float): Edge length of each voxel in meters
        min_points (int): Minimum number of points required for a voxel
                          to be considered "Full".

    Returns:
        voxel_centers (np.ndarray): Array of 3D vectors that define centers of all filled voxels
        voxel_bounds (dict) -->!!UNUSED WAS PART OF OLD DESIGN!!<-- : 
                            A lookup table that tells you, for each voxel
                            (indexed by ix, iy, iz), the minimum and maximum
                            XYZ coordinates of the points that ended up
                            inside that voxel.
    """
    voxels = defaultdict(list)

    # Assign points to voxel indices
    for point in points:
        voxel_idx = (
            int(np.floor(point[0] / voxel_size)),
            int(np.floor(point[1] / voxel_size)),
            int(np.floor(point[2] / voxel_size))
        )
        voxels[voxel_idx].append(point)

    voxel_centers = []
    voxel_bounds = {}

    # Compute bounds and centers for valid voxels
    for voxel_idx, voxel_points in voxels.items():
        if len(voxel_points) < min_points:
            # Skip sparse voxels to suppress noise and reduce computation
            continue

        voxel_points = np.asarray(voxel_points)
        vmin = voxel_points.min(axis=0)
        vmax = voxel_points.max(axis=0)
        center = (vmin + vmax) / 2

        voxel_centers.append(center)
        voxel_bounds[voxel_idx] = (vmin, vmax)

    return np.asarray(voxel_centers), voxel_bounds


def blob_voxels(voxel_centers, voxel_size):
    """
    Groups touching voxels into a single obstacle or "blob"

    Voxels are considered touching if they are 26-connected
    (faces, edges, or corners). [see cached variable at the top]

    Args:
        voxel_centers (np.ndarray): Array of 3D vectors that define centers of all filled voxels
        voxel_size (float): Edge length of each voxel in meters

    Returns:
        blobs (list[np.ndarray]): A list of blobs, which contains an array of 3D vectors that 
                                  define centers of all voxels in the blob

    """
    if len(voxel_centers) == 0:
        return []

    # Quantize voxel centers back to integer voxel indices
    # Using indices avoids floating-point drift issues (Only really a problem in the previous design)
    voxel_index_to_center = {
        tuple(np.floor(center / voxel_size).astype(int)): center
        for center in voxel_centers
    }

    visited = set()
    blobs = []

    # Depth-first search over connected voxel indices
    for start_idx in voxel_index_to_center:
        if start_idx in visited:
            continue

        stack = [start_idx]
        blob_centers = []

        while stack:
            current = stack.pop()
            if current in visited:
                continue

            visited.add(current)
            blob_centers.append(voxel_index_to_center[current])

            for dx, dy, dz in NEIGHBOR_OFFSETS_26:
                neighbor = (
                    current[0] + dx,
                    current[1] + dy,
                    current[2] + dz
                )
                if neighbor in voxel_index_to_center and neighbor not in visited:
                    stack.append(neighbor)

        blobs.append(np.asarray(blob_centers))

    return blobs


# ========================
# Public API
# ========================

def blob_points_voxelized(points, voxel_size=0.02, min_points=5):
    """
    Converts a point cloud into voxeles, then blobs and then returns their bounding boxes.

    Each bounding box is represented as:
        (center_x, center_y, center_z,
         half_width, half_height, half_depth)

    The width, height, and depth values are *half-extents* — meaning they
    describe how far the blob extends from its center in each direction,
    not the full size

    Args:
        points (np.ndarray): Array of 3D vectors that make up the point cloud
                             you want to process.
        voxel_size (float): Edge length of each voxel in meters.
        min_points (int): Minimum number of points required for a voxel
                          to be considered "full"

    Returns:
        blobs (list[tuple]): A list of bounding boxes, one per blob, where
                             each box describes the blobs' position and
                             overall size in space
    """
    voxel_centers, _ = voxelize(
        points,
        voxel_size=voxel_size,
        min_points=min_points
    )

    voxel_blobs = blob_voxels(
        voxel_centers,
        voxel_size
    )

    blobs = []

    for blob_centers in voxel_blobs:
        vmin = blob_centers.min(axis=0)
        vmax = blob_centers.max(axis=0)

        center = (vmin + vmax) / 2
        half_extents = (vmax - vmin) / 2

        blobs.append((
            center[0], center[1], center[2],
            half_extents[0], half_extents[1], half_extents[2]
        ))

    return blobs
