import numpy as np
import serial
import time
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from ld14_driver import LD14Driver
from Blober import blob_points_voxelized


# -- Hardware variables --
LIDAR_PORT = "/dev/cu.usbserial-0001"
SERVO_PORT = "/dev/cu.usbserial-1420"

SERVO_MIN   = 0        # Minimum tilt angle (deg)
SERVO_MAX   = 45       # Maximum tilt angle (deg)
SERVO_STEPS = 30       # Number of discrete tilt positions
SERVO_DELAY = 0.05     # Time to allow servo to settle (sec)


# -- Initialize hardware --
print("Initializing LiDAR...")
driver = LD14Driver(LIDAR_PORT)
rotation_gen = driver.read_rotation()

print("Initializing Servo...")
servo = serial.Serial(SERVO_PORT, 9600)
time.sleep(2)      # Allow Arduino reset
servo.write(b"0\n")
servo.flush()
time.sleep(5)      # Give servo time to home


# -- Buffers --
point_cloud     = []   # Accumulated XYZ points
intensity_cloud = []   # Corresponding intensity values


# -- Helpers --
def lidar_scan_to_pointcloud(rotation_points, tilt_deg):
    """
    Convert a single 2D LiDAR rotation scan into XYZ points
    given the current servo tilt angle.
    """
    if not rotation_points:
        return None, None

    angles, distances, intensities = zip(*rotation_points)

    a = np.deg2rad(angles)
    d = np.asarray(distances)
    i = np.asarray(intensities)

    tilt = np.deg2rad(tilt_deg + 90)

    x = d * np.sin(tilt) * np.cos(a)
    y = -d * np.sin(tilt) * np.sin(a)
    z = d * np.cos(tilt)

    return np.column_stack((x, y, z)), i


def draw_bounding_box(ax, cx, cy, cz, w, h, d, color="red"):
    """
    Draw an axis-aligned 3D bounding box centered at (cx, cy, cz)
    with half-dimensions (w, h, d).
    """
    x = [cx - w, cx + w]
    y = [cy - h, cy + h]
    z = [cz - d, cz + d]

    corners = np.array([
        [x[0], y[0], z[0]],
        [x[1], y[0], z[0]],
        [x[1], y[1], z[0]],
        [x[0], y[1], z[0]],
        [x[0], y[0], z[1]],
        [x[1], y[0], z[1]],
        [x[1], y[1], z[1]],
        [x[0], y[1], z[1]],
    ])

    edges = [
        (0,1),(1,2),(2,3),(3,0),
        (4,5),(5,6),(6,7),(7,4),
        (0,4),(1,5),(2,6),(3,7)
    ]

    for i, j in edges:
        ax.plot(
            [corners[i, 0], corners[j, 0]],
            [corners[i, 1], corners[j, 1]],
            [corners[i, 2], corners[j, 2]],
            color=color,
            linewidth=1.5
        )


# -- Sweep --
tilt_angles = np.linspace(SERVO_MIN, SERVO_MAX, SERVO_STEPS)

print("Starting sweep...")


# -- Main loop --
for tilt in tilt_angles:
    print(f"Sweeping at tilt angle: {tilt:.1f}°")

    servo.write(f"{int(tilt)}\n".encode())
    servo.flush()
    time.sleep(SERVO_DELAY)

    try:
        rotation = next(rotation_gen)
    except StopIteration:
        print("LiDAR stopped.")
        break

    pts, intensities = lidar_scan_to_pointcloud(rotation, tilt)
    if pts is None:
        continue

    point_cloud.append(pts)
    intensity_cloud.append(intensities)

print("Sweep complete.")


# -- Blob detection --
print("Detecting blobs...")

all_pts = np.vstack(point_cloud)

blobs = blob_points_voxelized(
    all_pts.tolist(),
    voxel_size=0.15,   # in meters
    min_points=5
)

print(f"Detected {len(blobs)} blobs")


# -- Plotting --
plt.style.use("dark_background")

fig = plt.figure(figsize=(9, 7))
ax = fig.add_subplot(111, projection="3d")

# Uncomment to view raw point cloud (It gets really noisy if this is left on)
# ax.scatter(all_pts[:,0], all_pts[:,1], all_pts[:,2], s=1, c="cyan")

for blob in blobs:
    cx, cy, cz, w, h, d = blob
    draw_bounding_box(ax, cx, cy, cz, w, h, d)

ax.set_xlim(-6, 6)
ax.set_ylim(-6, 6)
ax.set_zlim(-6, 6)

ax.set_xlabel("X (right)")
ax.set_ylabel("Y (forward)")
ax.set_zlabel("Z (height)")
ax.set_title("LD14 3D LiDAR Sweep with Bounding Boxes")

plt.show()
