import numpy as np
import serial
import time
import matplotlib.pyplot as plt
from collections import deque
from ld14_driver import LD14Driver

# -- Hardware variables --
LIDAR_PORT = "/dev/cu.usbserial-0001"
IMU_PORT   = "/dev/cu.usbserial-1420"

MAX_SCANS  = 1          # Number of LiDAR scans to keep and display to avoid my computer blowing up
PLOT_LIMIT = 1          # Size of the plot at the end
AXIS_LEN   = 0.05       # Length of arrows showing the rotation offset of the LIDAR
IMU_BUFFER = 500        # Number of IMU datasets to store in buffer and interpolate against

# -- Initialize hardware --
print("Initializing LiDAR...")
driver = LD14Driver(LIDAR_PORT)
rotation_gen = driver.read_rotation()

print("Initializing IMU...")
imu = serial.Serial(IMU_PORT, 115200, timeout=0.01)
time.sleep(2)  # Give the IMU time to boot and start streaming

# -- Helper functions --
def read_imu_nonblocking(ser, buffer: deque):
    """
    Read all currently available IMU data without blocking.

    Each valid line is expected to contain:
        roll, pitch, yaw, ax, ay, az

    Only orientation (roll, pitch, yaw) is used here. Values are stored
    along with a timestamp so they can be interpolated later.
    """
    while ser.in_waiting:
        line = ser.readline().decode().strip()
        try:
            values = list(map(float, line.split(",")))
            if len(values) != 6:
                continue
            r, p, y, *_ = values
            buffer.append((time.time(), np.array([r, p, y])))
        except:
            # Ignore any bad lines of data
            continue

def rotation_matrix(roll, pitch, yaw):
    """
    Construct a 3x3 rotation matrix from roll, pitch, and yaw angles
    (in radians), using ZYX rotation order.
    """
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)

    Rz = np.array([[cy, -sy, 0],
                   [sy,  cy, 0],
                   [ 0,   0, 1]])

    Ry = np.array([[ cp, 0, sp],
                   [  0, 1,  0],
                   [-sp, 0, cp]])

    Rx = np.array([[1,  0,   0],
                   [0, cr, -sr],
                   [0, sr,  cr]])

    return Rz @ Ry @ Rx


def lidar_scan_to_points(rotation_points):
    """
    Convert a single LiDAR rotation scan into XYZ points
    in the LiDAR’s local coordinate frame.
    """
    angles, distances, _ = zip(*rotation_points)
    a = np.deg2rad(angles)
    d = np.asarray(distances)

    x =  d * np.cos(a)  
    y = -d * np.sin(a) 
    z = np.zeros_like(x)

    return np.column_stack((x, y, z))

# -- Data buffers --
scan_buffer = deque(maxlen=MAX_SCANS)   # Recent LiDAR scans after being converted to  world frame
imu_buffer  = deque(maxlen=IMU_BUFFER)         # Recent IMU samples for interpolation

# -- Plot setup --
plt.style.use("dark_background") # Please don't change this :)
plt.ion()

fig = plt.figure(figsize=(9, 7))
ax_plot = fig.add_subplot(111, projection="3d")

# Fix axes once so they don’t reset every frame
ax_plot.set_xlim(-PLOT_LIMIT, PLOT_LIMIT)
ax_plot.set_ylim(-PLOT_LIMIT, PLOT_LIMIT)
ax_plot.set_zlim(-PLOT_LIMIT, PLOT_LIMIT)

ax_plot.set_xlabel("X (right)")
ax_plot.set_ylabel("Y (forward)")
ax_plot.set_zlabel("Z (up)")
ax_plot.set_title("World-Fixed LiDAR–IMU View (Rotation Only)")

# Initialize empty point cloud scatter 
# Without caching like this; it becomes too much and the plot become desynced
scatter = ax_plot.scatter([], [], [], s=3, c="cyan")

# Setup arrows that show the current rotation of the LIDAR
imu_quivers = {
    'x': ax_plot.quiver(0, 0, 0, AXIS_LEN, 0, 0, color='r', linestyle='--'),
    'y': ax_plot.quiver(0, 0, 0, 0, AXIS_LEN, 0, color='g', linestyle='--'),
    'z': ax_plot.quiver(0, 0, 0, 0, 0, AXIS_LEN, color='b', linestyle='--')
}

# Setup arrows that align with the world coordinates 
ax_plot.quiver(0, 0, 0, AXIS_LEN, 0, 0, color='r')
ax_plot.quiver(0, 0, 0, 0, AXIS_LEN, 0, color='g')
ax_plot.quiver(0, 0, 0, 0, 0, AXIS_LEN, color='b')


# -- Initial yaw reference --
initial_yaw = None  # Used to lock the world frame to the first IMU reading

print("Scanning...")


# -- Main loop --
try:
    while True:
        # Pull in any new IMU data
        read_imu_nonblocking(imu, imu_buffer)

        # Skip if no IMU data yet
        if len(imu_buffer) == 0:
            continue

        # Get the most recent IMU reading
        roll, pitch, yaw = np.deg2rad(imu_buffer[-1][1])

        # Get the next LiDAR rotation scan
        try:
            rotation = next(rotation_gen)
        except StopIteration:
            break

        # Establish world frame using initial yaw
        if initial_yaw is None:
            initial_yaw = yaw

        R0_inv = rotation_matrix(0, 0, initial_yaw).T
        R_imu  = rotation_matrix(roll, pitch, yaw)

        # Apply rotation 
        R_world = R0_inv @ R_imu

        # Transform LiDAR points into world frame
        pts = lidar_scan_to_points(rotation)
        pts_world = (R_world @ pts.T).T
        scan_buffer.append(pts_world)

        # Update point cloud visualization
        all_pts = np.vstack(scan_buffer)
        scatter._offsets3d = (all_pts[:, 0], all_pts[:, 1], all_pts[:, 2])

        # Update IMU orientation axes
        for axis in imu_quivers.values():
            axis.remove()

        imu_quivers['x'] = ax_plot.quiver(0, 0, 0, *(R_imu[:, 0] * AXIS_LEN), color='r', linestyle='--')
        imu_quivers['y'] = ax_plot.quiver(0, 0, 0, *(R_imu[:, 1] * AXIS_LEN), color='g', linestyle='--')
        imu_quivers['z'] = ax_plot.quiver(0, 0, 0, *(R_imu[:, 2] * AXIS_LEN), color='b', linestyle='--')

        plt.draw()
        plt.pause(0.001)

except KeyboardInterrupt:
    print("\nStopped.")


plt.ioff()
plt.show()
