import numpy as np
import serial
import time
import matplotlib.pyplot as plt
from collections import deque

# -- Hardware variables --
IMU_PORT = "/dev/cu.usbserial-1420"
BAUDRATE = 115200

# -- Visualization parameters --
AXIS_LEN   = 0.5     # Length of orientation axes
ACC_SCALE  = 0.2     # Scaling factor for acceleration arrow
IMU_BUFFER = 500     # Number of IMU samples to store

# -- Filtering parameters --
LPF_ALPHA  = 0.05    # Low-pass filter coefficient
ACC_THRESH = 0.05    # Minimum acceleration magnitude to visualize (g-units)

# -- Initialize hardware --
print("Initializing IMU...")
imu = serial.Serial(IMU_PORT, BAUDRATE, timeout=0.01)
time.sleep(2)  # Give the IMU time to boot and start streaming


# -- Helper functions --
def read_imu_nonblocking(ser, buffer: deque):
    """
    Read all currently available IMU data without blocking.

    Each valid line is expected to contain:
        roll, pitch, yaw, ax, ay, az

    Values are stored along with a timestamp so the most
    recent measurement can always be accessed.
    """
    while ser.in_waiting:
        line = ser.readline().decode().strip()
        try:
            values = list(map(float, line.split(",")))
            if len(values) != 6:
                continue

            r, p, y, ax, ay, az = values
            buffer.append((time.time(),
                           np.array([r, p, y, ax, ay, az])))
        except:
            # Ignore malformed or partial lines
            continue


def rotation_matrix(roll, pitch, yaw):
    """
    Construct a 3x3 rotation matrix from roll, pitch, and yaw
    angles (in radians), using ZYX rotation order.
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


# -- Data buffers --
imu_buffer = deque(maxlen=IMU_BUFFER)

# -- Plot setup --
plt.ion()
fig = plt.figure(figsize=(7, 7))
ax_plot = fig.add_subplot(111, projection="3d")

# Fix axes so they don’t reset every frame
ax_plot.set_xlim(-1, 1)
ax_plot.set_ylim(-1, 1)
ax_plot.set_zlim(-1, 1)

ax_plot.set_xlabel("X")
ax_plot.set_ylabel("Y")
ax_plot.set_zlabel("Z")
ax_plot.set_title("IMU Debug: Acceleration & Orientation (g-units)")

# World-frame reference axes
ax_plot.quiver(0, 0, 0, AXIS_LEN, 0, 0, color='r')
ax_plot.quiver(0, 0, 0, 0, AXIS_LEN, 0, color='g')
ax_plot.quiver(0, 0, 0, 0, 0, AXIS_LEN, color='b')

# -- State variables --
acc_smooth = np.zeros(3)  # Smoothed acceleration (world frame)

print("Streaming IMU data...")


# -- Main loop --
try:
    while True:
        # Pull in any new IMU data
        read_imu_nonblocking(imu, imu_buffer)
        if len(imu_buffer) == 0:
            continue

        # Use most recent IMU sample
        _, imu_data = imu_buffer[-1]
        roll, pitch, yaw, ax, ay, az = imu_data
        roll, pitch, yaw = np.deg2rad([roll, pitch, yaw])

        # Compute IMU orientation
        R = rotation_matrix(roll, pitch, yaw)

        # Rotate acceleration into world frame
        acc_body = np.array([ax, ay, az])      # Already in g-units
        acc_world = R @ acc_body

        # Remove gravity (assumed +Z in world frame)
        acc_world_nog = acc_world - np.array([0, 0, 1.0])

        # Low-pass filter acceleration for visualization stability
        acc_smooth = (1 - LPF_ALPHA) * acc_smooth + LPF_ALPHA * acc_world_nog

        # Clear dynamic plot elements
        ax_plot.cla()

        # Restore static axis settings
        ax_plot.set_xlim(-1, 1)
        ax_plot.set_ylim(-1, 1)
        ax_plot.set_zlim(-1, 1)
        ax_plot.set_xlabel("X")
        ax_plot.set_ylabel("Y")
        ax_plot.set_zlabel("Z")
        ax_plot.set_title("IMU Debug: Acceleration & Orientation (g-units)")

        # Draw world-frame axes
        ax_plot.quiver(0, 0, 0, AXIS_LEN, 0, 0, color='r')
        ax_plot.quiver(0, 0, 0, 0, AXIS_LEN, 0, color='g')
        ax_plot.quiver(0, 0, 0, 0, 0, AXIS_LEN, color='b')

        # Draw IMU orientation axes (dashed)
        ax_plot.quiver(0, 0, 0, *(R[:, 0] * AXIS_LEN), color='r', linestyle='--')
        ax_plot.quiver(0, 0, 0, *(R[:, 1] * AXIS_LEN), color='g', linestyle='--')
        ax_plot.quiver(0, 0, 0, *(R[:, 2] * AXIS_LEN), color='b', linestyle='--')

        # Draw acceleration vector (if significant)
        if np.linalg.norm(acc_world_nog) > ACC_THRESH:
            ax_plot.quiver(
                0, 0, 0,
                *(acc_smooth * ACC_SCALE),
                color='yellow',
                linewidth=2
            )

        plt.draw()
        plt.pause(0.001)

except KeyboardInterrupt:
    print("\nStopped.")

plt.ioff()
plt.show()
