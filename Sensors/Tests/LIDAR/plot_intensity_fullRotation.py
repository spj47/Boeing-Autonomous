import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from ld14_driver import LD14Driver

PORT = "/dev/cu.usbserial-0001"
driver = LD14Driver(PORT)

PLOT_LIMITS = 0.1

# Generator that yields full rotations of data
rotation_gen = driver.read_rotation()

# -- Plotting --
plt.style.use("dark_background")
fig, ax = plt.subplots()

# Initialize scatter plot with empty data
scat = ax.scatter([], [], s=5, c=[], cmap='viridis', vmin=0, vmax=255)

ax.set_aspect("equal")
ax.set_xlim(-PLOT_LIMITS, PLOT_LIMITS)
ax.set_ylim(-PLOT_LIMITS, PLOT_LIMITS)
ax.set_title("LD14 Live LiDAR Scan (Intensity)")
ax.set_xlabel("X (m)")
ax.set_ylabel("Y (m)")

def update(_):
    try:
        rotation_points = next(rotation_gen)
    except StopIteration:
        return scat,

    if not rotation_points:
        return scat,

    # Convert polar coordinates (angle, distance) to Cartesian coordinates
    angles_deg, distances, intensities = zip(*rotation_points)
    angles_rad = np.radians(angles_deg)
    xs = np.array(distances) * np.cos(angles_rad)
    ys = np.array(distances) * np.sin(angles_rad)

    offsets = np.column_stack((xs, ys))
    scat.set_offsets(offsets)
    scat.set_array(np.array(intensities))

    return scat,

ani = FuncAnimation(fig, update, interval=50)
plt.show()
