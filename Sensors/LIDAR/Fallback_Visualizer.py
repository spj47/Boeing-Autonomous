import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from matplotlib.animation import FuncAnimation
from LIDAR_Fallback import LidarObjectDetector

PORT = 'COM3'
PLOT_LIMITS = 5

detector = LidarObjectDetector(PORT)
rotation_gen = detector.detect_objects()

plt.style.use("dark_background")
fig, ax = plt.subplots()

scat = ax.scatter([], [], s=5, c=[], cmap='viridis', vmin=0, vmax=255)
ax.set_aspect("equal")
ax.set_xlim(-PLOT_LIMITS, PLOT_LIMITS)
ax.set_ylim(-PLOT_LIMITS, PLOT_LIMITS)
ax.set_title("LD14 Live LiDAR Scan with Bounding Boxes")
ax.set_xlabel("X (m)")
ax.set_ylabel("Y (m)")

rect_patches = []

def update(_):
    try:
        bounding_boxes = next(rotation_gen)
    except StopIteration:
        return scat,

    points_x = []
    points_y = []
    intensities = []

    for bbox in bounding_boxes:
        cluster_points = bbox['points']
        xs, ys, ints = zip(*cluster_points)
        points_x.extend(xs)
        points_y.extend(ys)
        intensities.extend(ints)

    if points_x:
        offsets = np.column_stack((points_x, points_y))
        scat.set_offsets(offsets)
        scat.set_array(np.array(intensities))

    for rect in rect_patches:
        rect.remove()
    rect_patches.clear()

    for bbox in bounding_boxes:
        x0, x1 = bbox['min_x'], bbox['max_x']
        y0, y1 = bbox['min_y'], bbox['max_y']
        rect = Rectangle((x0, y0), x1 - x0, y1 - y0, edgecolor='red', facecolor='none', linewidth=1.5)
        ax.add_patch(rect)
        rect_patches.append(rect)

    return scat, *rect_patches

ani = FuncAnimation(fig, update, interval=50)
plt.show()
