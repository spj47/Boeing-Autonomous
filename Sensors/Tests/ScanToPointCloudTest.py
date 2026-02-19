import numpy as np
import matplotlib.pyplot as plt

def lidar_point(tilt_deg, angle_deg, distance):
    """
    Compute a LiDAR point using rotation matrices
    """

    # Convert to radians
    a = np.deg2rad(angle_deg + 90) # +90 to have Y+ be forward 
    tilt = np.deg2rad(tilt_deg + 180) # +180 since that's how the servos are mounted

    # Ray in LiDAR frame before tilt
    # Angle rotates around Z, so ray lies in XY plane
    ray = np.array([
        np.cos(a),   # x
        np.sin(a),   # y
        0.0          # z
    ])

    # Rotation around X axis (tilt)
    Rx = np.array([
        [1, 0, 0],
        [0, np.cos(tilt), -np.sin(tilt)],
        [0, np.sin(tilt),  np.cos(tilt)]
    ])

    # Apply tilt
    tilted_ray = Rx @ ray

    # Scale by distance
    point = distance * tilted_ray

    return point[0], point[1], point[2]



def animate_to(ax, scatter, line, current, target, steps=50):
    """Smoothly animate from current value to target value."""
    for i in range(1, steps + 1):
        value = current + (target - current) * (i / steps)
        x, y, z = lidar_point(*value)

        # Update point
        scatter._offsets3d = ([x], [y], [z])

        # Update vector line from origin
        line.set_data([0, x], [0, y])
        line.set_3d_properties([0, z])

        # Keep axes equal
        max_range = max(abs(x), abs(y), abs(z), 1)
        ax.set_xlim(-max_range, max_range)
        ax.set_ylim(-max_range, max_range)
        ax.set_zlim(-max_range, max_range)

        plt.pause(0.01)

    return target


def main():
    # Initial values
    tilt = 0
    angle = 0
    distance = 5

    fig = plt.figure(figsize=(6, 6))
    ax = fig.add_subplot(111, projection="3d")

    x, y, z = lidar_point(tilt, angle, distance)

    # Red point
    scatter = ax.scatter([x], [y], [z], c="red", s=50)

    # Blue vector line from origin
    line, = ax.plot([0, x], [0, y], [0, z], c="blue")

    plt.ion()
    plt.show()

    current = np.array([tilt, angle, distance])

    print("Enter commands like:")
    print("  T 10   → set tilt to 10°")
    print("  A 90   → set angle to 90°")
    print("  D 7    → set distance to 7")
    print("Press Ctrl+C to exit.\n")

    while True:
        cmd = input("Command: ").strip().split()

        if len(cmd) != 2:
            print("Invalid format. Use: T/A/D number")
            continue

        axis, value = cmd[0].upper(), float(cmd[1])

        target = current.copy()

        if axis == "T":
            target[0] = value
        elif axis == "A":
            target[1] = value
        elif axis == "D":
            target[2] = value
        else:
            print("Unknown command. Use T, A, or D.")
            continue

        current = animate_to(ax, scatter, line, current, target)


if __name__ == "__main__":
    main()
