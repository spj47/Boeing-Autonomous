import numpy as np
import serial
import time
from ld14_driver import LD14Driver
from Blober import blob_points_voxelized

class LD14Scanner:
    def __init__(self, lidar_port, servo_port, servo_min=135, servo_max=180, servo_steps=30, servo_delay=0.05):
        """
        Initializes LiDAR and servo.
        """
        self.lidar_port = lidar_port
        self.servo_port = servo_port
        self.servo_min = servo_min
        self.servo_max = servo_max
        self.servo_steps = servo_steps
        self.servo_delay = servo_delay

        # Initialize hardware
        self.driver = LD14Driver(lidar_port)
        self.rotation_gen = self.driver.read_rotation()

        self.servo = serial.Serial(servo_port, 9600)
        time.sleep(2)  # Arduino reset
        self.servo.write(b"170\n")
        self.servo.flush()
        time.sleep(5)  # Servo home

        # Precompute tilt angles
        self.tilt_angles = np.linspace(self.servo_min, self.servo_max, self.servo_steps)

        # NEW: track sweep direction (+1 = forward, -1 = backward)
        self.scan_direction = 1

    def _lidar_scan_to_pointcloud(self, rotation_points, tilt_deg):
        """
        Convert a single 2D LiDAR rotation scan into XYZ points,
        matching the logic of lidar_point().
        """
        if not rotation_points:
            return None

        angles, distances, _ = zip(*rotation_points)

        # Convert to radians and apply +90° shift (same as lidar_point)
        a = np.deg2rad(np.asarray(angles) + 90.0)
        d = np.asarray(distances)

        # Ray in LiDAR frame before tilt (same as lidar_point)
        x0 = d * np.cos(a)
        y0 = d * np.sin(a)
        z0 = np.zeros_like(d)

        # Tilt rotation around X axis
        tilt = np.deg2rad(tilt_deg + 180)
        Rx = np.array([
            [1, 0, 0],
            [0, np.cos(tilt), -np.sin(tilt)],
            [0, np.sin(tilt),  np.cos(tilt)]
        ])

        # Stack and rotate
        points = np.vstack((x0, y0, z0))
        points_tilted = Rx @ points

        return points_tilted.T



    def get_raw_points(self):
        """
        Perform a sweep and return all raw XYZ points as a single numpy array.
        Now supports bouncing between min and max tilt.
        """
        point_cloud = []

        if self.scan_direction == 1:
            angles = self.tilt_angles
        else:
            angles = self.tilt_angles[::-1]

        for tilt in angles:
            self.servo.write(f"{int(tilt)}\n".encode())
            self.servo.flush()
            time.sleep(self.servo_delay)

            try:
                rotation = next(self.rotation_gen)
            except StopIteration:
                break

            pts = self._lidar_scan_to_pointcloud(rotation, tilt)
            if pts is not None:
                point_cloud.append(pts)

        self.scan_direction *= -1

        if point_cloud:
            return np.vstack(point_cloud)
        else:
            return np.empty((0, 3))

    def get_voxels(self, voxel_size=0.15, min_points=5):
        """
        Returns voxelized point clusters (blobs) from the LiDAR sweep.
        """
        all_pts = self.get_raw_points()
        if all_pts.size == 0:
            return []

        blobs = blob_points_voxelized(
            all_pts.tolist(),
            voxel_size=voxel_size,
            min_points=min_points
        )
        return blobs

    def get_bounding_boxes(self, voxel_size=0.15, min_points=5):
        """
        Returns bounding boxes (cx, cy, cz, w, h, d) for each voxelized blob.
        """
        blobs = self.get_voxels(voxel_size, min_points)
        boxes = []
        for blob in blobs:
            boxes.append(blob)
        return boxes
    
    def get_single_rotation_points(self):
        """
        Reads one LiDAR rotation and returns XYZ points.
        """
        try:
            rotation = next(self.rotation_gen)
        except StopIteration:
            return np.empty((0, 3))

        tilt = self.current_tilt if hasattr(self, "current_tilt") else self.servo_min

        pts = self._lidar_scan_to_pointcloud(rotation, tilt)
        if pts is None:
            return np.empty((0, 3))

        return pts
    
    def start_servo_sweep(self):
        """
        Moves servo one step each call. Call repeatedly in your loop.
        """
        if not hasattr(self, "tilt_index"):
            self.tilt_index = 0

        angles = self.tilt_angles if self.scan_direction == 1 else self.tilt_angles[::-1]

        self.current_tilt = angles[self.tilt_index]
        self.servo.write(f"{int(self.current_tilt)}\n".encode())
        self.servo.flush()

        time.sleep(self.servo_delay)

        self.tilt_index += 1
        if self.tilt_index >= len(angles):
            self.tilt_index = 0
            self.scan_direction *= -1

    def set_servo_angle(self, angle):
        self.current_tilt = angle
        self.servo.write(f"{int(angle)}\n".encode())



