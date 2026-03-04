import math
from LIDAR_Fallback import LidarObjectDetector
from yolo import YOLODetector


class Sensors:
    def __init__(self, LIDARPort):
        self.lidar = LidarObjectDetector(LIDARPort)
        self.yolo = YOLODetector()
        self.cameraOffsetFromLIDAR = (0.5, 0.5)
        self.coneSize = (0.15, 0.15)
        self.imageSize = 256
        self.cameraHeight = 0.8
        self.cameraPitchDeg = 0
        self.horizontalFOVDeg = 70
        self.verticalFOVDeg = 55

    def bbox_center(self, bbox):
        return (
            (bbox["min_x"] + bbox["max_x"]) / 2,
            (bbox["min_y"] + bbox["max_y"]) / 2
        )

    def camera_to_lidar(self, x_cam, y_cam):
        dx, dy = self.cameraOffsetFromLIDAR
        return x_cam + dx, y_cam + dy

    def resize_to_cone(self, center_x, center_y):
        half_w = self.coneSize[0] / 2
        half_h = self.coneSize[1] / 2
        return {
            "min_x": center_x - half_w,
            "max_x": center_x + half_w,
            "min_y": center_y - half_h,
            "max_y": center_y + half_h
        }

    def match(self, lidar_boxes, camera_points, threshold=0.3):
        matched = set()
        for cam_x, cam_y in camera_points:
            for i, box in enumerate(lidar_boxes):
                cx, cy = self.bbox_center(box)
                if math.hypot(cx - cam_x, cy - cam_y) < threshold:
                    matched.add(i)
        return matched

    def image_to_camera_coords(self, px, py):
        h_fov = math.radians(self.horizontalFOVDeg)
        v_fov = math.radians(self.verticalFOVDeg)
        pitch = math.radians(self.cameraPitchDeg)

        nx = (px / self.imageSize) * 2 - 1
        ny = 1 - (py / self.imageSize) * 2

        theta_x = nx * (h_fov / 2)
        theta_y = ny * (v_fov / 2)

        total_vertical_angle = pitch + theta_y

        if total_vertical_angle <= 0:
            return None, None

        forward_distance = self.cameraHeight / math.tan(total_vertical_angle)
        lateral_distance = forward_distance * math.tan(theta_x)

        return forward_distance, lateral_distance

    def GetObstacles(self, useCamera=False):
        lidar_gen = self.lidar.detect_objects()
        timestamp, lidar_boxes = next(lidar_gen)

        if not useCamera:
            return timestamp, [
                {**box, "is_cone": False}
                for box in lidar_boxes
            ]

        frame, annotated, results = self.yolo.detect()

        camera_points_lidar_frame = []

        for box in results[0].boxes:
            cls_id = int(box.cls[0])
            label = self.yolo.model.names[cls_id]

            if label.lower() != "cone":
                continue

            x1, y1, x2, y2 = box.xyxy[0]
            cx_img = float((x1 + x2) / 2)
            cy_img = float((y1 + y2) / 2)

            x_cam, y_cam = self.image_to_camera_coords(cx_img, cy_img)

            if x_cam is None:
                continue

            x_lidar, y_lidar = self.camera_to_lidar(x_cam, y_cam)
            camera_points_lidar_frame.append((x_lidar, y_lidar))

        matched_indices = self.match(
            lidar_boxes,
            camera_points_lidar_frame
        )

        final_obstacles = []

        for i, box in enumerate(lidar_boxes):
            cx, cy = self.bbox_center(box)

            if i in matched_indices:
                resized = self.resize_to_cone(cx, cy)
                final_obstacles.append({
                    **resized,
                    "is_cone": True
                })
            else:
                final_obstacles.append({
                    **box,
                    "is_cone": False
                })

        return timestamp, final_obstacles