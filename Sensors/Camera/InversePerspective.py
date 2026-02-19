import numpy as np
from yolo import YOLODetector

class DetectionRays3D:
    def __init__(self, img_width=640, img_height=480, fx=800, fy=800, near_z=0.5, far_z=10.0):
        self.IMG_W = img_width
        self.IMG_H = img_height
        self.FX = fx
        self.FY = fy
        self.CX = img_width / 2
        self.CY = img_height / 2
        self.NEAR_Z = near_z
        self.FAR_Z = far_z
        self.detector = YOLODetector()
    
    def pixel_to_ray(self, u, v):
        """
        Convert a pixel (u, v) to a 3D ray in camera/world coordinates.
        """
        x_cam = (u - self.CX) / self.FX
        y_cam = (v - self.CY) / self.FY
        z_cam = 1.0

        ray_cam = np.array([x_cam, y_cam, z_cam])
        ray_cam /= np.linalg.norm(ray_cam)

        # Convert to Y-forward, X-right, Z-up (like LIDAR)
        ray_world = np.array([
            ray_cam[0],
            ray_cam[2],
            -ray_cam[1],
        ])
        return ray_world

    def box_to_rays(self, box_xyxy):
        """
        Convert a bounding box [x1, y1, x2, y2] to 3D near/far rays.
        """
        x1, y1, x2, y2 = box_xyxy
        corners = [(x1, y1), (x2, y1), (x2, y2), (x1, y2)]
        rays = [self.pixel_to_ray(u, v) for u, v in corners]

        near_pts = np.array([r * self.NEAR_Z for r in rays])
        far_pts  = np.array([r * self.FAR_Z  for r in rays])

        return near_pts, far_pts

    def get_detection_rays(self):
        """
        Runs YOLO detection once and returns a list of rays for each detected box.
        Each element is a tuple: (near_pts, far_pts, class_id)
        """
        frame, annotated, results = self.detector.detect()
        rays_list = []

        if results and results[0].boxes is not None:
            boxes = results[0].boxes.xyxy.cpu().numpy()
            classes = results[0].boxes.cls.cpu().numpy()

            for box, cls in zip(boxes, classes):
                near_pts, far_pts = self.box_to_rays(box)
                rays_list.append((near_pts, far_pts, int(cls)))

        return rays_list

    def close(self):
        self.detector.close()
