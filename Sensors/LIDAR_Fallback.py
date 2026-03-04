import math
from ld14_driver import LD14Driver

class LidarObjectDetector:
    def __init__(self, port):
        self.driver = LD14Driver(port)
    
    @staticmethod
    def _polar_to_cartesian(angle_deg, distance):
        angle_rad = math.radians(angle_deg)
        x = distance * math.cos(angle_rad)
        y = -distance * math.sin(angle_rad)
        return x, y

    def detect_objects(self, k=0.02):
        for timestamp, rotation_points in self.driver.read_rotation():
            cartesian_points = [
                self._polar_to_cartesian(a, d) + (i,)
                for a, d, i in rotation_points
            ]
            
            cartesian_points = [p for p in cartesian_points if p[1] >= 0]

            objects = []
            current_cluster = []
            
            for i, point in enumerate(cartesian_points):
                if i == 0:
                    current_cluster.append(point)
                    continue
                
                prev_point = cartesian_points[i - 1]
                distance_threshold = k * math.hypot(point[0], point[1])
                dist = math.hypot(point[0] - prev_point[0], point[1] - prev_point[1])
                
                if dist <= distance_threshold:
                    current_cluster.append(point)
                else:
                    if current_cluster:
                        objects.append(current_cluster)
                    current_cluster = [point]
            
            if current_cluster:
                objects.append(current_cluster)

            bounding_boxes = []
            for cluster in objects:
                xs, ys, ints = zip(*cluster)
                bbox = {
                    'min_x': min(xs),
                    'max_x': max(xs),
                    'min_y': min(ys),
                    'max_y': max(ys)
                }
                bounding_boxes.append(bbox)
            
            yield timestamp, bounding_boxes
