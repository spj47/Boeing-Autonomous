import cv2
import numpy as np
import matplotlib.pyplot as plt
from yolo import YOLODetector

# Camera Varibles
IMG_W, IMG_H = 640, 480 # TODO: Don't remember the dimentions change these when they're found
FX, FY = 800, 800
CX, CY = IMG_W / 2, IMG_H / 2

NEAR_Z = 0.5
FAR_Z = 10.0


# Geometry
def pixel_to_ray(u, v):
    x_cam = (u - CX) / FX    
    y_cam = (v - CY) / FY   
    z_cam = 1.0   

    ray_cam = np.array([x_cam, y_cam, z_cam])
    ray_cam /= np.linalg.norm(ray_cam)

    # This Converts them to Y forward, X right, and Z up to conform with LIDAR 
    ray_world = np.array([
        ray_cam[0],
        ray_cam[2],
        -ray_cam[1],
    ])

    return ray_world


def box_to_pyramid(box_xyxy):
    x1, y1, x2, y2 = box_xyxy
    corners = [(x1, y1), (x2, y1), (x2, y2), (x1, y2)]
    rays = [pixel_to_ray(u, v) for u, v in corners]

    near_pts = np.array([r * NEAR_Z for r in rays])
    far_pts  = np.array([r * FAR_Z  for r in rays])

    return near_pts, far_pts


def draw_pyramid(ax, near_pts, far_pts, color="blue"):
    edges = [(0,1),(1,2),(2,3),(3,0)]
    for i, j in edges:
        ax.plot(*zip(near_pts[i], near_pts[j]), color=color)
        ax.plot(*zip(far_pts[i],  far_pts[j]),  color=color)
        ax.plot(*zip(near_pts[i], far_pts[i]),  color=color)


# Main loop
def main():
    detector = YOLODetector()

    plt.ion()
    fig = plt.figure(figsize=(7, 7))
    ax = fig.add_subplot(111, projection="3d")

    while True:
        frame, annotated, results = detector.detect()
        result = results[0]

        # ---- OpenCV image ----
        cv2.imshow("YOLO Detections", annotated)

        # ---- 3D plot ----
        ax.cla()
        ax.scatter(0, 0, 0, c="red", s=40)
        ax.text(0, 0, 0, "Camera", color="red")

        if result.boxes is not None:
            boxes = result.boxes.xyxy.cpu().numpy()
            classes = result.boxes.cls.cpu().numpy()

            for box, cls in zip(boxes, classes):
                near_pts, far_pts = box_to_pyramid(box)
                draw_pyramid(ax, near_pts, far_pts)

                label_pos = far_pts.mean(axis=0)
                ax.text(
                    label_pos[0],
                    label_pos[1],
                    label_pos[2],
                    result.names[int(cls)],
                    fontsize=8
                )

        ax.set_xlim(-5, 5)
        ax.set_ylim(-5, 5)
        ax.set_zlim(0, 10)
        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_zlabel("Z")
        ax.set_title("3D Back-Projected Detection Volumes")

        plt.pause(0.001)

        # ---- Exit on 'q' ----
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    detector.close()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
