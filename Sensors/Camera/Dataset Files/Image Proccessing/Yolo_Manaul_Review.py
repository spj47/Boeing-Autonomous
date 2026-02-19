import os
import cv2
import numpy as np

images_dir = "Sensors/Camera/Dataset Files/Image Proccessing/RawImages"
labels_dir = "Sensors/Camera/Dataset Files/Image Proccessing/Labels"
progress_file = "Sensors/Camera/Dataset Files/Image Proccessing/review_progress.txt"


class YOLOReviewer:
    HANDLE_SIZE = 8

    def __init__(self, images_dir, labels_dir):
        self.images_dir = images_dir
        self.labels_dir = labels_dir
        self.image_files = sorted(
            [f for f in os.listdir(images_dir) if f.lower().endswith((".jpg", ".png"))]
        )
        self.index = 0
        self.boxes = []
        self.selected_box = None
        self.drag_mode = None
        self.start_point = None
        self.load_progress()

    def load_progress(self):
        if os.path.exists(progress_file):
            with open(progress_file, "r") as f:
                self.reviewed = set(line.strip() for line in f.readlines())
        else:
            self.reviewed = set()

    def mark_reviewed(self):
        img_name = self.image_files[self.index]
        self.reviewed.add(img_name)
        with open(progress_file, "w") as f:
            for name in self.reviewed:
                f.write(name + "\n")

    def load_image(self):
        img_name = self.image_files[self.index]
        path = os.path.join(self.images_dir, img_name)
        self.image = cv2.imread(path)
        self.h, self.w = self.image.shape[:2]
        self.boxes = self.load_labels(img_name)
        self.original_boxes = [b.copy() for b in self.boxes]
        self.selected_box = None

    def load_labels(self, img_name):
        label_path = os.path.join(
            self.labels_dir, os.path.splitext(img_name)[0] + ".txt"
        )
        boxes = []
        if not os.path.exists(label_path):
            return boxes

        with open(label_path, "r") as f:
            for line in f:
                cls, x, y, w, h = map(float, line.strip().split())
                x1 = int((x - w / 2) * self.w)
                y1 = int((y - h / 2) * self.h)
                x2 = int((x + w / 2) * self.w)
                y2 = int((y + h / 2) * self.h)
                boxes.append([x1, y1, x2, y2])
        return boxes

    def save_labels(self):
        img_name = self.image_files[self.index]
        label_path = os.path.join(
            self.labels_dir, os.path.splitext(img_name)[0] + ".txt"
        )
        with open(label_path, "w") as f:
            for (x1, y1, x2, y2) in self.boxes:
                x_center = ((x1 + x2) / 2) / self.w
                y_center = ((y1 + y2) / 2) / self.h
                width = (x2 - x1) / self.w
                height = (y2 - y1) / self.h
                f.write(f"0 {x_center:.6f} {y_center:.6f} {width:.6f} {height:.6f}\n")

        self.mark_reviewed()
        print("Saved & Marked Reviewed:", img_name)

    def draw_boxes(self, img):
        for i, (x1, y1, x2, y2) in enumerate(self.boxes):
            color = (0, 255, 0)
            if self.selected_box == i:
                color = (0, 0, 255)

            cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)

            if self.selected_box == i:
                for px, py in [(x1,y1),(x2,y1),(x1,y2),(x2,y2)]:
                    cv2.rectangle(img,
                                  (px - self.HANDLE_SIZE, py - self.HANDLE_SIZE),
                                  (px + self.HANDLE_SIZE, py + self.HANDLE_SIZE),
                                  (255, 0, 0), -1)
        return img

    def mouse_callback(self, event, x, y, flags, param):

        if event == cv2.EVENT_LBUTTONDOWN:
            self.start_point = (x, y)
            self.drag_mode = None
            self.selected_box = None
            self.resize_corner = None

            for i, (x1, y1, x2, y2) in enumerate(self.boxes):

                # ---- Check corners first ----
                corners = {
                    "tl": (x1, y1),
                    "tr": (x2, y1),
                    "bl": (x1, y2),
                    "br": (x2, y2),
                }

                for name, (cx, cy) in corners.items():
                    if abs(x - cx) < self.HANDLE_SIZE and abs(y - cy) < self.HANDLE_SIZE:
                        self.selected_box = i
                        self.drag_mode = "resize"
                        self.resize_corner = name
                        return

                if x1 < x < x2 and y1 < y < y2:
                    self.selected_box = i
                    self.drag_mode = "move"
                    return

            self.drag_mode = "new"

        elif event == cv2.EVENT_MOUSEMOVE and self.drag_mode:

            if self.selected_box is None:
                return

            x1, y1, x2, y2 = self.boxes[self.selected_box]

            if self.drag_mode == "move":
                dx = x - self.start_point[0]
                dy = y - self.start_point[1]

                x1 += dx
                y1 += dy
                x2 += dx
                y2 += dy

                self.start_point = (x, y)

            elif self.drag_mode == "resize":
                if self.resize_corner == "tl":
                    x1, y1 = x, y
                elif self.resize_corner == "tr":
                    x2, y1 = x, y
                elif self.resize_corner == "bl":
                    x1, y2 = x, y
                elif self.resize_corner == "br":
                    x2, y2 = x, y

            x1 = max(0, min(self.w - 1, x1))
            x2 = max(0, min(self.w - 1, x2))
            y1 = max(0, min(self.h - 1, y1))
            y2 = max(0, min(self.h - 1, y2))

            x1, x2 = sorted([x1, x2])
            y1, y2 = sorted([y1, y2])

            self.boxes[self.selected_box] = [x1, y1, x2, y2]

        elif event == cv2.EVENT_LBUTTONUP:

            if self.drag_mode == "new":
                x1, y1 = self.start_point
                x2, y2 = x, y

                x1, x2 = sorted([x1, x2])
                y1, y2 = sorted([y1, y2])

                if abs(x2 - x1) > 5 and abs(y2 - y1) > 5:
                    self.boxes.append([x1, y1, x2, y2])

            self.drag_mode = None
            self.resize_corner = None
        
        elif event == cv2.EVENT_RBUTTONDOWN:
            self.auto_square_cone_adaptive_color(x, y)


    def auto_square_cone_adaptive_color(self, cx, cy):
        region_size = 120
        x1 = max(0, cx - region_size)
        y1 = max(0, cy - region_size)
        x2 = min(self.w, cx + region_size)
        y2 = min(self.h, cy + region_size)

        roi = self.image[y1:y2, x1:x2]
        if roi.size == 0:
            return

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        sample_size = 15
        sx1 = max(0, cx - x1 - sample_size // 2)
        sy1 = max(0, cy - y1 - sample_size // 2)
        sx2 = min(roi.shape[1], sx1 + sample_size)
        sy2 = min(roi.shape[0], sy1 + sample_size)

        patch = hsv[sy1:sy2, sx1:sx2]
        if patch.size == 0:
            return

        h_mean, s_mean, v_mean = np.mean(patch, axis=(0,1))
        h_std, s_std, v_std = np.std(patch, axis=(0,1))

        lower = np.array([max(0, h_mean - 2*h_std),
                        max(30, s_mean - 2*s_std),
                        max(30, v_mean - 2*v_std)], dtype=np.uint8)
        upper = np.array([min(179, h_mean + 2*h_std),
                        min(255, s_mean + 2*s_std),
                        min(255, v_mean + 2*v_std)], dtype=np.uint8)

        mask = cv2.inRange(hsv, lower, upper)

        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return

        min_dist = float('inf')
        best_box = None
        for cnt in contours:
            bx, by, bw, bh = cv2.boundingRect(cnt)
            center_x = bx + bw // 2
            center_y = by + bh // 2
            dist = (center_x - (cx - x1))**2 + (center_y - (cy - y1))**2
            if dist < min_dist:
                min_dist = dist
                best_box = (bx, by, bw, bh)

        if best_box is None:
            return

        bx, by, bw, bh = best_box
        abs_x1 = x1 + bx
        abs_y1 = y1 + by
        abs_x2 = abs_x1 + bw
        abs_y2 = abs_y1 + bh

        pad = max(bw, bh) // 100
        abs_x1 = max(0, abs_x1 - pad)
        abs_y1 = max(0, abs_y1 - pad)
        abs_x2 = min(self.w - 1, abs_x2 + pad)
        abs_y2 = min(self.h - 1, abs_y2 + pad)

        size = max(abs_x2 - abs_x1, abs_y2 - abs_y1)
        center_x = (abs_x1 + abs_x2) // 2
        center_y = (abs_y1 + abs_y2) // 2

        new_x1 = max(0, center_x - size // 2)
        new_y1 = max(0, center_y - size // 2)
        new_x2 = min(self.w - 1, center_x + size // 2)
        new_y2 = min(self.h - 1, center_y + size // 2)

        self.boxes.append([new_x1, new_y1, new_x2, new_y2])

    def run(self):
        cv2.namedWindow("Reviewer")
        cv2.setMouseCallback("Reviewer", self.mouse_callback)

        while True:
            self.load_image()

            while True:
                display = self.image.copy()
                display = self.draw_boxes(display)

                reviewed_count = len(self.reviewed)
                total = len(self.image_files)
                percent = (reviewed_count / total) * 100

                cv2.putText(display,
                            f"{self.index+1}/{total} | Reviewed: {reviewed_count} ({percent:.1f}%)",
                            (10, 25),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.7, (255,255,255), 2)

                cv2.imshow("Reviewer", display)
                key = cv2.waitKey(30) & 0xFF

                if key == ord("n"):
                    self.index = min(self.index + 1, total - 1)
                    break
                elif key == ord("p"):
                    self.index = max(self.index - 1, 0)
                    break
                elif key == ord("s"):
                    self.save_labels()
                elif key == ord("d") and self.selected_box is not None:
                    self.boxes.pop(self.selected_box)
                    self.selected_box = None
                elif key == ord("r"):
                    self.boxes = [b.copy() for b in self.original_boxes]
                elif key == ord("q"):
                    cv2.destroyAllWindows()
                    return


if __name__ == "__main__":
    reviewer = YOLOReviewer(images_dir, labels_dir)
    reviewer.run()
