import os
import cv2
import numpy as np

def orange_mask(rgb_frame):
    hsv = cv2.cvtColor(rgb_frame, cv2.COLOR_RGB2HSV)
    hsv[:, :, 2] = cv2.equalizeHist(hsv[:, :, 2])
    lower_orange = np.array([5, 80, 50])
    upper_orange = np.array([25, 255, 255])
    return cv2.inRange(hsv, lower_orange, upper_orange)


def clean_mask(mask):
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    return mask


def detect_cones(mask):
    """
    Returns a list of bounding boxes (x, y, w, h) for each detected cone
    """
    boxes = []
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < 50:  # filter small noise
            continue
        x, y, w, h = cv2.boundingRect(cnt)
        if h < w:  # ignore wide blobs
            continue
        boxes.append((x, y, w, h))
    return boxes

# =====================
# Prepare YOLO dataset
# =====================
images_dir = "Sensors/Camera/Dataset Files/Image Proccessing/RawImages"
labels_dir = "Sensors/Camera/Dataset Files/Image Proccessing/Labels"
os.makedirs(labels_dir, exist_ok=True)

for img_file in os.listdir(images_dir):
    if not img_file.lower().endswith((".jpg", ".png")):
        continue

    img_path = os.path.join(images_dir, img_file)
    img = cv2.imread(img_path)
    rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

    mask = orange_mask(rgb)
    mask = clean_mask(mask)
    boxes = detect_cones(mask)

    h_img, w_img = img.shape[:2]

    # Write YOLO label file
    label_file = os.path.join(labels_dir, os.path.splitext(img_file)[0] + ".txt")
    with open(label_file, "w") as f:
        for (x, y, w, h) in boxes:
            # YOLO format: class x_center y_center width height (normalized)
            x_center = (x + w / 2) / w_img
            y_center = (y + h / 2) / h_img
            width = w / w_img
            height = h / h_img
            f.write(f"0 {x_center:.6f} {y_center:.6f} {width:.6f} {height:.6f}\n")

print("YOLO labels generated for all images.")
