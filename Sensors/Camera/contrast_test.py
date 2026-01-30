import cv2
import numpy as np
from camera_interface import Webcam


def orange_mask(rgb_frame):
    """
    Create a binary mask where orange colors are white.
    Handles shadows by equalizing brightness.
    """
    # Convert RGB -> HSV
    hsv = cv2.cvtColor(rgb_frame, cv2.COLOR_RGB2HSV)

    # Equalize brightness to handle shadows
    v = hsv[:, :, 2]
    hsv[:, :, 2] = cv2.equalizeHist(v)

    # HSV thresholds for vivid orange
    lower_orange = np.array([7, 120, 50])  # hue, saturation, value
    upper_orange = np.array([17, 255, 255])

    mask = cv2.inRange(hsv, lower_orange, upper_orange)
    return mask


def clean_mask(mask):
    """
    Remove noise and fill gaps.
    """
    kernel = np.ones((5, 5), np.uint8)

    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    return mask


def detect_cones(mask, rgb_frame):
    """
    Find orange blobs and draw bounding boxes.
    """
    contours, _ = cv2.findContours(
        mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )

    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < 1500:  # Ignore small noise
            continue

        x, y, w, h = cv2.boundingRect(cnt)

        # Taller than wide filter
        if h < w:
            continue

        # Top half vs bottom half filter (same as last, but both seems to help)
        mask_roi = mask[y:y+h, x:x+w]
        half_h = h // 2

        top_half = mask_roi[0:half_h, :]
        bottom_half = mask_roi[half_h:h, :]

        # Find largest contour width in each half
        top_contours, _ = cv2.findContours(top_half, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        bottom_contours, _ = cv2.findContours(bottom_half, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        top_width = max((cv2.boundingRect(c)[2] for c in top_contours), default=0)
        bottom_width = max((cv2.boundingRect(c)[2] for c in bottom_contours), default=0)

        # If top is wider than bottom, it's likely not a cone
        if top_width > bottom_width:
            continue

        # Draw rectangle and label
        cv2.rectangle(rgb_frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cv2.putText(
            rgb_frame,
            "Cone",
            (x, y - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 0),
            2,
        )

    return rgb_frame


def main():
    cam = Webcam()
    cam.open()

    while True:
        frame = cam.take_picture(rgb=True)

        mask = orange_mask(frame)
        mask = clean_mask(mask)

        output = detect_cones(mask, frame.copy())

        cv2.imshow("Orange Mask", mask)
        cv2.imshow("Detected Cones", output)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cam.close()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
