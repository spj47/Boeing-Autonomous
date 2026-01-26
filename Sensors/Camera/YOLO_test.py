from ultralytics import YOLO
from camera_interface import Webcam
import cv2

# Initialize webcam
camera = Webcam(0)

try:
    # Open webcam
    camera.open()

    # Load YOLO model
    model = YOLO('Sensors/Camera/yolo26n.pt')

    print("Press 'q' to quit the live detection.")

    while True:
        # Capture frame in RGB (YOLO only likes RGB)
        frame = camera.take_picture(rgb=True)

        # Run YOLO inference on the frame
        results = model(frame)

        # Get annotated frame. Index is always 0 since we're only passing in the 1 frame
        annotated_frame = results[0].plot()

        # Convert annotated frame back to BGR from RGB for OpenCV display (CV only likes BGR)
        display_frame = cv2.cvtColor(annotated_frame, cv2.COLOR_RGB2BGR)

        # Show the new frame
        cv2.imshow("YOLO Detection", display_frame)

        # Press q to break out of the main loop
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    camera.close()
