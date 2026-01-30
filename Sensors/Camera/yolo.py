import cv2
from ultralytics import YOLO
from camera_interface import Webcam 

def main():
    model = YOLO("./Sensors/Camera/yolov8n.pt")  

    cam = Webcam()
    cam.open()

    print("Press 'q' to quit the live detection feed.")

    try:
        while True:
            frame = cam.take_picture(rgb=True)
            results = model.predict(frame, verbose=False)

            annotated_frame = results[0].plot() 

            annotated_frame_bgr = cv2.cvtColor(annotated_frame, cv2.COLOR_RGB2BGR)
            cv2.imshow("YOLO Object Detection", annotated_frame_bgr)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        cam.close()

if __name__ == "__main__":
    main()
