from ultralytics import YOLO
from camera_interface import Webcam

class YOLODetector:
    def __init__(self, model_path="./Sensors/best.pt", rgb=True):
        self.model = YOLO(model_path)
        self.cam = Webcam()
        self.rgb = rgb
        self.is_open = False
        self.open()

    def open(self):
        if not self.is_open:
            self.cam.open()
            self.is_open = True

    def close(self):
        if self.is_open:
            self.cam.close()
            self.is_open = False

    def get_frame(self):
        if not self.is_open:
            raise RuntimeError("Camera is not open.")
        return self.cam.take_picture(rgb=self.rgb)

    def detect(self):
        frame = self.get_frame()
        results = self.model.predict(frame, verbose=False)
        annotated = results[0].plot()
        return frame, annotated, results


