from ultralytics import YOLO

model = YOLO("yolov8n.pt")

# Train on your dataset
model.train(
    data="Sensors/Camera/Dataset Files/config.yaml",
    epochs=80,  
    imgsz=256,      
    batch=16,        
    name="cone_detector"
)
