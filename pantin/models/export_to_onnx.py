from ultralytics import YOLO

# Load YOLOv8-Pose model
model = YOLO('yolov8n-pose.pt')

# Export to ONNX with dynamic batch size
model.export(
    format='onnx',
    imgsz=640,  # Input image size (640x640)
    dynamic=False,  # Fixed batch size for TensorRT optimization
    simplify=True,  # Simplify ONNX graph
    opset=17  # ONNX opset version
)

print("✅ ONNX export complete: yolov8n-pose.onnx")
