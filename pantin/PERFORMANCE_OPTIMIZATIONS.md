# Quick Performance Optimizations Applied

## Changes Made:
1. Camera Resolution: 1280x720 → 640x480 (2x faster)
2. Confidence Threshold: 0.5 → 0.3 (more detections)
3. Frame Skipping: Process every 2nd frame (2x faster inference)

## Expected Performance:
- Previous: ~7 FPS
- Target: 25-40 FPS
- Total speedup: ~3-5x

## To Test:
```powershell
cd C:\Users\basti\source\repos\RealsenseBodyPose\build\bin\Release
.\RealsenseBodyPose.exe --model ..\..\..\models\yolov8n-pose.onnx
```

## Command-line Override:
You can still use full resolution:
```powershell
.\RealsenseBodyPose.exe --model ..\..\..\models\yolov8n-pose.onnx --width 1280 --height 720
```

## Next Steps for Even More Speed:
1. Build OpenCV with CUDA (60-90 FPS)
2. Fix TensorRT engine (80-120 FPS)
3. Use smaller YOLOv8 nano model variant
