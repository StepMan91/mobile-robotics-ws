# Models Directory

This directory contains the AI model files for pose estimation.

## Required Files

- `yolov8n-pose.engine` - TensorRT engine file (required for runtime)

## Optional Files

- `yolov8n-pose.pt` - Original PyTorch weights (can be deleted after conversion)
- `yolov8n-pose.onnx` - Intermediate ONNX format (can be deleted after conversion)
- `export_to_onnx.py` - Python script for ONNX export
- `build_tensorrt_engine.py` - Python script for TensorRT engine generation

## How to Generate Models

Follow the instructions in [MODEL_SETUP.md](../MODEL_SETUP.md)

## Note

Model files are excluded from git due to their large size. You must generate them locally following the model setup guide.
