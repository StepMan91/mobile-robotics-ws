# YOLOv8-Pose Model Setup Guide

This guide explains how to download, convert, and optimize the YOLOv8-Pose model for GPU-accelerated inference on your NVIDIA RTX 4070.

> [!IMPORTANT]
> **Prerequisites**: Complete [PREREQUISITES.md](file:///c:/Users/basti/source/repos/RealsenseBodyPose/PREREQUISITES.md) before proceeding. You need Python, CUDA, TensorRT, and all dependencies installed.

---

## Overview

The conversion pipeline is:
```
YOLOv8-Pose (.pt) → ONNX (.onnx) → TensorRT Engine (.engine)
```

- **PyTorch (.pt)**: Original model weights from Ultralytics
- **ONNX (.onnx)**: Intermediate format for cross-framework compatibility
- **TensorRT Engine (.engine)**: GPU-optimized runtime format for RTX 4070

---

## Step 1: Create Models Directory

```powershell
# Navigate to project root
cd C:\Users\basti\source\repos\RealsenseBodyPose

# Create models directory
mkdir models
cd models
```

---

## Step 2: Download YOLOv8-Pose Weights

The YOLOv8-Pose model comes in different sizes. We recommend **YOLOv8n-Pose** (nano) for real-time performance:

| Model | Size | Speed | Accuracy |
|-------|------|-------|----------|
| YOLOv8n-pose | 6.4 MB | ~100 FPS | Good |
| YOLOv8s-pose | 26 MB | ~70 FPS | Better |
| YOLOv8m-pose | 51 MB | ~45 FPS | Great |
| YOLOv8l-pose | 100 MB | ~30 FPS | Excellent |

### Download Options

**Option A: Automatic Download (Recommended)**
```powershell
# The Ultralytics library will auto-download on first use
python -c "from ultralytics import YOLO; model = YOLO('yolov8n-pose.pt')"
```

**Option B: Manual Download**
1. Visit: https://github.com/ultralytics/assets/releases
2. Download: `yolov8n-pose.pt` (~6 MB)
3. Place in `models/yolov8n-pose.pt`

---

## Step 3: Export to ONNX Format

ONNX is an intermediate format that TensorRT can consume.

### Create Export Script

Create `models/export_to_onnx.py`:

```python
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
```

### Run Export

```powershell
cd C:\Users\basti\source\repos\RealsenseBodyPose\models
python export_to_onnx.py
```

**Expected Output**:
- File created: `yolov8n-pose.onnx` (~12 MB)
- Console message: "Export complete"

---

## Step 4: Convert ONNX to TensorRT Engine

TensorRT engines are GPU-specific and highly optimized. We'll build an engine for your RTX 4070.

> [!WARNING]
> **GPU-Specific**: TensorRT engines are compiled for a specific GPU architecture. This engine will ONLY work on RTX 4070 (or other Ada Lovelace GPUs).

### Create Conversion Script

Create `models/build_tensorrt_engine.py`:

```python
import tensorrt as trt
import sys

def build_engine(onnx_file_path, engine_file_path, fp16_mode=True, max_batch_size=1):
    """
    Build TensorRT engine from ONNX file.
    
    Args:
        onnx_file_path: Path to ONNX model
        engine_file_path: Output path for TensorRT engine
        fp16_mode: Enable FP16 precision (faster, slightly less accurate)
        max_batch_size: Maximum batch size
    """
    # Create TensorRT logger
    TRT_LOGGER = trt.Logger(trt.Logger.WARNING)
    
    # Create builder
    builder = trt.Builder(TRT_LOGGER)
    network = builder.create_network(1 << int(trt.NetworkDefinitionCreationFlag.EXPLICIT_BATCH))
    parser = trt.OnnxParser(network, TRT_LOGGER)
    
    # Parse ONNX
    print(f"📖 Parsing ONNX file: {onnx_file_path}")
    with open(onnx_file_path, 'rb') as model:
        if not parser.parse(model.read()):
            print('❌ ERROR: Failed to parse ONNX file.')
            for error in range(parser.num_errors):
                print(parser.get_error(error))
            return None
    
    print("✅ ONNX file parsed successfully")
    
    # Configure builder
    config = builder.create_builder_config()
    
    # Set memory pool limit (8 GB for RTX 4070)
    config.set_memory_pool_limit(trt.MemoryPoolType.WORKSPACE, 8 * (1 << 30))
    
    # Enable FP16 mode for faster inference
    if fp16_mode and builder.platform_has_fast_fp16:
        config.set_flag(trt.BuilderFlag.FP16)
        print("✅ FP16 mode enabled")
    else:
        print("⚠️  FP16 mode not available, using FP32")
    
    # Build engine
    print("🔨 Building TensorRT engine... (this may take 2-5 minutes)")
    serialized_engine = builder.build_serialized_network(network, config)
    
    if serialized_engine is None:
        print("❌ ERROR: Failed to build TensorRT engine")
        return None
    
    # Save engine to file
    print(f"💾 Saving engine to: {engine_file_path}")
    with open(engine_file_path, 'wb') as f:
        f.write(serialized_engine)
    
    print("✅ TensorRT engine built successfully!")
    return engine_file_path

if __name__ == "__main__":
    onnx_path = "yolov8n-pose.onnx"
    engine_path = "yolov8n-pose.engine"
    
    # Build engine with FP16 precision
    build_engine(onnx_path, engine_path, fp16_mode=True)
    
    # Print file sizes
    import os
    onnx_size = os.path.getsize(onnx_path) / (1024 * 1024)
    engine_size = os.path.getsize(engine_path) / (1024 * 1024)
    
    print(f"\n📊 Model Sizes:")
    print(f"   ONNX:   {onnx_size:.2f} MB")
    print(f"   Engine: {engine_size:.2f} MB")
```

### Run Conversion

```powershell
cd C:\Users\basti\source\repos\RealsenseBodyPose\models
python build_tensorrt_engine.py
```

**Expected Output**:
```
📖 Parsing ONNX file: yolov8n-pose.onnx
✅ ONNX file parsed successfully
✅ FP16 mode enabled
🔨 Building TensorRT engine... (this may take 2-5 minutes)
💾 Saving engine to: yolov8n-pose.engine
✅ TensorRT engine built successfully!

📊 Model Sizes:
   ONNX:   12.34 MB
   Engine: 8.76 MB
```

> [!NOTE]
> **Build Time**: Engine building takes 2-5 minutes. This is a one-time process. The engine is cached for all future runs.

---

## Step 5: Verify Model Files

Check that all files are present:

```powershell
cd C:\Users\basti\source\repos\RealsenseBodyPose\models
dir
```

**Expected Files**:
```
yolov8n-pose.pt         # Original PyTorch weights (~6 MB)
yolov8n-pose.onnx       # ONNX intermediate format (~12 MB)
yolov8n-pose.engine     # TensorRT optimized engine (~9 MB)
export_to_onnx.py       # Export script
build_tensorrt_engine.py # Build script
```

> [!TIP]
> **Keep Only Engine**: For production, you only need the `.engine` file. You can delete `.pt` and `.onnx` to save space.

---

## Understanding YOLOv8-Pose Output

The model detects 17 keypoints per person (COCO format):

| Index | Joint Name | Index | Joint Name |
|-------|-----------|-------|-----------|
| 0 | Nose | 9 | Left Wrist |
| 1 | Left Eye | 10 | Right Wrist |
| 2 | Right Eye | 11 | Left Hip |
| 3 | Left Ear | 12 | Right Hip |
| 4 | Right Ear | 13 | Left Knee |
| 5 | Left Shoulder | 14 | Right Knee |
| 6 | Right Shoulder | 15 | Left Ankle |
| 7 | Left Elbow | 16 | Right Ankle |
| 8 | Right Elbow | | |

**Output Format** (per person):
- Bounding box: `[x, y, width, height]`
- Confidence: `0.0 - 1.0`
- 17 keypoints: `[x1, y1, conf1, x2, y2, conf2, ..., x17, y17, conf17]`

---

## Testing the Model (Optional)

Create a quick test script to verify the model works:

```python
import tensorrt as trt
import numpy as np

# Load engine
TRT_LOGGER = trt.Logger(trt.Logger.WARNING)
with open('yolov8n-pose.engine', 'rb') as f:
    engine = trt.Runtime(TRT_LOGGER).deserialize_cuda_engine(f.read())

if engine:
    print("✅ TensorRT engine loaded successfully")
    print(f"   Inputs: {engine.num_bindings // 2}")
    print(f"   Device: GPU")
else:
    print("❌ Failed to load engine")
```

---

## Alternative Models

If you need higher accuracy or have different performance requirements:

### Higher Accuracy (Slower)
```powershell
# YOLOv8s-pose (better accuracy, ~70 FPS)
python -c "from ultralytics import YOLO; YOLO('yolov8s-pose.pt').export(format='onnx')"
python build_tensorrt_engine.py  # Update paths in script

# YOLOv8m-pose (even better, ~45 FPS)
python -c "from ultralytics import YOLO; YOLO('yolov8m-pose.pt').export(format='onnx')"
```

### Different Input Size
```python
# Larger input = better small person detection but slower
model.export(format='onnx', imgsz=1280)  # Instead of 640
```

---

## Troubleshooting

### Issue: "No module named 'tensorrt'"
**Solution**: Install TensorRT Python wheel from `C:\TensorRT-8.6.1.6\python`

### Issue: "CUDA out of memory" during engine building
**Solution**: 
1. Close other GPU applications
2. Reduce workspace size in `build_tensorrt_engine.py` (try 4 GB instead of 8 GB)

### Issue: Engine building fails with "Unsupported ONNX opset"
**Solution**: Try different opset versions in export (opset=16 or opset=13)

### Issue: "Platform does not support fast FP16"
**Solution**: This shouldn't happen on RTX 4070. Verify CUDA/TensorRT installation.

### Issue: Very slow inference despite TensorRT
**Solution**:
1. Verify engine was built with FP16 enabled
2. Check Task Manager → GPU → CUDA usage (should be high during inference)
3. Rebuild engine with correct GPU specification

---

## Performance Expectations

With RTX 4070 + YOLOv8n-Pose + TensorRT FP16:

- **Input**: 640x640 RGB image
- **Inference Time**: ~8-12 ms per frame
- **FPS**: 80-120 FPS (inference only)
- **End-to-End**: 25-40 FPS (including camera capture + visualization)

> [!TIP]
> **Batch Processing**: For even higher throughput, process multiple frames in a batch. Modify the engine builder to support `max_batch_size > 1`.

---

**Next Step**: Proceed to [BUILD_GUIDE.md](file:///c:/Users/basti/source/repos/RealsenseBodyPose/BUILD_GUIDE.md) to compile the C++ application.
