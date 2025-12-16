# RealSense 3D Skeletal Tracking

A high-performance C++ application for real-time 3D human pose estimation using Intel RealSense D457 camera and GPU-accelerated AI inference on NVIDIA RTX 4070.

![Platform](https://img.shields.io/badge/platform-Windows%2010%2F11-blue)
![GPU](https://img.shields.io/badge/GPU-NVIDIA%20RTX%204070-green)
![C++](https://img.shields.io/badge/C%2B%2B-17-orange)
![CUDA](https://img.shields.io/badge/CUDA-12.3-green)

## Features

✅ **GPU-Accelerated**: TensorRT-optimized YOLOv8-Pose runs on RTX 4070 for maximum FPS  
✅ **Real-Time 3D Tracking**: Combines RGB pose estimation with depth sensing for 3D coordinates  
✅ **17 Keypoints**: Full COCO skeleton (head, shoulders, elbows, wrists, hips, knees, ankles)  
✅ **Multi-Person Support**: Detect and track multiple people simultaneously  
✅ **Live Visualization**: Real-time skeleton overlay with FPS counter  
✅ **Console Output**: 3D coordinates printed in meters for integration  
✅ **Robust Error Handling**: Graceful handling of camera disconnection and occlusion

## System Requirements

### Hardware
- **Camera**: Intel RealSense D457/D455 (USB 3.0 connection)
- **GPU**: NVIDIA RTX 4070 (or equivalent with CUDA compute capability 8.9)
- **RAM**: 8 GB minimum (16 GB recommended)
- **Storage**: ~40 GB for dependencies

### Software
- **OS**: Windows 10/11 (64-bit)
- **Visual Studio**: 2022 with C++ Desktop Development
- **CUDA**: 12.3+ 
- **TensorRT**: 8.6.x
- **OpenCV**: 4.8+ with CUDA support
- **RealSense SDK**: 2.54+

## Quick Start

### 1. Prerequisites Installation

Follow the detailed installation guide: **[PREREQUISITES.md](PREREQUISITES.md)**

This includes installing:
- Visual Studio 2022
- CUDA Toolkit 12.3
- cuDNN 8.9
- TensorRT 8.6
- OpenCV with CUDA support (build from source)
- Intel RealSense SDK 2.0

⚠️ **Important**: Installation order matters. Follow the guide sequentially.

### 2. Model Preparation

Follow the model setup guide: **[MODEL_SETUP.md](MODEL_SETUP.md)**

Convert YOLOv8-Pose to TensorRT engine:
```powershell
cd models
python export_to_onnx.py
python build_tensorrt_engine.py
```

### 3. Build the Project

See detailed build instructions: **[BUILD_GUIDE.md](BUILD_GUIDE.md)**

Quick build:
```powershell
mkdir build
cd build
cmake .. -G "Visual Studio 17 2022" -A x64
cmake --build . --config Release
```

### 4. Run the Application

```powershell
cd build\bin\Release
.\RealsenseBodyPose.exe --model ..\..\..\models\yolov8n-pose.engine
```

## Usage

### Command-Line Options

```
RealsenseBodyPose.exe [OPTIONS]

Options:
  --model <path>      Path to TensorRT engine file (required)
  --width <int>       Camera width (default: 1280)
  --height <int>      Camera height (default: 720)
  --fps <int>         Camera FPS (default: 30)
  --confidence <f>    Detection confidence threshold (default: 0.5)
  --help              Show help message
```

### Example Usage

```powershell
# Standard usage with default settings
.\RealsenseBodyPose.exe --model models\yolov8n-pose.engine

# High-resolution mode
.\RealsenseBodyPose.exe --model models\yolov8n-pose.engine --width 1920 --height 1080

# Lower confidence threshold for more detections
.\RealsenseBodyPose.exe --model models\yolov8n-pose.engine --confidence 0.3
```

### Output

**Console Output** (every frame):
```
========== 3D Skeleton Coordinates ==========

Person 1 (Confidence: 0.87):
  Nose           : X= 0.042m  Y=-0.123m  Z= 1.832m  (conf: 0.91)
  Left Shoulder  : X=-0.156m  Y= 0.089m  Z= 1.845m  (conf: 0.94)
  Right Shoulder : X= 0.234m  Y= 0.091m  Z= 1.841m  (conf: 0.93)
  Left Wrist     : X=-0.387m  Y= 0.421m  Z= 1.692m  (conf: 0.88)
  Right Wrist    : X= 0.445m  Y= 0.418m  Z= 1.698m  (conf: 0.87)
  ...
```

**Visualization Window**:
- RGB video stream
- Colored skeleton overlay
- Bounding boxes
- FPS counter
- Press ESC to quit

## Project Structure

```
RealsenseBodyPose/
├── CMakeLists.txt              # Build configuration
├── README.md                   # This file
├── PREREQUISITES.md            # Installation guide
├── MODEL_SETUP.md              # Model conversion guide
├── BUILD_GUIDE.md              # Build instructions
├── models/                     # AI model files (gitignored)
│   ├── yolov8n-pose.pt
│   ├── yolov8n-pose.onnx
│   └── yolov8n-pose.engine
└── src/                        # Source code
    ├── main.cpp                # Main application
    ├── Utils.h                 # Common utilities
    ├── RealSenseCamera.h/cpp   # Camera wrapper
    ├── PoseEstimator.h/cpp     # TensorRT inference
    ├── SkeletonProjector.h/cpp # 3D projection
    └── Visualizer.h/cpp        # Rendering
```

## Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    Main Application Loop                     │
└───────────┬─────────────────────────────────────────────────┘
            │
            ├─► [1] RealSenseCamera.captureFrames()
            │       ├─ RGB Image (1280x720)
            │       └─ Aligned Depth Image
            │
            ├─► [2] PoseEstimator.estimate() [GPU: RTX 4070]
            │       ├─ Input: RGB Image
            │       ├─ TensorRT YOLOv8-Pose Inference
            │       └─ Output: 2D Keypoints + Confidence
            │
            ├─► [3] SkeletonProjector.project()
            │       ├─ Sample depth at each 2D keypoint
            │       ├─ rs2_deproject_pixel_to_point()
            │       └─ Output: 3D Keypoints (X, Y, Z meters)
            │
            └─► [4] Visualizer.draw() + print3DCoordinates()
                    ├─ Render skeleton overlay
                    ├─ Display FPS
                    └─ Print 3D coords to console
```

## Performance

### Expected Metrics (RTX 4070 + YOLOv8n-Pose)

- **Inference Time**: 8-12 ms per frame
- **End-to-End FPS**: 25-40 FPS (including camera + visualization)
- **Latency**: <50 ms
- **GPU Memory**: ~2 GB
- **3D Accuracy**: ±2-5 cm at 1-3 meters distance

### Optimization Tips

1. **Higher FPS**: Use YOLOv8n-pose (nano model) with 640x640 input
2. **Higher Accuracy**: Use YOLOv8m-pose or YOLOv8l-pose (slower)
3. **Reduce Latency**: Lower camera resolution to 848x480
4. **Batch Processing**: Modify code to process multiple frames per inference

## Troubleshooting

### Camera Issues

**Problem**: "RealSense error: No device found"
- Ensure D457 is connected to USB 3.0 port (blue port)
- Update camera firmware using Intel RealSense Viewer
- Reinstall RealSense SDK drivers

**Problem**: Poor depth quality
- Clean camera lenses
- Avoid IR-interfering environments (sunlight, reflective surfaces)
- Adjust depth settings in `RealSenseCamera::Config`

### GPU/TensorRT Issues

**Problem**: "Failed to load TensorRT engine"
- Verify engine file exists and path is correct
- Rebuild engine on the target GPU (engines are GPU-specific)
- Check TensorRT version compatibility

**Problem**: Slow inference (no GPU acceleration)
- Verify GPU is detected: `nvidia-smi`
- Check CUDA installation: `nvcc --version`
- Ensure TensorRT DLLs are in PATH or copied to executable directory

**Problem**: "CUDA out of memory"
- Close other GPU applications
- Use smaller model (YOLOv8n instead of YOLOv8l)
- Reduce camera resolution

### Build Errors

**Problem**: CMake can't find packages
- Verify environment variables: `CUDA_PATH`, `TENSORRT_DIR`, `OpenCV_DIR`
- Restart terminal/computer after setting environment variables
- Check package installation paths

**Problem**: Linking errors
- Ensure all DLLs are accessible (add to PATH or copy to output directory)
- Verify 64-bit versions of all libraries (x64)
- Check Visual Studio 2022 is selected in CMake

## Integration

### Using 3D Coordinates in Your Application

The application outputs 3D coordinates to console. To integrate:

**Option 1: Redirect console output**
```powershell
.\RealsenseBodyPose.exe --model ... > output.txt
```

**Option 2: Modify source code**
Edit `Visualizer::print3DCoordinates()` to:
- Write to file
- Send over network (TCP/UDP)
- Publish to ROS topics
- Save to database

**Option 3: Use as library**
Include the classes in your own project:
```cpp
#include "RealSenseCamera.h"
#include "PoseEstimator.h"
#include "SkeletonProjector.h"

// Your application code
```

## Known Limitations

- **Single GPU Only**: Currently hardcoded to GPU 0
- **No Recording**: No built-in recording of 3D skeleton data
- **Windows Only**: Not tested on Linux (should work with minor CMake changes)
- **USB Bandwidth**: High FPS may require dedicated USB controller

## Future Enhancements

- [ ] Multi-GPU support
- [ ] Recording to file (CSV, JSON, or rosbag)
- [ ] Linux support
- [ ] ROS/ROS2 integration
- [ ] Web interface for remote monitoring
- [ ] Temporal filtering for smoother tracking

## License

This project is provided as-is for educational and research purposes.

**Dependencies Licenses**:
- Intel RealSense SDK: Apache 2.0
- TensorRT: NVIDIA EULA
- YOLOv8: AGPL-3.0
- OpenCV: Apache 2.0

## Credits

- **YOLOv8-Pose**: [Ultralytics](https://github.com/ultralytics/ultralytics)
- **Intel RealSense**: [librealsense](https://github.com/IntelRealSense/librealsense)
- **TensorRT**: NVIDIA Corporation
- **OpenCV**: OpenCV Team

## Support

For issues and questions:
1. Check [BUILD_GUIDE.md](BUILD_GUIDE.md) troubleshooting section
2. Review [PREREQUISITES.md](PREREQUISITES.md) for installation issues
3. Verify hardware compatibility

---

**Developed for high-performance real-time robotics and computer vision applications on Windows with NVIDIA RTX GPUs.**
