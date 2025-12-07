# Prerequisites Installation Guide

This guide provides step-by-step instructions for installing all dependencies required to build and run the RealSense 3D Skeletal Tracking application on Windows 10/11 with NVIDIA RTX 4070.

> [!WARNING]
> **Installation Order Matters**: Follow the sections in order. Some tools depend on others being installed first.

> [!IMPORTANT]
> **Administrator Privileges Required**: Many installers require administrator access. Right-click installers and select "Run as administrator" if prompted.

---

## 1. Visual Studio 2022 (C++ Compiler)

### Installation Steps

1. **Download**: [Visual Studio 2022 Community](https://visualstudio.microsoft.com/downloads/) (Free)

2. **Run Installer**: Launch `VisualStudioSetup.exe`

3. **Select Workloads**:
   - ✅ **Desktop development with C++**
   - Within this workload, ensure these individual components are checked:
     - MSVC v143 - VS 2022 C++ x64/x86 build tools (Latest)
     - Windows 10/11 SDK (Latest version)
     - C++ CMake tools for Windows
     - C++ ATL for latest build tools

4. **Install**: Click Install and wait (requires ~7-10 GB disk space)

5. **Verify Installation**:
```powershell
# Open Developer Command Prompt for VS 2022
cl
# Should display: Microsoft (R) C/C++ Optimizing Compiler Version 19.xx
```

---

## 2. CMake (Build System Generator)

### Installation Steps

1. **Download**: [CMake 3.27+ Windows x64 Installer](https://cmake.org/download/)
   - File: `cmake-3.27.x-windows-x86_64.msi`

2. **Install**: Run installer with default options
   - ✅ **Important**: Check "Add CMake to system PATH for all users"

3. **Verify Installation**:
```powershell
cmake --version
# Should show: cmake version 3.27.x or higher
```

---

## 3. NVIDIA CUDA Toolkit 12.x

> [!IMPORTANT]
> **RTX 4070 Compatibility**: RTX 4070 requires CUDA 11.8 or newer. We recommend CUDA 12.3 for best performance.

### Installation Steps

1. **Download**: [CUDA Toolkit 12.3.2](https://developer.nvidia.com/cuda-downloads)
   - Select: Windows → x86_64 → 11 → exe (local)
   - File: `cuda_12.3.2_546.12_windows.exe` (~3.5 GB)

2. **Install**:
   - Run installer as administrator
   - Choose **Custom (Advanced)** installation
   - Select all components (CUDA Toolkit, Visual Studio Integration, NSight tools)
   - Default installation path: `C:\Program Files\NVIDIA GPU Computing Toolkit\CUDA\v12.3`

3. **Verify Installation**:
```powershell
nvcc --version
# Should show: Cuda compilation tools, release 12.3, V12.3.xxx

nvidia-smi
# Should display RTX 4070 with Driver Version and CUDA Version
```

4. **Environment Variables** (Auto-configured by installer, but verify):
   - `CUDA_PATH` = `C:\Program Files\NVIDIA GPU Computing Toolkit\CUDA\v12.3`
   - `PATH` should include `%CUDA_PATH%\bin` and `%CUDA_PATH%\libnvvp`

---

## 4. cuDNN (CUDA Deep Neural Network Library)

### Installation Steps

1. **Download**: [cuDNN 8.9.7 for CUDA 12.x](https://developer.nvidia.com/cudnn)
   - ⚠️ Requires free NVIDIA Developer account
   - Select: cuDNN v8.9.7 for CUDA 12.x
   - File: `cudnn-windows-x86_64-8.9.7.29_cuda12-archive.zip`

2. **Extract and Copy Files**:
```powershell
# Extract ZIP to temporary location
# Navigate to extracted folder, then:

# Copy DLL files
xcopy bin\cudnn*.dll "C:\Program Files\NVIDIA GPU Computing Toolkit\CUDA\v12.3\bin\" /Y

# Copy header files
xcopy include\cudnn*.h "C:\Program Files\NVIDIA GPU Computing Toolkit\CUDA\v12.3\include\" /Y

# Copy library files
xcopy lib\x64\cudnn*.lib "C:\Program Files\NVIDIA GPU Computing Toolkit\CUDA\v12.3\lib\x64\" /Y
```

3. **Verify Installation**:
```powershell
# Check if cudnn64_8.dll exists
dir "C:\Program Files\NVIDIA GPU Computing Toolkit\CUDA\v12.3\bin\cudnn64_8.dll"
```

---

## 5. TensorRT 8.6.x

> [!NOTE]
> **TensorRT Purpose**: Optimizes neural network inference on NVIDIA GPUs. Critical for achieving high FPS with YOLOv8-Pose.

### Installation Steps

1. **Download**: [TensorRT 8.6.1.6 for CUDA 12.x](https://developer.nvidia.com/tensorrt)
   - ⚠️ Requires NVIDIA Developer account
   - Select: TensorRT 8.6.1.6 for Windows 10 and CUDA 12.0
   - File: `TensorRT-8.6.1.6.Windows10.x86_64.cuda-12.0.zip` (~1.2 GB)

2. **Extract to Permanent Location**:
```powershell
# Extract to: C:\TensorRT-8.6.1.6
# Directory structure should be:
#   C:\TensorRT-8.6.1.6\bin\
#   C:\TensorRT-8.6.1.6\include\
#   C:\TensorRT-8.6.1.6\lib\
```

3. **Add to System PATH**:
   - Open System Environment Variables
   - Edit `Path` variable
   - Add: `C:\TensorRT-8.6.1.6\bin`

4. **Set Environment Variable**:
```powershell
# Add system variable (use System Properties → Environment Variables):
# Variable: TENSORRT_DIR
# Value: C:\TensorRT-8.6.1.6
```

5. **Install Python Packages** (needed for model conversion):
```powershell
# Navigate to TensorRT directory
cd C:\TensorRT-8.6.1.6\python

# Install TensorRT Python wheel (use your Python version)
pip install tensorrt-8.6.1-cp311-none-win_amd64.whl
```

---

## 6. OpenCV 4.8+ with CUDA Support

> [!IMPORTANT]
> **Build from Source Required**: Pre-built OpenCV binaries do NOT include CUDA support. You must compile OpenCV yourself.

### Installation Steps

1. **Download OpenCV Source**:
```powershell
# Create build directory
mkdir C:\opencv_build
cd C:\opencv_build

# Download OpenCV 4.8.1 source
# Download from: https://github.com/opencv/opencv/archive/4.8.1.zip
# Extract to: C:\opencv_build\opencv-4.8.1

# Download OpenCV Contrib modules (optional but recommended)
# Download from: https://github.com/opencv/opencv_contrib/archive/4.8.1.zip
# Extract to: C:\opencv_build\opencv_contrib-4.8.1
```

2. **Configure with CMake**:
```powershell
cd C:\opencv_build
mkdir build
cd build

# Open CMake GUI
cmake-gui

# Set source: C:/opencv_build/opencv-4.8.1
# Set build: C:/opencv_build/build
# Click "Configure" → Select "Visual Studio 17 2022" + x64
```

3. **Enable CUDA Options in CMake GUI**:
   - ✅ `WITH_CUDA` = ON
   - ✅ `OPENCV_DNN_CUDA` = ON
   - ✅ `ENABLE_FAST_MATH` = ON
   - ✅ `CUDA_FAST_MATH` = ON
   - ✅ `BUILD_opencv_world` = ON (builds single library)
   - Set `CUDA_ARCH_BIN` = `8.9` (for RTX 4070 - Ada Lovelace architecture)
   - Set `OPENCV_EXTRA_MODULES_PATH` = `C:/opencv_build/opencv_contrib-4.8.1/modules`
   - Click "Configure" again, then "Generate"

4. **Build OpenCV** (This takes 30-60 minutes):
```powershell
# Open Developer Command Prompt for VS 2022
cd C:\opencv_build\build

# Build Release configuration
cmake --build . --config Release -j 8
```

5. **Install OpenCV**:
```powershell
# This copies binaries to C:\opencv_build\build\install
cmake --install . --config Release
```

6. **Set Environment Variables**:
```powershell
# Add system variable:
# Variable: OpenCV_DIR
# Value: C:\opencv_build\build\install

# Add to PATH:
# C:\opencv_build\build\install\x64\vc17\bin
```

7. **Verify CUDA Support**:
```powershell
# Create test C++ file to check cv::cuda::getCudaEnabledDeviceCount()
# Should return 1 (your RTX 4070)
```

---

## 7. Intel RealSense SDK 2.0

### Installation Steps

1. **Download**: [Intel RealSense SDK 2.54.2](https://github.com/IntelRealSense/librealsense/releases)
   - File: `Intel.RealSense.SDK-WIN10-2.54.2.exe` (~200 MB)

2. **Install**:
   - Run installer as administrator
   - Accept default installation path: `C:\Program Files (x86)\Intel RealSense SDK 2.0`
   - Install all components including:
     ✅ SDK Development Package
     ✅ RealSense Viewer
     ✅ Depth Quality Tool

3. **Verify Installation**:
```powershell
# Launch RealSense Viewer from Start Menu
# Connect RealSense D457 camera via USB-C
# Should detect camera as D450/D455 family
# Verify RGB and Depth streams work
```

4. **Environment Variable** (Auto-configured by installer):
   - `REALSENSE2_DIR` should point to SDK installation

---

## 8. Python 3.10+ (For Model Conversion)

> [!NOTE]
> **Purpose**: Python is only needed to convert YOLOv8-Pose model to ONNX/TensorRT format. Not required at runtime.

### Installation Steps

1. **Download**: [Python 3.11.x Windows x64 Installer](https://www.python.org/downloads/)

2. **Install**:
   - ✅ Check "Add Python to PATH"
   - Choose "Install Now"

3. **Install Required Packages**:
```powershell
pip install --upgrade pip
pip install ultralytics onnx onnxruntime-gpu tensorrt
```

---

## 9. Git (Version Control)

### Installation Steps

1. **Download**: [Git for Windows](https://git-scm.com/download/win)

2. **Install**: Use default options

3. **Verify**:
```powershell
git --version
```

---

## Final Verification Checklist

Before proceeding to build the project, verify all tools are accessible:

```powershell
# Open PowerShell and run these commands:

cl                          # Visual Studio C++ compiler
cmake --version             # CMake 3.27+
nvcc --version             # CUDA 12.3
nvidia-smi                 # Shows RTX 4070
python --version           # Python 3.10+
git --version              # Git

# Check environment variables:
echo $env:CUDA_PATH        # Should show CUDA installation path
echo $env:TENSORRT_DIR     # Should show TensorRT path
echo $env:OpenCV_DIR       # Should show OpenCV build path
```

> [!TIP]
> **Restart Required**: After installing all dependencies and setting environment variables, restart your computer to ensure all paths are properly loaded.

---

## Disk Space Requirements

- Visual Studio 2022: ~10 GB
- CUDA Toolkit 12.3: ~6 GB
- cuDNN: ~500 MB
- TensorRT: ~2 GB
- OpenCV (source + build): ~15 GB
- RealSense SDK: ~500 MB
- **Total: ~35 GB** (can delete OpenCV source after building)

---

## Troubleshooting

### Issue: "nvcc not found"
**Solution**: Restart terminal/computer after CUDA installation. Verify `CUDA_PATH` environment variable.

### Issue: OpenCV CMake can't find CUDA
**Solution**: Set `CUDA_TOOLKIT_ROOT_DIR` manually in CMake GUI to your CUDA installation path.

### Issue: RealSense camera not detected
**Solution**: 
1. Use USB 3.0/3.1 port (blue port)
2. Update camera firmware using RealSense Viewer
3. Reinstall RealSense SDK drivers

### Issue: TensorRT import error in Python
**Solution**: Ensure you installed the correct Python wheel for your Python version (e.g., cp311 for Python 3.11).

---

**Next Step**: Proceed to [MODEL_SETUP.md](file:///c:/Users/basti/source/repos/RealsenseBodyPose/MODEL_SETUP.md) to prepare the YOLOv8-Pose AI model.
