cmake_minimum_required(VERSION 3.20)
project(RealsenseBodyPose LANGUAGES CXX CUDA)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Dependencies
find_package(OpenCV REQUIRED)
# Manual RealSense SDK Configuration
set(REALSENSE_DIR "C:/Users/basti/Documents/RealSense SDK 2.0")
include_directories("${REALSENSE_DIR}/include")
set(REALSENSE_LIBS "${REALSENSE_DIR}/lib/x64/realsense2.lib")

# find_package(realsense2 REQUIRED) # Failed, using manual paths
find_package(CUDAToolkit REQUIRED)

# Manual TensorRT Configuration
set(TENSORRT_DIR "C:/Users/basti/source/repos/RealsenseBodyPose/ThirdParty/TensorRT-8.6.1.6")
include_directories("${TENSORRT_DIR}/include")
set(TENSORRT_LIBRARY 
    "${TENSORRT_DIR}/lib/nvinfer.lib"
    "${TENSORRT_DIR}/lib/nvinfer_plugin.lib"
    "${TENSORRT_DIR}/lib/nvonnxparser.lib"
    "${TENSORRT_DIR}/lib/nvparsers.lib"
)

# find_path(TENSORRT_INCLUDE_DIR NvInfer.h ...) # Failed, using manual paths

# Source files
file(GLOB SOURCES "src/*.cpp" "src/*.h" "src/*.hpp")

# Executable
add_executable(RealsenseBodyPose ${SOURCES})

# Include directories
target_include_directories(RealsenseBodyPose PRIVATE src ${TENSORRT_INCLUDE_DIR})

# Link libraries
target_link_libraries(RealsenseBodyPose PRIVATE
    ${OpenCV_LIBS}
    ${REALSENSE_LIBS}
    CUDA::cudart
    ${TENSORRT_LIBRARY}
)

# Properties
set_property(TARGET RealsenseBodyPose PROPERTY CUDA_SEPARABLE_COMPILATION ON)
