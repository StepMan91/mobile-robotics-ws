#define NOMINMAX
// Main Application - Real-Time 3D Skeletal Tracking

#include "DataRecorder.h"
#include "PoseEstimator.h"
#include "RealSenseCamera.h"
#include "SkeletonProjector.h"
#include "UdpSender.h"
#include "Utils.h"
#include "Visualizer.h"

#include <exception>
#include <iostream>
#include <signal.h>
#include <string>

using namespace RealsenseBodyPose;

// Global flag for graceful shutdown
volatile bool g_running = true;

void signalHandler(int signum) {
  appLog(LogLevel::INFO, "Interrupt signal received. Shutting down...");
  g_running = false;
}

void printUsage(const char *programName) {
  std::cout << "\n=== RealSense 3D Skeletal Tracking ===\n\n";
  std::cout << "Usage: " << programName << " [OPTIONS]\n\n";
  std::cout << "Options:\n";
  std::cout << "  --model <path>      Path to ONNX model file (required)\n";
  std::cout << "  --width <int>       Camera width (default: 1280)\n";
  std::cout << "  --height <int>      Camera height (default: 720)\n";
  std::cout << "  --fps <int>         Camera FPS (default: 30)\n";
  std::cout << "  --confidence <f>    Detection confidence threshold (default: "
               "0.5)\n";
  std::cout << "  --help              Show this help message\n\n";
  std::cout << "Example:\n";
  std::cout << "  " << programName << " --model models/yolov8n-pose.onnx\n\n";
}

int main(int argc, char *argv[]) {
  // Register signal handler for CTRL+C
  signal(SIGINT, signalHandler);
  signal(SIGTERM, signalHandler);

  // Parse command-line arguments
  std::string modelPath;
  int cameraWidth = 640;
  int cameraHeight = 480;
  int cameraFPS = 60;
  float confidenceThreshold = 0.3f;

  for (int i = 1; i < argc; i++) {
    std::string arg = argv[i];

    if (arg == "--help" || arg == "-h") {
      printUsage(argv[0]);
      return 0;
    } else if (arg == "--model" && i + 1 < argc) {
      modelPath = argv[++i];
    } else if (arg == "--width" && i + 1 < argc) {
      cameraWidth = std::stoi(argv[++i]);
    } else if (arg == "--height" && i + 1 < argc) {
      cameraHeight = std::stoi(argv[++i]);
    } else if (arg == "--fps" && i + 1 < argc) {
      cameraFPS = std::stoi(argv[++i]);
    } else if (arg == "--confidence" && i + 1 < argc) {
      confidenceThreshold = std::stof(argv[++i]);
    } else {
      std::cerr << "Unknown argument: " << arg << "\n";
      printUsage(argv[0]);
      return 1;
    }
  }

  // Validate required arguments
  if (modelPath.empty()) {
    std::cerr << "ERROR: --model argument is required\n";
    printUsage(argv[0]);
    return 1;
  }

  try {
    appLog(LogLevel::INFO, "=== RealSense 3D Skeletal Tracking ===");
    appLog(LogLevel::INFO, "Starting initialization...");

    // 1. Initialize RealSense Camera
    appLog(LogLevel::INFO, "\n[1/4] Initializing RealSense Camera...");
    RealSenseCamera::Config cameraConfig;
    cameraConfig.colorWidth = cameraWidth;
    cameraConfig.colorHeight = cameraHeight;
    cameraConfig.colorFPS = cameraFPS;
    cameraConfig.depthWidth = cameraWidth;
    cameraConfig.depthHeight = cameraHeight;
    cameraConfig.depthFPS = cameraFPS;
    cameraConfig.enableAlignment = true;

    RealSenseCamera camera(cameraConfig);
    camera.start();

    // 2. Initialize Pose Estimator
    appLog(LogLevel::INFO, "\n[2/4] Initializing GPU Pose Estimator...");
    PoseEstimator::Config poseConfig(modelPath);
    poseConfig.confidenceThreshold = confidenceThreshold;
    PoseEstimator poseEstimator(poseConfig);
    poseEstimator.initialize();

    // 3. Initialize Skeleton Projector
    appLog(LogLevel::INFO, "\n[3/4] Initializing 3D Projector...");
    SkeletonProjector projector(camera.getColorIntrinsics(),
                                camera.getDepthScale());
    appLog(LogLevel::INFO, "✅ 3D Projector initialized");

    // 4. Initialize Visualizer
    appLog(LogLevel::INFO, "\n[4/4] Initializing Visualizer...");
    Visualizer visualizer;
    appLog(LogLevel::INFO, "✅ Visualizer initialized");

    // 5. Initialize UDP Sender (Network Bridge)
    appLog(LogLevel::INFO, "\n[5/5] Initializing Network Bridge (UDP)...");
    UdpSender udpSender("172.31.69.131", 8888);
    if (udpSender.initialize()) {
      appLog(LogLevel::INFO, "✅ UDP Sender ready on 172.31.69.131:8888");
    } else {
      appLog(LogLevel::WARNING,
             "⚠️ UDP Sender failed to initialize. Network features disabled.");
    }

    // 6. Initialize Data Recorder
    appLog(LogLevel::INFO, "\n[6/6] Initializing Data Recorder...");
    DataRecorder recorder;
    appLog(LogLevel::INFO, "✅ Data Recorder initialized");

    appLog(LogLevel::INFO, "\n✅✅✅ All systems ready! ✅✅✅");
    appLog(LogLevel::INFO, "Press ESC to quit\n");

    // Performance monitoring
    FPSCounter fpsCounter;
    Timer frameTimer;

    // Main loop
    while (g_running) {
      frameTimer.reset();

      // Step 1: Capture frames from camera
      cv::Mat colorImage, depthImage;
      if (!camera.captureFrames(colorImage, depthImage, 5000)) {
        appLog(LogLevel::WARNING, "Failed to capture frames");
        continue;
      }

      // Step 2: Run pose estimation (GPU)
      std::vector<Skeleton> skeletons = poseEstimator.estimate(colorImage);

      // Step 3: Project 2D keypoints to 3D using depth
      if (!skeletons.empty()) {
        projector.project(skeletons, depthImage);
        // Step 3b: Send data via UDP
        udpSender.send(skeletons);

        // Step 3c: Record data
        if (recorder.isRecording()) {
          recorder.record(skeletons);
        }
      }

      // Step 4: Visualize results
      fpsCounter.tick();
      visualizer.draw(colorImage, skeletons, fpsCounter.getFPS());
      visualizer.drawRecordingStatus(colorImage, recorder.isRecording());

      visualizer.print3DCoordinates(skeletons);

      // Step 6: Display and check for quit
      int key = visualizer.show(colorImage);
      if (visualizer.shouldQuit(key)) {
        appLog(LogLevel::INFO, "ESC pressed. Exiting...");
        break;
      }

      // Handle 'r' key for recording (key code 114 is 'r')
      if (key == 'r' || key == 'R') {
        if (recorder.isRecording()) {
          recorder.stop();
        } else {
          recorder.start();
        }
      }

      // Log performance metrics (Disabled for build stability)
      /*
      static int frameCount = 0;
      if (++frameCount % 100 == 0) {
          // ...
      }
      */
    }

    appLog(LogLevel::INFO, "\n=== Shutting down ===");
    camera.stop();
    appLog(LogLevel::INFO, "✅ Shutdown complete");

    return 0;

  } catch (const std::exception &e) {
    appLog(LogLevel::ERR, "Fatal error: " + std::string(e.what()));
    std::cerr << "\nApplication terminated due to error.\n";
    return 1;
  }
}
