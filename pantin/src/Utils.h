// Utility header with common definitions

#pragma once

#include <chrono>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

namespace RealsenseBodyPose {

// Joint names for COCO 17-keypoint skeleton
enum class JointType {
  NOSE = 0,
  LEFT_EYE = 1,
  RIGHT_EYE = 2,
  LEFT_EAR = 3,
  RIGHT_EAR = 4,
  LEFT_SHOULDER = 5,
  RIGHT_SHOULDER = 6,
  LEFT_ELBOW = 7,
  RIGHT_ELBOW = 8,
  LEFT_WRIST = 9,
  RIGHT_WRIST = 10,
  LEFT_HIP = 11,
  RIGHT_HIP = 12,
  LEFT_KNEE = 13,
  RIGHT_KNEE = 14,
  LEFT_ANKLE = 15,
  RIGHT_ANKLE = 16
};

// 2D Keypoint structure
struct Keypoint2D {
  float x;          // Pixel x-coordinate
  float y;          // Pixel y-coordinate
  float confidence; // Confidence score [0-1]

  Keypoint2D() : x(0), y(0), confidence(0) {}
  Keypoint2D(float x_, float y_, float conf_)
      : x(x_), y(y_), confidence(conf_) {}

  bool isValid() const {
    return confidence > 0.3f;
  } // Threshold for valid keypoint
};

// 3D Keypoint structure
struct Keypoint3D {
  float x;          // X in meters (right)
  float y;          // Y in meters (down)
  float z;          // Z in meters (forward/depth)
  float confidence; // Confidence score [0-1]

  Keypoint3D() : x(0), y(0), z(0), confidence(0) {}
  Keypoint3D(float x_, float y_, float z_, float conf_)
      : x(x_), y(y_), z(z_), confidence(conf_) {}

  bool isValid() const { return confidence > 0.3f && z > 0.1f && z < 10.0f; }
};

// Person skeleton (17 keypoints)
struct Skeleton {
  std::vector<Keypoint2D> keypoints2D; // 2D keypoints from pose estimation
  std::vector<Keypoint3D> keypoints3D; // 3D keypoints after depth projection
  float bbox[4];                       // Bounding box [x, y, w, h]
  float overallConfidence;             // Overall detection confidence

  Skeleton() : overallConfidence(0.0f) {
    keypoints2D.resize(17);
    keypoints3D.resize(17);
    bbox[0] = bbox[1] = bbox[2] = bbox[3] = 0;
  }
};

// Skeleton bone connections for visualization (COCO format)
const std::vector<std::pair<int, int>> SKELETON_CONNECTIONS = {
    // Face
    {0, 1},
    {0, 2}, // Nose to eyes
    {1, 3},
    {2, 4}, // Eyes to ears

    // Torso
    {5, 6}, // Shoulders
    {5, 11},
    {6, 12},  // Shoulders to hips
    {11, 12}, // Hips

    // Left arm
    {5, 7},
    {7, 9}, // Shoulder → Elbow → Wrist

    // Right arm
    {6, 8},
    {8, 10}, // Shoulder → Elbow → Wrist

    // Left leg
    {11, 13},
    {13, 15}, // Hip → Knee → Ankle

    // Right leg
    {12, 14},
    {14, 16} // Hip → Knee → Ankle
};

// Color palette for visualization (BGR format for OpenCV)
const std::vector<cv::Scalar> JOINT_COLORS = {
    cv::Scalar(255, 0, 0),   // Nose - Blue
    cv::Scalar(255, 85, 0),  // Left Eye
    cv::Scalar(255, 170, 0), // Right Eye
    cv::Scalar(255, 255, 0), // Left Ear
    cv::Scalar(170, 255, 0), // Right Ear
    cv::Scalar(85, 255, 0),  // Left Shoulder - Green
    cv::Scalar(0, 255, 0),   // Right Shoulder
    cv::Scalar(0, 255, 85),  // Left Elbow
    cv::Scalar(0, 255, 170), // Right Elbow
    cv::Scalar(0, 255, 255), // Left Wrist - Cyan
    cv::Scalar(0, 170, 255), // Right Wrist
    cv::Scalar(0, 85, 255),  // Left Hip
    cv::Scalar(0, 0, 255),   // Right Hip - Red
    cv::Scalar(85, 0, 255),  // Left Knee
    cv::Scalar(170, 0, 255), // Right Knee
    cv::Scalar(255, 0, 255), // Left Ankle - Magenta
    cv::Scalar(255, 0, 170)  // Right Ankle
};

// Joint name lookup
inline std::string getJointName(int index) {
  const std::vector<std::string> names = {
      "Nose",        "Left Eye",      "Right Eye",      "Left Ear",
      "Right Ear",   "Left Shoulder", "Right Shoulder", "Left Elbow",
      "Right Elbow", "Left Wrist",    "Right Wrist",    "Left Hip",
      "Right Hip",   "Left Knee",     "Right Knee",     "Left Ankle",
      "Right Ankle"};
  return (index >= 0 && index < 17) ? names[index] : "Unknown";
}

// Performance timer utility
class Timer {
public:
  Timer() : start_(std::chrono::high_resolution_clock::now()) {}

  void reset() { start_ = std::chrono::high_resolution_clock::now(); }

  // Returns elapsed time in milliseconds
  double elapsed() const {
    auto end = std::chrono::high_resolution_clock::now();
    return std::chrono::duration<double, std::milli>(end - start_).count();
  }

private:
  std::chrono::time_point<std::chrono::high_resolution_clock> start_;
};

// FPS calculator
class FPSCounter {
public:
  FPSCounter()
      : frameCount_(0), fps_(0.0),
        lastUpdate_(std::chrono::high_resolution_clock::now()) {}

  void tick() {
    frameCount_++;
    auto now = std::chrono::high_resolution_clock::now();
    auto elapsed = std::chrono::duration<double>(now - lastUpdate_).count();

    if (elapsed >= 1.0) { // Update FPS every second
      fps_ = frameCount_ / elapsed;
      frameCount_ = 0;
      lastUpdate_ = now;
    }
  }

  double getFPS() const { return fps_; }

private:
  int frameCount_;
  double fps_;
  std::chrono::time_point<std::chrono::high_resolution_clock> lastUpdate_;
};

// Logging utilities
enum class LogLevel { INFO, WARNING, ERR };

#ifndef NOMINMAX
#define NOMINMAX
#endif

inline void appLog(LogLevel level, const std::string &message) {
  std::string prefix;
  switch (level) {
  case LogLevel::INFO:
    prefix = "[INFO] ";
    break;
  case LogLevel::WARNING:
    prefix = "[WARN] ";
    break;
  case LogLevel::ERR:
    prefix = "[ERROR] ";
    break;
  }
  std::cout << prefix << message << std::endl;
}

} // namespace RealsenseBodyPose
