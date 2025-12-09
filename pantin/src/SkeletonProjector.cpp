// Skeleton Projector Implementation

#include "SkeletonProjector.h"
#include <algorithm>

namespace RealsenseBodyPose {

SkeletonProjector::SkeletonProjector(const rs2_intrinsics &intrinsics,
                                     float depthScale)
    : intrinsics_(intrinsics), depthScale_(depthScale) {}

uint16_t SkeletonProjector::sampleDepth(const cv::Mat &depthImage, int x, int y,
                                        int radius) {
  // Bounds checking
  if (x < 0 || x >= depthImage.cols || y < 0 || y >= depthImage.rows) {
    return 0;
  }

  // Sample in a small neighborhood to reduce noise
  std::vector<uint16_t> validDepths;

  for (int dy = -radius; dy <= radius; dy++) {
    for (int dx = -radius; dx <= radius; dx++) {
      int sx = x + dx;
      int sy = y + dy;

      if (sx >= 0 && sx < depthImage.cols && sy >= 0 && sy < depthImage.rows) {
        uint16_t depth = depthImage.at<uint16_t>(sy, sx);
        if (depth > 0) { // Valid depth
          validDepths.push_back(depth);
        }
      }
    }
  }

  if (validDepths.empty()) {
    return 0;
  }

  // Return median depth for robustness
  std::sort(validDepths.begin(), validDepths.end());
  return validDepths[validDepths.size() / 2];
}

Keypoint3D SkeletonProjector::projectPoint(const Keypoint2D &pixel,
                                           uint16_t depth) {
  if (!pixel.isValid() || depth == 0) {
    return Keypoint3D(); // Invalid point
  }

  // Convert depth from millimeters to meters
  float depthMeters = depth * depthScale_;

  // Deproject pixel to 3D point using RealSense intrinsics
  float pixelCoords[2] = {pixel.x, pixel.y};
  float point3D[3];

  rs2_deproject_pixel_to_point(point3D, &intrinsics_, pixelCoords, depthMeters);

  // Create 3D keypoint with confidence from 2D detection
  return Keypoint3D(point3D[0], point3D[1], point3D[2], pixel.confidence);
}

void SkeletonProjector::project(std::vector<Skeleton> &skeletons,
                                const cv::Mat &depthImage) {
  if (depthImage.empty() || depthImage.type() != CV_16UC1) {
    appLog(LogLevel::ERR, "Invalid depth image for projection");
    return;
  }

  for (auto &skeleton : skeletons) {
    // Project each keypoint
    for (size_t i = 0; i < skeleton.keypoints2D.size(); i++) {
      const Keypoint2D &kpt2D = skeleton.keypoints2D[i];

      if (kpt2D.isValid()) {
        // Sample depth at keypoint location
        uint16_t depth = sampleDepth(depthImage, static_cast<int>(kpt2D.x),
                                     static_cast<int>(kpt2D.y));

        // Project to 3D
        skeleton.keypoints3D[i] = projectPoint(kpt2D, depth);
      } else {
        // Invalid 2D keypoint
        skeleton.keypoints3D[i] = Keypoint3D();
      }
    }
  }
}

} // namespace RealsenseBodyPose
