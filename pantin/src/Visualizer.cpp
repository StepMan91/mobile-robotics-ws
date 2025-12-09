// Visualizer Implementation

#include "Visualizer.h"
#include <iomanip>
#include <sstream>

namespace RealsenseBodyPose {

Visualizer::Visualizer(const Config &config) : config_(config) {
  cv::namedWindow(config_.windowName, cv::WINDOW_AUTOSIZE);
}

Visualizer::~Visualizer() { cv::destroyWindow(config_.windowName); }

void Visualizer::drawKeypoint(cv::Mat &image, const Keypoint2D &kpt,
                              const cv::Scalar &color) {
  if (!kpt.isValid() || kpt.confidence < config_.confidenceThreshold) {
    return;
  }

  cv::Point center(static_cast<int>(kpt.x), static_cast<int>(kpt.y));

  // Draw circle with white border
  cv::circle(image, center, config_.keypointRadius + 1,
             cv::Scalar(255, 255, 255), -1);
  cv::circle(image, center, config_.keypointRadius, color, -1);
}

void Visualizer::drawBone(cv::Mat &image, const Keypoint2D &kpt1,
                          const Keypoint2D &kpt2, const cv::Scalar &color) {
  if (!kpt1.isValid() || !kpt2.isValid() ||
      kpt1.confidence < config_.confidenceThreshold ||
      kpt2.confidence < config_.confidenceThreshold) {
    return;
  }

  cv::Point pt1(static_cast<int>(kpt1.x), static_cast<int>(kpt1.y));
  cv::Point pt2(static_cast<int>(kpt2.x), static_cast<int>(kpt2.y));

  // Draw line with white border for visibility
  cv::line(image, pt1, pt2, cv::Scalar(255, 255, 255),
           config_.boneThickness + 2);
  cv::line(image, pt1, pt2, color, config_.boneThickness);
}

void Visualizer::drawBBox(cv::Mat &image, const float *bbox,
                          const cv::Scalar &color) {
  cv::Rect rect(bbox[0], bbox[1], bbox[2], bbox[3]);
  cv::rectangle(image, rect, color, 2);
}

void Visualizer::drawFPS(cv::Mat &image, double fps) {
  std::stringstream ss;
  ss << std::fixed << std::setprecision(1) << "FPS: " << fps;

  // Draw background rectangle
  cv::Size textSize =
      cv::getTextSize(ss.str(), cv::FONT_HERSHEY_SIMPLEX, 0.7, 2, nullptr);
  cv::rectangle(image, cv::Point(10, 10),
                cv::Point(20 + textSize.width, 40 + textSize.height),
                cv::Scalar(0, 0, 0), -1);

  // Draw text
  cv::putText(image, ss.str(), cv::Point(15, 35), cv::FONT_HERSHEY_SIMPLEX, 0.7,
              cv::Scalar(0, 255, 0), 2);
}

void Visualizer::draw(cv::Mat &image, const std::vector<Skeleton> &skeletons,
                      double fps) {
  // Draw each detected person
  for (size_t personIdx = 0; personIdx < skeletons.size(); personIdx++) {
    const auto &skeleton = skeletons[personIdx];

    // Use different colors for different people
    cv::Scalar bboxColor = cv::Scalar(0, 255, 0); // Green
    if (personIdx == 1)
      bboxColor = cv::Scalar(255, 0, 0); // Blue
    if (personIdx == 2)
      bboxColor = cv::Scalar(0, 0, 255); // Red

    // Draw bounding box
    drawBBox(image, skeleton.bbox, bboxColor);

    // Draw confidence score
    std::stringstream ss;
    ss << std::fixed << std::setprecision(2) << skeleton.overallConfidence;
    cv::putText(image, ss.str(),
                cv::Point(skeleton.bbox[0], skeleton.bbox[1] - 5),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, bboxColor, 2);

    // Draw skeleton bones
    for (const auto &connection : SKELETON_CONNECTIONS) {
      int idx1 = connection.first;
      int idx2 = connection.second;

      const Keypoint2D &kpt1 = skeleton.keypoints2D[idx1];
      const Keypoint2D &kpt2 = skeleton.keypoints2D[idx2];

      // Use cyan color for bones
      drawBone(image, kpt1, kpt2, cv::Scalar(255, 255, 0));
    }

    // Draw keypoints (on top of bones)
    for (size_t i = 0; i < skeleton.keypoints2D.size(); i++) {
      drawKeypoint(image, skeleton.keypoints2D[i], JOINT_COLORS[i]);
    }
  }

  // Draw FPS counter
  if (config_.showFPS && fps > 0) {
    drawFPS(image, fps);
  }

  // Draw instructions
  cv::putText(image, "Press ESC to quit | 'r' to Record",
              cv::Point(10, image.rows - 15), cv::FONT_HERSHEY_SIMPLEX, 0.5,
              cv::Scalar(255, 255, 255), 1);
}

void Visualizer::drawRecordingStatus(cv::Mat &image, bool isRecording) {
  if (!isRecording)
    return;

  // Blinking effect for recording indicator (every 0.5s approx)
  auto now = std::chrono::steady_clock::now();
  auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                now.time_since_epoch())
                .count();

  if ((ms / 500) % 2 == 0) {
    // Draw red circle
    int radius = 10;
    cv::Point center(image.cols - 30, 30);
    cv::circle(image, center, radius, cv::Scalar(0, 0, 255), -1);

    // Draw REC text
    cv::putText(image, "REC", cv::Point(image.cols - 80, 35),
                cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);
  }
}

int Visualizer::show(const cv::Mat &image) {
  cv::imshow(config_.windowName, image);
  return cv::waitKey(1);
}

void Visualizer::print3DCoordinates(const std::vector<Skeleton> &skeletons) {
  if (!config_.show3DCoords || skeletons.empty()) {
    return;
  }

  std::cout << "\n========== 3D Skeleton Coordinates ==========\n";

  for (size_t personIdx = 0; personIdx < skeletons.size(); personIdx++) {
    const auto &skeleton = skeletons[personIdx];

    std::cout << "\nPerson " << (personIdx + 1)
              << " (Confidence: " << std::fixed << std::setprecision(2)
              << skeleton.overallConfidence << "):\n";

    // Print major joints only for clarity
    std::vector<int> majorJoints = {
        0,      // Nose (Head)
        5,  6,  // Shoulders
        9,  10, // Wrists (Hands)
        11, 12, // Hips
        15, 16  // Ankles (Feet)
    };

    for (int jointIdx : majorJoints) {
      const Keypoint3D &kpt3D = skeleton.keypoints3D[jointIdx];

      if (kpt3D.isValid()) {
        std::cout << "  " << std::setw(15) << std::left
                  << getJointName(jointIdx) << ": "
                  << "X=" << std::setw(7) << std::fixed << std::setprecision(3)
                  << kpt3D.x << "m  "
                  << "Y=" << std::setw(7) << std::fixed << std::setprecision(3)
                  << kpt3D.y << "m  "
                  << "Z=" << std::setw(7) << std::fixed << std::setprecision(3)
                  << kpt3D.z << "m  "
                  << "(conf: " << std::setprecision(2) << kpt3D.confidence
                  << ")\n";
      }
    }
  }

  std::cout << "============================================\n" << std::flush;
}

} // namespace RealsenseBodyPose
