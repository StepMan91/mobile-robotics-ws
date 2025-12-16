// Visualization Module

#pragma once

#include "Utils.h"
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>


namespace RealsenseBodyPose {

/**
 * @brief Real-time visualization of skeletal tracking
 */
class Visualizer {
public:
  /**
   * @brief Configuration for visualizer
   */
  struct Config {
    std::string windowName = "3D Skeletal Tracking";
    bool showFPS = true;
    bool show3DCoords = true; // Print 3D coords to console
    int keypointRadius = 5;
    int boneThickness = 2;
    float confidenceThreshold = 0.3f;

    Config() = default;
  };

  /**
   * @brief Constructor
   * @param config Visualizer configuration
   */
  explicit Visualizer(const Config &config = Config());

  /**
   * @brief Destructor
   */
  ~Visualizer();

  /**
   * @brief Draw skeletons on image
   * @param image Input/output image
   * @param skeletons Detected skeletons with 2D and 3D keypoints
   * @param fps Current FPS (optional)
   */
  void draw(cv::Mat &image, const std::vector<Skeleton> &skeletons,
            double fps = 0.0);

  /**
   * @brief Draw recording status indicator
   * @param image Image to draw on
   * @param isRecording Whether recording is active
   */
  void drawRecordingStatus(cv::Mat &image, bool isRecording);

  /**
   * @brief Display image in window
   * @param image Image to display
   * @return Key code pressed (-1 if no key)
   */
  int show(const cv::Mat &image);

  /**
   * @brief Print 3D skeleton coordinates to console
   * @param skeletons Skeletons to print
   */
  void print3DCoordinates(const std::vector<Skeleton> &skeletons);

  /**
   * @brief Check if user wants to quit (ESC key)
   * @return true if ESC was pressed
   */
  bool shouldQuit(int key) const { return key == 27; } // ESC key

private:
  Config config_;

  /**
   * @brief Draw single keypoint
   */
  void drawKeypoint(cv::Mat &image, const Keypoint2D &kpt,
                    const cv::Scalar &color);

  /**
   * @brief Draw bone connection between two keypoints
   */
  void drawBone(cv::Mat &image, const Keypoint2D &kpt1, const Keypoint2D &kpt2,
                const cv::Scalar &color);

  /**
   * @brief Draw bounding box
   */
  void drawBBox(cv::Mat &image, const float *bbox, const cv::Scalar &color);

  /**
   * @brief Draw FPS counter
   */
  void drawFPS(cv::Mat &image, double fps);
};

} // namespace RealsenseBodyPose
