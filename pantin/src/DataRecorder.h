#pragma once

#include "Utils.h"
#include <fstream>
#include <mutex>
#include <string>
#include <vector>


namespace RealsenseBodyPose {

/**
 * @brief Recorder for saving skeletal data to CSV
 *
 * Handles file creation with timestamps and thread-safe writing of skeleton
 * frames.
 */
class DataRecorder {
public:
  DataRecorder();
  ~DataRecorder();

  /**
   * @brief Start recording to a new file
   * @return true if file created successfully
   */
  bool start();

  /**
   * @brief Stop current recording
   */
  void stop();

  /**
   * @brief Check if currently recording
   */
  bool isRecording() const;

  /**
   * @brief Record a frame of skeletal data
   * @param skeletons Vector of detected skeletons
   */
  void record(const std::vector<Skeleton> &skeletons);

  /**
   * @brief Get current file path
   */
  std::string getCurrentFilePath() const { return currentFilePath_; }

private:
  std::ofstream file_;
  bool isRecording_;
  std::string currentFilePath_;
  std::mutex mutex_;
  long long frameCount_;

  // Helper to get current timestamp string
  std::string getTimestampString() const;
};

} // namespace RealsenseBodyPose
