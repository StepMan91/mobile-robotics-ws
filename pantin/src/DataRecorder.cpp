#include "DataRecorder.h"
#include <ctime>
#include <direct.h> // For _mkdir on Windows
#include <iomanip>
#include <iostream>
#include <sstream>

namespace RealsenseBodyPose {

DataRecorder::DataRecorder() : isRecording_(false), frameCount_(0) {
  // Ensure recordings directory exists
  _mkdir("recordings");
}

DataRecorder::~DataRecorder() { stop(); }

bool DataRecorder::start() {
  std::lock_guard<std::mutex> lock(mutex_);

  if (isRecording_) {
    return true;
  }

  std::string timestamp = getTimestampString();
  currentFilePath_ = "recordings/recording_" + timestamp + ".csv";

  file_.open(currentFilePath_);
  if (!file_.is_open()) {
    std::cerr << "Failed to create recording file: " << currentFilePath_
              << std::endl;
    return false;
  }

  // Write CSV Header
  file_ << "Timestamp,FrameIndex,PersonID,Confidence,";

  // Define names consistent with UDP update
  const std::vector<std::string> jointNames = {
      "Nose",         "LeftEye",       "RightEye",  "LeftEar",    "RightEar",
      "LeftShoulder", "RightShoulder", "LeftElbow", "RightElbow", "LeftWrist",
      "RightWrist",   "LeftHip",       "RightHip",  "LeftKnee",   "RightKnee",
      "LeftAnkle",    "RightAnkle"};

  // Standard 17 Keypoints
  for (const auto &name : jointNames) {
    file_ << name << "_X," << name << "_Y," << name << "_Z," << name
          << "_Confidence,";
  }
  // Added Pelvis
  file_ << "Pelvis_X,Pelvis_Y,Pelvis_Z,Pelvis_Confidence";

  file_ << "\n";

  isRecording_ = true;
  frameCount_ = 0;
  std::cout << "[REC] Started recording to " << currentFilePath_ << std::endl;

  return true;
}

void DataRecorder::stop() {
  std::lock_guard<std::mutex> lock(mutex_);

  if (isRecording_) {
    file_.close();
    isRecording_ = false;
    std::cout << "[REC] Stopped recording. Saved " << frameCount_ << " frames."
              << std::endl;
  }
}

bool DataRecorder::isRecording() const { return isRecording_; }

void DataRecorder::record(const std::vector<Skeleton> &skeletons) {
  std::lock_guard<std::mutex> lock(mutex_);

  if (!isRecording_ || !file_.is_open()) {
    return;
  }

  auto now = std::chrono::system_clock::now();
  auto timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
                       now.time_since_epoch())
                       .count();

  for (size_t i = 0; i < skeletons.size(); i++) {
    const auto &skel = skeletons[i];

    file_ << timestamp << "," << frameCount_ << "," << i
          << "," // Person ID (just index for now)
          << std::fixed << std::setprecision(4) << skel.overallConfidence
          << ",";

    // Write 3D keypoints (Standard 17)
    for (size_t k = 0; k < skel.keypoints3D.size(); k++) {
      const auto &kp = skel.keypoints3D[k];
      file_ << kp.x << "," << kp.y << "," << kp.z << "," << kp.confidence
            << ",";
    }

    // Calculate and Write Pelvis (Midpoint of Hips 11 & 12)
    float px = 0, py = 0, pz = 0, pConf = 0;
    if (skel.keypoints3D.size() > 12) {
      const auto &lHip = skel.keypoints3D[11];
      const auto &rHip = skel.keypoints3D[12];
      if (lHip.isValid() && rHip.isValid()) {
        px = (lHip.x + rHip.x) / 2.0f;
        py = (lHip.y + rHip.y) / 2.0f;
        pz = (lHip.z + rHip.z) / 2.0f;
        pConf = (lHip.confidence + rHip.confidence) / 2.0f;
      }
    }
    file_ << px << "," << py << "," << pz << "," << pConf;

    file_ << "\n";
  }

  frameCount_++;
}

std::string DataRecorder::getTimestampString() const {
  auto now = std::chrono::system_clock::now();
  std::time_t now_c = std::chrono::system_clock::to_time_t(now);
  std::tm now_tm;
  localtime_s(&now_tm, &now_c);

  std::stringstream ss;
  ss << std::put_time(&now_tm, "%Y%m%d_%H%M%S");
  return ss.str();
}

} // namespace RealsenseBodyPose
