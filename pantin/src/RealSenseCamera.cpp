// RealSense Camera Implementation

#include "RealSenseCamera.h"
#include "Utils.h"
#include <iostream>

namespace RealsenseBodyPose {

RealSenseCamera::RealSenseCamera(const Config &config)
    : config_(config), aligner_(nullptr),
      depthScale_(0.001f) // Default: 1mm = 0.001m
      ,
      pipelineStarted_(false) {
  if (config_.enableAlignment) {
    aligner_ = new rs2::align(RS2_STREAM_COLOR);
  }
}

RealSenseCamera::~RealSenseCamera() {
  stop();
  if (aligner_) {
    delete aligner_;
  }
}

rs2::config RealSenseCamera::configurePipeline() {
  rs2::config cfg;

  // Enable color stream
  cfg.enable_stream(RS2_STREAM_COLOR, config_.colorWidth, config_.colorHeight,
                    RS2_FORMAT_BGR8, // OpenCV-compatible BGR format
                    config_.colorFPS);

  // Enable depth stream
  cfg.enable_stream(RS2_STREAM_DEPTH, config_.depthWidth, config_.depthHeight,
                    RS2_FORMAT_Z16, // 16-bit depth in millimeters
                    config_.depthFPS);

  return cfg;
}

void RealSenseCamera::extractIntrinsics(const rs2::pipeline_profile &profile) {
  // Get color stream profile
  auto colorStream =
      profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
  colorIntrinsics_ = colorStream.get_intrinsics();

  // Get depth scale
  auto depthSensor = profile.get_device().first<rs2::depth_sensor>();
  depthScale_ = depthSensor.get_depth_scale();

  appLog(LogLevel::INFO, "Camera Intrinsics:");
  appLog(LogLevel::INFO,
         "  Resolution: " + std::to_string(colorIntrinsics_.width) + "x" +
             std::to_string(colorIntrinsics_.height));
  appLog(LogLevel::INFO,
         "  Focal Length: fx=" + std::to_string(colorIntrinsics_.fx) +
             " fy=" + std::to_string(colorIntrinsics_.fy));
  appLog(LogLevel::INFO,
         "  Principal Point: cx=" + std::to_string(colorIntrinsics_.ppx) +
             " cy=" + std::to_string(colorIntrinsics_.ppy));
  appLog(LogLevel::INFO,
         "  Depth Scale: " + std::to_string(depthScale_) + " meters per unit");
}

void RealSenseCamera::start() {
  try {
    appLog(LogLevel::INFO, "Initializing RealSense camera...");

    // Configure and start pipeline
    rs2::config cfg = configurePipeline();
    rs2::pipeline_profile profile = pipeline_.start(cfg);
    pipelineStarted_ = true;

    // Extract intrinsics
    extractIntrinsics(profile);

    // Get device info
    auto device = profile.get_device();
    std::string deviceName = device.get_info(RS2_CAMERA_INFO_NAME);
    std::string serialNumber = device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
    std::string firmwareVersion =
        device.get_info(RS2_CAMERA_INFO_FIRMWARE_VERSION);

    appLog(LogLevel::INFO, "✅ Camera started successfully:");
    appLog(LogLevel::INFO, "  Device: " + deviceName);
    appLog(LogLevel::INFO, "  Serial: " + serialNumber);
    appLog(LogLevel::INFO, "  Firmware: " + firmwareVersion);
    appLog(LogLevel::INFO, "  Streams: " + std::to_string(config_.colorWidth) +
                               "x" + std::to_string(config_.colorHeight) +
                               " @ " + std::to_string(config_.colorFPS) +
                               " FPS");

    // Discard first few frames to allow auto-exposure to stabilize
    appLog(LogLevel::INFO, "Waiting for auto-exposure to stabilize...");
    for (int i = 0; i < 30; i++) {
      pipeline_.wait_for_frames();
    }
    appLog(LogLevel::INFO, "Camera ready!");

  } catch (const rs2::error &e) {
    throw std::runtime_error("RealSense error: " + std::string(e.what()));
  } catch (const std::exception &e) {
    throw std::runtime_error("Camera initialization failed: " +
                             std::string(e.what()));
  }
}

void RealSenseCamera::stop() {
  if (pipelineStarted_) {
    pipeline_.stop();
    pipelineStarted_ = false;
    appLog(LogLevel::INFO, "Camera stopped");
  }
}

bool RealSenseCamera::captureFrames(cv::Mat &colorImage, cv::Mat &depthImage,
                                    int timeout_ms) {
  try {
    // Wait for frames with timeout
    rs2::frameset frames = pipeline_.wait_for_frames(timeout_ms);

    // Apply alignment if enabled
    if (aligner_) {
      frames = aligner_->process(frames);
    }

    // Get color frame
    rs2::video_frame colorFrame = frames.get_color_frame();
    if (!colorFrame) {
      appLog(LogLevel::WARNING, "No color frame received");
      return false;
    }

    // Get depth frame
    rs2::depth_frame depthFrame = frames.get_depth_frame();
    if (!depthFrame) {
      appLog(LogLevel::WARNING, "No depth frame received");
      return false;
    }

    // Convert to OpenCV Mat
    colorImage =
        cv::Mat(cv::Size(colorFrame.get_width(), colorFrame.get_height()),
                CV_8UC3, (void *)colorFrame.get_data(), cv::Mat::AUTO_STEP)
            .clone();

    depthImage =
        cv::Mat(cv::Size(depthFrame.get_width(), depthFrame.get_height()),
                CV_16UC1, (void *)depthFrame.get_data(), cv::Mat::AUTO_STEP)
            .clone();

    return true;

  } catch (const rs2::error &e) {
    appLog(LogLevel::ERR, "Frame capture error: " + std::string(e.what()));
    return false;
  } catch (const std::exception &e) {
    appLog(LogLevel::ERR, "Unexpected error: " + std::string(e.what()));
    return false;
  }
}

std::string RealSenseCamera::getSerialNumber() const {
  if (!pipelineStarted_) {
    return "N/A";
  }
  try {
    auto profile = pipeline_.get_active_profile();
    auto device = profile.get_device();
    return device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
  } catch (...) {
    return "Unknown";
  }
}

} // namespace RealsenseBodyPose
