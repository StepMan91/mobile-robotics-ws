// GPU-Accelerated Pose Estimator using OpenCV DNN with CUDA

#pragma once

#include "Utils.h"
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <memory>
#include <vector>
#include <string>

namespace RealsenseBodyPose {

/**
 * @brief GPU-accelerated pose estimation using OpenCV DNN with CUDA backend
 * 
 * Loads YOLOv8-Pose ONNX model and performs inference on RTX 4070
 */
class PoseEstimator {
public:
    /**
     * @brief Configuration for pose estimator
     */
    struct Config {
        std::string modelPath;          // Path to ONNX model file
        int inputWidth = 640;           // Model input width
        int inputHeight = 640;          // Model input height
        float confidenceThreshold = 0.5f;  // Minimum confidence for detection
        float nmsThreshold = 0.45f;     // Non-max suppression threshold
        int maxDetections = 10;         // Maximum number of people to detect
        
        Config() = default;
        explicit Config(const std::string& path) : modelPath(path) {}
    };
    
    /**
     * @brief Constructor
     * @param config Pose estimator configuration
     */
    explicit PoseEstimator(const Config& config);
    
    /**
     * @brief Destructor
     */
    ~PoseEstimator();
    
    /**
     * @brief Initialize model and GPU backend
     * @throws std::runtime_error if initialization fails
     */
    void initialize();
    
    /**
     * @brief Run pose estimation on RGB image
     * @param image Input RGB image (CV_8UC3)
     * @return Vector of detected skeletons (2D keypoints only)
     */
    std::vector<Skeleton> estimate(const cv::Mat& image);
    
    /**
     * @brief Check if estimator is initialized
     * @return true if ready for inference
     */
    bool isInitialized() const { return initialized_; }
    
    /**
     * @brief Get input dimensions
     */
    cv::Size getInputSize() const { return cv::Size(config_.inputWidth, config_.inputHeight); }

private:
    Config config_;
    bool initialized_;
    
    // OpenCV DNN network
    cv::dnn::Net net_;
    
    // Preprocessing parameters
    float scaleX_;
    float scaleY_;
    float padX_;
    float padY_;
    
    /**
     * @brief Load ONNX model
     */
    void loadModel();
    
    /**
     * @brief Preprocess image for model input
     * @param image Input image
     * @return Preprocessed blob
     */
    cv::Mat preprocess(const cv::Mat& image);
    
    /**
     * @brief Postprocess model output to extract skeletons
     * @param output Raw model output
     * @param imageWidth Original image width
     * @param imageHeight Original image height
     * @return Detected skeletons with 2D keypoints
     */
    std::vector<Skeleton> postprocess(const cv::Mat& output, int imageWidth, int imageHeight);
    
    /**
     * @brief Apply Non-Maximum Suppression
     * @param skeletons Input skeletons
     * @return Filtered skeletons after NMS
     */
    std::vector<Skeleton> applyNMS(const std::vector<Skeleton>& skeletons);
    
    /**
     * @brief Calculate Intersection over Union for two bounding boxes
     */
    float calculateIoU(const float* box1, const float* box2);
};

} // namespace RealsenseBodyPose
