// GPU-Accelerated Pose Estimator using OpenCV DNN with CUDA

#include "PoseEstimator.h"
#include <iostream>
#include <fstream>
#include <algorithm>
#include <numeric>
#include <cmath>

namespace RealsenseBodyPose {

PoseEstimator::PoseEstimator(const Config& config)
    : config_(config)
    , initialized_(false)
    , scaleX_(1.0f)
    , scaleY_(1.0f)
    , padX_(0.0f)
    , padY_(0.0f)
{
}

PoseEstimator::~PoseEstimator() {
    // OpenCV DNN cleans up automatically
}

void PoseEstimator::initialize() {
    std::cout << "[INFO] Initializing GPU Pose Estimator..." << std::endl;
    
    // Load model
    loadModel();
    
    // Check CUDA availability
    if (cv::cuda::getCudaEnabledDeviceCount() > 0) {
        cv::cuda::DeviceInfo deviceInfo;
        std::cout << "[INFO] ✅ GPU: " << deviceInfo.name() << std::endl;
        std::cout << "[INFO]   Compute Capability: " << deviceInfo.majorVersion() << "." 
                  << deviceInfo.minorVersion() << std::endl;
    } else {
        std::cout << "[WARN] ⚠️  No CUDA device found, using CPU" << std::endl;
    }
    
    initialized_ = true;
    std::cout << "[INFO] ✅ Pose Estimator initialized successfully!" << std::endl;
}

void PoseEstimator::loadModel() {
    std::cout << "[INFO] Loading ONNX model: " << config_.modelPath << std::endl;
    
    // Check file exists
    std::ifstream file(config_.modelPath);
    if (!file.good()) {
        throw std::runtime_error("Model file not found: " + config_.modelPath);
    }
    file.close();
    
    // Load ONNX model
    try {
        net_ = cv::dnn::readNetFromONNX(config_.modelPath);
    } catch (const cv::Exception& e) {
        throw std::runtime_error("Failed to load ONNX model: " + std::string(e.what()));
    }
    
    if (net_.empty()) {
        throw std::runtime_error("Failed to load ONNX model - network is empty");
    }
    
    // Set CUDA backend if available
    if (cv::cuda::getCudaEnabledDeviceCount() > 0) {
        net_.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
        net_.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA);
        std::cout << "[INFO] ✅ Using CUDA backend for inference" << std::endl;
    } else {
        net_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
        net_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
        std::cout << "[WARN] Using CPU backend for inference" << std::endl;
    }
    
    std::cout << "[INFO] ✅ Model loaded successfully" << std::endl;
}

std::vector<Skeleton> PoseEstimator::estimate(const cv::Mat& image) {
    if (!initialized_) {
        throw std::runtime_error("Pose estimator not initialized");
    }
    
    if (image.empty()) {
        return {};
    }
    
    // Preprocess
    cv::Mat blob = preprocess(image);
    
    // Inference
    net_.setInput(blob);
    cv::Mat output = net_.forward();
    
    // Postprocess
    return postprocess(output, image.cols, image.rows);
}

cv::Mat PoseEstimator::preprocess(const cv::Mat& image) {
    // Calculate letterbox resize parameters
    float scaleW = static_cast<float>(config_.inputWidth) / image.cols;
    float scaleH = static_cast<float>(config_.inputHeight) / image.rows;
    float scale = std::min(scaleW, scaleH);
    
    int newW = static_cast<int>(image.cols * scale);
    int newH = static_cast<int>(image.rows * scale);
    
    // Calculate padding
    padX_ = (config_.inputWidth - newW) / 2.0f;
    padY_ = (config_.inputHeight - newH) / 2.0f;
    
    // Store scale for postprocessing
    scaleX_ = scale;
    scaleY_ = scale;
    
    // Resize image
    cv::Mat resized;
    cv::resize(image, resized, cv::Size(newW, newH));
    
    // Create padded image
    cv::Mat padded = cv::Mat::zeros(config_.inputHeight, config_.inputWidth, CV_8UC3);
    resized.copyTo(padded(cv::Rect(static_cast<int>(padX_), static_cast<int>(padY_), newW, newH)));
    
    // Convert to blob (CHW format, RGB, normalized to [0,1])
    cv::Mat blob = cv::dnn::blobFromImage(padded, 1.0/255.0, cv::Size(config_.inputWidth, config_.inputHeight),
                                          cv::Scalar(0,0,0), true, false);
    
    return blob;
}

std::vector<Skeleton> PoseEstimator::postprocess(const cv::Mat& output, int imageWidth, int imageHeight) {
    std::vector<Skeleton> skeletons;
    
    // YOLOv8-Pose output shape: [1, 56, 8400]
    // Channels: [x, y, w, h, confidence, {17 keypoints * 3}]
    // Each keypoint: [x, y, confidence]
    
    const float* data = (float*)output.data;
    int numAnchors = output.size[2];  // 8400
    int numChannels = output.size[1]; // 56
    
    for (int i = 0; i < numAnchors; i++) {
        // Get bbox confidence (index 4)
        float confidence = data[4 * numAnchors + i];
        
        if (confidence < config_.confidenceThreshold) {
            continue;
        }
        
        // Extract bounding box (indices 0-3)
        float cx = data[0 * numAnchors + i];  // center x
        float cy = data[1 * numAnchors + i];  // center y
        float w = data[2 * numAnchors + i];   // width
        float h = data[3 * numAnchors + i];   // height
        
        // Scale back to original image coordinates (undo letterbox)
        cx = (cx - padX_) / scaleX_;
        cy = (cy - padY_) / scaleY_;
        w = w / scaleX_;
        h = h / scaleY_;
        
        float x1 = cx - w / 2;
        float y1 = cy - h / 2;
        
        // Extract 17 keypoints
        Skeleton skeleton;
        skeleton.overallConfidence = confidence;
        skeleton.bbox[0] = x1;
        skeleton.bbox[1] = y1;
        skeleton.bbox[2] = w;
        skeleton.bbox[3] = h;
        
        for (int k = 0; k < 17; k++) {
            int baseIdx = 5 + k * 3;  // Start after bbox + confidence
            float kx = data[(baseIdx + 0) * numAnchors + i];
            float ky = data[(baseIdx + 1) * numAnchors + i];
            float kconf = data[(baseIdx + 2) * numAnchors + i];
            
            // Scale back to original coordinates
            kx = (kx - padX_) / scaleX_;
            ky = (ky - padY_) / scaleY_;
            
            skeleton.keypoints2D[k] = Keypoint2D(kx, ky, kconf);
        }
        
        skeletons.push_back(skeleton);
    }
    
    // Apply NMS
    return applyNMS(skeletons);
}

std::vector<Skeleton> PoseEstimator::applyNMS(const std::vector<Skeleton>& skeletons) {
    if (skeletons.empty()) {
        return {};
    }
    
    std::vector<Skeleton> result;
    std::vector<int> indices(skeletons.size());
    std::iota(indices.begin(), indices.end(), 0);
    
    // Sort by confidence (descending)
    std::sort(indices.begin(), indices.end(), [&skeletons](int a, int b) {
        return skeletons[a].overallConfidence > skeletons[b].overallConfidence;
    });
    
    std::vector<bool> suppressed(skeletons.size(), false);
    
    for (size_t i = 0; i < indices.size(); i++) {
        if (suppressed[i]) continue;
        
        int idx = indices[i];
        result.push_back(skeletons[idx]);
        
        if (result.size() >= static_cast<size_t>(config_.maxDetections)) {
            break;
        }
        
        // Suppress overlapping boxes
        for (size_t j = i + 1; j < indices.size(); j++) {
            if (suppressed[j]) continue;
            
            int jdx = indices[j];
            float iou = calculateIoU(skeletons[idx].bbox, skeletons[jdx].bbox);
            
            if (iou > config_.nmsThreshold) {
                suppressed[j] = true;
            }
        }
    }
    
    return result;
}

float PoseEstimator::calculateIoU(const float* box1, const float* box2) {
    float x1 = std::max(box1[0], box2[0]);
    float y1 = std::max(box1[1], box2[1]);
    float x2 = std::min(box1[0] + box1[2], box2[0] + box2[2]);
    float y2 = std::min(box1[1] + box1[3], box2[1] + box2[3]);
    
    float intersection = std::max(0.0f, x2 - x1) * std::max(0.0f, y2 - y1);
    float area1 = box1[2] * box1[3];
    float area2 = box2[2] * box2[3];
    float unionArea = area1 + area2 - intersection;
    
    return (unionArea > 0) ? (intersection / unionArea) : 0.0f;
}

} // namespace RealsenseBodyPose
