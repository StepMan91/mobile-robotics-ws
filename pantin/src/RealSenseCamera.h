// RealSense Camera Wrapper

#pragma once

#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <memory>
#include <stdexcept>

namespace RealsenseBodyPose {

/**
 * @brief Wrapper for Intel RealSense D457 camera
 * 
 * Handles initialization, configuration, and frame acquisition
 * with automatic alignment of depth to color frames.
 */
class RealSenseCamera {
public:
    /**
     * @brief Camera configuration parameters
     */
    struct Config {
        int colorWidth = 1280;
        int colorHeight = 720;
        int colorFPS = 30;
        int depthWidth = 1280;
        int depthHeight = 720;
        int depthFPS = 30;
        bool enableAlignment = true;  // Align depth to color
        
        Config() = default;
    };
    
    /**
     * @brief Constructor
     * @param config Camera configuration
     */
    explicit RealSenseCamera(const Config& config = Config());
    
    /**
     * @brief Destructor - stops camera pipeline
     */
    ~RealSenseCamera();
    
    /**
     * @brief Initialize and start camera
     * @throws std::runtime_error if camera initialization fails
     */
    void start();
    
    /**
     * @brief Stop camera pipeline
     */
    void stop();
    
    /**
     * @brief Capture next frameset
     * @param colorImage Output RGB image (CV_8UC3)
     * @param depthImage Output depth image (CV_16UC1, millimeters)
     * @param timeout_ms Timeout in milliseconds (default 5000)
     * @return true if frames captured successfully, false on timeout
     */
    bool captureFrames(cv::Mat& colorImage, cv::Mat& depthImage, int timeout_ms = 5000);
    
    /**
     * @brief Get camera intrinsics for deprojection
     * @return RealSense intrinsics structure
     */
    rs2_intrinsics getColorIntrinsics() const { return colorIntrinsics_; }
    
    /**
     * @brief Get depth scale (meters per depth unit)
     * @return Depth scale value
     */
    float getDepthScale() const { return depthScale_; }
    
    /**
     * @brief Check if camera is running
     * @return true if pipeline is active
     */
    bool isRunning() const { return pipelineStarted_; }
    
    /**
     * @brief Get device serial number
     * @return Serial number string
     */
    std::string getSerialNumber() const;

private:
    Config config_;
    rs2::pipeline pipeline_;
    rs2::align* aligner_;  // Depth-to-color alignment
    rs2_intrinsics colorIntrinsics_;
    float depthScale_;
    bool pipelineStarted_;
    
    /**
     * @brief Configure pipeline with desired stream settings
     * @return Configuration object
     */
    rs2::config configurePipeline();
    
    /**
     * @brief Extract intrinsics from pipeline profile
     * @param profile Pipeline profile after start
     */
    void extractIntrinsics(const rs2::pipeline_profile& profile);
};

} // namespace RealsenseBodyPose
