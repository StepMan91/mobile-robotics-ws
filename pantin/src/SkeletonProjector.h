// 3D Skeleton Projector - converts 2D keypoints + depth to 3D coordinates

#pragma once

#include "Utils.h"
#include "RealSenseCamera.h"
#include <opencv2/opencv.hpp>
#include <librealsense2/rs.hpp>
#include <vector>

namespace RealsenseBodyPose {

/**
 * @brief Projects 2D skeleton keypoints to 3D world coordinates using depth data
 */
class SkeletonProjector {
public:
    /**
     * @brief Constructor
     * @param intrinsics Camera intrinsics for deprojection
     * @param depthScale Depth scale (meters per depth unit)
     */
    explicit SkeletonProjector(const rs2_intrinsics& intrinsics, float depthScale);
    
    /**
     * @brief Project 2D skeletons to 3D using depth image
     * @param skeletons Input skeletons with 2D keypoints (will be modified)
     * @param depthImage Aligned depth image (CV_16UC1)
     */
    void project(std::vector<Skeleton>& skeletons, const cv::Mat& depthImage);
    
    /**
     * @brief Project single 2D point to 3D
     * @param pixel 2D pixel coordinates
     * @param depth Depth value in millimeters
     * @return 3D point in meters
     */
    Keypoint3D projectPoint(const Keypoint2D& pixel, uint16_t depth);
    
private:
    rs2_intrinsics intrinsics_;
    float depthScale_;
    
    /**
     * @brief Sample depth value at pixel location with averaging
     * @param depthImage Depth image
     * @param x Pixel x coordinate
     * @param y Pixel y coordinate
     * @param radius Sampling radius (default 2 pixels)
     * @return Average depth value (millimeters), or 0 if invalid
     */
    uint16_t sampleDepth(const cv::Mat& depthImage, int x, int y, int radius = 2);
};

} // namespace RealsenseBodyPose
