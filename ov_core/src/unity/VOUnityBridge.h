/**
 * @file VOUnityBridge.h
 * @brief Bridge class connecting Unity C API to VisualOdometry core
 */

#ifndef VO_UNITY_BRIDGE_H
#define VO_UNITY_BRIDGE_H

#include "vo_unity_api.h"

#include <memory>
#include <mutex>
#include <vector>

#include <Eigen/Dense>

namespace ov_core {

// Forward declarations
class VisualOdometry;
class TrackBase;

/**
 * @brief Singleton bridge class for Unity integration
 *
 * This class manages the lifecycle of the VisualOdometry system and provides
 * thread-safe access to its functionality through the C API.
 */
class VOUnityBridge {
public:
    /**
     * @brief Get the singleton instance
     * @return Reference to the singleton instance
     */
    static VOUnityBridge& getInstance();

    // Delete copy/move constructors and assignment operators
    VOUnityBridge(const VOUnityBridge&) = delete;
    VOUnityBridge& operator=(const VOUnityBridge&) = delete;
    VOUnityBridge(VOUnityBridge&&) = delete;
    VOUnityBridge& operator=(VOUnityBridge&&) = delete;

    /**
     * @brief Initialize the VO system
     * @param camera Camera intrinsic parameters
     * @param tracking Tracking parameters (can be nullptr for defaults)
     * @return Error code
     */
    VOErrorCode initialize(const VOCameraParams& camera,
                           const VOTrackingParams* tracking);

    /**
     * @brief Shutdown the VO system and release resources
     * @return Error code
     */
    VOErrorCode shutdown();

    /**
     * @brief Check if the system is initialized
     * @return true if initialized
     */
    bool isInitialized() const;

    /**
     * @brief Process a new camera frame
     * @param data Image pixel data
     * @param width Image width
     * @param height Image height
     * @param channels Number of channels (1 or 4)
     * @param timestamp Frame timestamp
     * @return Error code
     */
    VOErrorCode processFrame(const uint8_t* data, int width, int height,
                             int channels, double timestamp);

    /**
     * @brief Get the number of tracked features
     * @return Feature count, or -1 if not initialized
     */
    int getFeatureCount() const;

    /**
     * @brief Get tracked feature data
     * @param out Output array
     * @param maxCount Maximum features to retrieve
     * @param outCount Actual count retrieved
     * @return Error code
     */
    VOErrorCode getFeatures(VOFeature* out, int maxCount, int* outCount) const;

    /**
     * @brief Get the current camera pose
     * @param out Output pose structure
     * @return Error code
     */
    VOErrorCode getPose(VOPose* out) const;

    /**
     * @brief Get the 3D point cloud
     * @param out Output array for points (x,y,z triplets)
     * @param maxPoints Maximum points to retrieve
     * @param outCount Actual count retrieved
     * @return Error code
     */
    VOErrorCode getPointCloud(float* out, int maxPoints, int* outCount) const;

    /**
     * @brief Set maximum feature count
     * @param count New maximum (must be > 0)
     * @return Error code
     */
    VOErrorCode setMaxFeatures(int count);

    /**
     * @brief Set FAST detector threshold
     * @param threshold New threshold (must be > 0)
     * @return Error code
     */
    VOErrorCode setFastThreshold(int threshold);

private:
    VOUnityBridge();
    ~VOUnityBridge();

    /**
     * @brief Convert OpenCV pose to Unity coordinate system
     *
     * OpenCV: Right-handed, X-right, Y-down, Z-forward
     * Unity: Left-handed, X-right, Y-up, Z-forward
     *
     * @param pose 4x4 transformation matrix in OpenCV coordinates
     * @param out Output pose in Unity coordinates
     */
    void convertToUnityCoordinates(const Eigen::Matrix4d& pose, VOPose& out) const;

    std::unique_ptr<VisualOdometry> vo_;
    mutable std::mutex mutex_;
    bool initialized_;
    
    // Cached parameters
    VOCameraParams camera_params_;
    VOTrackingParams tracking_params_;
    
    // Last valid pose for fallback
    VOPose last_valid_pose_;
};

} // namespace ov_core

#endif // VO_UNITY_BRIDGE_H
