/**
 * @file VOUnityBridge.h
 * @brief Bridge class connecting Unity C API to full VIO (VioManager MSCKF)
 */

#ifndef VO_UNITY_BRIDGE_H
#define VO_UNITY_BRIDGE_H

#include "vo_unity_api.h"

#include <atomic>
#include <memory>
#include <mutex>
#include <unordered_map>
#include <vector>

#include <Eigen/Dense>
#include <opencv2/core.hpp>

#if defined(__ANDROID__) || defined(ANDROID)
#include "AndroidImuCollector.h"
#endif

// Forward declarations
namespace ov_srvins {
class VioManager;
struct VioManagerOptions;
} // namespace ov_srvins

/**
 * @brief Singleton bridge class for Unity integration with full MSCKF VIO
 *
 * This class manages the lifecycle of the VioManager (MSCKF) system and provides
 * thread-safe access to its functionality through the C API.
 */
class VOUnityBridge {
public:
  static VOUnityBridge &getInstance();

  VOUnityBridge(const VOUnityBridge &) = delete;
  VOUnityBridge &operator=(const VOUnityBridge &) = delete;
  VOUnityBridge(VOUnityBridge &&) = delete;
  VOUnityBridge &operator=(VOUnityBridge &&) = delete;

  /**
   * @brief Initialize the full VIO system
   */
  VOErrorCode initialize(const VOCameraParams &camera,
                         const VOImuParams *imu_params,
                         const VOExtrinsics *extrinsics,
                         const VOTrackingParams *tracking);

  VOErrorCode shutdown();
  bool isInitialized() const;

  /**
   * @brief Process a new camera frame through the VIO pipeline
   */
  VOErrorCode processFrame(const uint8_t *data, int width, int height,
                           int channels, double timestamp);

  /**
   * @brief Feed IMU measurement to VioManager
   */
  VOErrorCode feedImu(const VOImuData &imu);

  /**
   * @brief Start native IMU collection (Android only)
   * @param target_hz Target sampling rate
   */
  VOErrorCode startNativeImu(int target_hz);

  /**
   * @brief Stop native IMU collection
   */
  VOErrorCode stopNativeImu();

  /**
   * @brief Check if native IMU is running
   */
  bool isNativeImuRunning() const;

  /**
   * @brief Get native sensor timestamp in seconds (CLOCK_BOOTTIME)
   */
  double getNativeSensorTimestamp() const;

  /**
   * @brief Get current pose from VioManager state
   */
  VOErrorCode getPose(VOPose *out) const;

  int getFeatureCount() const;
  VOErrorCode getFeatures(VOFeature *out, int maxCount, int *outCount) const;
  VOErrorCode getPointCloud(float *out, int maxPoints, int *outCount) const;
  VOErrorCode setMaxFeatures(int count);
  VOErrorCode setFastThreshold(int threshold);

  VOErrorCode getDebugImage(uint8_t *out, int width, int height,
                            bool drawPoints, bool drawFlow) const;

  VOErrorCode setNativeTexture(void *ptr, int width, int height);
  void onRenderEvent(int eventID);
  VOErrorCode resetImu();

private:
  VOUnityBridge();
  ~VOUnityBridge();

  /**
   * @brief Convert VIO state pose (q_GtoI, p_IinG) to Unity coordinates
   *
   * sqrtVINS uses: right-handed, Z-up or Z-forward depending on context
   * Unity uses: left-handed, Y-up
   */
  void convertToUnityCoordinates(const Eigen::Vector3d &p_IinG,
                                 const Eigen::Matrix3d &R_GtoI,
                                 VOPose &out) const;

  std::shared_ptr<ov_srvins::VioManager> vio_manager_;
  mutable std::mutex mutex_;
  bool initialized_;

  // Cached parameters
  VOCameraParams camera_params_;

  // Last valid pose for fallback
  VOPose last_valid_pose_;

  // Current frame for debug visualization
  mutable cv::Mat current_frame_;

  // Previous feature positions for optical flow visualization
  mutable std::unordered_map<int, cv::Point2f> prev_feature_positions_;

  // Native texture
  void *native_texture_ptr_ = nullptr;
  int texture_width_ = 0;
  int texture_height_ = 0;

  // Frame counter for logging
  int frame_count_ = 0;

#if defined(__ANDROID__) || defined(ANDROID)
  // Native IMU collector
  std::unique_ptr<AndroidImuCollector> imu_collector_;
  // Cached IMU params for coordinate conversion in callback
  bool imu_use_native_ = false;
#endif

  // Native sensor timestamp (CLOCK_BOOTTIME seconds)
  mutable std::atomic<double> native_sensor_ts_{0.0};
};

#endif // VO_UNITY_BRIDGE_H
