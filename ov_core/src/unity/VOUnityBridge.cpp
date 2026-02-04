/**
 * @file VOUnityBridge.cpp
 * @brief Implementation of the Unity bridge class
 */

#include "VOUnityBridge.h"
#include "track/TrackBase.h"
#include "visual_odometry/VisualOdometry.h"

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <cmath>
#include <cstring>

namespace ov_core {

VOUnityBridge &VOUnityBridge::getInstance() {
  static VOUnityBridge instance;
  return instance;
}

VOUnityBridge::VOUnityBridge() : vo_(nullptr), initialized_(false) {
  // Initialize last valid pose to identity
  std::memset(&last_valid_pose_, 0, sizeof(VOPose));
  last_valid_pose_.qw = 1.0f;
  last_valid_pose_.valid = 0;

  // Initialize default tracking params
  tracking_params_.max_features = 500;
  tracking_params_.fast_threshold = 20;
  tracking_params_.grid_x = 5;
  tracking_params_.grid_y = 4;
  tracking_params_.min_px_dist = 20;
}

VOUnityBridge::~VOUnityBridge() { shutdown(); }

VOErrorCode VOUnityBridge::initialize(const VOCameraParams &camera,
                                      const VOTrackingParams *tracking) {
  std::lock_guard<std::mutex> lock(mutex_);

  // Validate camera parameters
  if (camera.fx <= 0 || camera.fy <= 0) {
    return VO_ERROR_INVALID_PARAM;
  }
  if (camera.width <= 0 || camera.height <= 0) {
    return VO_ERROR_INVALID_PARAM;
  }
  if (camera.cx < 0 || camera.cx >= camera.width || camera.cy < 0 ||
      camera.cy >= camera.height) {
    return VO_ERROR_INVALID_PARAM;
  }

  // Release previous resources if already initialized
  if (initialized_) {
    vo_.reset();
    initialized_ = false;
  }

  // Store parameters
  camera_params_ = camera;
  if (tracking != nullptr) {
    // Validate tracking parameters
    if (tracking->max_features <= 0 || tracking->fast_threshold <= 0) {
      return VO_ERROR_INVALID_PARAM;
    }
    tracking_params_ = *tracking;
  }

  // Create VisualOdometry configuration
  VisualOdometry::Config config;
  config.calibration.fx = camera.fx;
  config.calibration.fy = camera.fy;
  config.calibration.cx = camera.cx;
  config.calibration.cy = camera.cy;
  config.calibration.image_width = camera.width;
  config.calibration.image_height = camera.height;

  // Set distortion coefficients
  config.calibration.distortion.clear();
  config.calibration.distortion.push_back(camera.k1);
  config.calibration.distortion.push_back(camera.k2);
  config.calibration.distortion.push_back(camera.p1);
  config.calibration.distortion.push_back(camera.p2);

  // Set tracking parameters
  config.num_features = tracking_params_.max_features;
  config.fast_threshold = tracking_params_.fast_threshold;

  // Disable viewer for Unity (no Pangolin)
  // config.viewer_config.enabled = false; -> Removed as member does not exist

  try {
    vo_ = std::make_unique<VisualOdometry>(config);
    initialized_ = true;

    // Reset last valid pose
    std::memset(&last_valid_pose_, 0, sizeof(VOPose));
    last_valid_pose_.qw = 1.0f;
    last_valid_pose_.valid = 0;

    return VO_SUCCESS;
  } catch (const std::exception &e) {
    vo_.reset();
    return VO_ERROR_INVALID_PARAM;
  }
}

VOErrorCode VOUnityBridge::shutdown() {
  std::lock_guard<std::mutex> lock(mutex_);

  vo_.reset();
  initialized_ = false;

  // Reset last valid pose
  std::memset(&last_valid_pose_, 0, sizeof(VOPose));
  last_valid_pose_.qw = 1.0f;
  last_valid_pose_.valid = 0;

  return VO_SUCCESS;
}

bool VOUnityBridge::isInitialized() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return initialized_;
}

VOErrorCode VOUnityBridge::processFrame(const uint8_t *data, int width,
                                        int height, int channels,
                                        double timestamp) {
  std::lock_guard<std::mutex> lock(mutex_);

  if (!initialized_ || vo_ == nullptr) {
    return VO_ERROR_NOT_INITIALIZED;
  }

  // Validate image dimensions
  if (width != camera_params_.width || height != camera_params_.height) {
    return VO_ERROR_INVALID_IMAGE;
  }

  // Validate channels
  if (channels != 1 && channels != 4) {
    return VO_ERROR_INVALID_IMAGE;
  }

  try {
    cv::Mat gray;

    if (channels == 1) {
      // Already grayscale
      gray =
          cv::Mat(height, width, CV_8UC1, const_cast<uint8_t *>(data)).clone();
    } else if (channels == 4) {
      // Convert RGBA to grayscale
      cv::Mat rgba(height, width, CV_8UC4, const_cast<uint8_t *>(data));
      cv::cvtColor(rgba, gray, cv::COLOR_RGBA2GRAY);
    }

    // Save current frame for debug visualization
    current_frame_ = gray.clone();

    // Process the frame
    vo_->processFrame(gray, timestamp);

    return VO_SUCCESS;
  } catch (const std::exception &e) {
    return VO_ERROR_TRACKING_FAILED;
  }
}

int VOUnityBridge::getFeatureCount() const {
  std::lock_guard<std::mutex> lock(mutex_);

  if (!initialized_ || vo_ == nullptr) {
    return -1;
  }

  TrackBase *tracker = vo_->getTracker();
  if (tracker == nullptr) {
    return 0;
  }

  // Get features from camera 0
  auto last_obs = tracker->get_last_obs();
  if (last_obs.find(0) != last_obs.end()) {
    return static_cast<int>(last_obs[0].size());
  }

  return 0;
}

VOErrorCode VOUnityBridge::getFeatures(VOFeature *out, int maxCount,
                                       int *outCount) const {
  std::lock_guard<std::mutex> lock(mutex_);

  if (!initialized_ || vo_ == nullptr) {
    *outCount = 0;
    return VO_ERROR_NOT_INITIALIZED;
  }

  TrackBase *tracker = vo_->getTracker();
  if (tracker == nullptr) {
    *outCount = 0;
    return VO_SUCCESS;
  }

  auto last_obs = tracker->get_last_obs();
  auto last_ids = tracker->get_last_ids();

  // Get features from camera 0
  if (last_obs.find(0) == last_obs.end() ||
      last_ids.find(0) == last_ids.end()) {
    *outCount = 0;
    return VO_SUCCESS;
  }

  const auto &keypoints = last_obs[0];
  const auto &ids = last_ids[0];

  int count = std::min(maxCount, static_cast<int>(keypoints.size()));

  for (int i = 0; i < count; ++i) {
    out[i].id = static_cast<int>(ids[i]);
    out[i].x = keypoints[i].pt.x;
    out[i].y = keypoints[i].pt.y;
    out[i].status = 1; // Tracked
  }

  *outCount = count;
  return VO_SUCCESS;
}

VOErrorCode VOUnityBridge::getPose(VOPose *out) const {
  std::lock_guard<std::mutex> lock(mutex_);

  if (!initialized_ || vo_ == nullptr) {
    *out = last_valid_pose_;
    return VO_ERROR_NOT_INITIALIZED;
  }

  try {
    Eigen::Matrix4d pose = vo_->getCurrentPose();
    convertToUnityCoordinates(pose, *out);

    // Update last valid pose if this one is valid
    if (out->valid) {
      const_cast<VOUnityBridge *>(this)->last_valid_pose_ = *out;
    } else {
      // Return last valid pose on failure
      *out = last_valid_pose_;
      return VO_ERROR_POSE_FAILED;
    }

    return VO_SUCCESS;
  } catch (const std::exception &e) {
    *out = last_valid_pose_;
    return VO_ERROR_POSE_FAILED;
  }
}

VOErrorCode VOUnityBridge::getPointCloud(float *out, int maxPoints,
                                         int *outCount) const {
  std::lock_guard<std::mutex> lock(mutex_);

  if (!initialized_ || vo_ == nullptr) {
    *outCount = 0;
    return VO_ERROR_NOT_INITIALIZED;
  }

  try {
    std::vector<Eigen::Vector3d> points = vo_->getPointCloud();

    int count = std::min(maxPoints, static_cast<int>(points.size()));

    for (int i = 0; i < count; ++i) {
      // Convert to Unity coordinates (flip Y)
      out[i * 3 + 0] = static_cast<float>(points[i].x());
      out[i * 3 + 1] = static_cast<float>(-points[i].y()); // Flip Y
      out[i * 3 + 2] = static_cast<float>(points[i].z());
    }

    *outCount = count;
    return VO_SUCCESS;
  } catch (const std::exception &e) {
    *outCount = 0;
    return VO_ERROR_TRACKING_FAILED;
  }
}

VOErrorCode VOUnityBridge::setMaxFeatures(int count) {
  std::lock_guard<std::mutex> lock(mutex_);

  if (count <= 0) {
    return VO_ERROR_INVALID_PARAM;
  }

  tracking_params_.max_features = count;

  if (initialized_ && vo_ != nullptr) {
    TrackBase *tracker = vo_->getTracker();
    if (tracker != nullptr) {
      tracker->set_num_features(count);
    }
  }

  return VO_SUCCESS;
}

VOErrorCode VOUnityBridge::setFastThreshold(int threshold) {
  std::lock_guard<std::mutex> lock(mutex_);

  if (threshold <= 0) {
    return VO_ERROR_INVALID_PARAM;
  }

  tracking_params_.fast_threshold = threshold;

  // Note: FAST threshold update requires tracker reconfiguration
  // which is not directly supported by TrackBase interface.
  // The new threshold will be used on next initialization.

  return VO_SUCCESS;
}

void VOUnityBridge::convertToUnityCoordinates(const Eigen::Matrix4d &pose,
                                              VOPose &out) const {
  // Extract rotation matrix and translation
  Eigen::Matrix3d R = pose.block<3, 3>(0, 0);
  Eigen::Vector3d t = pose.block<3, 1>(0, 3);

  // Convert coordinate system: OpenCV (Y-down) to Unity (Y-up)
  // Flip Y axis
  out.px = static_cast<float>(t.x());
  out.py = static_cast<float>(-t.y());
  out.pz = static_cast<float>(t.z());

  // Convert rotation matrix to quaternion
  // First apply coordinate transformation to rotation
  Eigen::Matrix3d R_unity;
  R_unity(0, 0) = R(0, 0);
  R_unity(0, 1) = -R(0, 1);
  R_unity(0, 2) = R(0, 2);
  R_unity(1, 0) = -R(1, 0);
  R_unity(1, 1) = R(1, 1);
  R_unity(1, 2) = -R(1, 2);
  R_unity(2, 0) = R(2, 0);
  R_unity(2, 1) = -R(2, 1);
  R_unity(2, 2) = R(2, 2);

  // Convert to quaternion using Eigen
  Eigen::Quaterniond q(R_unity);
  q.normalize();

  out.qx = static_cast<float>(q.x());
  out.qy = static_cast<float>(q.y());
  out.qz = static_cast<float>(q.z());
  out.qw = static_cast<float>(q.w());

  // Check if pose is valid (not identity or near-zero)
  double det = R.determinant();
  out.valid = (std::abs(det - 1.0) < 0.1) ? 1 : 0;
}

VOErrorCode VOUnityBridge::getDebugImage(uint8_t *out, int width, int height,
                                         bool drawPoints, bool drawFlow) const {
  std::lock_guard<std::mutex> lock(mutex_);

  if (!initialized_ || vo_ == nullptr) {
    return VO_ERROR_NOT_INITIALIZED;
  }

  if (out == nullptr || width != camera_params_.width ||
      height != camera_params_.height) {
    return VO_ERROR_INVALID_PARAM;
  }

  if (current_frame_.empty()) {
    return VO_ERROR_INVALID_IMAGE;
  }

  try {
    // Convert grayscale to RGBA
    cv::Mat debugImg;
    cv::cvtColor(current_frame_, debugImg, cv::COLOR_GRAY2RGBA);

    // Get current features
    TrackBase *tracker = vo_->getTracker();
    if (tracker != nullptr) {
      auto last_obs = tracker->get_last_obs();
      auto last_ids = tracker->get_last_ids();

      if (last_obs.find(0) != last_obs.end() &&
          last_ids.find(0) != last_ids.end()) {
        const auto &keypoints = last_obs[0];
        const auto &ids = last_ids[0];

        for (size_t i = 0; i < keypoints.size(); ++i) {
          cv::Point2f pt(keypoints[i].pt.x, keypoints[i].pt.y);
          int id = static_cast<int>(ids[i]);

          // Draw optical flow (green line) if we have previous position
          if (drawFlow) {
            auto prevIt = prev_feature_positions_.find(id);
            if (prevIt != prev_feature_positions_.end()) {
              cv::Point2f prevPt = prevIt->second;
              // Only draw if movement is reasonable (not a reset)
              float dist = cv::norm(pt - prevPt);
              if (dist < width * 0.2f) {
                cv::line(debugImg, prevPt, pt, cv::Scalar(0, 255, 0, 255), 2);
              }
            }
          }

          // Draw feature point (red filled circle)
          if (drawPoints) {
            cv::circle(debugImg, pt, 4, cv::Scalar(255, 0, 0, 255), -1);
          }

          // Update previous position
          prev_feature_positions_[id] = pt;
        }
      }
    }

    // Copy to output buffer (RGBA format)
    std::memcpy(out, debugImg.data, width * height * 4);

    return VO_SUCCESS;
  } catch (const std::exception &e) {
    return VO_ERROR_TRACKING_FAILED;
  }
}

} // namespace ov_core
