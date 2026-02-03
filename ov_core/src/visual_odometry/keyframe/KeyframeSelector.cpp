/**
 * @file KeyframeSelector.cpp
 * @brief Implementation of keyframe selection logic for visual odometry
 */

#define _USE_MATH_DEFINES
#include <cmath>

#include "KeyframeSelector.h"

namespace ov_core {

KeyframeSelector::KeyframeSelector(const Config &config) : config_(config), last_keyframe_pose_(Eigen::Matrix4d::Identity()) {}

bool KeyframeSelector::shouldCreateKeyframe(const Eigen::Matrix4d &current_pose, int num_tracked_features,
                                            int frames_since_last_keyframe) {
  // Requirement 6.3: Force keyframe when tracked features drop below threshold
  // This takes priority to reinitialize tracking
  if (num_tracked_features < config_.min_tracked_features) {
    return true;
  }

  // Check minimum frames between keyframes
  if (frames_since_last_keyframe < config_.min_frames_between) {
    return false;
  }

  // Requirement 6.1: Check motion thresholds (translation and rotation)
  // Extract rotation and translation from current and last keyframe poses

  // Extract rotation matrices
  Eigen::Matrix3d R_current = current_pose.block<3, 3>(0, 0);
  Eigen::Matrix3d R_last = last_keyframe_pose_.block<3, 3>(0, 0);

  // Extract translation vectors
  Eigen::Vector3d t_current = current_pose.block<3, 1>(0, 3);
  Eigen::Vector3d t_last = last_keyframe_pose_.block<3, 1>(0, 3);

  // Compute relative transformation
  // T_relative = T_current * T_last^(-1)
  // For rotation: R_relative = R_current * R_last^T
  Eigen::Matrix3d R_relative = R_current * R_last.transpose();

  // Compute translation distance
  // The translation in the pose is the camera position in world frame (for world-to-camera transform)
  // We need to compute the actual camera movement
  // For T_wc (world to camera): camera position in world = -R^T * t
  Eigen::Vector3d cam_pos_current = -R_current.transpose() * t_current;
  Eigen::Vector3d cam_pos_last = -R_last.transpose() * t_last;
  double translation_distance = (cam_pos_current - cam_pos_last).norm();

  // Compute rotation angle from relative rotation matrix
  // Using the formula: angle = arccos((trace(R) - 1) / 2)
  double trace = R_relative.trace();
  // Clamp to valid range [-1, 3] to handle numerical errors
  trace = std::max(-1.0, std::min(3.0, trace));
  double rotation_angle_rad = std::acos((trace - 1.0) / 2.0);
  double rotation_angle_deg = rotation_angle_rad * 180.0 / M_PI;

  // Check if motion exceeds thresholds
  bool sufficient_translation = translation_distance > config_.min_translation;
  bool sufficient_rotation = rotation_angle_deg > config_.min_rotation;

  // Create keyframe if either translation OR rotation threshold is exceeded
  return sufficient_translation || sufficient_rotation;
}

void KeyframeSelector::updateLastKeyframePose(const Eigen::Matrix4d &pose) { last_keyframe_pose_ = pose; }

} // namespace ov_core
