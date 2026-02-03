/**
 * @file KeyframeSelector.h
 * @brief Keyframe selection logic for visual odometry
 */

#ifndef OV_CORE_KEYFRAME_SELECTOR_H
#define OV_CORE_KEYFRAME_SELECTOR_H

#include <Eigen/Dense>

namespace ov_core {

/**
 * @brief KeyframeSelector class for determining when to create new keyframes
 *
 * Keyframes are selected based on motion thresholds (translation and rotation)
 * and feature count thresholds to ensure sufficient baseline for triangulation.
 */
class KeyframeSelector {
public:
  /**
   * @brief Configuration for keyframe selection
   */
  struct Config {
    double min_translation = 0.1;   ///< Minimum translation distance (meters)
    double min_rotation = 5.0;      ///< Minimum rotation angle (degrees)
    int min_tracked_features = 50;  ///< Minimum number of tracked features
    int min_frames_between = 5;     ///< Minimum frames between keyframes
  };

  /**
   * @brief Constructor
   * @param config Keyframe selection configuration
   */
  explicit KeyframeSelector(const Config &config);

  /**
   * @brief Determine if a new keyframe should be created
   * @param current_pose Current camera pose (world to camera)
   * @param num_tracked_features Number of currently tracked features
   * @param frames_since_last_keyframe Number of frames since last keyframe
   * @return True if a new keyframe should be created
   */
  bool shouldCreateKeyframe(const Eigen::Matrix4d &current_pose, int num_tracked_features,
                            int frames_since_last_keyframe);

  /**
   * @brief Update the last keyframe pose
   * @param pose New keyframe pose
   */
  void updateLastKeyframePose(const Eigen::Matrix4d &pose);

  /**
   * @brief Get the last keyframe pose
   * @return Last keyframe pose
   */
  const Eigen::Matrix4d &getLastKeyframePose() const { return last_keyframe_pose_; }

private:
  Config config_;
  Eigen::Matrix4d last_keyframe_pose_;
};

} // namespace ov_core

#endif // OV_CORE_KEYFRAME_SELECTOR_H
