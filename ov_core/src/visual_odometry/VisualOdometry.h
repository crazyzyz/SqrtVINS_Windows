/**
 * @file VisualOdometry.h
 * @brief Main visual odometry class integrating all components
 */

#ifndef OV_CORE_VISUAL_ODOMETRY_H
#define OV_CORE_VISUAL_ODOMETRY_H

#include "calibration/CalibrationParser.h"
#include "keyframe/KeyframeSelector.h"
#include "map/MapManager.h"
#include "viewer/PangolinViewer.h"

#include <Eigen/Dense>
#include <memory>
#include <vector>
#include <opencv2/core.hpp>

namespace ov_core {

// Forward declarations
class TrackBase;

/**
 * @brief VisualOdometry class - main entry point for visual sparse mapping
 *
 * Integrates feature tracking, pose estimation, triangulation, and visualization
 * to provide a complete visual odometry pipeline.
 */
class VisualOdometry {
public:
  /**
   * @brief Configuration for the visual odometry system
   */
  struct Config {
    // Camera calibration (can be loaded from file or set manually)
    CameraCalibration calibration;

    // Feature tracking parameters
    int num_features = 500;   ///< Number of features to track
    int fast_threshold = 10;  ///< FAST detector threshold

    // Keyframe parameters
    KeyframeSelector::Config keyframe_config;

    // Visualization parameters
    PangolinViewer::Config viewer_config;

    // Convenience accessors for backward compatibility
    double fx() const { return calibration.fx; }
    double fy() const { return calibration.fy; }
    double cx() const { return calibration.cx; }
    double cy() const { return calibration.cy; }
    const std::vector<double>& distortion() const { return calibration.distortion; }
  };

  /**
   * @brief Constructor
   * @param config Visual odometry configuration
   */
  explicit VisualOdometry(const Config &config);

  /**
   * @brief Destructor
   */
  ~VisualOdometry();

  /**
   * @brief Load configuration from a calibration file
   * @param calibration_file Path to the calibration YAML file
   * @param camera_name Camera name in the file (default: "cam0")
   * @return Config with loaded calibration (or defaults if file not found)
   */
  static Config loadConfigFromFile(const std::string &calibration_file,
                                   const std::string &camera_name = "cam0");

  /**
   * @brief Get default configuration for a webcam
   * @param image_width Image width in pixels
   * @param image_height Image height in pixels
   * @return Config with default calibration values
   */
  static Config getDefaultConfig(int image_width = 640, int image_height = 480);

  /**
   * @brief Process a new frame
   * @param image Input image
   * @param timestamp Frame timestamp
   */
  void processFrame(const cv::Mat &image, double timestamp);

  /**
   * @brief Get the current camera pose
   * @return Current pose (world to camera transform)
   */
  Eigen::Matrix4d getCurrentPose() const;

  /**
   * @brief Get the current point cloud
   * @return Vector of 3D points
   */
  std::vector<Eigen::Vector3d> getPointCloud() const;

  /**
   * @brief Check if the system should quit
   * @return True if quit was requested
   */
  bool shouldQuit() const;

  /**
   * @brief Get the feature tracker for visualization
   * @return Pointer to the tracker
   */
  TrackBase* getTracker() const { return tracker_.get(); }

private:
  /**
   * @brief Initialize the system with the first frames
   * @param image First image
   */
  void initialize(const cv::Mat &image);

  /**
   * @brief Track features in a new frame
   * @param image New image
   */
  void track(const cv::Mat &image);

  /**
   * @brief Triangulate new 3D points from tracked features
   */
  void triangulateNewPoints();

  Config config_;
  Eigen::Matrix3d K_;  ///< Camera intrinsic matrix

  std::unique_ptr<TrackBase> tracker_;
  std::unique_ptr<KeyframeSelector> keyframe_selector_;
  std::unique_ptr<MapManager> map_manager_;
  std::unique_ptr<PangolinViewer> viewer_;

  bool initialized_ = false;
  Eigen::Matrix4d current_pose_;
  Eigen::Matrix4d last_keyframe_pose_;  ///< Pose of the last keyframe
  int frame_count_ = 0;
  int frames_since_last_keyframe_ = 0;  ///< Frame counter since last keyframe
  double first_keyframe_timestamp_ = 0.0;  ///< Timestamp of first keyframe

  // Triangulated 3D points and their associated feature IDs
  std::vector<size_t> triangulated_feature_ids_;  ///< Feature IDs with 3D points
  std::vector<Eigen::Vector3d> triangulated_points_3d_;  ///< Corresponding 3D points
};

} // namespace ov_core

#endif // OV_CORE_VISUAL_ODOMETRY_H
