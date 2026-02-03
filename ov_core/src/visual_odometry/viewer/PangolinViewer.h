/**
 * @file PangolinViewer.h
 * @brief 3D visualization using Pangolin library
 */

#ifndef OV_CORE_PANGOLIN_VIEWER_H
#define OV_CORE_PANGOLIN_VIEWER_H

#include <Eigen/Dense>
#include <atomic>
#include <mutex>
#include <opencv2/core.hpp>
#include <thread>
#include <vector>

namespace ov_core {

/**
 * @brief PangolinViewer class for 3D visualization
 *
 * Provides real-time 3D visualization of point cloud, camera trajectory,
 * and current camera pose using the Pangolin library.
 */
class PangolinViewer {
public:
  /**
   * @brief Configuration for the viewer
   */
  struct Config {
    int window_width = 1024;                          ///< Window width in pixels
    int window_height = 768;                          ///< Window height in pixels
    std::string window_name = "Visual Sparse Mapping"; ///< Window title
    float point_size = 2.0f;                          ///< Point rendering size
    float camera_size = 0.1f;                         ///< Camera frustum size
  };

  /**
   * @brief Constructor
   * @param config Viewer configuration
   */
  explicit PangolinViewer(const Config &config);

  /**
   * @brief Destructor - ensures clean shutdown
   */
  ~PangolinViewer();

  /**
   * @brief Start the visualization thread
   */
  void start();

  /**
   * @brief Stop the visualization thread
   */
  void stop();

  /**
   * @brief Update the point cloud data (thread-safe)
   * @param points Vector of 3D points
   */
  void updatePointCloud(const std::vector<Eigen::Vector3d> &points);

  /**
   * @brief Update the camera trajectory (thread-safe)
   * @param poses Vector of camera poses
   */
  void updateTrajectory(const std::vector<Eigen::Matrix4d> &poses);

  /**
   * @brief Update the current camera pose (thread-safe)
   * @param pose Current camera pose
   */
  void updateCurrentPose(const Eigen::Matrix4d &pose);

  /**
   * @brief Update the current image for display (thread-safe)
   * @param image Current camera image
   */
  void updateImage(const cv::Mat &image);

  /**
   * @brief Check if the viewer should quit
   * @return True if quit was requested
   */
  bool shouldQuit() const;

private:
  /**
   * @brief Main rendering loop (runs in separate thread)
   */
  void renderLoop();

  /**
   * @brief Draw the point cloud
   */
  void drawPointCloud();

  /**
   * @brief Draw the camera trajectory
   */
  void drawTrajectory();

  /**
   * @brief Draw the current camera frustum
   */
  void drawCurrentCamera();

  /**
   * @brief Draw a camera frustum at a given pose
   * @param pose Camera pose
   * @param size Frustum size
   */
  void drawCameraFrustum(const Eigen::Matrix4d &pose, float size);

  Config config_;
  std::thread render_thread_;
  std::atomic<bool> running_;
  std::atomic<bool> quit_requested_;

  // Thread-safe data storage
  mutable std::mutex data_mutex_;
  std::vector<Eigen::Vector3d> points_;
  std::vector<Eigen::Matrix4d> trajectory_;
  Eigen::Matrix4d current_pose_;
  cv::Mat current_image_;
};

} // namespace ov_core

#endif // OV_CORE_PANGOLIN_VIEWER_H
