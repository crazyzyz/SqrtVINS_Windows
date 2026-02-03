/**
 * @file MapManager.h
 * @brief Thread-safe map management for 3D points and camera trajectory
 */

#ifndef OV_CORE_MAP_MANAGER_H
#define OV_CORE_MAP_MANAGER_H

#include <Eigen/Dense>
#include <mutex>
#include <vector>

namespace ov_core {

/**
 * @brief MapManager class for managing 3D point cloud and camera trajectory
 *
 * Provides thread-safe operations for adding and retrieving map data,
 * allowing concurrent access from processing and visualization threads.
 */
class MapManager {
public:
  /**
   * @brief Structure representing a 3D map point
   */
  struct MapPoint {
    Eigen::Vector3d position;  ///< 3D position in world coordinates
    Eigen::Vector3f color;     ///< RGB color (0-1 range)
    size_t id;                 ///< Unique point ID
    int observations;          ///< Number of times this point was observed
  };

  /**
   * @brief Default constructor
   */
  MapManager() = default;

  /**
   * @brief Add new 3D points to the map
   * @param points Vector of 3D points in world coordinates
   */
  void addPoints(const std::vector<Eigen::Vector3d> &points);

  /**
   * @brief Add a camera pose to the trajectory
   * @param pose Camera pose (world to camera transform)
   */
  void addCameraPose(const Eigen::Matrix4d &pose);

  /**
   * @brief Get all map points (thread-safe)
   * @return Vector of map points
   */
  std::vector<MapPoint> getPointCloud() const;

  /**
   * @brief Get the camera trajectory (thread-safe)
   * @return Vector of camera poses
   */
  std::vector<Eigen::Matrix4d> getTrajectory() const;

  /**
   * @brief Get the current (latest) camera pose
   * @return Current camera pose
   */
  Eigen::Matrix4d getCurrentPose() const;

  /**
   * @brief Clear all map data
   */
  void clear();

  /**
   * @brief Get the number of points in the map
   * @return Number of map points
   */
  size_t getNumPoints() const;

private:
  mutable std::mutex mutex_;
  std::vector<MapPoint> points_;
  std::vector<Eigen::Matrix4d> trajectory_;
  size_t next_point_id_ = 0;
};

} // namespace ov_core

#endif // OV_CORE_MAP_MANAGER_H
