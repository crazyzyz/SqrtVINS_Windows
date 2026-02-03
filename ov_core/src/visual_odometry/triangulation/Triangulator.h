/**
 * @file Triangulator.h
 * @brief Linear triangulation for 3D point reconstruction from 2D observations
 */

#ifndef OV_CORE_TRIANGULATOR_H
#define OV_CORE_TRIANGULATOR_H

#include <Eigen/Dense>
#include <optional>
#include <vector>

namespace ov_core {

/**
 * @brief Triangulator class for computing 3D points from 2D observations
 *
 * This class provides methods for linear triangulation using SVD-based DLT method,
 * with filtering based on reprojection error and depth constraints.
 */
class Triangulator {
public:
  /**
   * @brief Triangulate a single 3D point from two 2D observations
   * @param pt1 Normalized camera coordinates of point in first frame
   * @param pt2 Normalized camera coordinates of point in second frame
   * @param T1 Camera pose of first frame (world to camera transform)
   * @param T2 Camera pose of second frame (world to camera transform)
   * @return 3D point in world coordinates, or nullopt if triangulation fails
   */
  static std::optional<Eigen::Vector3d> triangulate(const Eigen::Vector2d &pt1, const Eigen::Vector2d &pt2,
                                                    const Eigen::Matrix4d &T1, const Eigen::Matrix4d &T2);

  /**
   * @brief Triangulate multiple 3D points with filtering
   * @param pts1 Normalized camera coordinates in first frame
   * @param pts2 Normalized camera coordinates in second frame
   * @param T1 Camera pose of first frame (world to camera transform)
   * @param T2 Camera pose of second frame (world to camera transform)
   * @param max_reproj_error Maximum allowed reprojection error in pixels
   * @param min_depth Minimum allowed depth
   * @param max_depth Maximum allowed depth
   * @return Vector of valid 3D points in world coordinates
   */
  static std::vector<Eigen::Vector3d> triangulateBatch(const std::vector<Eigen::Vector2d> &pts1,
                                                       const std::vector<Eigen::Vector2d> &pts2,
                                                       const Eigen::Matrix4d &T1, const Eigen::Matrix4d &T2,
                                                       double max_reproj_error = 2.0, double min_depth = 0.1,
                                                       double max_depth = 100.0);

  /**
   * @brief Compute reprojection error for a 3D point
   * @param pt3d 3D point in world coordinates
   * @param pt2d 2D observation in pixel coordinates
   * @param T Camera pose (world to camera transform)
   * @param K Camera intrinsic matrix
   * @return Reprojection error in pixels
   */
  static double computeReprojectionError(const Eigen::Vector3d &pt3d, const Eigen::Vector2d &pt2d,
                                         const Eigen::Matrix4d &T, const Eigen::Matrix3d &K);
};

} // namespace ov_core

#endif // OV_CORE_TRIANGULATOR_H
