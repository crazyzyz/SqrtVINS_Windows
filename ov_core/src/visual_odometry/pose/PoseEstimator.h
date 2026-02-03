/**
 * @file PoseEstimator.h
 * @brief Camera pose estimation using PnP and Essential matrix methods
 */

#ifndef OV_CORE_POSE_ESTIMATOR_H
#define OV_CORE_POSE_ESTIMATOR_H

#include <Eigen/Dense>
#include <vector>

namespace ov_core {

/**
 * @brief PoseEstimator class for camera pose estimation
 *
 * Provides methods for estimating camera pose using PnP (when 3D points are available)
 * and Essential matrix decomposition (for 2D-2D correspondences).
 */
class PoseEstimator {
public:
  /**
   * @brief Result structure for pose estimation
   */
  struct Result {
    Eigen::Matrix4d pose;  ///< World to camera transform
    int num_inliers;       ///< Number of inliers from RANSAC
    bool success;          ///< Whether estimation succeeded
  };

  /**
   * @brief Estimate camera pose using PnP with RANSAC
   * @param pts3d 3D points in world coordinates
   * @param pts2d 2D observations in pixel coordinates
   * @param K Camera intrinsic matrix
   * @param ransac_iterations Number of RANSAC iterations
   * @return Pose estimation result
   */
  static Result solvePnP(const std::vector<Eigen::Vector3d> &pts3d, const std::vector<Eigen::Vector2d> &pts2d,
                         const Eigen::Matrix3d &K, int ransac_iterations = 100);

  /**
   * @brief Estimate relative pose using Essential matrix
   * @param pts1 2D points in first frame (pixel coordinates)
   * @param pts2 2D points in second frame (pixel coordinates)
   * @param K Camera intrinsic matrix
   * @return Pose estimation result (relative pose from frame 1 to frame 2)
   */
  static Result solveEssential(const std::vector<Eigen::Vector2d> &pts1, const std::vector<Eigen::Vector2d> &pts2,
                               const Eigen::Matrix3d &K);

  /**
   * @brief Decompose Essential matrix into R and t
   * @param E Essential matrix
   * @param R Output rotation matrix
   * @param t Output translation vector (unit length)
   */
  static void decomposeEssential(const Eigen::Matrix3d &E, Eigen::Matrix3d &R, Eigen::Vector3d &t);
};

} // namespace ov_core

#endif // OV_CORE_POSE_ESTIMATOR_H
