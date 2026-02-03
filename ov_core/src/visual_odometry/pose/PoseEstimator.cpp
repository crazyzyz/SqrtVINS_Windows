/**
 * @file PoseEstimator.cpp
 * @brief Implementation of camera pose estimation using PnP and Essential matrix methods
 */

#include "PoseEstimator.h"

#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>

namespace ov_core {

PoseEstimator::Result PoseEstimator::solvePnP(const std::vector<Eigen::Vector3d> &pts3d,
                                              const std::vector<Eigen::Vector2d> &pts2d, const Eigen::Matrix3d &K,
                                              int ransac_iterations) {
  Result result;
  result.pose = Eigen::Matrix4d::Identity();
  result.num_inliers = 0;
  result.success = false;

  // Need at least 4 points for PnP
  if (pts3d.size() < 4 || pts3d.size() != pts2d.size()) {
    return result;
  }

  // Convert Eigen vectors to OpenCV format
  std::vector<cv::Point3f> object_points;
  std::vector<cv::Point2f> image_points;
  object_points.reserve(pts3d.size());
  image_points.reserve(pts2d.size());

  for (size_t i = 0; i < pts3d.size(); ++i) {
    object_points.emplace_back(static_cast<float>(pts3d[i].x()), static_cast<float>(pts3d[i].y()),
                               static_cast<float>(pts3d[i].z()));
    image_points.emplace_back(static_cast<float>(pts2d[i].x()), static_cast<float>(pts2d[i].y()));
  }

  // Convert intrinsic matrix to OpenCV format
  cv::Mat camera_matrix;
  cv::eigen2cv(K, camera_matrix);
  camera_matrix.convertTo(camera_matrix, CV_64F);

  // No distortion coefficients (assume undistorted points)
  cv::Mat dist_coeffs = cv::Mat::zeros(4, 1, CV_64F);

  // Output rotation and translation vectors
  cv::Mat rvec, tvec;
  cv::Mat inliers;

  // Use solvePnPRansac for robust estimation
  bool pnp_success = cv::solvePnPRansac(object_points, image_points, camera_matrix, dist_coeffs, rvec, tvec, false,
                                        ransac_iterations,
                                        8.0f,  // reprojection error threshold
                                        0.99,  // confidence
                                        inliers, cv::SOLVEPNP_ITERATIVE);

  if (!pnp_success || inliers.empty()) {
    return result;
  }

  result.num_inliers = inliers.rows;

  // Convert rotation vector to rotation matrix
  cv::Mat R_cv;
  cv::Rodrigues(rvec, R_cv);

  // Convert to Eigen
  Eigen::Matrix3d R;
  Eigen::Vector3d t;
  cv::cv2eigen(R_cv, R);
  cv::cv2eigen(tvec, t);

  // Build 4x4 transformation matrix (world to camera)
  result.pose.block<3, 3>(0, 0) = R;
  result.pose.block<3, 1>(0, 3) = t;
  result.success = true;

  return result;
}


PoseEstimator::Result PoseEstimator::solveEssential(const std::vector<Eigen::Vector2d> &pts1,
                                                    const std::vector<Eigen::Vector2d> &pts2, const Eigen::Matrix3d &K) {
  Result result;
  result.pose = Eigen::Matrix4d::Identity();
  result.num_inliers = 0;
  result.success = false;

  // Need at least 5 points for Essential matrix estimation
  if (pts1.size() < 5 || pts1.size() != pts2.size()) {
    return result;
  }

  // Convert Eigen vectors to OpenCV format
  std::vector<cv::Point2f> points1, points2;
  points1.reserve(pts1.size());
  points2.reserve(pts2.size());

  for (size_t i = 0; i < pts1.size(); ++i) {
    points1.emplace_back(static_cast<float>(pts1[i].x()), static_cast<float>(pts1[i].y()));
    points2.emplace_back(static_cast<float>(pts2[i].x()), static_cast<float>(pts2[i].y()));
  }

  // Convert intrinsic matrix to OpenCV format
  cv::Mat camera_matrix;
  cv::eigen2cv(K, camera_matrix);
  camera_matrix.convertTo(camera_matrix, CV_64F);

  // Find Essential matrix using RANSAC
  cv::Mat mask;
  cv::Mat E = cv::findEssentialMat(points1, points2, camera_matrix, cv::RANSAC, 0.999, 1.0, mask);

  if (E.empty()) {
    return result;
  }

  // Count inliers
  result.num_inliers = cv::countNonZero(mask);

  // Recover pose from Essential matrix
  cv::Mat R_cv, t_cv;
  int num_good = cv::recoverPose(E, points1, points2, camera_matrix, R_cv, t_cv, mask);

  if (num_good < 10) {
    return result;
  }

  // Convert to Eigen
  Eigen::Matrix3d R;
  Eigen::Vector3d t;
  cv::cv2eigen(R_cv, R);
  cv::cv2eigen(t_cv, t);

  // Build 4x4 transformation matrix (relative pose from frame 1 to frame 2)
  result.pose.block<3, 3>(0, 0) = R;
  result.pose.block<3, 1>(0, 3) = t;
  result.success = true;

  return result;
}

void PoseEstimator::decomposeEssential(const Eigen::Matrix3d &E, Eigen::Matrix3d &R, Eigen::Vector3d &t) {
  // Perform SVD on Essential matrix
  Eigen::JacobiSVD<Eigen::Matrix3d> svd(E, Eigen::ComputeFullU | Eigen::ComputeFullV);

  Eigen::Matrix3d U = svd.matrixU();
  Eigen::Matrix3d V = svd.matrixV();

  // Ensure proper rotation matrices (det = 1)
  if (U.determinant() < 0) {
    U.col(2) *= -1;
  }
  if (V.determinant() < 0) {
    V.col(2) *= -1;
  }

  // W matrix for rotation decomposition
  Eigen::Matrix3d W;
  W << 0, -1, 0, 1, 0, 0, 0, 0, 1;

  // Two possible rotations: R1 = U * W * V^T, R2 = U * W^T * V^T
  // Two possible translations: t = ±U.col(2)
  // We return one solution; the caller should use cheirality check to select the correct one

  // Return the first solution (R1, +t)
  R = U * W * V.transpose();
  t = U.col(2);

  // Normalize translation to unit length
  double t_norm = t.norm();
  if (t_norm > 1e-10) {
    t /= t_norm;
  }
}

} // namespace ov_core
