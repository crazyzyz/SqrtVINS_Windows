/**
 * @file Triangulator.cpp
 * @brief Implementation of linear triangulation for 3D point reconstruction
 */

#include "Triangulator.h"

namespace ov_core {

std::optional<Eigen::Vector3d> Triangulator::triangulate(const Eigen::Vector2d &pt1, const Eigen::Vector2d &pt2,
                                                         const Eigen::Matrix4d &T1, const Eigen::Matrix4d &T2) {
  // Extract projection matrices P = K * [R | t] but since we use normalized coordinates, K = I
  // P = [R | t] (3x4 matrix)
  Eigen::Matrix<double, 3, 4> P1 = T1.block<3, 4>(0, 0);
  Eigen::Matrix<double, 3, 4> P2 = T2.block<3, 4>(0, 0);

  // Build the DLT matrix A for linear triangulation
  // For each observation (u, v) and projection matrix P:
  // u * P.row(2) - P.row(0) = 0
  // v * P.row(2) - P.row(1) = 0
  Eigen::Matrix4d A;
  A.row(0) = pt1.x() * P1.row(2) - P1.row(0);
  A.row(1) = pt1.y() * P1.row(2) - P1.row(1);
  A.row(2) = pt2.x() * P2.row(2) - P2.row(0);
  A.row(3) = pt2.y() * P2.row(2) - P2.row(1);

  // Solve using SVD - the solution is the right singular vector corresponding to smallest singular value
  Eigen::JacobiSVD<Eigen::Matrix4d> svd(A, Eigen::ComputeFullV);
  Eigen::Vector4d X_homogeneous = svd.matrixV().col(3);

  // Check for degenerate case (w ≈ 0)
  if (std::abs(X_homogeneous(3)) < 1e-10) {
    return std::nullopt;
  }

  // Convert from homogeneous to 3D coordinates
  Eigen::Vector3d X_world = X_homogeneous.head<3>() / X_homogeneous(3);

  // Check depth in both cameras
  Eigen::Vector4d X_hom;
  X_hom << X_world, 1.0;

  // Transform to camera 1 frame and check depth
  Eigen::Vector4d X_cam1 = T1 * X_hom;
  if (X_cam1(2) <= 0) {
    return std::nullopt; // Point behind camera 1
  }

  // Transform to camera 2 frame and check depth
  Eigen::Vector4d X_cam2 = T2 * X_hom;
  if (X_cam2(2) <= 0) {
    return std::nullopt; // Point behind camera 2
  }

  return X_world;
}


std::vector<Eigen::Vector3d> Triangulator::triangulateBatch(const std::vector<Eigen::Vector2d> &pts1,
                                                            const std::vector<Eigen::Vector2d> &pts2,
                                                            const Eigen::Matrix4d &T1, const Eigen::Matrix4d &T2,
                                                            double max_reproj_error, double min_depth, double max_depth) {
  std::vector<Eigen::Vector3d> valid_points;

  // Ensure input vectors have same size
  if (pts1.size() != pts2.size()) {
    return valid_points;
  }

  // Extract camera intrinsic matrix (identity for normalized coordinates)
  // For reprojection error computation, we need to convert back to pixel coordinates
  // Since we're working with normalized coordinates, we use identity K
  Eigen::Matrix3d K = Eigen::Matrix3d::Identity();

  for (size_t i = 0; i < pts1.size(); ++i) {
    // Triangulate the point
    auto result = triangulate(pts1[i], pts2[i], T1, T2);

    if (!result.has_value()) {
      continue; // Triangulation failed
    }

    Eigen::Vector3d pt3d = result.value();

    // Check depth constraints in both cameras
    Eigen::Vector4d pt3d_hom;
    pt3d_hom << pt3d, 1.0;

    // Depth in camera 1
    Eigen::Vector4d pt_cam1 = T1 * pt3d_hom;
    double depth1 = pt_cam1(2);
    if (depth1 < min_depth || depth1 > max_depth) {
      continue;
    }

    // Depth in camera 2
    Eigen::Vector4d pt_cam2 = T2 * pt3d_hom;
    double depth2 = pt_cam2(2);
    if (depth2 < min_depth || depth2 > max_depth) {
      continue;
    }

    // Check reprojection error in both frames
    // For normalized coordinates, reprojection error is computed in normalized space
    // then scaled by focal length (assumed ~1 for normalized coords, so error is in "normalized pixels")
    double reproj_error1 = computeReprojectionError(pt3d, pts1[i], T1, K);
    double reproj_error2 = computeReprojectionError(pt3d, pts2[i], T2, K);

    // Use max of both reprojection errors
    double max_error = std::max(reproj_error1, reproj_error2);
    if (max_error > max_reproj_error) {
      continue;
    }

    valid_points.push_back(pt3d);
  }

  return valid_points;
}

double Triangulator::computeReprojectionError(const Eigen::Vector3d &pt3d, const Eigen::Vector2d &pt2d,
                                              const Eigen::Matrix4d &T, const Eigen::Matrix3d &K) {
  // Transform 3D point to camera frame
  Eigen::Vector4d pt3d_hom;
  pt3d_hom << pt3d, 1.0;
  Eigen::Vector4d pt_cam = T * pt3d_hom;

  // Check for points behind camera
  if (pt_cam(2) <= 0) {
    return std::numeric_limits<double>::max();
  }

  // Project to normalized image coordinates
  Eigen::Vector2d pt_proj_norm(pt_cam(0) / pt_cam(2), pt_cam(1) / pt_cam(2));

  // Apply camera intrinsics to get pixel coordinates
  Eigen::Vector3d pt_proj_hom;
  pt_proj_hom << pt_proj_norm, 1.0;
  Eigen::Vector3d pt_pixel = K * pt_proj_hom;

  // If K is identity (normalized coordinates), pt_pixel = pt_proj_hom
  Eigen::Vector2d pt_proj(pt_pixel(0), pt_pixel(1));

  // Compute Euclidean distance
  return (pt_proj - pt2d).norm();
}

} // namespace ov_core
