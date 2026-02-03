/**
 * @file VisualOdometry.cpp
 * @brief Implementation of the main visual odometry class
 */

#include "VisualOdometry.h"

#include "calibration/CalibrationParser.h"
#include "pose/PoseEstimator.h"
#include "triangulation/Triangulator.h"

#include "cam/CamRadtan.h"
#include "feat/Feature.h"
#include "feat/FeatureDatabase.h"
#include "track/TrackKLT.h"
#include "utils/print.h"
#include "utils/sensor_data.h"

#include <algorithm>
#include <opencv2/imgproc.hpp>

namespace ov_core {

VisualOdometry::VisualOdometry(const Config &config) : config_(config) {
  // Build camera intrinsic matrix
  K_ = config_.calibration.getK();

  // Initialize keyframe selector
  keyframe_selector_ = std::make_unique<KeyframeSelector>(config_.keyframe_config);

  // Initialize map manager
  map_manager_ = std::make_unique<MapManager>();

  // Initialize Pangolin viewer
  viewer_ = std::make_unique<PangolinViewer>(config_.viewer_config);
  viewer_->start();

  // Initialize current pose to identity
  current_pose_ = Eigen::Matrix4d::Identity();

  // Create camera calibration for the tracker
  // The tracker uses CamBase for undistortion
  std::unordered_map<size_t, std::shared_ptr<CamBase>> cameras;
  Eigen::Matrix<DataType, 8, 1> cam_calib;
  cam_calib << static_cast<DataType>(config_.calibration.fx), 
               static_cast<DataType>(config_.calibration.fy),
               static_cast<DataType>(config_.calibration.cx), 
               static_cast<DataType>(config_.calibration.cy),
               static_cast<DataType>(config_.calibration.distortion.size() > 0 ? config_.calibration.distortion[0] : 0.0),
               static_cast<DataType>(config_.calibration.distortion.size() > 1 ? config_.calibration.distortion[1] : 0.0),
               static_cast<DataType>(config_.calibration.distortion.size() > 2 ? config_.calibration.distortion[2] : 0.0),
               static_cast<DataType>(config_.calibration.distortion.size() > 3 ? config_.calibration.distortion[3] : 0.0);

  auto camera = std::make_shared<CamRadtan>(config_.calibration.image_width, config_.calibration.image_height);
  camera->set_value(cam_calib);
  cameras[0] = camera;

  // Create KLT tracker
  // Parameters: cameras, num_features, num_aruco, stereo, histogram_method, fast_threshold, grid_x, grid_y, min_px_dist, ransac_thresh
  tracker_ = std::make_unique<TrackKLT>(
      cameras,
      config_.num_features,
      1024,  // num_aruco (not used for KLT)
      true,  // monocular (not stereo)
      TrackBase::HistogramMethod::HISTOGRAM,
      config_.fast_threshold,
      5,   // grid_x
      4,   // grid_y
      20,  // min_px_dist
      1.0  // ransac_thresh
  );

  PRINT_INFO("[VisualOdometry] Initialized with %d features, fx=%.1f, fy=%.1f, cx=%.1f, cy=%.1f\n",
             config_.num_features, config_.calibration.fx, config_.calibration.fy, 
             config_.calibration.cx, config_.calibration.cy);
}

VisualOdometry::~VisualOdometry() {
  if (viewer_) {
    viewer_->stop();
  }
}

VisualOdometry::Config VisualOdometry::loadConfigFromFile(const std::string &calibration_file,
                                                          const std::string &camera_name) {
  Config config;
  config.calibration = CalibrationParser::loadFromFile(calibration_file, camera_name);
  return config;
}

VisualOdometry::Config VisualOdometry::getDefaultConfig(int image_width, int image_height) {
  Config config;
  config.calibration = CalibrationParser::getDefaultCalibration(image_width, image_height);
  return config;
}


void VisualOdometry::processFrame(const cv::Mat &image, double timestamp) {
  // Convert to grayscale if needed
  cv::Mat gray;
  if (image.channels() == 3) {
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
  } else if (image.channels() == 4) {
    cv::cvtColor(image, gray, cv::COLOR_BGRA2GRAY);
  } else {
    gray = image;
  }

  // Feed image to tracker
  CameraData message;
  message.timestamp = timestamp;
  message.sensor_ids.push_back(0);
  message.images.push_back(gray);
  message.masks.push_back(cv::Mat::zeros(gray.size(), CV_8UC1));
  tracker_->feed_new_camera(message);

  frame_count_++;

  if (!initialized_) {
    initialize(gray);
  } else {
    track(gray);
  }

  // Update visualization
  if (viewer_) {
    viewer_->updateCurrentPose(current_pose_);
    viewer_->updateTrajectory(map_manager_->getTrajectory());
    
    // Convert MapPoints to Vector3d for viewer
    auto map_points = map_manager_->getPointCloud();
    std::vector<Eigen::Vector3d> points;
    points.reserve(map_points.size());
    for (const auto &mp : map_points) {
      points.push_back(mp.position);
    }
    viewer_->updatePointCloud(points);
    viewer_->updateImage(image);
  }
}

Eigen::Matrix4d VisualOdometry::getCurrentPose() const {
  return current_pose_;
}

std::vector<Eigen::Vector3d> VisualOdometry::getPointCloud() const {
  auto map_points = map_manager_->getPointCloud();
  std::vector<Eigen::Vector3d> points;
  points.reserve(map_points.size());
  for (const auto &mp : map_points) {
    points.push_back(mp.position);
  }
  return points;
}

bool VisualOdometry::shouldQuit() const {
  return viewer_ && viewer_->shouldQuit();
}


void VisualOdometry::initialize(const cv::Mat &image) {
  // Get current tracked features
  auto database = tracker_->get_feature_database();
  auto last_obs = tracker_->get_last_obs();
  auto last_ids = tracker_->get_last_ids();

  // Check if we have camera 0 observations
  if (last_obs.find(0) == last_obs.end() || last_ids.find(0) == last_ids.end()) {
    return;
  }

  int num_tracked = static_cast<int>(last_obs[0].size());
  
  // Need minimum features for initialization
  if (num_tracked < 50) {
    PRINT_DEBUG("[VisualOdometry] Waiting for more features: %d/50\n", num_tracked);
    return;
  }

  // First frame - just store as first keyframe
  if (frame_count_ == 1) {
    keyframe_selector_->updateLastKeyframePose(current_pose_);
    first_keyframe_timestamp_ = tracker_->get_feature_database()->get_oldest_timestamp();
    PRINT_INFO("[VisualOdometry] First keyframe set with %d features\n", num_tracked);
    return;
  }

  // During initialization, we can't use pose-based keyframe selection
  // because we haven't estimated the pose yet. Instead, use frame count
  // and check if we have enough feature motion (parallax)
  int frames_since_keyframe = frame_count_ - 1;
  
  // Wait at least 10 frames before trying to initialize
  if (frames_since_keyframe < 10) {
    return;
  }

  // Collect feature correspondences between first keyframe and current frame
  std::vector<Eigen::Vector2d> pts1_pixel, pts2_pixel;
  std::vector<size_t> matched_ids;

  for (size_t i = 0; i < last_ids[0].size(); ++i) {
    size_t feat_id = last_ids[0][i];
    Feature feat;
    if (!database->get_feature_clone(feat_id, feat)) {
      continue;
    }

    // Check if feature has observations in camera 0
    if (feat.timestamps.find(0) == feat.timestamps.end() || feat.timestamps[0].size() < 2) {
      continue;
    }

    // Get first and last observations
    const auto &timestamps = feat.timestamps[0];
    const auto &uvs = feat.uvs[0];
    const auto &uvs_norm = feat.uvs_norm[0];

    if (timestamps.size() < 2 || uvs.size() < 2) {
      continue;
    }

    // First observation (pixel coordinates)
    Eigen::Vector2d pt1(uvs[0](0), uvs[0](1));
    // Last observation (pixel coordinates)
    Eigen::Vector2d pt2(uvs.back()(0), uvs.back()(1));

    pts1_pixel.push_back(pt1);
    pts2_pixel.push_back(pt2);
    matched_ids.push_back(feat_id);
  }

  PRINT_INFO("[VisualOdometry] Found %zu feature correspondences for initialization\n", pts1_pixel.size());

  if (pts1_pixel.size() < 20) {
    PRINT_INFO("[VisualOdometry] Not enough correspondences for initialization (need 20)\n");
    return;
  }

  // Estimate relative pose using Essential matrix
  auto result = PoseEstimator::solveEssential(pts1_pixel, pts2_pixel, K_);

  if (!result.success || result.num_inliers < 10) {
    PRINT_INFO("[VisualOdometry] Essential matrix estimation failed (inliers: %d, need 10)\n", result.num_inliers);
    return;
  }

  PRINT_INFO("[VisualOdometry] Essential matrix estimated with %d inliers\n", result.num_inliers);

  // Update current pose (relative to first keyframe which is at identity)
  current_pose_ = result.pose;

  // Triangulate initial points
  // Convert pixel coordinates to normalized coordinates for triangulation
  std::vector<Eigen::Vector2d> pts1_norm, pts2_norm;
  for (size_t i = 0; i < pts1_pixel.size(); ++i) {
    // Normalize: (u - cx) / fx, (v - cy) / fy
    Eigen::Vector2d p1_norm((pts1_pixel[i].x() - K_(0, 2)) / K_(0, 0),
                            (pts1_pixel[i].y() - K_(1, 2)) / K_(1, 1));
    Eigen::Vector2d p2_norm((pts2_pixel[i].x() - K_(0, 2)) / K_(0, 0),
                            (pts2_pixel[i].y() - K_(1, 2)) / K_(1, 1));
    pts1_norm.push_back(p1_norm);
    pts2_norm.push_back(p2_norm);
  }

  // First keyframe pose is identity
  Eigen::Matrix4d T1 = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d T2 = current_pose_;

  // Triangulate points
  auto triangulated_points = Triangulator::triangulateBatch(pts1_norm, pts2_norm, T1, T2, 2.0, 0.1, 100.0);

  PRINT_INFO("[VisualOdometry] Triangulated %zu initial 3D points\n", triangulated_points.size());

  if (triangulated_points.size() < 10) {
    PRINT_WARNING("[VisualOdometry] Too few triangulated points, retrying...\n");
    return;
  }

  // Add points to map
  map_manager_->addPoints(triangulated_points);

  // Add camera poses to trajectory
  map_manager_->addCameraPose(T1);
  map_manager_->addCameraPose(current_pose_);

  // Update keyframe selector
  keyframe_selector_->updateLastKeyframePose(current_pose_);
  last_keyframe_pose_ = current_pose_;

  // Store triangulated point IDs for tracking
  // We'll associate 3D points with feature IDs for PnP tracking
  triangulated_feature_ids_.clear();
  triangulated_points_3d_.clear();
  
  // Store the successfully triangulated points with their feature IDs
  size_t tri_idx = 0;
  for (size_t i = 0; i < pts1_norm.size() && tri_idx < triangulated_points.size(); ++i) {
    // Check if this point was successfully triangulated (simple heuristic)
    auto pt_opt = Triangulator::triangulate(pts1_norm[i], pts2_norm[i], T1, T2);
    if (pt_opt.has_value()) {
      // Check depth constraints
      Eigen::Vector4d pt_hom;
      pt_hom << pt_opt.value(), 1.0;
      double depth1 = (T1 * pt_hom)(2);
      double depth2 = (T2 * pt_hom)(2);
      if (depth1 > 0.1 && depth1 < 100.0 && depth2 > 0.1 && depth2 < 100.0) {
        triangulated_feature_ids_.push_back(matched_ids[i]);
        triangulated_points_3d_.push_back(pt_opt.value());
      }
    }
  }

  initialized_ = true;
  frames_since_last_keyframe_ = 0;

  PRINT_INFO("[VisualOdometry] Initialization complete! Map has %zu points\n", map_manager_->getNumPoints());
}


void VisualOdometry::track(const cv::Mat &image) {
  frames_since_last_keyframe_++;

  // Get current tracked features
  auto database = tracker_->get_feature_database();
  auto last_obs = tracker_->get_last_obs();
  auto last_ids = tracker_->get_last_ids();

  if (last_obs.find(0) == last_obs.end() || last_ids.find(0) == last_ids.end()) {
    PRINT_WARNING("[VisualOdometry] No features tracked\n");
    return;
  }

  int num_tracked = static_cast<int>(last_obs[0].size());

  // Build 2D-3D correspondences for PnP
  std::vector<Eigen::Vector3d> pts3d;
  std::vector<Eigen::Vector2d> pts2d;
  std::vector<size_t> matched_3d_indices;

  for (size_t i = 0; i < last_ids[0].size(); ++i) {
    size_t feat_id = last_ids[0][i];

    // Check if this feature has a 3D point
    auto it = std::find(triangulated_feature_ids_.begin(), triangulated_feature_ids_.end(), feat_id);
    if (it != triangulated_feature_ids_.end()) {
      size_t idx = std::distance(triangulated_feature_ids_.begin(), it);
      pts3d.push_back(triangulated_points_3d_[idx]);
      
      // Get current 2D observation (pixel coordinates)
      const cv::KeyPoint &kp = last_obs[0][i];
      pts2d.push_back(Eigen::Vector2d(kp.pt.x, kp.pt.y));
      matched_3d_indices.push_back(idx);
    }
  }

  PRINT_DEBUG("[VisualOdometry] Frame %d: %d tracked, %zu 2D-3D matches\n", 
              frame_count_, num_tracked, pts3d.size());

  // Try PnP if we have enough 2D-3D correspondences
  bool pose_estimated = false;
  if (pts3d.size() >= 6) {
    auto result = PoseEstimator::solvePnP(pts3d, pts2d, K_, 100);
    
    if (result.success && result.num_inliers >= 10) {
      current_pose_ = result.pose;
      pose_estimated = true;
      PRINT_DEBUG("[VisualOdometry] PnP succeeded with %d inliers\n", result.num_inliers);
    }
  }

  // Fallback to Essential matrix if PnP fails
  if (!pose_estimated) {
    // Collect correspondences from last keyframe to current frame
    std::vector<Eigen::Vector2d> pts1_pixel, pts2_pixel;
    
    for (size_t i = 0; i < last_ids[0].size(); ++i) {
      size_t feat_id = last_ids[0][i];
      Feature feat;
      if (!database->get_feature_clone(feat_id, feat)) {
        continue;
      }

      if (feat.timestamps.find(0) == feat.timestamps.end() || feat.timestamps[0].size() < 2) {
        continue;
      }

      const auto &uvs = feat.uvs[0];
      if (uvs.size() < 2) {
        continue;
      }

      // Use first and last observations
      pts1_pixel.push_back(Eigen::Vector2d(uvs[0](0), uvs[0](1)));
      pts2_pixel.push_back(Eigen::Vector2d(uvs.back()(0), uvs.back()(1)));
    }

    if (pts1_pixel.size() >= 10) {
      auto result = PoseEstimator::solveEssential(pts1_pixel, pts2_pixel, K_);
      if (result.success && result.num_inliers >= 10) {
        // Essential matrix gives relative pose, need to compose with last keyframe pose
        current_pose_ = result.pose * last_keyframe_pose_;
        pose_estimated = true;
        PRINT_DEBUG("[VisualOdometry] Essential matrix fallback succeeded with %d inliers\n", result.num_inliers);
      }
    }
  }

  if (!pose_estimated) {
    PRINT_WARNING("[VisualOdometry] Pose estimation failed, keeping previous pose\n");
    return;
  }

  // Check if we should create a new keyframe
  bool should_keyframe = keyframe_selector_->shouldCreateKeyframe(
      current_pose_, num_tracked, frames_since_last_keyframe_);

  if (should_keyframe) {
    triangulateNewPoints();
    
    // Update keyframe state
    keyframe_selector_->updateLastKeyframePose(current_pose_);
    last_keyframe_pose_ = current_pose_;
    frames_since_last_keyframe_ = 0;

    // Add pose to trajectory
    map_manager_->addCameraPose(current_pose_);

    PRINT_INFO("[VisualOdometry] New keyframe created. Map has %zu points\n", map_manager_->getNumPoints());
  }
}

void VisualOdometry::triangulateNewPoints() {
  auto database = tracker_->get_feature_database();
  auto last_obs = tracker_->get_last_obs();
  auto last_ids = tracker_->get_last_ids();

  if (last_obs.find(0) == last_obs.end() || last_ids.find(0) == last_ids.end()) {
    return;
  }

  // Collect features that don't have 3D points yet
  std::vector<Eigen::Vector2d> pts1_norm, pts2_norm;
  std::vector<size_t> new_feature_ids;

  for (size_t i = 0; i < last_ids[0].size(); ++i) {
    size_t feat_id = last_ids[0][i];

    // Skip if already triangulated
    if (std::find(triangulated_feature_ids_.begin(), triangulated_feature_ids_.end(), feat_id) 
        != triangulated_feature_ids_.end()) {
      continue;
    }

    Feature feat;
    if (!database->get_feature_clone(feat_id, feat)) {
      continue;
    }

    if (feat.timestamps.find(0) == feat.timestamps.end() || feat.timestamps[0].size() < 2) {
      continue;
    }

    const auto &uvs = feat.uvs[0];
    if (uvs.size() < 2) {
      continue;
    }

    // Get first and last observations and normalize
    Eigen::Vector2d p1_pixel(uvs[0](0), uvs[0](1));
    Eigen::Vector2d p2_pixel(uvs.back()(0), uvs.back()(1));

    Eigen::Vector2d p1_norm((p1_pixel.x() - K_(0, 2)) / K_(0, 0),
                            (p1_pixel.y() - K_(1, 2)) / K_(1, 1));
    Eigen::Vector2d p2_norm((p2_pixel.x() - K_(0, 2)) / K_(0, 0),
                            (p2_pixel.y() - K_(1, 2)) / K_(1, 1));

    pts1_norm.push_back(p1_norm);
    pts2_norm.push_back(p2_norm);
    new_feature_ids.push_back(feat_id);
  }

  if (pts1_norm.empty()) {
    return;
  }

  // Triangulate using last keyframe pose and current pose
  Eigen::Matrix4d T1 = last_keyframe_pose_;
  Eigen::Matrix4d T2 = current_pose_;

  std::vector<Eigen::Vector3d> new_points;
  std::vector<size_t> valid_feature_ids;

  for (size_t i = 0; i < pts1_norm.size(); ++i) {
    auto pt_opt = Triangulator::triangulate(pts1_norm[i], pts2_norm[i], T1, T2);
    
    if (!pt_opt.has_value()) {
      continue;
    }

    Eigen::Vector3d pt3d = pt_opt.value();

    // Check depth constraints
    Eigen::Vector4d pt_hom;
    pt_hom << pt3d, 1.0;
    double depth1 = (T1 * pt_hom)(2);
    double depth2 = (T2 * pt_hom)(2);

    if (depth1 < 0.1 || depth1 > 100.0 || depth2 < 0.1 || depth2 > 100.0) {
      continue;
    }

    // Check reprojection error
    double reproj1 = Triangulator::computeReprojectionError(pt3d, pts1_norm[i], T1, Eigen::Matrix3d::Identity());
    double reproj2 = Triangulator::computeReprojectionError(pt3d, pts2_norm[i], T2, Eigen::Matrix3d::Identity());

    if (reproj1 > 2.0 || reproj2 > 2.0) {
      continue;
    }

    new_points.push_back(pt3d);
    valid_feature_ids.push_back(new_feature_ids[i]);
  }

  // Add new points to map and tracking lists
  if (!new_points.empty()) {
    map_manager_->addPoints(new_points);
    
    for (size_t i = 0; i < valid_feature_ids.size(); ++i) {
      triangulated_feature_ids_.push_back(valid_feature_ids[i]);
      triangulated_points_3d_.push_back(new_points[i]);
    }

    PRINT_DEBUG("[VisualOdometry] Triangulated %zu new points\n", new_points.size());
  }
}

} // namespace ov_core
